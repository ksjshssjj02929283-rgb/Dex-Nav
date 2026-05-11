# nav2_smac_planner 2D Shortcut + Smoother 设计说明

## 文档目的

本文档整理 `nav2_smac_planner` 中现有 `smoother` 的原理，并给出仅针对 `SmacPlanner2D` 的 `shortcut + smoother` 优化方案，便于后续实现与评审。

本文档只讨论：

- 现有 `Smoother` 的数学原理与代码行为
- `SmacPlanner2D` 中增加 `shortcut` 的推荐接入方式

本文档不讨论：

- `SmacPlannerHybrid` / `SmacPlannerLattice` 的 shortcut 设计
- 基于 Dubins / Reeds-Shepp / motion primitive 的局部重连

## 当前 2D 规划链路

`SmacPlanner2D` 当前的主流程如下：

1. 在 costmap 上执行二维 A* 搜索
2. 将离散网格路径转换为世界坐标路径
3. 调用 `_smoother->smooth(plan, costmap, time_remaining)` 做后处理
4. 根据配置修正终点朝向

对应代码入口：

- `mobile_navigation/nav2_smac_planner/src/smac_planner_2d.cpp`
- `mobile_navigation/nav2_smac_planner/src/smoother.cpp`

从职责划分上看：

- A* 负责生成可行离散路径
- `Smoother` 负责对路径做几何后处理

因此如果要增加 `shortcut`，最自然的方式是放在 `SmacPlanner2D` 的后处理阶段，而不是改 A* 主循环。

## 现有 Smoother 的原理

### 核心思想

当前 `Smoother` 本质上是一个离散路径上的迭代平滑器。它不是重新规划器，也不是基于障碍代价场的全局优化器，而是一个保守的、局部的几何平滑过程。

它的目标可以理解为同时满足两件事：

- 平滑后的路径不要偏离原始 A* 路径太远
- 路径局部的折线拐角要被抑制，整体更光滑

### 数学形式

设原始路径点为：

\[
x_i = (x_i^{orig}, y_i^{orig}), \quad i=0,\dots,N
\]

平滑后的路径点为：

\[
y_i = (x_i, y_i), \quad i=0,\dots,N
\]

其中首尾点通常保持不动，只优化中间点。可以把它隐含的优化目标写为：

\[
J(\{y_i\}) =
\alpha \sum_{i=1}^{N-1}\|y_i - x_i\|^2 +
\beta \sum_{i=1}^{N-1}\|y_{i+1} - 2y_i + y_{i-1}\|^2
\]

其中：

- 第一项是 data term，约束平滑结果贴近原始路径
- 第二项是 smoothness term，惩罚二阶差分，抑制折角和局部抖动

代码中并没有显式构造这个目标函数再调用通用求解器，而是直接使用迭代更新。

### 迭代更新公式

对每个中间点、每个维度分别更新：

\[
y_i \leftarrow y_i + w_{data}(x_i - y_i) + w_{smooth}(y_{i+1} + y_{i-1} - 2y_i)
\]

这正是 `smoother.cpp` 中的更新逻辑。

这条公式可以直观理解为两股“力”：

- `w_data (x_i - y_i)`：把当前点往原始路径位置拉回去
- `w_smooth (y_{i+1} + y_{i-1} - 2y_i)`：把当前点往相邻点的平均位置拉过去，从而消除尖角

如果某个点处在明显折线上，那么第二项会更大，该点会逐步向邻域连线收缩，折角就会变缓。

### 收敛条件

每轮迭代后，代码会累计点位置变化量：

\[
\Delta = \sum_i \|y_i^{new} - y_i^{old}\|
\]

当：

\[
\Delta < tolerance
\]

时认为收敛并停止。

### 参数含义

当前 `Smoother` 相关参数包括：

- `w_data`
- `w_smooth`
- `tolerance`
- `max_iterations`
- `do_refinement`

含义如下：

- `w_data` 越大，越贴近原始路径
- `w_smooth` 越大，路径越趋向平滑，但偏移也更明显
- `tolerance` 越小，收敛要求越严格
- `max_iterations` 控制最大迭代轮数
- `do_refinement` 表示是否在结果上继续做额外几轮细化

### 代码层面的保护机制

当前平滑器不是“只管平滑，不管后果”，它做了几层保护：

1. 超过最大迭代次数则停止，并回退到上一条有效路径
2. 超过本次规划剩余时间预算则停止，并回退
3. 若平滑后点位落入碰撞区域，则停止，并回退到上一条有效路径

因此它是一个保守的后处理器。

### 对 2D A* 的局限

对于 `SmacPlanner2D`，当前 `Smoother` 可以做到：

- 减少栅格折线的锯齿感
- 让局部转折更自然

但它做不到：

- 主动删除冗余中间点
- 显式检查“两个远一点的路径点能否直接连成直线”
- 把一段本可直连的折线，强制简化成单一直线段

原因是它的更新是局部迭代的，只使用 `i-1, i, i+1` 的邻域信息，不做跨多个点的可见性判断。

换句话说，它擅长“拉直”，但不擅长“删点重连”。

## 为什么需要 Shortcut

在二维 A* 中，常见情况是：

- A* 因为网格离散，路径由许多小折线组成
- 这些折线局部虽然不共线，但从几何上看，一些较远的点之间其实可以直接直线连接

例如：

```text
A -> B -> C -> D
```

其中如果 `A -> D` 整段都无碰撞，那么最优的几何表达往往不是让 `B`、`C` 通过局部平滑慢慢“靠近直线”，而是直接删掉 `B`、`C`，改成：

```text
A -> D
```

这就是 shortcut 的价值。

因此对 `SmacPlanner2D`，更合理的后处理链路应为：

1. 先做 `shortcut`，删除可以被直线替代的冗余拐点
2. 再做现有 `smoother`，把保留下来的折线进一步柔化

## 2D Shortcut 的设计目标

仅针对 `SmacPlanner2D`，shortcut 设计目标如下：

- 只处理二维 A* 路径
- 不修改通用 `Smoother` 行为
- 不影响 `Hybrid` 和 `Lattice`
- 尽量少改现有框架
- 只基于“直线无碰撞”做几何 shortcut

这个目标下，shortcut 应该放在 `SmacPlanner2D::createPlan()` 内，而不是放进通用 `Smoother` 中。

## 推荐接入位置

当前 2D 规划器在 A* 之后直接调用 smoother：

```cpp
_smoother->smooth(plan, costmap, time_remaining);
```

推荐修改为：

```cpp
shortcutPath(plan, costmap, shortcut_time_budget);
_smoother->smooth(plan, costmap, smoothing_time_budget);
```

即：

- `shortcut` 先执行
- `smoother` 后执行

这样职责清晰：

- `shortcut` 负责路径简化
- `smoother` 负责局部平滑

## Shortcut 的数学定义

给定路径点序列：

\[
p_0, p_1, \dots, p_n
\]

如果存在某两个点 `p_i` 与 `p_j`，满足：

1. `j > i + 1`
2. `p_i` 到 `p_j` 的直线段完全无碰撞

则可以将中间点：

\[
p_{i+1}, \dots, p_{j-1}
\]

全部删除，保留：

\[
p_i, p_j
\]

这一步本质上是路径的可见性简化。

## 直线无碰撞判定

对于两个点 `p_i = (x_i, y_i)` 和 `p_j = (x_j, y_j)`，直线段可写为：

\[
p(t) = (1 - t) p_i + t p_j,\quad t \in [0,1]
\]

实现中可按固定步长采样，例如步长 `d_s`：

1. 计算线段长度

\[
L = \|p_j - p_i\|
\]

2. 采样数

\[
K = \lceil L / d_s \rceil
\]

3. 逐点检查：

\[
t_k = \frac{k}{K},\quad k=0,\dots,K
\]

若任一采样点落在碰撞区域，则该 shortcut 无效。

### 2D 碰撞标准

为了和当前 `Smoother` 保持一致，可以采用相同 costmap 判定方式：

- 若 `cost > MAX_NON_OBSTACLE`
- 且 `cost != UNKNOWN`

则认为该点不可通过。

如果后续希望更严格，也可以改为调用 2D collision checker 做 footprint 检查，但第一版没有必要复杂化。

## Shortcut 的推荐算法

建议使用贪心最远可连策略。

伪代码如下：

```text
输入: path = [p0, p1, ..., pn]
输出: simplified_path

new_path = [p0]
i = 0

while i < n:
  best_j = i + 1

  for j from n downto i + 2:
    if line(p_i, p_j) collision-free:
      best_j = j
      break

  new_path.push_back(p_best_j)
  i = best_j
```

该方法的特点：

- 实现简单
- 行为直观
- 对 2D A* 路径通常有明显效果

## Shortcut + Smoother 的组合效果

对 2D A*，两者作用不同：

### Shortcut 的作用

- 删除可以被直线替代的冗余点
- 显著减少路径折线段数量
- 压缩路径表示复杂度

### Smoother 的作用

- 将保留下来的局部折线继续平滑
- 减少残余锯齿
- 让转折更自然

因此推荐的处理顺序必须是：

```text
shortcut -> smoother
```

而不是反过来。

原因是如果先 smoother，再 shortcut，那么：

- smoother 仍然会在大量冗余点上做迭代
- shortcut 的收益会被部分稀释

先删点再平滑，效率和结果都更合理。

## 仅针对 2D 的原因

之所以当前方案只建议用于 `SmacPlanner2D`，是因为二维 A* 路径只考虑平面位置，不显式约束曲率和运动学。

对于 `Hybrid` 和 `Lattice`：

- 路径包含朝向信息
- 可能存在前进/倒车段
- 可能受最小转弯半径约束

这时“只要几何上可直连就 shortcut”并不总是合法，所以不应直接复用 2D 的直线 shortcut。

## 推荐代码改动范围

如果后续实现该方案，推荐只修改：

- `mobile_navigation/nav2_smac_planner/include/nav2_smac_planner/smac_planner_2d.hpp`
- `mobile_navigation/nav2_smac_planner/src/smac_planner_2d.cpp`

不建议修改：

- 通用 `Smoother`
- `SmacPlannerHybrid`
- `SmacPlannerLattice`

## 推荐新增接口

建议在 `SmacPlanner2D` 中新增以下方法：

```cpp
bool shortcutPath(
  nav_msgs::msg::Path & path,
  const nav2_costmap_2d::Costmap2D * costmap,
  const double & max_time);

bool isLineCollisionFree(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const nav2_costmap_2d::Costmap2D * costmap) const;
```

其中：

- `shortcutPath()` 负责整条路径的简化
- `isLineCollisionFree()` 负责两点直连可行性判断

## 推荐参数

建议只给 `SmacPlanner2D` 增加少量参数：

```yaml
GridBased:
  do_shortcut: true
  shortcut_sampling_dist: 0.02
```

可选扩展参数：

```yaml
GridBased:
  shortcut_max_time_ratio: 0.2
  shortcut_min_points: 3
```

含义建议如下：

- `do_shortcut`：是否启用 shortcut
- `shortcut_sampling_dist`：直线碰撞检测采样间距
- `shortcut_max_time_ratio`：shortcut 最多使用剩余规划时间的比例
- `shortcut_min_points`：路径点数低于该值时不做 shortcut

## 推荐执行流程

在 `SmacPlanner2D::createPlan()` 中，推荐的处理顺序如下：

1. A* 生成原始路径
2. 转成世界坐标路径
3. 计算剩余规划时间
4. 若启用 shortcut，则先执行 `shortcutPath()`
5. 重新计算剩余时间
6. 调用 `_smoother->smooth()`
7. 处理最终朝向

伪代码如下：

```cpp
double time_remaining = ...;

if (_do_shortcut) {
  shortcutPath(plan, costmap, time_remaining * _shortcut_max_time_ratio);
}

double time_remaining_after_shortcut = ...;
_smoother->smooth(plan, costmap, time_remaining_after_shortcut);
```

## 实现上的边界条件

实现时应注意：

- 路径点数小于 3 时直接跳过 shortcut
- 始终保留起点与终点
- shortcut 失败时保留原路径，不应报错
- shortcut 超时则提前退出，保留当前已简化结果或原始结果
- 终点朝向仍由现有 `SmacPlanner2D` 逻辑处理，不在 shortcut 中处理

## 测试建议

建议至少覆盖以下测试：

- 无障碍折线路径，shortcut 后点数明显减少
- 直连穿过障碍物时，shortcut 不删点
- shortcut 后 smoother 仍能正常运行
- 路径点数极少时不崩溃
- shortcut 超时时能安全退出

## 结论

现有 `Smoother` 的本质是一个带数据项约束的离散迭代平滑器，适合做局部几何柔化，但不擅长删除冗余折点。

对于 `SmacPlanner2D`，更合理的优化方向是：

```text
2D A* -> shortcut -> smoother
```

其中：

- `shortcut` 负责删掉可以被直线替代的中间点
- `smoother` 负责将保留下来的折线继续平滑

该方案只需要在 `SmacPlanner2D` 中增加一层轻量后处理，不需要改动现有 A* 与通用 `Smoother` 框架，属于低风险、易落地的增强方式。
