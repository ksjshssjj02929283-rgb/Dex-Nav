# loop_navigation_bt 行为树与循环导航逻辑整理

本文档基于当前代码实现整理，重点说明：
- 当前行为树的真实执行逻辑
- `LoopNavigateFourPoints` 中“路网导航”是如何实现的
- 该节点在导航过程中的状态变化
- 它与 `Fallback` 节点之间的实际关系

## 1. 相关入口与配置

当前包里主要有两套循环导航入口：

### 1.1 充电版循环导航
- 入口文件：`src/main.cpp`
- 行为树文件：`config/loop_navigate_bt.xml`

主程序会：
- 注册 `LoopNavigateFourPoints`
- 注册 `CheckBatteryAboveThreshold`
- 注册 `CancelNavigation`
- 注册 `CallCharging`
- 注册 `Undocking`
- 把 `navigation_points.json` 路径放入 blackboard 的 `navigation_points`
- 以 10Hz 执行 `tree.tickRoot()`

### 1.2 无充电版循环导航
- 入口文件：`src/main_loop_nocharging.cpp`
- 行为树文件：`config/loop_nocharging_bt.xml`

相比充电版，它用 `NavigateToFirstPointAndStop` 替代充电流程，在低电量时回到第一个点并结束程序。

### 1.3 独立充电树
- 入口文件：`src/main_loop_charging.cpp`
- 行为树文件：`config/loop_charging_bt.xml`

这是一棵单独的调试/测试树，不是循环导航主流程的核心部分。

## 2. 当前行为树结构

### 2.1 充电版主树

`config/loop_navigate_bt.xml` 当前结构是：

```xml
<ReactiveSequence name="MainLoop">
  <LoopNavigateFourPoints points="{navigation_points}"/>
  <Fallback name="BatteryCheck">
    <CheckBatteryAboveThreshold threshold="0"/>
    <Sequence name="ChargingSequence">
      <CancelNavigation/>
      <CallCharging/>
      <RetryUntilSuccessful num_attempts="-1">
        <CheckBatteryAboveThreshold threshold="80"/>
      </RetryUntilSuccessful>
      <Undocking duration="4.0"/>
    </Sequence>
  </Fallback>
</ReactiveSequence>
```

### 2.2 无充电版主树

`config/loop_nocharging_bt.xml` 当前结构是：

```xml
<ReactiveSequence name="MainLoop">
  <LoopNavigateFourPoints points="{navigation_points}"/>
  <Fallback name="BatteryCheck">
    <CheckBatteryAboveThreshold threshold="96"/>
    <Sequence name="StopSequence">
      <CancelNavigation/>
      <NavigateToFirstPointAndStop points="{navigation_points}"/>
    </Sequence>
  </Fallback>
</ReactiveSequence>
```

## 3. 行为树整体执行逻辑

根节点是 `ReactiveSequence`，它的关键特性是：
- 每次 tick 都从第一个子节点开始
- 遇到 `RUNNING` 就停止本次向后传播
- 遇到 `FAILURE` 就直接返回 `FAILURE`
- 只有前面的子节点返回 `SUCCESS`，后面的子节点才会被 tick

因此当前主树的真实执行顺序是：

1. 先 tick `LoopNavigateFourPoints`
2. 只有它返回 `SUCCESS` 时，后面的 `Fallback(BatteryCheck)` 才会执行
3. 如果 `LoopNavigateFourPoints` 返回 `RUNNING`，那么本次 tick 到此结束，`Fallback` 根本不会运行
4. 如果 `LoopNavigateFourPoints` 返回 `FAILURE`，根节点直接失败，`Fallback` 同样不会运行

这意味着：
- `Fallback` 不是“导航过程中始终并行监控电量”
- 它更像是“导航节点本轮结束后，再补做一次电量/充电判断”

## 4. 循环导航中的“路网导航”如何实现

当前“路网导航”并不是把整张路网交给一个专门的图搜索节点，而是把 `navigation_points.json` 中的点列当作一条有顺序的路线，分成两层动作执行：
- 先用 `navigate_to_pose` 到最近的路线点
- 再用 `navigate_through_poses` 沿点列跑一段或整段路线

具体逻辑都在 `LoopNavigateFourPoints` 中。

### 4.1 初始化

在 `onStart()` 中，节点先做这些事情：
- 检查 blackboard 上的 `nav_paused`
- 检查 blackboard 上的 `should_stop_navigation`
- 首次进入时加载 `navigation_points.json`
- 等待 `/navigate_through_poses` 和 `/navigate_to_pose` action server

如果这些前置条件不满足，会直接返回 `FAILURE` 或 `SUCCESS`。

### 4.2 先找当前位置最近点

当满足下面任一条件时，会先做“最近点对齐”：
- `initial_alignment_done_ == false`
- `recovery_after_failure_ == true`

处理流程是：

1. 调 `getCurrentPose()` 获取机器人当前位置
2. `getCurrentPose()` 会在短时间窗口内多次尝试 TF
3. 成功后调用 `findNearestGoalIndex()` 计算最近点
4. 根据最近点位置调用 `chooseInitialForward()` 决定初始方向
5. 先发送一个 `navigate_to_pose`，把机器人拉到最近路线点

这里的最近点选择是几何距离最近：
- 遍历 `goals_`
- 比较当前位置与每个目标点的二维欧氏距离
- 取距离最小的那个索引

### 4.3 最近点对齐完成后，发送部分路线

如果 `pending_partial_route_ == true`，说明最近点已经确定，接下来会生成一段“补齐路线”：

- 若方向是正向，则从 `start_index + 1` 一直取到末尾
- 若方向是反向，则从 `start_index - 1` 一直取到开头

这段路线通过 `sendRoute()` 发送给 `navigate_through_poses`。

### 4.4 部分路线结束后，开始整段往返循环

当部分路线结束后：
- 如果刚才是正向补齐，则下一次整段走反向
- 如果刚才是反向补齐，则下一次整段走正向

之后节点会一直在整段正向和整段反向之间来回切换：
- 正向：`goals_` 原顺序
- 反向：`reverse(goals_)`

因此当前“循环导航”本质上是：
- 先接入当前路线上的最近点
- 再跑到一端
- 然后整条路线首尾往返

### 4.5 路线失败后的恢复

如果 `navigate_through_poses` 返回 `ABORTED`：
- 节点不会直接让整棵树失败
- 而是把 `recovery_after_failure_` 置为 `true`
- 记录失败前的方向 `recovery_forward_`
- 下一个 tick 重新从“找最近点 -> 回到路线 -> 按原方向继续”开始

这是一种“重入路线”的恢复机制。

## 5. `LoopNavigateFourPoints` 的状态变化

`LoopNavigateFourPoints` 是 `StatefulActionNode`，所以它有 `onStart()` / `onRunning()` / `onHalted()` 三段状态机。

### 5.1 `onStart()` 可能返回什么

`onStart()` 可能返回：

- `SUCCESS`
  - `nav_paused == true`
  - `should_stop_navigation == true`
  - 最近点刚好已经在当前方向终点，部分路线为空，被直接跳过

- `RUNNING`
  - 成功发送 `navigate_to_pose`
  - 成功发送 `navigate_through_poses`

- `FAILURE`
  - 点位文件缺失或为空
  - action server 不可用
  - 当前位姿在重试窗口内仍获取失败
  - 路线为空且被拒绝发送

### 5.2 `onRunning()` 在最近点对齐阶段

当 `running_action_ == NAVIGATE_TO_POINT` 时：

- 目标还没被 action server 接收：`RUNNING`
- 已接收但结果还没回来：`RUNNING`
- `SUCCEEDED`：返回 `SUCCESS`
- `ABORTED`：返回 `SUCCESS`
- `CANCELED`：返回 `FAILURE`
- goal 被 server 拒绝：返回 `FAILURE`

这里要注意一个关键点：
- 最近点对齐 `ABORTED` 并不会返回 `FAILURE`
- 它会通过修改内部标志，让下一次 tick 重新走对齐逻辑

也就是说，这里的 `SUCCESS` 有时表示“这一步完成了”，有时表示“这一步准备重新组织下一轮导航”，不完全等同于“真正成功抵达”。

### 5.3 `onRunning()` 在路线导航阶段

当 `running_action_ == NAVIGATE_ROUTE` 时：

- route goal 尚未被接受：`RUNNING`
- goal 已接受但路线还没完成：`RUNNING`
- `SUCCEEDED`：返回 `SUCCESS`
- `ABORTED`：返回 `SUCCESS`
- `CANCELED`：返回 `FAILURE`
- goal 被拒绝：返回 `FAILURE`

同样要注意：
- 路线 `ABORTED` 被设计成返回 `SUCCESS`
- 目的不是告诉树“这轮导航完全成功”
- 而是允许下一次 tick 继续进入恢复逻辑

### 5.4 `onHalted()`

当外部 halt 该节点时：
- 如果存在 `navigate_through_poses` goal，就取消它
- 如果存在 `navigate_to_pose` goal，就取消它
- 清理内部 `result_ready` 和 `running_action_`

## 6. 路网导航时节点处于什么状态

按当前实现，路网导航过程中 `LoopNavigateFourPoints` 的状态大致如下：

### 6.1 正在找最近点并发送对齐目标
- 首次 tick 进入 `onStart()`
- 发送 `navigate_to_pose`
- 节点返回 `RUNNING`

### 6.2 正在走向最近点
- 后续 tick 进入 `onRunning()`
- 只要 action 还没结束，就持续返回 `RUNNING`

### 6.3 最近点对齐结束
- 对齐成功：返回 `SUCCESS`
- 对齐失败但准备重试：也可能返回 `SUCCESS`
- 对齐被取消或被拒绝：返回 `FAILURE`

### 6.4 正在执行 `navigate_through_poses`
- 发送路线 goal 后返回 `RUNNING`
- 后续 tick 只要路线还没完成，就一直 `RUNNING`

### 6.5 路线执行结束
- `SUCCEEDED`：返回 `SUCCESS`
- `ABORTED`：返回 `SUCCESS`，准备下次从最近点重入
- `CANCELED` 或 goal 被拒：返回 `FAILURE`

因此，“路网导航进行中”这个阶段，`LoopNavigateFourPoints` 几乎一直都是 `RUNNING`。

## 7. 它和 `Fallback` 的关系

这是当前行为树里最容易误解的地方。

### 7.1 `Fallback` 的基本语义

`Fallback` 会按顺序 tick 子节点：

1. 第一个子节点返回 `SUCCESS`，则 `Fallback` 返回 `SUCCESS`
2. 第一个子节点返回 `FAILURE`，才会继续 tick 第二个子节点
3. 某个子节点返回 `RUNNING`，则 `Fallback` 返回 `RUNNING`

在当前树中：

- 充电版 `Fallback`
  - 子节点1：`CheckBatteryAboveThreshold`
  - 子节点2：`ChargingSequence`

- 无充电版 `Fallback`
  - 子节点1：`CheckBatteryAboveThreshold`
  - 子节点2：`StopSequence`

### 7.2 但它排在 `LoopNavigateFourPoints` 后面

由于根节点是：

```xml
<ReactiveSequence>
  <LoopNavigateFourPoints/>
  <Fallback/>
</ReactiveSequence>
```

所以只有当 `LoopNavigateFourPoints` 返回 `SUCCESS` 时，`Fallback` 才会在同一个 tick 被执行。

这带来几个直接结论：

#### 结论 1：路网导航 `RUNNING` 时，`Fallback` 不参与

当机器人正在：
- 前往最近点
- 执行 `navigate_through_poses`

`LoopNavigateFourPoints` 都会返回 `RUNNING`。  
此时 `ReactiveSequence` 会停止向后 tick，`Fallback` 完全不运行。

#### 结论 2：`LoopNavigateFourPoints` 返回 `FAILURE` 时，`Fallback` 也不会参与

例如：
- 获取当前位置超时失败
- goal 被 action server 拒绝
- 导航被取消

这时根节点会直接 `FAILURE`，主循环通常会退出，后面的 `Fallback` 仍然不会有机会补救。

#### 结论 3：`Fallback` 主要只在两类时机运行

- 路线成功跑完后
- 路线 `ABORTED`，但节点内部把它转换成 `SUCCESS` 之后

换句话说，当前 `Fallback` 更像是：
- “一段导航结束后的后处理分支”

而不是：
- “导航过程中的实时监控分支”

### 7.3 `CancelNavigation` 在当前树中的实际作用

从 XML 看，`CancelNavigation` 像是为了“在切入充电/停止前打断当前导航”。

但在当前树结构下，它通常只有在 `LoopNavigateFourPoints` 已经先返回 `SUCCESS` 之后才有机会被 tick。  
因此它不是一个严格意义上的“导航过程中即时抢占”节点。

它更像是：
- 在进入充电/停止流程前，做一次兜底取消

## 8. 无充电版的额外结束逻辑

无充电版里，`NavigateToFirstPointAndStop` 完成后会把 blackboard 上的 `low_battery_shutdown` 置为 `true`。

主循环在下一轮 tick 前会先检查这个标志：
- 如果为 `true`，直接退出程序

所以“无充电版停止”不是靠行为树根节点返回 `FAILURE` 退出，而是：
- 先让树正常走完停止流程
- 再由主循环读 blackboard 标志主动结束

## 9. 当前实现最重要的理解结论

如果只看当前代码，可以把它概括成下面这几条：

1. `LoopNavigateFourPoints` 是主导节点，负责整条路线的接入、往返和失败重入。
2. 所谓“路网导航”当前实现方式是：
   - `navigate_to_pose` 到最近路线点
   - `navigate_through_poses` 沿点列执行部分路线或整段路线
3. 在实际导航过程中，`LoopNavigateFourPoints` 大多数时间返回 `RUNNING`。
4. 由于根节点是 `ReactiveSequence`，`LoopNavigateFourPoints` 一旦 `RUNNING`，后面的 `Fallback` 就不会被 tick。
5. 因此当前 `Fallback` 不是运行中的并发监控，而是导航节点返回 `SUCCESS` 后才有机会执行的后续分支。
6. 当前结构下，如果想实现“导航进行中实时电量检查并立即切充电/停止”，仅靠现在这个节点顺序是不够的，必须调整树结构或改成别的控制组合。

## 10. 备注

当前 XML 注释、阈值文字说明和代码实际值之间存在一些不一致，例如：
- `loop_navigate_bt.xml` 里注释写的是 `35%`
- 但实际配置值写的是 `threshold="0"`

因此后续如果要继续调整行为树，建议优先以当前代码和 XML 实际值为准，再统一修正文档与注释。
