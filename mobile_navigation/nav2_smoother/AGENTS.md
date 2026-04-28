# nav2_smoother 说明

## 1. 这个包在导航链路里的位置

`nav2_smoother` 是一个独立的 Task Server，对外提供 `smooth_path` action。

它的职责不是重新规划，而是对 `planner_server` 产出的离散路径做后处理，使路径更平滑、更连续，必要时再做碰撞检查。随后 BT 再把平滑后的路径交给 `controller_server` 的 `FollowPath`。

典型链路应为：

`ComputePathToPose / ComputePathThroughPoses`
-> `SmoothPath`
-> `FollowPath`

注意：

- `FollowPath` 自身不负责调用 `nav2_smoother`，它只消费输入端口里的 `path`。
- 是否使用平滑路径，完全取决于 BT XML 是否把 `SmoothPath` 插到了规划和跟踪之间。

## 2. 当前仓库中的实际状态

当前默认主导航树 `mobile_navigation/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml` 的核心链路是：

`ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"`
-> `FollowPath path="{path}" controller_id="FollowPath"`

中间没有 `SmoothPath` 节点。

因此，当前 `FollowPath` 并没有直接使用 `nav2_smoother` 的输出路径。

但是需要区分另一件事：

- 在 `mobile_navigation/nav2_bringup/params/w1_nav2_params.yaml`
- 以及 `mobile_navigation/nav2_bringup/params/w1_nav2_rgbd_params.yaml`

里，`GridBased` 当前配置为 `nav2_smac_planner/SmacPlannerLattice`，并且设置了：

```yaml
smooth_path: True
smoother:
  max_iterations: 1000
  w_smooth: 0.3
  w_data: 0.2
  tolerance: 1.0e-10
  do_refinement: true
  refinement_num: 2
```

这表示：

- 当前系统大概率已经在使用 `SmacPlannerLattice` 自带的规划后平滑。
- 但没有走 `nav2_smoother` 这条独立的 server/action/BT 链路。

所以“当前有没有路径平滑”这个问题，答案应拆成两层：

- `nav2_smoother` 独立平滑链路：当前主树没有使用。
- `SmacPlannerLattice` 内置平滑：当前参数中已开启。

## 3. nav2_smoother 的工作原理

### 3.1 SmootherServer 做了什么

`mobile_navigation/nav2_smoother/src/nav2_smoother.cpp`

`SmootherServer` 在收到 `smooth_path` action 后，会：

1. 根据 `smoother_id` 选择一个平滑插件。
2. 取出 goal 里的原始路径 `goal->path`。
3. 调用对应插件的 `smooth(path, max_smoothing_duration)`。
4. 发布平滑后的路径到 `plan_smoothed`。
5. 如果 `check_for_collisions` 为真，则逐点检查平滑后的路径是否碰撞。
6. 返回 `smoothed_path`、`smoothing_duration`、`was_completed`。

其中：

- `was_completed = false` 通常表示超时或算法未在允许时间内完成，会返回当前可用结果。
- `check_for_collisions` 是平滑后的附加安全检查，不是所有插件内部都做完整碰撞约束。

### 3.2 为什么要先分段再平滑

`mobile_navigation/nav2_smoother/include/nav2_smoother/smoother_utils.hpp`

平滑前会先调用 `findDirectionalPathSegments(path)` 对路径分段，避免把不该连续平滑的部分硬抹平。分段依据主要有两类：

- 方向反转 cusp：通过前后两段位移向量点积小于 0 判断。
- 原地转向段：位置几乎不变，但 yaw 发生明显变化。

这样做的意义是：

- 前进段、倒车段、原地转向段分别处理，避免把倒车拐点抹掉。
- 对非完整约束底盘更安全，能保留轨迹的运动学意图。

平滑完成后还会调用 `updateApproximatePathOrientations()`，按相邻路径点重新估计朝向；若该段是倒车段，则会把朝向翻转 180 度，使姿态与运动方向一致。

## 4. 两种平滑插件的原理

### 4.1 SimpleSmoother

对应实现：

`mobile_navigation/nav2_smoother/src/simple_smoother.cpp`

这是一个迭代优化型平滑器，核心更新公式可理解为：

- 一项把当前点拉回原始离散路径，避免偏离太多，对应 `w_data`
- 一项把当前点向前后邻点的均值靠拢，减少折线感，对应 `w_smooth`

每轮对内部点的 `x/y` 更新近似为：

`y_i = y_i + w_data * (x_i - y_i) + w_smooth * (y_{i+1} + y_{i-1} - 2 * y_i)`

可以把它理解成“数据保真 + 二阶平滑”的折中。

主要特性：

- 首尾点固定，中间点迭代调整。
- 当总改变量小于 `tolerance` 时认为收敛。
- 超过 `max_its` 或超过 `max_smoothing_duration` 会提前退出。
- 若提供了 costmap，会检查更新后的点是否进入障碍物；一旦引入不可行点，就回退到上一版可行路径。
- 可通过 `do_refinement` 做额外几轮递归细化。

适用场景：

- 想显著削弱折线感。
- 希望在保留原路径拓扑的同时做较柔和的形状优化。
- 对障碍物可行性比较敏感，希望平滑过程具备基础避障约束。

主要参数含义：

- `w_data`：越大越贴原路径，改动越保守。
- `w_smooth`：越大越趋向光滑，但过大可能拉偏路径。
- `tolerance`：收敛阈值。
- `max_its`：最大迭代次数。
- `do_refinement`：是否追加几轮细化。

### 4.2 SavitzkyGolaySmoother

对应实现：

`mobile_navigation/nav2_smoother/src/savitzky_golay_smoother.cpp`

这是一个固定窗口的 Savitzky-Golay 滤波器实现，当前使用 7 点滤波核：

`[-2, 3, 6, 7, 6, 3, -2] / 21`

对路径点的 `x/y` 坐标做局部卷积平滑，本质上属于信号滤波型平滑。

主要特性：

- 计算量小、速度快、结果稳定。
- 更像“去噪”和“减小局部抖动”，不会像迭代优化那样大幅改变整体形状。
- 边界点使用重复端点的方式处理。
- 支持 `do_refinement` 和 `refinement_num` 多次重复滤波。
- 该实现本身不在插件内部做 costmap 可行性检查，更多依赖 `SmootherServer` 的事后碰撞检查。

适用场景：

- 路径总体可用，只是局部采样抖动、锯齿较明显。
- 希望低时延、轻量级处理。
- 不希望明显改变原路径拓扑。

## 5. 当前 FollowPath 为什么没直接使用平滑路径

`mobile_navigation/nav2_behavior_tree/plugins/action/follow_path_action.cpp`

`FollowPathAction::on_tick()` 只做三件事：

1. 读取输入端口 `path`
2. 读取 `controller_id`
3. 读取 `goal_checker_id`

也就是说，控制器拿到什么路径，完全由 BT 黑板变量决定。

而当前主树把 `ComputePathToPose` 的输出直接写进 `{path}`，随后 `FollowPath path="{path}"` 直接消费它，所以不会自动去找 `nav2_smoother`。

## 6. 要让 FollowPath 使用平滑路径，推荐怎么接

最直接的方法，是在 BT 里把 `SmoothPath` 插到规划和控制之间，并让 `FollowPath` 消费 `smoothed_path`。

可参考下面的最小接入方式：

```xml
<PipelineSequence name="NavigateWithReplanning">
  <RateController hz="0.2">
    <RecoveryNode number_of_retries="1" name="ComputePathToPose">
      <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
      <ClearEntireCostmap
        name="ClearGlobalCostmap-Context"
        service_name="global_costmap/clear_entirely_global_costmap"/>
    </RecoveryNode>
  </RateController>

  <RecoveryNode number_of_retries="1" name="SmoothPath">
    <SmoothPath
      unsmoothed_path="{path}"
      smoothed_path="{smoothed_path}"
      smoother_id="simple_smoother"
      max_smoothing_duration="0.5"
      check_for_collisions="true"/>
    <ClearEntireCostmap
      name="ClearGlobalCostmap-AfterSmooth"
      service_name="global_costmap/clear_entirely_global_costmap"/>
  </RecoveryNode>

  <RecoveryNode number_of_retries="1" name="FollowPath">
    <FollowPath path="{smoothed_path}" controller_id="FollowPath"/>
    <ClearEntireCostmap
      name="ClearLocalCostmap-Context"
      service_name="local_costmap/clear_entirely_local_costmap"/>
  </RecoveryNode>
</PipelineSequence>
```

关键点只有两个：

- `SmoothPath` 的输入接 `{path}`，输出接 `{smoothed_path}`。
- `FollowPath` 的 `path` 端口改为 `{smoothed_path}`。

## 7. 接入前的必要条件

如果要真正跑通上面的 BT，至少要确认以下几点：

1. `bt_navigator.plugin_lib_names` 已加载 `nav2_smooth_path_action_bt_node`
2. `navigation_launch.py` 已启动 `smoother_server`
3. `lifecycle_manager_navigation` 已管理 `smoother_server`
4. 参数文件里 `smoother_server.ros__parameters` 已正确配置插件

本仓库现状：

- 第 1、2、3 项已经具备。
- `w1_nav2_params.yaml` 和 `w1_nav2_rgbd_params.yaml` 中 `smoother_server` 参数块当前是注释状态。

如果保持注释：

- 节点可能仍会按默认参数启动。
- 但工程上不建议依赖默认值，最好显式写出 `smoother_plugins` 与插件参数。

## 8. 是否建议在当前框架里再接一层 nav2_smoother

建议先区分目标，再决定是否加：

### 方案 A：继续只用 Smac 内置平滑

适合：

- 当前轨迹已较平顺。
- 更关心链路简单、延迟更低、调参点更少。
- MPPI 已能稳定跟踪。

优点：

- 改动最小。
- 不增加额外 action 调用和 BT 节点。

风险：

- 平滑能力受限于 planner 内置实现。
- 难以按场景切换不同 smoother。

### 方案 B：在 BT 中接入 nav2_smoother

适合：

- 希望把“规划”和“平滑”分成两层，便于独立调参。
- 需要按任务切换 `simple_smoother` / `savitzky_golay_smoother`。
- 想在平滑后显式做碰撞检查。

优点：

- 职责更清晰。
- 可观测性更好，能单独看 `smooth_path` 耗时和结果。

风险：

- 增加一点点导航链路时延。
- 若 `Smac` 内置平滑仍开启，可能形成“双重平滑”，导致局部转角被过度抹平。

对当前 MPPI 控制器，更推荐的工程做法通常是二选一：

- 要么仅保留 `Smac` 内置平滑。
- 要么减弱或关闭 `Smac` 的 `smooth_path`，改为显式接入 `nav2_smoother`。

不建议一开始就两层都拉满。

## 9. 最小验证方法

如果后续把 `SmoothPath` 接进 BT，建议按下面顺序做低成本验证：

1. 先观察 `plan_smoothed` 话题是否有输出

```bash
ros2 topic echo /plan_smoothed
```

2. 检查 `FollowPath` 实际消费的是不是平滑后的黑板路径

可通过 BT 日志、RViz path 显示，或临时在控制器入口打印收到的路径首尾点与数量。

3. 重点观察以下现象是否变差

- 倒车 cusp 附近是否被抹平
- 终点附近是否出现多余摆动
- 狭窄区域是否更贴障
- 规划重算后控制是否更滞后

4. 若使用 MPPI，重点看

- 跟踪误差是否降低
- `cmd_vel` 是否更连续
- 局部最优或过度保守是否增加

## 10. 一句话结论

`nav2_smoother` 的本质是“对规划路径做独立后处理的 server + plugin 体系”；当前仓库主导航树没有把它接到 `FollowPath` 前面，所以 `FollowPath` 目前并未直接使用 `nav2_smoother` 的结果，而更可能是在使用 `SmacPlannerLattice` 自带的内置平滑结果。
