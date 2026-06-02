# Nav2 局部规划与停障优先避障框架

本文档记录在当前 `dexe_mobile_nav` 的 Navigation2 链路中，如何从现有“全局规划 -> 路径跟踪”扩展为“全局规划 -> 局部规划/停障避障 -> 路径跟踪”。这是设计框架文档，不表示相关代码已经实现。

## 现有链路

当前主链路位于：

- 行为树：`mobile_navigation/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`
- 参数：`mobile_navigation/nav2_bringup/params/w1_nav2_params.yaml`
- 全局规划 action：`planner_server/compute_path_to_pose`
- 路径跟踪 action：`controller_server/follow_path`
- 默认控制器插件：`nav2_mppi_controller::MPPIController`

现有数据流为：

```text
{goal}
  -> ComputePathToPose(planner_id="SmacGridBased")
  -> {path}
  -> FollowPath(controller_id="FollowPath")
  -> controller_server / MPPI
  -> /cmd_vel
```

`planner_server` 已支持多个 planner 插件，并通过 `active_planner` 与 `override_planner_id` 支持运行时热切换。当前配置中已有 `RouteGridPlanner`、`SmacGridBased`、`ThetaStarBased`。

## 目标链路

推荐在全局规划和控制器之间插入“局部路径生成层”：

```text
{goal}
  -> ComputePathToPose(global planner)
  -> {global_path}
  -> SelectLocalGoalFromPath
  -> {local_goal}
  -> 停障优先判断
      -> 障碍短时存在：停止并等待
      -> 障碍持续超时：ComputePathToPose(local planner)
  -> {local_path}
  -> FollowPath
  -> controller_server / MPPI
  -> /cmd_vel
```

核心原则：

- 全局 planner 负责长距离路径和任务目标。
- 局部 planner 只负责当前位置到全局路径前方局部目标点的短距离避障路径。
- `FollowPath` 仍只消费 `nav_msgs/msg/Path`，不感知路径来自全局规划还是局部规划。
- 停障/避障策略由行为树编排，避免把流程状态塞进控制器或 planner 插件。

## Planner 插件设计

建议新增一个局部规划插件，例如 `LocalAvoidancePlanner`，仍实现 `nav2_core::GlobalPlanner` 接口。虽然接口名是 `GlobalPlanner`，但在 Nav2 中它本质是“给定 start 和 goal 输出 `nav_msgs/msg/Path` 的 planner 插件”，可以用于短距离局部路径生成。

示例参数：

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["RouteGridPlanner", "SmacGridBased", "ThetaStarBased", "LocalAvoidancePlanner"]

    LocalAvoidancePlanner:
      plugin: "nav2_local_planner/LocalAvoidancePlanner"
      max_planning_time: 0.2
      local_goal_distance: 2.0
      max_path_length: 4.0
      obstacle_cost_threshold: 220.0
      allow_unknown: false
      use_local_costmap: true
      prefer_global_path: true
```

局部 planner 的输入建议为：

- `start`：当前机器人位姿。
- `goal`：从 `{global_path}` 上提取的 `{local_goal}`。
- `planner_id`：`LocalAvoidancePlanner`。

局部 planner 的输出为：

- `{local_path}`：短距离可跟踪路径，供 `FollowPath` 消费。

## 行为树节点建议

建议新增或复用以下 BT 节点：

| 节点 | 类型 | 职责 |
| --- | --- | --- |
| `SelectLocalGoalFromPath` | SyncAction | 从 `{global_path}` 中按前瞻距离提取 `{local_goal}` |
| `GetCurrentPose` | SyncAction 或 Condition | 从 TF 获取当前机器人位姿并写入 `{robot_pose}` |
| `IsLocalPathBlocked` | Condition | 检查机器人前方路径或局部窗口内是否存在超过阈值的障碍 |
| `PersistentObstacleTimer` | Condition | 障碍持续超过配置时间后返回成功，障碍消失时重置计时 |
| `ControllerCancel` | CancelAction | 停障前取消正在执行的 `FollowPath` action goal |

如果现有 `TruncatePath` 或 `TruncatePathLocal` 能满足局部路径截断需求，可优先复用；若需要输出单个局部目标点，则新增 `SelectLocalGoalFromPath` 更清晰。

## 行为树骨架

示例 XML 只表达结构，具体标签名应以实际实现的 BT 插件为准：

```xml
<PipelineSequence name="NavigateWithGlobalAndLocalPlanning">

  <RateController hz="0.333">
    <RecoveryNode number_of_retries="6" name="GlobalPlanningRetry">
      <ComputePathToPose goal="{goal}" path="{global_path}" planner_id="SmacGridBased"/>
      <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
    </RecoveryNode>
  </RateController>

  <SelectLocalGoalFromPath
    global_path="{global_path}"
    local_goal="{local_goal}"
    lookahead_dist="2.0"
    min_remaining_dist="0.3"/>

  <ReactiveFallback name="StopThenAvoidOrTrack">
    <Sequence name="ObstacleHandling">
      <IsLocalPathBlocked
        path="{global_path}"
        check_distance="2.0"
        cost_threshold="220"/>

      <ReactiveFallback name="WaitOrAvoid">
        <Sequence name="StopWhileObstacleIsRecent">
          <Inverter>
            <PersistentObstacleTimer seconds="3.0" reset_when_clear="true"/>
          </Inverter>
          <ControllerCancel server_name="controller_server"/>
          <Wait wait_duration="0.2" server_name="wait_local"/>
        </Sequence>

        <Sequence name="AvoidPersistentObstacle">
          <GetCurrentPose pose="{robot_pose}"/>
          <ComputePathToPose
            start="{robot_pose}"
            goal="{local_goal}"
            path="{local_path}"
            planner_id="LocalAvoidancePlanner"/>
          <FollowPath path="{local_path}" controller_id="FollowPath"/>
        </Sequence>
      </ReactiveFallback>
    </Sequence>

    <FollowPath path="{global_path}" controller_id="FollowPath"/>
  </ReactiveFallback>
</PipelineSequence>
```

## 停障优先逻辑

推荐状态语义：

1. 无障碍：跟踪 `{global_path}`。
2. 前方检测窗口发现障碍：取消当前 `FollowPath`，进入短周期 `Wait` 或自定义停车动作，确保机器人停止。
3. 障碍在 `obstacle_wait_timeout` 内消失：恢复跟踪 `{global_path}`，不触发绕行。
4. 障碍持续超过 `obstacle_wait_timeout`：调用 `LocalAvoidancePlanner`，输出 `{local_path}`。
5. 局部路径跟踪完成或障碍消失：重新回到全局路径跟踪。
6. 局部规划失败：清理 local costmap 后重试；仍失败则进入现有恢复逻辑。

建议默认参数：

```yaml
local_planning:
  enabled: true
  lookahead_dist: 2.0
  check_distance: 2.0
  obstacle_wait_timeout: 3.0
  local_replan_hz: 2.0
  max_local_plan_length: 4.0
```

## 模块边界

- `nav2_bt_navigator`：加载行为树 XML 和 BT 插件，不实现避障算法。
- `nav2_behavior_tree`：新增条件/动作节点，负责黑板变量、计时、路径截取、障碍检测和 action 编排。
- `nav2_planner`：继续承载 `compute_path_to_pose` action server 和 planner 插件加载。
- `nav2_local_planner`：实现短距离局部避障规划算法，读取 costmap 并输出 `nav_msgs/msg/Path`。
- `nav2_controller`：保持现有 `follow_path` action 语义，继续由 `MPPIController` 跟踪输入路径。

## 验证建议

不默认执行 `colcon build`。实现后建议按以下顺序验证：

1. 静态检查：确认新增 BT 插件库已加入 `bt_navigator.plugin_lib_names`。
2. 参数检查：确认 `LocalAvoidancePlanner` 已加入 `planner_plugins`。
3. BT 装载检查：启动导航时确认 XML 中所有节点可注册。
4. 场景测试：无障碍、短时障碍、长时障碍、局部规划失败四类场景。
5. Action 查询：使用 `ros2 action list`、`ros2 action type /compute_path_to_pose`、`ros2 action type /follow_path` 确认接口仍一致。

## 后续实现注意事项

- `override_planner_id: true` 会忽略 BT 传入的 `planner_id`，如果局部规划也复用 `planner_server/compute_path_to_pose`，需要为局部规划调用保留 planner_id 语义，或扩展 planner_server 使全局热切换不影响局部 planner。
- 停障时要明确取消或覆盖当前 `FollowPath` goal，避免控制器继续沿旧路径输出速度。
- `PersistentObstacleTimer` 应在障碍消失时重置，避免下一次障碍直接进入绕行。
- `{global_path}`、`{local_path}`、`{local_goal}`、`{robot_pose}` 等 blackboard key 必须命名清晰，避免覆盖现有 `{path}` 造成调试困难。
