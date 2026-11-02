# BT Navigator

The BT Navigator (Behavior Tree Navigator) module implements the NavigateToPose and NavigateThroughPoses task interfaces. It is a [Behavior Tree](https://github.com/BehaviorTree/BehaviorTree.CPP/blob/master/docs/BT_basics.md)-based implementation of navigation that is intended to allow for flexibility in the navigation task and provide a way to easily specify complex robot behaviors.

See its [Configuration Guide Page](https://navigation.ros.org/configuration/packages/configuring-bt-navigator.html) for additional parameter descriptions, as well as the [Nav2 Behavior Tree Explanation](https://navigation.ros.org/behavior_trees/index.html) pages explaining more context on the default behavior trees and examples provided in this package.

## Overview

The BT Navigator receives a goal pose and navigates the robot to the specified destination(s). To do so, the module reads an XML description of the Behavior Tree from a file, as specified by a Node parameter, and passes that to a generic [BehaviorTreeEngine class](../nav2_behavior_tree/include/nav2_behavior_tree/behavior_tree_engine.hpp) which uses the [Behavior-Tree.CPP library](https://github.com/BehaviorTree/BehaviorTree.CPP) to dynamically create and execute the BT. The BT XML can also be specified on a per-task basis so that your robot may have many different types of navigation or autonomy behaviors on a per-task basis.

## 本工程 NavigateToPose 执行链路

本工程默认单目标导航行为树为 `behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`。

1. `NavigateToPoseNavigator` 接收 `navigate_to_pose` action goal，或通过 `goal_pose` 话题转发出的自调用 action goal。
2. `goalReceived()` 加载本次请求指定的 BT XML；未指定时使用默认 XML。
3. `initializeGoalPose()` 获取当前机器人位姿用于日志与反馈初始化，并将目标位姿写入 BT blackboard 的 `{goal}`。
4. 行为树中的 `ComputePathToPose goal="{goal}" path="{path}" planner_id="SmacGridBased"` 调用 `planner_server/compute_path_to_pose`，将规划结果写入 `{path}`。
5. 行为树中的 `FollowPath path="{path}" controller_id="FollowPath"` 调用 `controller_server/follow_path`，把路径交给局部控制器。
6. `controller_server` 选择 `FollowPath` 控制器插件，调用 `setPlan()` 后循环计算速度并发布 `/cmd_vel`。
7. `onLoop()` 每次 BT tick 后发布 `navigate_to_pose` action feedback，包括当前位姿、剩余路径长度、估计剩余时间、恢复次数和导航耗时。

默认 XML 使用 `PipelineSequence` 串联全局规划和局部控制。`AdaptiveReplanning` 的
`enable_global_replanning` 是仅用于 `NavigateToPose` 的总开关，默认值为 `true`：开启时保留周期、
路径偏离、路径障碍和恢复触发的重规划；关闭时只执行首次规划和目标变化后的规划。当前默认
`hz="0.02"`，即无其他触发条件时每 50 秒周期重规划一次。新路径写入 `{path}` 后，
`FollowPathAction` 会重新发送 `follow_path` goal，`controller_server::updateGlobalPath()` 接收 pending
goal 并更新控制器中的路径。

规划失败时，行为树优先清理 global costmap 并重试；控制失败时，行为树优先清理 local costmap、等待后重试。恢复动作只负责重试调度，规划算法仍由 `planner_server` 插件执行，速度控制仍由 `controller_server` 插件执行。
