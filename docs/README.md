# dexe_mobile_nav 文档

本目录记录仓库级维护说明，当前重点覆盖核心目录功能总览、Nav2 主链路和 `package.xml` 元数据同步。

## 三个核心目录功能总览

当前工作区的移动机器人导航能力主要由 `dexe_sensor_process`、`mobile_perception` 和 `mobile_navigation` 三个目录组成。三者的边界可以按数据链路理解：传感器预处理先把深度相机数据转换为点云，感知与末端控制包提供 AprilTag、泊车和底盘精控能力，导航包负责 Nav2 全局规划、局部控制、代价地图、行为树和生命周期管理。

### `dexe_sensor_process`

`dexe_sensor_process` 是传感器数据预处理包，目前核心可执行节点为 `rgbd_to_pointcloud`。该节点订阅 RGBD 相机深度图和相机内参，将深度图转换为 `sensor_msgs/msg/PointCloud2`，并按参数输出原始点云与预处理点云。

主要入口：

- 源码：`dexe_sensor_process/src/rgbd_to_pointcloud.cpp`
- 单相机启动：`dexe_sensor_process/launch/rgbd_to_pointcloud.launch.py`
- 前后双相机启动：`dexe_sensor_process/launch/multi_camera.launch.py`
- 自动模式前视点云启动：`dexe_sensor_process/launch/front_p2c.launch.py`
- 自动模式后视点云启动：`dexe_sensor_process/launch/rear_p2c.launch.py`
- 包配置：`dexe_sensor_process/package.xml`、`dexe_sensor_process/CMakeLists.txt`

节点默认话题规则：

- 输入深度图：`{input_topic}/depth`，未指定 `input_topic` 时为 `/orbbec/{camera_name}/depth`。
- 输入相机内参：`{input_topic}/camera_info`。
- 原始点云输出：`{input_topic}/pointcloud2_raw`。当前会随深度回调发布；如果后续需要降低带宽或 CPU 压力，可以增加参数控制是否发布 raw，或仅在有订阅者时发布。
- 预处理点云输出：`{input_topic}/pointcloud2_filtered`。
- LaserScan 输出参数名为 `output_topic_laserscan`，源码中保留了点云压缩为 `LaserScan` 的实现，但当前发布路径处于注释状态，实际主输出是 raw 与 filtered 点云。

主要处理逻辑：

- 使用最近一次收到的 `CameraInfo` 中 `fx`、`fy`、`cx`、`cy` 反投影深度图。
- 深度单位按毫米换算为米，当前截断距离为 3.0 m。
- 为降低计算量，反投影时按固定步长采样像素。
- filtered 点云先执行体素降采样，再复制一份转换到 `ground_filter_frame`，默认 `base_link`，按该坐标系下的 `z` 高度判断是否保留障碍点；最终发布的 filtered 点云仍保持原始相机 frame，便于 Nav2 costmap 按传感器来源处理。
- `ground_filter_min_height` 以下的点认为接近地面并剔除；`ground_filter_max_height` 以上的点认为过高，不作为近地障碍输入。
- 地面过滤不再使用相机坐标系 Y 轴阈值，避免前后 RGBD 相机存在俯仰安装角时，不同距离的地面点漏过滤。
- `flag_voxel_filter` 当前实际生效；`flag_mls_filter`、`flag_sor_filter` 参数已声明并由 launch 传入，但源码当前未执行 MLS 和 SOR 过滤。

常用参数：

- `camera_name`：默认 `rear`，用于构造 `/orbbec/{camera_name}`。
- `input_topic`、`output_topic_raw`、`output_topic_filtered`、`output_topic_laserscan`：显式覆盖输入输出话题。
- `flag_voxel_filter`、`ds_leaf_size`：体素降采样开关与体素尺寸。
- `ground_filter_frame`：地面过滤使用的坐标系，默认 `base_link`。
- `ground_filter_min_height`：`ground_filter_frame` 下低于该高度的点认为是地面并剔除；调高会过滤更多低矮点，但可能漏掉低矮障碍物。
- `ground_filter_max_height`：`ground_filter_frame` 下高于该高度的点不作为近地障碍输入；调低会减少高处点进入 costmap。
- `flag_mls_filter`、`mls_polynomial_order`、`mls_search_radius`、`flag_sor_filter`、`sor_mean_k`、`sor_stddev_mul_thresh`：保留参数，当前代码未实际应用到 filtered 输出。

与导航的关系：

- `multi_camera.launch.py` 默认输出 `/orbbec/front/pointcloud2_raw`、`/orbbec/front/pointcloud2_filtered`、`/orbbec/rear/pointcloud2_raw`、`/orbbec/rear/pointcloud2_filtered`。
- 自动模式脚本分别启动 `front_p2c.launch.py` 和 `rear_p2c.launch.py`，用于持续发布前后 RGBD filtered 点云；排查现场时应同时检查安装包 launch 和部署脚本。
- `mobile_navigation/nav2_bringup/params/w1_nav2_rgbd_params.yaml` 使用 Orbbec 前后 filtered 点云作为 costmap 的 PointCloud2 observation source。
- `mobile_navigation/nav2_bringup/params/w1_nav2_params.yaml` 也引用了前后 Orbbec filtered 点云作为障碍层输入之一。

### `mobile_perception`

`mobile_perception` 是感知、目标发布、精确泊车和底盘末端控制包。它包含 C++ 主线节点和一批 Python 调试/编排脚本，其中 C++ 节点是当前更稳定的工程入口。

主要入口：

- README：`mobile_perception/README.md`
- 启动文件：`mobile_perception/launch/mobile_control.launch.py`、`mobile_perception/launch/mobile_control_action.launch.py`
- 参数：`mobile_perception/config/mobile_control_params.yaml`、`mobile_perception/config/mobile_control_action_params.yaml`
- 相机和 tag 配置：`mobile_perception/config/camera_config*.json`、`mobile_perception/config/tags_config.json`
- docking 配置：`mobile_perception/config/docking_config*.json`、`mobile_perception/config/docking.yaml`

核心 C++ 节点：

- `apriltag_detection_node`：按服务启动/停止 AprilTag 检测，发布 `apriltag_poses`，并发布调试 TF。
- `apriltag_alignment_tf_node`：持续检测 AprilTag，发布 raw/aligned tag TF、调试位姿和基于 tag 的目标位姿。
- `mobile_control_node`：订阅 `control/ctrl_pose` 和 `feedback/odom`，输出 `control/cmd_vel`，用于目标位姿下的底盘末端精控。
- `mobile_control_action_node`：通过 `control/precise_control` action 接收精控目标，执行 TF 转换、控制、反馈和取消处理。
- `obstacle_detector_node`：订阅局部 costmap，检测车体前方矩形区域是否存在障碍物，并发布 Bool 与 RViz Marker。
- `charging_pose_service_node`：采样 `map -> base_link` 并保存充电位姿 JSON。

Python 脚本功能：

- `detect_apriltags*.py`：Python 版 AprilTag 检测和服务化调试。
- `control_robot*.py`、`goal_controller*.py`：早期控制和仿真控制脚本。
- `nav_then_precise*.py`：Nav2 导航后接 AprilTag 精控的编排脚本。
- `nav_goal_publisher*.py`、`nav_goal_server.py`：向 Nav2 发送导航目标或提供目标服务。
- `create_charging_pose.py`、`creat_charging_pose_service.py`：充电位姿采集辅助脚本。

关键接口：

- `control/cmd_vel`：底盘控制速度输出。不要让 `mobile_control_node` 与 `mobile_control_action_node` 同时控制同一条速度链路。
- `feedback/odom`：控制节点使用的底盘里程计反馈。
- `control/ctrl_pose`：`mobile_control_node` 的目标位姿输入。
- `control/precise_control`：`mobile_control_action_node` 的精控 action。
- `apriltag_poses`、`apriltag_poses_debug`、`apriltag_target_pose`：AprilTag 检测与对齐目标输出。
- `start_apriltag_detection`、`stop_apriltag_detection`：AprilTag 检测资源开关服务。
- `obstacle_detector/has_obstacle`、`obstacle_detector/markers`、`obstacle_detector/detection_box`：前方障碍检测结果和可视化输出。

### `mobile_navigation`

`mobile_navigation` 是基于 ROS 2 Navigation2 的导航栈目录，包含 Nav2 上游包、本仓库定制包和 W1 机器人参数。它负责定位、地图服务、行为树导航、全局规划、局部控制、代价地图、恢复行为、路径平滑、速度平滑、RViz 插件和消息接口。

主要入口：

- Nav2 bringup：`mobile_navigation/nav2_bringup`
- 默认参数：`mobile_navigation/nav2_bringup/params/nav2_params.yaml`
- W1 导航参数：`mobile_navigation/nav2_bringup/params/w1_nav2_params.yaml`
- W1 RGBD 导航参数：`mobile_navigation/nav2_bringup/params/w1_nav2_rgbd_params.yaml`
- 主启动文件：`mobile_navigation/nav2_bringup/launch/bringup_launch.py`
- 导航节点启动：`mobile_navigation/nav2_bringup/launch/navigation_launch.py`

启动链路：

- `bringup_launch.py` 负责组合 localization、SLAM 或 navigation 子 launch，并支持 namespace、composition、respawn、map、params_file、use_sim_time 等参数。
- `navigation_launch.py` 启动或加载 `controller_server`、`smoother_server`、`planner_server`、`behavior_server`、`bt_navigator`、`waypoint_follower`、`velocity_smoother` 和 `lifecycle_manager_navigation`。
- 本仓库在 navigation launch 中把 `controller_server` 的 `cmd_vel` remap 到 `cmd_vel_nav`，便于后级速度链路与其他控制来源做仲裁或接入。

导航主链路：

- `bt_navigator` 接收 `navigate_to_pose` action 或 `goal_pose` 目标，并通过行为树调度规划、控制和恢复。
- `planner_server` 提供 `compute_path_to_pose` action，调用 `nav2_core::GlobalPlanner` 插件生成 `nav_msgs/msg/Path`。
- `controller_server` 提供 `follow_path` action，调用 `nav2_core::Controller` 插件周期性计算速度。
- 默认 W1 参数中 `FollowPath` 使用 `nav2_mppi_controller::MPPIController`。
- `velocity_smoother` 对导航速度输出做后处理。
- `lifecycle_manager_navigation` 管理导航生命周期节点自动 configure/activate。

主要包分组：

- 导航编排与生命周期：`nav2_bringup`、`nav2_bt_navigator`、`nav2_behavior_tree`、`nav2_lifecycle_manager`、`nav2_waypoint_follower`、`nav2_simple_commander`。
- 规划：`nav2_planner`、`nav2_smac_planner`、`nav2_navfn_planner`、`nav2_theta_star_planner`、`nav2_route_planner`。
- 控制：`nav2_controller`、`nav2_mppi_controller`、`nav2_regulated_pure_pursuit_controller`、`nav2_rotation_shim_controller`、`nav2_graceful_controller`、`nav2_dwb_controller`。
- 地图与代价地图：`nav2_map_server`、`nav2_costmap_2d`、`nav2_voxel_grid`、`nonpersistent_voxel_layer`。
- 路径后处理和安全：`nav2_smoother`、`nav2_constrained_smoother`、`nav2_velocity_smoother`、`nav2_collision_monitor`、`nav2_behaviors`。
- 通用接口：`nav2_msgs`、`nav2_core`、`nav2_common`、`nav2_util`、`nav2_rviz_plugins`。

本仓库相对上游 Nav2 的重点定制：

- `nav2_planner` 增加 `active_planner` 和 `override_planner_id`，支持在不修改 BT 的情况下运行时热切换 planner。
- `w1_nav2_params.yaml` 中注册 `RouteGridPlanner`、`SmacGridBased`、`ThetaStarBased` 三个全局规划器，默认 `active_planner` 为 `SmacGridBased`，`override_planner_id` 为 `true`。
- `nav2_route_planner` 提供路网规划能力，可结合 costmap-aware bypass 逻辑处理路网段绕障。
- `nav2_theta_star_planner` 当前约束为只允许原始 costmap 自由栅格通行，避免路径穿越膨胀层。
- `nav2_mppi_controller` 接入 `VelocityDeadbandCritic`，用于惩罚落入底盘速度死区的采样轨迹；W1 参数已在 MPPI `critics` 列表中启用该 critic。
- `nav2_controller` 使用 `PoseProgressChecker`、`LongitudinalGoalChecker` 等更贴近车体纵向/横向误差的检查逻辑。
- `nav2_costmap_2d` 参数中启用静态层、障碍层、体素层、去噪层、膨胀层以及 keepout/speed costmap filters；RGBD 配置额外使用 Orbbec 前后点云。

常用运行示例：

```bash
ros2 launch nav2_bringup bringup_launch.py \
  map:=/path/to/map.yaml \
  params_file:=/path/to/w1_nav2_params.yaml \
  use_sim_time:=false
```

切换规划器示例：

```bash
ros2 param set /planner_server active_planner RouteGridPlanner
ros2 param set /planner_server active_planner SmacGridBased
ros2 param set /planner_server active_planner ThetaStarBased
```

维护注意事项：

- 修改 `mobile_navigation/nav2_bringup/params/w1_nav2_params.yaml` 或 `w1_nav2_rgbd_params.yaml` 时，需要同时确认 `dexe_sensor_process` 输出话题、雷达话题和 TF frame 是否匹配。
- 修改 planner/controller 插件 ID 时，需要同步检查 BT XML、planner/controller server 参数和本文档的导航链路说明。
- 修改 `cmd_vel` remap 或精控速度链路时，需要同时检查 `mobile_perception` 中的 `control/cmd_vel`，避免导航控制和精控节点竞争同一底盘入口。

## Nav2 行为树导航链路

- `bt_navigator` 接收 `navigate_to_pose` action 或 `goal_pose` 话题目标后，将目标位姿写入行为树 blackboard 的 `{goal}`。
- 默认单目标行为树为 `mobile_navigation/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`。
- 全局规划由 `<ComputePathToPose goal="{goal}" path="{path}" planner_id="SmacGridBased"/>` 发起，BT 节点把请求转发到 `planner_server` 的 `compute_path_to_pose` action。
- `planner_server` 接收请求后确定起点、转换到 global frame，并调用当前有效 planner 插件的 `createPlan()` 生成 `nav_msgs/msg/Path`。
- 成功规划后的路径写回 blackboard `{path}`，随后 `<FollowPath path="{path}" controller_id="FollowPath"/>` 将路径发到 `controller_server` 的 `follow_path` action。
- `controller_server` 根据 `controller_id` 选择控制器插件，先调用 `setPlan(path)`，再按 `controller_frequency` 循环调用 `computeVelocityCommands()` 并发布 `/cmd_vel`。
- 本工程默认 `FollowPath` 控制器插件为 `nav2_mppi_controller::MPPIController`，配置位于 `mobile_navigation/nav2_bringup/params/w1_nav2_params.yaml`。
- `PipelineSequence` 允许跟踪路径时继续 tick 前置规划分支；`RateController` 控制重规划频率；`FollowPathAction::on_wait_for_result()` 检测 `{path}` 变化并重新发送 `follow_path` goal，使控制器在导航中接收新路径。

### Action 类型与查询入口

- 全局规划 action 名称：`compute_path_to_pose`；类型：`nav2_msgs/action/ComputePathToPose`；定义文件：`mobile_navigation/nav2_msgs/action/ComputePathToPose.action`。
- 局部控制 action 名称：`follow_path`；类型：`nav2_msgs/action/FollowPath`；定义文件：`mobile_navigation/nav2_msgs/action/FollowPath.action`。
- 运行中查询 action 列表：`ros2 action list`。
- 查询某个 action 的类型：`ros2 action type /compute_path_to_pose`、`ros2 action type /follow_path`。
- 查看 action 字段：`ros2 interface show nav2_msgs/action/ComputePathToPose`、`ros2 interface show nav2_msgs/action/FollowPath`。
- 从源码查询定义：`rg -n "ComputePathToPose|FollowPath|compute_path_to_pose|follow_path" mobile_navigation/nav2_msgs mobile_navigation/nav2_behavior_tree mobile_navigation/nav2_planner mobile_navigation/nav2_controller`。
- 全局规划 action 接收端：`mobile_navigation/nav2_planner/src/planner_server.cpp` 创建 `compute_path_to_pose` action server，`PlannerServer::computePlan()` 接收请求并调用 `PlannerServer::getPlan()`，最终调用 planner 插件接口 `nav2_core::GlobalPlanner::createPlan()` 计算路径。
- 局部控制 action 接收端：`mobile_navigation/nav2_controller/src/controller_server.cpp` 创建 `follow_path` action server，`ControllerServer::computeControl()` 接收请求，`setPlannerPath()` 调用控制器插件 `setPlan()`，`computeAndPublishVelocity()` 调用 `nav2_core::Controller::computeVelocityCommands()` 计算速度并发布 `/cmd_vel`。

## nav2_theta_star_planner 规划约束

- `nav2_theta_star_planner` 的 `ThetaStar::isSafe()` 仅将原始 costmap 代价为 `0` 的栅格视为自由通行。
- 当 `allow_unknown` 为 `true` 时，未知栅格仍可参与规划；膨胀层代价栅格不参与邻居扩展和 LOS 检查。
- 本约束用于避免 Theta* 生成穿越膨胀层的全局路径。

## package.xml 元数据同步

- 根目录 `package.xml` 作为仓库级元数据来源，当前版本号为 `0.4.0`。
- 各功能包 `package.xml` 已同步使用 `0.4.0` 版本号。
- 各功能包 maintainer 统一为 `dexforce <dexforce@dexforce.com>`。
- 各功能包保留原有 `description`，不强制改为根包的 CI 说明。
- Dexforce 自研包使用 `Proprietary` license。
- `mobile_navigation` 下的 Nav2 上游包保留原有 Apache、BSD、MIT、LGPL 等 license，不追加 `Proprietary`。

## 验证方式

本文档更新默认不执行构建。维护后可通过 `sed` 阅读新增章节、`rg` 检查路径和关键接口名称；修改 `package.xml` 时再使用 XML 解析和文本搜索验证包清单字段一致性。
