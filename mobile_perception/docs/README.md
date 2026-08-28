# mobile_perception 文档

本目录记录 `mobile_perception` 包的补充维护信息。包级使用说明仍以 `../README.md` 为准。

## mobile_control_node

`control/get_control_status` 服务中的取消状态分为两段：

- `CANCEL CONTROL`：已收到取消请求，节点正在按当前里程计反馈速度平滑停车。
- `CANCELED`：取消停车已完成；该状态与 `SUCCEED` 一样用于完成态反馈，查询后可回到空闲状态。

## mobile_control_action_node

`mobile_control_action_node` 是 Action 版底盘末端精控节点，接口为
`control/precise_control` (`nav_interfaces/action/PreciseControl`)。

Action goal 中的 `target_pose` 使用 `geometry_msgs/msg/PoseStamped`。节点接收到 goal 后在内部执行
`mobile_control_node` 同等的 TF 转换、PID 控制、速度滤波、到点判定和平滑取消逻辑，并直接发布
`control/cmd_vel`。

主要接口：

- Action `control/precise_control` (`nav_interfaces/action/PreciseControl`)。
- 按任务订阅 `feedback/odom` (`nav_msgs/msg/Odometry`)。
- 发布 `control/cmd_vel` (`geometry_msgs/msg/Twist`)。

运行注意：

- 不订阅 `control/ctrl_pose`。
- 同一时间只接受一个 active goal。
- 默认配置按任务创建 TF listener 和 Odom 订阅；空闲时释放，降低加密通信域中的常驻 CPU 开销。
- 接收 goal 后先进入 `WARMING`，只有 TF 和本次任务首帧 Odom 均就绪才进入控制；等待超时会保持零速度并中止目标。
- 客户端 cancel 后会平滑停车，完成后返回 `CANCELLED`。
- 不应与 `mobile_control_node` 同时控制同一个 `control/cmd_vel` 链路。

默认启动入口：

```bash
ros2 launch mobile_perception mobile_control_action.launch.py
```

默认参数文件：

- `config/mobile_control_action_params.yaml`

## 接口辅助节点的故障隔离

- `pub_current_position.py` 使用 10 Hz ROS Timer 发布 `feedback/chassis_pose`，不再使用
  `spin_once + sleep` 手写循环。每次先以零超时检查 `map <- base_link` TF，再执行零超时
  查询；TF 长期缺失时查询周期从 0.1 秒逐级退避到 1 秒，恢复后自动回到 10 Hz。
- `nav_goal_server.py` 对 `navigate_to_pose` 的首次等待上限为 10 秒。服务器未就绪时节点
  继续处理回调并每 2 秒非阻塞重试，只保留最新待发目标。旧目标取消完成前不会发送
  新目标，过期的反馈、取消和结果回调通过目标代次编号丢弃。目标响应等待上限为 10 秒，
  取消完成等待上限为 15 秒；服务器已退出时复位旧状态并重连，端点仍存在但响应不确定时
  不重复发送旧目标，取消阶段会重试取消请求。

## 当前二维码检测与 Action 精控链路

当前应用的二维码检测节点使用
`mobile_perception/src/apriltag_alignment_tf_node.cpp`，控制节点使用
`mobile_control_action_node` 的 Action 接口。推荐链路为：

1. 启动 `apriltag_alignment_tf_node`。
2. 调用 `start_apriltag_detection` 开始检测二维码。
3. 节点检测到 AprilTag 后发布 `apriltag_target_pose`。
4. 上游编排节点或客户端读取目标位姿，并作为 `target_pose` 发送到
   `control/precise_control`。
5. `mobile_control_action_node` 将目标转换到配置的控制参考坐标系（默认 `odom`），
   再在控制循环中转换回 `base_link` 计算误差并发布 `control/cmd_vel`。

### 二维码检测节点

`apriltag_alignment_tf_node` 默认从 `config/camera_config.json` 读取第一路相机配置，
从 `config/tags_config.json` 读取 tag 尺寸。节点启动后只创建启停服务；真正的图像订阅、
detector、TF listener/broadcaster 和发布器会在调用 `start_apriltag_detection` 后创建。
调用 `stop_apriltag_detection` 后会释放这些资源，以降低空闲时 CPU 占用。

主要接口：

- 服务 `start_apriltag_detection` (`std_srvs/srv/Trigger`)。
- 服务 `stop_apriltag_detection` (`std_srvs/srv/Trigger`)。
- 订阅图像话题：默认来自 `camera_config.json` 的 `cameras[0].topic_name`，可通过
  `image_topic` 参数覆盖。
- 发布 `apriltag_poses_debug` (`geometry_msgs/msg/PoseArray`)：检测到的 raw tag 位姿，
  frame 为 `base_link`。
- 发布 `apriltag_target_pose` (`geometry_msgs/msg/PoseStamped`)：用于后续精控的目标位姿。
- 可选发布 TF：`apriltag_raw_<id>`、`apriltag_aligned_<id>`、`apriltag_target_<id>`。

坐标和目标点逻辑：

- 节点依赖 `base_link -> camera_link` TF，`camera_link` 默认来自相机配置，也可通过参数覆盖。
- raw tag 位姿先由相机坐标转换到 `base_link`。
- aligned tag 姿态会移除固定 tag-frame 姿态偏置，使居中的充电二维码与 `base_link` 姿态对齐。
- `apriltag_target_pose` 以 `apriltag_aligned_<id>` 姿态为基准，沿其 X 轴偏移
  `target_forward_offset`，默认值为 `0.67` 米。
- `target_frame` 默认是 `base_link`；如果改成其他 frame，节点会通过 TF 将目标位姿转换到
  指定 frame 后发布。

常用参数：

- `image_topic`
- `camera_link`
- `tag_size`
- `pose_topic`
- `publish_tag_tf`
- `publish_aligned_tf`
- `raw_frame_prefix`
- `aligned_frame_prefix`
- `publish_target_pose`
- `publish_target_tf`
- `target_pose_topic`
- `target_frame`
- `target_frame_prefix`
- `target_forward_offset`

运行示例：

```bash
ros2 run mobile_perception apriltag_alignment_tf_node
ros2 service call /start_apriltag_detection std_srvs/srv/Trigger {}
```

停止检测：

```bash
ros2 service call /stop_apriltag_detection std_srvs/srv/Trigger {}
```

### Action 精控节点

`mobile_control_action_node` 提供 `control/precise_control`
(`nav_interfaces/action/PreciseControl`)。Action goal 的 `target_pose` 是
`geometry_msgs/msg/PoseStamped`，可直接使用 `apriltag_target_pose` 中的位姿，前提是
TF 树能把该目标从其 `header.frame_id` 转换到 `control_reference_frame`。

控制行为：

- 收到 goal 后按需创建 TF listener 和 Odom 订阅，进入 `WARMING`；TF 和首帧 Odom
  就绪后才将目标转换到 `control_reference_frame` 并锁存。该参考系默认是连续性更好的
  `odom`；等待超过 `input_ready_timeout_s` 会保持零速度并 abort 该 goal。按任务创建的
  TF Buffer 不包含接单前的历史数据，因此懒加载模式使用资源就绪时的最新 TF 锁存目标。
- 控制循环中持续查询 `base_link <- control_reference_frame` TF，把锁存目标转回
  `base_link` 计算位置和角度误差。
- 根据目标输入只判断一次 `STRAIGHT` 或 `ROTATE` 模式，任务期间不会切换模式。
- `ROTATE` 模式始终保持线速度为零；参考坐标系中的小幅位置漂移不再触发直线控制，
  原地转向到位只检查角度误差、制动状态和 Odom 实测角速度。
- `ROTATE` 模式会保存输入角度的正负号，并连续展开每周期角误差；精确 `+180°`
  锁定为逆时针，精确 `-180°` 锁定为顺时针，不再由 `±π` 边界抖动选择方向。
- 原地转向支持 `pure_rotation.velocity_curve` 选择 `step`（阶跃）或 `sine`
  （正弦平滑）速度曲线。正弦模式只使用单向的起步和收尾包络，不使用会周期换向的完整正弦波。
- 两种原地转向曲线都会根据剩余角度和
  `rotate_to_heading_max_angular_decel` 计算制动速度上限；进入停止区后先减速至零，
  停稳后误差仍超限才重新修正。只有角度到位且
  Odom 实测角速度低于 `rotate_stop_angular_vel` 后，才进入停稳确认。
- 直线控制支持 `straight.velocity_curve` 选择 `step` 或 `sine`。两种模式都会根据
  剩余距离、`decel_x_max` 和 `straight.brake_safety_factor` 提前限制速度；
  `sine` 还会在 `straight.sine_slowdown_distance` 范围内平滑收尾。
- 目标容差外的直线速度不会低于 `min_v`，用于避免命令落入底盘低速死区后长时间
  爬行；进入位置容差后仍按 `decel_x_max` 平滑降到零。
- 进入位置容差后不再直接清零线速度，而是按 `decel_x_max` 继续减速；位置到位且
  Odom 实测线速度低于 `straight.stop_linear_vel` 后，才进入停稳确认。
- 发布 `control/cmd_vel`，并订阅 `feedback/odom` 用于限加速度和平滑停车。
- feedback 的 `current_status` 包括 `WAITING_FOR_INPUTS`、`IDLE`、`CONTROL`、
  `CANCELLING`、`SUCCEEDED`。
- result 成功时 `success=true`、`message=SUCCEEDED`；取消完成时
  `success=false`、`message=CANCELLED`。

运行示例：

```bash
ros2 launch mobile_perception mobile_control_action.launch.py
```

发送示例 goal：

```bash
ros2 action send_goal /control/precise_control nav_interfaces/action/PreciseControl \
  "{target_pose: {header: {frame_id: base_link}, pose: {position: {x: 0.67, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

实机注意：

- 不要同时运行 `mobile_control_node` 和 `mobile_control_action_node` 控制同一个
  `control/cmd_vel` 链路。
- `lazy_runtime_resources=true` 是 CE 配置的低 CPU 模式；如果现场发现动态 TF/Odom
  端点无法在规定时间内就绪，可先改为 `false` 回退到原有常驻模式，再继续排查。
- 使用二维码目标控制前，确认目标frame、`control_reference_frame`、`base_link`
  和 `camera_link` TF 连通；默认至少需要 `odom -> base_link` 可用。
- `apriltag_target_pose` 默认发布在 `base_link`，如果上游需要跨时间使用目标，建议客户端在接收后尽快发送 Action goal，或将 `target_frame` 配置为稳定的全局 frame。
- 使用 Action cancel 时，节点不会立即断速度，而是按当前 `feedback/odom` 速度平滑停车后返回
  `CANCELLED`。

## obstacle_detector_node

`obstacle_detector_node` 是 C++ 前方障碍检测节点，来自旧工作区
`/home/lovebc/dexforce_ws/dexe_mobile_nav/mobile_perception` 的实现，并已接入当前分支的
`mobile_perception` 包。

节点订阅局部代价地图 `local_costmap/costmap`，根据 `base_link` 在代价地图坐标系下的最新 TF，
检查车体前方矩形区域是否存在代价值达到阈值的栅格。

主要接口：

- 发布 `obstacle_detector/has_obstacle` (`std_msgs/msg/Bool`)。
- 发布 `obstacle_detector/markers` (`visualization_msgs/msg/Marker`)。
- 发布 `obstacle_detector/detection_box` (`visualization_msgs/msg/Marker`)。

常用参数：

- `costmap_topic`
- `robot_base_frame`
- `front_distance`
- `left_width`
- `right_width`
- `obstacle_threshold`
- `detection_frequency`

运行示例：

```bash
ros2 run mobile_perception obstacle_detector_node
```
