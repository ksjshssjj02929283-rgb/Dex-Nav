# mobile_perception

`mobile_perception` 是 `dexe_mobile_nav` 下的 ROS 2 感知与精确泊车控制包，主要用于 AprilTag 识别、相机到车体坐标转换、充电位姿保存、导航目标发布和底盘末端精控。

当前包使用 `ament_cmake`，同时包含 C++ 节点与 Python 脚本。C++ 节点是当前更清晰的主线实现，Python 脚本中保留了调试、仿真和早期编排逻辑。

## 目录结构

```text
mobile_perception/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── mobile_control.launch.py
│   └── mobile_control_action.launch.py
├── config/
│   ├── camera_config.json
│   ├── camera_config_1280.json
│   ├── camera_config_shenzhen.json
│   ├── camera_config_simulation.json
│   ├── tags_config.json
│   ├── docking_config.json
│   ├── docking_config_simulation.json
│   ├── mobile_control_params.yaml
│   ├── mobile_control_action_params.yaml
│   ├── docking.yaml
│   └── params.yaml
├── src/
│   ├── apriltag_detection_node.cpp
│   ├── apriltag_alignment_tf_node.cpp
│   ├── mobile_control_action_node.cpp
│   ├── mobile_control/
│   │   ├── straight_controller.cpp
│   │   └── rotation_controller.cpp
│   ├── obstacle_detector_node.cpp
│   └── charging_pose_service.cpp
├── include/
│   └── mobile_perception/
│       ├── mobile_control/
│       │   ├── pid_controller.hpp
│       │   ├── straight_controller.hpp
│       │   └── rotation_controller.hpp
│       └── obstacle_detector_node.hpp
├── test/
│   └── test_mobile_control_controllers.cpp
├── scripts/
│   ├── detect_apriltags*.py
│   ├── control_robot*.py
│   ├── nav_then_precise*.py
│   ├── nav_goal_*.py
│   ├── goal_controller*.py
│   └── 其他调试脚本
└── images/
```

## 核心节点

| 节点/脚本 | 类型 | 作用 |
| --- | --- | --- |
| `apriltag_detection_node` | C++ | 按服务启动/停止 AprilTag 检测，发布 `apriltag_poses`。 |
| `apriltag_alignment_tf_node` | C++ | 持续检测 AprilTag，发布调试姿态和原始/对齐后的 tag TF。 |
| `mobile_control_node` | C++（legacy，可选） | 仅在 `src/mobile_control_node.cpp` 存在时构建，订阅目标位姿并输出底盘速度。 |
| `mobile_control_action_node` | C++ | 通过 `control/precise_control` action 生成一次短 Path；直行跟踪 S 曲线 Trajectory，旋转使用直接角度反馈，并用速度感知 GoalChecker 判定完成。 |
| `obstacle_detector_node` | C++ | 订阅局部代价地图，检测车体前方矩形区域是否存在障碍物，并发布结果与 RViz Marker。 |
| `charging_pose_service_node` | C++ | 通过服务采样 `map -> base_link`，保存充电位姿 JSON。 |
| `detect_apriltags.py` / `detect_apriltags_service.py` | Python | Python 版 AprilTag 检测实现，依赖 `pupil_apriltags`。 |
| `control_robot.py` / `control_robot_simulation.py` | Python | 基于 `apriltag_poses` 的早期精控脚本。 |
| `nav_then_precise.py` / `nav_then_precise_bt.py` | Python | Nav2 导航后接 AprilTag 精控的编排脚本。 |
| `nav_goal_publisher.py` / `nav_goal_publisher_loop.py` | Python | 向 Nav2 `navigate_to_pose` action 发布目标。 |

## 主要话题与服务

### `mobile_control_node`

订阅：

- `control/ctrl_pose` (`geometry_msgs/msg/PoseStamped`)：控制目标位姿，可为非 `map` 坐标系，节点会尝试转换到 `map`。
- `feedback/odom` (`nav_msgs/msg/Odometry`)：底盘实际速度反馈，用于限加速度和平滑停止。

发布：

- `control/cmd_vel` (`geometry_msgs/msg/Twist`)：底盘控制速度。

服务：

- `control/get_control_status` (`std_srvs/srv/Trigger`)：返回 `IDLE`、`CONTROL`、`SUCCEED`、`CANCEL CONTROL` 或 `CANCELED`。
- `control/cancel_chassis_control` (`std_srvs/srv/Trigger`)：取消当前控制并按反馈速度平滑停车。

### `mobile_control_action_node`

Action：

- `control/precise_control` (`nav_interfaces/action/PreciseControl`)：接收 `geometry_msgs/msg/PoseStamped` 目标位姿，节点内部完成 TF 转换、精控、反馈和取消。

订阅：

- `feedback/odom` (`nav_msgs/msg/Odometry`)：底盘实际速度反馈，用于限加速度和平滑停止。

发布：

- `control/cmd_vel` (`geometry_msgs/msg/Twist`)：底盘控制速度。

运行约束：

- `mobile_control_action_node` 不订阅 `control/ctrl_pose`，也不调用 `mobile_control_node` 的状态或取消服务。
- 同一时间只接受一个 active goal；已有 goal 执行时会拒绝新 goal。
- 客户端 cancel 会触发平滑停车，停车完成后 action 返回 `CANCELLED`。
- 不要让 `mobile_control_node` 和 `mobile_control_action_node` 同时控制同一个 `control/cmd_vel` 链路，否则两个节点会竞争发布底盘速度。

控制模块边界：

- `SpeedProfilePlanner` 是无 Lifecycle/TF/costmap 的 pluginlib 接口；当前只注册
  `SCurveSpeedProfilePlanner`，由 Ruckig 分别在距离域和角度域为直行/倒车与原地旋转
  生成速度、加速度和 jerk 有界的 S 曲线。公共
  `planSpeedProfileWithTerminalSlowdown()` 在规划阶段先扣除末端 slow-down 距离/角度：
  主段从起点规划到 slow-down point，并以 slow-down speed 作为末速度；末端段从同一
  `(position, velocity, acceleration)` 连续状态规划到 goal 零速。Action 与曲线工具
  共用该函数，controller 不负责在 slow-down 边界截断速度。
- `ShortTrajectoryBuilder` 将接收 goal 时冻结的两点短 Path 与一维 profile 组合为
  `RobotTrajectory`。直行/倒车点携带纵向速度，原地旋转点携带角速度；两种模式都只在
  goal 接收时规划一次。
- `StraightController` 的纵向控制只保留两级基础 PID：外环
  `straight.position_pid` 根据当前 tracking point 在 `base_link` 下的带符号 x 误差
  生成速度修正，并与 S 曲线参考速度相加；active profile 内计划 `|ref_v|` 是目标速度
  不得突破的距离可行包络。内环 `straight.speed_pid` 根据目标速度与实测速度之差生成
  加速度修正，再与 S 曲线 `ref_a`（包括负的减速度）合成命令。命令不再按正/倒车限制输出
  符号，也不再维护 high/low/reverse PID、最小线速度、末段 recovery 或越点特例。
  加速度从上一周期命令积分，实测速度只进入速度误差；这样执行器尚未克服 deadband 时，
  PID 仍可自然累积有效命令，无需在 controller 中再加最小速度分支。
  当 profile 到达末点时 tracking point 就是 goal，越点会让 x 误差自然换符号，PID
  因而可反向修正纵向误差，最终 success 仍由 GoalChecker 的三轴、v/w 与 dwell 判定。
  `straight.stop_distance=0.05 m` 定义末端低速段：主 S 段只规划
  `max(0, total_distance - 0.05 m)`，并保证到 slow-down point 时速度已经连续降到
  `straight.stop_speed=0.03 m/s`；剩余 5 cm 使用第二段 S 曲线继续规划到 goal 零速。
  controller 内同值约束只作为异常反馈下的安全上限，不承担边界变速。
  `angular_z` 同时叠加当前 S 曲线 tracking point 在 `base_link` 下的横向误差
  `straight.lateral_pid` 与 goal yaw 误差 `straight.heading_pid`，倒车时只反转横向转向
  符号。直行规划前会检查 goal、当前机器人和路径期望车体朝向的三组角度差，任一超过
  `straight.max_initial_heading_error`（默认 `5°`）即 `ABORTED`，不进入 S 规划与控制。
- `RotationController` 是连续两级控制器：第一级将角度域 S 曲线的参考角速度与
  tracking yaw 外环修正相加，并用目标剩余角构造
  `sqrt(2 * max_alpha * abs(goal_yaw_error))` 制动速度包络，避免外环在减速段吃掉 S 曲线
  预留的停车距离；第二级用 `rotation.angular_velocity_pid` 跟踪“目标角速度 - 实测角速度”，
  PID 输出作为角加速度修正，与正常段 S 曲线角加速度前馈相加。制动包络生效时则直接按
  目标角速度与上一周期命令之差生成有界制动前馈。最终角速度命令限制在
  `±0.75 rad/s`，角加速度限制为 `±1.0 rad/s²`。
- `rotation.terminal_slowdown_angle` 定义末端低速段：主角速度 S 曲线在
  `total_angle - terminal_slowdown_angle` 处以有效 terminal speed 收尾，剩余角度从该
  连续状态继续规划到 goal 零速。RotationController 在末段只限制目标不得高于 terminal
  speed，不再把已经低于该值的规划参考重新抬高。最终 stop 范围由
  `goal_checker.yaw_tolerance` 定义，再由实测 v/w 和 dwell 判定成功。
- 目标机日志显示 `|cmd_w|≈0.005～0.016 rad/s` 时底盘可能不动；因此
  `rotation.min_angular_velocity=0.02 rad/s` 作为可调的 deadband 补偿下限。当前
  `terminal_angular_velocity=0.01 rad/s` 低于该下限，运行时会明确告警并使用可执行的
  `0.02 rad/s`；若底盘经标定可执行 `0.01 rad/s`，应把最小角速度同步下调到不高于
  terminal 值。StraightController 不再在控制核心内增加最小线速度特例；若目标机仍受
  线速度 deadband 影响，应先调外环 PID，再在底盘驱动/统一速度接口层处理 deadband。
- 旋转 S 曲线按实际有符号角进度加有界时间预瞄采样；Action 计算采样进度，
  RotationController 本身不维护 lag-recovery 或 `TERMINAL_YAW`。计划 `|ref_w|` 是 yaw
  外环不得突破的距离可行包络。曲线结束后参考角速度自然为零，tracking yaw 外环继续消除目标角残差，
  制动速度包络在越点后禁止目标角速度继续远离 goal；普通段、末段和越点纠偏仍共用同一个
  `computeCommand()`。Action 对旋转 goal error 做跨 `±pi` 连续展开，避免 180° goal
  在角度归一化分支点改变已经规划的旋转方向。
- 直行轨迹参考以实际空间进度为主，并在 `trajectory_tracking.preview_time` 范围内做有界时间预瞄；
  默认预瞄 `0.05 s`，用于跳过 S 曲线 `v=0、a=0` 的起始状态自锁，同时不会按墙钟时间
  无限制跑完整条轨迹。
- `PreciseGoalChecker` 按运动模式选择几何门：直行/倒车检查目标在 `base_link` 下的
  纵向、横向和 yaw；原地旋转只把 yaw 纳入成功判定，纵向/横向仅作为诊断量，不阻塞
  success。两种模式都继续检查实测线速度、实测角速度和连续停稳时间。当前阈值为纵向/横向
  `0.005 m`、yaw `0.008 rad`、`|v| <= 0.005 m/s`、`|w| <= 0.01 rad/s`、
  连续 `0.2 s`，全部可配置。
- 位姿先进入容差而速度尚未达标时，直行继续跟踪 S 曲线，旋转继续执行角度-角速度反馈；
  只有实测速度也进入静止阈值后才发布零速并开始连续停稳确认。
- `mobile_control_action_node` 只负责 Action、TF/odom freshness、一次规划、轨迹执行、
  fail-fast、取消和发布。TF/odom 中断或执行超时会先发布零速度再 `ABORTED`。
- goal 转成固定的 map 坐标后不再保留请求时间戳；每个控制周期用最新共同 TF 快照计算
  `map -> base_link` 相对误差，避免 goal 时间戳滑出 TF 缓存后出现 past extrapolation。
- 整条链路不启动 ControllerServer、不创建 costmap、不在线重规划，也不使用
  `SpeedPlanState`；因此仍适用于允许靠近障碍物的末端精控。

观测话题：

- `control/planned_path` (`nav_msgs/msg/Path`)：每个 accepted goal 生成并发布一次。
- `control/planned_trajectory` (`nav_interfaces/msg/RobotTrajectory`)：直行是一次性 S 曲线轨迹；
  旋转是一次性角度域 S 曲线轨迹。使用 reliable + transient-local QoS，便于后启动的调试工具查看。

阶段日志：

- `1/7 收到Goal / Goal已接受`：输入 frame、时间戳和目标位姿。
- `2/7 控制前角度检查 / 路径规划完成`：直行 goal/robot/path 三组角度差和阈值，以及
  固定 map 短路径、模式、起终点和运动量；超限时不进入 3/7。
- `3/7 S速度规划完成 / S角速度规划完成`：插件、轨迹点数、时长及速度/加速度/jerk 峰值；
  旋转同时输出两级控制器参数。
- `4/7 控制开始`：进入 Trajectory 跟踪。
- `5/7 控制中`：不再输出径向 `distance`，两种模式都输出 goal 纵向/横向误差；
  直行另输出当前有符号空间进度、S 参考、有效 target v、speed PID、
  `min_v_applied/recovery_applied` 和双闭环误差；旋转输出
  `phase=ANGLE_SPEED_CASCADE`、S 曲线 `ref_w/ref_alpha`、tracking yaw、外环目标角速度、
  角速度 PID 修正、实测角速度和 `cmd_w`。由 `logging.control_status_period` 节流，
  默认 `0.5 s`。
- `6/7 到点检查`：输出 `position_required`；直行检查纵向/横向/yaw，旋转只检查 yaw，
  两种模式都检查速度和连续停稳时间；
  success 前额外输出一条不节流的 `最终到点精度`。
- `7/7 接口返回`：`SUCCEEDED`、`CANCELED`、`ABORTED` 或 `REJECTED`；成功时输出
  `goal_longitudinal/goal_lateral/yaw/v/w`，不混入径向 distance。

同一次 Action 的所有阶段均携带由 goal UUID 派生的同一 `plan_id`，可直接按
`plan_id` 过滤整条执行链。

### `apriltag_detection_node`

服务：

- `start_apriltag_detection` (`std_srvs/srv/Trigger`)：创建图像订阅、TF listener 和 AprilTag detector。
- `stop_apriltag_detection` (`std_srvs/srv/Trigger`)：释放检测资源。

发布：

- `apriltag_poses` (`geometry_msgs/msg/PoseArray`)：检测到的 tag 在 `base_link` 下的位姿。
- 调试 TF `base_link -> apriltag_detection_debug_<id>`：与 `apriltag_poses` 中对应 pose 完全一致，用于在 RViz 中判断 raw tag 视觉测量是否抖动；不作为控制输入。

依赖 TF：

- `base_link -> camera_link`，其中 `camera_link` 来自 `config/camera_config.json`。

### `apriltag_alignment_tf_node`

订阅：

- 调用 `start_apriltag_detection` 后订阅图像；调用 `stop_apriltag_detection` 后取消订阅。
- 默认图像话题来自 `config/camera_config.json` 的 `cameras[0].topic_name`，可通过参数 `image_topic` 覆盖。

发布：

- 以下话题和 TF 仅在 `start_apriltag_detection` 启动检测后创建并发布，`stop_apriltag_detection` 后关闭发布。
- 默认 `apriltag_poses_debug` (`geometry_msgs/msg/PoseArray`)。
- 默认 TF 子坐标系 `apriltag_raw_<id>` 和 `apriltag_aligned_<id>`。
- 默认 `apriltag_target_pose` (`geometry_msgs/msg/PoseStamped`)：以 `apriltag_aligned_<id>` 为基准，沿其 X 轴前移 `target_forward_offset` 后发布到 `target_frame`，默认绑定在 `base_link` 下。
- 默认 TF 子坐标系 `apriltag_target_<id>`：与 `apriltag_target_pose` 使用同一个目标位姿，便于 RViz 可视化。

服务：

- `start_apriltag_detection` (`std_srvs/srv/Trigger`)：创建 detector、TF listener/broadcaster、图像订阅和发布器，开始检测。
- `stop_apriltag_detection` (`std_srvs/srv/Trigger`)：释放 detector、TF、图像订阅和发布器，停止检测以降低 CPU 占用。

运行日志：

- 检测启动后每 `5 s` 输出一次 `[TagStats]`，停止检测时输出累计 `[TagSummary]`。
- `image/detect/target/lost/last` 分别表示图像、检出、目标发布、连续丢失和最近目标间隔。
- `gray` 为最近帧灰度均值/标准差，`tf_err/cv_err` 为 TF 和图像转换错误数。
- 灰度数据只用于现场对比，不能单独证明环境过暗。
- 没有图像或当前统计窗口没有发布目标时使用 WARN 级别；正常识别时使用 INFO 级别。

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

### `obstacle_detector_node`

订阅：

- `local_costmap/costmap` (`nav_msgs/msg/OccupancyGrid`)：默认局部代价地图，可通过参数 `costmap_topic` 覆盖。

发布：

- `obstacle_detector/has_obstacle` (`std_msgs/msg/Bool`)：前方检测区域内是否存在代价值达到阈值的障碍物。
- `obstacle_detector/markers` (`visualization_msgs/msg/Marker`)：检测到的障碍物采样点。
- `obstacle_detector/detection_box` (`visualization_msgs/msg/Marker`)：当前检测矩形框，有障碍物时红色，无障碍物时绿色。

依赖 TF：

- `costmap.header.frame_id -> robot_base_frame`，默认 `robot_base_frame` 为 `base_link`。

常用参数：

- `costmap_topic`：默认 `local_costmap/costmap`。
- `robot_base_frame`：默认 `base_link`。
- `front_distance`：车体前方检测距离，默认 `1.0` 米。
- `left_width` / `right_width`：检测矩形左右宽度，默认各 `0.5` 米。
- `obstacle_threshold`：障碍物代价值阈值，默认 `50`。
- `detection_frequency`：检测频率，默认 `10.0` Hz；小于等于 0 时回退到 10 Hz。

## 配置文件

| 文件 | 说明 |
| --- | --- |
| `config/camera_config.json` | 当前默认相机配置，包含图像话题、`camera_link`、内参和畸变参数。 |
| `config/camera_config_simulation.json` | 仿真相机配置，默认图像话题为 `/camera/image_raw`。 |
| `config/camera_config_1280.json` / `camera_config_shenzhen.json` | 其他实机相机标定配置。 |
| `config/tags_config.json` | AprilTag family、tag 尺寸和检测约束。 |
| `config/mobile_control_params.yaml` | `mobile_control_node` 控制频率、速度限幅、增益、死区和收敛阈值。 |
| `config/mobile_control_action_params.yaml` | `mobile_control_action_node` 控制频率、速度限幅、增益、死区和收敛阈值。 |
| `config/docking_config.json` | 实机充电位姿和预充电位姿。 |
| `config/docking_config_simulation.json` | 仿真充电位姿。 |
| `config/params.yaml` | Nav2 目标发布脚本使用的示例目标。 |
| `config/docking.yaml` | 旧版/备用 docking 控制参数和目标点。 |

切换仿真与实机时，重点检查：

- `camera_config*.json` 中的 `topic_name`、`camera_link`、内参和畸变参数。
- `docking_config*.json` 中的目标位姿。
- Python 旧脚本中是否读取了固定文件名，例如 `docking_config.json` 或 `docking_config_simulation.json`。
- TF 树是否包含 `map`、`base_link` 和相机坐标系。

## 依赖

ROS 2 依赖主要来自 `CMakeLists.txt` 和 `package.xml`：

- `rclcpp`
- `rclpy`
- `cv_bridge`
- `OpenCV`
- `apriltag`
- `apriltag_ros`
- `tf2`
- `tf2_ros`
- `tf2_geometry_msgs`
- `geometry_msgs`
- `sensor_msgs`
- `nav_msgs`
- `std_srvs`
- `ament_index_cpp`
- `nlohmann_json`
- `yaml-cpp`

Python 脚本还可能依赖：

- `pupil-apriltags`
- `opencv-python`
- `numpy`
- `scipy`
- `transforms3d`
- `py_trees`
- `nav2_msgs`
- 仓库内自定义消息/服务包，例如 `custom_interfaces`、`chassis_interfaces`

示例安装命令：

```bash
python3 -m pip install pupil-apriltags opencv-python scipy transforms3d py_trees
sudo apt-get install ros-humble-control-toolbox ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
sudo apt-get install ros-humble-apriltag-ros ros-humble-rqt ros-humble-rqt-plot ros-humble-rqt-common-plugins
```

ARM 环境如遇到二进制兼容问题，历史记录中使用过：

```bash
python3 -m pip install numpy==1.26.0
```

国内网络环境可使用镜像源：

```bash
python3 -m pip install pupil-apriltags -i https://pypi.tuna.tsinghua.edu.cn/simple
python3 -m pip install opencv-python -i https://pypi.tuna.tsinghua.edu.cn/simple
```

## 运行示例

先 source 工作空间：

```bash
source install/setup.bash
```

启动底盘精控节点：

```bash
ros2 launch mobile_perception mobile_control.launch.py
```

使用自定义参数文件：

```bash
ros2 launch mobile_perception mobile_control.launch.py params_file:=/path/to/mobile_control_params.yaml
```

启动 Action 版底盘精控节点：

```bash
ros2 launch mobile_perception mobile_control_action.launch.py
```

使用自定义 Action 版参数文件：

```bash
ros2 launch mobile_perception mobile_control_action.launch.py params_file:=/path/to/mobile_control_action_params.yaml
```

发送 Action 精控目标：

```bash
ros2 action send_goal /control/precise_control nav_interfaces/action/PreciseControl \
  "{target_pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

启动 C++ AprilTag 检测节点，并通过服务开始检测：

```bash
ros2 run mobile_perception apriltag_detection_node
ros2 service call /start_apriltag_detection std_srvs/srv/Trigger {}
```

停止检测：

```bash
ros2 service call /stop_apriltag_detection std_srvs/srv/Trigger {}
```

启动 AprilTag 对齐 TF 调试节点：

```bash
ros2 run mobile_perception apriltag_alignment_tf_node
ros2 service call /start_apriltag_detection std_srvs/srv/Trigger {}
ros2 service call /stop_apriltag_detection std_srvs/srv/Trigger {}
```

启动前方障碍检测节点：

```bash
ros2 run mobile_perception obstacle_detector_node
```

示例覆盖检测范围和代价地图话题：

```bash
ros2 run mobile_perception obstacle_detector_node --ros-args \
  -p costmap_topic:=/local_costmap/costmap \
  -p front_distance:=1.0 \
  -p left_width:=0.5 \
  -p right_width:=0.5 \
  -p obstacle_threshold:=50
```

发布一个精控目标：

```bash
ros2 run mobile_perception ctrl_goal_publisher.py
```

查询精控状态：

```bash
ros2 service call /control/get_control_status std_srvs/srv/Trigger {}
```

取消精控：

```bash
ros2 service call /control/cancel_chassis_control std_srvs/srv/Trigger {}
```

保存当前充电位姿：

```bash
ros2 run mobile_perception charging_pose_service_node
ros2 service call /save_charging_pose std_srvs/srv/Trigger {}
```

注意：`charging_pose_service.cpp` 当前保存路径硬编码为 `/home/dexforce/w1/install/mobile_perception/lib/mobile_perception/charging_pose.json`，在其他机器或工作空间上使用前应先修改为参数化路径或确认该路径存在且可写。

## 调试命令

查看 TF 树：

```bash
ros2 run tf2_tools view_frames
```

手动发布速度，低速测试底盘链路：

```bash
ros2 topic pub /control/cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 1
ros2 topic pub /control/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 1
```

执行脱桩动作：

```bash
ros2 run mobile_perception undocking.py --timeout 1 --direction 1 --speed 0.015
```

生成精控生产 S 曲线图（需要以 `BUILD_TESTING=ON` 构建，默认输出到
`~/.dexe/navigation/mobile_control_speed_profiles`）：

```bash
python3 mobile_perception/test/plot_mobile_control_speed_profiles.py \
  --planner-executable build/mobile_perception/mobile_control_speed_profile_dump \
  --params-file mobile_perception/config/mobile_control_action_params.yaml
```

Python 只读取参数、调用 C++ 工具和绘图，不复制任何速度规划或 slowdown 公式。C++ 工具
逐点复用运行时同一套 `planSpeedProfileWithTerminalSlowdown()`、
`SCurveSpeedProfilePlanner`、`sampleSpeedProfileForTracking()`、
`StraightController::computeCommand()` 与 `RotationController::computeCommand()`。每张图
同时显示组合 S profile 原始速度、运行时采样速度和 controller target，并标出 slow-down
point 之后的末端规划段：

- 直线读取 `straight.stop_distance/stop_speed`；
- 旋转读取 `rotation.terminal_slowdown_angle/terminal_angular_velocity`，并显示受
  `rotation.min_angular_velocity` 约束后的实际有效 slowdown speed；
- 所有默认值直接来自指定的 `mobile_control_action_params.yaml`，命令行参数只作为显式
  override。

默认生成平移 `0.1～1.0m` 每 `0.1m`、旋转 `0～180°` 每 `30°` 共 17 组 PNG/CSV 和
`summary.csv`。图中的 `runtime controller target` 是生产 controller 的目标速度；
最终 `cmd_v/cmd_w` 仍取决于实测速度、PID 状态与底盘响应，不能由离线曲线代替实机记录。

## 控制策略简述

`mobile_control_node` 会把目标转换到 `base_link` 下，根据目标相对车体的位置和角度计算速度：

- 距离未收敛时使用 `rho-alpha-beta` 类控制律。
- 目标在车体后方且横向误差较小时进入倒车模式。
- 接近目标时降低距离增益，并解耦部分最终姿态修正。
- 位置收敛后单独旋转对齐航向。
- 输出速度经过加速度限制和低通滤波。
- 到达后持续发布零速若干周期，状态服务先返回一次 `SUCCEED`，之后回到 `IDLE`。

## 已知注意事项

- `package.xml` 中依赖声明少于 `CMakeLists.txt` 实际使用的依赖，新增节点或清理包依赖时需要同步维护。
- `CMakeLists.txt` 安装的 Python 脚本不是 `scripts/` 下全部文件，例如 `control_robot_1.py`、`control_robot_pre.py`、`pid_controller.py`、`get_control_status.py` 等未全部安装。
- 部分 Python 脚本仍引用旧包名、旧话题或自定义接口，使用前应先检查当前系统中的话题、服务、TF 和接口包是否匹配。
- `camera_config.json` 是 C++ AprilTag 节点默认读取的文件；如果要运行仿真配置，建议改为参数化或明确替换配置文件。
- 精控和泊车相关脚本会发布底盘速度，实机运行前必须确认急停、障碍物状态和速度限幅。
