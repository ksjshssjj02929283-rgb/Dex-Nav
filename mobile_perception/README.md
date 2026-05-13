# mobile_perception

`mobile_perception` 是 `dexe_mobile_nav` 下的 ROS 2 感知与精确泊车控制包，主要用于 AprilTag 识别、相机到车体坐标转换、充电位姿保存、导航目标发布和底盘末端精控。

当前包使用 `ament_cmake`，同时包含 C++ 节点与 Python 脚本。C++ 节点是当前更清晰的主线实现，Python 脚本中保留了调试、仿真和早期编排逻辑。

## 目录结构

```text
mobile_perception/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── mobile_control.launch.py
├── config/
│   ├── camera_config.json
│   ├── camera_config_1280.json
│   ├── camera_config_shenzhen.json
│   ├── camera_config_simulation.json
│   ├── tags_config.json
│   ├── docking_config.json
│   ├── docking_config_simulation.json
│   ├── mobile_control_params.yaml
│   ├── docking.yaml
│   └── params.yaml
├── src/
│   ├── apriltag_detection_node.cpp
│   ├── apriltag_alignment_tf_node.cpp
│   ├── mobile_control_node.cpp
│   ├── obstacle_detector_node.cpp
│   └── charging_pose_service.cpp
├── include/
│   └── mobile_perception/
│       └── obstacle_detector_node.hpp
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
| `mobile_control_node` | C++ | 订阅目标位姿并输出底盘速度，实现末端精确控制与平滑停止。 |
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

- `control/get_control_status` (`std_srvs/srv/Trigger`)：返回 `IDLE`、`CONTROL`、`SUCCEED` 或 `CANCEL CONTROL`。
- `control/cancel_chassis_control` (`std_srvs/srv/Trigger`)：取消当前控制并按反馈速度平滑停车。

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
- `nav2_util`
- `nav2_graceful_controller`

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
