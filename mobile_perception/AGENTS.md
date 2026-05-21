# mobile_perception 维护说明

本文档面向在 `dexe_mobile_nav/mobile_perception` 内工作的自动化编码代理和开发者。所有说明以当前包结构为准。

## 默认沟通语言

- 本目录内新增或更新面向维护者的说明文档时，默认使用中文。
- 命令、路径、节点名、话题名、服务名、参数名和代码标识符可保留英文原文。
- 除非用户明确要求英文，不要把本目录文档改写成英文。

## 包定位

- 这是 ROS 2 Humble 风格的 `ament_cmake` 包，包名为 `mobile_perception`。
- 包内同时包含 C++ 节点和 Python 脚本。
- 当前主线 C++ 节点位于 `src/`：
  - `apriltag_detection_node.cpp`
  - `apriltag_alignment_tf_node.cpp`
  - `mobile_control_node.cpp`
  - `mobile_control_action_node.cpp`
  - `obstacle_detector_node.cpp`
  - `charging_pose_service.cpp`
- `include/mobile_perception/` 当前存放 `obstacle_detector_node.hpp`，新增 C++ 节点头文件时保持包名命名空间一致。
- Python 脚本位于 `scripts/`，包含历史调试脚本、仿真脚本和 Nav2 编排脚本。修改前应确认脚本是否已被 `CMakeLists.txt` 安装。

## 工作边界

- 修改本包时，优先保持已有 ROS 2 接口兼容，尤其是话题、服务、参数和 frame 名称。
- 不要无关重命名节点、话题、服务或配置文件。
- 不要把实机配置与仿真配置混在一起；相机、tag、docking 位姿相关改动必须说明适用场景。
- 不要删除看似旧的 Python 脚本，除非用户明确要求清理；很多脚本可能仍用于现场调试。
- 不要主动执行 `colcon build`、`catkin build`、`cmake --build`、`make` 或其他构建命令。需要构建验证时，先询问用户。

## 重要文件

- `CMakeLists.txt`：声明 C++ 可执行文件、Python 脚本安装列表、`launch/config/images` 安装目录。
- `package.xml`：ROS 包元信息和部分依赖声明。新增依赖时要同时检查 `CMakeLists.txt`。
- `launch/mobile_control.launch.py`：启动 `mobile_control_node`，默认加载 `config/mobile_control_params.yaml`。
- `config/mobile_control_params.yaml`：精控节点控制参数。
- `config/camera_config.json`：C++ AprilTag 节点默认读取的实机相机配置。
- `config/tags_config.json`：AprilTag family 和 tag 尺寸。
- `config/docking_config.json`：实机充电位姿配置。
- `config/*simulation*`：仿真配置，不要误用于实机。

## 代码风格

- C++ 代码继续使用 ROS 2 `rclcpp`、`tf2`、`cv_bridge`、OpenCV 和 `nlohmann_json` 的现有写法。
- Python 节点继续使用 `rclpy`，脚本入口保持 `if __name__ == '__main__':`。
- 配置优先走 YAML/JSON 或 ROS 参数；新增硬编码路径前必须有充分理由。
- 对底盘速度、TF 转换、位姿保存等高风险逻辑，改动要尽量小，并保留清晰日志。
- 注释可以使用中文，但应解释意图或风险，不写重复代码含义的空注释。

## 接口约束

### `mobile_control_node`

保持以下接口兼容：

- 订阅 `control/ctrl_pose` (`geometry_msgs/msg/PoseStamped`)
- 订阅 `feedback/odom` (`nav_msgs/msg/Odometry`)
- 发布 `control/cmd_vel` (`geometry_msgs/msg/Twist`)
- 服务 `control/get_control_status` (`std_srvs/srv/Trigger`)
- 服务 `control/cancel_chassis_control` (`std_srvs/srv/Trigger`)

状态服务返回文本当前包括：

- `IDLE`
- `CONTROL`
- `SUCCEED`
- `CANCEL CONTROL`
- `CANCELED`

修改这些字符串前，需要确认上游调用方没有依赖精确文本。

### `mobile_control_action_node`

保持以下接口语义：

- Action `control/precise_control` (`nav_interfaces/action/PreciseControl`)
- 订阅 `feedback/odom` (`nav_msgs/msg/Odometry`)
- 发布 `control/cmd_vel` (`geometry_msgs/msg/Twist`)

该节点是独立 Action 版精控入口，不订阅 `control/ctrl_pose`，不调用 `mobile_control_node` 的状态或取消服务。修改时应保持同一时间只允许一个 active goal，客户端取消应进入平滑停车并在停车完成后返回 `CANCELLED`。

不要让 `mobile_control_node` 和 `mobile_control_action_node` 在同一底盘速度链路上同时运行；二者都会发布 `control/cmd_vel`。

### AprilTag 节点

`apriltag_detection_node` 需要保持：

- 服务 `start_apriltag_detection`
- 服务 `stop_apriltag_detection`
- 发布 `apriltag_poses`
- 发布调试 TF `base_link -> apriltag_detection_debug_<id>`，该 TF 与 `apriltag_poses` 中对应 pose 完全一致，只用于诊断 raw tag 视觉测量抖动，不应被控制逻辑依赖。
- 使用 `base_link` 作为输出位姿 frame

`apriltag_alignment_tf_node` 默认发布：

- `apriltag_poses_debug`
- `apriltag_raw_<id>`
- `apriltag_aligned_<id>`
- `apriltag_target_pose`
- `apriltag_target_<id>`

该节点与 `apriltag_detection_node` 使用一致的启停服务：

- `start_apriltag_detection`：创建 detector、TF listener/broadcaster、图像订阅和发布器后开始检测与发布。
- `stop_apriltag_detection`：释放 detector、TF、图像订阅和发布器，停止检测与发布以降低 CPU 占用。

修改 `camera_link`、图像话题或 tag 尺寸时，优先通过参数或配置文件完成。
目标点发布逻辑以 `apriltag_aligned_<id>` 的姿态为基准，沿其 X 轴偏移 `target_forward_offset`，默认发布在 `base_link` 下，并可通过 `publish_target_tf` 发布目标 TF。修改这段逻辑时要保持参数可配置，不要写死 frame 名或距离。

### `obstacle_detector_node`

保持以下接口兼容：

- 订阅 `local_costmap/costmap` (`nav_msgs/msg/OccupancyGrid`)，可通过参数 `costmap_topic` 覆盖。
- 发布 `obstacle_detector/has_obstacle` (`std_msgs/msg/Bool`)。
- 发布 `obstacle_detector/markers` (`visualization_msgs/msg/Marker`)。
- 发布 `obstacle_detector/detection_box` (`visualization_msgs/msg/Marker`)。

常用参数包括 `robot_base_frame`、`front_distance`、`left_width`、`right_width`、`obstacle_threshold` 和 `detection_frequency`。修改检测区域或采样策略时，优先保持参数可配置，并注意局部代价地图分辨率对 CPU 占用和漏检风险的影响。

## 配置修改规则

- 改相机内参时，必须同时检查 `frame_width`、`frame_height`、`fx`、`fy`、`cx`、`cy` 和 `dist_coeffs`。
- 改 tag 尺寸时，只改 `config/tags_config.json` 中的 `tag_size`，除非代码已明确支持其他来源。
- 改控制参数时，优先修改 `config/mobile_control_params.yaml`，不要直接把实验值写死进 `mobile_control_node.cpp`。
- 改 docking 位姿时，明确区分：
  - `docking_config.json`：实机。
  - `docking_config_simulation.json`：仿真。
- 保存位姿相关代码当前存在硬编码路径，修改时优先改为 ROS 参数或包共享目录下可配置路径。

## 验证建议

不主动构建。若用户同意验证，可按风险从低到高选择：

```bash
python3 -m py_compile dexe_mobile_nav/mobile_perception/scripts/*.py
```

```bash
colcon build --packages-select mobile_perception
```

运行前检查：

```bash
ros2 topic list
ros2 service list
ros2 run tf2_tools view_frames
```

实机控制前必须确认：

- 急停可用。
- 底盘速度链路接收的是预期话题。
- `control/cmd_vel` 与底盘实际订阅话题已正确桥接或 remap。
- `map`、`base_link`、相机 frame 的 TF 连通。
- 相机图像话题与 `camera_config.json` 一致。

## 常见风险

- `package.xml` 依赖声明不完整，新增构建依赖时容易只改 `CMakeLists.txt`。
- 部分 Python 脚本引用旧包名或旧接口，例如自定义 `custom_interfaces`、`chassis_interfaces`。
- 部分脚本发布 `cmd_vel`，部分发布 `control/cmd_vel` 或 `control/cmd_vel_nav`，使用前必须确认实际底盘订阅链路。
- `charging_pose_service.cpp` 的保存路径是硬编码绝对路径，跨机器运行容易失败。
- C++ AprilTag 节点默认读取 `camera_config.json`，不会自动切换到仿真配置。

## 文档维护

- README 面向使用者，说明怎么配置、运行和排查。
- AGENTS 面向维护者和自动化代理，说明约束、风险和代码边界。
- 更新运行入口、话题、服务、参数或配置文件时，应同步更新 `README.md` 和本文件。
