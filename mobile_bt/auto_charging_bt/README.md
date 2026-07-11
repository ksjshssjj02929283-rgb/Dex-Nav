# auto_charging_bt

`auto_charging_bt` 是一个 ROS 2 自动回充行为树包，负责把机器人导航到充电区域，再基于 AprilTag 位姿反馈完成近距离对接，最后通过电源板诊断信息确认是否进入充电状态。

## 功能概览

回充流程由 `config/charging_bt.xml` 描述：

1. 入口先连续采样 `/diagnostics_agg`，确认当前是“已充电”“已连接未充电”还是“未连接未充电”。
2. 已经处于充电状态时直接返回成功，不再导航或对接。
3. 已连接但未充电时先执行 `Undocking` 退桩，再进入正常回充流程。
4. 未连接未充电时先通过 Nav2 `NavigateToPose` 导航到充电位，导航最多重试 3 次。
5. 导航成功后，对接流程最多重试 3 次；每次尝试先调用 `start_apriltag_detection`。
6. `Dock` 订阅 `apriltag_target_pose`，用 `ReverseDockingController` 三阶段锁定目标并发布速度到 `control/cmd_vel_nav`。
7. 倒车对接到位后执行低速后退，检测到充电器连接或 `battery_charging=true` 任一条件满足就立即停止；否则达到距离/时间限制后停止。
8. `CheckCharging` 从 `/diagnostics_agg` 判断 `battery_charging` 是否连续稳定为 true；确认成功后默认以 `0.01 m/s` 向前发布 `0.5 s`，减小机器人继续顶住充电桩的机械压力，随后发布零速度；确认失败时停止检测、退桩并触发下一次对接重试。

## 目录结构

```text
auto_charging_bt/
├── CMakeLists.txt
├── package.xml
├── include/
│   ├── auto_charging_bt/auto_charging_bt.hpp
│   ├── auto_charging_bt/reverse_docking_bt_controller.hpp
│   ├── docking_controller.h
│   └── reverse_docking_controller.h
├── src/
│   ├── auto_charging_bt.cpp
│   ├── docking_controller.cpp
│   ├── reverse_docking_controller.cpp
│   ├── reverse_docking_bt_controller.cpp
│   ├── auto_charging_action_server_node.cpp
│   └── service_node.cpp
├── config/
│   ├── auto_charging_bt_params.yaml
│   ├── charging_bt.xml
│   └── charging_pose.json
├── launch/
│   ├── auto_charging_action.launch.py
│   ├── auto_charging_service.launch.py
│   └── auto_charging_bt.launch.py
└── test/
    ├── test_docking_controller.cpp
    ├── test_odom_docking_controller.cpp
    ├── test_reverse_docking_controller.cpp
    └── auto_charging_test.py
```

## 主要节点

### `auto_charging_action_server_node`

推荐入口。该节点提供 `/auto_charging` Action，Action 类型为 `chassis_interfaces/action/AutoCharging`。

行为：

- 接收 `force` 字段。
- 同一时间只允许一个回充任务执行。
- 通过 feedback 返回当前行为树状态和重试次数。
- 成功、失败或取消时会尝试停止 AprilTag 检测并发布零速度。
- 行为树 XML 从包安装后的 `share/auto_charging_bt/config/charging_bt.xml` 加载。
- `auto_charging_action.launch.py` 会加载 `share/auto_charging_bt/config/auto_charging_bt_params.yaml` 中的入口状态确认、末端后退、退桩、充电确认和 `Dock` 倒车对接参数。

启动示例：

```bash
ros2 launch auto_charging_bt auto_charging_action.launch.py
```

### `auto_charging_service_node`

该节点提供 `/auto_charging` Trigger 服务。

启动示例：

```bash
ros2 launch auto_charging_bt auto_charging_service.launch.py
```

注意：`service_node.cpp` 已使用包安装路径加载行为树 XML，并设置行为树所需的 `charging_node` blackboard；`auto_charging_service.launch.py` 会加载同一份 `auto_charging_bt_params.yaml`，并启动发布 `apriltag_target_pose` 的 `apriltag_alignment_tf_node`。导航目标 JSON 仍使用现场硬编码路径，使用前需要确认该文件存在。

### `test_docking_controller`

独立对接控制器测试节点，会读取本包 `config/charging_pose.json`，订阅 `apriltag_poses`，发布速度到 `control/cmd_vel_nav`。

```bash
ros2 run auto_charging_bt test_docking_controller
```

### `test_odom_docking_controller`

`base_link` 坐标系下的独立控制器测试节点。该节点订阅 `base_link` 下的目标位姿 `geometry_msgs/msg/PoseStamped`，每收到一帧就实时更新控制目标；当前机器人位姿固定为 `base_link` 原点，然后调用 `DockingController::getVelocity()` 发布速度。

默认接口：

- 订阅：`apriltag_target_pose` (`geometry_msgs/msg/PoseStamped`)，`header.frame_id` 应为 `base_link`。
- 发布：`control/cmd_vel_nav` (`geometry_msgs/msg/Twist`)。
- TF：持续发布当前控制目标 `base_link -> base_link_docking_target`。

启动示例：

```bash
ros2 run auto_charging_bt test_odom_docking_controller
```

发布一个 `base_link` 下的测试目标：

```bash
ros2 topic pub --once /apriltag_target_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: base_link}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

如果底盘使用 `base_footprint`，可以通过参数切换：

```bash
ros2 run auto_charging_bt test_odom_docking_controller --ros-args -p base_frame:=base_footprint
```

该测试节点默认控制参数比包内控制器默认值更激进，便于 `base_link` 下验证：

- `kP=0.16`
- `kA=0.28`
- `kB=-0.02`
- `max_linear_speed=0.25`
- `max_angular_speed=0.60`

如需现场调参：

```bash
ros2 run auto_charging_bt test_odom_docking_controller --ros-args -p kP:=0.20 -p kA:=0.35 -p max_linear_speed:=0.30 -p max_angular_speed:=0.80
```

### `test_reverse_docking_controller`

倒车回充控制器独立测试节点。该节点订阅 `base_link` 下的目标位姿 `geometry_msgs/msg/PoseStamped`，采用三阶段控制：启动后先保持零速度，采样 `initial_sampling_duration` 秒内收到的目标位姿并计算去极值平均值，然后将该平均目标从 `base_link` 转到固定坐标系 `fixed_frame`（默认 `odom`）锁定；第一阶段粗靠近后停车，按 `second_sampling_duration` 重新采样近距离目标；第二阶段到位后再次停车，按 `final_sampling_duration` 采样最终目标并进入最终精对准。锁定后不实时追随视觉目标，但控制循环会用最新 TF 把固定目标转回当前 `base_link` 计算误差，因此误差会随机器人运动变化。

`ReverseDockingController` 与原 `DockingController` 分离，面向“Nav2 已到充电前置点，机器人姿态基本满足直接后退充电”的场景。控制器默认连续低速倒车，同时根据角度误差修正方向；只有角度误差超过较大阈值时才临时抑制线速度，优先修正姿态。

内部阶段参考 `lat_lon_controller` 的末端控制思想：

- `APPROACHING`：直接倒车靠近目标，线速度比例 `1.0`。
- `ADJUST_HEADING`：接近目标后降低倒车速度，优先收敛倒车方向误差，线速度比例 `0.75`。
- `ADJUST_DISTANCE`：方向基本收敛后继续修正纵向距离和横向误差，线速度比例 `0.75`。
- `ROTATING_TO_GOAL`：距离很近后低速或近似原地修正最终姿态，线速度比例 `0.25`；该阶段使用最终姿态 PID，并且不再用近距离下容易跳变的倒车方向误差覆盖角速度。
- `FINISH`：位置和姿态都满足阈值，输出零速度。

测试节点外层阶段：

- `INITIAL_SAMPLING`：起步前采样目标，期间发布零速度。
- `FIRST_APPROACH`：使用初始平均目标倒车，按第一阶段较宽松阈值粗接近。
- `SECOND_SAMPLING`：第一阶段到位后停止运动，近距离重新采样目标。
- `SECOND_APPROACH`：使用二次平均目标继续接近，按第二阶段阈值和限速参数控制。
- `FINAL_SAMPLING`：第二阶段到位后再次停止运动，重新采样最终目标；如果最终采样比第二阶段到位误差明显变差，会按参数重采样。
- `FINAL_APPROACH`：使用最终平均目标低速精调，线速度和角速度会额外缩放和限幅。
- `FINISHED`：最终阶段位置和角度都到位后持续发布零速度。

默认接口：

- 订阅：`apriltag_target_pose` (`geometry_msgs/msg/PoseStamped`)，`header.frame_id` 应为 `base_link`。
- 发布：`control/cmd_vel_nav` (`geometry_msgs/msg/Twist`)。
- TF：持续发布当前控制目标 `base_link -> reverse_docking_target`，表示锁定目标转换回当前 `base_link` 后的相对误差。
- 静态目标 TF：锁定后持续发布 `fixed_frame -> reverse_docking_static_target`，表示最终锁定目标在固定坐标系下的位置，默认 parent 为 `odom`。
- 日志：锁定平均目标和每阶段到位时打印目标误差；控制过程中节流打印测试阶段、控制器阶段、原始速度、限速后速度、`local_x`、`local_y`、距离、倒车方向误差和最终姿态误差。进入外层阶段时会打印 `-------------------------` 横幅；低频关键事件保留英文并追加中文说明；高频节流日志保留英文，便于 `analyze_auto_charging_log.py` 继续按英文关键字统计。

启动示例：

```bash
ros2 run auto_charging_bt test_reverse_docking_controller
```

常用调参示例：

```bash
ros2 run auto_charging_bt test_reverse_docking_controller --ros-args -p max_linear_speed:=0.12 -p linear_gain:=0.25 -p heading_gain:=1.0
```

常用参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `initial_sampling_duration` | `1.0` | 起步前目标采样时长，采样期间发布零速度，结束后使用平均目标点进入第一阶段控制 |
| `second_sampling_duration` | `2.0` | 第一阶段到位后的近距离二次采样时长，采样期间发布零速度 |
| `final_sampling_duration` | `1.0` | 第二阶段到位后的最终采样时长 |
| `final_resampling_duration` | `2.0` | 最终采样跳变被拒绝后的重采样时长 |
| `fixed_frame` | `odom` | 锁定最终目标点使用的固定坐标系，控制时每周期转换回当前 `base_link` |
| `static_target_frame` | `reverse_docking_static_target` | 固定坐标系下最终锁定目标的 TF 子坐标系，仅用于 RViz 可视化 |
| `linear_gain` | `0.40` | 纵向位置误差到倒车速度的比例 |
| `heading_gain` | `1.20` | 倒车方向误差到角速度的比例 |
| `final_yaw_gain` | `0.90` | 最终姿态 PID 的 P 增益 |
| `final_yaw_i_gain` | `0.0` | 最终姿态 PID 的 I 增益，默认关闭积分 |
| `final_yaw_d_gain` | `0.05` | 最终姿态 PID 的 D 增益，用于降低末端旋转过冲 |
| `final_yaw_i_limit` | `0.30` | 最终姿态 PID 积分限幅 |
| `final_yaw_offset` | `0.0` | 最终姿态误差偏置。当前 `apriltag_target_pose` 已经表示机器人最终背对充电桩的姿态，默认不额外加减 180 度；只有现场目标姿态定义需要修正时才调整该值 |
| `final_max_angular_speed` | `0.20` | `ROTATING_TO_GOAL` 最终姿态修正阶段的角速度上限 |
| `final_min_angular_speed` | `0.0` | `ROTATING_TO_GOAL` 最终姿态修正阶段的角速度下限，默认不强制最小角速度 |
| `second_stage_linear_scale` | `0.9` | 第二阶段对控制器线速度输出的比例缩放 |
| `second_stage_angular_scale` | `0.9` | 第二阶段对控制器角速度输出的比例缩放 |
| `second_stage_max_linear_speed` | `0.10` | 第二阶段线速度绝对值上限 |
| `second_stage_max_angular_speed` | `0.30` | 第二阶段角速度绝对值上限 |
| `final_stage_linear_scale` | `0.5` | 最终阶段对控制器线速度输出的比例缩放 |
| `final_stage_angular_scale` | `0.7` | 最终阶段对控制器角速度输出的比例缩放 |
| `final_stage_max_linear_speed` | `0.08` | 最终阶段线速度绝对值上限 |
| `final_stage_max_angular_speed` | `0.20` | 最终阶段角速度绝对值上限 |
| `approach_distance` | `0.25` | 小于该距离后进入末端调整阶段 |
| `large_heading_threshold` | `0.35` | 倒车方向误差超过该值时暂停线速度，优先修正姿态 |
| `linear_tolerance` | `0.020` | 纵向到位阈值 |
| `lateral_tolerance` | `0.020` | 横向到位阈值 |
| `final_yaw_tolerance` | `0.05` | 最终姿态到位阈值 |

### 行为树 `Dock`

`Dock` 已接入 `ReverseDockingController`，运行时不再读取本包 `config/charging_pose.json` 作为近距离对接目标。它订阅 perception 节点发布的 `apriltag_target_pose`，默认要求目标位姿在 `base_link` 下；起步先采样并平均目标，锁定到 `fixed_frame`（默认 `odom`）后开始第一阶段倒车。第一阶段到位后停车并重新采样近距离目标进入第二阶段，第二阶段到位后再采样最终目标进入最终精对准。最终阶段到位后，`Dock` 会继续执行 `backward()` 做末端插入；一旦检测到充电器连接或 `battery_charging=true` 就停止后退，然后返回成功给行为树。

默认接口：

- 订阅：`apriltag_target_pose` (`geometry_msgs/msg/PoseStamped`)，`header.frame_id` 应为 `base_link`。
- 发布：`control/cmd_vel_nav` (`geometry_msgs/msg/Twist`)。
- TF：锁定目标后发布 `fixed_frame -> reverse_docking_static_target` 和 `base_link -> reverse_docking_target`，用于观察固定目标和当前相对误差。

## ROS 接口

### Action

- 名称：`/auto_charging`
- 类型：`chassis_interfaces/action/AutoCharging`
- 入口：`auto_charging_action_server_node`

### Service

- 名称：`/auto_charging`
- 类型：`std_srvs/srv/Trigger`
- 入口：`auto_charging_service_node`

### 订阅

- `/diagnostics_agg` (`diagnostic_msgs/msg/DiagnosticArray`)：读取电源板充电状态、电量和充电器连接状态。
- `apriltag_target_pose` (`geometry_msgs/msg/PoseStamped`)：`Dock` 读取由 perception 节点输出的最终倒车对接目标，默认坐标系为 `base_link`。
- `apriltag_poses` (`geometry_msgs/msg/PoseArray`)：旧 `test_docking_controller` 仍可用于独立测试原 `DockingController`。

### 发布

- `control/cmd_vel_nav` (`geometry_msgs/msg/Twist`)：对接、后退、脱离充电桩阶段的速度输出。

### 调用的外部服务

- `start_apriltag_detection` (`std_srvs/srv/Trigger`)
- `stop_apriltag_detection` (`std_srvs/srv/Trigger`)

### 调用的外部 Action

- `navigate_to_pose` (`nav2_msgs/action/NavigateToPose`)

## 配置文件

### `config/charging_bt.xml`

行为树主流程文件。当前主流程为：

- `Fallback AutoChargingRoot`
- `IsCharging`：入口稳定采样确认已充电时直接成功
- `IsConnectedToCharger`：入口稳定采样确认已连接未充电时先 `Undocking`，再进入正常流程
- `ChargingFlow`：导航最多重试 3 次，Dock/CheckCharging 最多重试 3 次
- 每次 Dock 尝试启动 AprilTag 检测；失败恢复时执行 `StopDetection` 和 `Undocking`
- 成功或整体失败后由 Action/Service 入口兜底调用 `stopDetection()`

### `config/auto_charging_bt_params.yaml`

Action 和 Service launch 共同加载的参数文件，当前包含入口状态确认、末端后退、充电确认后的顶桩压力释放、退桩和 `dock.reverse.*` 倒车对接参数。该文件使用 `/**` 通配节点名，因为参数由进程内的 `AutoChargingNode` 读取，而不是 Action server 外层节点读取。

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `entry_state.max_duration` | `0.5` | 入口状态确认最长采样时长 |
| `entry_state.sample_interval` | `0.1` | 入口状态采样间隔 |
| `entry_state.required_stable_count` | `3` | 入口状态连续一致次数要求 |
| `check_charging.required_stable_count` | `5` | `CheckCharging` 连续检测到充电的次数要求 |
| `pressure_relief.speed` | `0.01` | 充电确认成功后的前进速度，单位 m/s |
| `pressure_relief.duration` | `0.5` | 充电确认成功后固定发送前进速度的时长，单位 s |
| `pressure_relief.control_rate` | `50.0` | 顶桩压力释放速度的发布频率，单位 Hz |

### `config/charging_pose.json`

旧 `DockingController` 独立测试节点使用的目标位姿配置。当前行为树 `Dock` 已改用 `apriltag_target_pose`，不再读取该文件作为近距离倒车目标。字段格式如下：

```json
{
  "charging_pose": {
    "position": {
      "x": -0.59,
      "y": 0.0,
      "z": 0.2233
    },
    "quaternion": {
      "x": 0.5118663850946755,
      "y": 0.502399087034448,
      "z": 0.48490665874768624,
      "w": 0.5004532879892161
    }
  }
}
```

注意：`readChargingPose()` 读取的是另一种导航目标格式，当前 Action/Service 入口硬编码读取 `/home/dexforce/w1/install/mobile_perception/lib/mobile_perception/charging_pose.json`，该文件需要包含 `frame_id`、`position` 和 `orientation`。

## 开发约定

- 修改控制器、行为树节点、ROS 接口、参数、launch、配置文件或测试节点时，需要同步更新本包 `README.md` 和 `AGENTS.md`。
- 新增回充控制律时，优先创建独立控制类和独立测试节点；确认控制行为后，再接入 `Dock` 或行为树流程。
- 未经现场确认，不要直接运行会发布速度、启动导航、触发回充或调用 `/auto_charging` 的命令。

## 参数

`config/auto_charging_bt_params.yaml` 中配置了以下通用行为参数：

| 参数 | 默认示例 | 说明 |
| --- | --- | --- |
| `backward.initial_speed` | `-0.02` | 初始后退速度 |
| `backward.connected_speed` | `-0.005` | 兼容旧参数；当前检测到连接或正在充电会立即停止后退，不再切换慢速等待 |
| `backward.max_distance` | `0.13` | 后退最大距离 |
| `backward.stable_duration` | `1.0` | 兼容旧参数；当前不再等待连接稳定时长 |
| `backward.max_duration` | `7.0` | 后退最大时长 |
| `backward.control_rate` | `50.0` | 后退控制频率 |
| `backward.max_dt` | `0.1` | 后退距离积分最大时间步长 |
| `undocking.speed` | `0.10` | 脱离充电桩速度 |
| `undocking.duration` | `2.0` | 脱离充电桩时长 |
| `entry_state.max_duration` | `0.5` | 入口状态确认最长采样时长 |
| `entry_state.sample_interval` | `0.1` | 入口状态采样间隔 |
| `entry_state.required_stable_count` | `3` | 入口状态连续一致次数要求 |
| `check_charging.required_stable_count` | `5` | 充电确认连续稳定次数 |
| `pressure_relief.speed` | `0.01` | 充电确认成功后的前进速度，单位 m/s |
| `pressure_relief.duration` | `0.5` | 固定发送前进速度的时长，单位 s |
| `pressure_relief.control_rate` | `50.0` | 前进速度发布频率，单位 Hz |

`Dock` 反向对接参数使用 `dock.reverse.*` 命名空间：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `dock.reverse.target_topic` | `apriltag_target_pose` | 倒车对接目标位姿 topic |
| `dock.reverse.base_frame` | `base_link` | 控制器计算误差使用的机器人基坐标系 |
| `dock.reverse.fixed_frame` | `odom` | 锁定平均目标点使用的固定坐标系 |
| `dock.reverse.initial_sampling_duration` | `1.0` | 起步前目标采样时长 |
| `dock.reverse.second_sampling_duration` | `2.0` | 第一阶段到位后的二次采样时长 |
| `dock.reverse.final_sampling_duration` | `1.0` | 第二阶段到位后的最终采样时长 |
| `dock.reverse.max_sample_age` | `1.5` | 允许使用的 AprilTag 目标消息最大年龄 |
| `dock.reverse.linear_gain` | `0.40` | 纵向误差到倒车速度的比例 |
| `dock.reverse.heading_gain` | `1.20` | 倒车方向误差到角速度的比例 |
| `dock.reverse.final_yaw_gain` | `0.90` | 末端姿态 PID 的 P 增益 |
| `dock.reverse.final_yaw_i_gain` | `0.0` | 末端姿态 PID 的 I 增益 |
| `dock.reverse.final_yaw_d_gain` | `0.05` | 末端姿态 PID 的 D 增益 |
| `dock.reverse.final_yaw_offset` | `0.0` | 最终姿态误差偏置，默认不额外加减 180 度 |
| `dock.reverse.max_linear_speed` | `0.18` | 第一阶段线速度上限 |
| `dock.reverse.max_angular_speed` | `0.45` | 第一阶段角速度上限 |
| `dock.reverse.final_max_angular_speed` | `0.20` | 末端姿态修正角速度上限 |
| `dock.reverse.second_stage_linear_scale` | `0.9` | 第二阶段线速度缩放 |
| `dock.reverse.second_stage_angular_scale` | `0.9` | 第二阶段角速度缩放 |
| `dock.reverse.second_stage_max_linear_speed` | `0.10` | 第二阶段线速度上限 |
| `dock.reverse.second_stage_max_angular_speed` | `0.30` | 第二阶段角速度上限 |
| `dock.reverse.final_stage_linear_scale` | `0.5` | 最终阶段线速度缩放 |
| `dock.reverse.final_stage_angular_scale` | `0.7` | 最终阶段角速度缩放 |
| `dock.reverse.final_stage_max_linear_speed` | `0.08` | 最终阶段线速度上限 |
| `dock.reverse.final_stage_max_angular_speed` | `0.20` | 最终阶段角速度上限 |

## 依赖

主要依赖：

- `rclcpp`
- `rclcpp_action`
- `geometry_msgs`
- `diagnostic_msgs`
- `nav2_msgs`
- `std_srvs`
- `behaviortree_cpp_v3`
- `nlohmann_json`
- `ament_index_cpp`
- `tf2`
- `tf2_ros`
- `tf2_geometry_msgs`
- `chassis_interfaces`
- 外部包 `mobile_perception` 提供 AprilTag 检测节点和启停服务

## 测试脚本

`test/auto_charging_test.py` 用于多轮回充流程测试：

```bash
python3 test/auto_charging_test.py --count 5
```

脚本会随机导航到预设点位，然后调用 `/auto_charging` Action，并统计成功率。该脚本会发布速度和调用真实导航/回充流程，只应在确认现场安全后运行。

## 已知注意事项

- `launch/auto_charging_bt.launch.py` 引用的 `auto_charging_bt_node` 当前没有在 `CMakeLists.txt` 中构建。
- Action 入口的行为树 XML 已通过 `ament_index_cpp` 从包内加载；导航目标 JSON 仍使用硬编码路径。
- Service 入口的行为树 XML 已通过 `ament_index_cpp` 从包内加载；导航目标 JSON 仍使用硬编码路径。
- 行为树 `Dock` 已不再使用 `DockingController`；该控制器仍保留给 `test_docking_controller` 等旧测试节点使用。
- 维护接口、TF、Action 或感知依赖时，需要同步复核 `package.xml` 和 `CMakeLists.txt`。
