# auto_charging_bt

`auto_charging_bt` 是 ROS 2 自动回充行为树包。它先使用 Nav2 导航到充电桩前置点，再根据 AprilTag 感知结果完成低速倒车对接，最后从电源板诊断信息确认是否开始充电。

## 回充流程

正式流程由 `config/charging_bt.xml` 定义：

```text
自动回充 AutoChargingRoot
├── 已经充电：直接成功
├── 已连接但未充电：先退桩，再进入正常回充
└── 正常回充
    ├── 导航到充电前置点，最多尝试 3 次
    └── 对接和充电确认，最多尝试 3 次
        ├── 启动 AprilTag 检测
        ├── 三阶段倒车对接
        ├── 末端低速插入
        └── 连续确认充电；失败则停止检测、退桩并重试
```

入口会先对 `/diagnostics_agg` 稳定采样，区分“已充电”“已连接但未充电”和“未连接未充电”。导航与对接分别计数，某一次对接失败不会重新执行已经成功的前置导航。

## 推荐启动与调用

启动 Action server 和 AprilTag 对齐节点：

```bash
ros2 launch auto_charging_bt auto_charging_action.launch.py
```

发送一次回充目标：

```bash
ros2 action send_goal /auto_charging chassis_interfaces/action/AutoCharging "{force: true}" --feedback
```

Action 是推荐接口：

- 名称：`/auto_charging`
- 类型：`chassis_interfaces/action/AutoCharging`
- 同一时间只执行一个回充目标。
- feedback（反馈）包含当前行为树状态和尝试次数。
- 成功、失败或取消时会停止 AprilTag 检测并发布零速度兜底。

兼容的 Trigger service 入口：

```bash
ros2 launch auto_charging_bt auto_charging_service.launch.py
ros2 service call /auto_charging std_srvs/srv/Trigger "{}"
```

Action 与 Service 名称相同但接口类型不同，调用前应使用 `ros2 action list` 或 `ros2 service list` 确认实际启动的入口。

## 设置充电前置导航点

将机器人遥控到充电桩正前方约 `0.5 m`，确认定位和朝向正确，然后调用：

```bash
ros2 service call /save_charging_pose std_srvs/srv/Trigger "{}"
```

该服务由 `mobile_perception/charging_pose_service_node` 提供，它采样 `map -> base_link` 并保存到：

```text
/home/dexforce/w1/install/mobile_perception/lib/mobile_perception/charging_pose.json
```

当前 Action 和 Service 源码都硬编码读取这个文件。调用后应确认响应 `success=true`，并检查文件存在且内容包含：

```json
{
  "frame_id": "map",
  "position": {"x": 0.0, "y": 0.0, "z": 0.0},
  "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
}
```

“约 `0.5 m`”是现场放置要求，程序不会自动检查机器人到充电桩的实际距离。

## 两种目标位姿不要混淆

| 目标 | 用途 | 来源 |
| --- | --- | --- |
| 充电前置导航点 | Nav2 `/navigate_to_pose` 的目标 | `/save_charging_pose` 保存的 `mobile_perception/.../charging_pose.json` |
| 近距离对接目标 | 倒车控制器计算机器人相对最终对接位的误差 | `apriltag_target_pose`，默认坐标系为 `base_link` |
| 本包 `config/charging_pose.json` | 旧 `test_docking_controller` 的测试目标 | 本包源码/安装配置 |

正式行为树 `Dock` 不读取本包 `config/charging_pose.json` 作为近距离目标。

## 倒车对接原理

`Dock` 使用 `ReverseDockingController`，外层分为三次停车采样和三段接近：

1. `INITIAL_SAMPLING`：保持零速度，采样并过滤 `apriltag_target_pose`。
2. `FIRST_APPROACH`：把平均目标锁定到 `odom`，进行第一阶段粗靠近。
3. `SECOND_SAMPLING`：停车后重新采样近距离目标。
4. `SECOND_APPROACH`：使用更小阈值和更低限速继续靠近。
5. `FINAL_SAMPLING`：最终停车采样；若结果相对上一阶段明显变差，会有限次数重采样。
6. `FINAL_APPROACH`：低速修正最终位置和姿态。
7. `backward()`：视觉到位后继续低速插入，检测到连接或充电立即停车。

目标锁定后不会直接把 `base_link` 下的相对坐标当常量。程序把目标锁定在固定坐标系 `odom`，每个控制周期再转换回当前 `base_link`，因此误差会随机器人运动正确变化。

底层控制器状态：

- `APPROACHING`：倒车靠近。
- `ADJUST_HEADING`：优先减小倒车方向误差。
- `ADJUST_DISTANCE`：修正纵向距离和横向误差。
- `ROTATING_TO_GOAL`：使用最终姿态 PID 低速修正 yaw。
- `FINISH`：位置和姿态同时满足阈值后输出零速度。

## ROS 接口

### 订阅

- `/diagnostics_agg` (`diagnostic_msgs/msg/DiagnosticArray`)：充电状态、电量和连接状态。
- `apriltag_target_pose` (`geometry_msgs/msg/PoseStamped`)：正式倒车对接目标。
- `apriltag_poses` (`geometry_msgs/msg/PoseArray`)：仅旧控制器测试使用。

### 发布

- `control/cmd_vel_nav` (`geometry_msgs/msg/Twist`)：对接、末端插入、退桩和压力释放速度。
- `odom -> reverse_docking_static_target`：锁定目标可视化 TF。
- `base_link -> reverse_docking_target`：当前相对误差可视化 TF。

### 调用的外部接口

- `/navigate_to_pose` (`nav2_msgs/action/NavigateToPose`)
- `start_apriltag_detection` (`std_srvs/srv/Trigger`)
- `stop_apriltag_detection` (`std_srvs/srv/Trigger`)

## 参数配置

参数文件为 `config/auto_charging_bt_params.yaml`。它使用 `/**`，因为参数实际由 Action/Service 进程内部创建的 `AutoChargingNode` 读取。

### 入口与充电确认

| 参数 | 当前值 | 作用 |
| --- | ---: | --- |
| `entry_state.max_duration` | `0.5 s` | 入口状态最长采样时间 |
| `entry_state.sample_interval` | `0.1 s` | 入口状态采样间隔 |
| `entry_state.required_stable_count` | `3` | 连续一致次数 |
| `check_charging.required_stable_count` | `5` | 连续检测到正在充电的次数 |

### 末端动作

| 参数 | 当前值 | 作用 |
| --- | ---: | --- |
| `backward.initial_speed` | `-0.02 m/s` | 视觉到位后的插入速度 |
| `backward.max_distance` | `0.16 m` | 最大插入距离 |
| `backward.max_duration` | `8.0 s` | 最大插入时间 |
| `undocking.speed` | `0.10 m/s` | 退桩速度 |
| `undocking.duration` | `2.0 s` | 参数默认退桩时间；XML 可通过端口覆盖 |
| `pressure_relief.speed` | `0.01 m/s` | 确认充电后的前进卸压速度 |
| `pressure_relief.duration` | `1.0 s` | 当前卸压持续时间 |

检测到充电器连接或 `battery_charging=true` 后，末端插入会立即停止，不会继续使用 `connected_speed` 慢速顶桩。

### `dock.reverse.*`

重点参数：

| 参数 | 当前值 | 作用 |
| --- | ---: | --- |
| `target_topic` | `apriltag_target_pose` | 感知目标话题 |
| `base_frame` | `base_link` | 控制误差坐标系 |
| `fixed_frame` | `odom` | 锁定目标坐标系 |
| `initial_sampling_duration` | `1.0 s` | 初始采样时间 |
| `second_sampling_duration` | `2.0 s` | 第二阶段采样时间 |
| `final_sampling_duration` | `1.0 s` | 最终采样时间 |
| `linear_gain` | `0.40` | 纵向控制增益 |
| `heading_gain` | `1.20` | 倒车方向控制增益 |
| `final_yaw_gain` | `0.90` | 最终姿态 PID 的 P 增益 |
| `final_yaw_d_gain` | `0.05` | 最终姿态 PID 的 D 增益 |
| `final_yaw_offset` | `0.0 rad` | 最终姿态偏置，当前不额外旋转 180° |
| `max_linear_speed` | `0.18 m/s` | 控制器最大线速度 |
| `max_angular_speed` | `0.45 rad/s` | 控制器最大角速度 |
| `final_max_angular_speed` | `0.20 rad/s` | 最终姿态修正角速度上限 |

完整分阶段阈值、采样过滤、速度缩放和加速度限制以 YAML 为准。修改这些参数会影响对接稳定性、碰撞风险、触点压紧程度和成功率，必须在真实机器人上保守验证。

## 测试节点

### `test_reverse_docking_controller`

单独验证当前倒车对接控制器：

```bash
ros2 run auto_charging_bt test_reverse_docking_controller
```

它会真实发布 `control/cmd_vel_nav`，只有在机器人周围安全且急停可用时才能运行。

### `test_odom_docking_controller`

验证旧 `DockingController` 对 `base_link` 目标的响应：

```bash
ros2 run auto_charging_bt test_odom_docking_controller
```

### `test_docking_controller`

旧 AprilTag PoseArray 对接测试，会读取本包 `config/charging_pose.json`：

```bash
ros2 run auto_charging_bt test_docking_controller
```

这些测试节点和正式行为树可能使用不同参数，不应把独立测试结果直接等同于正式回充行为。

## 目录结构

```text
auto_charging_bt/
├── config/
│   ├── charging_bt.xml
│   ├── auto_charging_bt_params.yaml
│   └── charging_pose.json
├── include/
├── launch/
│   ├── auto_charging_action.launch.py
│   ├── auto_charging_service.launch.py
│   └── auto_charging_bt.launch.py
├── src/
└── test/
```

## 常见问题

### `/auto_charging` 不存在

```bash
ros2 action list | grep auto_charging
ros2 node list | grep auto_charging
```

正式流程应确认 `auto_charging_action_server_node` 已启动。

### 无法读取前置点 JSON

```bash
cat /home/dexforce/w1/install/mobile_perception/lib/mobile_perception/charging_pose.json
```

检查路径、JSON 格式和运行用户权限。该路径为硬编码，换用户、换部署目录或换安装前缀后不会自动适配。

### 修改 XML/YAML 后没有生效

Action/Service 使用 `ament_index_cpp::get_package_share_directory()` 读取安装目录配置。非 `--symlink-install` 时，需要由用户重新构建并重新 source，再检查 `install/auto_charging_bt/share/auto_charging_bt/config/`。

### Action 与 Service 行为不完全一致

两个 launch 都启动 AprilTag 对齐节点，但 Action launch 当前额外设置 `target_forward_offset=0.67`，Service launch 没有显式设置。联调时应先统一确认感知节点的实际参数。

### 旧 launch 启动失败

`auto_charging_bt.launch.py` 引用了 `auto_charging_bt_node`，但该可执行文件当前没有在 `CMakeLists.txt` 中生成。请优先使用 `auto_charging_action.launch.py`。

## 安全说明

回充过程会发送 Nav2 目标并直接发布底盘速度。真实机器人测试时必须清空运动区域、保持急停可用并安排人员观察。未经现场确认，不要运行回充 Action、Service 或对接测试节点。
