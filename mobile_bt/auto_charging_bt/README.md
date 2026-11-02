# auto_charging_bt

`auto_charging_bt` 是 ROS 2 自动回充行为树包，负责依次完成：

1. 判断机器人当前是否正在充电。
2. 根据 `/command_board/navigation_backend` 选择是否使用 Nav2 导航到地图中的唯一 `CHARGING` 点。
3. 根据 AprilTag 目标进行低速倒车对接。
4. 末端低速压入，并确认是否开始充电。

## 1. 快速使用

### 1.1 启动回充 Action

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch auto_charging_bt auto_charging_action.launch.py
```

以上命令在已经完成构建的 ROS 2 工作空间根目录执行。

该 launch 会启动：

- `/auto_charging` Action server。
- AprilTag 对齐节点 `apriltag_alignment_tf_node`。

### 1.2 确认接口存在

```bash
ros2 action list | grep auto_charging
ros2 action info /auto_charging
```

Action 名称和类型：

```text
/auto_charging
chassis_interfaces/action/AutoCharging
```

### 1.3 调用一次回充

推荐使用空 Goal：

```bash
ros2 action send_goal /auto_charging chassis_interfaces/action/AutoCharging "{}"
```

需要同时查看 feedback（反馈）时使用：

```bash
ros2 action send_goal /auto_charging chassis_interfaces/action/AutoCharging \
  "{}" --feedback
```

`goal` 是 `ros2 action send_goal` 的必填参数，完全省略会由命令行直接报错，Goal 不会发送。
传入 `"{}"` 时，布尔字段 `force` 使用默认值 `false`，Action 仍会正常执行。

当前源码只打印 `force`，尚未使用它改变回充逻辑：

| Goal 内容 | `force` 实际值 | 当前执行效果 |
| --- | --- | --- |
| `"{}"` | `false`（默认值） | 正常执行完整回充流程 |
| `"{force: false}"` | `false` | 正常执行完整回充流程 |
| `"{force: true}"` | `true` | 正常执行完整回充流程 |

因此 `force=true` 和 `force=false` 目前没有区别。该字段从命名上预留给“强制回充”，但相关业务
逻辑尚未实现；当前 `true` 不会跳过入口状态判断或任何安全流程。

`--feedback` 只让终端持续显示 `current_state` 和 `retry_count`，不会改变机器人行为。
`retry_count` 表示导航重试次数与对接重试次数之和，两个阶段的第一次尝试都不计为重试；需要区分
具体是哪个阶段发生重试时，查看 `[NavigationRetry]` 和 `[DockRetry]` 日志。

同一时间只允许一个回充任务执行。已有任务运行时，新任务会返回失败。

## 2. 导航后端与地图回充点

回充启动时订阅 `/command_board/navigation_backend`（`std_msgs/msg/String`，transient local）：

- `data: nav2`：读取当前地图中唯一的 `CHARGING` 点，先完成 Nav2 前置导航，再进入二维码对接。
- `data: path_navigation`：调度侧负责把机器人送到回充区域，回充流程跳过前置导航，直接进入二维码对接。

话题超时、空值或其它值都会使回充在运动前失败。任务执行期间只有导航后端状态值实际变化才会
立即停止；同值重复发布仍表示同一个当前状态，不会中断本次回充。当前生效值可通过下面命令确认：

```bash
ros2 topic echo --once /command_board/navigation_backend
```

前置导航点统一保存在当前地图的 `assets/map_assets.json` 中，点位类型必须为
`CHARGING`，且整个文件只能有一个 `CHARGING` 点。例如：

```json
{
  "name": "charging_point",
  "type": "CHARGING",
  "pose": {"x": 3.0, "y": -1.0, "yaw": 3.14},
  "tolerance": 0.1
}
```

可以使用 `map_asset_manager` 的点位服务或 RViz 地图资产工具配置该点；不再使用
`/save_charging_pose`，也不再生成独立的 `charging_pose.json`。

导航后端为 `nav2` 时，回充启动会等待 `/map_manager/map_state`，读取当前地图资产，
并在运动前检查 `CHARGING` 点数量。零个或多个都会直接失败并打印点位名称。
读取成功后会在本次回充任务期间持续监控 `map_folder`、`map_name` 和 `epoch`；任一字段变化都表示
当前地图已切换或重新加载，任务会立即停止行为树、取消尚在执行的 Nav2 目标并返回失败，避免把旧地图
回充点作为新地图坐标继续使用。

导航后端为 `path_navigation` 时，不访问地图资产，机器人直接进入二维码识别和精确对接阶段。

Action 取消时，如果 Nav2 目标句柄尚未返回，回充节点会继续处理回调，拿到本次目标句柄后执行定向取消。
Nav2 接受取消请求后，节点还会继续等待该目标进入最终终态，再结束回充 Action，避免把“已接受取消”误当成
“导航已经停止”。末端压入、退桩和压力释放这些同步运动也会在每个控制周期检查 Action 取消、地图变化和
导航后端变化；检测到中断后立即发布零速度，由外层结束当前任务。

入口充电状态在导航后端和地图读取之前确认；机器人已经稳定处于充电状态时直接返回成功，不要求
`/command_board/navigation_backend` 或地图中的 `CHARGING` 点可用。

## 3. 完整回充流程

正式流程由 `config/charging_bt.xml` 定义：

```text
自动回充
├── 1. 检查入口充电状态
│   ├── 已经充电：直接返回成功
│   └── 未充电：执行正常回充
│
└── 2. 正常回充
    ├── navigation_backend=nav2：导航到唯一 CHARGING 点，最多尝试 3 次
    │   └── navigation_backend=path_navigation：跳过该阶段
    └── 对接与充电确认，最多尝试 3 次
        ├── 启动 AprilTag 检测
        ├── 三阶段倒车对接
        ├── 末端低速压入
        └── 确认充电状态
```

失败后的处理规则：

1. 导航失败：重新导航，最多尝试 `3` 次。
2. 二维码检测或视觉对接失败：停止检测，不退桩，重新尝试对接。
3. 视觉对接和末端压入成功，但没有确认充电：停止检测、退桩，然后重新尝试对接。
4. 导航和对接分别计数；对接失败不会重新执行已经成功的前置导航。

## 4. 两类目标位姿

| 目标           | 用途                             | 来源                                                                        |
| -------------- | -------------------------------- | --------------------------------------------------------------------------- |
| 充电前置导航点 | Nav2`/navigate_to_pose` 的目标 | 当前地图 `assets/map_assets.json` 中唯一的 `type=CHARGING` 点 |
| 近距离对接目标 | 计算机器人相对最终对接位的误差   | `apriltag_target_pose`，默认位于 `base_link`                            |

本包不再使用 `config/charging_pose.json` 作为正式回充配置。

## 5. 三阶段倒车对接

对接过程按以下顺序执行：

1. `INITIAL_SAMPLING`：机器人停车，采集并过滤 AprilTag 目标。
2. `FIRST_APPROACH`：锁定目标，进行第一阶段粗靠近。
3. `SECOND_SAMPLING`：停车，近距离重新采样。
4. `SECOND_APPROACH`：降低速度并使用更严格阈值继续靠近。
5. `FINAL_SAMPLING`：最终采样；结果明显变差时进行有限次数重采样。
6. `FINAL_APPROACH`：低速修正最终位置和姿态。
7. `backward()`：视觉到位后进行末端低速压入；检测到充电时立即停车。

采样目标最初位于 `base_link`。控制器会把目标锁定到固定坐标系 `odom`，机器人运动时再把它转换回
当前 `base_link`，因此控制误差会随机器人位置变化，而不是一直使用一份不变的相对坐标。

## 6. 日志查看与失败分类

### 6.1 主要日志标签

| 标签                                            | 含义                                        |
| ----------------------------------------------- | ------------------------------------------- |
| `[NavigationAttempt]` / `[NavigationRetry]` | 前置导航尝试和重试                          |
| `[TagStats]`                                  | AprilTag 图像、检出率、目标发布率和错误统计 |
| `[DockAttempt]` / `[DockRetry]`             | 对接尝试和重试                              |
| `[DockStage]`                                 | 当前对接阶段                                |
| `[DockSample]`                                | 采样开始、样本不足和样本丢弃                |
| `[DockControl]`                               | 当前目标误差和速度指令                      |
| `[ChargingCheck]`                             | 充电和连续确认次数                          |
| `[DockFailure]`                               | 统一失败分类、中文原因、处置和诊断计数      |
| `[DockSuccess]`                               | 视觉对接、末端压入或充电确认成功            |

### 6.2 统一失败分类

| 分类码                     | 含义                           | 优先检查                           |
| -------------------------- | ------------------------------ | ---------------------------------- |
| `NO_TARGET`              | 没有收到二维码目标             | 相机、检测节点、目标话题           |
| `QR_DETECTION_TIMEOUT`   | 停车采样阶段连续超过配置时间没有收到有效二维码目标 | 相机、检测服务、目标话题和机器人朝向 |
| `FRAME_MISMATCH`         | 目标坐标系不符合配置           | `base_frame` 和感知输出 frame    |
| `INVALID_SAMPLES`        | 收到目标，但样本全部无效       | 时间戳、距离限制、坐标系           |
| `INSUFFICIENT_SAMPLES`   | 有效样本未达到最低数量         | 遮挡、反光、近距离视野、检测稳定性 |
| `FINAL_SAMPLE_UNSTABLE`  | 最终采样结果反复变差           | 标签抖动、TF、近距离识别质量       |
| `TF_LOCK_FAILED`         | 无法把目标锁定到`odom`       | `odom -> base_link` TF           |
| `TF_TRACKING_FAILED`     | 运动时无法转换回`base_link`  | TF 连续性和时间戳                  |
| `CONTROL_TIMEOUT`        | 有有效目标，但控制未按时到位   | 速度输出、底盘响应、到位阈值       |
| `DETECTION_START_FAILED` | 启动检测服务失败               | 检测节点和服务状态                 |
| `CHARGING_NOT_CONFIRMED` | 已完成压入，但没有确认稳定充电 | 触点、电源板诊断、充电字段       |

典型失败日志：

```text
[DockFailure] type=NO_TARGET，二维码检测或视觉对接超时；
原因=未收到二维码目标消息；
处置=停止检测、不退桩，由外层行为树决定是否重试；
诊断=stage=INITIAL_SAMPLING ... total_recv=0 total_valid=0 ...
```

诊断字段中：

- `total_recv` / `total_valid`：本次对接累计收到的目标数 / 有效样本数。
- `reject(...)`：因坐标系、时间戳或距离被丢弃的样本数。
- `tf(lock=...,tracking=...)`：目标锁定和动态跟踪的 TF 失败次数。
- `quality(final_inconsistent=...)`：最终样本不稳定并触发重采样的次数。

## 7. 主要配置

参数文件：

```text
config/auto_charging_bt_params.yaml
```

| 参数组                | 作用                                      |
| --------------------- | ----------------------------------------- |
| `entry_state.*`     | 回充开始时对充电状态去抖                  |
| `check_charging.*`  | 连续确认充电成功                          |
| `charging.navigation_backend_wait_timeout_sec` | 等待 command_board 当前导航后端的最长时间 |
| `charging.map_state_wait_timeout_sec` | `nav2` 后端等待当前地图状态的最长时间 |
| `charging.qr_detection_timeout_sec` | 停车采样阶段连续未收到有效二维码目标的最长时间；有效目标会刷新计时 |
| `charging.dock_timeout_sec` | 单次 Dock 阶段的总超时时间          |
| `dock.reverse.*`    | AprilTag 采样、三阶段阈值、速度和控制增益 |
| `backward.*`        | 视觉到位后的末端低速压入                  |
| `undocking.*`       | 充电确认失败时退桩                        |
| `pressure_relief.*` | 确认充电后向前卸载顶桩压力                |

当前关键值：

| 参数或行为树端口             |        当前值 |
| ---------------------------- | ------------: |
| 导航最大尝试次数             |         `3` |
| 对接最大尝试次数             |         `3` |
| 导航后端等待超时             |     `5.0 s` |
| 当前地图状态等待超时         |     `5.0 s` |
| 二维码连续丢失超时           |     `5.0 s` |
| 单次 Dock 阶段总超时          |    `60.0 s` |
| 单次充电确认超时             |    `20.0 s` |
| `dock.reverse.fixed_frame` |      `odom` |
| `backward.initial_speed`   | `-0.02 m/s` |
| `backward.max_distance`    |    `0.15 m` |
| `undocking.speed`          |  `0.10 m/s` |
| `undocking.duration`       |     `3.0 s` |
| `pressure_relief.duration` |     `0.8 s` |

`charging.qr_detection_timeout_sec` 是停车采样阶段的二维码看门狗阈值：进入初始、第二或最终停车采样后，
如果连续没有收到有效二维码目标就累计丢失时间，每收到一次有效目标会刷新计时。连续丢失超过阈值时，
本次对接立即失败并输出 `QR_DETECTION_TIMEOUT`。采样完成后目标已锁定到 `odom`；倒车运动期间不再读取
二维码，也不执行该看门狗，即使二维码离开视野仍按锁定目标继续。整个对接流程始终受
`charging.dock_timeout_sec` 的总时间限制。
完整参数及注释以 YAML 和 `charging_bt.xml` 为准。速度、距离、姿态阈值和超时修改后必须在真实机器人上保守验证。

## 8. ROS 接口

### 8.1 对外接口

| 名称               | 类型                                       | 作用                                               |
| ------------------ | ------------------------------------------ | -------------------------------------------------- |
| `/auto_charging` | `chassis_interfaces/action/AutoCharging` | 推荐的自动回充入口                                 |
| `/auto_charging` | `std_srvs/srv/Trigger`                   | 兼容 Service 入口，仅在 Service 版本节点启动时存在 |

Action 和 Service 名称相同，但接口类型不同。联调前使用 `ros2 action list` 和 `ros2 service list` 确认实际启动的入口。

兼容 Service 的启动与调用：

```bash
ros2 launch auto_charging_bt auto_charging_service.launch.py
ros2 service call /auto_charging std_srvs/srv/Trigger "{}"
```

### 8.2 依赖接口

| 名称                         | 类型                                    | 作用                       |
| ---------------------------- | --------------------------------------- | -------------------------- |
| `/command_board/navigation_backend` | `std_msgs/msg/String`             | 决定先导航还是直接对接     |
| `/navigate_to_pose`        | `nav2_msgs/action/NavigateToPose`     | 导航到充电前置点           |
| `/diagnostics_agg`         | `diagnostic_msgs/msg/DiagnosticArray` | 获取电量和充电状态     |
| `apriltag_target_pose`     | `geometry_msgs/msg/PoseStamped`       | 近距离对接目标             |
| `start_apriltag_detection` | `std_srvs/srv/Trigger`                | 启动 AprilTag 检测         |
| `stop_apriltag_detection`  | `std_srvs/srv/Trigger`                | 停止 AprilTag 检测         |
| `control/cmd_vel_nav`      | `geometry_msgs/msg/Twist`             | 对接、压入、退桩和卸压速度 |

## 9. 常见问题

### 9.1 `/auto_charging` 不存在

```bash
ros2 action list | grep auto_charging
ros2 node list | grep auto_charging
```

确认 `auto_charging_action_server_node` 已启动，并确认当前终端已 source 正确工作空间。

### 9.2 无法读取地图回充点

先确认导航后端确实为 `nav2`：

```bash
ros2 topic echo --once /command_board/navigation_backend
```

确认 `/map_manager/map_state` 已发布，并检查当前地图的
`assets/map_assets.json` 中恰好存在一个 `type=CHARGING` 点，且该点包含有限的
`pose.x`、`pose.y` 和 `pose.yaw`。

### 9.3 修改 XML 或 YAML 后没有生效

程序通过包的安装目录读取配置。非 `--symlink-install` 构建时，修改源码后需要重新构建、重新
source，并检查：

```bash
colcon build --packages-select auto_charging_bt --parallel-workers 8
source install/setup.bash
```

```text
install/auto_charging_bt/share/auto_charging_bt/config/
```

### 9.4 新回充请求立即失败

Action server 使用执行锁保证单任务运行。如果已有回充任务尚未结束，新请求会被中止并返回
`Another auto charging task is already running`。

## 10. 安全说明

回充会发送 Nav2 目标，并在近距离对接阶段直接发布底盘速度。真实机器人测试前必须：

1. 清空机器人和充电桩周围的运动区域。
2. 保持急停可用。
3. 安排人员现场观察。
4. 确认前置导航点、AprilTag 方向和充电桩位置正确。

未确认现场安全时，不要调用回充 Action、兼容 Service 或任何对接测试节点。
