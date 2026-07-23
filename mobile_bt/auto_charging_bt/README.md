# auto_charging_bt

`auto_charging_bt` 是 ROS 2 自动回充行为树包，负责依次完成：

1. 判断机器人当前是否已连接或正在充电。
2. 使用 Nav2 导航到充电桩前置点。
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

## 2. 首次使用前保存充电前置点

### 2.1 保存步骤

1. 将机器人遥控到充电桩正前方约 `0.5 m`。
2. 确认机器人定位正常，朝向适合直接开始倒车对接。
3. 调用保存服务：

```bash
ros2 service call /save_charging_pose std_srvs/srv/Trigger "{}"
```

4. 确认服务响应中的 `success=true`。
5. 检查保存结果：

```bash
cat /home/dexforce/w1/install/mobile_perception/lib/mobile_perception/charging_pose.json
```

该文件由 `mobile_perception/charging_pose_service_node` 保存，正式回充会从这里读取 Nav2
前置导航点。当前路径在 Action 和 Service 源码中固定为：

```text
/home/dexforce/w1/install/mobile_perception/lib/mobile_perception/charging_pose.json
```

文件至少应包含 `frame_id`、`position` 和 `orientation`：

```json
{
  "frame_id": "map",
  "position": {"x": 0.0, "y": 0.0, "z": 0.0},
  "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
}
```

“约 `0.5 m`”是现场标定要求，保存服务不会自动检查机器人与充电桩的距离。

## 3. 完整回充流程

正式流程由 `config/charging_bt.xml` 定义：

```text
自动回充
├── 1. 检查入口充电状态
│   ├── 已经充电：直接返回成功
│   ├── 已连接但未充电：先退桩，再执行正常回充
│   └── 未连接且未充电：执行正常回充
│
└── 2. 正常回充
    ├── 导航到充电前置点，最多尝试 3 次
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
| 充电前置导航点 | Nav2`/navigate_to_pose` 的目标 | `/save_charging_pose` 保存的 `mobile_perception/.../charging_pose.json` |
| 近距离对接目标 | 计算机器人相对最终对接位的误差   | `apriltag_target_pose`，默认位于 `base_link`                            |

本包的 `config/charging_pose.json` 只供旧测试节点使用，正式回充行为树不会读取它作为前置导航点或实时对接目标。

## 5. 三阶段倒车对接

对接过程按以下顺序执行：

1. `INITIAL_SAMPLING`：机器人停车，采集并过滤 AprilTag 目标。
2. `FIRST_APPROACH`：锁定目标，进行第一阶段粗靠近。
3. `SECOND_SAMPLING`：停车，近距离重新采样。
4. `SECOND_APPROACH`：降低速度并使用更严格阈值继续靠近。
5. `FINAL_SAMPLING`：最终采样；结果明显变差时进行有限次数重采样。
6. `FINAL_APPROACH`：低速修正最终位置和姿态。
7. `backward()`：视觉到位后进行末端低速压入；检测到连接或充电时立即停车。

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
| `[ChargingCheck]`                             | 连接、充电和连续确认次数                    |
| `[DockFailure]`                               | 统一失败分类、中文原因、处置和诊断计数      |
| `[DockSuccess]`                               | 视觉对接、末端压入或充电确认成功            |

### 6.2 统一失败分类

| 分类码                     | 含义                           | 优先检查                           |
| -------------------------- | ------------------------------ | ---------------------------------- |
| `NO_TARGET`              | 没有收到二维码目标             | 相机、检测节点、目标话题           |
| `FRAME_MISMATCH`         | 目标坐标系不符合配置           | `base_frame` 和感知输出 frame    |
| `INVALID_SAMPLES`        | 收到目标，但样本全部无效       | 时间戳、距离限制、坐标系           |
| `INSUFFICIENT_SAMPLES`   | 有效样本未达到最低数量         | 遮挡、反光、近距离视野、检测稳定性 |
| `FINAL_SAMPLE_UNSTABLE`  | 最终采样结果反复变差           | 标签抖动、TF、近距离识别质量       |
| `TF_LOCK_FAILED`         | 无法把目标锁定到`odom`       | `odom -> base_link` TF           |
| `TF_TRACKING_FAILED`     | 运动时无法转换回`base_link`  | TF 连续性和时间戳                  |
| `CONTROL_TIMEOUT`        | 有有效目标，但控制未按时到位   | 速度输出、底盘响应、到位阈值       |
| `DETECTION_START_FAILED` | 启动检测服务失败               | 检测节点和服务状态                 |
| `CHARGING_NOT_CONFIRMED` | 已完成压入，但没有确认稳定充电 | 触点、电源板诊断、连接和充电字段   |

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
| `entry_state.*`     | 回充开始时对连接和充电状态去抖            |
| `check_charging.*`  | 连续确认充电成功                          |
| `dock.reverse.*`    | AprilTag 采样、三阶段阈值、速度和控制增益 |
| `backward.*`        | 视觉到位后的末端低速压入                  |
| `undocking.*`       | 已连接未充电或充电确认失败时退桩          |
| `pressure_relief.*` | 确认充电后向前卸载顶桩压力                |

当前关键值：

| 参数或行为树端口             |        当前值 |
| ---------------------------- | ------------: |
| 导航最大尝试次数             |         `3` |
| 对接最大尝试次数             |         `3` |
| 单次视觉对接超时             |    `60.0 s` |
| 单次充电确认超时             |    `20.0 s` |
| `dock.reverse.fixed_frame` |      `odom` |
| `backward.initial_speed`   | `-0.02 m/s` |
| `backward.max_distance`    |    `0.15 m` |
| `undocking.speed`          |  `0.10 m/s` |
| `pressure_relief.duration` |     `0.8 s` |

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
| `/navigate_to_pose`        | `nav2_msgs/action/NavigateToPose`     | 导航到充电前置点           |
| `/diagnostics_agg`         | `diagnostic_msgs/msg/DiagnosticArray` | 获取连接、电量和充电状态   |
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

### 9.2 无法读取充电前置点

```bash
cat /home/dexforce/w1/install/mobile_perception/lib/mobile_perception/charging_pose.json
```

检查文件是否存在、JSON 格式是否正确，以及运行用户是否有读取权限。

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
