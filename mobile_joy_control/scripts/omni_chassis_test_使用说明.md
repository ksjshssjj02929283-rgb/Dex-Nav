# 全向底盘测试脚本使用说明

## 1. 文档目的

本文档用于指导全向底盘运动测试脚本 `omni_chassis_test.py` 的使用。

脚本可以完成以下测试：

- 沿可配置命令坐标系的 X 正/负方向直行；
- 沿可配置命令坐标系的 Y 正/负方向横移；
- 沿可配置命令坐标系下任意指定角度平移；
- 原地向左或向右旋转指定角度；
- 按用户定义的速度、距离、旋转角度和动作顺序执行有限循环测试；
- 记录 map 位姿、理论目标、里程计速度、IMU 倾角和四轮电机反馈；
- 记录底盘左右汇总电流、温度、状态字和错误码；
- 自动生成 CSV、测试汇总和曲线图片；
- 每一步运动前调用 `SteeringAngleControl` 静止预对舵，到位后才发送非零速度；
- 停止发送运动速度后继续采样一段时间，用于观察停车过冲。

平移模式只发送线速度，令 `angular.z=0`；原地旋转模式只发送 `angular.z`，令线速度为 0。脚本不提供无限循环，避免车辆在无人值守时持续运动。

---

## 2. 测试前安全检查

底盘测试会真实动车。开始前必须完成以下检查：

- 清空机器人周围人员、线缆、支架和其他障碍物；
- 确认急停按钮可用，并安排人员随时准备按下急停；
- 第一次测试建议先将驱动轮悬空，确认四轮方向和转速正确；
- 落地测试先使用较低速度和较短时间，例如 `0.05 m/s`、`1 s`；
- 停止导航、遥控器、手柄及其他可能发送底盘速度的节点；
- 确认定位或建图系统已经正常工作，能够提供 `map -> base_link` TF；
- 确认 `/feedback/odom` 正常更新；
- 不要在狭小区域直接进行长时间或高速测试。

脚本默认向以下话题发送速度：

```text
/control/cmd_vel_omni
```

开始前使用下面的命令检查是否还有其他发布者：

```bash
ros2 topic info /control/cmd_vel_omni --verbose
```

如果除了测试脚本和正常的底盘控制链路以外，还有导航、手柄或其他测试节点在发送该话题，应先停止这些节点。脚本检测到其他发布者时会输出警告，但不会替用户强制关闭其他节点。

---

## 3. 坐标系与角度定义

平移测试方向由 `command_frame_id` 决定，默认值为建图或定位使用的
`map`。也可以设为 `odom`、`base_link` 或其他能够查询到 TF 的坐标系。

- `0°`：命令坐标系 X 正方向；
- `90°`：命令坐标系 Y 正方向；
- `180°`：命令坐标系 X 负方向；
- `-90°`：命令坐标系 Y 负方向；
- 正角度：从命令坐标系 X 正方向开始逆时针旋转；
- 负角度：从命令坐标系 X 正方向开始顺时针旋转。

模式与方向对应关系：

| mode 参数 | 运动方向 | 对应角度 |
|---|---|---:|
| `x_positive` | X 正方向直行 | `0°` |
| `x_negative` | X 负方向直行 | `180°` |
| `y_positive` | Y 正方向横移 | `90°` |
| `y_negative` | Y 负方向横移 | `-90°` |
| `angle` | 任意角度平移 | 由 `translation_angle_deg` 指定 |
| `rotate_left` | 原地逆时针旋转 | 由 `rotation_angle_deg` 指定 |
| `rotate_right` | 原地顺时针旋转 | 由 `rotation_angle_deg` 指定 |

机器人自身朝向不会改变上述方向定义。脚本会查询
`command_frame_id -> base_link`，把该坐标系中的速度转换为 region 接收的
车体坐标系速度。`geometry_msgs/Twist` 没有 `header.frame_id`，所以
`command_frame_id` 是脚本参数，而不是消息字段。

---

## 4. 脚本使用的话题

默认话题如下：

| 用途 | 话题 | 消息类型 |
|---|---|---|
| 底盘速度命令 | `/control/cmd_vel_omni` | `geometry_msgs/msg/Twist` |
| 起步静止预对舵 | `/steering_angle_control` | `nav_interfaces/action/SteeringAngleControl` |
| 里程计反馈 | `/feedback/odom` | `nav_msgs/msg/Odometry` |
| 四轮位置和速度 | `/feedback/omni_chassis_motor` | `chassis_interfaces/msg/ChassisMotorControl` |
| 底盘 IMU | `/imu/chassis/refined` | `sensor_msgs/msg/Imu` |
| 底盘汇总状态 | `/feedback/robot_server_state` | `std_msgs/msg/String`，内容为 JSON |
| 轨迹记录位姿变换 | `map -> base_link` | TF |
| 命令速度变换 | `command_frame_id -> base_link` | TF |

其中 `/feedback/robot_server_state` 当前能够提供：

- 左右汇总速度；
- 左右汇总电流；
- 驱动器温度；
- 左右温度；
- 底盘状态字；
- 左右错误码。

当前机器人没有通过该通道发布底盘电机力矩，也没有发布四个舵向电机和四个行走电机各自独立的电流、温度或力矩。测试脚本不会伪造这些数据。

---

## 5. 编译

进入 ROS 2 工作空间。下面以 `/home/dexforce/omni_test` 为例，如果实际路径不同，需要替换为实际路径。

```bash
cd /home/dexforce/omni_test
source /opt/ros/humble/setup.bash
colcon build --packages-select mobile_joy_control
source install/setup.bash
```

如果工作空间已经完成编译，只需要执行：

```bash
cd /home/dexforce/omni_test
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## 6. 启动测试节点

推荐为输出目录指定绝对路径：

```bash
cd /home/dexforce/omni_test
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run mobile_joy_control omni_chassis_test.py --ros-args \
  -p output_dir:=/home/dexforce/omni_test/omni_test_logs \
  -p verbose:=true
```

正常启动后会看到类似日志：

```text
全向底盘测试节点已启动
命令话题=/control/cmd_vel_omni
odom=/feedback/odom
电机反馈=/feedback/omni_chassis_motor
IMU=/imu/chassis/refined
底盘状态=/feedback/robot_server_state
```

启动节点本身不会立即动车。只有执行：

```bash
ros2 param set /omni_chassis_test run_test true
```

后，脚本才会开始一次测试。

### 6.1 不编译时直接运行源码

如果依赖已经安装或已经通过工作空间构建，也可以直接运行源码：

```bash
cd /home/dexforce/omni_test
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 src/mobile_joy_control/scripts/omni_chassis_test.py --ros-args \
  -p output_dir:=/home/dexforce/omni_test/omni_test_logs \
  -p settle_tolerance_rad:=0.08 \
  -p verbose:=true
```

如果仓库不是放在工作空间的 `src/mobile_joy_control` 下，应把脚本路径改为实际路径。

---

## 7. 常用测试参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `mode` | `x_positive` | 运动模式 |
| `speed_mps` | `0.10` | 平移速度绝对值，单位 m/s，必须大于 0 |
| `linear_accel_mps2` | `0.50` | 平移起步加速度斜坡，单位 m/s²；只在本脚本发布 Twist 时生效 |
| `linear_decel_mps2` | `0.50` | 平移停车减速度斜坡，单位 m/s²；只在本脚本发布 Twist 时生效 |
| `duration_sec` | `3.0` | 整个平移运动时间，包含加速、匀速和减速，单位 s |
| `command_frame_id` | `map` | 平移速度的解释坐标系；支持 `map`、`odom`、`base_link` 或其他有效 TF frame |
| `translation_angle_deg` | `0.0` | `mode=angle` 时相对 `command_frame_id` +X 的平移角度 |
| `presteer_enabled` | `true` | 每一步首个非零速度前是否静止预对舵并等待到位 |
| `steering_action_name` | `/steering_angle_control` | 静止预对舵 Action 名称 |
| `action_timeout_sec` | `3.0` | 预对舵 Action 执行超时，单位 s |
| `settle_tolerance_rad` | `0.08` | 预对舵最大舵角到位误差，单位 rad |
| `steering_feedback_timeout_sec` | `0.2` | 选择最近 π 等价解所用舵角反馈的最大接收延迟 |
| `chassis_length` | `0.460` | 前后轮距，单位 m，必须与 region 一致 |
| `chassis_width` | `0.345` | 左右轮距，单位 m，必须与 region 一致 |
| `steering_limit_rad` | `2.513274` | 舵角绝对限位，单位 rad，必须与 region 一致 |
| `angular_speed_radps` | `0.30` | 原地旋转角速度绝对值，单位 rad/s |
| `rotation_angle_deg` | `90.0` | 单次原地旋转角度，范围 `(0, 360]` |
| `settle_time_sec` | `2.0` | 停止后继续采样的时间，用于观察过冲 |
| `verbose` | `true` | 是否周期输出详细日志 |
| `log_period_sec` | `0.20` | verbose 日志周期，单位 s |
| `overshoot_threshold_m` | `0.01` | 过冲判定阈值，单位 m |
| `rotation_overshoot_threshold_deg` | `2.0` | 原地旋转过冲判定阈值，单位 deg |
| `loop_steps` | 见下文 | 用户定义的循环动作字符串数组 |
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11
例如把预对舵到位阈值调为 `0.05 rad`（约 `2.86°`），需在测试空闲时执行：

```bash
ros2 param set /omni_chassis_test settle_tolerance_rad 0.05
```

也可以在启动脚本时直接指定：

```bash
python3 src/mobile_joy_control/scripts/omni_chassis_test.py --ros-args \
  -p settle_tolerance_rad:=0.05
```
| `loop_count` | `1` | 完整执行 `loop_steps` 的次数，范围 `1~1000` |
| `loop_pause_sec` | `1.0` | 相邻动作之间持续发零速的等待时间，单位 s |
| `publish_rate_hz` | `20.0` | 速度命令发布频率，默认周期为 50 ms |
| `run_test` | `false` | 设置为 `true` 后启动一次测试 |
| `run_loop` | `false` | 设置为 `true` 后启动一次有限循环测试 |
| `cancel_test` | `false` | 设置为 `true` 后立即取消测试并发送零速 |
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
以下参数在节点启动时读取，修改后应重启脚本：

- `publish_rate_hz`
- `command_topic`
- `steering_action_name`
- `odom_topic`
- `motor_feedback_topic`
- `imu_topic`
- `robot_state_topic`
- `map_frame`
- `base_frame`
- `output_dir`

运动参数应在测试空闲时修改。测试进行中修改运动参数会被拒绝。

---

## 8. 执行测试

每次测试的基本流程为：

1. 设置运动模式；
2. 设置速度；
3. 设置运行时间；
4. 如有需要，设置停止后采样时间和过冲阈值；
5. 设置 `run_test=true` 启动测试；
6. 脚本保持零速，计算第一帧运动对应的四轮舵角并调用预对舵 Action；
7. 四轮反馈全部进入容差后，脚本重新记录正式起点，并按加减速度参数生成完整速度斜坡；
8. 等待脚本自动结束；
9. 查看日志、CSV、汇总和图片。

一次测试结束后，`run_test` 会自动恢复为 `false`。节点不会退出，可以继续修改参数并执行下一次测试。

### 8.0.1 起步预对舵

默认 `presteer_enabled=true`。每次单次测试以及循环测试的每一个步骤都会执行：

```text
持续发布零 Twist
  -> 根据第一帧 base_link Twist 计算 FL/FR/RL/RR 目标舵角
  -> 根据最新舵角反馈选择距离最近的 π 等价解
  -> 调用 /steering_angle_control
  -> 等待 Action 报告四轮到位
  -> 重新采集 map 起点并从 t=0 放行非零速度
```

Action 被拒绝、执行失败、超时、反馈过旧或测试被取消时，不会发送非零测试
速度。`summary.txt` 会保存目标舵角、对舵耗时、最终最大误差和结果消息。

只有排查旧行为时才建议临时关闭：

```bash
ros2 param set /omni_chassis_test presteer_enabled false
```

正常落地测试应保持为 `true`。

### 8.0.2 平移加减速度斜坡

平移测试默认由脚本使用 `0.50 m/s²` 加速，并在运动结束前使用
`0.50 m/s²` 减速到零。该功能不修改也不依赖 region 内部的速度处理：

```bash
ros2 param set /omni_chassis_test linear_accel_mps2 0.5
ros2 param set /omni_chassis_test linear_decel_mps2 0.5
```

单次测试的 `duration_sec` 包含加速、匀速和减速时间。如果设置
`speed_mps=0.5`、加减速度均为 `0.5 m/s²`、`duration_sec=3.0`，速度曲线
为“加速 `1.0 s` → 保持 `1.0 s` → 减速 `1.0 s`”，理论距离为
`1.0 m`。

如果运行时间不足以达到 `speed_mps`，脚本自动生成三角形速度曲线。例如
上述参数改为 `duration_sec=1.0` 时，实际峰值速度为 `0.25 m/s`，随后
立即进入减速，理论距离为 `0.125 m`，不会违反加减速度限制。

该斜坡只作用于平移速度，原地旋转保持原来的角速度指令。循环测试的
平移步骤仍以指定距离为准，脚本会自动延长该步骤的时间以补偿起步斜坡。

### 8.0.3 对比 map 与 odom 命令方向

`geometry_msgs/Twist` 没有 Header，因此脚本不能在消息里真正填写
`header.frame_id`。`command_frame_id` 是脚本对输入速度数值的解释方式：

- `map`：使用 `map -> base_link` 把固定 map 方向速度转换成车体速度；
- `odom`：只使用 `odom -> base_link`，不会使用 `map -> odom`；
- `base_link`：不旋转速度，直接赋值到 `Twist.linear.x/y`。

先执行原有 map +X 测试：

```bash
ros2 param set /omni_chassis_test command_frame_id map
ros2 param set /omni_chassis_test mode x_positive
ros2 param set /omni_chassis_test speed_mps 0.05
ros2 param set /omni_chassis_test duration_sec 3.0
ros2 param set /omni_chassis_test run_test true
```

机器人回到相同起始条件后，再执行 odom +X：

```bash
ros2 param set /omni_chassis_test command_frame_id odom
ros2 param set /omni_chassis_test mode x_positive
ros2 param set /omni_chassis_test speed_mps 0.05
ros2 param set /omni_chassis_test duration_sec 3.0
ros2 param set /omni_chassis_test run_test true
```

CSV 中的 `command_frame_yaw_in_map_deg` 会逐周期记录命令坐标系在 map
中的旋转角；`summary.txt` 还会给出其平均值、最小值、最大值和变化范围。
如果 odom 测试相对 map 测试稳定相差约 12°，且这些统计值也接近这个角度，
应重点检查 `map -> odom` 的初始化或定位修正。

### 8.1 X 正方向直行

```bash
ros2 param set /omni_chassis_test mode x_positive
ros2 param set /omni_chassis_test speed_mps 0.05
ros2 param set /omni_chassis_test duration_sec 1.0
ros2 param set /omni_chassis_test settle_time_sec 2.0
ros2 param set /omni_chassis_test run_test true
```

### 8.2 X 负方向直行

```bash
ros2 param set /omni_chassis_test mode x_negative
ros2 param set /omni_chassis_test speed_mps 0.05
ros2 param set /omni_chassis_test duration_sec 1.0
ros2 param set /omni_chassis_test settle_time_sec 2.0
ros2 param set /omni_chassis_test run_test true
```

### 8.3 Y 正方向横移

```bash
ros2 param set /omni_chassis_test mode y_positive
ros2 param set /omni_chassis_test speed_mps 0.05
ros2 param set /omni_chassis_test duration_sec 1.0
ros2 param set /omni_chassis_test settle_time_sec 2.0
ros2 param set /omni_chassis_test run_test true
```

### 8.4 Y 负方向横移

```bash
ros2 param set /omni_chassis_test mode y_negative
ros2 param set /omni_chassis_test speed_mps 0.05
ros2 param set /omni_chassis_test duration_sec 1.0
ros2 param set /omni_chassis_test settle_time_sec 2.0
ros2 param set /omni_chassis_test run_test true
```

### 8.5 任意角度平移

下面的例子表示沿 `command_frame_id` 坐标系 `30°` 方向，以
`0.05 m/s` 运行 `1 s`：

```bash
ros2 param set /omni_chassis_test mode angle
ros2 param set /omni_chassis_test translation_angle_deg 30.0
ros2 param set /omni_chassis_test speed_mps 0.05
ros2 param set /omni_chassis_test duration_sec 1.0
ros2 param set /omni_chassis_test settle_time_sec 2.0
ros2 param set /omni_chassis_test run_test true
```

任意角度模式还可以使用负角度，例如 `-45°`：

```bash
ros2 param set /omni_chassis_test translation_angle_deg -45.0
ros2 param set /omni_chassis_test run_test true
```

### 8.6 原地旋转

下面的例子表示以 `0.30 rad/s` 原地向左旋转 `90°`。脚本会自动用“旋转角度 ÷ 角速度”计算发送命令的持续时间，因此原地旋转模式不使用 `duration_sec`：

```bash
ros2 param set /omni_chassis_test mode rotate_left
ros2 param set /omni_chassis_test angular_speed_radps 0.30
ros2 param set /omni_chassis_test rotation_angle_deg 90.0
ros2 param set /omni_chassis_test settle_time_sec 2.0
ros2 param set /omni_chassis_test run_test true
```

向右旋转只需要把模式改为：

```bash
ros2 param set /omni_chassis_test mode rotate_right
ros2 param set /omni_chassis_test run_test true
```

`rotate_left` 为逆时针，`rotate_right` 为顺时针。两者都以机器人当前 map 航向为起点。

### 8.7 用户自定义循环测试

循环中的每个字符串表示一个动作，格式如下：

```text
固定方向平移：模式,速度m/s,距离m
任意角度平移：angle,速度m/s,距离m,命令坐标系角度deg
原地旋转：rotate_left或rotate_right,角速度rad/s,旋转角度deg
```

例如，下面定义“沿命令坐标系 X+ 前进 0.5 m → 左转 90° →
沿命令坐标系 45° 平移 0.3 m → 右转 90°”，完整动作重复 3 次：

```bash
ros2 param set /omni_chassis_test loop_steps \
  '["x_positive,0.08,0.50", "rotate_left,0.30,90.0", "angle,0.06,0.30,45.0", "rotate_right,0.30,90.0"]'
ros2 param set /omni_chassis_test loop_count 3
ros2 param set /omni_chassis_test loop_pause_sec 1.0
ros2 param set /omni_chassis_test run_loop true
```

这里平移步骤使用“距离”而不是“时间”，脚本内部自动计算：

```text
能达到目标速度时：
  平移时间 = 加速时间 + 匀速时间 + 减速时间
距离短、未达到目标速度时：
  脚本按加速度和减速度计算可达到的三角形曲线峰值及总时间
旋转时间 = 旋转角度（rad）÷ 角速度
总动作数 = loop_steps 中的动作数 × loop_count
```

每一步都会单独执行停止观察、保存 CSV/图片/summary，然后保持零速等待 `loop_pause_sec` 再进入下一步。整个循环结束后，输出目录还会生成一个 `loop_时间戳_summary.txt`，列出所有步骤的结果目录。

循环运行期间不能修改运动参数。如需提前停止整个循环：

```bash
ros2 param set /omni_chassis_test cancel_test true
```

注意：

- 只有 `mode=angle` 时，`translation_angle_deg` 才会生效；
- 循环里的 `angle` 步骤直接在该步骤字符串中写 map 角度；
- 单个旋转步骤范围为 `(0, 360]°`，需要多段旋转时应拆成多个步骤；
- `loop_count` 必须是 `1~1000` 的有限整数，不支持无限循环；
- 循环开始前应先用低速、短距离和 `loop_count=1` 验证动作顺序。

---

## 9. 实时查看日志

测试节点的普通文本日志默认保存在输出目录下：

```text
/home/dexforce/omni_test/omni_test_logs/omni_chassis_test.log
```

可以在另一个终端实时查看：

```bash
tail -f /home/dexforce/omni_test/omni_test_logs/omni_chassis_test.log
```

verbose 日志包括：

- 当前测试阶段；
- 已运行时间和设定运行时间；
- 命令坐标系中的平移角度；
- 原地旋转角速度、目标角度和累计实际旋转角度；
- 当前 map 位姿；
- 当前理论位姿；
- 理论终点；
- X、Y 和 yaw 终点误差；
- odom 反馈速度；
- IMU roll、pitch、总倾角和加速度；
- 四轮位置与速度；
- 左右汇总电流；
- 驱动器及左右温度；
- 底盘状态字和左右错误码。

如果日志过于频繁，可以调大日志周期：

```bash
ros2 param set /omni_chassis_test log_period_sec 0.5
```

如果不需要周期详细日志：

```bash
ros2 param set /omni_chassis_test verbose false
```

---

## 10. 正常停止、取消测试与紧急处理

### 10.1 正常停止

平移达到 `duration_sec`，或原地旋转达到自动计算的命令时间后，脚本会自动：

1. 停止发送非零速度；
2. 周期发送零速；
3. 继续采样 `settle_time_sec`；
4. 保存结果；
5. 返回空闲状态。

### 10.2 取消当前测试

```bash
ros2 param set /omni_chassis_test cancel_test true
```

脚本会立即取消当前测试并发布零速。如果正在执行循环，剩余动作也会全部取消，不会继续启动下一步。已经采集到的数据会保存为：

```text
samples_aborted.csv
```

### 10.3 退出测试节点

在运行测试脚本的终端按：

```text
Ctrl-C
```

脚本退出前会连续发布多次零速。

### 10.4 紧急情况

发生运动方向错误、底盘失控、速度异常或周围人员进入测试区时：

1. 优先按下机器人硬件急停；
2. 不要只依赖终端命令；
3. 确认底盘完全停止后再排查原因。

如果脚本仍能响应，可以同时执行取消命令：

```bash
ros2 param set /omni_chassis_test cancel_test true
```

---

## 11. 输出目录与结果文件

每次测试都会创建一个独立目录，目录名包含时间、模式和速度。例如：

```text
omni_test_logs/
├── omni_chassis_test.log
├── loop_20260728_150000_000000_summary.txt
├── latest -> 20260728_150000_000000_x_positive_0.050mps
└── 20260728_150000_000000_x_positive_0.050mps/
    ├── summary.txt
    ├── samples.csv
    ├── trajectory_map.png
    ├── odom_velocity_response.png
    ├── imu_tilt.png
    ├── chassis_electrical_status.png
    └── motor_feedback.png
```

`latest` 软链接始终指向最近一次测试目录。

循环测试仍然为每个动作建立独立目录，目录名中会带有循环会话名和 `step0001`、`step0002` 等序号；循环汇总文件位于输出根目录。

查看最近一次测试汇总：

```bash
cat /home/dexforce/omni_test/omni_test_logs/latest/summary.txt
```

查看最近一次测试的全部文件：

```bash
ls -lh /home/dexforce/omni_test/omni_test_logs/latest/
```

---

## 12. 结果文件说明

### 12.1 summary.txt

测试摘要，主要包括：

- 测试模式、速度、时间和角度；
- 理论运动距离；
- map 起点、理论终点和实际终点；
- 最终 X 误差；
- 最终 Y 误差；
- 最终 yaw 误差；
- 纵向误差；
- 横向误差；
- 最大停车过冲；
- IMU 最大倾角变化；
- 左右最大绝对电流；
- 最终状态字和错误码；
- 温度数据是否有效；
- 当前没有底盘力矩数据的说明。

### 12.2 samples.csv

完整逐周期原始数据，可用于后续 Python、Excel、MATLAB 或其他工具分析。

主要字段包括：

- `time_sec`：本次测试已运行时间；
- `phase`：当前阶段，通常为 `running` 或 `settling`；
- `map_x/map_y/map_yaw_rad`：实际 map 位姿；
- `theory_x/theory_y`：当前时刻理论位置；
- `target_error_x/target_error_y/target_error_yaw_rad`：相对最终目标误差；
- `command_frame_id`：本次命令使用的参考坐标系；
- `command_frame_yaw_in_map_rad/deg`：命令坐标系在 map 中的当前 yaw；
- `cmd_vx_command_frame/cmd_vy_command_frame`：设置坐标系中的原始命令速度；
- `cmd_vx_map/cmd_vy_map`：map 下命令速度；
- `cmd_vx_base/cmd_vy_base`：转换到底盘坐标系后的命令速度；
- `cmd_wz`：原地旋转命令角速度；
- `rotation_progress_rad`：按指定旋转方向累计的实际旋转角度，可跨越 ±π；
- `odom_vx_base/odom_vy_base/odom_wz`：原始 odom 速度；
- `odom_vx_map/odom_vy_map`：转换到 map 下的 odom 速度；
- `odom_age_sec`：odom 消息时间戳相对当前时间的差值；
- `imu_roll_deg/imu_pitch_deg/imu_tilt_deg`：IMU 倾角；
- `imu_angular_velocity_*`：IMU 角速度；
- `imu_linear_acceleration_*`：IMU 线加速度；
- `front_left/front_right/rear_left/rear_right_position`：四轮位置；
- `front_left/front_right/rear_left/rear_right_speed`：四轮速度；
- `chassis_current_left/right`：左右汇总电流；
- `chassis_driver_temperature`：驱动器温度；
- `chassis_temperature_left/right`：左右温度；
- `chassis_temperature_valid`：温度是否被判定为有效；
- `chassis_status_word`：底盘状态字；
- `chassis_error_code_left/right`：左右错误码；
- `robot_state_receive_age_sec`：底盘汇总状态距离当前采样时刻的接收延迟。

### 12.3 trajectory_map.png

平移测试显示：

- map 下的实际运动轨迹；
- 理论直线轨迹；
- 起点；
- 理论终点；
- 实际终点。

该图适合观察：

- 直线运动是否跑偏；
- 横移时是否产生纵向偏差；
- 任意角度运动方向是否正确；
- 最终位置是否达到理论目标。

### 12.4 odom_velocity_response.png

显示：

- map 下命令速度；
- map 下 odom 反馈速度；
- 理论停止时刻；
- 沿运动方向的实际进度；
- 理论运动距离。

原地旋转测试显示：

- 命令角速度和 odom 反馈角速度；
- 按指定方向累计的实际旋转角；
- 理论旋转角；
- 命令停止时刻。

该图适合观察：

- odom 速度响应是否滞后；
- 加速过程是否过慢；
- 停止后速度是否拖尾；
- 是否出现速度过冲；
- 停止后位置是否继续前进。

如果怀疑命令速度和 odom 速度相差约 `100 ms`，可以对比命令曲线与 odom 曲线的上升沿和下降沿。CSV 中的数据可以用于进一步做互相关或时间对齐分析。

### 12.5 imu_tilt.png

显示：

- 绝对 roll、pitch 和总倾角；
- 相对测试起点的 roll、pitch 变化；
- 三轴角速度；
- 三轴线加速度。

IMU 绝对角度会受到安装角度影响。分析加减速造成的俯仰或横移造成的侧倾时，应优先查看相对起点变化，而不是只看绝对角度。

### 12.6 chassis_electrical_status.png

显示：

- 左右汇总电流；
- 驱动器及左右温度；
- 状态字；
- 左右错误码。

当前机器人上温度字段可能持续为 0。脚本会保留原始值，但会将三个温度全零标记为“未上报或无效”，不会解释为真实的 `0°C`。

该图片不包含力矩，因为当前机器人状态通道没有发布底盘力矩。

### 12.7 motor_feedback.png

显示四个轮组的：

- 位置变化；
- 速度变化。

位置和速度单位沿用底层 `ChassisMotorControl` 消息定义。该图适合观察：

- 四轮转向是否同步；
- 某个轮组是否响应异常；
- 轮速是否存在明显不一致；
- 停止后是否仍有轮速残留。

---

## 13. 如何理解位置误差

### 13.1 最终 X、Y 和 yaw 误差

脚本使用下面的方式计算最终误差：

```text
X 误差 = 实际终点 X - 理论终点 X
Y 误差 = 实际终点 Y - 理论终点 Y
yaw 误差 = 实际终点 yaw - 理论终点 yaw
```

正负号表示误差方向。

### 13.2 纵向误差

沿设定运动方向的距离误差：

- 正值：实际走得比理论距离更远；
- 负值：实际走得比理论距离更短。

### 13.3 横向误差

垂直于设定运动方向的偏移量，适合判断直线运动或横移是否跑偏。

### 13.4 最大过冲

测试停止后，实际位置超过理论终点的最大距离。

如果最大过冲超过 `overshoot_threshold_m`，汇总文件中的 `overshoot_detected` 会标记为 `True`。

原地旋转使用累计角度计算角度误差和旋转过冲，判定阈值由 `rotation_overshoot_threshold_deg` 设置。累计计算不会因为 yaw 从 `+180°` 跳到 `-180°` 而丢失旋转进度。

### 13.5 这些误差是否等于纯里程计误差

不完全等于。

脚本的目标位置和最终位置在 `map` 坐标系下计算，因此误差可能同时包含：

- 轮式里程计累计误差；
- 轮胎打滑；
- 地面不平；
- 四轮安装和标定误差；
- SLAM 或定位系统对 `map -> odom` 的修正；
- TF 和传感器时间延迟。

因此 map 下误差适合评价整套机器人系统的最终运动效果，但不能单独当作纯轮式里程计误差。

如果需要严格评估底盘绝对精度，建议同时使用激光跟踪仪、动作捕捉系统、全站仪或外部固定标记进行独立测量。

---

## 14. 当前 region 下的低速测试注意事项

当前 region 已没有驻车锁存、`0.8 s` 保护期或 Unpark 状态机。

- 零 Twist：四轮轮速为零并保持当前反馈舵角；
- 非零 Twist：立即进入普通几何解算；
- 如果舵角误差超过 region 的 Two-Phase 阈值，四个驱动轮保持零速并先对舵；
- 测试脚本不会主动调用 `SteeringAngleControl`。

因此低速测试不再需要超过旧版驻车解除阈值，但仍应确认没有导航、遥控器
或其他节点同时发布 `/control/cmd_vel_omni`。

---

## 15. 发布周期设置

脚本默认速度发布频率为：

```text
20 Hz，也就是每 50 ms 发布一次
```

如果希望每 `20 ms` 发布一次，可以启动时设置为 `50 Hz`：

```bash
ros2 run mobile_joy_control omni_chassis_test.py --ros-args \
  -p output_dir:=/home/dexforce/omni_test/omni_test_logs \
  -p publish_rate_hz:=50.0
```

如果希望每 `10 ms` 发布一次，可以启动时设置为 `100 Hz`：

```bash
ros2 run mobile_joy_control omni_chassis_test.py --ros-args \
  -p output_dir:=/home/dexforce/omni_test/omni_test_logs \
  -p publish_rate_hz:=100.0
```

注意：

- `publish_rate_hz` 只控制测试脚本发送速度和采样的周期；
- 它不会自动修改 `mobile_cmd_omni_region.cpp` 内部控制定时器周期；
- 它也不会修改 odom 发布周期、CAN/EtherCAT 通信周期或底层驱动器响应周期；
- 分析约 `100 ms` 延迟时，需要同时检查命令发布周期、控制节点周期、底层总线周期和 odom 发布周期。

修改 `publish_rate_hz` 后需要重启测试脚本。

---

## 16. 常见问题

### 16.1 设置 run_test=true 后一直不开始

脚本启动测试至少需要：

- `/feedback/odom` 消息；
- `map -> base_link` TF。

检查 odom：

```bash
ros2 topic echo /feedback/odom --once
```

检查 TF：

```bash
ros2 run tf2_ros tf2_echo map base_link
```

如果没有 map TF，应先启动建图或定位系统。

### 16.2 参数设置成功，但机器人不动

依次检查：

1. 是否仍处于硬件急停；
2. 底盘驱动是否使能；
3. 是否有其他节点覆盖 `/control/cmd_vel_omni`；
4. 底盘是否处于驻车保持状态；
5. 测试速度是否正好为或低于 `0.04 m/s`；
6. `mobile_cmd_omni` 控制节点是否正在运行；
7. 四轮反馈和 odom 是否正常更新。

检查速度话题：

```bash
ros2 topic echo /control/cmd_vel_omni
```

检查发布者：

```bash
ros2 topic info /control/cmd_vel_omni --verbose
```

### 16.3 没有 IMU 图片

检查默认 IMU：

```bash
ros2 topic echo /imu/chassis/refined --once
```

如果实际 IMU 话题不同，在启动脚本时指定：

```bash
ros2 run mobile_joy_control omni_chassis_test.py --ros-args \
  -p imu_topic:=/实际的imu话题
```

如果 IMU 消息将 `orientation_covariance[0]` 设置为 `-1`，表示没有有效姿态，脚本不会把默认四元数错误解释为水平姿态。

### 16.4 温度一直是 0

当前机器人底层可能没有真正上报温度。脚本会：

- 在 CSV 中保留收到的原始 0；
- 将 `chassis_temperature_valid` 写为 `0`；
- 在汇总中标记为 `unavailable_or_all_zero`；
- 在图片中提示温度全零或不可用。

不要把该值解释为真实环境或驱动器温度。

### 16.5 没有力矩数据

当前已确认的话题和底层共享状态中没有底盘力矩字段，因此脚本无法记录真实力矩。

若后续驱动器新增力矩或转矩电流反馈，应先明确：

- 新话题名称；
- 消息类型；
- 单位；
- 电机编号与四个轮组的对应关系；
- 数值是目标值还是实际反馈值。

确认后再扩展脚本。

### 16.6 没有生成图片

脚本生成图片依赖 `matplotlib`。如果日志提示未安装，可以检查：

```bash
python3 -c "import matplotlib; print(matplotlib.__version__)"
```

如果运行环境确实缺少该依赖，应按照机器人现有的软件管理流程处理。不要在未确认的情况下对机器人执行系统级 `update`、`upgrade` 或批量安装操作。

即使没有 `matplotlib`，CSV 和 summary 仍会保留。

### 16.7 机器人运动方向与预期不一致

检查：

- 当前使用的是否为 map 坐标系；
- 定位结果中的机器人 yaw 是否正确；
- `map -> base_link` TF 是否跳变；
- 底盘控制节点是否使用相同的底盘坐标定义；
- 四个轮组的位置顺序和正负方向是否正确。

### 16.8 odom 速度比控制速度晚约 100 ms

可能的延迟来源包括：

- 测试脚本发布周期；
- `mobile_cmd_omni` 控制周期；
- 底盘总线通信周期；
- 电机驱动器自身响应；
- odom 计算和发布周期；
- ROS 2 消息排队；
- 时间戳来源不一致。

建议先执行：

1. 将脚本发布频率设置为 `50 Hz`；
2. 保持测试速度和时间不变；
3. 对比 `odom_velocity_response.png` 的上升沿和下降沿；
4. 检查 `samples.csv` 中命令速度、odom 速度和消息时间字段；
5. 再决定是否需要调整控制节点内部周期。

---

## 17. 推荐测试顺序

第一次使用建议按以下顺序执行：

1. 驱动轮悬空，`0.05 m/s`、`1 s`，测试 X 正方向；
2. 确认四轮方向和速度正确；
3. 落地后清空测试区域；
4. 依次测试 X 正、X 负、Y 正、Y 负；
5. 测试 `30°`、`45°`、`-45°` 等任意角度；
6. 低角速度测试左转、右转各 `30°`，确认转向符号正确；
7. 再测试左转、右转各 `90°`；
8. 使用 `loop_count=1` 验证自定义循环的动作顺序；
9. 确认每一步都能停稳后，再增加循环次数；
10. 对比最终误差、横向误差、旋转角误差和最大过冲；
11. 检查 IMU 倾角和左右电流是否存在明显异常；
12. 确认低速结果稳定后，再逐步增加速度、距离和循环次数。

建议每次只改变一个变量。例如保持运行时间和角度不变，只改变速度；或者保持速度不变，只改变运动方向。

---

## 18. 测试记录模板

可将下面内容复制到每次测试记录中：

```text
测试日期：
测试人员：
软件版本/提交号：
地图名称：
地面材质：
载荷情况：
电池电量：

运动模式：
速度：
运行时间：
平移角度：
旋转角速度：
旋转角度：
循环动作：
循环次数：
停止采样时间：
发布频率：

起点：
理论终点：
实际终点：
X 误差：
Y 误差：
yaw 误差：
纵向误差：
横向误差：
最大过冲：

IMU 最大 roll 变化：
IMU 最大 pitch 变化：
左右最大电流：
底盘状态字：
左右错误码：

异常现象：
结果目录：
备注：
```
