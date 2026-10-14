# mobile_joy_control 使用说明

本包负责手柄遥控、姿态与动作脚本触发，以及系统停止命令下发。支持差速和全向底盘，按设备名称选择 Logitech 或北通参数组。

**急停功能只负责定义按钮并发布请求。实际停止、急停状态保持和解除由 supervisor、底盘等其他模块实现。**

## 大致框架

```text
手柄 → joy_node → /joy → mobile_joy_control
                         ├── 遥控速度 → cmd_vel_mux → /control/cmd_vel → 底盘
                         ├── 停止命令 → /supervisor/system_command → 其他模块
                         └── 姿态与动作脚本

全向导航：导航速度 → omni_start_coordinator → cmd_vel_mux
                       └── 请求底盘静止对舵
```

- `mobile_joy_control`：识别手柄输入，处理遥控、组合键和脚本按钮。
- `omni_start_coordinator`：全向导航起步前请求底盘对舵，对舵完成后放行导航速度。底盘解算和电机控制由底盘模块负责。
- `joy_only.launch.py`：启动手柄驱动、遥控节点和速度选择器；底盘配置为 `omni` 时额外启动起步协调节点。
- `w1_keyboard_control.py`：独立键盘遥控脚本，默认启动文件不运行它。

## 接口

| 节点 | 方向 | 话题 | 消息类型 | 用途 |
| --- | --- | --- | --- | --- |
| mobile_joy_control | 订阅 | `/joy` | `sensor_msgs/msg/Joy` | 按键和摇杆输入 |
| mobile_joy_control | 发布 | `/control/cmd_vel_joy` | `geometry_msgs/msg/Twist` | 差速遥控速度 |
| mobile_joy_control | 发布 | `/control/cmd_vel_omni_joy` | `geometry_msgs/msg/Twist` | 全向遥控速度 |
| mobile_joy_control | 发布 | `/control/joy_in` | `std_msgs/msg/Int16` | 手柄状态 |
| mobile_joy_control | 发布 | `/supervisor/system_command` | `supervisor_interfaces/msg/SystemCommand` | 软件停止、硬件急停请求 |
| omni_start_coordinator | 订阅 | `/control/cmd_vel_nav_presteer` | `geometry_msgs/msg/Twist` | 对舵前的导航速度 |
| omni_start_coordinator | 发布 | `/control/cmd_vel_nav` | `geometry_msgs/msg/Twist` | 经起步协调后输出的导航速度 |
| omni_start_coordinator | 订阅 | `/control/nav_auto_mode`、`/control/nav_auto_task_id` | `std_msgs/msg/Bool`、`std_msgs/msg/UInt64` | 自动模式与任务边界 |
| omni_start_coordinator | 订阅 | `/received_global_plan` | `nav_msgs/msg/Path` | 起步对舵预测路径 |
| omni_start_coordinator | 订阅 | `/feedback/chassis_motor` | `chassis_interfaces/msg/ChassisMotorFeedback` | 底盘转向反馈 |
| omni_start_coordinator | 动作客户端 | `/steering_angle_control` | `nav_interfaces/action/SteeringAngleControl` | 请求底盘静止对舵 |

遥控速度按 `omni_mode` 选择一个话题发布。`joy_in` 状态值：`-2` 无手柄数据，`-1` 未初始化，`0` 遥控未使能，`1` 遥控使能或正在松键减速。

## 功能和按键

以下编号对应当前 **F710 D 模式**的 `/joy.buttons` 下标。使用其他模式或手柄时，需要根据 `/joy` 核对编号。

| 实体按键 | 编号 | 功能 |
| --- | --- | --- |
| RB | 5 | 初始化及保护期完成后，按住使能遥控、松开后按配置平滑减速；未就绪时按下不接管 Mux |
| LB | 4 | 按住提速；普通上限为 0.4 m/s、0.75 rad/s，提速上限为 0.6 m/s、1.0 rad/s |
| Y | 3 | 执行 `standby.json`，进入待命姿态 |
| A | 1 | 执行 `rest.json`，进入休息姿态 |
| X | 0 | 先进入待命姿态，再回放 `bulls` 动作 |
| B | 2 | 先进入待命姿态，再回放 `guns` 动作 |
| RT + LB | 7 + 4 | 软件停止按钮组合，发布 `command=5`（`QUICKSTOP`） |
| RT + LT | 7 + 6 | 硬件急停按钮组合，发布 `command=6`（`EMERGENCY_STOP`） |

**遥控与初始化：**左摇杆上下控制前后，右摇杆左右控制转向；轴 5、4 分别提供固定线速度和固定角速度输入，最终输出仍受当前速度上限约束。全向模式增加侧向移动。首次使用先将左右摇杆向内斜上推，再松开全部按键并让摇杆回零，等待 0.5 秒保护期。遥控未使能且未执行脚本时，长期闲置需重新初始化，默认超时为 30 秒。

**脚本控制：**姿态按钮调用 `slowly_move_to.py`，动作按钮调用 `joy_shuffle_replay_in_folder.py`。需要完成初始化、释放遥控使能并等待松键减速结束；同一时间只执行一个脚本流程。脚本路径和记录文件在参数文件中配置。

**停止组合：**两键按下即可发送，无需长按、摇杆初始化或 RB 使能。每次按住 RT 去重，支持软件停止升级为硬件急停；三键在同一条消息中按下时优先发 6。松开 RT 后可以再次发送，不需要重启节点。RT+LB 成立时不同时启用 LB 提速。

本模块不因停止请求清零遥控速度、锁住操作或中断脚本，也不发送急停解除命令。请求下发后，停止与恢复流程由接收模块负责。原有松开 RB 减速、手柄数据超时清速等普通遥控保护继续生效。

## 配置和使用

主要参数位于 `config/params.yaml`：`logitech`、`beitong` 为两套设备配置。`omni_mode` 需要与运行配置 `robot_config.yaml` 中的 `chassis_config.chassis_type` 保持一致；当前两套手柄配置均为差速模式。启动文件默认读取 `~/w1/dexe_robot_server_data/robot_config.yaml`，可用 `chassis_config_file` 启动参数指定其他路径。

停止组合的公共键为 `stop_enable_button`，软件停止键为 `quick_stop_button`，硬件急停键为 `emergency_stop_button`。公共键设为 `-1` 关闭两种组合，单个功能键设为 `-1` 关闭对应组合；北通停止组合默认关闭。起步协调节点的接口可通过该节点的参数配置，表中为默认值。

在已构建并加载 ROS 环境的工作空间中启动：

```bash
ros2 launch mobile_joy_control joy_only.launch.py
```

可通过以下命令查看按键和停止请求：

```bash
ros2 topic echo /joy
ros2 topic echo /supervisor/system_command
```

键盘脚本使用 `i`、`,` 前后移动，`j`、`l` 转向；大写键支持侧向移动，`q/z`、`w/x`、`e/c` 调整速度。默认发布 `/cmd_vel`，使用时按实际底盘或速度选择器输入重映射。
