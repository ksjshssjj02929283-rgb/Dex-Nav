# mobile_perception

`mobile_perception` 是 `dexe_mobile_nav` 中的 ROS 2 功能包，包含底盘直控、AprilTag 检测、前方障碍检测和充电位姿保存等功能。包使用 `ament_cmake` 构建，主要节点使用 C++ 实现，同时保留部分 Python 辅助脚本。

## 目录结构

```text
dexe_mobile_nav/mobile_perception/
├── config/                         # 参数和传感器配置
│   └── mobile_control_action_params.yaml
├── launch/                         # ROS 2 启动文件
│   └── mobile_control_action.launch.py
├── src/                            # C++ 节点源码
├── include/mobile_perception/      # C++ 头文件
├── scripts/                        # Python 辅助与调试脚本
├── test/                           # 底盘速度响应与直控效果测试
│   ├── chassis_test_common.py
│   ├── precise_straight_action_test.py
│   ├── precise_rotation_action_test.py
│   └── README.md
├── images/                         # 图像资源
├── CMakeLists.txt
└── package.xml
```

## 主要功能

| 节点                           | 功能                                                               |
| ------------------------------ | ------------------------------------------------------------------ |
| `mobile_control_action_node` | 通过 Action 接收目标位姿，执行直线运动或原地转向，并输出底盘速度。 |
| `apriltag_detection_node`    | 按服务启动或停止 AprilTag 检测，发布标签位姿。                     |
| `apriltag_alignment_tf_node` | 发布 AprilTag 原始、对齐和目标位姿 TF，便于调试与目标生成。        |
| `obstacle_detector_node`     | 根据局部代价地图检测车体前方区域内的障碍物。                       |

`mobile_control_node` 是保留的旧版底盘直控节点。当前建议使用 Action 接口的 `mobile_control_action_node`，不要让两个节点同时向 `/control/cmd_vel` 发布速度。

## 底盘直控

`mobile_control_action_node` 的主要接口如下：

| 类型   | 名称                         | 说明                                                |
| ------ | ---------------------------- | --------------------------------------------------- |
| Action | `/control/precise_control` | 接收`nav_interfaces/action/PreciseControl` 目标。 |
| 订阅   | `/feedback/odom`           | 获取底盘实际线速度和角速度。                        |
| 发布   | `/control/cmd_vel`         | 输出底盘线速度和角速度。                            |
| TF     | `odom -> base_link`        | 默认用于锁定目标并计算运动误差。                    |

节点根据目标位移自动选择直线模式或原地转向模式。同一时间只执行一个目标；取消目标时会先平滑停车，再返回取消结果。

### 配置文件

底盘直控参数统一配置在：

```text
dexe_mobile_nav/mobile_perception/config/mobile_control_action_params.yaml
```

主要参数分为以下几类：

- 公共参数：控制频率、参考坐标系、到位确认时间、速度和加减速度上限。
- 直线参数：速度曲线、升速时间、减速距离、停车速度阈值和直线 PID。
- 原地转向参数：最大角速度、升速时间、减速角度范围和制动系数。
- 到位参数：直线位置容差、航向容差和停稳速度阈值。

修改参数后需要重启节点才能生效。运行时可使用以下命令确认实际参数：

```bash
ros2 param get /mobile_control_action_node control_reference_frame
ros2 param get /mobile_control_action_node xy_tolerance
```

## 底盘位姿发布

`pub_current_position.py` 订阅 `/localization/odometry/filtered`，每收到一帧有效
Odom 就发布一帧 `/feedback/chassis_pose`，发布频率跟随融合 Odom（当前为 25 Hz），
不再通过 TF 查询位姿。

## 使用方式

### 启动底盘直控

```bash
cd dexe_mobile_nav
source install/setup.bash
ros2 launch mobile_perception mobile_control_action.launch.py
```

指定参数文件：

```bash
ros2 launch mobile_perception mobile_control_action.launch.py \
  params_file:="$(pwd)/mobile_perception/config/mobile_control_action_params.yaml"
```

### 发送直线目标

以下命令表示以当前 `base_link` 为起点向前运动 `1.0 m`：

```bash
ros2 action send_goal /control/precise_control \
  nav_interfaces/action/PreciseControl \
  "{target_pose: {header: {frame_id: base_link}, pose: {position: {x: 1.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

向后运动时，将 `position.x` 设置为负值。

### 发送原地转向目标

以下命令表示原地逆时针转动 `90°`：

```bash
ros2 action send_goal /control/precise_control \
  nav_interfaces/action/PreciseControl \
  "{target_pose: {header: {frame_id: base_link}, pose: {orientation: {z: 0.707107, w: 0.707107}}}}" \
  --feedback
```

目标角度的正值表示逆时针，负值表示顺时针。原地转向开始后，节点会锁定旋转模式和目标方向。

### 启动其他节点

```bash
# AprilTag 检测
ros2 run mobile_perception apriltag_detection_node
ros2 service call /start_apriltag_detection std_srvs/srv/Trigger "{}"

# AprilTag 对齐与目标 TF
ros2 run mobile_perception apriltag_alignment_tf_node

# 前方障碍检测
ros2 run mobile_perception obstacle_detector_node

```

## 测试脚本

`mobile_perception/test` 目录中的三个脚本彼此独立，可分别运行：

| 脚本                                | 功能                                                 |
| ----------------------------------- | ---------------------------------------------------- |
| `chassis_test_common.py`          | 发送阶跃、梯形或正弦速度，统计底盘对速度指令的响应。 |
| `precise_straight_action_test.py` | 测试直线底盘直控的耗时、最终误差、横向偏移和过冲。   |
| `precise_rotation_action_test.py` | 测试原地转向底盘直控的耗时、最终误差和过冲。         |

运行前先加载 ROS 2 和当前工作空间环境：

```bash
cd dexe_mobile_nav
source install/setup.bash
cd mobile_perception/test
```

速度阶跃响应示例：

```bash
python3 chassis_test_common.py \
  --waveform step \
  --linear 0.1 \
  --angular 0 \
  --duration 10
```

直线和原地转向的单次可视化测试：

```bash
python3 precise_straight_action_test.py --distance-m 0.5 --mode visual
python3 precise_rotation_action_test.py --angle-deg 90 --mode visual
```

压测模式可设置重复次数，默认在正负方向之间交替执行：

```bash
python3 precise_straight_action_test.py \
  --distance-m 0.5 \
  --mode stress \
  --test-count 100
```

可视化模式会在 `test/results` 中生成原始数据、统计结果、中文报告和曲线图；压测模式只输出逐轮结果并生成 `stress_report.txt`。完整参数和更多示例见 `dexe_mobile_nav/mobile_perception/test/README.md`，也可运行：

```bash
python3 <脚本名称> --help
```

## 运行注意事项

- 启动前确认 `/feedback/odom` 和 `odom -> base_link` TF 正常更新。
- 实机运行前确认急停可用、运动区域无障碍物，并检查速度上限。
- `/control/cmd_vel` 同一时间只应有一个有效控制来源，避免多个节点竞争输出。
- 修改控制参数时一次只调整一类参数，并记录调整前后的实机结果。
