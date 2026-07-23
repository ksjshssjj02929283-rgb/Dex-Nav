# loop_navigation_bt

## 1. 功能

本包读取 JSON 导航点，通过 Nav2 `/navigate_to_pose` 循环导航，并记录到点误差、导航耗时和移动距离。

支持两种模式：

1. 无回充循环导航：用于导航精度和稳定性测试。
2. 循环导航与自动回充：低电量时调用 `/auto_charging`，充电完成后退桩并继续导航。

## 2. 启动方式

### 2.1 无回充模式

默认不检查电量：

```bash
ros2 launch loop_navigation_bt loop_nocharging.launch.py
```

启用低电量检查：

```bash
ros2 launch loop_navigation_bt loop_nocharging.launch.py check_battery:=true
```

电量低于阈值后，机器人返回 JSON 中第一个点并退出，不执行回充。

### 2.2 自动回充模式

启动循环导航前，先确认 `/auto_charging` Action（动作服务）已经存在：

```bash
ros2 action list -t | grep '^/auto_charging '
```

正常情况下应输出：

```text
/auto_charging [chassis_interfaces/action/AutoCharging]
```

如果没有输出，再启动回充 Action server：

```bash
ros2 launch auto_charging_bt auto_charging_action.launch.py
```

然后确认已经采集回充点，并且文件是有效 JSON：

```bash
test -s /home/dexforce/w1/install/mobile_perception/lib/mobile_perception/charging_pose.json && \
  python3 -m json.tool /home/dexforce/w1/install/mobile_perception/lib/mobile_perception/charging_pose.json
```

如果文件不存在、为空或解析失败，需要先将机器人移动到充电桩前置点，再采集回充点：

```bash
ros2 service call /save_charging_pose std_srvs/srv/Trigger "{}"
```

以上两项检查通过后，启动循环导航：

```bash
ros2 launch loop_navigation_bt loop_navigation_bt.launch.py
```

注意：文件存在且 JSON 格式正确，只能证明回充点已经保存，不能证明点位位置和朝向正确；首次使用或充电桩位置变化后仍需现场确认。

执行流程：

1. 等待有效电量。
2. 按顺序循环导航。
3. 低电量时取消当前目标并执行回充。
4. 电量达到恢复阈值后退桩。
5. 从原点位序号继续导航。

## 3. 参数配置

参数文件：

```text
config/loop_navigation_bt_params.yaml
```

主要参数：

1. `loop.tick_rate_hz`：行为树 tick 频率。
2. `navigation.wait_time_sec`：点位未单独配置时的默认等待时间。
3. `navigation.action_server_wait_timeout_sec`：等待 Nav2 Action server 的超时时间。
4. `navigation.odom_topic`：移动距离统计使用的里程计话题。
5. `navigation.odom_max_increment_m`：单帧里程计最大有效增量。
6. `battery.low_threshold`：触发低电量流程的阈值。
7. `battery.resume_threshold`：充电后允许恢复导航的阈值。
8. `charging.force`：是否强制执行完整回充流程。
9. `undocking.duration_sec`：退桩持续时间。
10. `undocking.linear_speed`：退桩线速度。
11. `goal_error_stats.*`：误差阈值、采样次数和统计间隔。

## 4. 导航点配置

点位文件：

```text
config/navigation_points.json
```

主要字段：

1. `frame_id`：目标坐标系，通常为 `map`。
2. `rounds`：完整遍历全部点位的轮数。
3. `id`：点位名称，用于日志标识。
4. `wait_time_sec`：当前点位的等待时间，可选；未配置时使用 YAML 默认值。
5. `position`：目标位置。
6. `orientation`：目标四元数姿态。

示例：

```json
{
  "frame_id": "map",
  "rounds": 10,
  "points": [
    {
      "id": "point_1",
      "wait_time_sec": 2.0,
      "position": {"x": 1.0, "y": 2.0, "z": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    }
  ]
}
```

## 5. 日志与统计

### 5.1 终端日志

1. 包内节点日志统一使用 UTC+8 时间。
2. 导航开始时输出点位序号、ID 和目标坐标。
3. 到位后输出目标坐标、实际坐标和到点误差。
4. 电量日志输出当前电量和判断阈值。

### 5.2 误差文件

运行时会在进程工作目录创建并覆盖：

```text
nav_error_log.txt
```

文件记录导航结果、耗时、点位信息、目标与实际位姿、纵向/横向/yaw 误差及汇总统计。

### 5.3 移动距离

1. 本次移动：当前 Nav2 目标执行期间的里程计增量和。
2. 累计移动：程序启动后所有导航目标的增量和，切换轮次不会清零。
3. 回充等待和退桩不计入导航距离。
4. 该数值会受到里程计噪声、重复目标和取消重试影响，不应直接视为绝对准确的真实里程。

## 6. ROS 接口

1. Action 客户端：`/navigate_to_pose`。
2. 回充 Action 客户端：`/auto_charging`。
3. 电量订阅：`/diagnostics_agg`。
4. 误差订阅：`/navigation_goal_error`。
5. TF 回退：需要 `map` 到机器人基坐标系的变换。

## 7. 常用检查

### 7.1 检查 Nav2 Action

```bash
ros2 action list | grep navigate_to_pose
```

### 7.2 检查回充 Action

```bash
ros2 action info /auto_charging
```

### 7.3 检查电量与里程计

```bash
ros2 topic echo /diagnostics_agg --once
ros2 topic info /feedback/odom --verbose
```

### 7.4 配置修改未生效

节点读取安装目录中的配置：

```text
install/loop_navigation_bt/share/loop_navigation_bt/config/
```

非 `--symlink-install` 时，修改 YAML、XML 或 JSON 后需要重新构建并重新 source 工作空间。

## 8. 安全说明

1. 启动前确认定位和 Nav2 状态正常。
2. 真实机器人测试时保持运动区域无障碍。
3. 保持急停可用并安排人员观察。
4. 自动回充模式会直接控制对接和退桩，首次测试应使用低速并保持安全距离。
