# loop_navigation_bt

## 1. 功能

本包从当前地图的 `assets/map_assets.json` 读取导航点坐标，再按
`config/navigation_points.json` 指定的顺序，通过 Nav2 `/navigate_to_pose`
循环导航，并记录到点误差、导航耗时和移动距离。

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

电量低于阈值后，机器人返回导航计划中的第一个点并退出，不执行回充。

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

然后确认当前地图的 `assets/map_assets.json` 中存在且仅存在一个 `type=CHARGING` 点。

以上两项检查通过后，启动循环导航：

```bash
ros2 launch loop_navigation_bt loop_navigation_bt.launch.py
```

注意：文件存在且 JSON 格式正确，只能证明地图资产可读取，不能证明回充点位置和朝向正确；首次使用或充电桩位置变化后仍需现场确认。

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
2. `navigation.map_state_wait_timeout_sec`：启动时等待当前地图状态的超时时间。
3. `navigation.action_server_wait_timeout_sec`：等待 Nav2 Action server 的超时时间。
4. `navigation.odom_topic`：移动距离统计使用的里程计话题。
5. `navigation.odom_max_increment_m`：单帧里程计最大有效增量。
6. `battery.low_threshold`：触发低电量流程的阈值。
7. `battery.resume_threshold`：充电后允许恢复导航的阈值。
8. `charging.force`：是否强制执行完整回充流程。
9. `undocking.duration_sec`：退桩持续时间。
10. `undocking.linear_speed`：退桩线速度。
11. `goal_error_stats.summary_interval_rounds`：每完成多少个完整轮次输出一次区间误差统计并重印明细表头，默认 `5`。
12. `goal_error_stats.desired_position_tolerance`：高精度阈值，默认 `0.01` 米；横向和纵向误差必须分别满足。
13. `goal_error_stats.accept_position_tolerance`：接收精度阈值，默认 `0.02` 米；横向和纵向误差必须分别满足。
14. `goal_error_stats.sample_count`：TF 回退时尝试采集的样本数，默认 `14`。
15. `goal_error_stats.trim_count_per_side`：每项误差排序后分别从最小端和最大端去除的样本数，默认每端 `2` 个。
16. `goal_error_stats.sample_interval_sec`：相邻 TF 样本的时间间隔，默认 `0.1` 秒。
17. `goal_error_stats.use_goal_checker_error`：是否优先采用 GoalChecker 发布的单次最终误差。

## 4. 导航点配置

导航计划文件：

```text
config/navigation_points.json
```

该文件只负责三件事：

1. `rounds`：完整遍历全部点位的轮数；设为 `0` 表示持续运行。
2. `points` 数组的先后顺序：机器人实际执行的点位顺序。
3. `id` 和 `wait_time_sec`：点位名称以及到点或失败后的停留时间，单位为秒。

示例：

```json
{
  "rounds": 10,
  "points": [
    {
      "id": "point_1",
      "wait_time_sec": 2.0
    },
    {
      "id": "point_2",
      "wait_time_sec": 5.0
    }
  ]
}
```

启动时节点订阅 `/map_manager/map_state`，按下面的路径读取当前地图资产：

```text
<map_folder>/<map_name>/assets/map_assets.json
```

导航计划中的每个 `id` 必须与 `map_assets.json` 的 `points[].name` 完全一致。
坐标 `x`、`y` 和朝向 `yaw` 只从地图资产读取；计划文件出现坐标字段时程序会拒绝启动。
程序启动后会持续监听地图版本。如果运行过程中地图名称、资产路径或 `epoch`
发生变化，程序会取消当前行为树并提前停止，防止把旧地图坐标发送给新地图。
确认新地图定位正常后，需要重新启动本程序。

新增或修改坐标应使用 RViz 的地图资产工具，或使用
`/map_asset_manager/sample_point`、`/map_asset_manager/add_point` 等地图资产接口。
本包不再提供 `set_navigation_points.py`。

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

文件面向现场测试人员，逐次记录轮次、点位、成功或失败、导航耗时、纵向误差、
横向误差、角度误差、本次行驶距离和累计距离，所有数值都在表头标明单位。纵向和
横向误差是相对目标朝向的分量，表格中均显示误差绝对值。测试结束或程序退出时
追加简洁汇总；未获得有效数据时显示“未获取”，不会用 `0` 冒充有效误差。
明细和汇总表会按终端显示宽度自动补齐空格：中文按双宽字符处理，点位名称按实际
最长内容扩展，文字列居中或左对齐，数值列右对齐。
默认每完成 5 个完整轮次，文件会输出从第 1 轮截至当前轮次的总体和各点累计汇总，
例如第 5 轮统计第 1～5 轮，第 10 轮统计第 1～10 轮。汇总分为成功和失败两张表，
每张表分别包含次数占比、平均耗时、平均纵向误差、平均横向误差、平均角度误差、
高精度占比和接收精度占比；所有平均值和精度占比只使用该结果类型的数据，
成功与失败样本不会混合计算。汇总后在下一轮明细前重新打印表头。平均误差采用
绝对值；两档精度都要求横向和纵向误差分别不超过对应阈值。统计间隔及两档阈值均在
`loop_navigation_bt_params.yaml` 中配置；无效误差不参与平均误差和精度占比计算。
无法取得 GoalChecker 最终误差或关闭其优先开关时，程序回退到 TF 连续采样。默认
尝试采样 14 次，每项误差分别排序并从最小端和最大端各去除 2 个，使用剩余 10 个
样本求平均。实际有效样本不足时会自动减少剔除数量，保证至少保留 1 个样本。
汇总中的测试状态会明确区分“正常完成”“提前停止”和“测试失败”，目标被拒绝、
意外取消、地图切换或配置错误不会再被报告成“全部成功”。

其中“导航耗时”只统计 Nav2 执行当前目标的时间，不包含点位配置的停留时间。

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
5. 当前地图订阅：`/map_manager/map_state`（Transient Local，可靠）。
6. TF 回退：需要 `map` 到机器人基坐标系的变换。

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

### 7.4 检查当前地图和点位

```bash
ros2 topic echo --once /map_manager/map_state
ros2 service call /map_asset_manager/get_all_nav_points nav_interfaces/srv/GetAllNavPoints "{}"
```

### 7.5 配置修改未生效

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
