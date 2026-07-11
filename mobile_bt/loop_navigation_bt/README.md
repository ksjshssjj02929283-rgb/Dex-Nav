# loop_navigation_bt

`loop_navigation_bt` 是循环导航与回充联调包。它读取 JSON 点位，使用 Nav2 `/navigate_to_pose` 顺序导航，并把每次到点误差写入 `nav_error_log.txt`。

## 当前提供的运行模式

### 无回充循环导航

推荐用于导航精度和稳定性测试：

```bash
ros2 launch loop_navigation_bt loop_nocharging.launch.py
```

默认 `check_battery:=false`，不依赖底盘电量信息，完成 `navigation_points.json` 中 `rounds` 指定的轮数后退出。

启用低电量检查：

```bash
ros2 launch loop_navigation_bt loop_nocharging.launch.py check_battery:=true
```

启用后，电量低于 `30%` 时会取消当前导航，导航到 JSON 中第一个点，然后退出；该模式不会触发回充。

### 循环导航与自动回充

```bash
ros2 launch loop_navigation_bt loop_navigation_bt.launch.py
```

该入口加载 `loop_navigate_bt.xml`：正常时循环导航；电量低于 `30%` 时取消当前目标并调用 `/auto_charging` Action；电量恢复到 XML 当前配置的 `80%` 后执行退桩并继续导航。

使用前必须先启动 `auto_charging_bt` 的 Action server：

```bash
ros2 launch auto_charging_bt auto_charging_action.launch.py
```

注意：源码中的 XML 注释写有“直到 ≥90%”，但实际节点参数是 `threshold="80"`，运行行为以 `80%` 为准。

### 连续回充测试入口

包内还包含 `loop_charging_bt.xml` 和 `loop_charging_bt_node`，用于重复调用 `/auto_charging` Action。它会实际驱动机器人，不应用作普通导航测试。

## 行为树结构

无回充模式：

```text
主循环 ReactiveSequence
├── LoopNavigateFourPoints：按点位和轮数导航
└── BatteryCheck
    ├── 电量检查关闭或电量 ≥30%：继续
    └── 电量 <30%
        ├── CancelNavigation：取消当前导航
        └── NavigateToFirstPointAndStop：返回第一个点并退出
```

带回充模式：

```text
主循环 ReactiveSequence
├── LoopNavigateFourPoints：循环导航
└── BatteryCheck
    ├── 电量 ≥30%：继续
    └── 电量 <30%
        ├── CancelNavigation：取消当前导航
        ├── CallCharging：调用 /auto_charging Action
        ├── 等待电量 ≥80%
        └── Undocking：退桩后继续循环
```

## 导航点配置

配置文件：

```text
config/navigation_points.json
```

当前字段：

- `frame_id`：目标坐标系，当前为 `map`。
- `rounds`：完整遍历全部点位的轮数，当前为 `100`。
- `points`：点位数组；当前是 3 个点，不是固定“四个点”。
- `id`：日志中使用的点位名称。
- `position`、`orientation`：目标位置和四元数姿态。

节点通过 `ament_index_cpp::get_package_share_directory()` 读取安装目录中的配置：

```text
install/loop_navigation_bt/share/loop_navigation_bt/config/navigation_points.json
```

非 `--symlink-install` 时，修改源码 JSON 后需要由用户重新构建并重新 source，不能假设运行节点会自动读取源码文件。

也可以使用脚本辅助记录点位：

```bash
ros2 run loop_navigation_bt set_navigation_points.py
```

执行前应先查看脚本提示并确保定位正常。

## 导航误差日志

`LoopNavigateFourPoints` 启动时以截断方式创建：

```text
nav_error_log.txt
```

文件记录：

- 导航结果和耗时；
- 点序号、点 ID；
- 目标位姿与实际位姿；
- 目标坐标系下的纵向、横向和 yaw（航向角）误差；
- 距离误差；
- 是否进入期望阈值和接受阈值；
- 程序结束时的总体和分点统计。

日志使用相对路径，因此实际位置取决于进程工作目录。每次启动都会覆盖旧文件，需要保留历史结果时应提前复制或重命名。

默认优先从 `/navigation_goal_error` 获取 goal checker（到点判断器）最终误差；没有有效消息时回退到 TF 窗口采样。可以关闭该优先路径：

```bash
ros2 launch loop_navigation_bt loop_nocharging.launch.py use_goal_checker_error:=false
```

## ROS 接口

### Action 客户端

- `/navigate_to_pose`，类型 `nav2_msgs/action/NavigateToPose`。
- `/auto_charging`，类型 `chassis_interfaces/action/AutoCharging`，仅带回充模式使用。

### 订阅

- `/diagnostics_agg`：从 PowerBoard 诊断项读取 `battery_percentage`。
- `/navigation_goal_error`：读取导航结束时 goal checker 的误差诊断。

### TF

误差统计回退路径需要 `map` 到机器人基坐标系的 TF 可用。

## 目录结构

```text
loop_navigation_bt/
├── config/
│   ├── loop_navigate_bt.xml
│   ├── loop_nocharging_bt.xml
│   ├── loop_charging_bt.xml
│   └── navigation_points.json
├── include/loop_navigation_bt/
├── launch/
│   ├── loop_navigation_bt.launch.py
│   └── loop_nocharging.launch.py
├── scripts/set_navigation_points.py
└── src/
```

## 常见问题

### `/navigate_to_pose` 不可用

确认 Nav2 已启动：

```bash
ros2 action list | grep navigate_to_pose
```

### 带回充模式提示 `/auto_charging` 不可用

确认启动的是 Action server，不是同名 Trigger service：

```bash
ros2 action list | grep auto_charging
ros2 action info /auto_charging
```

### 没有电量数据

使用默认的无回充模式 `check_battery:=false`。关闭检查后程序不会因低电量自动返回，底盘应具备独立低电量保护。

### 找不到 `nav_error_log.txt`

从启动命令所在目录开始查找，或使用：

```bash
find . -name nav_error_log.txt
```

### 修改 XML/JSON 后行为没有变化

检查当前 source 的工作空间和安装目录文件。非软链接安装下，源码改动必须重新构建后才会复制到 `install/.../share`。

## 安全说明

本包会发送 Nav2 目标，带回充模式还会触发倒车对接和退桩。真实机器人测试时应清空运动区域、保持急停可用，并安排人员观察。
