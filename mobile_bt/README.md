# mobile_bt 使用说明

本目录主要包含以下两个 Behavior Tree（行为树）功能包：

- `auto_charging_bt`：可选导航到地图中的唯一回充点，并通过 AprilTag 完成倒车对接和充电确认。
- `loop_navigation_bt`：按照 JSON 中配置的点位和轮数执行循环导航测试，可选择是否检测低电量。

以下命令默认在机器人 ROS 2、Nav2、定位和相关节点已经启动，并且当前终端已经 `source` 对应工作空间的情况下执行。

## 一、自动回充

### 1. 设置地图回充点

在当前地图的 `assets/map_assets.json` 中配置唯一一个 `type=CHARGING` 的点位，坐标格式为
`pose.x`、`pose.y` 和 `pose.yaw`。可以使用 `map_asset_manager` 服务或 RViz 地图资产工具配置，
不再使用 `/save_charging_pose` 或独立的 `charging_pose.json`。

回充流程以 `/command_board/navigation_backend` 为唯一选择来源：

- `data: nav2`：校验唯一回充点并先执行 Nav2 导航。
- `data: path_navigation`：调度侧负责前置移动，回充直接进入二维码精确对接。

### 2. 执行一次回充

确认 `/auto_charging` Action server、Nav2、AprilTag 感知和底盘控制均已启动后，执行：

```bash
ros2 action send_goal /auto_charging chassis_interfaces/action/AutoCharging "{force: true}"
```

如需在当前终端持续查看 Action feedback（反馈）和最终结果，可以加 `--feedback`：

```bash
ros2 action send_goal /auto_charging chassis_interfaces/action/AutoCharging "{force: true}" --feedback
```

回充整体流程为：

```text
读取充电状态
├── 已在充电：直接成功
├── 已连接但未充电：先退桩
└── 未连接：继续正常回充
    └── 按 navigation_backend 选择是否导航到唯一 CHARGING 点
        └── 启动 AprilTag 检测
            └── 根据 apriltag_target_pose 倒车对接
                └── 检查充电状态并结束
```

运行过程中机器人会调用 Nav2 导航并向 `control/cmd_vel_nav` 发布速度。真实机器人周围必须留人观察，并确保急停可用。

## 二、导航点循环测试

### 1. 配置导航点和测试轮数

循环导航点位位于：

```text
loop_navigation_bt/config/navigation_points.json
```

文件中的主要字段为：

- `rounds`：循环轮数。
- `points`：按顺序执行的导航计划，每个点包含 `id` 和 `wait_time_sec`。

导航点的坐标和姿态不写在该 JSON 中，而是根据 `id` 从当前地图的
`assets/map_assets.json` 读取；坐标统一使用 `map` 坐标系。

如果运行的是已安装工作空间，程序实际读取的是安装目录下的：

```text
install/loop_navigation_bt/share/loop_navigation_bt/config/navigation_points.json
```

非 `--symlink-install` 构建时，修改源码中的 JSON 后需要由用户重新构建并重新 `source install/setup.bash`，否则运行时仍可能读取旧配置。

### 2. 启动不回充的循环导航测试

```bash
ros2 launch loop_navigation_bt loop_nocharging.launch.py
```

该 launch 默认参数为：

- `check_battery:=false`：不检测低电量，完成 `rounds` 指定的轮数后退出。
- `use_goal_checker_error:=true`：优先从 `/navigation_goal_error` 获取最终误差；读取不到时回退到 TF 采样。

如需启用低电量检测：

```bash
ros2 launch loop_navigation_bt loop_nocharging.launch.py check_battery:=true
```

启用后，当电量低于行为树中配置的 `30%` 阈值时，程序会取消当前导航，导航到配置中的第一个点，然后退出；此 launch 不会触发自动回充。

### 3. `nav_error_log.txt`

循环导航节点启动时会在进程的当前工作目录创建或覆盖：

```text
nav_error_log.txt
```

它记录每次导航的结果、耗时、点位编号和 ID、纵向误差、横向误差、航向误差、
本次行驶距离、累计行驶距离，以及误差是否落入期望阈值和接受阈值。程序正常结束或收到
ROS shutdown 时，还会在文件末尾写入统计结果。

注意：该文件使用相对路径创建，所以实际位置取决于启动进程的工作目录。可以在启动前进入希望保存日志的目录，例如：

```bash
cd ~/logs
ros2 launch loop_navigation_bt loop_nocharging.launch.py
```

测试结束后查看：

```bash
less nav_error_log.txt
```

每次启动都会以截断方式打开该文件，旧内容会被覆盖；需要保留历史测试结果时，请在下一次启动前复制并重命名。

## 三、常用检查命令

```bash
# 检查回充相关服务和 Action
ros2 action list | grep auto_charging

# 检查 Nav2 导航 Action
ros2 action list | grep navigate_to_pose

```

如果 `/auto_charging` 不存在，请确认 `auto_charging_bt` 的 Action launch 是否已启动。
