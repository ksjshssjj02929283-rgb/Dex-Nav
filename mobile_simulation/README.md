# W1 mobile simulation

本目录提供 W1 的四轮全向与双轮差速 Gazebo 仿真。两种底盘只在运动学、
物理模型和 Nav2 运动模型上不同，对 navigation、SLAM 和上层应用暴露同一套接口。

## 启动入口

```bash
# 全向导航
ros2 launch w1_nav2 omni_sim.launch.py

# 差速导航
ros2 launch w1_nav2 diff_sim.launch.py

# 建图环境；随后在 SLAM 工作区启动 dexe Cartographer
ros2 launch w1_nav2 omni_sim_mapping.launch.py
ros2 launch w1_nav2 diff_sim_mapping.launch.py
ros2 launch cartographer_ros dexe_slam_2d.launch.py use_sim_time:=true map_name:=sim_map
```

差速导航仿真不再在启动命令后追加独立参数，统一修改
`w1_nav2/config/diff_sim.yaml`。该文件可配置前后 RGBD、前后激光雷达、IMU、Gazebo
界面、渲染后端、机器人出生位置、地图、Nav2 参数文件和地图对齐量；RViz、仿真时钟
以及独立 Nav2 进程模式属于固定启动行为。关闭某个 RGBD 时，也会停止对应点云转换节点。

全向仿真入口仍支持原有 launch 参数，两种底盘使用的 `params_file` 默认值按底盘类型区分。

四个导航/建图入口统一使用 `w1_nav2/rviz/diff_sim.rviz`，全向与差速不再维护
两份 RViz 显示配置。`swerve_robot_description/rviz/omni_display.rviz` 只保留为
机器人描述包的独立可视化资产，不参与 `w1_nav2` 仿真启动链。

## 对外接口契约

| 接口 | 类型/方向 |
|---|---|
| `/control/cmd_vel` | `geometry_msgs/msg/Twist`，底盘订阅 |
| `/feedback/odom` | `nav_msgs/msg/Odometry`，底盘发布 |
| `/laser_scan/front`、`/laser_scan/rear` | `sensor_msgs/msg/LaserScan`，发布 |
| `/imu/chassis` | `sensor_msgs/msg/Imu`，发布 |
| `/orbbec/{front,rear}/rgb` | RGB 图像，发布 |
| `/orbbec/{front,rear}/depth` | 深度图，发布，最大 5 m |
| `/orbbec/{front,rear}/camera_info` | 相机内参，发布 |
| `/orbbec/{front,rear}/pointcloud2_filtered` | RGBD 恢复并滤除 `base_link z <= 0.05 m` 后的点云 |

TF 主链统一为：

```text
map -> odom -> base_footprint -> base_link -> sensors/body
```

导航模式的 `map -> odom` 由 `nav2_bringup` 发布；建图模式不发布静态
`map -> odom`，由 SLAM 唯一负责。仿真源端直接使用标准接口，不在 Nav2 或
Cartographer 入口散落兼容 remap。

## 目录

- `swerve-sim-container/`：全向底盘模型、控制器和里程计。
- `diff-sim-container/`：差速底盘自包含模型与生成脚本。
- `w1_nav2/`：统一启动入口、Rviz 配置和接口契约测试。
- `aws-robomaker-small-warehouse-world/`：仓库世界与仿真地图。
