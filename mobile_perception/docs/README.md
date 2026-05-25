# mobile_perception 文档

本目录记录 `mobile_perception` 包的补充维护信息。包级使用说明仍以 `../README.md` 为准。

## mobile_control_node

`control/get_control_status` 服务中的取消状态分为两段：

- `CANCEL CONTROL`：已收到取消请求，节点正在按当前里程计反馈速度平滑停车。
- `CANCELED`：取消停车已完成；该状态与 `SUCCEED` 一样用于完成态反馈，查询后可回到空闲状态。

## mobile_control_action_node

`mobile_control_action_node` 是 Action 版底盘末端精控节点，接口为
`control/precise_control` (`nav_interfaces/action/PreciseControl`)。

Action goal 中的 `target_pose` 使用 `geometry_msgs/msg/PoseStamped`。节点接收到 goal 后在内部执行
`mobile_control_node` 同等的 TF 转换、PID 控制、速度滤波、到点判定和平滑取消逻辑，并直接发布
`control/cmd_vel`。

主要接口：

- Action `control/precise_control` (`nav_interfaces/action/PreciseControl`)。
- 订阅 `feedback/odom` (`nav_msgs/msg/Odometry`)。
- 发布 `control/cmd_vel` (`geometry_msgs/msg/Twist`)。

运行注意：

- 不订阅 `control/ctrl_pose`。
- 同一时间只接受一个 active goal。
- 客户端 cancel 后会平滑停车，完成后返回 `CANCELLED`。
- 不应与 `mobile_control_node` 同时控制同一个 `control/cmd_vel` 链路。

默认启动入口：

```bash
ros2 launch mobile_perception mobile_control_action.launch.py
```

默认参数文件：

- `config/mobile_control_action_params.yaml`

## obstacle_detector_node

`obstacle_detector_node` 是 C++ 前方障碍检测节点，来自旧工作区
`/home/lovebc/dexforce_ws/dexe_mobile_nav/mobile_perception` 的实现，并已接入当前分支的
`mobile_perception` 包。

节点订阅局部代价地图 `local_costmap/costmap`，根据 `base_link` 在代价地图坐标系下的最新 TF，
检查车体前方矩形区域是否存在代价值达到阈值的栅格。

主要接口：

- 发布 `obstacle_detector/has_obstacle` (`std_msgs/msg/Bool`)。
- 发布 `obstacle_detector/markers` (`visualization_msgs/msg/Marker`)。
- 发布 `obstacle_detector/detection_box` (`visualization_msgs/msg/Marker`)。

常用参数：

- `costmap_topic`
- `robot_base_frame`
- `front_distance`
- `left_width`
- `right_width`
- `obstacle_threshold`
- `detection_frequency`

运行示例：

```bash
ros2 run mobile_perception obstacle_detector_node
```
