# mobile_perception 文档

本目录记录 `mobile_perception` 包的补充维护信息。包级使用说明仍以 `../README.md` 为准。

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
