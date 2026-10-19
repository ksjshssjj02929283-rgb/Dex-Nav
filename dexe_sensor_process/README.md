# dexe_sensor_process

`dexe_sensor_process` 是 RGBD 传感器预处理包，当前核心节点是 `rgbd_to_pointcloud`。节点只从深度图和 `CameraInfo` 重建 `sensor_msgs/msg/PointCloud2`，随后发布原始点云和过滤点云，供 Nav2 costmap 或 RViz 使用。

## 主要文件

- `src/rgbd_to_pointcloud.cpp`：唯一 ROS 节点，读取公共参数、共享 TF，并管理相机处理通道。
- `src/camera_pipeline.hpp`：单相机订阅、深度反投影、点云/Scan 发布与独立跨帧状态。
- `src/obstacle_extractor.hpp` / `.cpp`：`ObstacleExtractor` 多态接口及 `HeightFilterExtractor`、`UVDisparityExtractor` 实现。
- `launch/rgbd_to_pointcloud.launch.py`：单相机通用启动入口。
- `launch/multi_camera.launch.py`：在一个 `rgbd_to_pointcloud` 节点内处理前后双 RGBD 相机。
- `launch/front_p2c.launch.py`：自动模式中前视 RGBD 点云转换入口。
- `launch/rear_p2c.launch.py`：自动模式中后视 RGBD 点云转换入口。

## 单节点双相机与算法选择

```bash
ros2 launch dexe_sensor_process multi_camera.launch.py
# 只启用前相机
ros2 launch dexe_sensor_process multi_camera.launch.py enable_rear:=false
```

双相机入口只创建一个 ROS 节点。前后相机保持各自的输入/输出话题，不拼接点云，也不等待彼此同步。
两条通道共享 TF Buffer 和公共算法参数，但分别拥有 CameraInfo、外参缓存、分帧计数、Scan 时间基线及算法实例。
UV 的地面拟合、随机数与查找表缓冲也按相机隔离。相机回调在单线程 executor 中串行执行；同一节点的 TF 订阅使用专用线程，支持带超时的 TF 查询。

- `cameras: [front, rear]` 启用两条通道；相机名称必须非空且不重复，只允许字母、数字和下划线。
- 多相机话题可通过 `front.input_topic`、`rear.input_topic` 及各自的
  `<camera>.output_topic_raw` / `output_topic_filtered` / `output_topic_laserscan` 覆盖。
- `cameras` 默认空列表，兼容原有 `camera_name` 和无前缀话题参数；单相机 launch 入口仍然可用。
  同时运行 `front_p2c.launch.py` 与 `rear_p2c.launch.py` 仍会产生两个节点，双相机部署应迁移到 `multi_camera.launch.py`。
- `obstacle_extraction_method` 在启动时选择策略：`height_filter` 先重建 raw 点云，再体素降采样和高度过滤；
  `uv_disparity` 直接从深度图提取障碍物，无 raw 订阅者时省略 raw 重建。
- `auto` 仍读取 `hardware_path`：硬件版本 `<= v0.24` 使用高度法，更高版本使用 UV；读取失败回退高度法。
  不支持的算法名称会使启动失败，避免配置拼写错误被忽略。
- 双相机 launch 的 `use_sim_time`、`depth_max_range`、`ground_filter_min_height` 参数可覆盖 YAML；
  不传时保留 YAML 值。`enable_front` / `enable_rear` 都为 false 时不启动处理节点。

例如自定义话题配置：

```yaml
/**:
  ros__parameters:
    cameras: [front, rear]
    obstacle_extraction_method: height_filter
    front.input_topic: /orbbec/front
    rear.input_topic: /orbbec/rear
```

## 话题规则

节点通过 `input_topic` 决定输入话题前缀。如果没有显式指定，默认使用 `/orbbec/{camera_name}`。

- 输入深度图：`{input_topic}/depth`
- 输入相机内参：`{input_topic}/camera_info`
- 原始点云输出：`{input_topic}/pointcloud2_raw`
- 过滤点云输出：`{input_topic}/pointcloud2_filtered`
- LaserScan 输出参数：`output_topic_laserscan`

当 `enable_laserscan=true` 时，节点把过滤后的点云转换成 `LaserScan` 并发布；关闭时发布
`pointcloud2_filtered`。原始点云 `pointcloud2_raw` 在两种模式下都会发布。

每处理一帧深度图都会发布一次 `LaserScan`：即使过滤后点云为 0 点（空旷视野，没有检测到
障碍），也会发布一帧 `ranges` 全为 `inf` 的 `LaserScan`，并沿用该深度图的采集时间戳。
因此话题节奏与输入深度图一致，不会因为“这一帧看不到障碍”而停更。下游
（`collision_monitor`、传感器监测）据此区分“看不到障碍”（有效空帧）和“相机/转换节点
没有数据”（话题真的停更），不会把空视野误判为数据源失效。

## 输入帧二分频与时间语义

- `division_mode`：默认 `true`。开启后严格按“处理一帧、跳过一帧”工作；关闭后处理每个满足前置条件的输入帧。
- 该参数不是目标频率控制。输入稳定为 15 Hz 时，开启后的理论处理频率约为 7.5 Hz，而不是 10 Hz。
- 节点不改写输入消息的 `header.stamp`；点云和 `LaserScan` 继续沿用源深度图时间戳。
- `LaserScan.scan_time` 使用相邻两次成功生成 Scan 的源时间戳差值。第一帧、零时间戳或时间戳不单调时为 `0.0`，并重新建立时间基线。

如后续需要稳定的独立输出节拍，应另行实现 Timer + latest-frame 模式；它不属于当前
`division_mode` 的语义。

## 点云生成流程

1. 订阅深度图和最近一次 `CameraInfo`。
2. 使用 `fx`、`fy`、`cx`、`cy` 将深度像素反投影为相机坐标系下的 `x/y/z` 点。
3. 根据 encoding 将实机 `16UC1/mono16` 毫米深度或 Gazebo `32FC1` 米深度转换为米；`depth_max_range` 默认值为 `3.0 m`，W1 仿真配置为 `5.0 m`。
4. 为降低计算量，反投影时按固定像素步长采样。
5. 发布 `pointcloud2_raw`，该点云不做地面过滤。
6. 高度法对 raw 点云做体素降采样和地面高度过滤；UV 法直接从深度图提取障碍物。按 `enable_laserscan` 发布 Scan 或过滤点云。

节点不接受已有 PointCloud2 作为输入。仿真与真机使用同一条 `Depth + CameraInfo -> PointCloud2` 重建路径；Gazebo 发布的 `/orbbec/{front,rear}/points` 仅可作为仿真测量对照，不属于本包输入接口。

## 坐标系与地面过滤

地面过滤的关键点是：**内部用 `base_link` 高度判断地面，但最终发布仍保持原始相机坐标系**。

具体流程：

```text
相机 frame 下的 raw 点云
└── 体素降采样
    ├── 保留一份相机 frame 点云，用于最终发布
    └── 复制一份临时转到 ground_filter_frame，默认 base_link
        └── 根据 base_link 下的 z 高度判断是否保留
            └── 通过高度判断后，发布对应的相机 frame 点
```

这样做的原因：

- 用 `base_link` 的 `z` 判断高度，比直接用相机坐标系 Y 轴更符合“离地高度”的物理意义。
- 最终仍发布原始相机 frame，例如 `chassis_depth_front_link`，可以让 Nav2 costmap 按真实传感器来源处理 marking 和 clearing。
- 不建议把 filtered 点云直接发布成 `base_link` frame，否则 costmap 可能把传感器来源理解成机器人中心，影响射线清除和障碍物标记。

## 地面过滤参数

- `ground_filter_frame`：地面过滤使用的临时判断坐标系，默认 `base_link`。
- `ground_filter_min_height`：小于等于该高度的点认为是地面或近地噪声并剔除，默认 `0.05 m`。
- `ground_filter_max_height`：高于该高度的点不作为近地障碍输入，默认 `1.5 m`。

调参建议：

- 地面残留较多时，可以适当调高 `ground_filter_min_height`，例如 `0.08` 或 `0.10`。
- 低矮障碍物被漏检时，应降低 `ground_filter_min_height`。
- 高处点或远处噪声进入 costmap 时，可以适当降低 `ground_filter_max_height`。

注意：旧的 `ground_filter_threshold` 相机 Y 轴阈值已经移除，后续不要再通过相机坐标系 Y 轴调地面过滤。

## 与 Nav2 Costmap 的关系

Nav2 costmap 应订阅过滤后的点云：

- `/orbbec/front/pointcloud2_filtered`
- `/orbbec/rear/pointcloud2_filtered`

判断过滤结果时应直接检查上述 topic。若 costmap 配置使用
`nav2_costmap_2d::ObstacleLayer`，则不会发布 `voxel_grid` 或
`voxel_marked_cloud`；RViz 中遗留的 VoxelGrid display 不能代表当前 filtered 点云。

修改点云输出话题时，需要同步检查：

- `mobile_navigation/nav2_bringup/params/w1_nav2_params.yaml`
- `mobile_navigation/nav2_bringup/params/w1_nav2_rgbd_params.yaml`

常用检查命令：

```bash
ros2 topic info -v /orbbec/front/pointcloud2_filtered
ros2 topic echo /orbbec/front/pointcloud2_filtered --once --field header
ros2 topic echo /orbbec/front/pointcloud2_filtered --once --field fields
ros2 topic echo /orbbec/front/pointcloud2_filtered --once --field width
ros2 param get /local_costmap/local_costmap obstacle_layer.orbbec_front_marked.topic
```

期望 filtered 点云满足：

```text
Publisher count: 1
fields: x, y, z
width: 大于 0 时表示当前有有效点
frame_id: 相机深度 frame，例如 chassis_depth_front_link
```

## 维护注意

- `flag_voxel_filter` 当前实际生效。
- `flag_mls_filter`、`flag_sor_filter`、`mls_*`、`sor_*` 参数目前有声明和传入，但源码未真正执行 MLS/SOR 过滤。
- TF 失败时，节点会跳过本帧 `pointcloud2_filtered` 发布，避免 costmap 被无效空点云反复清除。
- 如果 TF 正常但高度过滤后确实没有有效障碍点，节点会发布带 `x/y/z` 字段的合法空点云，避免 Nav2 costmap 因 `Field x does not exist` 崩溃。
- `pointcloud2_raw` 仅在有订阅者时发布；所有输出均无订阅者时跳过本次处理。
