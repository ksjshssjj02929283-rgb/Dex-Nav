# dexe_sensor_process

`dexe_sensor_process` 是 RGBD 传感器预处理包，当前核心节点是 `rgbd_to_pointcloud`。它订阅 RGBD 相机的深度图和 `CameraInfo`，反投影生成 `sensor_msgs/msg/PointCloud2` 点云，并发布原始点云和过滤后的点云，供 Nav2 costmap 或 RViz 使用。

## 主要文件

- `src/rgbd_to_pointcloud.cpp`：RGBD 深度图转点云、体素降采样、地面高度过滤的核心实现。
- `launch/rgbd_to_pointcloud.launch.py`：单相机通用启动入口。
- `launch/multi_camera.launch.py`：前后双 RGBD 相机同时启动入口。
- `launch/front_p2c.launch.py`：自动模式中前视 RGBD 点云转换入口。
- `launch/rear_p2c.launch.py`：自动模式中后视 RGBD 点云转换入口。

## 话题规则

节点通过 `input_topic` 决定输入话题前缀。如果没有显式指定，默认使用 `/orbbec/{camera_name}`。

- 输入深度图：`{input_topic}/depth`
- 输入相机内参：`{input_topic}/camera_info`
- 原始点云输出：`{input_topic}/pointcloud2_raw`
- 过滤点云输出：`{input_topic}/pointcloud2_filtered`
- LaserScan 输出参数：`output_topic_laserscan`

当前代码保留了点云转 `LaserScan` 的实现，但发布路径仍处于注释状态，实际稳定输出是 `pointcloud2_raw` 和 `pointcloud2_filtered`。

## 点云生成流程

1. 订阅深度图和最近一次 `CameraInfo`。
2. 使用 `fx`、`fy`、`cx`、`cy` 将深度像素反投影为相机坐标系下的 `x/y/z` 点。
3. 深度单位按毫米转米，当前深度截断距离为 `3.0 m`。
4. 为降低计算量，反投影时按固定像素步长采样。
5. 发布 `pointcloud2_raw`，该点云不做地面过滤。
6. 对 raw 点云做体素降采样和地面高度过滤后，发布 `pointcloud2_filtered`。

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
- `ground_filter_min_height`：低于该高度的点认为是地面或近地噪声并剔除，默认 `0.05 m`。
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
- `pointcloud2_raw` 当前会随深度回调发布；如后续需要降低带宽或 CPU 压力，可增加参数控制是否发布 raw，或仅在有订阅者时发布。
