# nav2_costmap_2d 整理说明

## 1. 这个功能包做什么

`nav2_costmap_2d` 是 Navigation2 中二维代价地图的核心包，负责把静态地图、实时感知障碍、障碍膨胀、区域禁行、区域限速等信息汇总成规划器和控制器使用的 costmap。

对这个仓库里的实现，最值得先记住的是两条主线：

- `plugins`：普通 layer，负责往代价地图里写障碍、空闲区、膨胀结果等。
- `filters`：costmap filter，基于一张额外的 `OccupancyGrid mask` 对最终 costmap 或机器人行为施加区域规则。

从 `src/layered_costmap.cpp` 可以看到执行顺序：

1. 先收集所有 `plugin` 和 `filter` 的更新边界。
2. 没有 filter 时，plugin 直接顺序更新 `combined_costmap_`。
3. 有 filter 时，plugin 先更新 `primary_costmap_`。
4. 再把 `primary_costmap_` 拷贝到 `combined_costmap_`。
5. 最后执行 filter，避免 filter 的结果反向污染下一轮 plugin 内部状态。

这意味着：

- layer 更偏“地图生产者”。
- filter 更偏“区域规则叠加器”。

## 2. 主要类关系

### 2.1 `Costmap2DROS`

- 文件：`src/costmap_2d_ros.cpp`
- 作用：Lifecycle ROS 包装层。
- 负责参数读取、TF、插件加载、话题发布、更新线程和清图服务。
- 默认普通插件是：
  - `static_layer`
  - `obstacle_layer`
  - `inflation_layer`
- `filters` 默认是空，只有配置了 `filters:` 才会加载。

### 2.2 `LayeredCostmap`

- 文件：`src/layered_costmap.cpp`
- 作用：统一维护 master costmap，并按顺序调度所有 layer/filter。
- 这里是理解“plugin 与 filter 执行链”的关键入口。

### 2.3 `Layer`

- 文件：`include/nav2_costmap_2d/layer.hpp`
- 作用：所有插件统一基类。
- 核心接口：
  - `updateBounds()`
  - `updateCosts()`
  - `reset()`
  - `isClearable()`

### 2.4 `CostmapLayer`

- 文件：`include/nav2_costmap_2d/costmap_layer.hpp`
- 作用：带内部栅格缓存的 layer 基类。
- `StaticLayer` / `ObstacleLayer` / `VoxelLayer` / `RangeSensorLayer` 都建立在这类“自己维护一层局部 costmap”的模式上。

### 2.5 `CostmapFilter`

- 文件：
  - `include/nav2_costmap_2d/costmap_filters/costmap_filter.hpp`
  - `plugins/costmap_filters/costmap_filter.cpp`
- 作用：所有 filter 的公共基类。
- 统一提供：
  - `filter_info_topic`
  - `enable/disable` 服务
  - 机器人位姿缓存
  - mask 坐标转换
  - `process()` 扩展点

filter 的共同模式基本都是：

1. 先订阅 `nav2_msgs/msg/CostmapFilterInfo`
2. 再从其中拿到 `filter_mask_topic`
3. 订阅一张 `OccupancyGrid` mask
4. 按机器人位置或 costmap 更新窗口去解释 mask

## 3. 当前包内注册的插件

注册点在 `costmap_plugins.xml`。

### 3.1 普通 layer

#### `StaticLayer`

- 文件：`plugins/static_layer.cpp`
- 作用：把 `OccupancyGrid` 静态地图拷到 costmap。
- 典型来源：`map_server`
- 支持：
  - 全图加载
  - 增量更新 `map_topic_updates`
  - `track_unknown_space`
  - `use_maximum`

适合放：

- 墙体
- 固定货架
- 固定禁行区
- 已经画进地图的虚拟墙

#### `ObstacleLayer`

- 文件：`plugins/obstacle_layer.cpp`
- 作用：订阅 `LaserScan` / `PointCloud2`，做障碍标记与射线清除。
- 关键能力：
  - `marking`
  - `clearing`
  - `raytrace`
  - footprint clearing
  - 多 observation source

适合放：

- 实时二维障碍
- 临时障碍
- 激光/点云观测到的动态环境

#### `VoxelLayer`

- 文件：`plugins/voxel_layer.cpp`
- 作用：在 `ObstacleLayer` 的基础上增加 3D voxel 体素表达。
- 关键能力：
  - `z_voxels`
  - `z_resolution`
  - `origin_z`
  - `mark_threshold`
  - `publish_voxel_map`

适合放：

- RGB-D / 深度相机 / 3D 点云
- 需要利用高度信息判断障碍时

#### `InflationLayer`

- 文件：`plugins/inflation_layer.cpp`
- 作用：对致命障碍周围做代价膨胀。
- 关键参数：
  - `inflation_radius`
  - `cost_scaling_factor`
  - `inflate_unknown`
  - `inflate_around_unknown`

适合放：

- 给规划器留安全边界
- 避免路径擦边
- 将 footprint / inscribed radius 影响编码进代价图

#### `RangeSensorLayer`

- 文件：`plugins/range_sensor_layer.cpp`
- 作用：处理超声、红外等 `sensor_msgs/msg/Range`。
- 特点：
  - 按概率模型更新 costmap
  - 支持 `VARIABLE` / `FIXED` / `ALL`
  - 可针对最大测距做清除

适合放：

- 近距离补盲
- 超声防撞
- 激光覆盖不到的小范围探测

#### `DenoiseLayer`

- 文件：`plugins/denoise_layer.cpp`
- 作用：对已有 costmap 结果去噪，去掉孤立点或过小障碍簇。
- 关键参数：
  - `minimal_group_size`
  - `group_connectivity_type`

适合放：

- 点云噪声较多
- 想减少小碎点导致的局部规划抖动

### 3.2 costmap filter

#### `KeepoutFilter`

- 文件：`plugins/costmap_filters/keepout_filter.cpp`
- 作用：读取 keepout mask，把 mask 中更严格的代价覆盖到最终 `combined_costmap_`。
- 行为特点：
  - 直接改最终代价图
  - 会影响全局规划、局部规划和碰撞检查
  - 如果 mask 中某栅格值更高，就覆盖 master costmap 当前值

通常用法：

- 用 `LETHAL_OBSTACLE` 形成硬禁行区
- 用较高代价形成软约束区

注意：

- 它不是根据几何对象实时画线，而是读取一张已经准备好的 mask 栅格图。
- 它要求 `CostmapFilterInfo` 里的 `base` 和 `multiplier` 保持默认值。

#### `SpeedFilter`

- 文件：`plugins/costmap_filters/speed_filter.cpp`
- 作用：根据机器人当前位置所在 mask 栅格，发布 `nav2_msgs/msg/SpeedLimit`。
- 行为特点：
  - 不改障碍分布
  - 只改区域速度约束
  - 支持百分比限速和绝对限速两种模式

适合放：

- 路口限速
- 人机混行区限速
- 狭窄通道降速

#### `BinaryFilter`

- 文件：`plugins/costmap_filters/binary_filter.cpp`
- 作用：根据当前位置 mask 值触发一个布尔状态发布。
- 行为特点：
  - 不改 costmap
  - 不限速
  - 只是区域触发开关

适合放：

- 进某区域切换模式
- 进某区域打开额外设备、灯光、蜂鸣器等

## 4. filter 和 layer 的核心差别

可以直接按下面方式区分：

### layer

- 主要生产地图内容。
- 一般基于静态地图、激光、点云、Range 等输入。
- 典型结果是障碍、空闲区、膨胀代价。

### filter

- 主要在“已有地图结果之上”施加区域规则。
- 输入通常是一张额外 mask。
- 结果可能是：
  - 改代价图
  - 发限速消息
  - 发布布尔状态

一句话总结：

- `layer` 管“地图长什么样”。
- `filter` 管“某些区域怎么额外约束”。

## 5. 当前工程里的实际配置

从 `mobile_navigation/nav2_bringup/params/w1_nav2_rgbd_params.yaml` 可以看到，这个项目里 local/global costmap 都实际启用了：

- 普通插件：
  - `obstacle_layer`
  - `voxel_layer`
  - `denoise_layer`
  - `inflation_layer`
  - `static_layer`（global 中）
- filter：
  - `keepout_filter`
  - `speed_filter`

这说明当前项目已经采用了 Nav2 官方的 filter 机制来做：

- 区域禁行
- 区域限速

结合当前 `mobile_navigation/nav2_bringup/params/w1_nav2_params.yaml`，还有一个直接结论：

- `global_costmap` 已启用 `filters: ["keepout_filter", "speed_filter"]`
- `planner_server` 当前使用的是 `nav2_theta_star_planner/ThetaStarPlanner`

因此在当前配置下：

- 全局规划会直接使用包含 `keepout_filter` 结果的 `global_costmap`
- `keepout_filter` 对全局路径搜索生效
- `speed_filter` 不直接改变全局路径拓扑，它主要影响速度约束，不是禁行搜索边界

## 6. filter 的 `OccupancyGrid mask` 需要一直发吗

结论：

- 不需要像激光、点云那样持续高频发布。
- 订阅关系会一直保持，但运行时通常只使用“最后收到的一张 mask”。

从实现上看：

- `CostmapFilter::activate()` 会调用 `initializeFilter(filter_info_topic_)`，说明 filter 激活后会一直保留订阅链路。
- `KeepoutFilter` / `SpeedFilter` / `BinaryFilter` 都是先收一次 `CostmapFilterInfo`，再去订阅对应的 `filter_mask_topic`。
- `maskCallback()` 收到新的 `OccupancyGrid` 后，会把它缓存到 filter 内部成员中。
- 后续每次 `process()` 执行时，直接使用这份最新缓存，而不是要求上游每个周期都重发。
- 订阅 QoS 使用了 `transient_local + reliable + keep_last(1)`，设计意图就是保留最近一份有效 mask，新的订阅者也能拿到最后一次发布的数据。

因此更准确地说是：

- `filter_info_topic` 一般发一次就够，配置改了再发。
- `OccupancyGrid mask` 一般也是发一次就够，区域变了再发新图。
- 只要 filter 没重置、没切换上游链路，它就会持续复用最后收到的 mask。

工程建议：

- 静态禁行区/限速区：发布一次即可。
- 低频变化区域：有修改时重发一次。
- 高频动态区域：也可以更新，但本质仍是“刷新最后一张 mask”，不是必须持续流式发布。

当前分支还补了一点日志行为优化：

- filter 启用但尚未收到 mask 时，只打印一次 `Filter mask was not received`
- 收到有效 mask 后，会重新放开这条一次性告警
- 这样在上游未发布 mask 的阶段不会持续刷 WARN，但后续重新初始化或再次进入“无 mask”状态时，仍能看到提示

## 7. `keepout_filter` 是否支持“两点位虚拟墙”

结论先说：

- `KeepoutFilter` 本身不支持“传两个点位就生成一条虚拟墙”的原生接口。
- 它支持的是“读取一张 mask 栅格图”，不是“读取一组点/线段/多边形命令”。

原因从实现上很直接：

- `KeepoutFilter::filterInfoCallback()` 只接收 `CostmapFilterInfo`
- 然后只会去订阅 `filter_mask_topic`
- `maskCallback()` 接收的是 `nav_msgs/msg/OccupancyGrid`
- `process()` 只是把 costmap 窗口映射到这张 mask 上逐格覆盖

所以它的输入模型是：

- 一张栅格 mask

而不是：

- 两个点
- 一条线
- 多边形列表
- 类似 ROS1 prohibition layer 的几何对象配置

### 7.1 但能不能实现“两点位虚拟墙”效果

可以实现“效果等价”，但方式不是 `keepout_filter` 自己生成，而是你上游先把两点生成到 mask 里。

常见做法：

1. 先把两个点 rasterize 成一条窄线或窄矩形区域
2. 生成一张 keepout `OccupancyGrid`
3. 把这张 mask 发布给 `keepout_filter`

这样 planner/controller 最终看到的效果，就是一条不能穿过的虚拟墙。

也就是说：

- `keepout_filter` 不支持“两点输入”
- 但支持“两点先转 mask，再形成虚拟墙”

### 7.2 适合你的推荐方案

如果你场景里虚拟墙是静态或低频变化：

- 推荐直接用 keepout mask
- 两点线段在离线工具或单独节点中画成栅格线

如果你场景里虚拟墙需要频繁动态增删：

- 建议单独做一个“虚拟墙生成节点”
- 输入是两点或多边形
- 输出是 keepout `OccupancyGrid`

这样可以继续复用现有 `keepout_filter`，不用改 `nav2_costmap_2d` 主体代码。

## 8. 这个包里是否有类似 ROS1 `prohibition_layer` 的禁行能力

结论：

- 没有找到名为 `prohibition_layer` 的插件。
- 但“禁行能力”是有的，主要通过下面两类方式实现。

### 8.1 方式一：`KeepoutFilter`

这是最接近 ROS1 `prohibition_layer` 语义的现成能力。

特点：

- 专门做区域禁行
- 支持软/硬约束
- 不需要把禁行区直接写死到静态底图
- 可以独立维护一张 keepout mask

适合：

- 独立管理禁行区域
- 后续还想和限速 mask 一样做区域规则扩展

### 8.2 方式二：`StaticLayer`

如果你把禁行区域直接画进静态地图，`StaticLayer` 一样能实现禁行。

特点：

- 最简单
- 对规划器透明
- 不需要 filter info / mask 额外链路

缺点：

- 禁行区与静态地图耦合更紧
- 动态改动不方便

### 8.3 `ObstacleLayer` / `VoxelLayer` 能不能做禁行

从“结果上”可以，因为它们都能把某些栅格标成障碍。

但语义上不推荐把它们当 `prohibition_layer` 用，原因是：

- 它们本质上是传感器障碍层
- 更适合处理实时观测障碍
- 不适合表达“这是规则性禁行，不是物理障碍”

## 9. 如果要从 ROS1 `prohibition_layer` 迁移到这里，推荐怎么做

推荐优先级如下：

### 9.1 方案 A：迁移到 `KeepoutFilter`

适合：

- 你原来就是在维护禁行区
- 禁行区希望独立于地图维护
- 未来还可能动态切换

优点：

- 与 Nav2 官方 filter 机制一致
- 语义最清晰
- 后续扩展到 speed/binary filter 也自然

### 9.2 方案 B：迁移到 `StaticLayer`

适合：

- 禁行区几乎不变
- 更想减少系统链路复杂度

优点：

- 配置简单
- 稳定

缺点：

- 修改禁行区要改地图资源

### 9.3 方案 C：自定义 layer / filter

适合：

- 必须原生支持“两点位生成虚拟墙”
- 必须在线增删多条线段或多边形
- 不想额外维护 mask 栅格生成节点

实现建议：

- 可以新做一个自定义 filter 或 layer
- 内部直接维护线段/多边形集合
- 再利用 `Costmap2D` 已有的：
  - `raytraceLine()`
  - `polygonOutlineCells()`
  - `setConvexPolygonCost()`
  来栅格化虚拟墙或禁行区

## 10. 工程建议

结合当前仓库，比较推荐的落地方式是：

### 10.1 想做静态虚拟墙

- 直接维护 keepout mask
- 把线段画到 mask 图里
- 交给 `keepout_filter`

### 10.2 想做动态虚拟墙

- 单独写一个 ROS2 节点
- 输入：两个点或多边形
- 输出：`OccupancyGrid keepout mask`
- 仍复用 `keepout_filter`

### 10.3 想完全复刻 ROS1 prohibition layer 的“几何对象式配置”

- 需要自定义一个新插件
- 当前 `nav2_costmap_2d` 现成插件里没有直接等价物

## 11. 一页式结论

- `nav2_costmap_2d` 的普通插件负责生成地图，filter 负责叠加区域规则。
- 当前包内普通插件主要有：`StaticLayer`、`ObstacleLayer`、`VoxelLayer`、`InflationLayer`、`RangeSensorLayer`、`DenoiseLayer`。
- 当前包内 filter 主要有：`KeepoutFilter`、`SpeedFilter`、`BinaryFilter`。
- `KeepoutFilter` 是现成的禁行能力，但输入必须是一张 `OccupancyGrid mask`。
- filter 会一直保留订阅和缓存，但 mask 通常不需要持续高频发布，发一次后会复用最新一份。
- `KeepoutFilter` 不支持“直接输入两个点位生成虚拟墙”。
- 若把两点先栅格化成 mask 里的线/窄带区域，`KeepoutFilter` 可以达到同等效果。
- 当前包里没有现成的 ROS1 `prohibition_layer` 同名或同接口插件。
- 如果要做 Nav2 下的禁行迁移，优先推荐 `KeepoutFilter`；如果禁行区固定不变，也可以直接画进 `StaticLayer` 对应静态地图。
