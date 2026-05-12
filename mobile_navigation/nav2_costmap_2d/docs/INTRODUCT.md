# nav2_costmap_2d 插件说明

## 1. 功能包整体作用

`nav2_costmap_2d` 用来维护导航中的二维代价地图。它会把静态地图、激光/点云障碍物、膨胀结果、特殊区域限制等信息按插件方式叠加到一张代价图里，供全局规划、局部规划和控制器使用。

这个包里的插件分成两类：

- `plugins_`：普通 Layer 插件，负责生成或更新代价地图本身。
- `filters_`：Costmap Filter 插件，基于额外的滤波掩膜对已有代价图或机器人行为做区域约束。

从 [layered_costmap.cpp](/home/dex/workspace_pzc/dexe_mobile_nav/mobile_navigation/nav2_costmap_2d/src/layered_costmap.cpp) 的实现可以看到：

- 没有 filter 时，普通插件直接顺序写入 `combined_costmap_`。
- 有 filter 时，普通插件先写入 `primary_costmap_`。
- 然后把 `primary_costmap_` 复制到 `combined_costmap_`。
- 最后再执行 `filters_`，避免 filter 的结果反过来影响普通 layer 下一轮的内部计算。

这意味着：

- 普通 layer 更关注“地图上哪里有障碍、哪里可通行、代价值是多少”。
- filter 更关注“机器人进入某区域后，要额外施加什么规则”。

## 2. 当前包内注册的插件

根据 [costmap_plugins.xml](/home/dex/workspace_pzc/dexe_mobile_nav/mobile_navigation/nav2_costmap_2d/costmap_plugins.xml)，这个包提供以下插件。

### 2.1 普通 Layer 插件

#### `InflationLayer`

文件：

- [inflation_layer.cpp](/home/dex/workspace_pzc/dexe_mobile_nav/mobile_navigation/nav2_costmap_2d/plugins/inflation_layer.cpp)

作用：

- 对障碍物周围做“膨胀”。
- 距离障碍越近，代价值越高。
- 让规划器和控制器主动远离障碍，而不是只在碰撞边界附近才规避。

典型用途：

- 将机器人 footprint 与障碍安全距离编码到代价图中。
- 让路径更平滑、更安全。

#### `ObstacleLayer`

文件：

- [obstacle_layer.cpp](/home/dex/workspace_pzc/dexe_mobile_nav/mobile_navigation/nav2_costmap_2d/plugins/obstacle_layer.cpp)

作用：

- 订阅 `LaserScan` 或 `PointCloud2`。
- 将观测到的障碍物标记到地图上。
- 通过 raytrace 清除传感器视线上的空闲区域。

典型用途：

- 处理实时动态障碍物。
- 处理局部环境感知结果。

#### `StaticLayer`

文件：

- [static_layer.cpp](/home/dex/workspace_pzc/dexe_mobile_nav/mobile_navigation/nav2_costmap_2d/plugins/static_layer.cpp)

作用：

- 订阅 `OccupancyGrid`，通常来自 `map_server`。
- 把静态地图内容拷贝到 costmap。
- 可选订阅增量更新。

典型用途：

- 提供建筑、墙体、固定禁行区域等静态底图。

#### `VoxelLayer`

作用：

- 与 `ObstacleLayer` 类似，但内部使用 3D voxel 体素结构。
- 更适合处理带高度信息的点云障碍。

典型用途：

- 处理三维传感器数据。
- 在有上下层结构或障碍高度差时，比纯 2D 障碍层更稳健。

#### `RangeSensorLayer`

作用：

- 处理超声、红外等 range sensor。
- 把距离传感器结果转换为障碍或空闲信息。

典型用途：

- 补充激光雷达以外的近距离感知。

#### `DenoiseLayer`

作用：

- 过滤噪声造成的孤立小障碍或小障碍簇。

典型用途：

- 降低误检噪声对规划的影响。

### 2.2 Costmap Filter 插件

#### `KeepoutFilter`

文件：

- [keepout_filter.hpp](/home/dex/workspace_pzc/dexe_mobile_nav/mobile_navigation/nav2_costmap_2d/include/nav2_costmap_2d/costmap_filters/keepout_filter.hpp)
- [keepout_filter.cpp](/home/dex/workspace_pzc/dexe_mobile_nav/mobile_navigation/nav2_costmap_2d/plugins/costmap_filters/keepout_filter.cpp)

作用：

- 读取 keepout mask。
- 将掩膜中对应区域的代价值写入最终 costmap。
- 常见效果是把某些区域直接标成致命障碍或高代价区域，从而禁止规划和控制进入。

工作机制：

1. 订阅 `CostmapFilterInfo`。
2. 从 `filter_mask_topic` 再订阅一张 `OccupancyGrid` 掩膜图。
3. 遍历当前更新窗口，把 master costmap 中的每个点映射到 mask。
4. 若 mask 中该点有有效值，且比原代价更“严格”，就覆盖 master_grid 中的代价值。

实现细节：

- 当 mask 和 costmap 不在同一坐标系时，会通过 TF 做坐标变换。
- 当 mask 与 global frame 一致时，会先计算重叠区域，只遍历交集，减少计算量。
- 它要求 `CostmapFilterInfo` 里的 `base` 和 `multiplier` 为默认值，否则会报错提示配置不符合 keepout 用法。

适用场景：

- 禁止进入电梯井口、楼梯口、危险区域。
- 仓储环境中限制机器人进入人工作业区。
- 对某些区域做“软禁行”或“硬禁行”。

一句话理解：

- `KeepoutFilter` 是直接改地图代价的 filter，结果会反映在规划和避障上。

#### `SpeedFilter`

文件：

- [speed_filter.hpp](/home/dex/workspace_pzc/dexe_mobile_nav/mobile_navigation/nav2_costmap_2d/include/nav2_costmap_2d/costmap_filters/speed_filter.hpp)
- [speed_filter.cpp](/home/dex/workspace_pzc/dexe_mobile_nav/mobile_navigation/nav2_costmap_2d/plugins/costmap_filters/speed_filter.cpp)

作用：

- 读取 speed mask。
- 根据机器人当前位置所在 mask 栅格值，计算当前区域的限速。
- 向外发布 `nav2_msgs/msg/SpeedLimit`，让控制链路按区域动态限速。

工作机制：

1. 订阅 `CostmapFilterInfo`。
2. 读取 `base`、`multiplier` 和 `type`。
3. 根据 `type` 判断限速是“百分比模式”还是“绝对速度模式”。
4. 再订阅 `filter_mask_topic` 对应的 mask。
5. 每次处理时，只关心“机器人当前所在 mask 栅格”。
6. 从该格子的值计算 `speed_limit = base + mask_value * multiplier`。
7. 若限速变化了，就发布新的 `SpeedLimit` 消息。

两种模式：

- `SPEED_FILTER_PERCENT`：发布相对于最大速度的百分比限速。
- `SPEED_FILTER_ABSOLUTE`：发布绝对速度值，单位通常是 `m/s`。

特殊值处理：

- mask 值 `0` 表示不限速。
- mask 值 `-1` 表示未知，对这种值会报错并忽略。
- 百分比模式下若计算结果不在 `[0, 100]`，会回退为不限速。
- 绝对值模式下若计算结果小于 `0`，也会回退为不限速。

适用场景：

- 路口、人机混行区域限速。
- 弯道、坡道、狭窄通道限速。
- 不同区域使用不同最大速度策略。

一句话理解：

- `SpeedFilter` 不直接修改代价图障碍分布，而是根据位置发布速度约束。

#### `BinaryFilter`

文件：

- [binary_filter.hpp](/home/dex/workspace_pzc/dexe_mobile_nav/mobile_navigation/nav2_costmap_2d/include/nav2_costmap_2d/costmap_filters/binary_filter.hpp)
- [binary_filter.cpp](/home/dex/workspace_pzc/dexe_mobile_nav/mobile_navigation/nav2_costmap_2d/plugins/costmap_filters/binary_filter.cpp)

作用：

- 根据机器人所在位置对应的 mask 值，输出一个布尔状态。
- 当 `base + mask_value * multiplier` 超过阈值 `flip_threshold` 时，发布非默认状态；否则恢复默认状态。

工作机制：

- 订阅 filter info 和 mask。
- 读取机器人当前位置对应 mask 单元。
- 判断是否超过阈值。
- 状态切换时发布 `std_msgs/msg/Bool`。

适用场景：

- 进入某区域后打开或关闭某个功能开关。
- 例如进入特殊地面区域时切换控制模式、切换灯光/蜂鸣器策略、切换附属设备行为。

一句话理解：

- `BinaryFilter` 是位置触发的二值开关。

## 3. Keepout 与 SpeedFilter 的核心区别

`KeepoutFilter` 和 `SpeedFilter` 都依赖 filter mask，但它们的作用层级不同。

### `KeepoutFilter`

- 直接作用在 `combined_costmap_` 上。
- 修改的是地图代价值。
- 结果会直接影响路径规划和局部避障。

### `SpeedFilter`

- 不改障碍物代价分布。
- 只根据当前位置发布速度限制消息。
- 结果主要影响控制器允许的速度上限。

可以简单理解为：

- `KeepoutFilter` 管“能不能走”。
- `SpeedFilter` 管“能走多快”。

## 4. Costmap Filter 的共同依赖

所有 filter 都继承自 [costmap_filter.hpp](/home/dex/workspace_pzc/dexe_mobile_nav/mobile_navigation/nav2_costmap_2d/include/nav2_costmap_2d/costmap_filters/costmap_filter.hpp)，公共能力包括：

- 统一的初始化入口 `initializeFilter()`。
- 统一的处理入口 `process()`。
- 坐标变换 `transformPose()`。
- 世界坐标到掩膜坐标转换 `worldToMask()`。
- 获取 mask 数据 `getMaskData()`。
- 启停服务 `enableCallback()`。

这说明 filter 插件的标准模式基本都是：

1. 收 `CostmapFilterInfo`。
2. 再收 `OccupancyGrid` mask。
3. 把机器人位置或地图窗口映射到 mask。
4. 根据 mask 值执行特定策略。

## 5. filter 类型常量

在 [filter_values.hpp](/home/dex/workspace_pzc/dexe_mobile_nav/mobile_navigation/nav2_costmap_2d/include/nav2_costmap_2d/costmap_filters/filter_values.hpp) 中定义了类型常量：

- `KEEPOUT_FILTER = 0`
- `SPEED_FILTER_PERCENT = 1`
- `SPEED_FILTER_ABSOLUTE = 2`
- `BINARY_FILTER = 3`

其中速度 filter 还定义了：

- `SPEED_MASK_UNKNOWN = -1`
- `SPEED_MASK_NO_LIMIT = 0`
- `NO_SPEED_LIMIT = 0.0`

## 6. 使用上的理解建议

如果从系统设计角度看，这个包里的插件可以按职责这样记忆：

- `StaticLayer`：静态底图。
- `ObstacleLayer` / `VoxelLayer` / `RangeSensorLayer`：实时感知障碍。
- `DenoiseLayer`：去掉噪声。
- `InflationLayer`：给障碍留安全边界。
- `KeepoutFilter`：按区域禁行或强约束。
- `SpeedFilter`：按区域限速。
- `BinaryFilter`：按区域触发布尔开关。

如果你后面要继续配置这个包，最值得优先区分的是：

- 哪些插件会改变 costmap 数值本身。
- 哪些插件只是根据位置输出额外控制信号。

前者会直接改变规划路径，后者更多改变机器人在该区域的行为策略。
