# navigation_adapter

`navigation_adapter` 是导航适配层功能包，用于把上层业务语义和资产语义转换为 Nav2 可直接消费的接口。

当前包只暴露一个 ROS 节点：

```text
/navigation_adapter_node
```

节点内部由两个普通模块类组成：

```text
NavigationAdapterNode
  ├── AreaFilterAdapter
  └── AssetNavigationAdapter
```

## 职责边界

- `map_asset_manager` 仍是地图资产真值源，负责点位、路线、区域的保存、查询和发布。
- `navigation_adapter` 不保存资产，只做运行时适配。
- `AreaFilterAdapter` 负责把区域资产转换为 Nav2 costmap filter 的 `CostmapFilterInfo + OccupancyGrid mask`。
- `AssetNavigationAdapter` 负责把点位名称、路线名称转换为标准 Nav2 action 调用。

## AreaFilterAdapter

输入：

- `/map_asset_manager/areas`：`nav_interfaces/msg/AreaInfoArray`
- `/map`：`nav_msgs/msg/OccupancyGrid`
- `map_manager/map_name`：`std_msgs/msg/String`

输出：

- `/keepout_filter_mask`：`nav_msgs/msg/OccupancyGrid`
- `/speed_filter_mask`：`nav_msgs/msg/OccupancyGrid`
- `/keepout_costmap_filter_info`：`nav2_msgs/msg/CostmapFilterInfo`
- `/speed_costmap_filter_info`：`nav2_msgs/msg/CostmapFilterInfo`
- `~/status`：`std_msgs/msg/String`

区域类型映射（对齐最新 `nav_interfaces/AreaInfo` 9 类常量）：

| AreaInfo.type             | 本层处理     | 说明                                                          |
| ------------------------- | ------------ | ------------------------------------------------------------- |
| `VIRTUAL_WALL` (1)        | keepout mask | 线段栅格化 + `virtual_wall_width` 半径膨胀                    |
| `RESTRICTED_AREA` (2)     | keepout mask | 多边形扫描线填充                                              |
| `SPEED_LIMIT_AREA` (3)    | speed mask   | 多边形扫描线填充 + `max_speed` 编码到 `1..100`                |
| `DETOUR_AREA` (4)         | ignored      | 语义为"遇障绕行"，属运行时行为，不适合静态 costmap filter     |
| `STOP_OBSTACLE_AREA` (5)  | ignored      | `stop_distance` 由停障行为层消费，不映射 mask                 |
| `PARKING_AREA` (6)        | ignored      | 停车点关联属于资产/任务调度层                                 |
| `CHARGING_AREA` (7)       | ignored      | 充电点/充电方式属于资产/调度层                                |
| `ELEVATOR_AREA` (8)       | ignored      | 电梯进出由专用电梯适配层处理                                  |
| `BUMPY_AREA` (9)          | ignored      | `depth_min_height` 影响深度相机参数，不在 costmap filter 范畴 |
| 其它（未识别 type）       | unknown      | 上游 enum 演进尚未跟进，会输出 WARN 日志便于发现              |

所有被忽略的区域都会按类型计入 `ignored_by_type[t]`，并在状态 topic (`~/status`) 和重建 INFO 日志中以 `ignored_by_type=parking:1,bumpy:2,unknown:1` 形式输出，避免静默吞掉新类型导致难以排障。

> 设计原则：navigation_adapter 仅承担"区域资产 -> Nav2 costmap filter"这一段适配。`DETOUR/STOP_OBSTACLE/PARKING/CHARGING/ELEVATOR/BUMPY` 属于"行为/资产/调度"语义，应由对应适配层独立消费，不在本包内做语义混搭。

区域 mask 使用全量重建模式，但会比较地图几何和区域资产内容签名。若 `map_asset_manager` 周期发布相同区域，adapter 不会重复发布 mask/info，避免 Nav2 filter 反复刷新。

变化检测采用结构化 hash，而不是字符串拼接。hash 输入包括地图 frame、栅格宽高、分辨率、origin，以及区域名称、类型、`max_speed`、`stop_distance`、`depth_min_height`、`linked_point_name`、`charger_type`、`attribute` 和 polygon 点位。区域会先按 `name/type` 排序后参与 hash，因此 `AreaInfoArray` 发布顺序变化不会误触发重建。即便区域当前不映射 mask（例如 BUMPY_AREA），只要其内容字段变化也会进入 hash，保证未来扩展时变化检测保持完整。

浮点字段不会直接按二进制值 hash，而是先量化成整数，避免 `1.0` 与 `1.0000000001` 这类浮点抖动触发无意义重建。当前默认量化精度为：位置/距离 `1 mm`、速度 `1 mm/s`、角度 `0.0001 rad`、地图 resolution `1e-6 m`。

栅格化策略面向大地图场景：

- `RESTRICTED_AREA` 和 `SPEED_LIMIT_AREA` 使用扫描线填充 polygon，按栅格行计算交点并填充区间，避免在包围盒内对每个 cell 重复执行 point-in-polygon。
- `SPEED_LIMIT_AREA` 重叠时比较原始 `max_speed`，保留更严格的限速，再编码到 `1..100` mask 值。
- `VIRTUAL_WALL` 使用线段采样/DDA 风格遍历线段经过格子，再按 `virtual_wall_width` 做半径膨胀，表达窄带禁行，不闭合、不面填充。
- 仍保持 full snapshot rebuild，不做增量 patch，避免地图切换或区域删除后出现旧 mask 残留。

## AssetNavigationAdapter

对外 action：

- `~/navigate_to_point`：`nav_interfaces/action/NavigateToPoint`
- `~/navigate_route`：`nav_interfaces/action/NavigateRoute`

内部调用：

- `/map_asset_manager/get_point`
- `/map_asset_manager/get_all_points`
- `/map_asset_manager/get_route`
- `/navigate_to_pose`
- `/navigate_through_poses`

点位导航流程：

```text
NavigateToPoint(point_name)
  -> /map_asset_manager/get_point
  -> NavPoint(x, y, theta)
  -> /navigate_to_pose
```

路线导航流程：

```text
NavigateRoute(route_name)
  -> /map_asset_manager/get_route
  -> point_names[]
  -> /map_asset_manager/get_all_points
  -> startup point snapshot
  -> PoseStamped[]
  -> /navigate_through_poses
```

路线导航采用“启动时点位快照”语义：每次执行路线前只获取一次 `get_all_points`，用临时 `name -> NavPoint` 映射按 `point_names[]` 原顺序组装 `PoseStamped[]`。导航过程中点位资产变化不会影响当前已经下发的路线；路线中任一点位在快照中缺失时，adapter action 失败，且不会部分下发 Nav2 goal。

点位导航仍使用单次 `get_point`，避免为了单点任务拉取全量点位。

运行日志：

- 区域内容变化并重新发布 filter 时，会输出当前 map、区域数量、keepout/speed/ignored 数量和 render hash。
- action goal 被接受时会输出目标点位或路线名。
- 下发 Nav2 `NavigateToPose` / `NavigateThroughPoses` 时会输出目标位姿、路线点数和点位快照大小。
- action 成功完成时会输出成功日志。

退出行为：

- 节点使用显式 executor 管理 spin 和析构顺序。
- action worker 线程不再 detach，shutdown 时会 join，避免 Ctrl+C 后线程继续访问已析构模块。
- 若正在等待 Nav2 action result 时收到 shutdown，会尽快退出等待；ROS context 仍有效时尝试转发 cancel，context 已关闭时直接结束本层 action。

### Action 调用示例

按点位名称导航：

```bash
ros2 action send_goal --feedback \
  /navigation_adapter_node/navigate_to_point \
  nav_interfaces/action/NavigateToPoint \
  "{point_name: 'station_A', use_pose: false}"
```

使用自定义 pose 导航：

```bash
ros2 action send_goal --feedback \
  /navigation_adapter_node/navigate_to_point \
  nav_interfaces/action/NavigateToPoint \
  "{
    point_name: '',
    use_pose: true,
    pose: {
      header: {frame_id: 'map'},
      pose: {
        position: {x: 1.0, y: 2.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      }
    },
    tolerance: 0.0
  }"
```

按路线名称导航：

```bash
ros2 action send_goal --feedback \
  /navigation_adapter_node/navigate_route \
  nav_interfaces/action/NavigateRoute \
  "{route_name: 'patrol_route'}"
```

调试提示：

```bash
ros2 action list | grep navigation_adapter_node
ros2 action info /navigation_adapter_node/navigate_to_point
ros2 action info /navigation_adapter_node/navigate_route
```

`ros2 action send_goal` 运行后可用 `Ctrl+C` 停止客户端显示，但这不等同于取消 action。需要取消正在执行的 goal 时，建议由上层业务节点通过 action client 发送 cancel；命令行调试时可直接观察 adapter 日志和 Nav2 action 状态。

## 参数

参数文件：`params/navigation_adapter_params.yaml`

所有参数都在 `navigation_adapter_node.ros__parameters` 下维护。当前分为两组：

- 区域过滤参数：地图、区域、mask、filter info topic，以及 speed 编码参数。
- 资产导航参数：资产查询 service、Nav2 action、对外 action、frame 和超时参数。
- `get_all_points_service`：默认 `/map_asset_manager/get_all_nav_points`，仅路线导航启动时用于构建临时点位快照。

## 与 map_asset_manager 的框架对比

### 数据结构与访问性能

`map_asset_manager` 的核心数据结构更偏“资产数据库”：

- 点位、路线、区域分别由 `PointStore`、`RouteStore`、`AreaStore` 管理。
- store 内部使用 `std::unordered_map<std::string, T>`，按名称查询是平均 O(1)，适合 `get_point`、`get_route` 这种真值源查询。
- store 通过 `std::mutex` 做线程保护，返回值多为拷贝对象或快照，接口安全性较好，但大量资产时 `getAll*()` 和 `get*Map()` 会产生完整拷贝。
- `RouteStore` 中路线只保存 `point_names[]`，不复制点位坐标，资产关系表达清晰，也避免点位更新后路线坐标失效。

`navigation_adapter` 的数据结构更偏“运行时转换缓存”：

- `AreaFilterAdapter` 缓存最新 `/map`、最新 `AreaInfoArray`、两张 mask，以及 `speed_limits_grid_`。
- mask 使用 `OccupancyGrid.data` 直接承载 Nav2 filter 输入，数据结构贴近下游接口，减少二次转换。
- 区域变化检测使用结构化 hash，避免大区域快照下反复构造长字符串；区域按 `name/type` 排序后参与 hash，发布顺序变化不会误触发。
- `AssetNavigationAdapter` 不长期缓存点位和路线。点位导航单次查询 `get_point`；路线导航在启动时调用一次 `get_route` 和一次 `get_all_points`，只构建本次任务的临时快照。

当前判断：

- `map_asset_manager` 的数据结构适合作为资产真值源，名称索引清晰，查询效率稳定。
- `navigation_adapter` 的数据结构适合作为 V1 适配层，后续性能压力主要来自超大地图下的 mask 全量重建、全量点位快照大小和区域发布频率。

### 模块边界与职责清晰度

`map_asset_manager` 的边界更成熟：

- 顶层 `MapAssetManagerNode` 只负责装配共享组件、加载保存和地图切换。
- `NavPointManager`、`RouteManager`、`AreaManager` 分别负责点、路线、区域接口。
- store 层只管内存数据，不直接依赖 ROS service/action。
- `DataEventBus` 用于点位删除/重命名后联动路线，降低 manager 之间的直接耦合。

`navigation_adapter` 已按相同方向收敛：

- 顶层 `NavigationAdapterNode` 只装配模块。
- `AreaFilterAdapter` 和 `AssetNavigationAdapter` 都是普通模块类，通过同一个 `rclcpp::Node*` 注册接口。
- `AreaFilterAdapter` 面向 Nav2 filter，`AssetNavigationAdapter` 面向 Nav2 action，两者没有共享状态，模块边界清楚。

当前差异：

- `map_asset_manager` 有独立 store 和 event bus，适合复杂资产联动。
- `navigation_adapter` 暂无独立缓存层和事件层，这是合理的，因为它不保存资产；但如果后续增加更多导航适配能力，需要避免把所有运行时状态都堆进单个 adapter 类。

### 运行时性能路径

`map_asset_manager` 的主要性能路径：

- 单点/单路线查询：名称索引，平均 O(1)。
- 全量发布 marker/areas：周期性遍历所有资产，资产量大时有线性开销。
- 保存/加载：文件级全量读写，适合资产规模中等的地图项目。

`navigation_adapter` 的主要性能路径：

- 区域 mask 重建：任一有效输入变化后，全量清空 mask，再按当前区域快照重新渲染；相同 hash 的周期区域消息不会重复发布。
- polygon 填充：使用扫描线按行填充交点区间，复杂度更接近 `区域覆盖行数 * polygon 顶点数 + 写入 cell 数`，比包围盒内逐 cell 几何判断更适合万米级地图。
- virtual wall：按线段经过的格子采样并膨胀半径，长线段不再扫描整块包围盒。
- speed 区域重叠：通过 `speed_limits_grid_` 比较原始限速，语义正确，额外内存开销为一张 float grid。
- 点位/路线导航：点位导航一次 `get_point`；路线导航一次 `get_route + 一次 get_all_points`，避免长路线 N 次 service 往返。

当前判断：

- 区域过滤保持全量重建策略，逻辑简单、安全，适合区域全量快照和地图切换场景。
- 当前栅格化已避免最重的逐 cell 几何判断路径，后续大地图压力主要来自 mask 全量清空/写入本身。
- 路线导航使用启动时快照，启动延迟和路线点数量不再线性绑定到 service 往返次数。

### 可优化方向

优先级较高：

- 拆分 AreaFilterAdapter 内部职责：后续可拆成 `AreaClassifier`、`MaskRenderer`、`FilterPublisher` 等内部类或文件，降低单文件复杂度。
- 增加 mask 渲染性能观测：在重建时记录区域数量、渲染耗时、keepout/speed 写入 cell 数，状态 topic 或 debug 日志可输出。
- 对超大地图进一步优化 mask 清空：当前 full rebuild 会重置完整 `OccupancyGrid.data`，万米级高分辨率地图下可评估 dirty bounds 或分块工作缓冲，但要避免旧区域残留。

中期优化：

- 增加参数分组约定：当前所有参数在同一 node namespace 下，后续可采用名称前缀如 `area_filter.*`、`asset_navigation.*`，提升配置可读性。
- 如路线非常多且 `get_all_points` 返回量巨大，可考虑让 `map_asset_manager` 增加批量按名称查询接口，但不建议在 adapter 内做长期点位缓存。
- 如果区域编辑频率变高，可考虑在保持全量快照语义的前提下，将渲染任务放入后台 worker，并对连续输入做 debounce。

暂不建议优化：

- 不建议在 `navigation_adapter` 内长期缓存资产真值，否则会和 `map_asset_manager` 形成双真值源。
- 不建议为了减少 service 调用而让路线消息直接复制点位坐标，除非资产接口明确升级为“路线快照”语义。
- 不建议把 area 直接转换逻辑下放到 UI 或业务节点，否则 AreaType 到 Nav2 约束的映射会分散。

## 验证建议

编译：

```bash
colcon build --packages-select navigation_adapter
```

接口检查：

```bash
ros2 node list
ros2 action list | grep navigation_adapter_node
ros2 topic list | grep filter
```

期望只看到一个节点 `/navigation_adapter_node`，并能看到：

```text
/navigation_adapter_node/navigate_to_point
/navigation_adapter_node/navigate_route
```
