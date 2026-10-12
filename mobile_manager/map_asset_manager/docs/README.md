# map_asset_manager

> **当前对外接口（2026-10-12）**：读 `~/get_assets`、写 `~/set_assets`、存版本 `~/save_assets`、回滚 `~/rollback_assets`、
> 变更事件 `~/asset_changes`、当前图快照 `~/assets`（`AssetSnapshot`，含区域与房间）、路径解析 `~/resolve_nav_points`；
> `~/get_all_nav_points` 仅为机器人本机客户端兼容保留。资产坐标变换用离线工具 `transform_assets_tool`。
> 已删除：`get_point` / `get_route` / `get_route_graph` / `sample_point` / `transform_assets` 服务，
> `~/areas` / `~/rooms` 话题及 `AreaInfoArray` / `RoomInfoArray` 消息。下文涉及这些的章节为历史记录，以本段为准。

## 1. 功能定位

`map_asset_manager` 是“当前地图资产”管理节点，职责边界应收口为地图资产真值源，而不是运行时导航执行节点。

它统一维护当前地图下的三类语义资产：

- `NavPoint`：导航点、充电点、停车点、取货点、电梯点等。
- `Route`：由导航点名称组成的有序路线。
- `Area`：虚拟墙、禁行区、减速区、充电区、电梯区等多边形区域。

推荐边界不是“在资产节点里直接发起导航”，而是三层模型：

```text
资产层：map_asset_manager
  -> 负责点 / 路线 / 区域的存储、查询、持久化、marker、结构化发布

业务/任务/界面上层
  -> 查询点位和路线
  -> 按业务时机调用标准 NavigateToPose / NavigateThroughPoses

导航适配层
  -> 订阅 areas 或查询 area_list
  -> 将 Area 转成禁行区、限速区、虚拟墙等导航真实输入
```

也就是说：

```text
map_asset_manager 只定义和发布资产语义
不直接承诺导航系统的执行细节
```

当前实现已经具备的资产层能力：

- 导航点：增删改查、采样当前机器人位姿建点、Marker 发布。
- 路线：增删查、全部查询、删除点后的级联清理、全量静态 Marker 发布。
- 区域：增删查、`AreaInfoArray` 发布、polygon Marker 发布。
- 持久化：统一保存到 `{map_folder}/{map_name}/map_assets.json`。
- RViz 插件：面板 + 地图交互工具，支持添加导航点、绘制区域。

说明：

- 当前源码里仍存在 `navigate_to_point`、`navigate_route`、`get_status` 和 `NavigationExecutor` 相关实现。
- 这些更适合视为现阶段遗留/待收口能力，而不是这个包的推荐长期职责。

## 2. 架构概览

推荐架构下，`map_asset_manager` 只保留资产层职责：

```text
MapAssetManagerNode
  ├── PointStore / RouteStore / AreaStore
  ├── NavPointManager / RouteManager / AreaManager
  ├── FileManager
  └── DataEventBus
```

职责划分：

- `Store`：只管内存数据和基本 CRUD。
- `Manager`：只管 ROS2 service/topic 和业务流程编排。
- `FileManager`：负责 `map_assets.json` 读写。
- `DataEventBus`：负责同进程内资产联动事件。
- `MapAssetManagerNode`：负责地图切换、共享组件初始化、热重载和总装配。

推荐的系统分工是：

```text
map_asset_manager
  -> 资产真值源

上层业务节点
  -> 查询点和路线
  -> 调标准导航 action

导航适配节点
  -> 消费 Area
  -> 翻译为导航约束输入
```

这套拆分的直接好处是：

- 资产编辑态和运行态导航解耦。
- `map_asset_manager` 不依赖 Nav2 也能独立运行。
- 导航入口可以来自任务系统、BT、调度器或 UI，而不必反向耦合进资产节点。
- `Area -> 导航约束` 的映射规则可以在一个中转节点集中维护，而不会散落到多个上层模块。

## 3. 地图上下文与持久化

节点启动后会等待：

- Topic：`map_manager/map_name`，使用 `transient_local` QoS 获取当前地图名。
- Topic：`map_manager/map_folder`，使用 `transient_local` QoS 获取地图根目录。

加载路径为：

```text
{map_folder}/{map_name}/map_assets.json
```

地图切换流程：

```text
收到新的 map_folder / map_name
-> 两者任一更新后尝试检查上下文是否齐备
-> 组合出新的 assets_file
-> 清空 point / route / area 三个 store
-> 重新加载 map_assets.json
-> 发布 DATA_RELOADED
-> 重新发布导航点、区域和路线可视化
```

如果资产文件不存在，不视为错误，首次写入时自动创建。

## 4. 数据模型

### 4.1 NavPoint

定义见 [types.hpp](/home/dex/tmp_fixworkspace/dexe_mobile_application/mobile_manager/map_asset_manager/include/map_asset_manager/types.hpp)：

```cpp
struct NavPoint {
    std::string name;
    std::string type;
    double x;
    double y;
    double yaw;
    double tolerance;
};
```

常见 `type`：

- `STATION`
- `CHARGING`
- `PARKING`
- `PICKUP`
- `LIFT`

说明：

- `tolerance` 是资产属性，供上层执行导航时使用。
- `sample_point` 当前通过 `map -> base_link` TF 连续采样 11 次取平均后建点。

推荐用法：

```text
上层通过 get_point 获取 x / y / yaw / tolerance
-> 上层自行组装 PoseStamped
-> 调标准 NavigateToPose
```

### 4.2 Route

```cpp
struct Route {
    std::string name;
    std::vector<std::string> point_names;
};
```

`Route` 只保存点名，不保存坐标。

这意味着它本质上是“可被上层消费的资产”，而不是“资产节点内部自带执行逻辑的任务对象”。

推荐用法：

```text
上层通过 get_route 获取 point_names
-> 再查询对应 NavPoint
-> 拼出 PoseStamped[]
-> 调标准 NavigateThroughPoses
```

这样做的好处：

- 点位更新后，上层天然拿到最新坐标。
- 路线资产保持轻量，不和某个固定执行器绑定。
- 导航入口属于上层运行时编排，而不是资产层内部职责。

当前实现补充：

- `get_all_routes` 已不再只是返回路线名列表，而是直接返回完整 `Route` 列表。
- 每条路线当前都会发布静态可视化：连线、朝向箭头和带序号的文本标签。

### 4.3 Area

当前 `Area` 已不是早期占位结构，而是与 `nav_interfaces/msg/AreaInfo` 对齐的运行时模型：

```cpp
struct Area {
    std::string name;
    uint32_t type;
    std::vector<geometry_msgs::msg::Point> polygon;
    float max_speed;
    float stop_distance;
    std::string linked_point_name;
    std::string charger_type;
    std::string attribute;
};
```

支持的区域类型常量：

- `VIRTUAL_WALL`
- `RESTRICTED_AREA`
- `SPEED_LIMIT_AREA`
- `DETOUR_AREA`
- `STOP_OBSTACLE_AREA`
- `PARKING_AREA`
- `CHARGING_AREA`
- `ELEVATOR_AREA`

其中常见字段含义：

- `max_speed`：减速区使用。
- `stop_distance`：停障区使用。
- `linked_point_name`：停车区/充电区关联点位。
- `charger_type`：充电区扩展属性。
- `attribute`：通用扩展字段，便于后续兼容升级。

`Area` 和点/路线不完全一样。点和路线更接近“上层直接消费的资产”，而 `Area` 更接近“需要二次语义翻译的资产”。

推荐生效路径：

```text
map_asset_manager
  -> 保存 Area
  -> 发布 marker
  -> 发布 AreaInfoArray

导航适配节点
  -> 订阅 areas 或查询 get_all_areas
  -> 按 AreaType 翻译成导航约束输入
```

不推荐两种做法：

- 不推荐由 `map_asset_manager` 直接把 `Area` 映射成某种具体导航实现细节。
- 不推荐由多个 UI/任务节点各自分散实现 `Area -> 导航约束` 映射。

更稳的方式是单独一个“导航适配/中转节点”收口：

- `VIRTUAL_WALL` -> 虚拟墙/边界输入
- `RESTRICTED_AREA` -> 禁行区输入
- `SPEED_LIMIT_AREA` -> 限速输入
- 其他类型按导航系统能力继续扩展

## 5. 统一资产文件格式

当前统一使用单文件持久化：

```json
{
  "points": [
    {
      "name": "station_A",
      "type": "STATION",
      "tolerance": 0.2,
      "pose": {
        "x": 1.0,
        "y": 2.0,
        "yaw": 1.57
      }
    }
  ],
  "routes": [
    {
      "name": "route_A",
      "points": ["station_A", "station_B"]
    }
  ],
  "areas": [
    {
      "name": "slow_zone_A",
      "type": "SPEED_LIMIT_AREA",
      "max_speed": 0.3,
      "stop_distance": 0.0,
      "linked_point_name": "",
      "charger_type": "",
      "attribute": "",
      "polygon": [
        {"x": 0.0, "y": 0.0},
        {"x": 1.0, "y": 0.0},
        {"x": 1.0, "y": 1.0}
      ]
    }
  ]
}
```

补充说明：

- `Area.type` 在文件里存字符串，如 `CHARGING_AREA`。
- `Route.points` 保存的是点名数组。
- 当前文件读写由 `FileManager::loadAllAssetsFromFile()` / `saveAllAssetsToFile()` 统一负责。

## 6. 启动方式

```bash
ros2 launch map_asset_manager map_asset_manager.launch.py
```

默认参数文件：

```text
mobile_manager/map_asset_manager/params/map_asset_manager_params.yaml
```

当前参数文件里仍有一些导航执行相关参数，例如：

- `default_tolerance`
- `controller_server_name`

从推荐架构角度看：

- `default_tolerance` 仍可以作为点位默认资产属性的参考参数保留。
- `controller_server_name` 更像导航执行层参数，长期更适合移出资产层。

## 7. ROS2 接口

节点名：

```text
/map_asset_manager_node
```

### 7.1 通用接口

```text
Service:
  /map_asset_manager_node/reload_data      std_srvs/srv/Trigger

Topic:
  /map_asset_manager_node/status           std_msgs/msg/String
```

`reload_data` 行为：

- 清空三个 store。
- 从当前 `map_assets.json` 重新加载。
- 发布 `DATA_RELOADED` 事件。

`status` 每 5 秒发布一次，当前内容格式类似：

```text
map=floor_1 points=12 routes=3 areas=5
```

地图上下文补充说明：

- 当前实现不再同步调用 `map_manager/get_map_folder` 服务。
- `map_folder` 和 `map_name` 任一先到都可以，等二者齐备后再真正切图加载。

### 7.2 导航点接口

```text
Services:
  /map_asset_manager_node/add_point
  /map_asset_manager_node/sample_point
  /map_asset_manager_node/remove_point
  /map_asset_manager_node/update_point
  /map_asset_manager_node/get_point
  /map_asset_manager_node/get_all_points

Topic:
  /map_asset_manager_node/nav_points_marker
```

查看接口定义：

```bash
ros2 interface show nav_interfaces/srv/AddNavPoint
ros2 interface show nav_interfaces/srv/SampleNavPoint
```

手动添加导航点：

```bash
ros2 service call /map_asset_manager_node/add_point nav_interfaces/srv/AddNavPoint \
"{point: {name: 'station_A', type: 'STATION', x: 1.0, y: 2.0, theta: 1.57, tolerance: 0.2}}"
```

按当前机器人位置采样建点：

```bash
ros2 service call /map_asset_manager_node/sample_point nav_interfaces/srv/SampleNavPoint \
"{name: 'charging_A', point_type: 2}"
```

查询全部点：

```bash
ros2 service call /map_asset_manager_node/get_all_points nav_interfaces/srv/GetAllNavPoints "{}"
```

推荐导航入口：

```text
上层 get_point
-> 取 x / y / yaw / tolerance
-> 自行调用标准 NavigateToPose
```

补充说明：

- `add_point` 支持同名覆盖。
- `sample_point` 不支持 `FREE_GOAL` 类型。
- 当前源码中虽仍存在 `get_status` 和 `navigate_to_point` 相关实现，但不建议作为长期架构依赖。

### 7.3 路线接口

```text
Services:
  /map_asset_manager_node/add_route
  /map_asset_manager_node/remove_route
  /map_asset_manager_node/get_route
  /map_asset_manager_node/get_all_routes
```

查看接口定义：

```bash
ros2 interface show nav_interfaces/srv/AddRoute
ros2 interface show nav_interfaces/srv/GetAllRoutes
```

添加路线：

```bash
ros2 service call /map_asset_manager_node/add_route nav_interfaces/srv/AddRoute \
"{name: 'route_A', point_names: ['station_A', 'station_B']}"
```

查询全部路线：

```bash
ros2 service call /map_asset_manager_node/get_all_routes nav_interfaces/srv/GetAllRoutes "{}"
```

推荐导航入口：

```text
上层 get_route
-> 解析 point_names
-> 逐个查询点位或直接缓存点表
-> 拼出 PoseStamped[]
-> 调标准 NavigateThroughPoses
```

补充说明：

- `add_route` 要求 `point_names` 中每个点都已存在。
- 当前同名路线不会覆盖，重复添加会失败。
- 删除导航点后，`RouteStore::onPointRemoved()` 会自动把该点从所有路线里移除并保存。
- `/map_asset_manager_node/route_markers` 当前已承担“持久化全量路线可视化”，不是只在运行时短暂显示。
- 当前源码中虽仍存在 `navigate_route` 相关实现，但不建议作为长期架构依赖。

### 7.4 区域接口

```text
Services:
  /map_asset_manager_node/add_area
  /map_asset_manager_node/remove_area
  /map_asset_manager_node/get_all_areas

Topics:
  /map_asset_manager_node/area_markers
  /map_asset_manager_node/areas
```

查看接口定义：

```bash
ros2 interface show nav_interfaces/srv/AddAreaInfo
ros2 interface show nav_interfaces/srv/GetAllAreas
ros2 interface show nav_interfaces/msg/AreaInfo
```

添加一个减速区示例：

```bash
ros2 service call /map_asset_manager_node/add_area nav_interfaces/srv/AddAreaInfo \
"{area_info: {name: 'slow_zone_A', type: 3, max_speed: 0.3, stop_distance: 0.0, linked_point_name: '', charger_type: '', attribute: '', polygon: [{x: 0.0, y: 0.0, theta: 0.0}, {x: 1.0, y: 0.0, theta: 0.0}, {x: 1.0, y: 1.0, theta: 0.0}]}}"
```

查询全部区域：

```bash
ros2 service call /map_asset_manager_node/get_all_areas nav_interfaces/srv/GetAllAreas "{}"
```

推荐使用语义：

- `areas` 和 `get_all_areas` 是资产输出接口。
- 禁行区、限速区、虚拟墙等不是 `map_asset_manager` 的直接最终输出承诺。
- 这些能力应由导航适配节点根据 `AreaType` 做统一转换。

补充说明：

- `area_markers` 使用 `transient_local` QoS，晚启动的 RViz 也能拿到最近一次区域可视化。
- `areas` 发布 `nav_interfaces/msg/AreaInfoArray`，适合给导航适配节点、规则模块或 UI 消费。
- `VIRTUAL_WALL` 的 marker 轮廓按线段发布，不自动闭合。

## 8. 事件联动

当前 `DataEventBus` 已定义：

- `POINT_ADDED`
- `POINT_UPDATED`
- `POINT_REMOVED`
- `ROUTE_ADDED`
- `ROUTE_UPDATED`
- `ROUTE_REMOVED`
- `AREA_ADDED`
- `AREA_UPDATED`
- `AREA_REMOVED`
- `DATA_RELOADED`

目前已接入的核心联动：

- `POINT_REMOVED` -> `RouteManager::onPointRemoved()`
- `POINT_UPDATED`（带旧名时）-> `RouteManager::onPointRenamed()`

注意：

- 当前 `UpdateNavPoint` 服务实现里还没有真正的 `old_name` 传递，因此“点重命名后自动改路线引用”这条链路在接口层还没有完全打通。
- `Area` 目前还没有订阅点删除/重命名事件，因此 `linked_point_name` 的级联一致性仍需后续补齐。

## 9. RViz 插件

如果当前环境编译到了 RViz 相关依赖，会生成插件库 `map_asset_rviz_plugin`，包含：

- Panel：`map_asset_rviz_plugin/MapAssetManagerPanel`
- Tool：`map_asset_rviz_plugin/AddNavPointTool`
- Tool：`map_asset_rviz_plugin/AddAreaTool`

面板能力：

- `导航点` 标签页：点位查询、增删改等。
- `路线` 标签页：路线增删查与调用。
- `区域` 标签页：区域增删查。
- 顶部 `Service prefix`：默认 `/map_asset_manager_node`，便于切换目标节点。

交互工具：

- `添加导航点 [N]`：在地图上单击放置点位。
- `绘制区域 [A]`：依次落多边形顶点，完成区域绘制。

## 10. 验证建议

按当前仓库的低成本验证方式，优先建议：

1. 启动节点后先确认地图上下文是否建立：

```bash
ros2 topic echo --once /map_asset_manager_node/status
```

2. 验证点位增删查：

```bash
ros2 service call /map_asset_manager_node/add_point nav_interfaces/srv/AddNavPoint \
"{point: {name: 'test_p1', type: 'STATION', x: 0.5, y: 0.5, theta: 0.0, tolerance: 0.1}}"
ros2 service call /map_asset_manager_node/get_point nav_interfaces/srv/GetNavPoint \
"{name: 'test_p1'}"
```

3. 验证路线级联：

先创建包含 `test_p1` 的路线，再删除 `test_p1`，随后调用 `get_route` 检查点名是否已被移除。

4. 验证区域发布：

```bash
ros2 topic echo --once /map_asset_manager_node/areas
```

5. 验证推荐接入路径：

- 上层通过 `get_point` 或 `get_route` 获取资产后，单独调用标准 Nav2 action。
- 导航适配节点订阅 `areas` 后，再转换为导航侧真实输入。

## 11. 当前已知边界

当前实现已经能支撑地图资产管理闭环，但从推荐架构角度仍有几处边界需要注意：

- 当前源码中仍有导航 action 与执行器相关实现，但不建议继续作为资产层能力扩展。
- 路线目前只有增删查，没有单独的更新服务。
- 点重命名到路线引用的自动传播，受限于 `UpdateNavPoint` 接口，目前尚未真正落地。
- 区域与点位的引用关系还没有像路线一样做事件级联清理。
- `Area -> 导航约束` 的翻译规则还没有从资产层外部单独收口成专门适配节点。

这些边界不是文档遗漏，而是当前实现和推荐架构之间仍存在的收口项。
