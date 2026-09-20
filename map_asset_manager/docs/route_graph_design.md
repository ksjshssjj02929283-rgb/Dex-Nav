# 路网拓扑图集成方案 — RouteGraph

> 本文档记录将 ROS1 `create_waypoint` 路网功能迁移并集成到 `map_asset_manager` 的设计方案。

---

## 一、概念厘清：两种"Route"的区别

当前 `map_asset_manager` 中已有 `Route`（任务路线），新增的 `RouteGraph`（路网拓扑图）是完全不同的概念：

| 维度       | 任务路线 Route            | 路网拓扑图 RouteGraph            |
| -------- | --------------------- | --------------------------- |
| **语义**   | 一次任务要经过的 N 个导航点（有序序列） | 地图上所有可通行连接的拓扑网络             |
| **结构**   | 点序列 `[A, B, C]`       | 有向/无向图 `节点 + 边`             |
| **使用时机** | 执行任务时指定路径             | 规划算法寻路时查询邻接关系               |
| **增删频率** | 随任务增删                 | 随地图建设缓慢演化                   |
| **节点来源** | 仅引用 NavPoint（有语义）     | NavPoint + Junction（纯拓扑中间点） |

**结论**：`Route` 和 `RouteGraph` 并列存储在 `map_assets.json`，互不替代。

---

## 二、create_waypoint 现有方案分析

### 2.1 数据格式

使用 GeoJSON `FeatureCollection`，存储在 `{map_folder}/{map_name}/map.json`：

```json
{
  "type": "FeatureCollection",
  "features": [
    {"type":"Feature","properties":{"id":"p_0","yaw":0.0,"name":"checkpoint1"},
     "geometry":{"type":"Point","coordinates":[10.5,20.3,0.0]}},
    {"type":"Feature","properties":{"id":"l_0","startid":"p_0","endid":"p_1","maxspeed":8},
     "geometry":{"type":"LineString","coordinates":[[10.5,20.3,0],[15.1,25.8,0]]}},
    {"type":"Feature","properties":{"id":"b_0","startid":"p_1","endid":"p_2"},
     "geometry":{"type":"MultiPoint","coordinates":[[15.1,25.8,0],[18.0,28.0,0],[20.5,30.2,0]]}}
  ]
}
```

三类 Feature：
- `p_N`（Point）：路网节点（含 yaw、name）
- `l_N`（LineString）：直线边（含 maxspeed、xytolerance、thtolerance、lanewidth、bidirectional）
- `b_N`（MultiPoint）：贝塞尔曲线边（控制点列表）

### 2.2 RViz 工具（7 个）

| 工具 | 快捷键 | 功能 |
|------|--------|------|
| WayPoint | p | 添加路网节点 |
| WayLine | c | 添加直线边（连接两节点） |
| WayCurveLine | — | 添加贝塞尔曲线边 |
| WayDel | d | 删除节点或边 |
| SaveWay | — | 保存到文件 |
| LoadWay | — | 从文件加载 |
| PickPoint | — | 吸附到已有点 |

---

## 三、改进点分析

### 3.1 节点类型缺失（重要）

**问题**：`create_waypoint` 的路网节点没有类型区分，中间拓扑节点和语义导航点混在一起。  
**改进**：引入 `NavPoint.type` 新增 `JUNCTION` 类型（仅用于拓扑路网，不出现在任务选点列表中）。

### 3.2 ID 语义不足

**问题**：节点 ID 自动递增（`p_0`, `p_1`），边 ID 也是（`l_0`, `b_0`），无语义，批量导入后难以管理。  
**改进**：节点复用 `NavPoint.name`（用户命名），边用 `{from}→{to}` 或用户命名，支持按名称 CRUD。

### 3.3 GeoJSON 格式冗余

**问题**：GeoJSON 是地理信息标准格式，对机器人室内导航来说 payload 冗余（type/geometry 嵌套层次深）。  
**改进**：在 `map_assets.json` 中使用简洁的扁平 JSON 结构，节省解析开销。

### 3.4 方向性隐式

**问题**：边的方向由 `startid → endid` 隐式决定，是否双向只能从属性推断，语义不清晰。  
**改进**：`RouteEdge` 增加 `bool bidirectional` 字段，明确双向/单向。

### 3.5 无 ROS2 服务接口

**问题**：`create_waypoint` 是纯 RViz 工具，没有服务化，外部系统（任务调度器、Web UI）无法动态增删路网。  
**改进**：提供完整 ROS2 CRUD 服务接口（`add_route_edge` / `remove_route_edge` / `get_route_graph`）。

### 3.6 工具数量过多

**问题**：7 个工具中 SaveWay/LoadWay 在 ROS2 + map_manager 体系下不需要（自动持久化），PickPoint 可内建到 AddEdge 工具中；原方案缺少独立的路网节点增删工具（路网节点与导航点概念不同）。  
**改进**：调整为 5 个工具：AddJunctionTool / DeleteJunctionTool / AddEdgeTool / AddCurveEdgeTool / DeleteEdgeTool，自动持久化。

### 3.7 贝塞尔控制点交互

**现有做法**：`create_waypoint` 的 WayCurveLine 要求用户依次点击所有控制点，最后右键确认。  
**保留并改进**：保持手动点击控制点的方式（用户明确指定曲线形状），改进点在于：实时 Ogre 贝塞尔曲线预览（每移动鼠标即刷新，而非点击后才更新）；同时将"自动切线"作为可选快捷方式（勾选后从两端 NavPoint yaw 自动生成初始控制点，用户可在此基础上继续调整）。

### 3.8 无级联删除

**问题**：删除路网节点后，引用该节点的边数据不会自动清理，存在悬空引用。  
**改进**：接入 `DataEventBus`，订阅 `POINT_REMOVED` 事件，触发所有引用该节点的边的级联删除和持久化。

---

## 四、数据模型

### 4.1 NavPoint 与 Junction 的职责划分

**NavPoint 不做任何改动**，现有三种类型（STANDARD / CHARGING / PARKING）保持不变。NavPoint 通过 `RouteEndpoint` 引用机制参与路网，不需要在自身数据结构中感知路网的存在。

**Junction** 是路网拓扑图私有的中间连接节点，独立存储在 `route_graph.junctions` 中，由 `RouteGraphManager` 独立管理其生命周期（增删改查），不注入 NavPointManager，也不出现在 `map_assets.json` 的 `points` 数组中。

两类节点通过 `RouteEndpoint`（含 `type` 字段）统一被路网边引用，消除命名歧义。

### 4.2 RouteJunction 数据结构

```cpp
struct RouteJunction {
    std::string id;    // 路网内唯一 ID（用户命名，如 "jct_corridor_turn"）
    float x;           // map 坐标系
    float y;
    float yaw;         // 方向（影响贝塞尔自动切线方向）
};
```

### 4.3 RouteEndpoint 与 RouteEdge 数据结构

```cpp
struct RouteEndpoint {
    std::string id;    // NavPoint.name 或 RouteJunction.id
    std::string type;  // "nav" | "junction"
};

struct RouteEdge {
    std::string    id;             // 唯一 ID，建议 "{from.id}→{to.id}"
    RouteEndpoint  from;           // 起点（nav 或 junction）
    RouteEndpoint  to;             // 终点（nav 或 junction）
    std::string    edge_type;      // "LINE" | "BEZIER"
    bool           bidirectional;  // true = 双向通行
    float          max_speed;      // 最大速度 m/s（0 = 系统默认）
    float          xy_tolerance;   // 途经点 XY 容差 m
    float          th_tolerance;   // 途经点朝向容差 rad
    float          lane_width;     // 通道宽度 m（0 = 不限）
    std::string    obs_policy;     // "STOP" | "DETOUR"
    std::vector<std::pair<float,float>> control_points;  // 贝塞尔控制点
};
```

坐标解析规则：执行路网导航时，按 `type` 路由查询——`"nav"` 从 `PointStore` 取坐标，`"junction"` 从 `JunctionStore` 取坐标，两处查不到则拒绝执行并报错。

### 4.4 map_assets.json 格式扩展

```json
{
  "points": [...],
  "routes": [...],
  "areas": [...],
  "route_graph": {
    "junctions": [
      { "id": "jct_corridor_turn", "x": 18.0, "y": 28.0, "yaw": 1.57 }
    ],
    "edges": [
      {
        "id": "entrance→hall",
        "from": { "id": "entrance", "type": "nav" },
        "to":   { "id": "hall",     "type": "nav" },
        "edge_type": "LINE",
        "bidirectional": true,
        "max_speed": 0.8,
        "xy_tolerance": 0.3,
        "th_tolerance": 0.5,
        "lane_width": 1.0,
        "obs_policy": "DETOUR",
        "control_points": []
      },
      {
        "id": "hall→jct_corridor_turn",
        "from": { "id": "hall",               "type": "nav" },
        "to":   { "id": "jct_corridor_turn",  "type": "junction" },
        "edge_type": "BEZIER",
        "bidirectional": false,
        "max_speed": 0.5,
        "xy_tolerance": 0.2,
        "th_tolerance": 0.4,
        "lane_width": 0.8,
        "obs_policy": "STOP",
        "control_points": [[19.0, 27.5], [20.5, 28.8]]
      }
    ]
  }
}
```

---

## 五、nav_interfaces 消息/服务定义

### 5.1 新增消息

**`RouteEndpoint.msg`**

```
string id     # NavPoint.name 或 RouteJunction.id
string type   # "nav" | "junction"
```

**`RouteJunction.msg`**

```
string  id
float32 x
float32 y
float32 yaw
```

**`RouteEdge.msg`**

```
string                        id
nav_interfaces/RouteEndpoint  from
nav_interfaces/RouteEndpoint  to
string                        edge_type      # LINE / BEZIER
bool                          bidirectional
float32                       max_speed
float32                       xy_tolerance
float32                       th_tolerance
float32                       lane_width
string                        obs_policy     # STOP / DETOUR
geometry_msgs/Point[]         control_points
```

### 5.2 新增服务

**Junction 管理**

```
# AddRouteJunction.srv
nav_interfaces/RouteJunction junction
---
bool    success
string  message
```

```
# RemoveRouteJunction.srv
string id
---
bool    success
string  message
```

**Edge 管理**

```
# AddRouteEdge.srv
nav_interfaces/RouteEdge edge
---
bool    success
string  message
```

```
# RemoveRouteEdge.srv
string id
---
bool    success
string  message
```

```
# UpdateRouteEdge.srv（修改边属性）
nav_interfaces/RouteEdge edge
---
bool    success
string  message
```

**全量查询**

```
# GetRouteGraph.srv
---
bool                           success
nav_interfaces/RouteJunction[] junctions    # 路网自有中间节点（含 x/y/yaw）
nav_interfaces/RouteJunction[] nav_points   # 被边引用的 NavPoint（复用 RouteJunction 格式表达，含 x/y/yaw）
nav_interfaces/RouteEdge[]     edges
```

> **设计说明**：`nav_points` 返回所有在边中出现的 NavPoint 坐标信息，使消费方无需额外调用 `GetAllNavPoints` 即可得到路网完整数据。
> 复用 `RouteJunction` 格式表达 NavPoint（只使用 id/x/y/yaw 四个字段），避免新增 msg 类型。详见 D034。

---

## 六、服务端点（RouteGraphManager）

**Junction 管理**（由 RouteGraphManager 独立处理，不经过 NavPointManager）：

| 服务名 | 类型 | 功能 |
|--------|------|------|
| `~/add_route_junction` | AddRouteJunction | 添加路网中间节点 |
| `~/remove_route_junction` | RemoveRouteJunction | 删除路网中间节点，同时级联删除引用它的边 |

**Edge 管理**（from/to 校验逻辑：`type="nav"` 查 PointStore，`type="junction"` 查 JunctionStore）：

| 服务名 | 类型 | 功能 |
|--------|------|------|
| `~/add_route_edge` | AddRouteEdge | 添加路网边（校验两端节点均存在） |
| `~/remove_route_edge` | RemoveRouteEdge | 按 ID 删除边 |
| `~/update_route_edge` | UpdateRouteEdge | 更新边属性 |
| `~/get_route_graph` | GetRouteGraph | 获取全量 junctions + edges |

> **设计原则**：消费方应始终通过 `GetRouteGraph` 服务接口获取路网数据，**不直读 `map_assets.json`**。
> `JunctionStore` + `RouteEdgeStore` 内存是权威数据源（运行时增删立即可见）；JSON 是持久化剧本，全量重写期间存在不完整状态。

话题发布：

| 话题名 | 消息类型 | QoS | 功能 |
|--------|----------|-----|------|
| `~/route_graph_markers` | `visualization_msgs/MarkerArray` | transient_local | 路网可视化（NavPoint 节点蓝色圆环、Junction 节点灰色空心圆、边箭头） |

---

## 七、RViz 工具设计（5 个）

路网工具的端点选取支持**两类节点**：NavPoint（蓝色圆环高亮）和 Junction（灰色空心圆高亮）。工具内部根据点击到的节点类型自动填入 `{ id, type }` 字段，用户无需手动区分。

### 7.1 AddJunctionTool（添加路网中间节点）

Junction 是 RouteGraphManager 私有的拓扑中间点，不进入 NavPointManager，专门通过此工具添加。

**交互流程**：
1. 激活后在地图上左键单击放置位置
2. 弹出对话框，填写 Junction ID（如 `jct_corridor_turn`）和 yaw
3. 确认后调用 `~/add_route_junction`，route_graph_markers 立即更新
4. 可连续添加多个节点，右键或 Esc 退出

**视觉渲染**：灰色空心圆 + 细方向线（区别于 NavPoint 的实心彩色圆柱）。

### 7.2 DeleteJunctionTool（删除路网中间节点）

**交互流程**：
1. 激活后点击地图上的 Junction 节点（灰色空心圆高亮）
2. 弹出确认框，显示"将同时删除引用该节点的 N 条路网边"
3. 确认后调用 `~/remove_route_junction`，RouteGraphManager 内部级联删除关联边
4. Marker 更新

> **注意**：NavPoint 由 `NavPointTab` 管理，此工具不可删除 NavPoint。NavPoint 被删除时，路网边通过 EventBus 自动级联清理（见第八节）。

### 7.3 AddEdgeTool（添加直线边）

**交互流程**：
1. 激活后弹前置属性对话框（填写 bidirectional/max_speed/xy_tolerance/lane_width/obs_policy）
2. 在地图上点击第一个端点：
   - 悬停到 NavPoint 附近（半径 0.3m）→ 蓝色圆环高亮，点击后 `from = {id: "name", type: "nav"}`
   - 悬停到 Junction 附近 → 灰色方框高亮，点击后 `from = {id: "jct_x", type: "junction"}`
3. 点击第二个端点（同上逻辑）
4. 确认后调用 `~/add_route_edge`（后端校验两端节点均存在，否则返回错误），工具退出

### 7.4 AddCurveEdgeTool（添加贝塞尔曲线边）

**交互流程**：
1. 激活后弹属性对话框（填写边属性，含可选"自动切线"复选框）
2. 点击起点端点（吸附逻辑同 AddEdgeTool，支持 NavPoint 和 Junction）
3. 点击终点端点
4. **手动控制点模式（默认）**：在起终点之间依次左键点击控制点，鼠标移动时实时刷新 Ogre 贝塞尔曲线预览；右键完成
5. **自动切线模式（勾选后）**：从两端节点 yaw 自动生成初始两个控制点（Hermite 切线），Ogre 曲线即时预览；可继续左键微调控制点，右键确认
6. 确认后调用 `~/add_route_edge`（control_points 含所有中间控制点）

### 7.5 DeleteEdgeTool（删除边）

**交互流程**：
1. 激活后，在地图上点击已有路网边 Marker（箭头高亮）
2. 弹出确认对话框，调用 `~/remove_route_edge`
3. 持久化，Marker 更新

---

## 八、EventBus 级联集成

### 8.1 NavPoint 删除 → 路网边级联

NavPoint 被删除时（`POINT_REMOVED` 事件），`MapAssetManagerNode` 同时通知 `RouteManager` 和 `RouteGraphManager`：

```cpp
// MapAssetManagerNode::initialize()
event_bus_->subscribe(EventType::POINT_REMOVED, [this](const Event& e) {
    route_manager_->onPointRemoved(e.name);
    route_graph_manager_->onNavPointRemoved(e.name);  // 新增
});

// RouteGraphManager::onNavPointRemoved()
void RouteGraphManager::onNavPointRemoved(const std::string& nav_name) {
    // 删除所有 from.type=="nav"&&from.id==nav_name 或 to.type=="nav"&&to.id==nav_name 的边
    auto removed = edge_store_->removeEdgesContainingNav(nav_name);
    if (!removed.empty()) {
        save_fn_();
        publishMarkers();
        RCLCPP_INFO(..., "Cascaded: removed %zu route edges referencing nav '%s'",
                    removed.size(), nav_name.c_str());
    }
}
```

### 8.2 Junction 删除 → 路网边级联（内部处理，无需 EventBus）

Junction 的整个生命周期由 `RouteGraphManager` 内部管理，删除时直接调用边级联逻辑，不经过 EventBus：

```cpp
// RouteGraphManager::removeJunctionCallback()
void RouteGraphManager::removeJunctionCallback(...) {
    const std::string& id = req->id;
    auto removed_edges = edge_store_->removeEdgesContainingJunction(id);
    junction_store_->remove(id);
    save_fn_();
    publishMarkers();
    res->success = true;
    res->message = "Removed junction and " + std::to_string(removed_edges.size()) + " edges";
}
```

---

## 九、代码目录结构

```
map_asset_manager/
├── include/map_asset_manager/
│   └── route_graph_manager/
│       ├── junction_store.hpp       # Junction 数据层（CRUD）
│       ├── route_edge_store.hpp     # 边数据层（CRUD + removeEdgesContainingNav/Junction）
│       └── route_graph_manager.hpp  # 服务节点（持有 PointStore 引用用于端点校验）
├── src/route_graph_manager/
│   ├── junction_store.cpp
│   ├── route_edge_store.cpp
│   └── route_graph_manager.cpp
└── rviz_plugins/
    ├── include/map_asset_rviz_plugin/
    │   ├── route_graph_tab.hpp       # 路网边列表 + 刷新
    │   ├── add_junction_tool.hpp
    │   ├── delete_junction_tool.hpp
    │   ├── add_edge_tool.hpp
    │   ├── add_curve_edge_tool.hpp
    │   └── delete_edge_tool.hpp
    └── src/
        ├── route_graph_tab.cpp
        ├── add_junction_tool.cpp
        ├── delete_junction_tool.cpp
        ├── add_edge_tool.cpp
        ├── add_curve_edge_tool.cpp
        └── delete_edge_tool.cpp
```

---

## 十、实施路线

### Phase 1：数据层（1～2 天）

- [ ] 在 `nav_interfaces` 新增 `RouteEndpoint.msg`、`RouteJunction.msg`、`RouteEdge.msg`
- [ ] 新增 `AddRouteJunction.srv`、`RemoveRouteJunction.srv`、`AddRouteEdge.srv`、`RemoveRouteEdge.srv`、`UpdateRouteEdge.srv`、`GetRouteGraph.srv`
- [ ] 实现 `JunctionStore`（CRUD）
- [ ] 实现 `RouteEdgeStore`（CRUD + `removeEdgesContainingNav` + `removeEdgesContainingJunction`）
- [ ] 扩展 `FileManager` 支持 `route_graph.junctions` + `route_graph.edges` JSON 读写
- [ ] 单元测试（JunctionStore / RouteEdgeStore）

### Phase 2：服务层（1～2 天）

- [ ] 实现 `RouteGraphManager` 节点（6 个服务 + 1 个 Marker 话题），构造注入 `PointStore` 引用用于端点 nav 类型校验
- [ ] `MapAssetManagerNode` 注入 `route_graph_manager_`，EventBus 补充 `POINT_REMOVED` → `onNavPointRemoved()` 订阅
- [ ] 集成测试（AddRouteEdge 两端校验 / Junction 级联删除 / NavPoint 删除级联删除路网边）

### Phase 3：RViz 插件（2～3 天）

- [ ] `AddEdgeTool`：吸附逻辑 + 属性对话框 + 直线边提交
- [ ] `AddCurveEdgeTool`：自动切线 + 手动控制点 + 贝塞尔预览
- [ ] `DeleteEdgeTool`：边选中高亮 + 删除确认
- [ ] `RouteGraphTab`：路网边列表 + 详情显示 + 刷新

### Phase 4：文档与迁移（0.5 天）

- [ ] 更新 `map_asset_manager_api.md`（新增 RouteGraph 章节）
- [ ] 更新 `architecture_reference.md`
- [ ] 编写从 `create_waypoint` map.json 迁移到 `map_assets.json` 的转换脚本（Python，一次性工具）

---

## 十一、与现有架构兼容性

| 维度 | 兼容性说明 |
|------|-----------|
| `map_assets.json` | 向后兼容：新增顶级 key `route_graph`，旧文件不含该 key 时路网为空 |
| NavPoint 类型 | `JUNCTION` 新增常量，不影响 `STANDARD`/`CHARGING`/`PARKING` 的现有行为 |
| EventBus | 订阅同一 `POINT_REMOVED` 事件，无接口变更 |
| RViz 插件 | `RouteGraphTab` 作为第 4 个标签页，不改动现有 3 个标签 |
| `create_waypoint` | 迁移脚本一次性转换旧 `map.json`，不需要运行旧包 |

---

## 十二、实现阶段记录（2026-04-14 → 2026-05-07）

> 本节记录设计方案在实际开发中的调整、踩坑与最终实现形态，是对第七至十节的补充与修正。

### 12.1 工具集调整（vs 原设计方案）

| 原设计 | 实际实现 | 变更原因 |
|---|---|---|
| `DeleteJunctionTool` + `DeleteEdgeTool`（各独立）| `RouteGraphDeleteTool`（统一删除） | 悬停高亮可自动判断目标类型，无需两个工具 |
| 无编辑状态控制工具 | `RouteGraphStartTool [R]` + `RouteGraphSaveTool [S]` | 需显式编辑生命周期，防止误操作 |

**最终工具清单（6 个）**：
- `AddJunctionTool`：左键单击放置 → 对话框填 ID/yaw → 调用 `add_route_junction`
- `AddEdgeTool`：左键选起点 → 左键选终点 → 对话框 → 调用 `add_route_edge`；终点自动成为下一段起点（链式连接）；右键取消当前段；再次右键退出
- `AddCurveEdgeTool`：左键选起终点 → 右键确认（对话框 → 提交）
- `RouteGraphDeleteTool`：悬停高亮最近 junction 或 edge → Del 键删除；右键退出
- `RouteGraphStartTool [R]`：设 `s_route_graph_editing = true`，异步加载路网数据
- `RouteGraphSaveTool [S]`：检查 `s_route_graph_editing`，调用 `save_assets`，设标志为 false

### 12.2 编辑模式标志（s_route_graph_editing）

```cpp
// route_graph_tools_common.hpp
inline bool s_route_graph_editing = false;  // C++17 inline，同一 .so 内唯一实例
```

**生命周期**：

```
[R 激活] → s_route_graph_editing = true → 异步加载路网数据
                ↓
[用户操作：AddJunction / AddEdge / AddCurveEdge / Delete]
（每个工具 activate() 和 processMouseEvent() 首先检查此标志，若 false 则早返回）
                ↓
[S 激活] → 检查 s_route_graph_editing → save_assets() → s_route_graph_editing = false
```

### 12.3 ROS 回调 → Qt 主线程安全模式（标准范式）

所有路网工具的 `async_send_request` 回调均遵循以下模式：

```cpp
cli->async_send_request(req,
    [this, cli](SomeSrv::SharedFuture fut) {
        auto resp = fut.get();
        // ① 只处理 ROS 数据，不操作 Ogre/Qt 对象

        QMetaObject::invokeMethod(this, [this, /* 捕获需要的数据 */]() {
            // ② 在 Qt 主线程中操作 Ogre/Qt
            refreshPreview();
            context_->queueRender();
        }, Qt::QueuedConnection);
    });
```

**关键约束**：
- RViz2 `Tool` 类不继承 `QObject`，无法 `emit` 信号
- `QueuedConnection` 保证 lambda 在下一次 Qt 事件循环执行
- 避免在回调中创建 `std::thread`（可能引发竞争）

### 12.4 loadEndpointsAsync 设计

解决"两个独立异步请求结果合并"问题的 common 工具函数：

```cpp
// route_graph_tools_common.hpp
struct EndpointInfo { std::string id, type; float x, y; };

inline void loadEndpointsAsync(
    rclcpp::Node::SharedPtr node,
    const std::string& prefix,
    std::function<void(std::vector<EndpointInfo>)> on_done)
{
    struct State {
        std::mutex mu;
        std::vector<EndpointInfo> eps;
        int done_count = 0;
    };
    auto state = std::make_shared<State>();

    // 并行发起两个请求
    // 请求1：GetAllNavPoints → 填 type="nav"
    // 请求2：GetRouteGraph.junctions → 填 type="junction"
    // 任一回调完成后检查 done_count，两者均完成则调用 on_done(eps)
}
```

**使用方**：
- `AddEdgeTool::activate()` → `loadEndpointsAsync(..., [this](eps){ all_endpoints_ = eps; })`
- `AddCurveEdgeTool::activate()` → 同上
- `RouteGraphDeleteTool::reloadGraph()` → 合并结果后再发起 `GetRouteGraph` 获取边列表

### 12.5 Marker 渲染方案（publishMarkers 实现形态）

| 对象 | Marker 类型 | 颜色 | 尺寸 | 附加 |
|---|---|---|---|---|
| NavPoint | `SPHERE` | 紫色 (r=0.7, g=0.1, b=0.9) | 直径 0.20m | `TEXT_VIEW_FACING` 淡紫色名称 @z=0.35 |
| Junction | `SPHERE` | 浅蓝色 (r=0.3, g=0.75, b=1.0) | 直径 0.25m | `TEXT_VIEW_FACING` 白色 ID @z=0.35 |
| 边（直线/曲线）| `LINE_STRIP` | 青蓝色 | 单向 0.04m / 双向 0.10m | — |
| 边方向箭头 | `ARROW`（pose 形式） | 同边颜色 | scale.x=箭头全长 | 位于路径几何中点，切线方向 |
| 边 ID 标签 | `TEXT_VIEW_FACING` | 黄色 (r=0.9, g=0.9, b=0.3) | scale.z=0.15 | 位于路径中点上方 0.25m |

**BEZIER 采样**：Bernstein 40 段，`binom(n,i) * (1-t)^(n-i) * t^i` 加权控制点

**箭头方向计算**：
- 直线（2 点路径）：`atan2(p1.y - p0.y, p1.x - p0.x)`，中点 `(p0+p1)/2`
- 曲线（N 点路径）：中间索引 `mi = size/2`，`atan2(path[mi+1].y - path[mi-1].y, ...)`

### 12.6 RouteGraphTab 功能清单

| 功能 | 实现说明 |
|---|---|
| 路网边列表 | QTableWidget，列：边 ID / from / to / 类型 / 双向 |
| 刷新 | 调用 `GetRouteGraph` 服务，更新表格 |
| Junction 列表 | 独立区域，显示 ID / x / y |
| panel 坐标显示 | `QString("[%1] x=%2 y=%3 yaw=%4").arg(...).arg(j.x, 0, 'f', 2)` — 注意：不可用 `%.2f` C 格式串与 Qt `arg()` 混用 |
