# map_asset_manager 接口文档

节点名称：`map_asset_manager`

所有服务/Action 的完整路径均以 `/map_asset_manager/` 为前缀。

---

## 前置条件

节点启动后会等待话题 `map_manager/map_name`（transient_local QoS）。收到地图名称后，自动从
`<map_base_folder>/<map_name>/map_assets.json` 加载并持久化三类资产（点位、路线、区域）。

---

## 一、NavPointManager — 导航点管理

### 1.1 服务接口

#### add_point — 手动添加导航点

```
服务名：/map_asset_manager/add_point
类型：nav_interfaces/srv/AddNavPoint
```

| 字段 | 类型 | 说明 |
|------|------|------|
| point.name | string | 点位名称（唯一键）|
| point.type | string | 类型：STATION / CHARGING / PARKING / PICKUP / LIFT |
| point.x | float32 | 地图坐标 x（米）|
| point.y | float32 | 地图坐标 y（米）|
| point.theta | float32 | 朝向（弧度）|
| point.tolerance | float32 | 到达精度（米），0 表示使用默认值 0.05 m |

**示例：添加一个充电站点**

```bash
ros2 service call /map_asset_manager/add_point nav_interfaces/srv/AddNavPoint \
  "{point: {name: 'charging_1', type: 'CHARGING', x: 1.5, y: 2.3, theta: 0.0, tolerance: 0.1}}"
```

---

#### sample_point — 采样当前机器人位姿为导航点

从 TF（`map` → `base_link`）连续采样 11 次取平均，自动记录当前机器人位置。

```
服务名：/map_asset_manager/sample_point
类型：nav_interfaces/srv/SampleNavPoint
```

| 字段 | 类型 | 说明 |
|------|------|------|
| name | string | 点位名称 |
| point_type | uint32 | 1=STATION, 2=CHARGING, 3=PARKING, 4=PICKUP, 5=LIFT（不支持 0=FREE_GOAL）|

**示例：在当前位置采样一个停靠站**

```bash
ros2 service call /map_asset_manager/sample_point nav_interfaces/srv/SampleNavPoint \
  "{name: 'station_A', point_type: 1}"
```

---

#### remove_point — 删除导航点

删除后会自动级联更新所有包含该点的路线（从路线点列表中移除该点）。

```
服务名：/map_asset_manager/remove_point
类型：nav_interfaces/srv/RemoveNavPoint
```

```bash
ros2 service call /map_asset_manager/remove_point nav_interfaces/srv/RemoveNavPoint \
  "{name: 'station_A'}"
```

---

#### update_point — 更新导航点信息

```
服务名：/map_asset_manager/update_point
类型：nav_interfaces/srv/UpdateNavPoint
```

```bash
ros2 service call /map_asset_manager/update_point nav_interfaces/srv/UpdateNavPoint \
  "{point: {name: 'station_A', type: 'STATION', x: 2.0, y: 3.0, theta: 1.57, tolerance: 0.05}}"
```

---

#### get_point — 查询单个导航点

```
服务名：/map_asset_manager/get_point
类型：nav_interfaces/srv/GetNavPoint
```

```bash
ros2 service call /map_asset_manager/get_point nav_interfaces/srv/GetNavPoint \
  "{name: 'station_A'}"
```

---

#### get_all_points — 查询所有导航点

```
服务名：/map_asset_manager/get_all_points
类型：nav_interfaces/srv/GetAllNavPoints
```

```bash
ros2 service call /map_asset_manager/get_all_points nav_interfaces/srv/GetAllNavPoints "{}"
```

---

#### get_status — 查询当前导航状态

```
服务名：/map_asset_manager/get_status
类型：nav_interfaces/srv/GetStatus
```

返回字段：
- `success`：是否有效
- `name`：当前导航目标点名称
- `status`：IDLE / NAVIGATING / SUCCESS / FAILED / CANCELED

```bash
ros2 service call /map_asset_manager/get_status nav_interfaces/srv/GetStatus "{}"
```

---

### 1.2 Action 接口

#### navigate_to_point — 导航到指定点位

```
Action 名：/map_asset_manager/navigate_to_point
类型：nav_interfaces/action/NavigateToPoint
```

**Goal 字段：**

| 字段 | 类型 | 说明 |
|------|------|------|
| point_name | string | 目标点名称（优先使用）|
| use_pose | bool | 为 true 时使用下方 pose 字段 |
| pose | geometry_msgs/PoseStamped | 自定义位姿（point_name 为空时有效）|
| tolerance | float64 | 自定义精度（仅 use_pose 时有效）|

**Feedback：** `current_status`（NAVIGATING / SUCCESS / FAILED / CANCELED）

**Result：** `success`, `message`

**示例：按名称导航**

```bash
ros2 action send_goal /map_asset_manager/navigate_to_point \
  nav_interfaces/action/NavigateToPoint \
  "{point_name: 'station_A', use_pose: false}"
```

**示例：用自定义位姿导航**

```bash
ros2 action send_goal /map_asset_manager/navigate_to_point \
  nav_interfaces/action/NavigateToPoint \
  "{point_name: '', use_pose: true, pose: {header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: 1.5, z: 0.0}, orientation: {w: 1.0}}}, tolerance: 0.1}"
```

**示例：发送目标并实时查看反馈**

```bash
ros2 action send_goal --feedback /map_asset_manager/navigate_to_point \
  nav_interfaces/action/NavigateToPoint \
  "{point_name: 'charging_1', use_pose: false}"
```

**取消导航：**

```bash
# 先查找 goal_id，再取消
ros2 action list  # 查看活跃 action
# 或在 send_goal 终端按 Ctrl+C（发送 cancel）
```

---

### 1.3 话题

| 话题                                          | 类型                                   | 说明                         |     |
| ------------------------------------------- | ------------------------------------ | -------------------------- | --- |
| `/map_asset_manager/nav_points_marker` | `visualization_msgs/msg/MarkerArray` | 所有点位的 RViz 可视化（圆柱体 + 文字标签） |     |

颜色对照：
- CHARGING → 绿色
- STATION → 蓝色
- PICKUP → 橙色
- PARKING → 灰色

---

## 二、RouteManager — 路线管理

路线是一组有序导航点名称的集合，执行时按顺序依次通过所有点。

### 2.1 服务接口

#### add_route — 添加路线

```
服务名：/map_asset_manager/add_route
类型：nav_interfaces/srv/AddRoute
```

要求：`point_names` 中所有点位必须已存在于 NavPointManager 中。

```bash
ros2 service call /map_asset_manager/add_route nav_interfaces/srv/AddRoute \
  "{name: 'patrol_route', point_names: ['station_A', 'station_B', 'charging_1']}"
```

---

#### remove_route — 删除路线

```
服务名：/map_asset_manager/remove_route
类型：nav_interfaces/srv/RemoveRoute
```

```bash
ros2 service call /map_asset_manager/remove_route nav_interfaces/srv/RemoveRoute \
  "{name: 'patrol_route'}"
```

---

#### get_route — 查询路线

```
服务名：/map_asset_manager/get_route
类型：nav_interfaces/srv/GetRoute
```

返回字段：`success`, `name`, `point_names`（有序点名列表）

```bash
ros2 service call /map_asset_manager/get_route nav_interfaces/srv/GetRoute \
  "{name: 'patrol_route'}"
```

---

### 2.2 Action 接口

#### navigate_route — 按路线导航

按路线中的点位顺序调用 Nav2 `navigate_through_poses`，一次性规划穿越所有点。

```
Action 名：/map_asset_manager/navigate_route
类型：nav_interfaces/action/NavigateRoute
```

**Goal：** `route_name`（路线名称）

**Feedback：**
- `current_status`：NAVIGATING / SUCCESS / FAILED / CANCELED
- `current_point_index`：当前目标点索引
- `total_points`：路线总点数

**示例：按路线巡逻**

```bash
ros2 action send_goal --feedback /map_asset_manager/navigate_route \
  nav_interfaces/action/NavigateRoute \
  "{route_name: 'patrol_route'}"
```

---

### 2.3 级联联动

| 事件 | 效果 |
|------|------|
| 删除导航点 | 自动从所有路线的点列表中移除该点 |
| 更新导航点（重命名）| 自动同步所有路线中的点名称 |

---

### 2.4 话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/map_asset_manager/route_markers` | `visualization_msgs/msg/MarkerArray` | 路线执行时的可视化（箭头 + 球体 + 序号标签）|

---

## 三、AreaManager — 区域管理

> **当前状态：框架占位，尚未实现服务接口。**
>
> 待 `nav_interfaces` 补充 Area 相关 srv/action 后实现。预计服务：
> `add_area` / `remove_area` / `update_area` / `get_area` / `get_all_areas`

**已有话题（无内容）：**
- `/map_asset_manager/area_markers`：`visualization_msgs/msg/MarkerArray`

---

## 四、节点级接口

### reload_data — 重新加载资产文件

```
服务名：/map_asset_manager/reload_data
类型：std_srvs/srv/Trigger
```

```bash
ros2 service call /map_asset_manager/reload_data std_srvs/srv/Trigger "{}"
```

### status — 节点状态

每 5 秒发布一次当前加载的地图名及各资产数量。

```bash
ros2 topic echo /map_asset_manager/status
# 输出示例：data: "map=floor_1 points=5 routes=2 areas=0"
```

---

## 五、典型工作流

### 场景：建点 → 建路线 → 执行巡逻

```bash
# Step 1：采样当前位置为 STATION 点
ros2 service call /map_asset_manager/sample_point nav_interfaces/srv/SampleNavPoint \
  "{name: 'pt_1', point_type: 1}"

# Step 2：手动添加充电点（坐标已知）
ros2 service call /map_asset_manager/add_point nav_interfaces/srv/AddNavPoint \
  "{point: {name: 'charger', type: 'CHARGING', x: 0.5, y: 0.5, theta: 3.14, tolerance: 0.1}}"

# Step 3：确认点位
ros2 service call /map_asset_manager/get_all_points nav_interfaces/srv/GetAllNavPoints "{}"

# Step 4：创建路线
ros2 service call /map_asset_manager/add_route nav_interfaces/srv/AddRoute \
  "{name: 'loop', point_names: ['pt_1', 'charger']}"

# Step 5：执行路线导航
ros2 action send_goal --feedback /map_asset_manager/navigate_route \
  nav_interfaces/action/NavigateRoute "{route_name: 'loop'}"

# Step 6：查询导航状态
ros2 service call /map_asset_manager/get_status nav_interfaces/srv/GetStatus "{}"
```

---

## 六、导航点类型常量参考

| 常量名 | uint32 值 | 字符串值 | 颜色 |
|--------|-----------|----------|------|
| STATION | 1 | "STATION" | 蓝色 |
| CHARGING | 2 | "CHARGING" | 绿色 |
| PARKING | 3 | "PARKING" | 灰色 |
| PICKUP | 4 | "PICKUP" | 橙色 |
| LIFT | 5 | "LIFT" | 白色 |

> `FREE_GOAL (0)` 仅在 `navigate_to_point` Action 的 `use_pose` 模式下使用，不可通过 `sample_point` 服务保存。
