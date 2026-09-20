# create_waypoint 逻辑分析

## 1. 整体架构

```
RViz 工具（插件）
  ↓ 发布 ROS topic（create_waypoint/Points）
create_way_points 节点
  ↓ 操作内存中的 GeoJSON 数据
  ↓ 发布 visualization_msgs/MarkerArray（可视化反馈）
  ↓ 保存到 map.json 文件
```

### 通信消息 `create_waypoint/Points.msg`

```
geometry_msgs/Point[] points   # 点列表（路网点或控制点坐标）
geometry_msgs/Point  mouse_pos # 实时鼠标位置（用于预览线段）
bool                 is_stop   # false=预览/中间状态，true=提交/执行
```

`is_stop` 是整个系统的核心状态位：
- `false` → 工具还在交互中，节点只更新可视化预览
- `true`  → 工具交互结束，节点执行写入操作

---

## 2. 数据存储格式（GeoJSON FeatureCollection）

文件路径：`{map_folder}/{map_name}/map.json`

| 元素类型 | GeoJSON geometry.type | id 前缀 | 说明 |
|---|---|---|---|
| 路网点 | `Point` | `p_N` | 单个坐标点，带朝向、名称 |
| 直线边 | `LineString` | `l_N` | 两端 Point 的连线，存 startid/endid |
| 曲线边 | `MultiPoint` | `b_N` | 贝塞尔控制点序列，存 startid/endid |

**路网点属性字段：**

```json
{
  "id": "p_1",
  "name": "",
  "yaw": 0.0,
  "isyawfix": false,
  "taskid": 0,
  "offset": 0,
  "qr": { ... }   // 可选，ArUco 标记信息
}
```

**直线/曲线边公共属性字段：**

```
id, startid, endid, direct(110), maxspeed(8), minspeed(0),
lanewidth(1), obstype(0), maxtheta(1), mintheta(-1),
xytolerance(0.2), thtolerance(0.2), slowdis(2), followdis(0.4)
```

---

## 3. 全局计数器

节点启动时从文件解析已有数据，维护以下计数器（单调递增，不复用）：

| 变量 | 作用 |
|---|---|
| `point_num` | 下一个 Point 的编号（`p_{point_num}`）|
| `line_num` | 下一个 LineString 的编号（`l_{line_num}`）|
| `curve_line_num` | 下一个 MultiPoint 的编号（`b_{curve_line_num}`）|
| `json_num` | `way_points_json["features"]` 当前末尾下标 |

**is_new_waypoint 标志**：`false` 时节点拒绝所有写入操作（防误改）。只有 start 工具将其置为 `true`，save 工具将其置回 `false`。

---

## 4. 各工具详细交互逻辑

### 4.1 start 工具（LoadWay）

| 属性 | 值 |
|---|---|
| 快捷键 | `d` |
| 发布 topic | `create_waypoint/load_waypoints`（`std_msgs/Bool`）|

**交互流程：**

```
激活（点击工具栏按钮）
  → activate() 立即发布 Bool{true}
  → 节点 NewWaypointCallBack：
      读取 ROS param: /map_manager/map_name, /map_manager/map_folder
      清空内存数据（way_points_json, 所有计数器）
      读取 map.json → 解析 GeoJSON → 重置计数器到已有最大值+1
      设 is_new_waypoint = true
      发布 MarkerArray（刷新显示）
  无鼠标交互
```

**关键逻辑**：文件读取失败时，节点仍初始化一个空的 FeatureCollection，计数器从 1 开始。

---

### 4.2 save 工具（SaveWay）

| 属性 | 值 |
|---|---|
| 快捷键 | `d`（与 start 相同，实际靠工具栏区分）|
| 发布 topic | `create_waypoint/save_waypoints`（`std_msgs/Bool`）|

**交互流程：**

```
激活（点击工具栏按钮）
  → activate() 立即发布 Bool{true}
  → 节点 SaveWaypointCallBack：
      检查 is_new_waypoint==true（否则直接返回）
      读取 ROS param 获取文件路径
      设 is_new_waypoint = false
      将内存 GeoJSON 写入 map.json
      发布 MarkerArray
  无鼠标交互
```

---

### 4.3 point_tool（WayPoint）— 添加路网点

| 属性 | 值 |
|---|---|
| 快捷键 | `p` |
| 发布 topic | `create_waypoint/point`（`Points`）|

**交互流程：**

```
鼠标移动（任意）：
  发布 Points{ points=[], mouse_pos={x,y,0}, is_stop=false }
  → 节点 PointCallback：is_new_waypoint 检查 → 跳过写入
  → PublishWaypoint flag=0：不添加数据，仅刷新 marker（鼠标位置紫色球）

左键按下：
  发布 Points{ points=[{x,y,0}], is_stop=true }
  → 节点 PointCallback：
      检查 is_new_waypoint
      AddPoint("", pose)：
        检查该坐标 0.316m 范围内是否已有点（IsExistPointWayJson，阈值 sqrt(0.1)≈0.316m）
        若无重复 → 写入 GeoJSON，point_num++, json_num++
  → PublishWaypoint flag=0：刷新显示

工具切换（deactivate）：
  清空 points_（不提交）
```

**特点**：单次左键即时写入，无需右键确认，无状态积累。

---

### 4.4 line_tool（WayLine）— 添加直线边

| 属性 | 值 |
|---|---|
| 快捷键 | `c` |
| 发布 topic | `create_waypoint/line`（`Points`）|

**交互流程：**

```
初始状态：selecting_=false, points_=[]

左键按下（第 1 次）：
  selecting_ = true
  points_.push_back({x,y,0})
  发布 Points{ points=[P1], mouse_pos={x,y}, is_stop=false }
  → 节点 LineCallback：AddLines（只有1个点时调用 AddPoint，添加点但不加线）
  → PublishWaypoint flag=1, is_stop=false：显示 P1→鼠标 的预览虚线

左键按下（第 2 次及之后）：
  points_.push_back({x,y,0})
  发布 Points{ points=[P1,P2,...], is_stop=false }
  → 节点：AddLines 对相邻对添加线
  → PublishWaypoint flag=1：显示最后点→鼠标的预览线

鼠标移动（selecting_=true）：
  更新 mouse_pos，发布同上（预览线实时跟随鼠标）

右键按下（selecting_=true）：
  is_stop=true，发布最终 Points
  selecting_=false，points_.clear()
  → 节点 LineCallback：AddLines 提交所有连线
  → PublishWaypoint flag=1：刷新（is_stop=true，不再显示预览线）

工具切换（deactivate）：
  若 points_.size()>0：发布 is_stop=true（同右键，自动提交）
  清空 points_
```

**特点**：
- 每次左键都将点加入序列，自动连成折线
- 若点坐标附近没有已有路网点，节点会自动创建（AddLine 内部调用 AddPoint）
- 折线上的相邻两点自动去重检查

---

### 4.5 curve_line_tool（WayCurveLine）— 添加贝塞尔曲线边

| 属性 | 值 |
|---|---|
| 快捷键 | `c`（与 line_tool 相同，实际靠工具栏区分）|
| 发布 topic | `create_waypoint/curve_line`（`Points`）|

**交互流程（与 line_tool 完全相同的鼠标逻辑）：**

```
左键：将点加入 points_ 序列，发布 is_stop=false
  → 节点 CurveLineCallback：is_stop=false 时只刷新可视化（不写入）
  → PublishWaypoint flag=2, is_stop=false：
      显示已有点对之间的连线段（预览控制多边形）
      显示最后点→鼠标的预览线

右键（selecting_=true）：
  发布 is_stop=true
  → 节点 CurveLineCallback：is_stop=true 时调用 AddCurveLine
  → AddCurveLine：
      取 points[0] 为起点，points[n-1] 为终点
      若起/终点不存在则 AddPoint
      写入 GeoJSON MultiPoint（geometry.type="MultiPoint"）
        coordinates = 全部控制点序列（包含起点、中间点、终点）
        properties = startid/endid（起终点的 p_N ID）
  → PublishWaypoint flag=2：渲染已保存的贝塞尔曲线

工具切换（deactivate）：同 line_tool，自动提交
```

**与 line_tool 的区别：**

| 对比项 | line_tool | curve_line_tool |
|---|---|---|
| 中间提交 | 每次左键即时添加线（AddLine） | 左键只预览，不写入 |
| 最终提交 | 右键：逐段 LineString | 右键：整体 MultiPoint（贝塞尔控制点）|
| 渲染方式 | LINE_LIST（直线段）| LINE_STRIP（贝塞尔采样曲线，步长0.01）|
| 节点存储 | geometry.type="LineString" | geometry.type="MultiPoint" |

---

### 4.6 del_tool（WayDel）— 删除元素

| 属性 | 值 |
|---|---|
| 快捷键 | `d` |
| 发布 topic | `create_waypoint/del`（`Points`）|

**交互流程：**

```
左键按下：
  清空 points_，加入当前坐标 {x,y,0}
  发布 Points{ points=[{x,y,0}], is_stop=false }
  → 节点 DelCallback：is_stop=false →
      PublishWaypoint flag=3：
        GetElementName：在 GeoJSON 中找距离<0.316m 的 Point 或直线/曲线上的点
        高亮该元素（红色 selected_id）并刷新显示

Del 键按下（processKeyEvent）：
  若 points_.size()>0：
    发布 Points{ points=已有点, is_stop=true }
  清空 points_
  → 节点 DelCallback：is_stop=true → Del(root, point)
      Del 函数逻辑：
        判断点击位置是否命中 Point（0.316m）：
          是 → DelPoint：
            先 DelLines（反复调 DelLine 直到没有连接该点的线/曲线）
            再从 features 中移除该 Point
          否 → GetElementName 获取最近元素 id →
            DelElementWay(id)：按 id 直接删除对应 Feature
      发布 MarkerArray 刷新

工具切换（deactivate）：清空（不提交）
```

**删除逻辑细节：**
- 删除 Point 时会**级联删除**所有以该点为 startid 或 endid 的 LineString 和 MultiPoint
- 删除 Line/CurveLine 时只删该元素本身，不删端点
- 点击判断距离阈值：$d^2 < 0.1$，即 $d < 0.316$m
- 线段判断用点到线段投影距离（超出线段范围返回 1000，视为未命中）

---

## 5. 可视化（PublishWaypoint）

节点统一通过 `/create_waypoints`（MarkerArray）发布可视化，每次先发送 `DELETEALL` 清除旧 marker，再重建。

### 5.1 常规元素颜色

| 元素 | 未选中颜色 | 已选中/相关颜色 |
|---|---|---|
| Point（球） | 绿色 0.4m | 红色 0.8m |
| Point 名称（文字） | 绿色 | — |
| LineString（线） | 绿色 0.05m | 红色 0.1m |
| MultiPoint 曲线（线带）| 绿色 0.05m | 红色 0.1m |

### 5.2 预览元素颜色（flag 参数）

| flag | 场景 | 额外渲染 |
|---|---|---|
| 0 | point_tool | 紫色球（当前点击位置）+ 紫色球（mouse_pos）|
| 1 | line_tool（is_stop=false）| 紫色线（最后已选点→鼠标）|
| 2 | curve_line_tool（is_stop=false）| 紫色折线段（已选点序列）+ 紫色线（最后点→鼠标）|
| 3 | del_tool（is_stop=false）| GetElementName 高亮选中元素（红色）|
| 5 | 刷新（无预览）| 仅显示已保存元素 |

---

## 6. 防误改保护

```
is_new_waypoint == false（初始/保存后状态）
  → PointCallback / LineCallback / CurveLineCallback / DelCallback 全部直接返回，不写入任何数据
  → 只有 start 工具（load_waypoints）能将其设为 true
  → save 工具（save_waypoints）执行后将其置回 false
```

---

## 7. 与 map_asset_manager 版本对照

| 功能 | create_waypoint（ROS1）| map_asset_manager（ROS2）|
|---|---|---|
| 数据格式 | GeoJSON（map.json）| 服务调用（add/remove services）+ 文件持久化 |
| 通信方式 | ROS topic（Points msg）| ROS2 Service（同步/异步）|
| 点类型 | 统一 p_N | NavPoint（已命名）+ Junction（路网节点）|
| 边类型 | LineString / MultiPoint | RouteEdge（LINE / BEZIER）|
| 防误改 | is_new_waypoint 标志 | reload_data（Start）/ save_assets（Save）服务 |
| 端点 snap | 无（按坐标距离 0.316m 匹配）| 有（kSnapRadius=0.5m，activate 时加载端点列表）|
| 实时预览 | mouse_pos 字段 + flag 参数 | Ogre ManualObject（RViz 内部渲染）|
