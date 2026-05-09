# Nav Point Manager - 系统架构

## 整体架构

```
┌─────────────────────────────────────────────────────────────┐
│                  NavPointManagerNode                         │
│                     (主节点)                                  │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │ FileManager  │  │PointDatabase │  │NavigationExec│      │
│  │              │  │              │  │    utor      │      │
│  │ JSON读写     │  │ 内存数据库   │  │ Nav2调用     │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
└─────────────────────────────────────────────────────────────┘
         │                    │                    │
         ▼                    ▼                    ▼
    points.json          unordered_map         Nav2 Actions
    routes.json          (线程安全)            (navigate_to_pose)
                                               (navigate_through_poses)
```

## 数据流

### 1. 启动流程
```
main()
  → NavPointManagerNode::NavPointManagerNode()
    → initialize()
      → 创建 FileManager
      → 创建 PointDatabase
      → 创建 NavigationExecutor
      → 创建 ROS2 接口
    → loadData()
      → FileManager::loadPointsFromFile()
      → FileManager::loadRoutesFromFile()
      → PointDatabase::setPoints()
      → PointDatabase::setRoutes()
    → publishMarkers()
```

### 2. 导航到点位流程
```
用户请求
  → NavigationExecutor::navigateToPoint(name)
    → PointDatabase::getPoint(name)
    → setNav2GoalTolerance(tolerance)
    → 创建 PoseStamped
    → nav2_client_->async_send_goal()
    → 状态更新为 NAVIGATING
    → Nav2 执行导航
    → resultCallback()
      → 状态更新为 SUCCESS/FAILED/CANCELED
```

### 3. 路网导航流程
```
用户请求
  → NavigationExecutor::navigateRoute(route_name)
    → PointDatabase::getRoute(route_name)
    → 遍历 point_names
      → PointDatabase::getPoint(point_name)
      → 创建 PoseStamped 列表
    → nav2_through_poses_client_->async_send_goal()
    → 状态更新为 NAVIGATING
    → Nav2 执行路网导航
    → resultCallbackThroughPoses()
      → 状态更新为 SUCCESS/FAILED/CANCELED
```

### 4. 热加载流程
```
用户调用 reload_points 服务
  → reloadPointsCallback()
    → loadData()
      → FileManager::loadPointsFromFile()
      → FileManager::loadRoutesFromFile()
      → PointDatabase::setPoints()
      → PointDatabase::setRoutes()
    → publishMarkers()
```

## 核心类详解

### 1. FileManager
**职责：** JSON文件的读写操作

**关键方法：**
- `loadPointsFromFile()` - 从JSON加载点位
- `savePointsToFile()` - 保存点位到JSON
- `loadRoutesFromFile()` - 从JSON加载路网
- `saveRoutesToFile()` - 保存路网到JSON

**线程安全：** 使用 `file_mutex_` 保护文件操作

### 2. PointDatabase
**职责：** 内存中的点位和路网数据库

**数据结构：**
```cpp
std::unordered_map<std::string, NavPoint> points_map_;
std::unordered_map<std::string, Route> routes_map_;
```

**关键方法：**
- `addPoint()` / `removePoint()` / `updatePoint()` / `getPoint()`
- `addRoute()` / `removeRoute()` / `getRoute()`
- `getAllPoints()` / `getAllRoutes()`

**线程安全：** 使用 `data_mutex_` 保护所有数据访问

### 3. NavigationExecutor
**职责：** 执行导航任务，与Nav2交互

**关键方法：**
- `navigateToPoint()` - 导航到指定点位
- `navigateToPose()` - 导航到指定姿态
- `navigateRoute()` - 执行路网导航
- `cancelNavigation()` - 取消当前导航

**状态管理：**
```cpp
enum class NavState {
    IDLE,
    NAVIGATING,
    SUCCESS,
    FAILED,
    CANCELED
};
```

**线程安全：** 使用 `state_mutex_` 保护状态

### 4. NavPointManagerNode
**职责：** 主节点，整合所有组件，提供ROS2接口

**ROS2接口：**
- Services: `~/reload_points`
- Topics: `~/nav_points_marker`, `~/navigation_status`

**定时器：**
- `marker_timer_` - 定期发布可视化标记（5秒）
- `status_timer_` - 定期发布导航状态（500ms）

## 线程模型

```
MultiThreadedExecutor
  │
  ├─ Service Callbacks (独立线程)
  │   └─ reloadPointsCallback()
  │
  ├─ Timer Callbacks (独立线程)
  │   ├─ publishMarkers()
  │   └─ statusTimerCallback()
  │
  └─ Action Callbacks (独立线程)
      ├─ goalResponseCallback()
      ├─ feedbackCallback()
      └─ resultCallback()
```

所有共享数据访问都通过 mutex 保护，确保线程安全。

## 扩展点

### 1. 添加新的点位类型
在 `types.hpp` 中定义，在 `getColorForType()` 中添加颜色映射。

### 2. 添加新的服务接口
在 `NavPointManagerNode` 中添加服务定义和回调函数。

### 3. 自定义导航策略
在 `NavigationExecutor` 中扩展导航逻辑。

### 4. 持久化其他数据
在 `FileManager` 中添加新的加载/保存方法。

## 性能考虑

1. **查找效率：** 使用 `unordered_map` 提供 O(1) 查找
2. **并发性能：** 使用细粒度锁，减少锁竞争
3. **内存占用：** 点位数据在内存中，适合中小规模应用
4. **I/O性能：** 文件操作异步进行，不阻塞主线程

## 错误处理

1. **文件不存在：** 返回错误，记录日志
2. **JSON格式错误：** 捕获异常，返回错误信息
3. **点位不存在：** 返回 `std::nullopt`
4. **Nav2连接失败：** 超时返回，记录错误
5. **导航失败：** 状态更新为 FAILED，发布状态消息

## 测试策略

1. **单元测试：** 测试各个类的独立功能
2. **集成测试：** 测试组件间的交互
3. **端到端测试：** 测试完整的导航流程
4. **压力测试：** 测试并发访问和大量数据

## 未来优化方向

1. 使用数据库替代JSON文件（如SQLite）
2. 添加点位验证（地图边界检查）
3. 支持点位分组和标签
4. 添加导航历史记录
5. 支持多机器人协调
6. 添加Web界面进行可视化管理
