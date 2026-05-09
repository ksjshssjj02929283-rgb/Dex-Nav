# Nav Point Manager

统一的导航点管理节点，负责管理导航点、路网、JSON持久化和导航执行。

## 功能特性

1. **导航点管理（CRUD）**
   - 添加、删除、更新、查询导航点
   - 支持不同类型的点位（CHARGING, STATION, PICKUP, PARKING）
   - 每个点位可设置独立的容差

2. **路网管理**
   - 定义和管理路网（多个点位组成的路径）
   - 支持路网导航

3. **JSON持久化**
   - 自动保存点位和路网到JSON文件
   - 支持热加载

4. **导航执行**
   - 统一的导航入口
   - 调用 Nav2 进行实际导航
   - 状态管理和反馈

5. **RViz可视化**
   - 在RViz中显示所有导航点
   - 不同类型点位使用不同颜色

## 编译

```bash
cd /path/to/workspace
colcon build --packages-select nav_point_manager
source install/setup.bash
```

## 运行

```bash
ros2 launch nav_point_manager nav_point_manager.launch.py
```

## 配置文件

### points.json
```json
{
  "points": [
    {
      "name": "station_A",
      "type": "STATION",
      "tolerance": 0.3,
      "pose": {
        "x": 3.2,
        "y": 5.1,
        "yaw": 1.57
      }
    }
  ]
}
```

### routes.json
```json
{
  "routes": [
    {
      "name": "route_A",
      "points": ["station_A", "station_B"]
    }
  ]
}
```

## ROS2 接口

### Services

- `~/reload_points` (std_srvs/Trigger) - 重新加载点位和路网文件

### Topics

- `~/nav_points_marker` (visualization_msgs/MarkerArray) - 点位可视化标记
- `~/navigation_status` (std_msgs/String) - 导航状态

## 点位类型和颜色

- **CHARGING** - 红色
- **STATION** - 蓝色
- **PICKUP** - 绿色
- **PARKING** - 黄色

## 依赖

- ROS2 Humble
- Nav2
- nlohmann/json
- nav_interfaces (自定义消息包)

## 架构

```
nav_point_manager/
├── include/nav_point_manager/
│   ├── types.hpp              # 数据结构定义
│   ├── file_manager.hpp       # JSON文件管理
│   ├── point_database.hpp     # 点位数据库
│   ├── navigation_executor.hpp # 导航执行器
│   └── nav_point_manager_node.hpp # 主节点
├── apps/
│   ├── file_manager.cpp
│   ├── point_database.cpp
│   ├── navigation_executor.cpp
│   ├── nav_point_manager_node.cpp
│   └── main.cpp
├── config/
│   ├── points.json            # 点位配置
│   └── routes.json            # 路网配置
├── launch/
│   └── nav_point_manager.launch.py
└── params/
    └── nav_point_manager_params.yaml
```

## 线程安全

所有数据访问都使用 mutex 保护，确保线程安全。
