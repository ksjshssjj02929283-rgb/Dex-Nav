# Nav Point Manager - 快速开始指南

## 系统概述

nav_point_manager 是一个统一的导航点管理系统，提供：
- 导航点的增删改查
- 路网管理
- JSON持久化存储
- 与 Nav2 的集成
- RViz 可视化

## 安装依赖

### 1. 安装 nlohmann/json

```bash
sudo apt-get update
sudo apt-get install nlohmann-json3-dev
```

### 2. 确保已安装 Nav2

```bash
sudo apt-get install ros-humble-nav2-*
```

## 编译

```bash
cd /home/dex/workspace_pzc/dexe_mobile_application/mobile_manager
colcon build --packages-select nav_point_manager
source install/setup.bash
```

## 配置

### 1. 编辑点位文件 (config/points.json)

```json
{
  "points": [
    {
      "name": "home",
      "type": "STATION",
      "tolerance": 0.2,
      "pose": {
        "x": 0.0,
        "y": 0.0,
        "yaw": 0.0
      }
    },
    {
      "name": "charging",
      "type": "CHARGING",
      "tolerance": 0.1,
      "pose": {
        "x": 5.0,
        "y": 5.0,
        "yaw": 1.57
      }
    }
  ]
}
```

### 2. 编辑路网文件 (config/routes.json)

```json
{
  "routes": [
    {
      "name": "patrol",
      "points": ["home", "charging", "home"]
    }
  ]
}
```

## 运行

### 启动节点

```bash
ros2 launch nav_point_manager nav_point_manager.launch.py
```

### 指定自定义配置文件

```bash
ros2 launch nav_point_manager nav_point_manager.launch.py \
  points_file:=/path/to/your/points.json \
  routes_file:=/path/to/your/routes.json
```

## 使用示例

### 1. 查看导航状态

```bash
ros2 topic echo /nav_point_manager_node/navigation_status
```

### 2. 重新加载配置

```bash
ros2 service call /nav_point_manager_node/reload_points std_srvs/srv/Trigger
```

### 3. 在 RViz 中查看点位

```bash
rviz2
```

添加 MarkerArray 显示，topic 设置为：`/nav_point_manager_node/nav_points_marker`

## 代码结构

```
nav_point_manager/
├── include/nav_point_manager/
│   ├── types.hpp                    # 数据结构（NavPoint, Route, NavState）
│   ├── file_manager.hpp             # JSON 文件读写
│   ├── point_database.hpp           # 内存数据库（线程安全）
│   ├── navigation_executor.hpp      # 导航执行器（调用 Nav2）
│   └── nav_point_manager_node.hpp   # 主节点
│
├── apps/
│   ├── file_manager.cpp
│   ├── point_database.cpp
│   ├── navigation_executor.cpp
│   ├── nav_point_manager_node.cpp
│   └── main.cpp                     # 主函数
│
├── config/
│   ├── points.json                  # 点位配置
│   └── routes.json                  # 路网配置
│
├── launch/
│   └── nav_point_manager.launch.py  # 启动文件
│
└── params/
    └── nav_point_manager_params.yaml # 参数配置
```

## 核心组件说明

### 1. FileManager
- 负责 JSON 文件的读写
- 线程安全的文件操作
- 支持点位和路网的序列化/反序列化

### 2. PointDatabase
- 使用 unordered_map 存储点位和路网
- 提供线程安全的 CRUD 操作
- 支持批量操作

### 3. NavigationExecutor
- 封装 Nav2 的 Action 调用
- 管理导航状态
- 支持单点导航和路网导航
- 动态设置导航容差

### 4. NavPointManagerNode
- 主节点，整合所有组件
- 提供 ROS2 服务接口
- 发布可视化标记
- 发布导航状态

## 扩展开发

### 添加新的服务接口

1. 在 `nav_point_manager_node.hpp` 中声明服务
2. 在 `nav_point_manager_node.cpp` 中实现回调
3. 在 `initialize()` 中创建服务

### 添加新的点位类型

1. 在 `config/points.json` 中使用新类型
2. 在 `getColorForType()` 中添加颜色映射

## 故障排查

### 编译错误：找不到 nlohmann/json

```bash
sudo apt-get install nlohmann-json3-dev
```

### 运行时错误：找不到配置文件

检查 launch 文件中的路径配置，确保文件存在。

### Nav2 连接失败

确保 Nav2 已经启动：
```bash
ros2 launch nav2_bringup navigation_launch.py
```

## 性能优化

- 所有数据访问都使用 mutex 保护
- 使用 unordered_map 提供 O(1) 查找
- 异步 Action 调用避免阻塞
- 定时器控制可视化更新频率

## 未来改进

- [ ] 添加 Action 接口用于导航
- [ ] 支持动态添加/删除点位的服务
- [ ] 添加点位验证（检查是否在地图内）
- [ ] 支持点位分组管理
- [ ] 添加导航历史记录
- [ ] 支持多机器人协调

## 许可证

Apache License 2.0
