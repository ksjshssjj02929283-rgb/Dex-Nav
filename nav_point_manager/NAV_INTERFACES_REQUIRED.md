# Nav Interfaces 消息定义

nav_point_manager 需要在 nav_interfaces 包中定义以下自定义消息、服务和动作。

## 消息定义 (msg/)

### NavPoint.msg
```
string name
string type
float64 x
float64 y
float64 theta
float64 tolerance
std_msgs/Header header
```

## 服务定义 (srv/)

### AddNavPoint.srv
```
nav_interfaces/NavPoint point
---
bool success
string message
```

### RemoveNavPoint.srv
```
string name
---
bool success
string message
```

### UpdateNavPoint.srv
```
nav_interfaces/NavPoint point
---
bool success
string message
```

### GetNavPoint.srv
```
string name
---
bool success
nav_interfaces/NavPoint point
```

### GetAllNavPoints.srv
```
---
bool success
uint32 count
nav_interfaces/NavPoint[] points
```

### AddRoute.srv
```
string name
string[] point_names
---
bool success
string message
```

### RemoveRoute.srv
```
string name
---
bool success
string message
```

### GetRoute.srv
```
string name
---
bool success
string name
string[] point_names
```

### GetStatus.srv
```
---
bool success
string status
string name
```

## 动作定义 (action/)

### NavigateToPoint.action
```
# Goal
string point_name
bool use_pose
geometry_msgs/PoseStamped pose
float64 tolerance
---
# Result
bool success
string message
---
# Feedback
string current_status
```

### NavigateRoute.action
```
# Goal
string route_name
---
# Result
bool success
string message
---
# Feedback
string current_status
uint32 current_point_index
uint32 total_points
```

## 创建这些消息的步骤

1. 在 nav_interfaces 包中创建目录结构：
```bash
cd nav_interfaces
mkdir -p msg srv action
```

2. 创建上述所有 .msg, .srv, .action 文件

3. 更新 nav_interfaces/CMakeLists.txt：
```cmake
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NavPoint.msg"
  "srv/AddNavPoint.srv"
  "srv/RemoveNavPoint.srv"
  "srv/UpdateNavPoint.srv"
  "srv/GetNavPoint.srv"
  "srv/GetAllNavPoints.srv"
  "srv/AddRoute.srv"
  "srv/RemoveRoute.srv"
  "srv/GetRoute.srv"
  "srv/GetStatus.srv"
  "action/NavigateToPoint.action"
  "action/NavigateRoute.action"
  DEPENDENCIES std_msgs geometry_msgs
)
```

4. 更新 nav_interfaces/package.xml：
```xml
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

5. 编译：
```bash
colcon build --packages-select nav_interfaces
```

## 注意事项

- 确保 nav_interfaces 包先于 nav_point_manager 编译
- 所有消息定义都遵循 ROS2 命名规范
- NavPoint 消息包含 header 字段用于时间戳和坐标系信息
