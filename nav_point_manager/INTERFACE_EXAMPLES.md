# Nav Point Manager - 接口使用示例

本文档展示如何使用 nav_point_manager 提供的所有 ROS2 接口。

## 1. 点位管理服务

### 1.1 添加点位
```bash
ros2 service call /nav_point_manager_node/add_point nav_interfaces/srv/AddNavPoint \
"{point: {name: 'station_C', type: 'STATION', x: 10.0, y: 5.0, theta: 0.0, tolerance: 0.2}}"
```

### 1.2 删除点位
```bash
ros2 service call /nav_point_manager_node/remove_point nav_interfaces/srv/RemoveNavPoint \
"{name: 'station_C'}"
```

### 1.3 更新点位
```bash
ros2 service call /nav_point_manager_node/update_point nav_interfaces/srv/UpdateNavPoint \
"{point: {name: 'station_A', type: 'STATION', x: 3.5, y: 5.2, theta: 1.6, tolerance: 0.25}}"
```

### 1.4 查询单个点位
```bash
ros2 service call /nav_point_manager_node/get_point nav_interfaces/srv/GetNavPoint \
"{name: 'station_A'}"
```

### 1.5 获取所有点位
```bash
ros2 service call /nav_point_manager_node/get_all_points nav_interfaces/srv/GetAllNavPoints
```

## 2. 路网管理服务

### 2.1 添加路网
```bash
ros2 service call /nav_point_manager_node/add_route nav_interfaces/srv/AddRoute \
"{name: 'patrol_route', point_names: ['station_A', 'station_B', 'charging_point']}"
```

### 2.2 删除路网
```bash
ros2 service call /nav_point_manager_node/remove_route nav_interfaces/srv/RemoveRoute \
"{name: 'patrol_route'}"
```

### 2.3 查询路网
```bash
ros2 service call /nav_point_manager_node/get_route nav_interfaces/srv/GetRoute \
"{name: 'route_A'}"
```

## 3. 系统管理服务

### 3.1 重新加载配置
```bash
ros2 service call /nav_point_manager_node/reload_points std_srvs/srv/Trigger
```

### 3.2 查询导航状态
```bash
ros2 service call /nav_point_manager_node/get_status nav_interfaces/srv/GetStatus
```

## 4. 导航 Action

### 4.1 导航到指定点位（使用点位名称）
```bash
ros2 action send_goal /nav_point_manager_node/navigate_to_point \
nav_interfaces/action/NavigateToPoint \
"{point_name: 'station_A', use_pose: false}"
```

### 4.2 导航到自定义姿态
```bash
ros2 action send_goal /nav_point_manager_node/navigate_to_point \
nav_interfaces/action/NavigateToPoint \
"{point_name: '', use_pose: true, pose: {header: {frame_id: 'map'}, \
pose: {position: {x: 5.0, y: 3.0, z: 0.0}, orientation: {w: 1.0}}}, tolerance: 0.15}"
```

### 4.3 路网导航
```bash
ros2 action send_goal /nav_point_manager_node/navigate_route \
nav_interfaces/action/NavigateRoute \
"{route_name: 'patrol_route'}"
```

### 4.4 取消导航
```bash
# 获取 action goal ID
ros2 action list -v

# 取消指定的 goal
ros2 action send_goal --feedback /nav_point_manager_node/navigate_to_point \
nav_interfaces/action/NavigateToPoint "{point_name: 'station_A'}" &

# 然后取消
# Ctrl+C 或使用 action cancel 命令
```

## 5. Topic 订阅

### 5.1 订阅导航状态
```bash
ros2 topic echo /nav_point_manager_node/navigation_status
```

### 5.2 在 RViz 中查看点位标记
```bash
# 启动 RViz
rviz2

# 添加 MarkerArray 显示
# Topic: /nav_point_manager_node/nav_points_marker
```

## 6. Python 示例

### 6.1 添加点位（Python）
```python
import rclpy
from rclpy.node import Node
from nav_interfaces.srv import AddNavPoint
from nav_interfaces.msg import NavPoint

class PointAdder(Node):
    def __init__(self):
        super().__init__('point_adder')
        self.client = self.create_client(AddNavPoint, '/nav_point_manager_node/add_point')

    def add_point(self, name, x, y, theta, point_type='STATION', tolerance=0.2):
        request = AddNavPoint.Request()
        request.point.name = name
        request.point.type = point_type
        request.point.x = x
        request.point.y = y
        request.point.theta = theta
        request.point.tolerance = tolerance

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Result: {response.success}, {response.message}')
        else:
            self.get_logger().error('Service call failed')

def main():
    rclpy.init()
    adder = PointAdder()
    adder.add_point('test_point', 1.0, 2.0, 0.0)
    adder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 6.2 导航到点位（Python）
```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_interfaces.action import NavigateToPoint

class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        self.action_client = ActionClient(
            self, NavigateToPoint, '/nav_point_manager_node/navigate_to_point')

    def navigate(self, point_name):
        goal_msg = NavigateToPoint.Goal()
        goal_msg.point_name = point_name
        goal_msg.use_pose = False

        self.action_client.wait_for_server()

        send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        self.get_logger().info(f'Result: {result.success}, {result.message}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Status: {feedback.current_status}')

def main():
    rclpy.init()
    navigator = Navigator()
    navigator.navigate('station_A')
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 7. C++ 示例

### 7.1 添加点位（C++）
```cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_interfaces/srv/add_nav_point.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("point_adder");

    auto client = node->create_client<nav_interfaces::srv::AddNavPoint>(
        "/nav_point_manager_node/add_point");

    auto request = std::make_shared<nav_interfaces::srv::AddNavPoint::Request>();
    request->point.name = "test_point";
    request->point.type = "STATION";
    request->point.x = 1.0;
    request->point.y = 2.0;
    request->point.theta = 0.0;
    request->point.tolerance = 0.2;

    auto future = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        RCLCPP_INFO(node->get_logger(), "Result: %s, %s",
                   response->success ? "true" : "false",
                   response->message.c_str());
    }

    rclcpp::shutdown();
    return 0;
}
```

## 8. 完整工作流示例

```bash
# 1. 启动节点
ros2 launch nav_point_manager nav_point_manager.launch.py

# 2. 添加几个点位
ros2 service call /nav_point_manager_node/add_point nav_interfaces/srv/AddNavPoint \
"{point: {name: 'home', type: 'STATION', x: 0.0, y: 0.0, theta: 0.0, tolerance: 0.2}}"

ros2 service call /nav_point_manager_node/add_point nav_interfaces/srv/AddNavPoint \
"{point: {name: 'work', type: 'STATION', x: 5.0, y: 5.0, theta: 1.57, tolerance: 0.2}}"

# 3. 创建路网
ros2 service call /nav_point_manager_node/add_route nav_interfaces/srv/AddRoute \
"{name: 'commute', point_names: ['home', 'work']}"

# 4. 查看所有点位
ros2 service call /nav_point_manager_node/get_all_points nav_interfaces/srv/GetAllNavPoints

# 5. 导航到点位
ros2 action send_goal /nav_point_manager_node/navigate_to_point \
nav_interfaces/action/NavigateToPoint "{point_name: 'work', use_pose: false}"

# 6. 执行路网导航
ros2 action send_goal /nav_point_manager_node/navigate_route \
nav_interfaces/action/NavigateRoute "{route_name: 'commute'}"

# 7. 查询状态
ros2 service call /nav_point_manager_node/get_status nav_interfaces/srv/GetStatus
```

## 9. 故障排查

### 服务不可用
```bash
# 检查节点是否运行
ros2 node list

# 检查服务列表
ros2 service list

# 检查服务类型
ros2 service type /nav_point_manager_node/add_point
```

### Action 不响应
```bash
# 检查 action 列表
ros2 action list

# 查看 action 信息
ros2 action info /nav_point_manager_node/navigate_to_point
```

### 点位不显示在 RViz
```bash
# 检查 topic
ros2 topic list

# 查看 marker 消息
ros2 topic echo /nav_point_manager_node/nav_points_marker
```
