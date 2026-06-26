# Robot Status Monitor

机器人状态监控节点，用于监控ROS2节点运行状态和语音服务进程状态，并发布系统健康信息。

## 功能特性

- **ROS2节点监控**：定期检查指定的ROS2节点是否在运行
- **语音服务监控**：检查语音服务进程是否正在运行
- **双话题发布**：
  - `/robot_status` - 机器人系统状态
  - `/audio_server_status` - 语音服务状态
- 支持通过参数配置监控的节点列表和进程名
- 提供详细的日志信息

## 使用方法

### 1. 构建包

```bash
cd /path/to/your/workspace
colcon build --packages-select robot_status_msgs robot_status_monitor
source install/setup.bash
```

### 2. 启动监控节点

```bash
# 启动综合监控节点（推荐）
ros2 launch robot_status_monitor system_monitor.launch.py

# 或者单独启动各个监控节点
ros2 run robot_status_monitor node_monitor          # 仅ROS2节点监控
ros2 run robot_status_monitor audio_server_monitor  # 仅语音服务监控
```

### 3. 自定义监控配置

编辑 `launch/system_monitor.launch.py` 文件，修改参数：

```python
parameters=[{
    'check_nodes': ['/your_node1', '/your_node2'],           # ROS2节点
    'audio_process_names': ['your_audio_service', 'speech'], # 语音服务进程名
    'check_interval': 2.0                                    # 检查间隔
}]
```

## 参数说明

- `check_nodes`: 需要监控的ROS2节点名称列表
- `audio_process_names`: 需要监控的语音服务进程名称列表
- `check_interval`: 检查间隔时间（秒）

## 发布的话题

- `/robot_status` (robot_status_msgs/RobotStatus): 机器人系统状态信息
- `/audio_server_status` (robot_status_msgs/AudioServerStatus): 语音服务状态信息

## 消息格式

### RobotStatus
```
bool is_ok          # 系统是否正常
string[] error_nodes # 异常/离线的节点列表
```

### AudioServerStatus
```
bool is_online      # 语音服务是否在线
string status       # 状态描述
```

## 依赖

- rclpy
- robot_status_msgs
- python3-psutil

## 许可证

Apache 2.0
