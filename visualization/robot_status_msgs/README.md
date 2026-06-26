# robot_status_msgs

ROS2消息定义包，为机器人状态监控系统提供标准化的消息类型。

## 📋 包概述

该包定义了机器人状态监控系统所需的所有ROS2消息类型，包括：
- 🤖 机器人系统状态消息
- 🔊 语音服务状态消息
- 📡 标准化的状态通信接口

## 📁 文件结构

```
robot_status_msgs/
├── CMakeLists.txt           # CMake构建配置
├── package.xml              # ROS2包配置
├── README.md               # 本文档
└── msg/                    # 消息定义文件
    ├── RobotStatus.msg     # 机器人状态消息
    └── AudioServerStatus.msg # 语音服务状态消息
```

## 📋 消息定义

### RobotStatus.msg

机器人系统整体状态消息：

```msg
# Robot system status message
bool is_ok              # Overall system health status
string[] error_nodes    # List of nodes with errors/issues
```

**字段说明:**
- `is_ok`: 布尔值，表示机器人系统是否正常
  - `true`: 所有关键节点运行正常
  - `false`: 存在异常或故障节点
- `error_nodes`: 字符串数组，包含所有异常节点的名称列表

**使用示例:**
```python
# 发布正常状态
msg = RobotStatus()
msg.is_ok = True
msg.error_nodes = []

# 发布异常状态
msg = RobotStatus()
msg.is_ok = False
msg.error_nodes = ['arm_controller', 'gripper_node']
```

### AudioServerStatus.msg

语音交互服务状态消息：

```msg
# Audio server status message
bool is_online     # Whether the audio service is online
string status      # Status description (e.g., "running", "stopped", "error")
```

**字段说明:**
- `is_online`: 布尔值，表示语音服务是否在线
  - `true`: 语音服务正常运行
  - `false`: 语音服务离线或异常
- `status`: 字符串，详细状态描述（通常UI不显示此字段）

**使用示例:**
```python
# 发布在线状态
msg = AudioServerStatus()
msg.is_online = True
msg.status = "running"

# 发布离线状态
msg = AudioServerStatus()
msg.is_online = False
msg.status = "process not found"
```

## 🔧 构建和安装

### 构建包

```bash
# 在工作空间根目录
colcon build --packages-select robot_status_msgs

# 加载环境
source install/setup.bash
```

### 验证安装

```bash
# 检查消息类型
ros2 interface show robot_status_msgs/msg/RobotStatus
ros2 interface show robot_status_msgs/msg/AudioServerStatus

# 查看话题
ros2 topic list | grep status
```

## 📡 话题约定

### 标准话题名称

| 话题名 | 消息类型 | 功能描述 |
|--------|----------|----------|
| `/robot_status` | `RobotStatus` | 机器人系统状态 |
| `/audio_server_status` | `AudioServerStatus` | 语音服务状态 |

### 发布频率建议

- **机器人状态**: 1-5 Hz（根据系统复杂度）
- **语音服务状态**: 0.5-1 Hz（较低频率即可）

### QoS设置建议

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# 状态消息QoS配置
status_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)
```

## 🔗 相关包

此消息包被以下包使用：

- `robot_status_monitor`: 系统监控和消息发布
- `robot_status_rviz`: RViz状态显示插件
- `robot_status_foxglove`: Foxglove状态显示插件

### 版本兼容性

- 向后兼容：只在消息末尾添加新字段
- 避免删除或重命名现有字段
- 使用合适的默认值

## 📊 性能考虑

### 消息大小优化
- 使用合适的数据类型（如uint8代替int32）
- 避免过大的数组字段
- 考虑消息分割（如大量错误节点时）

### 网络带宽
- 控制发布频率，避免不必要的高频发布
- 使用QoS设置优化网络传输
- 考虑消息压缩（对于大型消息）

## 📝 更新日志

### v0.1.0 (2025-09-01)
- 🎉 初始版本
- ✅ 基础RobotStatus消息

## 🤝 贡献

欢迎提交Issue和Pull Request来改进消息定义！
