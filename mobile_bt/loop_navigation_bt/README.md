# Loop Navigation Behavior Tree (循环导航行为树)

## 概述

`loop_navigation_bt` 是一个自主移动机器人的循环导航行为树模块，实现了以下功能：

1. **电量检测**：通过订阅 `/feedback/battery` 话题实时获取机器人电量
2. **循环导航**：在电量充足（>35%）时循环导航四个预设点
3. **自动回充**：当电量低于35%时自动触发回充服务
4. **持续监控**：回充完成后继续检测电量，电量充足则继续导航

## 项目结构

```
loop_navigation_bt/
├── CMakeLists.txt                    # CMake 构建配置
├── package.xml                       # ROS2 包配置
├── README.md                         # 本文件
├── include/
│   └── loop_navigation_bt/
│       └── loop_navigation_bt.hpp    # 头文件，定义所有行为树节点
├── src/
│   ├── loop_navigation_bt.cpp        # 行为树节点实现
│   └── main.cpp                      # 主程序入口
├── config/
│   ├── loop_navigation_bt.xml        # 行为树结构定义
│   └── navigation_points.json        # 导航点配置
└── launch/
    └── loop_navigation_bt.launch.py  # ROS2 启动文件
```

## 行为树结构

```
MainTree (WhileDoElse)
├── Condition: CheckBatteryAboveThreshold (threshold=35%)
├── Do: LoopNavigateFourPoints (循环导航)
└── Else: CallAutoChargeService (调用回充服务)
```

### 执行流程

1. **检查电量** (`CheckBatteryAboveThreshold`)
   - 订阅 `/feedback/battery` 话题
   - 检查电量是否 > 35%
   - SUCCESS: 电量充足，执行导航
   - FAILURE: 电量不足，执行回充

2. **循环导航** (`LoopNavigateFourPoints`)
   - 循环导航四个预设点
   - 每完成一个点后移动到下一个点
   - 在导航过程中持续检查电量
   - 若电量 ≤ 35%，停止导航并返回 FAILURE

3. **自动回充** (`CallAutoChargeService`)
   - 调用 `/auto_charge` 服务
   - 执行 `ros2 service call /auto_charge Trigger "{}"`
   - 回充完成后继续循环

## 关键节点说明

### 1. CheckBatteryAboveThreshold
- **类型**: SyncActionNode
- **功能**: 检查当前电量是否大于阈值
- **输入端口**: `threshold` (double, 默认35.0)
- **返回值**: SUCCESS (电量充足) / FAILURE (电量不足)

### 2. NavigateToPoint
- **类型**: SyncActionNode
- **功能**: 导航到指定的单个点
- **输入端口**: `goal` (geometry_msgs::msg::PoseStamped)
- **返回值**: SUCCESS / FAILURE

### 3. LoopNavigateFourPoints
- **类型**: StatefulActionNode
- **功能**: 循环导航四个预设点
- **输入端口**: `points` (std::vector<geometry_msgs::msg::PoseStamped>)
- **返回值**: RUNNING (继续循环) / FAILURE (电量不足或导航失败)
- **特性**:
  - 自动循环遍历所有点
  - 在导航过程中持续检查电量
  - 电量低于35%时停止并返回FAILURE

### 4. CallAutoChargeService
- **类型**: SyncActionNode
- **功能**: 调用回充服务
- **服务**: `/auto_charge` (std_srvs::srv::Trigger)
- **返回值**: SUCCESS (回充成功) / FAILURE (回充失败)

## 配置文件

### navigation_points.json
定义循环导航的四个点坐标：

```json
{
  "frame_id": "map",
  "points": [
    {"position": {"x": 0.0, "y": 0.0, "z": 0.0}, ...},
    {"position": {"x": 5.0, "y": 0.0, "z": 0.0}, ...},
    {"position": {"x": 5.0, "y": 5.0, "z": 0.0}, ...},
    {"position": {"x": 0.0, "y": 5.0, "z": 0.0}, ...}
  ]
}
```

**修改方式**:
1. 编辑 `config/navigation_points.json`
2. 修改每个点的 `position` 和 `orientation`
3. 重新编译或直接修改安装目录中的文件

### loop_navigation_bt.xml
定义行为树的结构和逻辑，使用 BehaviorTree.CPP v3 的 XML 格式。

## 编译和运行

### 编译
```bash
cd ~/w1
colcon build --packages-select loop_navigation_bt
source install/setup.bash
```

### 运行
```bash
# 方式1: 使用启动文件
ros2 launch loop_navigation_bt loop_navigation_bt.launch.py

# 方式2: 直接运行节点
ros2 run loop_navigation_bt loop_navigation_bt_node
```

## 依赖项

- `rclcpp`: ROS2 C++ 客户端库
- `rclcpp_action`: ROS2 Action 客户端
- `sensor_msgs`: 传感器消息定义
- `geometry_msgs`: 几何消息定义
- `nav2_msgs`: Nav2 消息定义
- `behaviortree_cpp_v3`: 行为树库
- `nlohmann_json`: JSON 解析库
- `std_srvs`: 标准服务定义

## 话题和服务

### 订阅话题
- `/feedback/battery` (sensor_msgs::msg::BatteryState)
  - 获取机器人电量信息
  - 字段: `percentage` (0-100)

### 调用服务
- `/auto_charge` (std_srvs::srv::Trigger)
  - 触发自动回充
  - 请求: 空
  - 响应: `success` (bool), `message` (string)

### 调用 Action
- `navigate_to_pose` (nav2_msgs::action::NavigateToPose)
  - 导航到指定位置

## 日志输出

程序会输出详细的日志信息，包括：
- 🔋 电量检查信息
- 🎯 导航目标信息
- ✅ 成功操作
- ❌ 失败操作
- ⚠️  警告信息
- 🔄 循环状态

## 故障排除

### 1. 无法连接到 nav2 服务器
- 确保 nav2 已启动
- 检查 `navigate_to_pose` action 是否可用

### 2. 无法读取导航点配置
- 检查 `navigation_points.json` 文件路径
- 确保 JSON 格式正确

### 3. 无法调用回充服务
- 确保 `/auto_charge` 服务已注册
- 检查服务节点是否运行

### 4. 电量信息为 0
- 检查 `/feedback/battery` 话题是否发布
- 确保电池驱动程序正常工作

## 扩展功能

### 修改电量阈值
在 `loop_navigation_bt.xml` 中修改：
```xml
<CheckBatteryAboveThreshold threshold="35.0" />
```

### 修改导航点
编辑 `config/navigation_points.json` 文件，添加或修改点的坐标。

### 添加新的行为树节点
1. 在 `loop_navigation_bt.hpp` 中定义新类
2. 在 `loop_navigation_bt.cpp` 中实现
3. 在 `main.cpp` 中注册到工厂
4. 在 XML 中使用新节点

## 许可证

Apache License 2.0

## 作者

DexForce Team


