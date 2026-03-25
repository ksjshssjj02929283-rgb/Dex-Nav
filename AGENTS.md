# AGENTS.md - Agent 代码开发指南

本文档为 AI 代理提供代码开发规范和构建命令参考。

## 项目概述

本项目是基于 ROS2 Navigation2 (nav2) 的移动机器人导航系统，包含以下主要模块：
- `mobile_perception`: 机器人感知节点（AprilTag检测、充电桩控制、移动控制）
- `mobile_navigation`: Navigation2 导航堆栈
- `dexe_sensor_process`: 传感器处理
- `utils`: 工具脚本

## 1. 构建命令

> **重要：AI 代理禁止执行 `colcon build` 命令。** 本工作区包含多个同名包副本（`nav2_mppi_controller-main/`、`navigation2-humble_main/`），`colcon build` 会因重复包名报错且无法通过 `--allow-overriding` 解决。所有构建操作由用户手动完成。

### 完整构建
```bash
# 确保先 source ROS 环境
source /opt/ros/humble/setup.bash

# 使用项目提供的构建脚本
./build.sh

# 或手动构建
colcon build --packages-up-to nav2_bringup --parallel-workers 8
```

### 单独构建某个包
```bash
colcon build --packages-select mobile_perception --parallel-workers 8
```

### 运行单个测试
```bash
# 构建并运行特定包的测试
colcon test --packages-select <package_name> --event-handlers console_direct+

# 查看测试结果
colcon test-result --verbose

# 示例：运行 nav2_costmap_2d 单元测试
colcon test --packages-select nav2_costmap_2d --event-handlers console_direct+
colcon test-result --test-result-base install/nav2_costmap_2d/test_results/nav2_costmap_2d --verbose

# 直接运行可执行测试文件（如果已构建）
./install/nav2_costmap_2d/lib/nav2_costmap_2d/test_costmap_2d
```

### 测试类型
- **单元测试**: `test/unit/` 目录下，使用 GTest
- **集成测试**: `test/integration/` 目录下
- **回归测试**: `test/regression/` 目录下

## 2. 代码风格规范

### C++ 风格 (.clang-format)

```yaml
BasedOnStyle: Google
IndentWidth: 4
ColumnLimit: 120
UseTab: Never
Standard: c++17
ContinuationIndentWidth: 4
AccessModifierOffset: -4
BinPackParameters: false
SortIncludes: true
```

**关键规则：**
- 缩进：4 空格
- 行宽限制：120 字符
- C++ 标准：C++17
- 不使用 Tab
- 使用 Google 代码风格
- include 按字母顺序排序

### Python 风格 (.style.yapf)

```yaml
[style]
based_on_style = google
```

**关键规则：**
- 使用 Google Python 风格
- yapf 版本：0.43.0

### 代码头注释

所有源文件需包含版权头：

**C++/C:**
```cpp
// ----------------------------------------------------------------------------
// Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
//
// All rights reserved.
// ----------------------------------------------------------------------------
```

**Python:**
```python
# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------
```

## 3. 命名规范

### 文件命名
- C++ 头文件: `*.hpp`, `*.h`
- C++ 源文件: `*.cpp`
- Python 文件: `*.py`
- Launch 文件: `*.launch.py`

### 变量/函数命名
- 变量: `snake_case` (如 `control_frequency_`, `target_pose`)
- 函数: `snake_case` (如 `calculateVelocity()`, `targetCallback()`)
- 类名: `CamelCase` (如 `MobileControlNode`, `GoalController`)
- 常量: `kCamelCase` (如 `kRho`, `kAlpha`)

### ROS 命名空间
- 主题、服务: `snake_case` (如 `control/cmd_vel`, `feedback/odom`)
- 参数: `snake_case` (如 `control_frequency`, `max_velocity`)

## 4. Import 排序

### C++ (使用 clang-format 自动排序)
```cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <algorithm>
```

### Python
```python
# 标准库
import time
import sys
import signal
import math
import json
import os
import argparse

# 第三方库
import numpy as np
import transforms3d as t3d
from scipy.spatial.transform import Rotation as R

# ROS/项目库
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from tf2_ros import Buffer, TransformListener
```

## 5. 错误处理

### C++ ROS 节点
```cpp
try {
    // 可能抛出异常的代码
    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
        "base_link", "map", tf2::TimePointZero);
    tf2::doTransform(target_pose_odom_, target_pose_in_base, transform);
} catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Cannot transform goal to map: %s", ex.what());
    return;
}
```

### 日志级别
- `RCLCPP_DEBUG`: 调试信息
- `RCLCPP_INFO`: 一般信息
- `RCLCPP_WARN`: 警告
- `RCLCPP_ERROR`: 错误
- `RCLCPP_FATAL`: 致命错误

## 6. 代码组织

### ROS2 节点结构
```cpp
class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("node_name")
    {
        // 参数声明
        declare_parameters();
        
        // 初始化
        init();
        
        // 创建发布者/订阅者/服务
        create_publishers();
        create_subscriptions();
        create_services();
        
        // 创建定时器
        timer_ = create_wall_timer(...);
    }

private:
    // 成员变量
    rclcpp::Publisher<...>::SharedPtr publisher_;
    rclcpp::Subscription<...>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 回调函数
    void callback(const ...::SharedPtr msg) {...}
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### 目录结构
```
package/
├── src/              # C++ 源文件
├── include/          # 头文件
├── scripts/          # Python 脚本
├── launch/           # ROS2 launch 文件
├── config/           # 配置文件
├── test/             # 测试文件
│   ├── unit/         # 单元测试
│   ├── integration/  # 集成测试
│   └── regression/   # 回归测试
└── CMakeLists.txt
```

## 7. 格式检查

### 运行格式检查
```bash
# 检查代码风格
python3 utils/check_style.py

# 自动修复代码风格
python3 utils/check_style.py --apply
```

### 手动格式化
```bash
# C++ 格式化
clang-format -style=file -i src/*.cpp

# Python 格式化
yapf --style=.style.yapf -i scripts/*.py
```

## 8. 常用开发命令

```bash
# Source 工作空间
source install/setup.bash

# 运行节点
ros2 run mobile_perception mobile_control_node

# 查看话题
ros2 topic list
ros2 topic echo /control/cmd_vel

# 查看节点
ros2 node list

# 查看参数
ros2 param list

# 运行 launch 文件
ros2 launch mobile_perception mobile_control.launch.py
```

## 9. 注意事项

1. **线程安全**: ROS2 节点默认单线程，使用 `MultiThreadedExecutor` 实现多线程
2. **TF2**: 始终处理 `tf2::TransformException` 异常
3. **智能指针**: 优先使用 `std::make_shared<T>()` 和 `SharedPtr`
4. **参数**: 使用 `declare_parameter()` 声明参数，提供默认值
5. **生命周期**: 使用 LifecycleNode 实现更精细的节点生命周期管理
6. **测试**: 新功能应编写对应的单元测试和集成测试
7. ****

## 10. 最近会话补充

- 2026-03-25：已核对 `mobile_navigation/nav2_mppi_controller` 中 `VelocityDeadbandCritic` 与上游同步状态，确认当前实现逻辑正确。
- 2026-03-25：已在 `mobile_navigation/nav2_mppi_controller/test/critics_tests.cpp` 中补充 `VelocityDeadbandCritic` 单测。
- 2026-03-25：已在 `mobile_navigation/nav2_bringup/params/w1_nav2_params.yaml` 与 `mobile_navigation/nav2_bringup/params/w1_nav2_rgbd_params.yaml` 中接入 `VelocityDeadbandCritic` 控制参数。
- 续接本轮工作时，优先查看工作区根目录 `Last_session.md`。
