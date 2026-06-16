# Mission Manager (ROS2)

> ROS2任务管理器 - 机器人任务调度和执行管理

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)](/)
[![Test Status](https://img.shields.io/badge/tests-passing-brightgreen)](/)

---

## 📋 目录

- [简介](#简介)
- [功能特性](#功能特性)
- [快速开始](#快速开始)
- [使用说明](#使用说明)
- [测试](#测试)
- [文档](#文档)
- [依赖](#依赖)
- [迁移说明](#迁移说明)

---

## 简介

`mission_manager`是一个ROS2任务管理节点，负责机器人任务的调度、执行和状态管理。支持多种任务类型，包括导航任务、充电任务、对接任务等。

### 主要能力
- ✅ 任务队列管理
- ✅ 任务优先级处理
- ✅ 自动充电管理
- ✅ 任务组批量执行
- ✅ 任务状态跟踪
- ✅ 异常处理和恢复

---

## 功能特性

### 支持的任务类型
1. **导航任务** - 移动到指定位置
2. **充电任务** - 自动前往充电桩充电
3. **对接任务** - 对接到指定设备
4. **等待任务** - 在当前位置等待
5. **自定义任务** - 可扩展的任务类型

### 核心功能
- **任务队列**: FIFO任务队列，支持任务优先级
- **任务组**: 批量任务管理，支持循环执行
- **自动充电**: 基于电量阈值的智能充电
- **状态管理**: 完整的任务状态机
- **暂停/恢复**: 支持任务的暂停和恢复
- **取消机制**: 支持单个任务或任务组的取消

---

## 快速开始

### 1. 编译
```bash
cd /home/dex/workspace_pzc
colcon build --packages-select mission_manager
source install/setup.bash
```

### 2. 启动节点
```bash
ros2 launch mission_manager mission_manager.launch.py
```

### 3. 查看状态
```bash
# 查看节点信息
ros2 node info /mission_manager

# 获取管理器状态
ros2 service call /mission_manager/get_mission_status nav_interfaces/srv/GetStatus

# 监听状态话题
ros2 topic echo /mission_manager/manager_status
```

---

## 使用说明

### 参数配置

参数文件位置: `params/mission_manager_params.yaml`

```yaml
mission_manager:
  mission_manager:
    auto_charge_threshold: 30.0      # 自动充电阈值 (%)
    force_charge_threshold: 20.0     # 强制充电阈值 (%)
    respawn_task: false              # 任务失败是否重启
```

### 服务接口

#### 任务管理
```bash
# 添加单个任务
ros2 service call /mission_manager/add_task nav_interfaces/srv/AddTask

# 添加充电任务
ros2 service call /mission_manager/add_charge_task nav_interfaces/srv/AddTask

# 添加任务组
ros2 service call /mission_manager/add_task_group nav_interfaces/srv/AddTaskGroup

# 清空所有任务
ros2 service call /mission_manager/clear_all_tasks std_srvs/srv/Trigger
```

#### 任务控制
```bash
# 暂停当前任务
ros2 service call /mission_manager/pause_task std_srvs/srv/Trigger

# 恢复任务
ros2 service call /mission_manager/resume_task std_srvs/srv/Trigger

# 取消当前任务
ros2 service call /mission_manager/cancel_task std_srvs/srv/Trigger

# 取消任务组
ros2 service call /mission_manager/cancel_task_group std_srvs/srv/Trigger
```

#### 状态查询
```bash
# 获取管理器状态
ros2 service call /mission_manager/get_mission_status nav_interfaces/srv/GetStatus

# 获取当前任务信息
ros2 service call /mission_manager/get_current_task_info nav_interfaces/srv/GetCurrentTaskInfo
```

### 话题

#### 发布的话题
- `/mission_manager/manager_status` - 管理器状态 (10Hz)
- `/mission_manager/current_task_info` - 当前任务信息 (1Hz)

#### 订阅的话题
- `/robot_status` - 机器人状态
- `/charge_status` - 充电状态
- `/docking_pile_status` - 对接桩状态

---

## 测试

### 参数测试
```bash
cd params && python3 ../test/test_params.py
```

### 节点功能测试
```bash
# 终端1: 启动节点
ros2 launch mission_manager mission_manager.launch.py

# 终端2: 运行测试
python3 test/test_mission_manager.py
```

### 运行所有测试
```bash
cd test && bash run_all_tests.sh
```

**测试状态**: ✅ 全部通过 (8/8)

详见: [TEST_RESULTS.md](TEST_RESULTS.md)

---

## 文档

### 完整文档
- [TEST_GUIDE.md](TEST_GUIDE.md) - 测试指南
- [QUICK_START.md](QUICK_START.md) - 快速开始
- [ROS2_MIGRATION.md](ROS2_MIGRATION.md) - ROS2迁移文档
- [CONVERSION_SUMMARY.md](CONVERSION_SUMMARY.md) - 转换总结
- [MIGRATION_COMPLETE.md](MIGRATION_COMPLETE.md) - 迁移完成报告
- [TEST_RESULTS.md](TEST_RESULTS.md) - 测试结果
- [README_pzc.md](README_pzc.md) - 原始文档
- [test/README.md](test/README.md) - 测试详细说明

### API文档
详见源代码注释和头文件。

---

## 依赖

### ROS2 包
- `rclcpp` - ROS2 C++ 客户端库
- `std_msgs` - 标准消息
- `std_srvs` - 标准服务
- `geometry_msgs` - 几何消息
- `nav2_util` - 导航工具
- `tf2_ros` - TF2 ROS接口
- `tf2_geometry_msgs` - TF2 几何消息转换

### 自定义包
- `nav_interfaces` - 自定义消息定义
- `nav_interfaces` - 自定义服务定义

### 第三方库
- `glog` - Google 日志库

---

## 迁移说明

本包从ROS1迁移而来，主要变更：

### ROS1 → ROS2 关键变更
1. **节点架构**: `ros::NodeHandle` → `rclcpp::Node`
2. **参数系统**: `ros::param` → `declare_parameter/get_parameter`
3. **时间**: `ros::Time` → `rclcpp::Time`
4. **日志**: `ROS_INFO` → `RCLCPP_INFO`
5. **TF**: `tf` → `tf2_ros`
6. **消息类型**: 添加 `::msg::` 和 `::srv::` 命名空间

### 依赖替换
- `nav_util` → `nav2_util` + 自定义适配器
- `dcm_msgs` → `nav_interfaces`
- `dcm_srvs` → `nav_interfaces`

详见: [ROS2_MIGRATION.md](ROS2_MIGRATION.md)

---

## 项目结构

```
mission_manager/
├── apps/                           # ROS2接口层
│   ├── mission_manager_node.cpp
│   ├── mission_manager_ros.hpp
│   └── mission_manager_ros.cpp
├── include/mission_manager/        # 头文件
│   ├── mission_manager.hpp
│   ├── mission_info.hpp
│   ├── nav_util_adapter.hpp        # nav_util适配器
│   └── tasks/                      # 任务类型
│       ├── task_charging.hpp
│       ├── task_docking_pile.hpp
│       ├── task_exit_pile.hpp
│       └── task_navigation.hpp
├── src/                            # 实现文件
│   ├── mission_manager.cpp
│   └── tasks/
│       ├── task_charging.cpp
│       ├── task_docking_pile.cpp
│       ├── task_exit_pile.cpp
│       └── task_navigation.cpp
├── params/                         # 参数文件
│   └── mission_manager_params.yaml
├── launch/                         # Launch文件
│   └── mission_manager.launch.py
├── test/                           # 测试文件
│   ├── test_params.py
│   ├── test_mission_manager.py
│   ├── run_all_tests.sh
│   └── README.md
├── CMakeLists.txt                  # CMake构建配置
├── package.xml                     # 包清单
└── README.md                       # 本文档
```

---

## 贡献

欢迎提交问题和拉取请求！

---

## 许可

[根据项目实际情况填写]

---

## 联系方式

[根据项目实际情况填写]

---

**版本**: 1.0.0 (ROS2)  
**最后更新**: 2025-10-22  
**状态**: ✅ 生产就绪

