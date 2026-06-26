# dexforce_description

DexForce W1机器人的完整URDF描述包，包含机器人的几何模型、物理属性和可视化配置。

## 📋 包概述

该包提供了DexForce W1机器人的完整描述，包括：
- 🤖 完整的URDF机器人模型
- 🎨 3D网格文件（碰撞和可视化）
- 🎛️ RViz配置文件
- 🚀 Launch文件

## 📁 文件结构

```
dexforce_description/
├── CMakeLists.txt          # CMake构建配置
├── package.xml             # ROS2包配置
├── README.md              # 本文档
├── launch/                # Launch文件
│   └── display.launch.py  # 机器人显示启动文件
├── meshes/                # 3D网格文件
│   ├── Collision/         # 碰撞检测网格
│   │   ├── chassis/       # 底盘
│   │   ├── head/          # 头部
│   │   ├── left_arm/      # 左臂
│   │   ├── left_hand/     # 左手
│   │   ├── right_arm/     # 右臂
│   │   ├── right_hand/    # 右手
│   │   └── torso/         # 躯干
│   └── Visual/            # 可视化网格
│       ├── chassis/       # 底盘视觉
│       ├── head/          # 头部视觉
│       ├── left_arm/      # 左臂视觉
│       ├── left_hand/     # 左手视觉
│       ├── right_arm/     # 右臂视觉
│       ├── right_hand/    # 右手视觉
│       └── torso/         # 躯干视觉
├── rviz/                  # RViz配置
│   └── display.rviz       # 默认显示配置
└── urdf/                  # URDF模型文件
    └── DexforceW1V020.urdf # 主URDF文件
```

## 🚀 使用方法

### 启动机器人显示

使用launch文件启动RViz显示机器人模型：

```bash
# 构建包
colcon build --packages-select dexforce_description
source install/setup.bash

# 启动显示
ros2 launch dexforce_description display.launch.py
```

### 手动启动RViz

```bash
# 发布机器人状态
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat src/dexforce_description/urdf/DexforceW1V020.urdf)"

# 启动RViz
rviz2 -d src/dexforce_description/rviz/display.rviz
```

### 在代码中使用


## 🤖 机器人规格

### 主要组件
- **底盘 (Chassis)**: 移动平台基座
- **躯干 (Torso)**: 主体结构
- **头部 (Head)**: 传感器安装点
- **双臂 (Arms)**: 7自由度机械臂 × 2
- **双手 (Hands)**: 多指机械手 × 2

### 自由度信息
- **总自由度**: 26 DOF
- **单臂自由度**: 7 DOF
- **单手自由度**: 6 DOF (每指2个关节 × 3指)
- **头部自由度**: 2 DOF

### 文件格式
- **URDF文件**: XML格式的机器人描述
- **网格文件**: OBJ格式（碰撞）+ STL格式（视觉）
- **配置文件**: YAML格式的RViz设置

## 🔧 配置说明

### Launch参数

`display.launch.py` 支持以下参数：

```bash
# 指定URDF文件路径
ros2 launch dexforce_description display.launch.py \
  urdf_file:=path/to/custom.urdf

# 启用GUI控制
ros2 launch dexforce_description display.launch.py \
  use_gui:=true

# 指定RViz配置
ros2 launch dexforce_description display.launch.py \
  rviz_config:=path/to/config.rviz
```

### URDF参数

主要URDF参数可在launch文件中配置：
- `robot_description`: 完整的URDF内容
- `use_sim_time`: 仿真时间设置
- `publish_frequency`: 状态发布频率

## 🛠️ 开发指南

### 修改机器人模型

1. **编辑URDF文件**
```bash
# 编辑主URDF文件
vim src/dexforce_description/urdf/DexforceW1V020.urdf
```

2. **更新网格文件**
```bash
# 替换网格文件（保持文件名不变）
cp new_mesh.obj src/dexforce_description/meshes/Visual/arm/
```

3. **测试更改**
```bash
# 重新构建并测试
colcon build --packages-select dexforce_description
ros2 launch dexforce_description display.launch.py
```

### 添加新组件

1. 在URDF中定义新的link和joint
2. 提供相应的网格文件
3. 更新RViz配置以显示新组件

## 📊 性能建议

### 网格优化
- **碰撞网格**: 使用简化的几何体（< 1000面）
- **视觉网格**: 平衡细节和性能（< 10000面）
- **文件大小**: 单个网格文件 < 5MB

### RViz显示
- 适当设置显示频率（10-30Hz）
- 按需启用/禁用复杂的可视化元素
- 使用合适的材质和颜色

## 🔗 相关包

- `robot_state_publisher`: 发布机器人状态
- `joint_state_publisher`: 发布关节状态
- `rviz2`: 3D可视化工具
- `robot_status_rviz`: 状态监控插件

## 📝 更新日志

### v0.2.0 (2025-09-01)
- ✅ 完善URDF模型描述
- ✅ 添加完整的网格文件
- ✅ 优化RViz配置
- ✅ 改进launch文件

### v0.1.0 (2025-08-01)
- 🎉 初始版本发布
- ✅ 基础URDF模型
- ✅ 基本网格文件

## 🤝 贡献

欢迎提交Issue和Pull Request来改进机器人模型！
