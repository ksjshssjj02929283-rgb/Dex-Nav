# dexhand_joint_converter

DexHand多指机械手关节数据转换服务，提供灵活手部控制的数据转换功能。

## 📋 包概述

该包提供了DexHand多指机械手的关节数据转换服务，将百分比控制信号转换为标准的ROS2关节状态消息，包括：
- 🤏 百分比到弧度的关节转换
- 📡 可配置的ROS2话题接口  
- 🎛️ 支持左右手独立控制
- 🔧 实时数据处理

## 📁 文件结构

```
dexhand_joint_converter/
├── package.xml                 # ROS2包配置
├── setup.cfg                   # Python包配置
├── setup.py                    # Python安装脚本
├── README.md                   # 本文档
├── config/                     # 配置文件目录
│   └── dexhand_params.yaml    # 参数配置文件
├── dexhand_joint_converter/    # Python包目录
│   ├── __init__.py            # 包初始化
│   └── dexhand_joint_converter.py # 主转换模块
├── resource/                   # 资源文件
│   └── dexhand_joint_converter # 包资源标识
└── test/                       # 测试文件
    ├── test_copyright.py       # 版权测试
    ├── test_flake8.py         # 代码风格测试
    └── test_pep257.py         # 文档字符串测试
```

## 🚀 功能特性

### 核心功能
- **百分比转换**: 将0-100%的控制信号转换为弧度角度
- **双手支持**: 独立处理左手和右手的关节数据
- **关节映射**: 支持DexHand特定的关节名称映射
- **实时处理**: 低延迟的数据处理能力
- **参数配置**: 可配置的话题名称

### 支持的关节
- 👍 **拇指关节**: THUMBMCP1(摆动) + THUMBMCP2(弯曲)
- 👆 **食指关节**: INDEXMCP(弯曲)
- 🖕 **中指关节**: MIDDLEMCP(弯曲)  
- 💍 **无名指关节**: RINGMCP(弯曲)
- 🤙 **小指关节**: LITTLEMCP(弯曲)

## 🚀 使用方法

### 启动转换服务

```bash
# 构建包
colcon build --packages-select dexhand_joint_converter
source install/setup.bash

# 使用默认参数启动
ros2 run dexhand_joint_converter dexhand_joint_converter

# 使用自定义话题名启动
ros2 run dexhand_joint_converter dexhand_joint_converter \
  --ros-args -p left_input_topic:=/custom/hand/left \
             -p right_input_topic:=/custom/hand/right \
             -p output_topic:=/custom_joint_states

# 使用参数文件启动
ros2 run dexhand_joint_converter dexhand_joint_converter \
  --ros-args --params-file src/dexhand_joint_converter/config/dexhand_params.yaml
```

### 话题接口

**输入话题:**
- `/feedback_sim/hand/left` (std_msgs/Float64MultiArray): 左手百分比控制信号 [0-100%]
- `/feedback_sim/hand/right` (std_msgs/Float64MultiArray): 右手百分比控制信号 [0-100%]

**输出话题:**
- `/joint_states` (sensor_msgs/JointState): 转换后的关节状态消息

### 数据格式说明

**输入数据格式 (Float64MultiArray):**
```python
# 左手/右手输入数据：6个元素的数组，范围0-100%
data: [thumb_swing, thumb_bend, index_bend, middle_bend, ring_bend, little_bend]

# 示例：所有关节弯曲50%
data: [50.0, 50.0, 50.0, 50.0, 50.0, 50.0]
```

**输出数据格式 (JointState):**
```python
# 关节名称 (左手示例)
name: ["LEFT_L_THUMBMCP1", "LEFT_L_THUMBMCP2", "LEFT_L_INDEXMCP", 
       "LEFT_L_MIDDLEMCP", "LEFT_L_RINGMCP", "LEFT_L_LITTLEMCP"]

# 关节位置 (弧度)
position: [0.785, 0.436, 0.654, 0.654, 0.654, 0.654]  # 对应50%弯曲
```

### 在代码中使用

## 🔧 配置参数

### ROS2参数

| 参数名 | 类型 | 默认值 | 描述 |
|--------|------|--------|------|
| `left_input_topic` | string | `/feedback_sim/hand/left` | 左手输入话题名称 |
| `right_input_topic` | string | `/feedback_sim/hand/right` | 右手输入话题名称 |
| `output_topic` | string | `/joint_states` | 关节状态输出话题名称 |
| `left_joint_ranges.thumbmcp1` | double[] | `[0.0, 90.0]` | 左手拇指摆动角度范围（度） |
| `left_joint_ranges.thumbmcp2` | double[] | `[0.0, 50.0]` | 左手拇指弯曲角度范围（度） |
| `left_joint_ranges.indexmcp` | double[] | `[0.0, 75.0]` | 左手食指弯曲角度范围（度） |
| `left_joint_ranges.middlemcp` | double[] | `[0.0, 75.0]` | 左手中指弯曲角度范围（度） |
| `left_joint_ranges.ringmcp` | double[] | `[0.0, 75.0]` | 左手无名指弯曲角度范围（度） |
| `left_joint_ranges.littlemcp` | double[] | `[0.0, 75.0]` | 左手小指弯曲角度范围（度） |
| `right_joint_ranges.*` | double[] | `同左手` | 右手各关节角度范围（度） |

### 设置参数

```bash
# 通过命令行设置参数
ros2 run dexhand_joint_converter dexhand_joint_converter \
  --ros-args -p left_input_topic:=/custom/left_hand \
             -p right_input_topic:=/custom/right_hand \
             -p output_topic:=/custom_joints \
             -p left_joint_ranges.thumbmcp1:="[0.0, 85.0]" \
             -p left_joint_ranges.indexmcp:="[0.0, 70.0]"

# 使用参数文件（推荐）
ros2 run dexhand_joint_converter dexhand_joint_converter \
  --ros-args --params-file config/dexhand_params.yaml

# 使用自定义范围配置
ros2 run dexhand_joint_converter dexhand_joint_converter \
  --ros-args --params-file config/dexhand_custom_ranges.yaml
```

### 参数文件示例

## 🎛️ 关节映射

### DexHand关节配置

### 转换公式

### 配置范围的优势

1. **硬件适配**: 根据实际机械手的物理限制调整
2. **安全防护**: 避免超出安全操作范围
3. **精度控制**: 在较小范围内获得更高的控制精度
4. **左右手差异**: 支持左右手不同的角度限制

## 🧪 测试

### 运行测试

```bash
# 运行所有测试
colcon test --packages-select dexhand_joint_converter

# 运行特定测试
python -m pytest src/dexhand_joint_converter/test/

# 代码风格检查
flake8 src/dexhand_joint_converter/dexhand_joint_converter/
```

### 功能测试

```bash
# 测试左手控制 - 发送握拳命令
ros2 topic pub /feedback_sim/hand/left std_msgs/Float64MultiArray "
data: [0.0, 80.0, 80.0, 80.0, 80.0, 80.0]
" --once

# 测试右手控制 - 发送张开手命令  
ros2 topic pub /feedback_sim/hand/right std_msgs/Float64MultiArray "
data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
" --once

# 监听转换结果
ros2 topic echo /joint_states

# 检查特定关节的位置
ros2 topic echo /joint_states --field position
```

### 配置验证测试

```bash
# 验证配置是否正确加载
ros2 run dexhand_joint_converter dexhand_joint_converter \
  --ros-args --params-file config/dexhand_custom_ranges.yaml

# 检查日志输出，确认范围设置
# 应该看到类似输出：
# [INFO] Left joint ranges (deg): [[0.0, 85.0], [0.0, 45.0], ...]

# 测试边界值 - 发送0%命令（应该得到最小角度）
ros2 topic pub /feedback_sim/hand/left std_msgs/Float64MultiArray "
data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
" --once

# 测试边界值 - 发送100%命令（应该得到最大角度）
ros2 topic pub /feedback_sim/hand/left std_msgs/Float64MultiArray "
data: [100.0, 100.0, 100.0, 100.0, 100.0, 100.0]
" --once

# 监听输出验证角度转换
ros2 topic echo /joint_states --field position

# 验证具体关节的角度值
ros2 topic echo /joint_states | grep position -A 1
```

### 配置文件切换

```bash
# 使用默认配置
ros2 run dexhand_joint_converter dexhand_joint_converter \
  --ros-args --params-file config/dexhand_params.yaml

# 切换到自定义范围配置
ros2 run dexhand_joint_converter dexhand_joint_converter \
  --ros-args --params-file config/dexhand_custom_ranges.yaml

# 运行时查看当前配置的参数
ros2 param list /dexhand_joint_converter
ros2 param get /dexhand_joint_converter left_joint_ranges.thumbmcp1
```

## 📊 性能监控

### 性能指标

```bash
# 监控话题频率
ros2 topic hz /hand_joints_converted

# 监控延迟
ros2 topic delay /hand_joints_converted

# 监控节点状态
ros2 node info /dexhand_joint_converter
```

### 性能优化建议

### 故障排除

**常见问题及解决方案:**

1. **没有输出数据**
   - 检查输入话题是否有数据：`ros2 topic echo /feedback_sim/hand/left`
   - 确认节点正在运行：`ros2 node list | grep dexhand`

2. **关节角度异常**
   - 验证输入数据范围（0-100%）
   - 检查关节角度限制设置

3. **话题名称错误**
   - 使用参数重新配置话题名称
   - 检查参数文件格式是否正确


## 🔗 相关包

- `general_joint_converter`: 通用关节转换服务
- `sensor_msgs`: ROS2传感器消息类型
- `robot_status_monitor`: 系统状态监控

## 🛠️ 开发指南


### 贡献指南

1. Fork项目并创建特性分支
2. 添加测试用例
3. 确保代码符合PEP8风格
4. 提交Pull Request

## 📝 更新日志

### v0.1.0 (2025-09-01)
- 🎉 初始版本发布
- ✅ 基础关节转换功能
- ✅ ROS2话题接口

## 🤝 贡献

欢迎提交Issue和Pull Request来改进关节转换功能！

## 📄 许可证

MIT License - 详见 [LICENSE](../LICENSE) 文件
