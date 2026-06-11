# Mission Manager 集成测试说明

## 📖 概述

本文档说明如何测试 `mission_manager` 与其他相关包（`map_manager_ros2` 和 `nav2_point_manager`）的集成。

## 🔗 包之间的依赖关系

```
mission_manager (任务管理)
    ↓ 依赖
nav2_point_manager (导航点管理)
    ↓ 依赖  
map_manager_ros2 (地图管理)
```

### 核心服务调用流程

```
1. 用户/应用
   ↓
2. map_manager/load_map → 加载地图
   ↓
3. nav_point_manager/set_map_changed → 加载该地图的导航点
   ↓
4. mission_manager/set_map_changed → 
   ↓ (内部调用)
5. nav_point_manager/get_all_nav_points → 获取所有导航点到mission_manager
```

## 🚀 快速测试

### 一键自动化测试（推荐）

```bash
cd /home/dex/workspace_pzc
source install/setup.bash
bash dexe_mobile_nav/mission_manager/test/quick_integration_test.sh
```

该脚本会：
- ✅ 自动启动所有3个节点
- ✅ 运行完整的集成测试
- ✅ 显示详细的测试结果
- ✅ 自动清理并停止节点

## 📋 测试文件

| 文件 | 说明 |
|------|------|
| `test_integration.py` | Python集成测试脚本 |
| `quick_integration_test.sh` | 一键测试脚本 |
| `README_INTEGRATION.md` | 本文档 |

## 🧪 手动测试步骤

### 1. 启动所有节点

**终端1 - Map Manager:**
```bash
ros2 run map_manager_ros2 map_manager_node
```

**终端2 - Nav Point Manager:**
```bash
ros2 run nav2_point_manager manager_node
```

**终端3 - Mission Manager:**
```bash
ros2 launch mission_manager mission_manager.launch.py
```

### 2. 测试地图管理

**终端4 - 测试命令:**
```bash
# 查看可用地图
ros2 service call /map_manager/get_map_list nav_interfaces/srv/GetMapList

# 加载地图（替换your_map为实际地图名）
ros2 service call /map_manager/load_map nav_interfaces/srv/LoadMap \
  "{map_name: 'your_map'}"
```

### 3. 测试导航点管理

```bash
# 设置地图变更
ros2 service call /nav_point_manager/set_map_changed std_srvs/srv/Trigger

# 获取所有导航点
ros2 service call /nav_point_manager/get_all_nav_points nav_interfaces/srv/GetAllNavPoints
```

### 4. 测试任务管理器

```bash
# 同步导航点到任务管理器
ros2 service call /mission_manager/set_map_changed std_srvs/srv/Trigger

# 获取状态
ros2 service call /mission_manager/get_mission_status nav_interfaces/srv/GetStatus
```

## 🎯 关键测试点

### ✅ 成功标准

1. **Map Manager**
   - 可以加载地图
   - 地图发布到 `/map` 话题

2. **Nav Point Manager**
   - `set_map_changed` 成功
   - 可以获取导航点列表
   - 导航点标记可视化

3. **Mission Manager**
   - `set_map_changed` 不超时
   - 成功从nav_point_manager获取导航点
   - 状态查询正常

### ⚠️ 常见问题

| 问题 | 原因 | 解决方案 |
|------|------|---------|
| set_map_changed超时 | nav_point_manager未运行 | 启动nav_point_manager节点 |
| 地图加载失败 | 地图文件不存在 | 检查地图文件路径 |
| 导航点为空 | 未调用set_map_changed | 先调用set_map_changed |

## 📊 预期性能

- **map_manager/load_map**: < 1秒
- **nav_point_manager/set_map_changed**: < 1秒
- **nav_point_manager/get_all_nav_points**: < 500毫秒
- **mission_manager/set_map_changed**: < 10秒（包含RPC调用）

## 📚 更多文档

- [完整集成测试文档](../INTEGRATION_TEST.md)
- [工作空间级集成测试](../../INTEGRATION_TESTING.md)
- [Mission Manager测试指南](../TEST_GUIDE.md)

## 💡 提示

1. **首次测试**: 使用自动化脚本（quick_integration_test.sh）
2. **调试问题**: 使用手动步骤逐一测试
3. **实际部署**: 确保所有测试都通过后再部署

---

**快速链接**:
- 自动化测试: `bash quick_integration_test.sh`
- Python测试: `python3 test_integration.py`
- 主文档: [INTEGRATION_TEST.md](../INTEGRATION_TEST.md)

