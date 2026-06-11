# Mission Manager 测试文档

## 测试文件说明

### 1. test_params.py
测试参数文件的格式和内容

**测试内容**:
- 参数文件是否存在
- YAML格式是否正确
- 必需参数是否都存在
- 参数类型是否正确
- 参数值是否合理

**运行方法**:
```bash
cd /home/dex/workspace_pzc/dexe_mobile_nav/mission_manager/test
python3 test_params.py
```

**预期输出**:
```
测试参数文件: .../mission_manager/params/mission_manager_params.yaml
✅ 参数文件存在
✅ 参数文件加载成功
✅ 参数文件格式正确
✅ mission_manager参数组存在
✅ 参数 auto_charge_threshold = 30.0 (float)
✅ 参数 force_charge_threshold = 20.0 (float)
✅ 参数 respawn_task = False (bool)
```

### 2. test_mission_manager.py
测试Mission Manager节点的ROS2服务接口

**测试内容**:
- 设置地图
- 获取管理器状态
- 清空所有任务
- 暂停和恢复任务
- 取消任务
- 获取当前任务信息

**前置条件**:
需要先启动mission_manager节点

**运行方法**:
```bash
# 终端1: 启动mission_manager节点
cd /home/dex/workspace_pzc
source install/setup.bash
ros2 launch mission_manager mission_manager.launch.py

# 终端2: 运行测试
cd /home/dex/workspace_pzc
source install/setup.bash
python3 /home/dex/workspace_pzc/dexe_mobile_nav/mission_manager/test/test_mission_manager.py
```

## 快速测试

### 1. 测试参数文件
```bash
cd /home/dex/workspace_pzc/dexe_mobile_nav/mission_manager/test
python3 test_params.py
```

### 2. 测试节点启动
```bash
cd /home/dex/workspace_pzc
source install/setup.bash
ros2 launch mission_manager mission_manager.launch.py
```

### 3. 检查节点是否运行
```bash
# 新终端
cd /home/dex/workspace_pzc
source install/setup.bash
ros2 node list | grep mission_manager
```

### 4. 检查服务是否可用
```bash
ros2 service list | grep mission_manager
```

### 5. 手动测试服务
```bash
# 获取状态
ros2 service call /mission_manager/get_mission_status nav_interfaces/srv/GetStatus

# 清空任务
ros2 service call /mission_manager/clear_all_tasks std_srvs/srv/Trigger

# 获取当前任务信息
ros2 service call /mission_manager/get_current_task_info nav_interfaces/srv/GetCurrentTaskInfo
```

### 6. 查看参数
```bash
ros2 param list | grep mission_manager
ros2 param get /mission_manager mission_manager.auto_charge_threshold
ros2 param get /mission_manager mission_manager.force_charge_threshold
ros2 param get /mission_manager mission_manager.respawn_task
```

### 7. 修改运行时参数
```bash
# 修改自动充电阈值
ros2 param set /mission_manager mission_manager.auto_charge_threshold 25.0

# 启用任务重启
ros2 param set /mission_manager mission_manager.respawn_task true
```

## 测试场景

### 场景1: 基本功能测试
1. 启动节点
2. 检查参数加载
3. 测试获取状态
4. 测试清空任务

### 场景2: 任务控制测试
1. 添加任务（需要nav_point_manager支持）
2. 暂停任务
3. 恢复任务
4. 取消任务

### 场景3: 参数配置测试
1. 使用自定义参数文件启动
2. 运行时修改参数
3. 验证参数生效

## 注意事项

1. **依赖服务**: 某些测试需要其他节点支持（如nav_point_manager）
2. **地图设置**: add_task服务需要先调用set_map_changed
3. **超时设置**: 服务调用超时设置为5秒
4. **日志输出**: 测试过程会输出详细日志

## 常见问题

### Q1: 测试失败 - 服务不可用
**原因**: mission_manager节点未启动或服务未就绪
**解决**: 先启动mission_manager节点，等待服务就绪

### Q2: 添加任务失败 - Map not set
**原因**: 未调用set_map_changed服务
**解决**: 先调用set_map_changed服务

### Q3: 参数加载失败
**原因**: 参数文件路径不正确或格式错误
**解决**: 检查参数文件是否存在，YAML格式是否正确

## 测试覆盖率

当前测试覆盖:
- ✅ 参数文件格式验证
- ✅ ROS2服务接口调用
- ✅ 状态查询功能
- ✅ 任务控制功能
- ⚠️  任务添加功能（需要额外依赖）
- ⚠️  任务执行流程（需要完整系统）

## 后续改进

1. 添加集成测试（包含nav_point_manager等依赖）
2. 添加性能测试
3. 添加异常场景测试
4. 自动化测试脚本

