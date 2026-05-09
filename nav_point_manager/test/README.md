# nav_point_manager 测试脚本说明

## 测试脚本列表

### 1. test_navigation.py
**功能**：发送导航目标点并接收导航结果

**特点**：
- 创建 `/mission_manager_node/set_navigation_success` 服务服务器，模拟 mission_manager 接收导航结果
- 可以发送带自定义 tolerance 的导航目标点
- 测试两种场景：自定义 tolerance 和默认 tolerance

**运行**：
```bash
ros2 run nav_point_manager test_navigation.py
# 或直接运行
./test_navigation.py
```

**测试案例**：
- 测试 1：发送目标点 (1.0, 2.0, 0.0)，tolerance=0.1
- 测试 2：发送目标点 (2.0, 3.0, 1.57)，tolerance=0.0（使用默认值）

---

### 2. test_status_monitor.py
**功能**：定时查询导航状态（每1秒查询一次）

**特点**：
- 调用 `/nav_point_manager_node/get_navigation_status` 服务
- 实时显示当前导航状态和目标点名称
- 当查询到 `succeeded` 状态时，该状态会被消费并切换为 `idle`

**运行**：
```bash
./test_status_monitor.py
```

**输出示例**：
```
[STATUS] idle
[STATUS] navigating - Point: point_1
[STATUS] SUCCEEDED - Point: point_1
[STATUS] idle
```

---

### 3. test_cancel_navigation.py
**功能**：取消当前导航

**特点**：
- 调用 `/nav_point_manager_node/cancel_navigation` 服务
- 立即返回取消结果

**运行**：
```bash
./test_cancel_navigation.py
```

---

### 4. test_tolerance_monitor.py
**功能**：监控 controller_server 的 lon/lat_goal_tolerance 参数（每1秒查询一次）

**特点**：
- 查询 `/controller_server/get_parameters` 服务
- 同时监控 `lon_goal_tolerance` (x方向) 和 `lat_goal_tolerance` (y方向)
- 只在参数值变化时打印，避免刷屏
- 显示参数变化前后的值

**运行**：
```bash
./test_tolerance_monitor.py
```

**输出示例**：
```
[TOLERANCE] lon_goal_tolerance = 0.0200, lat_goal_tolerance = 0.0200
[TOLERANCE] lon_goal_tolerance = 0.1000, lat_goal_tolerance = 0.1000
  Changed from (lon=0.0200, lat=0.0200) to (lon=0.1000, lat=0.1000)
```

---

### 5. test_nav_through_poses_file.py
**功能**：调用路网导航服务，从 JSON 文件读取路径点序列进行导航

**特点**：
- 调用 `/nav_point_manager_node/nav_through_poses_file` 服务
- 输入 JSON 文件名（位于 config 目录）
- 自动读取文件中的路径点并发送给 Nav2 的 NavigateThroughPoses action
- 不会在每个点停顿，适合路网连续导航

**运行**：
```bash
./test_nav_through_poses_file.py
```

**JSON 文件格式**（位于 `config/` 目录）：
```json
[
  {"x": 1.0, "y": 2.0, "theta": 0.0},
  {"x": 2.0, "y": 3.0, "theta": 1.57},
  {"x": 3.0, "y": 2.5, "theta": 3.14}
]
```

**输出示例**：
```
Sending request with filename: example_route.json
SUCCESS: NavigateThroughPoses goal sent successfully
```

---

## 完整测试流程

### 终端 1：启动 nav_point_manager
```bash
ros2 launch nav_point_manager nav_point_manager.launch.py
```

### 终端 2：监控导航状态
```bash
cd /home/dex/workspace_pzc/dexe_mobile_application/mobile_manager/nav_point_manager/test
./test_status_monitor.py
```

### 终端 3：监控 tolerance 参数
```bash
cd /home/dex/workspace_pzc/dexe_mobile_application/mobile_manager/nav_point_manager/test
./test_tolerance_monitor.py
```

### 终端 4：发送导航目标
```bash
cd /home/dex/workspace_pzc/dexe_mobile_application/mobile_manager/nav_point_manager/test
./test_navigation.py
```

### 终端 5（可选）：取消导航
```bash
cd /home/dex/workspace_pzc/dexe_mobile_application/mobile_manager/nav_point_manager/test
./test_cancel_navigation.py
```

---

## 注意事项

1. 运行测试前确保 Nav2 已启动
2. `test_navigation.py` 会模拟 mission_manager 的服务，如果真实的 mission_manager 在运行，可能会冲突
3. 所有脚本都需要 ROS2 环境已 source
4. 可以修改 `test_navigation.py` 中的坐标和 tolerance 值进行不同场景测试
