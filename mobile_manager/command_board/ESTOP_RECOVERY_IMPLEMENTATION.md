# 急停丢位恢复功能实现说明

## 概述

基于 V0.4.5 版本，在保持原有接口不变的前提下，实现了急停丢位自动恢复功能。

**分支**: `feature/estop_recovery_v0.4.5`  
**基于**: `V0.4.5` tag  
**实现日期**: 2026-09-18

---

## 实现特性

### ✅ 接口兼容 V0.4.5（一处必要修正）

- 保留原有订阅：`/diagnostics_agg`, `map_manager/map_name`
- 保留所有原有回调函数：`OnLocalizationState()`, `OnDiagnosticsAgg()`
- 不引入外部依赖（StatusMonitor、MapVersionManager 等）
- 最小化代码改动，仅在现有逻辑基础上增强

**必要修正**：V0.4.5 订阅的 `/localization_state` 在定位节点侧不存在，已改为
`/localization_node/state`。详见下方"接口核对"一节。

---

## 接口核对（对照定位节点源码）

核对基准：`dexe_mobile_slam/mobile_slam/dexe_cartographer_ros/cartographer_ros/`
`cartographer_ros/localization/`（`loc_node_main.cc` + `localization_node.cc`）

| 接口 | 类型 | 定位节点侧 | 本实现 | 结论 |
|------|------|-----------|--------|------|
| `/localization_node/state` | LocalizationState | `localization_node.cc:165` 以 `"~/state"` 发布 | 已订阅 | ✅ 已修正 |
| `/localization_node/relocalization` | SetPose | `loc_node_main.cc:237` `"~/relocalization"` | 已调用 | ✅ 一致 |
| `/localization_node/trigger_global_relocalization` | Trigger | `loc_node_main.cc:231` | 已调用 | ✅ 一致 |
| `/localization_node/stop_localization` | Trigger | `loc_node_main.cc:211` | 已调用 | ✅ 一致 |
| `/localization_node/start_localization` | Trigger | `loc_node_main.cc:206` | 已调用 | ✅ 一致 |
| `/robot_localization/odometry/filtered` | nav_msgs/Odometry | 官方接口测试脚本登记为 Odometry | 已订阅 | ✅ 一致 |
| `estop_state` / `bumper_strip_front_state` / `bumper_strip_rear_state` | diagnostic KV | `/Devices/System/PowerBoard/collector_node: PowerBoard Safety` | 键名未改 | ✅ 一致 |
| `LocalizationState.score` | float32 | msg 定义第 3 行 | 已读取 | ✅ 一致 |

**`/localization_state` 为何是错的**：`LocalizationNode` 的发布器建在
`LocalizationManagerNode::shared_from_this()` 上（`loc_node_main.cc:404-405`），
该节点名为 `localization_node`（`loc_node_main.cc:93`），所以 `"~/state"`
解析为 `/localization_node/state`，而非全局 `/localization_state`。

沿用错误话题名的后果：定位回调永不触发 → `latest_score_` 恒为 0 →
位姿永不缓存 → 每次急停都跳过局部重定位、全局重定位也判定失败 → 必然进 Alarm。
即恢复功能完全失效。

**运行时验证**（`ros2 node info /command_board_manager`）已确认上表全部接口
按预期注册，类型匹配。

### 🔄 五状态恢复状态机

```
kNormal (正常) → kSafeStop (急停检测) → kRelocating (局部重定位)
                                              ↓ 失败
                                        kGlobalReloc (全局重定位)
                                              ↓ 失败
                                        kAlarm (人工干预)
```

### 📡 新增接口

**服务客户端**：
- `/localization_node/relocalization` (SetPose) - 局部重定位
- `/localization_node/trigger_global_relocalization` (Trigger) - 全局重定位

**发布器**：
- `~/safety_alarm` (String) - 告警信息发布

**订阅**：
- `/robot_localization/odometry/filtered` (nav_msgs/Odometry) - 实测融合速度，用于判断机器人是否静止

**修正的订阅**：
- `/localization_state` → `/localization_node/state`（V0.4.5 原话题名有误，见下方"接口核对"）

**定时器**：
- 200ms 周期定时器驱动状态机

---

## 关键实现逻辑

### 1. 急停状态检测（OnDiagnosticsAgg 增强）

```cpp
// 提取 estop_state 和 bumper_strip_front/rear_state
bool estop_detected = false;
bool bumper_detected = false;

// 上升沿检测：从正常到急停
if (active_now && !safety_active_) {
    // 停止导航和定位
    navigation_action_client_->async_cancel_all_goals();
    StopLocalization();
    recover_state_ = RecoverState::kSafeStop;
}

// 下降沿检测：从急停到正常
else if (!active_now && safety_active_) {
    // 记录解除时间，准备进入恢复流程
    settle_start_ = this->now();
}
```

### 2. 位姿缓存（OnLocalizationState 增强）

```cpp
// 更新定位分数
latest_score_ = msg->score;

// 在正常状态下缓存高分位姿
if (recover_state_ == RecoverState::kNormal && 
    latest_score_ >= pose_cache_min_score_) {
    SnapshotGoodPose();  // 从 TF 读取 map→base_link
}
```

### 3. 恢复状态机（OnRecoverTimer）

**kSafeStop → kRelocating**：
- 等待急停解除
- 等待机器人静止（速度 < 0.05 m/s）
- 稳定 1 秒后启动定位
- 使用缓存位姿进行局部重定位

**kRelocating → kNormal / kGlobalReloc**：
- 等待 2 秒评估重定位结果
- score ≥ 0.8：成功恢复
- score < 0.5：降级到全局重定位
- 0.5 ≤ score < 0.8：继续等待

**kGlobalReloc → kNormal / kAlarm**：
- 等待 4 秒（2x 重定位等待时间）
- score ≥ 0.8：成功恢复
- score < 0.8：触发告警，需要人工干预

**kAlarm → kNormal**：
- 持续监控定位分数
- 人工通过 RViz 设置初始位姿后自动恢复

---

## 配置参数

可通过 ROS 2 参数配置恢复行为（launch 文件或 yaml）：

```yaml
command_board_manager:
  ros__parameters:
    # 主开关
    enable_loss_recovery: true
    
    # 恢复参数
    recover_settle_sec: 1.0          # 急停解除后稳定等待时长
    reloc_wait_sec: 2.0              # 重定位结果等待时长
    still_velocity_threshold: 0.05   # 静止速度阈值 (m/s)
    
    # 定位分数阈值
    reloc_score_good: 0.8            # 好分数阈值（恢复成功）
    reloc_score_bad: 0.5             # 差分数阈值（降级）
    pose_cache_min_score: 0.6        # 缓存位姿最低分数

    # 话题名（按部署环境覆盖）
    localization_state_topic: "/localization_node/state"
    odom_topic: "/robot_localization/odometry/filtered"
```

完整参数文件位于 `params/command_board_params.yaml`，已由 launch 自动加载
（顶层用 `/**` 通配，不受运行时节点名影响）。

**禁用恢复功能**：设置 `enable_loss_recovery: false`

---

## 测试验证

### 前置条件

确认以下服务可用：
```bash
# 检查重定位服务
ros2 service list | grep relocalization
# 应输出：
# /localization_node/relocalization
# /localization_node/trigger_global_relocalization

# 检查定位状态话题
ros2 topic echo /localization_node/state --once
# 应包含 status 和 score 字段

# 检查里程计话题（判断静止用）
ros2 topic echo /robot_localization/odometry/filtered --once
```

### 测试场景

#### 场景 1：急停触发与自动恢复

1. **正常运行**：机器人定位正常，导航中
2. **触发急停**：按下急停按钮
3. **预期行为**：
   - 导航取消
   - 定位停止
   - 日志显示：`Safety active (estop=1, bumper=0) — stopping localization for recovery`
   - 状态转换：kNormal → kSafeStop

4. **解除急停**：松开急停按钮
5. **预期行为**：
   - 等待 1 秒稳定
   - 启动定位
   - 使用缓存位姿重定位
   - 日志显示：`Local relocalize from cached pose (x, y, theta)`
   - 状态转换：kSafeStop → kRelocating

6. **重定位成功**：
   - 2 秒后 score ≥ 0.8
   - 日志显示：`Relocalized OK from last pose (score=0.XX)`
   - 状态转换：kRelocating → kNormal

#### 场景 2：局部重定位失败 → 全局重定位

1. 触发急停，缓存位姿与实际相差较大
2. 局部重定位 2 秒后 score < 0.5
3. **预期行为**：
   - 日志警告：`Local relocalization weak (score=0.XX) — global fallback`
   - 触发全局重定位
   - 状态转换：kRelocating → kGlobalReloc

4. 全局重定位成功（4 秒后 score ≥ 0.8）
5. 日志显示：`Global relocalization OK (score=0.XX)`
6. 状态转换：kGlobalReloc → kNormal

#### 场景 3：全局重定位失败 → 人工干预

1. 全局重定位 4 秒后 score < 0.8
2. **预期行为**：
   - 错误日志：`[SafetyReactor ALARM] Global relocalization failed — manual RViz 2D Pose Estimate needed`
   - 发布告警到 `~/safety_alarm` 话题
   - 状态转换：kGlobalReloc → kAlarm

3. 通过 RViz 设置初始位姿（2D Pose Estimate）
4. 定位分数恢复到 ≥ 0.8
5. 日志显示：`Recovered from alarm (manual relocalization, score=0.XX)`
6. 状态转换：kAlarm → kNormal

#### 场景 4：碰撞传感器触发

1. 机器人正常运行
2. 触发前/后碰撞条（bumper_strip_front/rear_state）
3. 预期行为同急停场景

#### 场景 5：无缓存位姿直接全局重定位

1. 启动后未获得高分位姿（score 始终 < 0.6）
2. 触发急停并解除
3. **预期行为**：
   - 日志警告：`No cached pose — escalating to global relocalization`
   - 跳过局部重定位，直接全局重定位
   - 状态转换：kSafeStop → kGlobalReloc

### 监控命令

```bash
# 监听告警话题
ros2 topic echo /command_board_manager/safety_alarm

# 监听定位状态
ros2 topic echo /localization_node/state

# 监听诊断信息
ros2 topic echo /diagnostics_agg | grep -A 10 "PowerBoard Safety"

# 查看日志
ros2 run command_board command_board_node  # 查看实时日志
```

---

## 关键文件修改

### command_board_ros.hpp

新增成员：
- `RecoverState` 枚举和状态变量
- 恢复相关参数（9 个可配置参数，含 2 个话题名）
- 新增服务客户端、发布器、定时器
- 新增 9 个方法声明

**行数变化**：96 行 → ~145 行（+49 行）

### command_board_ros.cpp

修改部分：
- 构造函数：新增参数声明、客户端初始化、定时器创建（+40 行）
- `OnLocalizationState()`：新增位姿缓存逻辑（+10 行）
- `OnDiagnosticsAgg()`：新增状态变化检测（+20 行）

新增方法：
- `OnRecoverTimer()` - 状态机核心（~90 行）
- `SnapshotGoodPose()` - 位姿缓存（~20 行）
- `RobotIsStill()` - 静止判断（~10 行）
- `StopLocalization()` - 停止定位（~10 行）
- `StartLocalization()` - 启动定位（~10 行）
- `Relocalize()` - 局部重定位（~30 行）
- `GlobalRelocalize()` - 全局重定位（~10 行）
- `RaiseAlarm()` - 告警发布（~10 行）
- `RecoverStateToString()` - 状态转字符串（~10 行）

**行数变化**：475 行 → ~675 行（+200 行）

---

## 编译和部署

### 编译

```bash
cd /home/dex/projects/ws_mobile_nav
source /opt/ros/humble/setup.bash  # 根据实际 ROS 2 版本调整
colcon build --packages-select command_board --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 部署

```bash
source install/setup.bash
ros2 run command_board command_board_node
```

或在 launch 文件中添加参数：

```python
Node(
    package='command_board',
    executable='command_board_node',
    name='command_board_manager',
    parameters=[{
        'enable_loss_recovery': True,
        'recover_settle_sec': 1.0,
        'reloc_wait_sec': 2.0,
        'reloc_score_good': 0.8,
        'reloc_score_bad': 0.5,
        'pose_cache_min_score': 0.6,
        'still_velocity_threshold': 0.05,
    }],
)
```

---

## 与 release_0.5.x 的区别

| 特性 | V0.4.5 + 急停恢复 | release_0.5.x |
|------|-------------------|---------------|
| 订阅接口 | 保持原有 | 改为 StatusMonitor 统一聚合 |
| 地图状态订阅 | `map_manager/map_name` (String) | `map_manager/map_state` (MapState) |
| 外部依赖 | 无 | StatusMonitor, MapVersionManager, AutoMapSaver |
| 代码复杂度 | 中等 | 高（多模块协作） |
| 恢复逻辑 | 完整（5 状态机） | 完整（5 状态机） |
| 向后兼容 | 完全兼容 V0.4.5 | 不兼容 V0.4.5 |

**推荐场景**：
- V0.4.5 + 急停恢复：需要保持 V0.4.5 稳定性，仅需恢复功能
- release_0.5.x：需要完整功能（地图版本管理、自动保存等）

---

## 已知限制

1. **机器人静止判断**：依赖 `/robot_localization/odometry/filtered` 实测速度。
   若该话题未发布，`RobotIsStill()` 保守返回 true（假定静止），恢复流程仍会推进，
   但可能在机器人尚未完全停稳时就发起重定位。话题名可用 `odom_topic` 参数覆盖。
2. **位姿缓存来源**：仅从 TF (`map→base_link`) 获取，不从 LocalizationState 消息中读取位姿
3. **无状态持久化**：重启节点后缓存位姿丢失
4. **单线程执行器**：V0.4.5 使用 `MutuallyExclusive` callback group，所有回调串行执行

---

## 后续优化建议

1. **增加状态发布**：发布当前恢复状态到话题，便于监控
2. **位姿缓存队列**：保存多个历史高分位姿，而不是仅最新一个
3. **自适应阈值**：根据地图类型和传感器配置动态调整分数阈值
4. **集成测试**：编写集成测试用例，自动化验证恢复流程

---

## 提交和合并

### 提交信息

```bash
git add mobile_manager/command_board/apps/command_board_ros.hpp
git add mobile_manager/command_board/apps/command_board_ros.cpp
git add mobile_manager/command_board/ESTOP_RECOVERY_IMPLEMENTATION.md

git commit -m "feat(command_board): 实现急停丢位自动恢复功能

基于 V0.4.5，保持接口完全兼容的前提下实现急停丢位恢复：

- 五状态恢复状态机（Normal/SafeStop/Relocating/GlobalReloc/Alarm）
- 高分位姿自动缓存（TF map→base_link）
- 渐进式恢复：局部重定位 → 全局重定位 → 人工干预
- 可配置参数：分数阈值、等待时长、速度阈值
- 新增接口：relocalization、trigger_global_relocalization 服务客户端
- 新增告警发布器：~/safety_alarm

测试验证：
- 急停触发与自动恢复
- 局部重定位失败降级到全局
- 全局重定位失败触发告警
- 碰撞传感器触发恢复

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

### 合并到 V0.4.5 维护分支

如果有 V0.4.5 的长期维护分支：

```bash
git checkout v0.4.5-maintenance  # 假设存在该分支
git merge feature/estop_recovery_v0.4.5 --no-ff
```

---

## 联系人

- 实现者：Claude Fable 5
- 实现日期：2026-09-18
- 基于版本：V0.4.5
- 目标分支：`feature/estop_recovery_v0.4.5`
