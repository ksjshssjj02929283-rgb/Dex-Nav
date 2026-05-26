# nav2_mppi_controller 与 Navigation2 上游差异对比

本文档记录本项目中 `nav2_mppi_controller` 与 Navigation2 上游的主要差异，作为后续同步/移植的参考依据。

> 对比基准 1：`mobile_navigation/nav2_mppi_controller-main/`，来源为 navigation2 仓库 `main` 分支。
> 对比基准 2：`navigation2-humble_main/nav2_mppi_controller/`，来源为 navigation2 仓库 `humble-main` 分支。
> 文档生成时间：2026-03-20
> 最后更新：2026-05-20（同步 Eigen 迁移后的状态，更新构建依赖、实现体系与后续回归关注点）

---

## 移植状态追踪

| 特性 | 相对 main 状态 | 相对 humble-main 状态 | 备注 |
|------|----------------|-----------------------|------|
| 加速度限制（ax/ay/az） | ⚠️ 部分移植 | ⚠️ 部分移植 | 已接入 `ax/ay/az` 约束链路，但需继续回归验证，详见 §2 |
| SG 滤波调用位置 | ✅ 已移植 | ✅ 已领先 humble-main | 当前已移至 `updateControlSequence()`，与 main 一致 |
| open_loop 模式 | ✅ 已移植 | ✅ 当前扩展项 | 仅 main 有，当前已同步 |
| VelocityDeadbandCritic | ✅ 已移植 | ✅ 已移植 | humble-main 与 main 都已有，当前已同步到 Eigen 实现体系 |
| kinematic_guard | ⏳ 未移植 | 不适用 | 仅 main 有 |
| TrajectoryValidator | ❌ 未移植 | 不适用 | 仅 main 有 |
| 底层库 Eigen | ✅ 已迁移 | ✅ 已对齐 | 当前已使用 `models::Vector` / `models::Matrix` 封装 Eigen Array |

---

## 1. 总体结论

### 1.1 相对 `main`

当前代码与 `main` 的主要差异如下：

| 类别 | 当前代码 | main 分支 | 影响程度 |
|------|---------|----------|---------|
| 底层张量库 | **Eigen**（经 `models::Vector/Matrix` 封装） | **Eigen** | 已对齐 |
| 加速度限制 | ⚠️ **部分移植**（已接入 `ax/ay/az`，仍需回归验证） | **有**（ax/ay/az） | ★★★★★ |
| SG 滤波位置 | `updateControlSequence()` 内 | `updateControlSequence()` 内 | 已对齐 |
| open_loop 模式 | **有** | **有** | 已对齐 |
| TrajectoryValidator 插件 | 无 | **有** | ★★★ 新功能 |
| VelocityDeadbandCritic | **有** | **有** | 已对齐 |
| 动态限速保护 | 无 | **有**（`kinematic_guard`） | ★★ 安全改进 |
| `evalControl()` 返回值 | `TwistStamped` | `tuple<Twist, 最优轨迹>` | ★★ 接口变化 |
| 依赖库 | `nav2_util` / Eigen | `nav2_ros_common` / Eigen | ★★ 仍有框架依赖差异 |

### 1.2 相对 `humble-main`

这次核对后确认，`DIFF_FROM_MAIN.md` 里的部分条目属于 `main` 独有差异，不能直接视为 `humble-main` 也具备：

- `open_loop`：`humble-main` 没有，但当前代码已从 `main` 同步
- `TrajectoryValidator`：`humble-main` 没有
- `kinematic_guard` / `isSpeedLimitActive()` / `reset(false)`：`humble-main` 没有
- SG 滤波移入 `updateControlSequence()`：`humble-main` 没有，仍保留旧调用位置

当前代码相对 `humble-main` 的主要差异如下：

| 类别 | 当前代码 | humble-main | 结论 |
|------|---------|-------------|------|
| 底层张量库 | Eigen | Eigen | 已迁移，主要差异转为具体实现与回归验证 |
| 加速度约束 | `vx/vy/wz` 已接入，仍需验证边界行为 | `vx/vy/wz` 全部接入 | 当前总体已接近等价 |
| VelocityDeadbandCritic | 有 | 有 | 已补齐 |
| PathAngleCritic | 旧实现 | 新实现 | 当前仍存在功能差异，详见 §4.2 |
| PathAlignLegacyCritic | 有 | 无 | 当前保留旧版 critic |
| `ALGORITHM.md` | 有 | 无 | 当前私有文档 |
| `path_handler.*` | 有 | 有 | 这不是差异点 |

---

## 2. 加速度限制差异详解

> 截至 2026-05-20，当前代码已经补入参数、结构体、`vx/vy/wz` 方向的约束链路，并完成 Eigen 迁移。与上游相比主要差异已收敛到具体行为语义、局部实现细节与后续回归验证，而不是底层张量库差异或功能缺失。

### 2.1 结构体与参数层

当前代码已经在 `ControlConstraints` 中加入了：

```cpp
float ax_max;
float ax_min;
float ay_min;
float ay_max;
float az_max;
```

并在 `src/optimizer.cpp` 中读取：

```cpp
getParam(s.base_constraints.ax_max, "ax_max", 3.0f);
getParam(s.base_constraints.ax_min, "ax_min", -3.0f);
getParam(s.base_constraints.ay_max, "ay_max", 3.0f);
getParam(s.base_constraints.ay_min, "ay_min", -3.0f);
getParam(s.base_constraints.az_max, "az_max", 3.5f);
```

当前还做了符号修正：

```cpp
s.base_constraints.ax_max = fabs(s.base_constraints.ax_max);
if (s.base_constraints.ax_min > 0.0) {
  s.base_constraints.ax_min = -1.0 * s.base_constraints.ax_min;
}
if (s.base_constraints.ay_min > 0.0) {
  s.base_constraints.ay_min = -1.0 * s.base_constraints.ay_min;
}
```

这一层与上游方向一致。

### 2.2 采样轨迹阶段：`MotionModel::predict()`

当前代码已对 `vx` / `vy` / `wz` 做逐步约束传播，整体思路已经与上游保持一致：

- `vx` / `vy`：按线加速度上下限做步进裁剪
- `wz`：按 `az_max * dt` 做角速度变化幅度裁剪

因此当前候选轨迹已经具备完整的速度变化平滑性约束，不再是早期文档中记录的“仅 `vx/vy` 生效”。

### 2.3 输出控制序列阶段：`applyControlSequenceConstraints()`

当前代码在 `src/optimizer.cpp` 中已经执行：

```cpp
const float max_delta_vx = s.model_dt * s.constraints.ax_max;
const float min_delta_vx = s.model_dt * s.constraints.ax_min;
const float max_delta_wz = s.model_dt * s.constraints.az_max;
```

随后会对输出序列逐点执行：

- `vx` 的速度幅值裁剪 + 加速度裁剪
- `vy` 的速度幅值裁剪 + 加速度裁剪（全向底盘）
- `wz` 的角速度幅值裁剪 + 角加速度裁剪

这意味着最终输出控制序列已经具备 `ax/ay/az` 完整约束闭环。

### 2.4 `MotionModel::initialize()`

当前代码已经补入：

```cpp
void initialize(const models::ControlConstraints & control_constraints, float model_dt)
{
  control_constraints_ = control_constraints;
  model_dt_ = model_dt;
}
```

并在 `src/optimizer.cpp` 的 `reset()` 中调用：

```cpp
motion_model_->initialize(settings_.constraints, settings_.model_dt);
```

这一点与上游思路一致。

### 2.5 当前实现的准确结论

如果以 `main` 或 `humble-main` 为基线，当前“加速度约束”更准确的描述应为：

- 已补齐：`ControlConstraints` 字段扩展、参数读取、`MotionModel::initialize()`、`vx` / `vy` / `wz` 的约束传播与输出裁剪
- 当前主要差异：底层容器已迁移到 Eigen，但仍需通过回归测试确认边界参数下的行为一致性
- 因此当前不再属于“功能缺失”，更适合描述为“功能已接入，待持续验证”

---

## 3. main 独有而 humble-main 没有的差异

以下内容仍然是当前代码相对 `main` 的真实差异，但不是当前代码相对 `humble-main` 的共同缺口。

### 3.1 SG 滤波调用位置变化

当前代码与 `main` 一致：

```cpp
void Optimizer::updateControlSequence() {
  // softmax 加权求和
  utils::savitskyGolayFilter(control_sequence_, control_history_, settings_);
  applyControlSequenceConstraints();
}
```

优势：

- SG 滤波直接作用于 softmax 聚合后的控制序列，再进入 `applyControlSequenceConstraints()`，顺序更合理
- 滤波结果会继续经过速度/加速度约束裁剪，最终输出更稳定，也更不容易因滤波引入瞬时越界
- 当 `iteration_count > 1` 时，每轮优化后的控制序列都会先平滑再裁剪，整体收敛行为通常比末尾一次性滤波更自然
- 这项改动使当前代码与 `main` 对齐，但相对 `humble-main` 属于前向增强

### 3.2 open_loop 模式

`main` 新增，当前代码也已同步：

```cpp
getParam(s.open_loop, "open_loop", false);
state_.speed = settings_.open_loop ? last_command_vel_ : robot_speed;
```

优势：

- 当底盘速度反馈存在延迟、丢帧、量测异常或低频更新时，优化器可用上一周期已发送控制量作为内部状态推进依据
- 能减少“感知到的当前速度”和“底盘真实执行速度”不一致导致的控制抖动
- 在反馈质量明显不稳定时，通常比继续依赖错误 `odom` 更容易得到可解释、可调的控制行为
- 对串口底盘、网口底盘或闭环速度反馈较慢的系统更实用

### 3.3 TrajectoryValidator 插件系统

`main` 新增 `OptimalTrajectoryValidator`、`trajectory_validators.xml` 以及优化后的轨迹验证流程。

优势：

- 将最优轨迹可行性检查从核心优化流程中独立出来，扩展性更好
- 能对输出轨迹增加更细粒度的合法性判定，降低“优化分数好但实际不可执行”的风险
- 可以支持 soft reset / hard failure 这类更细分的恢复策略，比单纯依赖 critic fail flag 更灵活
- 对后续接入更多安全约束、业务规则约束更友好

### 3.4 kinematic_guard / 动态限速保护

`main` 新增：

- `isSpeedLimitActive()`
- 动态参数更新前置拦截 `kinematic_guard`
- `reset(false)`，在 fallback 时保留动态限速状态

优势：

- 动态限速切换时不容易把约束链路打乱
- fallback 重置时能保留当前有效限速，避免出现“异常恢复后瞬时放开限速”的风险
- 能阻止在 speed limit 激活时动态改写关键运动学参数，减少运行时配置把控制逻辑撕裂的风险
- 对有区域限速、动态限速地图或临时速度约束需求的系统更安全

---

## 4. 当前代码相对 humble-main 还保留什么差异

### 4.1 VelocityDeadbandCritic

`humble-main` 和 `main` 都已有：

- `include/nav2_mppi_controller/critics/velocity_deadband_critic.hpp`
- `src/critics/velocity_deadband_critic.cpp`

当前代码已同步该 critic，因此这项共同缺口已经补齐。

功能说明：

- 对落在速度死区内的 `vx` / `vy` / `wz` 施加代价
- 促使优化器要么输出足够大的可执行速度，要么干脆选择零附近的真正停稳状态，避免"发了命令但底盘不动"的半失效控制
- 对有静摩擦、驱动死区、低速响应不线性的底盘，能直接改善低速抖动和原地微颤
- 当前 `workspace_pzc` 版本已同步到与 `main` 一致的 critic 头文件和核心打分逻辑

**Eigen 适配说明：**

`main` 使用 Eigen，当前代码也已迁移到 Eigen，但通过项目内 `models::Vector` / `models::Matrix` 做了一层类型封装。从 main 同步时优先保持这层封装，不直接在业务接口里扩散裸 `Eigen::Array...` 类型：

| main (Eigen) | 当前代码 |
|---|---|
| `Eigen::ArrayXf` | `models::Vector` |
| `Eigen::ArrayXXf` / row-major matrix | `models::Matrix` |
| `.abs()` / `.max(0.0f)` | 保持 Eigen Array API |
| `.rowwise().sum()` | 保持 Eigen Array API |
| `.pow(n)` / `.square()` | 保持 Eigen Array API |

同步时优先包含 `nav2_mppi_controller/tools/eigen_types.hpp` 或对应 `models/*` 头文件，避免重新引入旧版张量库依赖。

### 4.2 PathAngleCritic

`main` 与 `humble-main` 的 `PathAngleCritic` 当前基本一致，而当前工程中的实现仍保留旧版语义，差异不只是容器 API 适配，而是实际行为已经发生变化。

#### 4.2.1 `main` 与 `humble-main`

两者核心行为基本一致：

- 都支持 `mode` 参数：
  - `FORWARD_PREFERENCE`
  - `NO_DIRECTIONAL_PREFERENCE`
  - `CONSIDER_FEASIBLE_PATH_ORIENTATIONS`
- 都使用更严格的默认参数：
  - `cost_weight = 2.2`
  - `max_angle_to_furthest = 0.785398`（约 45 度）
- 都按**轨迹末端姿态**计算角度代价，而不是对整条轨迹的所有时间步求平均
- `humble-main` 额外读取 `enforce_path_inversion_`，并通过 `utils::getCriticGoal()` 决定近目标时的判定参考目标；`main` 当前则直接使用 `data.state.local_path_length < threshold_to_consider_` 作为早退条件

换句话说，`humble-main` 与 `main` 在 `PathAngleCritic` 上属于同一代实现，只是在“接近目标时如何决定是否启用 critic”这一步上有少量细节差异。

#### 4.2.2 当前工程的旧实现差异

当前 `mobile_navigation/nav2_mppi_controller/src/critics/path_angle_critic.cpp` 相对上游主要差异如下：

- **参数层更旧**
  - 使用 `forward_preference_` 布尔参数
  - 不支持 `mode`
  - 不支持 `CONSIDER_FEASIBLE_PATH_ORIENTATIONS`
  - 不支持 `enforce_path_inversion_`
- **默认参数更宽松**
  - `cost_weight = 2.0`
  - `max_angle_to_furthest = 1.2`（约 68.8 度）
- **启用/早退条件不同**
  - 当前版本用 `withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path)` 判断是否停止打分
  - `humble-main` 用 `getCriticGoal(..., enforce_path_inversion_)`
  - `main` 用 `data.state.local_path_length < threshold_to_consider_`
- **打分方式不同**
  - 当前版本对整条采样轨迹所有时间步都计算“轨迹朝向 vs 指向 offset 路径点方向”的角误差，并取均值
  - 上游新实现只看**每条轨迹末端**的朝向误差
- **反向行驶处理能力较弱**
  - 当前版本只能用 `forward_preference_ = false` 近似表达“不强制前进偏好”
  - 上游可明确区分“前进优先”“无方向偏好”“考虑路径可行朝向”三种模式

#### 4.2.3 行为影响判断

这项差异对实际控制行为是有感知的，尤其在急弯、U 型弯和需要倒车/掉头的路径段：

- 当前版本因为对整条轨迹做平均角误差，更容易在弯前提前放大角度代价
- 在 U 型弯入口，候选轨迹只要前半段尚未充分转头，就可能整条轨迹都被抬高代价
- 上游新实现只看末端姿态，更强调“这条轨迹最终有没有把车头转到合理方向”，对需要先摆姿态再入弯的场景通常更友好
- 当前版本缺少 `mode`，也限制了对倒车段、路径反转段和“按路径朝向执行”的细粒度控制

#### 4.2.4 当前结论

如果后续要继续向 `main` / `humble-main` 同步，`PathAngleCritic` 应视为**尚未完成同步**的功能点，而不是“仅参数略有不同”。

更准确的状态描述应为：

- 相对 `main`：⚠️ 旧实现，尚未同步到新语义版本
- 相对 `humble-main`：⚠️ 旧实现，尚未同步到新语义版本
- 同步难点不在业务逻辑本身，而在于：
  - 需要把上游 Eigen 末端列操作适配到当前 `models::Vector` / `models::Matrix` 类型封装
  - 需要补齐 `mode` / `enforce_path_inversion_` 相关参数与辅助函数调用链

`critics_tests.cpp` 中的 `ParametersHandler` 构造函数也有类似问题：main 使用 `(WeakPtr, std::string&)` 双参数构造，当前代码只有 `(WeakPtr)` 单参数构造，需要移除多余的 `name` 参数。

### 4.2 PathAlignLegacyCritic

当前代码仍保留：

- `include/nav2_mppi_controller/critics/path_align_legacy_critic.hpp`
- `src/critics/path_align_legacy_critic.cpp`

`humble-main` 中已经没有这套 legacy critic，因此这是当前代码的私有保留差异。

功能说明：

- `PathAlignLegacyCritic` 会在预测轨迹上按固定步长采样多个点
- 对每个采样点，遍历整条局部路径片段，寻找几何距离最近的路径点，再累计平均偏差
- 评价目标本质上是“轨迹点尽量贴近路径几何形状”，而不是“沿路径弧长稳定前进”
- 如果 `use_path_orientations` 打开，还会把航向误差一起计入距离
- 当机器人接近目标、尚未真正进入路径跟踪状态，或局部路径被动态障碍占据较多时，该 critic 会主动退出，让 `GoalCritic` / `ObstacleCritic` 接管

局限性：

- 每个轨迹采样点都要做一次“全路径最近点搜索”，计算量更高
- 按纯最近点匹配，容易在回头弯、折返段、局部回环附近发生对齐目标跳变
- 轨迹沿路径前进的语义较弱，更偏几何最近，而不是“沿路径弧长前进”

main 新实现 `PathAlignCritic` 的优势：

- 先计算路径累计弧长，再按轨迹已行进距离去匹配路径点，语义上更接近“沿路径推进”
- 能减少最近点搜索带来的跳点、回头吸附和局部极小值问题，路径跟踪更平滑
- 采样匹配复杂度更低，性能更稳定
- 与 `ObstacleCritic` 的配合逻辑更清晰，局部路径受阻时更容易自然偏离而不是强行贴路径

### 4.3 私有文档与构建差异

当前代码额外包含：

- `ALGORITHM.md`
- Eigen 类型封装与本项目私有迁移文档

这些都不是 `humble-main` 的内容。

---

## 5. 本项目定制修改（与上游 main / humble-main 都无关）

以下是本项目相对于任何版本上游均存在的私有修改，后续同步时需保留：

| 修改点 | 文件/模块 | 说明 |
|--------|-----------|------|
| 控制接口日志 | `nav2_controller/` | 增加 `cmd_vel` 发布日志打印 |
| 尾段路径优化 | `nav2_smac_planner/` | AGX 导航精度调优，约 2cm 误差 |
| 经过点问题修复 | `nav2_bt_navigator/` | `navigate_through_poses` 同频问题修复 |

---

## 6. 移植优先级建议

| 优先级 | 特性 | 工作量估计 | 说明 |
|--------|------|-----------|------|
| P2（中期） | kinematic_guard（§3.4） | 低（1 天） | 提升动态限速安全性 |
| P3（长期） | TrajectoryValidator（§3.3） | 高（1-2 周） | 与 main 新框架耦合更深 |
| 已完成 | 底层库 Eigen 迁移 | 已完成，需回归 | 源码已切到 Eigen；完整测试仍需继续处理 Costmap2DROS 测试生命周期问题 |

---

*文档生成时间：2026-03-20*
*最后核对时间：2026-05-20*
*对应源码分支：`pzc/feature/speedplan`*


## 7. 2026-03-25 会话补充

- 已核对 `mobile_navigation/nav2_mppi_controller/src/critics/velocity_deadband_critic.cpp` 与 `mobile_navigation/nav2_mppi_controller-main/src/critics/velocity_deadband_critic.cpp`，当前实现已迁移到 Eigen，行为逻辑应保持一致，仍需结合回归测试确认。
- 已在 `mobile_navigation/nav2_mppi_controller/test/critics_tests.cpp` 中新增 `VelocityDeadbandCritic` 单测，覆盖“超出死区无代价”和“低于死区产生期望代价”两类场景。
- 已在 `mobile_navigation/nav2_bringup/params/w1_nav2_params.yaml` 与 `mobile_navigation/nav2_bringup/params/w1_nav2_rgbd_params.yaml` 中将 `VelocityDeadbandCritic` 接入 MPPI `critics` 列表，并增加参数：`cost_power: 1`、`cost_weight: 35.0`、`deadband_velocities: [0.08, 0.08, 0.08]`。
- 当前 `motion_model` 配置为 `DiffDrive` 时，该 critic 实际只对 `vx` / `wz` 生效；`vy` 参数保留用于接口一致性或后续全向底盘场景。

*会话补充时间：2026-03-25*
