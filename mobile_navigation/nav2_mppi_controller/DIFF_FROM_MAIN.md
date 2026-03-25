# nav2_mppi_controller 与 Navigation2 上游差异对比

本文档记录本项目中 `nav2_mppi_controller` 与 Navigation2 上游的主要差异，作为后续同步/移植的参考依据。

> 对比基准 1：`mobile_navigation/nav2_mppi_controller-main/`，来源为 navigation2 仓库 `main` 分支。
> 对比基准 2：`navigation2-humble_main/nav2_mppi_controller/`，来源为 navigation2 仓库 `humble-main` 分支。
> 文档生成时间：2026-03-20
> 最后更新：2026-03-24（补充 humble-main 对比，修正 SG/open_loop/VelocityDeadbandCritic 状态，并补充 PathAlignLegacyCritic 与 main 新功能优势说明；修复 VelocityDeadbandCritic 和 critics_tests xtensor 适配问题）

---

## 移植状态追踪

| 特性 | 相对 main 状态 | 相对 humble-main 状态 | 备注 |
|------|----------------|-----------------------|------|
| 加速度限制（ax/ay/az） | ⚠️ 部分移植 | ⚠️ 部分移植 | 已接入 `ax/ay/az` 约束链路，但需继续回归验证，详见 §2 |
| SG 滤波调用位置 | ✅ 已移植 | ✅ 已领先 humble-main | 当前已移至 `updateControlSequence()`，与 main 一致 |
| open_loop 模式 | ✅ 已移植 | ✅ 当前扩展项 | 仅 main 有，当前已同步 |
| VelocityDeadbandCritic | ✅ 已移植 | ✅ 已移植 | humble-main 与 main 都已有，当前已同步（需 xtensor 适配） |
| kinematic_guard | ⏳ 未移植 | 不适用 | 仅 main 有 |
| TrajectoryValidator | ❌ 未移植 | 不适用 | 仅 main 有 |
| 底层库 xtensor → Eigen | ❌ 不移植 | ❌ 不移植 | 架构级差异，工作量过大 |

---

## 1. 总体结论

### 1.1 相对 `main`

当前代码与 `main` 的主要差异如下：

| 类别 | 当前代码 | main 分支 | 影响程度 |
|------|---------|----------|---------|
| 底层张量库 | **xtensor** | **Eigen** | ★★★★★ 架构级 |
| 加速度限制 | ⚠️ **部分移植**（已接入 `ax/ay/az`，仍需回归验证） | **有**（ax/ay/az） | ★★★★★ |
| SG 滤波位置 | `updateControlSequence()` 内 | `updateControlSequence()` 内 | 已对齐 |
| open_loop 模式 | **有** | **有** | 已对齐 |
| TrajectoryValidator 插件 | 无 | **有** | ★★★ 新功能 |
| VelocityDeadbandCritic | **有** | **有** | 已对齐 |
| 动态限速保护 | 无 | **有**（`kinematic_guard`） | ★★ 安全改进 |
| `evalControl()` 返回值 | `TwistStamped` | `tuple<Twist, 最优轨迹>` | ★★ 接口变化 |
| 依赖库 | `nav2_util` / xtensor | `nav2_ros_common` / Eigen | ★★ 构建差异 |

### 1.2 相对 `humble-main`

这次核对后确认，`DIFF_FROM_MAIN.md` 里的部分条目属于 `main` 独有差异，不能直接视为 `humble-main` 也具备：

- `open_loop`：`humble-main` 没有，但当前代码已从 `main` 同步
- `TrajectoryValidator`：`humble-main` 没有
- `kinematic_guard` / `isSpeedLimitActive()` / `reset(false)`：`humble-main` 没有
- SG 滤波移入 `updateControlSequence()`：`humble-main` 没有，仍保留旧调用位置

当前代码相对 `humble-main` 的主要差异如下：

| 类别 | 当前代码 | humble-main | 结论 |
|------|---------|-------------|------|
| 底层张量库 | xtensor | Eigen | 架构级差异仍然存在 |
| 加速度约束 | `vx/vy/wz` 已接入，仍需验证边界行为 | `vx/vy/wz` 全部接入 | 当前总体已接近等价 |
| VelocityDeadbandCritic | 有 | 有 | 已补齐 |
| PathAlignLegacyCritic | 有 | 无 | 当前保留旧版 critic |
| `ALGORITHM.md` | 有 | 无 | 当前私有文档 |
| `path_handler.*` | 有 | 有 | 这不是差异点 |

---

## 2. 加速度限制差异详解

> 截至 2026-03-24，当前代码已经补入参数、结构体、`vx/vy/wz` 方向的约束链路，与上游相比主要差异已收敛到 xtensor / Eigen 实现体系与后续回归验证，而不是功能缺失。

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
- 当前主要差异：xtensor 与 Eigen 的实现细节不同，仍需通过回归测试确认边界参数下的行为一致性
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

**xtensor 适配说明：**

`main` 使用 Eigen，当前代码使用 xtensor。从 main 同步时需要将 Eigen 风格调用转换为 xtensor 等价写法：

| main (Eigen) | 当前代码 (xtensor) |
|---|---|
| `.abs()` | `xt::abs()` |
| `.max(0.0f)` | `xt::maximum(..., 0.0f)` |
| `.rowwise().sum()` | `xt::sum(..., {1}, immediate)` |
| `.pow(n)` | `xt::pow(..., n)` |

同步时还需添加 `#include <xtensor/xmath.hpp>` 和 `using xt::evaluation_strategy::immediate;`。

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
- xtensor / xsimd 相关构建与实现

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
| P3（长期） | 底层库 xtensor → Eigen | 极高（2-3 周） | 架构级改动，建议单独立项 |

---

*文档生成时间：2026-03-20*
*最后核对时间：2026-03-24*
*对应源码分支：`pzc/feature/speedplan`*


## 7. 2026-03-25 会话补充

- 已核对 `mobile_navigation/nav2_mppi_controller/src/critics/velocity_deadband_critic.cpp` 与 `mobile_navigation/nav2_mppi_controller-main/src/critics/velocity_deadband_critic.cpp`，确认差异仅为 xtensor 适配与格式，行为逻辑一致。
- 已在 `mobile_navigation/nav2_mppi_controller/test/critics_tests.cpp` 中新增 `VelocityDeadbandCritic` 单测，覆盖“超出死区无代价”和“低于死区产生期望代价”两类场景。
- 已在 `mobile_navigation/nav2_bringup/params/w1_nav2_params.yaml` 与 `mobile_navigation/nav2_bringup/params/w1_nav2_rgbd_params.yaml` 中将 `VelocityDeadbandCritic` 接入 MPPI `critics` 列表，并增加参数：`cost_power: 1`、`cost_weight: 35.0`、`deadband_velocities: [0.08, 0.08, 0.08]`。
- 当前 `motion_model` 配置为 `DiffDrive` 时，该 critic 实际只对 `vx` / `wz` 生效；`vy` 参数保留用于接口一致性或后续全向底盘场景。

*会话补充时间：2026-03-25*
