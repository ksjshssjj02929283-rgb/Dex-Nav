# nav2_mppi_controller 与 Navigation2 上游差异对比

本文档记录本项目中 `nav2_mppi_controller` 与 Navigation2 上游的主要差异，作为后续同步/移植的参考依据。

> 对比基准 1：`mobile_navigation/nav2_mppi_controller-main/`，来源为 navigation2 仓库 `main` 分支。
> 对比基准 2：`navigation2-humble_main/nav2_mppi_controller/`，来源为 navigation2 仓库 `humble-main` 分支。
> 文档生成时间：2026-03-20
> 最后更新：2026-03-24（补充 humble-main 对比，并修正加速度约束移植状态）

---

## 移植状态追踪

| 特性 | 相对 main 状态 | 相对 humble-main 状态 | 备注 |
|------|----------------|-----------------------|------|
| 加速度限制（ax/ay/az） | ⚠️ 部分移植 | ⚠️ 部分移植 | 已接入 `ax/ay` 约束链路，`az_max` 参数已加但角加速度约束尚未完整生效，详见 §2 |
| SG 滤波调用位置 | ⏳ 未移植 | 与 humble-main 一致 | main 已移至 `updateControlSequence()`；humble-main 仍是旧位置 |
| open_loop 模式 | ⏳ 未移植 | 不适用 | 仅 main 有 |
| VelocityDeadbandCritic | ⏳ 未移植 | ⏳ 未移植 | humble-main 与 main 都已有 |
| kinematic_guard | ⏳ 未移植 | 不适用 | 仅 main 有 |
| TrajectoryValidator | ❌ 未移植 | 不适用 | 仅 main 有 |
| 底层库 xtensor → Eigen | ❌ 不移植 | ❌ 不移植 | 架构级差异，工作量过大 |

---

## 1. 总体结论

### 1.1 相对 `main`

当前代码与 `main` 的主要差异仍然成立：

| 类别 | 当前代码 | main 分支 | 影响程度 |
|------|---------|----------|---------|
| 底层张量库 | **xtensor** | **Eigen** | ★★★★★ 架构级 |
| 加速度限制 | ⚠️ **部分移植**（`ax/ay` 已接入，`az` 未完整生效） | **有**（ax/ay/az） | ★★★★★ |
| SG 滤波位置 | `evalControl()` 末尾 | `updateControlSequence()` 内 | ★★★ 行为差异 |
| open_loop 模式 | 无 | **有** | ★★★ 新功能 |
| TrajectoryValidator 插件 | 无 | **有** | ★★★ 新功能 |
| VelocityDeadbandCritic | 无 | **有** | ★★ 新 Critic |
| 动态限速保护 | 无 | **有**（`kinematic_guard`） | ★★ 安全改进 |
| `evalControl()` 返回值 | `TwistStamped` | `tuple<Twist, 最优轨迹>` | ★★ 接口变化 |
| 依赖库 | `nav2_util` / xtensor | `nav2_ros_common` / Eigen | ★★ 构建差异 |

### 1.2 相对 `humble-main`

这次核对后确认，`DIFF_FROM_MAIN.md` 里的部分条目属于 `main` 独有差异，不能直接视为 `humble-main` 也具备：

- `open_loop`：`humble-main` 没有
- `TrajectoryValidator`：`humble-main` 没有
- `kinematic_guard` / `isSpeedLimitActive()` / `reset(false)`：`humble-main` 没有
- SG 滤波移入 `updateControlSequence()`：`humble-main` 没有，仍保留旧调用位置

当前代码相对 `humble-main` 的主要差异如下：

| 类别 | 当前代码 | humble-main | 结论 |
|------|---------|-------------|------|
| 底层张量库 | xtensor | Eigen | 架构级差异仍然存在 |
| 加速度约束 | `vx/vy` 已接入，`wz/az_max` 未完整生效 | `vx/vy/wz` 全部接入 | 当前为**部分移植** |
| VelocityDeadbandCritic | 无 | 有 | 当前缺失 |
| PathAlignLegacyCritic | 有 | 无 | 当前保留旧版 critic |
| `ALGORITHM.md` | 有 | 无 | 当前私有文档 |
| `path_handler.*` | 有 | 有 | 这不是差异点 |

---

## 2. 加速度限制差异详解

> 截至 2026-03-24，当前代码已经补入参数、结构体、`vx/vy` 方向的约束链路，但与上游 `main` / `humble-main` 仍 **不完全等价**。
> 主要缺口是：`az_max` 虽然已加入参数和数据结构，但角速度 `wz` 的角加速度约束在当前实现中仍未完整施加。

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

这一层与上游方向基本一致。

### 2.2 采样轨迹阶段：`MotionModel::predict()`

当前代码在 [`motion_models.hpp`](/home/dex/workspace_pzc/dexe_mobile_nav/mobile_navigation/nav2_mppi_controller/include/nav2_mppi_controller/motion_models.hpp#L70) 已对 `vx` / `vy` 做逐步加速度裁剪，但对 `wz` 没有裁剪：

```cpp
// 暂时取消角加速度约束，直接赋值
auto cw     = xt::view(state.cwz, xt::all(), i - 1);
auto w_next = xt::view(state.wz, xt::all(), i);
xt::noalias(w_next) = cw;
```

而 `humble-main` 在 [`motion_models.hpp`](/home/dex/workspace_pzc/dexe_mobile_nav/navigation2-humble_main/nav2_mppi_controller/include/nav2_mppi_controller/motion_models.hpp#L97) 会先按 `az_max * dt` 对 `cwz` 做逐步裁剪，再写回 `wz`：

```cpp
state.cwz.col(i - 1) = state.cwz.col(i - 1)
  .cwiseMax(state.wz.col(i - 1) - max_delta_wz)
  .cwiseMin(state.wz.col(i - 1) + max_delta_wz);
state.wz.col(i) = state.cwz.col(i - 1);
```

结论：

- 当前候选轨迹在 `vx/vy` 维度上更接近上游
- 当前候选轨迹在 `wz` 维度上仍可能出现不满足角加速度约束的跳变

### 2.3 输出控制序列阶段：`applyControlSequenceConstraints()`

当前代码在 [`optimizer.cpp`](/home/dex/workspace_pzc/dexe_mobile_nav/mobile_navigation/nav2_mppi_controller/src/optimizer.cpp#L250) 中：

```cpp
const float max_delta_vx = s.model_dt * s.constraints.ax_max;
const float min_delta_vx = s.model_dt * s.constraints.ax_min;
// const float max_delta_wz = s.model_dt * s.constraints.az_max;
```

并且在后续只做 `wz` 幅值裁剪：

```cpp
// 暂时取消角加速度约束，只做速度限制
float wz_curr = std::clamp(control_sequence_.wz(i), -s.constraints.wz, s.constraints.wz);
control_sequence_.wz(i) = wz_curr;
```

而 `humble-main` 在 [`optimizer.cpp`](/home/dex/workspace_pzc/dexe_mobile_nav/navigation2-humble_main/nav2_mppi_controller/src/optimizer.cpp#L255) 到 [`optimizer.cpp`](/home/dex/workspace_pzc/dexe_mobile_nav/navigation2-humble_main/nav2_mppi_controller/src/optimizer.cpp#L301) 之间会继续执行：

```cpp
float max_delta_wz = s.model_dt * s.constraints.az_max;
...
wz_curr = utils::clamp(wz_last - max_delta_wz, wz_last + max_delta_wz, wz_curr);
```

结论：

- 当前最终输出序列具备 `vx/vy` 加速度限制
- 当前最终输出序列仅具备 `wz` 幅值限制，不具备完整角加速度限制

### 2.4 `MotionModel::initialize()`

当前代码已经补入：

```cpp
void initialize(const models::ControlConstraints & control_constraints, float model_dt)
{
  control_constraints_ = control_constraints;
  model_dt_ = model_dt;
}
```

并在 [`optimizer.cpp`](/home/dex/workspace_pzc/dexe_mobile_nav/mobile_navigation/nav2_mppi_controller/src/optimizer.cpp#L132) 的 `reset()` 中调用：

```cpp
motion_model_->initialize(settings_.constraints, settings_.model_dt);
```

这一点与上游思路一致。

### 2.5 当前实现的准确结论

如果以 `main` 或 `humble-main` 为基线，当前“新增的加速度约束”更准确的描述应为：

- 已补齐：`ControlConstraints` 字段扩展、参数读取、`MotionModel::initialize()`、`vx` / `vy` 的加速度/减速度限制
- 尚未补齐：`wz` 的角加速度限制（`az_max`）在采样阶段与输出阶段的完整闭环
- 因此不能写成“已与上游一致”，只能写成“xtensor 体系下部分移植”

---

## 3. main 独有而 humble-main 没有的差异

以下内容仍然是当前代码相对 `main` 的真实差异，但 **不是** 当前代码相对 `humble-main` 的差异补充。

### 3.1 SG 滤波调用位置变化

当前代码：

```cpp
geometry_msgs::msg::TwistStamped Optimizer::evalControl(...) {
  prepare(...);
  do { optimize(); } while (fallback(...));

  utils::savitskyGolayFilter(control_sequence_, control_history_, settings_);
  auto control = getControlFromSequenceAsTwist(...);
  return control;
}
```

`main`：

```cpp
void Optimizer::updateControlSequence() {
  // softmax 加权求和
  utils::savitskyGolayFilter(control_sequence_, control_history_, settings_);
  applyControlSequenceConstraints();
}
```

说明：`humble-main` 仍保留旧位置，因此这一项不是 `humble-main` 差异。

### 3.2 open_loop 模式

`main` 新增：

```cpp
getParam(s.open_loop, "open_loop", false);
state_.speed = settings_.open_loop ? last_command_vel_ : robot_speed;
```

说明：`humble-main` 没有该功能。

### 3.3 TrajectoryValidator 插件系统

`main` 新增 `OptimalTrajectoryValidator`、`trajectory_validators.xml` 以及优化后的轨迹验证流程。

说明：`humble-main` 没有该功能。

### 3.4 kinematic_guard / 动态限速保护

`main` 新增：

- `isSpeedLimitActive()`
- 动态参数更新前置拦截 `kinematic_guard`
- `reset(false)`，在 fallback 时保留动态限速状态

说明：`humble-main` 没有这套机制。

---

## 4. 当前代码相对 humble-main 还缺什么

### 4.1 VelocityDeadbandCritic

`humble-main` 和 `main` 都已有：

- `include/nav2_mppi_controller/critics/velocity_deadband_critic.hpp`
- `src/critics/velocity_deadband_critic.cpp`

当前代码没有该 critic，因此这是当前相对 `humble-main` 与 `main` 的共同缺口。

### 4.2 PathAlignLegacyCritic

当前代码仍保留：

- `include/nav2_mppi_controller/critics/path_align_legacy_critic.hpp`
- `src/critics/path_align_legacy_critic.cpp`

`humble-main` 中已经没有这套 legacy critic，因此这是当前代码的私有保留差异。

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
| P0（立即） | 补齐 `wz/az_max` 角加速度约束 | 低（0.5 天） | 先把当前“部分移植”补成与 humble-main 等价 |
| P1（近期） | SG 滤波调用位置（§3.1） | 低（0.5 天） | 与 main 行为对齐，更合理 |
| P1（近期） | VelocityDeadbandCritic（§4.1） | 低（1 天） | 对低速抖动改善直接 |
| P2（中期） | open_loop 模式（§3.2） | 低（1 天） | 对底盘速度反馈延迟场景有效 |
| P2（中期） | kinematic_guard（§3.4） | 低（1 天） | 提升动态限速安全性 |
| P3（长期） | TrajectoryValidator（§3.3） | 高（1-2 周） | 与 main 新框架耦合更深 |
| P3（长期） | 底层库 xtensor → Eigen | 极高（2-3 周） | 架构级改动，建议单独立项 |

---

*文档生成时间：2026-03-20*
*最后核对时间：2026-03-24*
*对应源码分支：`pzc/feature/speedplan`*
