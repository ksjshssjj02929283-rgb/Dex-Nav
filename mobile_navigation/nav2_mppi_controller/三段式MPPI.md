# 三段式 MPPI 参数说明

本文档整理当前 `nav2_mppi_controller` 中三段式 MPPI Profile 的切换规则、各阶段可覆盖参数，以及 `w1_nav2_params.yaml` 的配置现状。

> 📌 本文以当前代码/yaml 实际值为准（已与 `controller.cpp` / `w1_nav2_params.yaml` 对齐）。参数的**单一真相源是 `w1_nav2_params.yaml`**，本文表格用于解释设计意图，调参请改 yaml。

代码位置：

- 控制器三阶段读取与切换：`src/controller.cpp`（`selectProfile` / `configureProfiles` / `interpolateProfiles`）
- Profile 应用到速度约束：`src/optimizer.cpp`
- 路径进度和目标距离计算：`src/path_handler.cpp`
- 当前主要参数文件：`../nav2_bringup/params/w1_nav2_params.yaml`

## 1. 当前状态

三段式 Profile 已**固定启用，单阶段 MPPI 兼容逻辑已删除**（不再有 `profile_enabled` 开关）。profile 由 `controller.cpp::configureProfiles()` 给出代码默认，再由 `w1_nav2_params.yaml` 的 `profile_1/2/3` 覆盖。

当前阶段阈值（yaml 实际值）：

| 参数 | 当前值 | 影响 |
|---|---:|---|
| `align_enter_angle` | `0.52` (~30°) | **车头-路径夹角超此进对齐段 align**（按角度切，非距离）|
| `align_exit_angle` | `0.31` (~18°) | 夹角降至此退出对齐段（迟滞防抖）|
| `profile_goal_distance` | `1.5` | 距终点 ≤ 此进趋近段 approach（优先级最高）|
| `profile_transition_duration` | `1.0` | 阶段切换插值过渡时间(s) |
| `profile_switch_hysteresis` | `0.05` | 趋近段距离迟滞带 |
| `profile_start_distance` | `0.5` | ⚠️ **已废弃**：对齐段改按角度触发，此参数不再参与切换 |

`profile_1/2/3` 完整管理速度约束 + 行为类 critic（Goal、GoalLateral、GoalLongitudinal、GoalAngle、PathAlign、PathFollow、PathAngle、PreferForward）；**全局配置不再作为单阶段行为策略默认值**，三段已差异化（不再"等于全局"）。

## 2. 三阶段切换规则（按语义切：approach > align > track）

每周期从 `PathHandler` 获取：

| 变量 | 含义 |
|---|---|
| `goal_distance` | 机器人到最终目标点直线距离 |
| `heading_path_angle` | 车头朝向与**最近点路径切线**的夹角(rad) |

`selectProfile()` 优先级 **approach(趋近) > align(对齐) > track(跟踪)**（各带迟滞）：

```text
# 趋近段优先（距离）
if 当前在 approach 且 goal_distance <= profile_goal_distance + hysteresis: approach
if goal_distance <= profile_goal_distance: approach
# 对齐段（角度迟滞：进 align_enter_angle / 出 align_exit_angle）
if 当前在 align 且 heading_path_angle > align_exit_angle: align
if heading_path_angle > align_enter_angle: align
# 默认跟踪段
track
```

注意：
- **趋近段优先级最高**：近目标即使车头没对准路径也进 approach（对齐 GOAL 而非路径，GoalAngle 接管）。
- **对齐段按角度**：车头偏离路径 >align_enter(~30°) 即进 align（起点 OR 中途失准都触发），对齐后(<align_exit~18°)回 track。
- profile 数组索引：**0=align, 1=track, 2=approach**（沿用 profile_1/2/3 yaml 段名）。

## 3. 三个阶段的定位

```text
三段式 MPPI（profile 索引 0/1/2 = align/track/approach）
├── align 对齐段（heading_path_angle > align_enter_angle）
│   └── 快速转向对齐路径方向；PathAngle 主导、PathAlign/Follow 降低；保留后退机动避障（DesiredVelocity 关）
│
├── track 跟踪段（已对齐且未近目标，默认）
│   └── 较快速度严格贴线跟踪；PathAlign 主代价；避障/速度软约束全程在场
│
└── approach 趋近段（goal_distance <= profile_goal_distance，优先级最高）
    └── 降速精调到目标位姿；四目标 critic 主导；DesiredVelocity 软减速
```

## 4. Profile 可以覆盖哪些参数

### 4.1 速度约束

| 参数 | 含义 | 作用 |
|---|---|---|
| `vx_max` | 最大前进速度 | 限制前进速度上限 |
| `vx_min` | 最小线速度 | 负值表示允许倒车 |
| `vy_max` | 最大侧向速度 | DiffDrive 下通常不实际使用 |
| `wz_max` | 最大角速度 | 限制旋转速度上限 |

这些参数覆盖 `Optimizer` 的 `base_constraints`，再叠加速度限制层的 `speed_limit`。

### 4.2 Critic 参数（按阶段覆盖）

| Critic | 可覆盖参数 |
|---|---|
| `GoalCritic` | `cost_weight`, `threshold_to_consider` |
| `GoalLateralCritic` | `cost_weight`, `threshold_to_consider`, `last_n_points` |
| `GoalLongitudinalCritic` | `cost_weight`, `threshold_to_consider`, `last_n_points` |
| `GoalAngleCritic` | `cost_weight`, `threshold_to_consider` |
| `PathAngleCritic` | `cost_weight`, `threshold_to_consider`, `offset_from_furthest`, `max_angle_to_furthest`, `forward_preference` |
| `PathFollowCritic` | `cost_weight`, `threshold_to_consider`, `offset_from_furthest` |
| `PathAlignCritic` | `cost_weight`, `threshold_to_consider`, `offset_from_furthest`, `trajectory_point_step`, `max_path_occupancy_ratio` |
| `PreferForwardCritic` | `cost_weight`, `threshold_to_consider` |

> `GoalLateralCritic` / `GoalLongitudinalCritic` 分别惩罚候选轨迹点在**目标点坐标系**下的横向 / 纵向投影误差，坐标轴由目标终点朝向固定，不随候选轨迹自身朝向变化，是末段厘米级停靠的核心。

**不支持按阶段覆盖**（仍用 `FollowPath` 全局配置）：

| 类型 | 说明 |
|---|---|
| 采样参数 | `vx_std`, `vy_std`, `wz_std`, `batch_size`, `time_steps`, `model_dt` |
| 约束参数 | `ax_max`, `ax_min`, `az_max`, `jx_max`, `jz_max` |
| 障碍物 critic | `ObstaclesCritic`, `CostCritic`（含近目标膨胀过滤，见 §5）|
| 死区 critic | `VelocityDeadbandCritic` |

### 4.3 `threshold_to_consider` 的双重语义（重要，易踩坑）

每个 critic 都有一个 `threshold_to_consider`，它**不是评分项开关，而是该 critic 的「作用距离带」**（离目标多远时才有意义）。它配合 `withinPositionGoalTolerance(threshold)` 用，但 **Goal 类与 Path 类极性相反**：

| critic 类型 | score() 早退条件 | 效果 |
|---|---|---|
| **Goal 类**（GoalCritic / GoalAngle / GoalLateral / GoalLongitudinal）| `if (!within(thr)) return;` | **进 threshold 内才启用**（近目标才吸向终点 / 收朝向）|
| **Path 类**（PathAlign / PathFollow / PathAngle / PreferForward）| `if (within(thr)) return;` | **进 threshold 内就关闭**（近目标停止贴线，交棒给 Goal 类）|

记法：Goal 类"近了才上场"、Path 类"近了就下场"。一正一反，构成"远处跟线 → 近处吸向终点"的平滑交棒。各 critic 的 threshold 不同（如 GoalAngle 0.2m 才收最终朝向、PathAlign 0.5m 外才贴线），是必要的精细分工，不可合并成一个全局值。

**与三段式 profile 的分工（避免重复调度）**：profile 切换会连 `threshold_to_consider` 一起覆盖。两者都按到目标距离调 critic，职责应分层——

| 机制 | 只负责 |
|---|---|
| profile（weight 0/非0）| **段间调度**：哪个阶段启用哪些 critic |
| `threshold_to_consider` | **段内精细调度**：某阶段内部的距离 sub-band（如末段里 GoalAngle 0.2m 收朝向、GoalCritic 1.0m 起吸）|

实用结论：调"某 critic 在哪段开/关"改 profile 的 weight；调"某 critic 在该段内多近才起作用"改 threshold。前段里 weight=0 的 critic，其 threshold 设什么都无所谓。

## 5. 当前全局 MPPI 参数基线

`w1_nav2_params.yaml` 的 `FollowPath` 全局基线（profile 不覆盖的项以此为准）：

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `time_steps` | `40` | 预测步数 |
| `model_dt` | `0.1` | 每步时间，预测时域约 4.0 s（= time_steps × model_dt）|
| `batch_size` | `1500` | 采样轨迹数量 |
| `vx_std` / `vy_std` / `wz_std` | `0.25` / `0.2` / `0.3` | 采样标准差 |
| `vx_max` / `vx_min` | `0.7` / `-0.5` | 全局速度（运行时被 profile 覆盖）|
| `vy_max` / `wz_max` | `0.5` / `0.75` | 全局侧向/角速度上限 |
| `ax_max` / `ax_min` / `az_max` | `0.5` / `-0.5` / `0.7` | 加速度约束 |
| `jx_max` / `jz_max` | `1.0` / `1.5` | jerk 约束 |
| `motion_model` | `DiffDrive` | 差速模型 |
| `open_loop` | `true` | 用上一周期输出作为速度状态 |

### 5.1 近目标膨胀过滤（贴障到点）

属 controller 本体参数（不属单个 critic），让末段贴近障碍的目标点也能稳定到达：

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `near_goal_cost_relaxation_distance` | `0.5` | 距目标小于此值时，MPPI 障碍/代价 critic 启用近目标软膨胀过滤 |
| `near_goal_effective_inflation_radius` | `0.2` | MPPI 内部等效膨胀半径（**不修改** local costmap 真实 0.4m inflation）|

机制：近目标时把非致命的软膨胀 cost（`INSCRIBED_INFLATED` 及等效半径内）在 MPPI 评分里置为 FREE，**致命/未知障碍仍保留**，相当于末段把 MPPI 眼里的膨胀半径缩到 0.2m，让车敢贴障到点。

### 5.2 速度软约束 DesiredVelocityCritic（平滑减速，消除阶段硬降速）

全局 critic（不分阶段覆盖），按每个预测点到目标剩余距离算期望速度做**下降式软速度上限**：

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `cost_weight` | `8.0` | 软约束权重（bag 调）|
| `a_decel` | `0.3` | 期望减速度 m/s²，`v_desired=min(vx_max, sqrt(2·a·d))`；速度上限取全局 vx_max(0.7)|
| `cost_power` | `2` | |

机制：双边惩罚"当前(候选)速度与 v_desired 的差值"|vx − v_desired|（速度跟踪），使实际速度趋近期望速度——远离目标 v_desired=vx_max，接近目标平滑降到 0，MPPI 在时域内自行规划匀减速。配合 profile_3 vx_max 改回 0.7（删硬降）+ 末段禁倒车(vx_min0) + v_desired→0 速度阻尼，解决"二三阶段硬降速顿挫"与"到点前后磨蹭"。

## 6. 当前三阶段参数（实际值对照）

> ⚠️ 2026-06 起阶段按语义切（align/track/approach，见 §2），且团队在 yaml 持续调参 —— **本表数值仅示意，权威以 `w1_nav2_params.yaml` 为准**。对齐段(profile_1)已按 req-1 调为 PathAngle 18 / PathAlign 5 / PathFollow 1 / wz 0.9 / max_angle 0.3。

> 三段已差异化，下表为 yaml 实际值。`—` 表示该 critic 在该段权重为 0（关闭）。

| 项 | profile_1 起步 | profile_2 中间 | profile_3 末段 |
|---|:---:|:---:|:---:|
| `vx_max` / `vx_min` | 0.4 / −0.3 | 0.7 / −0.5 | 0.7 / 0 |
| `wz_max` | 0.75 | 0.75 | 0.30 |
| `GoalCritic` (weight, thr) | — | — | 4.0, 1.0 |
| `GoalLateralCritic` (weight, thr, n) | — | — | 35.0, 1.0, 10 |
| `GoalLongitudinalCritic` (weight, thr, n) | — | — | 25.0, 0.8, 10 |
| `GoalAngleCritic` (weight, thr) | — | — | 15.0, 0.2 |
| `PathAngleCritic` (weight, off, thr, maxang) | 7.0, 3, 0.3, 0.7 | 5.0, 3, 0.5, 0.9 | 1.0, 2, 0.5, 1.0 |
| `PathAlignCritic` (weight, off, step, thr) | 15.0, 3, 3, 0.3 | 20.0, 15, 2, 0.5 | 2.0, 15, 3, 0.5 |
| `PathFollowCritic` (weight, off, thr) | 7.0, 7, 1.2 | 4.0, 5, 0.5 | 1.0, 5, 0.5 |
| `PreferForwardCritic` (weight, thr) | 3.0, 0.25 | 5.0, 0.25 | 1.0, 1.0 |

设计意图：

| 阶段 | 意图 |
|---|---|
| **起步段** | 降 vx(0.4) 保 wz(0.75)；**PathAngle 7.0 + 早触发(max0.7) 主导快速对齐路径方向**，PathAlign 15 适度贴线；关闭全部目标类 critic（精调交给末段）|
| **中间段** | 全速(0.7)；**PathAlign 20 为主代价**严格贴线，PathFollow 4 弱推进减少切弯；关闭目标类 critic |
| **末段** | **vx 硬上限保持 0.7**（减速交给 DesiredVelocityCritic 软约束按距离平滑降速，消除二三阶段硬降速）、wz0.30、禁倒车(vx_min0)；**四目标 critic 主导**（横向 35 / 纵向 25 / 朝向 15 / 弱距离 4），路径类弱牵引(1~2) |

> ⚠️ 末段路径类 critic 是**弱值(1~2) 而非 0**：进入末段时若路径类直接归零会丢失路径牵引、导致二三阶段边界先减速再加速的顿挫。过渡插值上，速度上限线性过渡、目标类 critic 慢启动（详见 `interpolateProfiles`）。

## 6.5 ⭐ yaml 结构（2026-06 重构）

阶段配置已重构为更清晰的结构（DEC-009）：
- **块名语义化**：`profile_1/2/3` → `stage_align` / `stage_track` / `stage_approach`。
- **权重矩阵**：各 critic 三段权重集中在 `stage_weights:`（`CriticName: [align, track, approach]`），一行看全三段、调参最直观；非权重 sub-param(offset/threshold/last_n 等)留在 `stage_*` 块。
- **切换参数**：`approach_distance`(原 profile_goal_distance)、`align_enter_angle`/`align_exit_angle`；`profile_start_distance` 已删（对齐段改按角度切）。
- 代码侧：`enum Stage{STAGE_ALIGN/TRACK/APPROACH}` 消 magic index，`selectStage`/`stageName`/`configureStages`/`stages_`/`active_stage_` 语义命名。

## 7. 配置位置

三段式全部参数位于 `controller_server.ros__parameters.FollowPath` 下，与 `vx_max/vx_min/vy_max/wz_max` 同级。**完整最新配置以 `w1_nav2_params.yaml` 为准**（本文不再重复全量片段，避免两处漂移）。切换参数片段：

```yaml
profile_start_distance: 0.5
profile_goal_distance: 1.5
profile_transition_duration: 1.0
profile_switch_hysteresis: 0.05
# profile_1 / profile_2 / profile_3 各段参数见 yaml（值见 §6 对照表）
```

## 8. 调试日志怎么看

三段式相关日志重点字段：

```text
MPPI profiles: start_distance=0.500 goal_distance=1.500 transition_duration=1.000 switch_hysteresis=0.050
MPPI profile_0/1/2 velocity: vx=[..., ...] vy=... wz=...
MPPI profile state: active=... selected=... path_progress=... goal_distance=...
MPPI profile switched: 0 -> 1, reason=track/跟踪段, ... heading_angle=...
MPPI active constraints: vx=[..., ...] vy=... wz=...
```

| 日志字段 | 用途 |
|---|---|
| `path_progress` | 判断是否应离开起步段 |
| `goal_distance` | 判断是否应进入末段 |
| `active/selected` | 当前阶段与即将选择阶段是否一致 |
| `profile switched` | 确认阶段确实切换 + 原因 |
| `active constraints` | 确认 profile 速度约束已应用到 optimizer |

不再有 `enabled` 字段（三段式已固定）。若阶段未按预期切换，优先检查 `profile_start_distance`、`profile_goal_distance`、`profile_switch_hysteresis` 与实时 `path_progress`/`goal_distance`。

## 9. 风险和注意事项

- 三段式切换只改变速度约束和部分 critic 参数，**不重置 optimizer，也不重建 critic 插件**。
- 三段已差异化，调参时注意三段是一个整体：改末端停靠能力须同时考虑 MPPI 末段 critic + 速度限制 + `goal_checker`（单改 goal_checker 只延长精调时间，不改优化代价）。
- `profile_3` 的 `vx_max/wz_max` 已降到 0.2/0.30，终点更稳但到位时间可能变长；配合 `goal_checker.enable_high_precision` 开关平衡精度与到点速度。
- 近目标膨胀过滤只影响 MPPI 评分，不改真实 costmap；动态/致命障碍安全由 costmap 致命层 + `collision_monitor` 独立兜底。
- 若使用速度限制层，最终运行速度约束 = 当前 profile 速度约束 × speed limit 比例（或按绝对限速换算）。
