# Nav2 MPPI 控制器算法文档

> 本文档基于本仓库的源码（`nav2_mppi_controller/`）整理，结合项目实际修改情况。

---

## 目录

1. [算法概述](#1-算法概述)
2. [整体计算流程](#2-整体计算流程)
3. [核心数学推导](#3-核心数学推导)
4. [数据结构](#4-数据结构)
5. [运动模型](#5-运动模型)
6. [评估器系统（Critics）](#6-评估器系统critics)
7. [噪声生成](#7-噪声生成)
8. [关键参数说明](#8-关键参数说明)
9. [加速度限制分析](#9-加速度限制分析)
10. [速度规划集成建议](#10-速度规划集成建议)

---

## 1. 算法概述

**MPPI（Model Predictive Path Integral）** 是一种基于随机采样的模型预测控制（MPC）算法，适用于非线性系统和复杂约束场景。

### 核心思想

在每个控制周期内：

1. 以当前控制序列为均值，通过高斯噪声扰动生成 `batch_size` 条候选轨迹
2. 对每条轨迹用多个"评估器（Critic）"计算累积代价
3. 用**指数加权平均（软max）**更新最优控制序列
4. 取控制序列的第一个控制量输出，同时预留后续步骤热启动下一周期

### 相比传统 MPC 的优势

| 方面 | 传统 MPC | MPPI |
|------|---------|------|
| 约束处理 | 需要求解带约束的优化问题 | 通过代价函数软约束，无需凸优化 |
| 障碍物避免 | 线性化/近似处理 | 可直接使用代价地图，支持复杂场景 |
| 计算方式 | 串行迭代 | 可并行（批量矩阵运算，Eigen Array 加速） |
| 实时性 | 依赖 QP solver | 固定计算时间，更易保证实时性 |

---

## 2. 整体计算流程

```
每个控制周期：computeVelocityCommands()
│
├─ path_handler_.transformPath()      将全局路径变换到机器人坐标系
│
├─ optimizer_.evalControl()           MPPI 主体
│   │
│   ├─ prepare()                      初始化状态
│   │   ├─ state_.pose  ← robot_pose
│   │   ├─ state_.speed ← robot_speed
│   │   └─ path_ ← transformed_plan（转为 Eigen-backed Path）
│   │
│   ├─ do { optimize() } while (fallback)
│   │   │
│   │   └─ optimize()                 MPPI 迭代（iteration_count 次）
│   │       │
│   │       ├─ generateNoisedTrajectories()
│   │       │   ├─ setNoisedControls()
│   │       │   │   cvx[i,t] = control_seq.vx[t] + noise_vx[i,t]
│   │       │   │   cwz[i,t] = control_seq.wz[t] + noise_wz[i,t]
│   │       │   │   （clip 到速度范围后存入 state_.cvx/cwz）
│   │       │   │
│   │       │   ├─ generateNextNoises()   后台线程预生成下次噪声
│   │       │   │
│   │       │   ├─ updateStateVelocities()
│   │       │   │   state_.vx[:,0] = robot_speed.linear.x   （真实初速）
│   │       │   │   state_.vx[:,1:] = state_.cvx[:,0:-1]   （后续步 = 受约束控制）
│   │       │   │
│   │       │   └─ integrateStateVelocities()   速度前向积分 → 轨迹位姿
│   │       │       yaw[t]  = Σ(wz * dt) + yaw0
│   │       │       x[t]    = x0 + Σ(vx*cos(yaw) * dt)
│   │       │       y[t]    = y0 + Σ(vx*sin(yaw) * dt)
│   │       │
│   │       ├─ evalTrajectoriesScores()   各 Critic 评分，累加到 costs_
│   │       │
│   │       └─ updateControlSequence()    软max 加权更新
│   │           costs_ += gamma/σ² * Σ(u * δu)   （控制正则项）
│   │           w[i] = exp(-costs_[i] / temperature)
│   │           w = w / Σ(w)                       （归一化）
│   │           vx_new[t] = Σ(w[i] * cvx[i,t])
│   │           applyControlSequenceConstraints()  （硬速度限幅）
│   │
│   ├─ savitskyGolayFilter()           9点 Savitzky-Golay 平滑控制序列
│   │
│   ├─ getControlFromSequenceAsTwist() 取第[0]或第[1]步控制量输出
│   │
│   └─ shiftControlSequence()         （可选）控制序列前移，为下周期热启动
│
└─ return cmd (TwistStamped)
```

---

## 3. 核心数学推导

### 3.1 MPPI 目标函数

最小化期望代价：

```
J(u) = E[ φ(x_T) + Σ_{t=0}^{T-1} [ q(x_t) + (γ/2σ²) u_t^T u_t ] ]
```

其中：
- `φ(x_T)`：终端代价（GoalCritic 等）
- `q(x_t)`：运行代价（ObstaclesCritic、PathFollowCritic 等）
- `(γ/2σ²) u_t^T u_t`：控制平滑正则项（抑制抖动）

### 3.2 信息论最优更新律

对扰动控制 `ε ~ N(0, Σ)`，最优控制更新为：

```
u*_t = Σ_i [ w(τ_i) * (u_t + ε_{i,t}) ]

w(τ_i) = exp(-1/λ * S(τ_i)) / Σ_j exp(-1/λ * S(τ_j))

S(τ_i) = Σ_t [ q(x_t^i) + (γ/σ²) u_t * ε_{i,t} ]
```

其中 `λ` 即 `temperature` 参数。

**源码对应** — [optimizer.cpp:354-386](src/optimizer.cpp#L354)：

```cpp
// 控制正则项（γ 项）
for (Eigen::Index i = 0; i < costs_.size(); ++i) {
  costs_(i) += s.gamma / powf(s.sampling_std.vx, 2) *
    (control_sequence_.vx * bounded_noises_vx.row(i).transpose()).sum();
}

// 软max 权重
const auto costs_normalized = costs_ - costs_.minCoeff();  // 数值稳定
const auto exponents = (-1.0f / settings_.temperature * costs_normalized).exp();
const auto softmaxes = exponents / exponents.sum();

// 加权平均
control_sequence_.vx =
  (state_.cvx.colwise() * softmaxes).colwise().sum().transpose();
```

### 3.3 Savitzky-Golay 平滑滤波

对最优控制序列应用 **9点2阶 SG 滤波**，系数为：

```
h = [-21, 14, 39, 54, 59, 54, 39, 14, -21] / 231
```

**源码位置** — [tools/utils.hpp:451](include/nav2_mppi_controller/tools/utils.hpp#L451)

**效果**：相邻时间步控制量平滑过渡，避免加速度突变。

---

## 4. 数据结构

### 4.1 State — 批量状态矩阵

```
文件：include/nav2_mppi_controller/models/state.hpp
```

```
state.vx   [batch_size × time_steps]  实际线速度（积分用）
state.cvx  [batch_size × time_steps]  受约束的扰动线速度（含噪声）
state.wz   [batch_size × time_steps]  实际角速度
state.cwz  [batch_size × time_steps]  受约束的扰动角速度
state.pose  geometry_msgs::PoseStamped    当前机器人位姿
state.speed geometry_msgs::Twist          当前机器人速度
```

### 4.2 ControlSequence — 最优控制序列

```
文件：include/nav2_mppi_controller/models/control_sequence.hpp
```

```
control_sequence_.vx  [time_steps]  最优线速度序列
control_sequence_.wz  [time_steps]  最优角速度序列
control_sequence_.vy  [time_steps]  最优侧向速度（仅全向）
```

### 4.3 ControlConstraints — 约束

```
文件：include/nav2_mppi_controller/models/constraints.hpp
```

```cpp
struct ControlConstraints {
  float vx_max;   // 最大前进速度
  float vx_min;   // 最小速度（后退为负）
  float vy;       // 最大侧向速度（全向机）
  float wz;       // 最大角速度
  // 注意：无加速度字段
};
```

### 4.4 OptimizerSettings — 算法参数

```
文件：include/nav2_mppi_controller/models/optimizer_settings.hpp
```

```cpp
struct OptimizerSettings {
  ControlConstraints base_constraints;  // 配置文件中的原始约束
  ControlConstraints constraints;       // 运行时约束（setSpeedLimit 可动态修改）
  SamplingStd sampling_std;             // 采样标准差 {vx, vy, wz}
  float model_dt;
  float temperature;
  float gamma;
  unsigned int batch_size;
  unsigned int time_steps;
  unsigned int iteration_count;
  bool shift_control_sequence;
  size_t retry_attempt_limit;
};
```

---

## 5. 运动模型

```
文件：include/nav2_mppi_controller/motion_models.hpp
```

### 5.1 DiffDriveMotionModel（差速驱动，默认）

- 非全向，不使用 `vy`
- 无额外约束，直接使用速度限幅

### 5.2 OmniMotionModel（全向驱动）

- 全向，启用 `vy` 轴
- 允许侧向移动

### 5.3 AckermannMotionModel（阿克曼转向）

- 非全向
- 额外约束：最小转弯半径 `min_turning_r`
  ```cpp
  // 当 |vx|/|wz| < min_turning_r 时限制 wz
  wz = sign(wz) * vx / min_turning_r
  ```

---

## 6. 评估器系统（Critics）

每个 Critic 是一个插件，实现 `CriticFunction` 接口，对 `batch_size` 条轨迹的 `costs_` 张量累加代价。

### 6.1 内置 Critics 列表

| Critic | 功能 | 关键参数 |
|--------|------|---------|
| `ConstraintCritic` | 惩罚速度超限轨迹（软约束） | `cost_weight: 4.0` |
| `ObstaclesCritic` | 避障代价，支持足迹碰撞检测 | `critical_weight: 20.0`, `collision_cost: 10000` |
| `GoalCritic` | 引导靠近目标点 | `cost_weight: 5.0` |
| `GoalAngleCritic` | 靠近目标时对齐目标朝向 | — |
| `PathAlignCritic` | 与参考路径整体对齐 | — |
| `PathFollowCritic` | 跟踪路径前方的锚点 | `offset_from_furthest: 6` |
| `PathAngleCritic` | 与路径切线方向对齐 | — |
| `CostCritic` | 代价地图成本惩罚 | — |
| `PreferForwardCritic` | 偏好前进方向行驶 | — |
| `TwirlingCritic` | 抑制原地旋转 | — |

### 6.2 Critic 调用链

```
CriticManager::evalTrajectoriesScores(CriticData & data)
  for each critic:
    critic->score(data)   // 直接在 data.costs 上累加
```

### 6.3 ConstraintCritic 详解

对速度超出范围的轨迹施加惩罚：

```cpp
vel_total = sqrt(vx² + vy²)
out_of_max = max(vel_total - max_vel, 0)
out_of_min = max(min_vel - vel_total, 0)
costs += ((out_of_max + out_of_min) * dt * weight) ^ power
```

> **注意**：这是软约束，不能保证完全消除超速轨迹，需配合 `applyControlSequenceConstraints()` 的硬限幅使用。

---

## 7. 噪声生成

```
文件：src/noise_generator.cpp
      include/nav2_mppi_controller/tools/noise_generator.hpp
```

### 7.1 噪声分布

```
noise_vx[i,t] ~ N(0, vx_std²)
noise_wz[i,t] ~ N(0, wz_std²)
noise_vy[i,t] ~ N(0, vy_std²)   （仅全向）
```

### 7.2 异步生成机制

噪声生成在**独立线程**中运行，与主控制循环并行：

```
主线程：使用上一次生成的噪声计算轨迹
子线程：同时生成下一次迭代所需的噪声
```

当 `regenerate_noises: false`（默认）时，在一次 optimize() 内复用同一批噪声，避免重复生成开销。

---

## 8. 关键参数说明

配置位置：`nav2_bringup/params/w1_nav2_params.yaml` → `MPPIController` 节。

### 8.1 算法核心参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `batch_size` | 1000 | 采样轨迹数。越大效果越好，计算越慢 |
| `time_steps` | 56 | 预测时域（步数）。总预测时长 = time_steps × model_dt |
| `model_dt` | 0.05 s | 时间步长，**必须等于控制器频率的倒数** |
| `iteration_count` | 1 | MPPI 迭代次数。通常 1 次即可 |
| `temperature` | 0.3 | 软max 温度。越小越贪婪（选最优），越大越均匀 |
| `gamma` | 0.015 | 控制平滑性权重。越大越倾向平滑控制 |

### 8.2 速度约束参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `vx_max` | 0.5 m/s | 最大前进速度 |
| `vx_min` | -0.35 m/s | 最大后退速度 |
| `wz_max` | 1.9 rad/s | 最大角速度 |
| `vy_max` | 0.5 m/s | 最大侧向速度（全向） |

### 8.3 采样参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `vx_std` | 0.2 | 线速度采样标准差（控制探索范围） |
| `wz_std` | 0.4 | 角速度采样标准差 |
| `vy_std` | 0.2 | 侧向速度采样标准差 |

### 8.4 `setSpeedLimit` 动态接口

`MPPIController::setSpeedLimit(speed_limit, percentage)` 由 Nav2 速度滤波器（如代价地图速度限制层）调用，动态缩放 `constraints`（运行时约束），不修改 `base_constraints`（配置值）。

---

## 9. 加速度限制分析

### 9.1 结论

**MPPI 没有显式的加速度约束参数**（无 `acc_lim_x`、`acc_lim_theta` 类参数），通过以下三个机制间接限制加速度：

### 9.2 机制一：硬速度限幅（无加速度约束）

[optimizer.cpp:229-241](src/optimizer.cpp#L229) — `applyControlSequenceConstraints()`：

```cpp
// 裁剪速度范围，并交给运动模型处理平台特定约束
control_sequence_.vx = vx.min(vx_max).max(vx_min);
control_sequence_.wz = wz.min(wz_max).max(-wz_max);
motion_model_->applyConstraints(control_sequence_);
```

### 9.3 机制二：ConstraintCritic 软约束

软约束只能**引导采样趋向可行域**，无法精确限制加速度幅值。

### 9.4 机制三：Savitzky-Golay 滤波（最主要的加速度抑制）

[tools/utils.hpp:451-614](include/nav2_mppi_controller/tools/utils.hpp#L451) — 在最终输出前对控制序列做9点平滑：

```
滤波系数：h = [-21, 14, 39, 54, 59, 54, 39, 14, -21] / 231
```

- **作用**：平滑相邻时间步的速度变化，从而间接限制 `Δv/Δt`（加速度）
- **局限**：平滑强度固定，无法指定具体的加速度上限 `a_max`

### 9.5 参数调整建议（在无显式加速度限制时缓解抖动）

| 目标 | 调整方式 |
|------|---------|
| 减小线加速度抖动 | 调小 `vx_std`（如 0.1），减少探索范围 |
| 减小角加速度抖动 | 调小 `wz_std`（如 0.2） |
| 整体控制更平滑 | 调大 `gamma`（如 0.05） |
| 更保守地选择轨迹 | 调小 `temperature`（如 0.1） |
| 根本解决方案 | 在 `mobile_control_node` 底层加速度梯形限幅 |

---

## 10. 速度规划集成建议

当需要在 MPPI 控制器前增加**速度规划（速度剖面/加速度限制）**时，有三种方案，推荐程度从高到低：

### 方案 A：在 `controller.cpp` 输出端加速度限幅（推荐）

**修改位置**：[controller.cpp:100-113](src/controller.cpp#L100)

这是**侵入性最小、最稳妥**的方案：在 MPPI 计算出最优指令后、返回前，对输出速度做梯形加速限幅。

**修改思路**：

```cpp
// controller.hpp 中增加成员：
geometry_msgs::msg::Twist last_cmd_vel_;
double max_accel_vx_;   // 通过 declare_parameter 声明
double max_accel_wz_;

// computeVelocityCommands() 末尾，return 前插入：
double dt = 1.0 / controller_frequency_;
double dvx = cmd.twist.linear.x - last_cmd_vel_.linear.x;
double dwz = cmd.twist.angular.z - last_cmd_vel_.angular.z;

// 梯形限幅
dvx = std::clamp(dvx, -max_accel_vx_ * dt, max_accel_vx_ * dt);
dwz = std::clamp(dwz, -max_accel_wz_ * dt, max_accel_wz_ * dt);

cmd.twist.linear.x  = last_cmd_vel_.linear.x  + dvx;
cmd.twist.angular.z = last_cmd_vel_.angular.z + dwz;
last_cmd_vel_ = cmd.twist;
```

**优点**：
- 精确控制加速度上限
- 不影响 MPPI 内部采样（保留探索多样性）
- 代码集中，易于维护和测试

**缺点**：
- MPPI 内部采样时不感知加速度约束，可能生成实际无法执行的轨迹计划

**新增参数（`w1_nav2_params.yaml`）**：

```yaml
MPPIController:
  max_accel_vx: 1.0    # m/s²
  max_accel_wz: 2.0    # rad/s²
```

---

### 方案 B：新增 `AccelerationCritic`（MPPI 原生方式）

**新建文件**：
- `include/nav2_mppi_controller/critics/acceleration_critic.hpp`
- `src/critics/acceleration_critic.cpp`

在 Critic 中对每条候选轨迹计算相邻时间步的速度差，对加速度过大的轨迹施加惩罚：

```cpp
// 计算轨迹上每步的加速度
const auto accel_vx =
  (state.cvx.rightCols(state.cvx.cols() - 1) -
   state.cvx.leftCols(state.cvx.cols() - 1)) / settings_.model_dt;
const auto accel_wz =
  (state.cwz.rightCols(state.cwz.cols() - 1) -
   state.cwz.leftCols(state.cwz.cols() - 1)) / settings_.model_dt;

// 超出限制的部分作为代价
const auto excess_vx = (accel_vx.abs() - max_accel_vx_).max(0.0f);
const auto excess_wz = (accel_wz.abs() - max_accel_wz_).max(0.0f);

data.costs += cost_weight_ * (excess_vx + excess_wz).rowwise().sum();
```

**优点**：加速度约束融入 MPPI 采样过程，生成的轨迹计划本身即满足加速度约束

**缺点**：软约束，无法保证硬性边界；需注册到 `critics.xml` 并在参数文件中启用

**注册方式**（`critics.xml`）：

```xml
<class name="mppi::critics::AccelerationCritic"
       type="mppi::critics::AccelerationCritic"
       base_class_type="mppi::critics::CriticFunction">
  <description>Penalizes trajectories with large acceleration.</description>
</class>
```

---

### 方案 C：修改 `motion_models.hpp` 中的 `predict()` 函数（侵入性强，不推荐）

在 `predict()` 中对速度传播加入加速度裁剪：

```cpp
// 在基类 predict() 中，从第 t 步推算第 t+1 步速度时，限制变化率
void predict(models::State & state) override {
  for (int t = 1; t < time_steps; ++t) {
    auto dvx = state.cvx(:, t) - state.vx(:, t-1);
    state.vx(:, t) = state.vx(:, t-1) + clamp(dvx, -max_accel*dt, max_accel*dt);
  }
}
```

**缺点**：修改了所有运动模型的行为，影响面大；且 `predict()` 处理的是批量矩阵，直接裁剪会破坏采样多样性

---

### 方案对比总结

| 方案 | 是否硬约束 | 实现难度 | 推荐场景 |
|------|----------|---------|---------|
| **A：输出端梯形限幅** | **是** | 低 | 通用，快速上线 |
| B：AccelerationCritic | 否（软） | 中 | 希望轨迹计划本身平滑 |
| C：predict() 裁剪 | 是（近似） | 高 | 不推荐 |

**对于本项目（DiffDrive，AGX 导航精度调优场景），推荐方案 A**，修改 [controller.cpp:100](src/controller.cpp#L100) 附近，增加两个参数 `max_accel_vx` / `max_accel_wz`，在 `computeVelocityCommands()` 的 `return cmd` 前插入梯形限幅逻辑。

---

## 11. 速度规划集成设计（人形机器人场景）

### 11.1 背景与目标

**核心问题**：人形机器人上半身抖动及到位精度不足，根源在于：

| 问题 | 根本原因 | MPPI 现状 |
|------|---------|----------|
| 加减速抖动 | jerk（加加速度）过大 | 仅 SG 滤波，无硬约束 |
| 停车抖动 | 接近终点时采样振荡 | GoalCritic 权重不足时易振荡 |
| 到位精度差 | 速度不精确降为 0，超调 | 无精确的减速段规划 |

**目标**：在 MPPI 框架内，通过速度规划 + jerk 约束，实现：
1. 加减速过程满足 jerk 约束（上身不抖）
2. 终点精确停车（`v_goal = 0`，无超调）
3. 不引入控制不连续性（避免切换控制器）

---

### 11.2 设计评审：初版方案的关键缺陷

以下问题在实现前必须解决：

#### 缺陷 1：JerkLimiter 放在输出端会破坏 MPPI 最优控制

**问题描述**：将 JerkLimiter 放在 `controller.cpp` 输出端（`return cmd` 前），会产生：

```
MPPI 计算 → 输出 v_mppi → JerkLimiter 截断 → 实际输出 v_actual ≠ v_mppi
                                  ↓
             下周期：robot_speed = v_actual（真实状态）
                     control_history_ = v_mppi（MPPI记录的历史）
                     → 两者不一致 → SG 滤波边界错误 → 新的速度跳变
```

**本质**：输出端后处理切断了 MPPI 的**状态反馈一致性**，等效于在 MPC 中每周期引入一个未建模扰动，下周期的优化基于错误的初始状态展开。

**正确做法**：把 jerk 约束嵌入 MPPI **内部**（见 11.4 节方案 B）。

#### 缺陷 2：SpeedProfileCritic 的时间-弧长对齐错误

**问题描述**：文档初版的 Critic 实现：

```cpp
// 错误：直接用时间步对齐
const auto vx_diff = state_.cvx.rowwise() - ref_velocities.transpose();
// state_.cvx  : [batch_size × time_steps]  ← 每条轨迹在时间 t 的控制速度
// ref_velocities : [time_steps]            ← 速度剖面用时间 t 索引
```

这等价于假设所有 1000 条候选轨迹在同一时刻都在路径上的同一位置——但 MPPI 的候选轨迹有不同扰动，快轨迹在 t 时刻走得远，慢轨迹走得近，参考速度理应不同。

**正确做法**：速度剖面以**弧长 s** 为索引（`v(s)` 而非 `v(t)`），Critic 中先计算每条轨迹每步的弧长，再插值出对应参考速度：

```
s[i,t] = Σ sqrt(Δx²+Δy²)  ← 各轨迹各步实际弧长
v_ref[i,t] = interp(profile_s, profile_v, s[i,t])  ← 弧长插值
cost[i] += sum((cvx[i,t] - v_ref[i,t])²)
```

#### 缺陷 3：`s_goal` 使用全局路径长度导致速度剖面无效

`path_length`（全局路径总长）可能为 10m，但 MPPI 预测时域仅 `56 × 0.05 = 2.8s`，以 0.5m/s 最多预测 1.4m。若速度规划范围超出预测时域，减速段完全在时域外，速度剖面在时域内始终为巡航速度，规划失效。

**正确做法**：`s_goal` 取预测时域内路径截断弧长：

```
s_horizon = v_max × time_steps × model_dt × 1.2   // 留 20% 余量
s_goal = min(distance_to_goal, s_horizon)
```

当 `distance_to_goal < s_horizon` 时，规划自然包含完整减速段。

#### 缺陷 4：初始加速度 `a0 = 0` 假设导致速度剖面跳变

每帧都以 `a0=0` 启动 QP，而实际机器人可能正在加速，导致相邻帧速度剖面之间存在跳变（新剖面从零加速度出发，与真实状态不一致），实际上又引入了 jerk。

**正确做法**：在 `controller.hpp` 维护加速度估计，用低通滤波平滑：

```cpp
double estimated_accel_ = 0.0;
// 在 computeVelocityCommands() 中更新：
double raw_accel = (robot_speed.linear.x - prev_speed_) / dt;
estimated_accel_ = 0.7 * estimated_accel_ + 0.3 * raw_accel;  // 低通
```

#### 缺陷 5：SpeedProfileCritic + JerkLimiter 双层约束互相干扰

若同时使用 SpeedProfileCritic（软约束引导 MPPI）+ JerkLimiter（硬截断输出），JerkLimiter 频繁修改 MPPI 输出时，MPPI 的采样优化意义降低，退化为 JerkLimiter 驱动的开环控制。

**正确做法**：二选一。推荐将 jerk 约束内嵌为 MPPI 的运动模型约束（方案 B）。

#### 缺陷 6：角速度 wz 缺少速度规划

初版方案的 QP 只针对线速度（弧长 s），但差速机器人转弯时角速度突变同样引起侧向抖动。

**注意**：`wz` 不需要单独 QP。当路径曲率 `κ(s)` 已知，参考角速度可直接推导：

```
wz_ref(t) = vx_ref(t) × κ(s(t))
```

曲率由路径离散点数值微分获得，`wz_ref` 随 `vx_ref` 同步计算，无需额外求解。

---

### 11.3 关于到位精度：速度规划能否解决？

#### 速度规划能解决的部分

- **减速段超调**：速度规划保证 `v(s_goal) = 0`，机器人在目标弧长处速度恰好降为零，无动量超调
- **停车抖动**：jerk 约束使减速过程平滑，消除最后一段的速度振荡

#### 速度规划**无法**解决的部分

- **横向偏差**：速度规划是一维的（沿路径弧长），无法控制机器人偏离路径中心线的横向误差
- **定位噪声**：AMCL 定位误差 ±2~3cm，速度规划对此无影响
- **MPPI 采样随机性**：即使有速度剖面引导，MPPI 每周期的随机采样仍会引入微小位置偏差

#### 切换 PID 控制的问题

接近目标时切换 PID，**会引入速度命令跳变**（MPPI 输出某速度，PID 计算出另一个速度，切换瞬间不连续）。人形机器人对此更敏感。

#### 推荐方案：在 MPPI 框架内动态收紧采样

比切换 PID 更稳妥的方式是**距离自适应参数调整**，不切换控制器：

```cpp
// 在 computeVelocityCommands() 中，根据到目标距离动态调整
double dist_to_goal = hypot(goal.x - robot_pose.x, goal.y - robot_pose.y);

if (dist_to_goal < 0.3) {
  // 接近目标：收紧采样，降低速度上限
  optimizer_.setSpeedLimit(0.1, false);   // vx_max → 0.1 m/s
  optimizer_.setSamplingStd(0.05, 0.1);   // 收窄采样噪声
} else if (dist_to_goal < 0.8) {
  optimizer_.setSpeedLimit(0.2, false);
}
```

`setSpeedLimit()` 已在 [optimizer.cpp:420](src/optimizer.cpp#L420) 实现，可直接使用。`setSamplingStd` 需新增但改动极小。

这样做的效果：
- 低速采样 → MPPI 候选轨迹集中在目标附近 → GoalCritic 权重更有效
- 收窄 `vx_std` → 避免低速时随机噪声导致轨迹发散
- 全程 MPPI 控制，无切换不连续

---

### 11.4 修正后的架构设计

#### 正确架构

```
┌─────────────────────────────────────────────────────────────┐
│                     MPPI Controller                         │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  1. Path Handler + 曲率计算                           │   │
│  │     transformPath() → 局部路径                        │   │
│  │     κ(s) = 路径曲率（数值微分）                        │   │
│  └──────────────────────────────────────────────────────┘   │
│                         ↓                                   │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  2. Speed Planner（新增）                             │   │
│  │     输入：s0, v0, a0_est（估计加速度）, s_goal        │   │
│  │     输出：v(s) 弧长-速度曲线（非 v(t)）                │   │
│  │     约束：v_max, a_max, j_max                         │   │
│  │     s_goal = min(dist_to_goal, v_max×horizon×1.2)    │   │
│  └──────────────────────────────────────────────────────┘   │
│                         ↓                                   │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  3. Optimizer（修改内部运动模型）                      │   │
│  │     MotionModel::predict() 中加入加速度 ramp 约束      │   │
│  │     SpeedProfileCritic: 弧长插值对齐参考速度           │   │
│  │     距离自适应收紧 vx_max + sampling_std               │   │
│  └──────────────────────────────────────────────────────┘   │
│                         ↓                                   │
│                  cmd_vel 输出（无后处理截断）                │
└─────────────────────────────────────────────────────────────┘
```

**关键原则**：jerk 约束嵌入 MPPI 内部，不在输出端后处理。

#### 方案 A：在 `MotionModel::predict()` 中嵌入加速度 ramp（推荐，最小改动）

修改位置：[motion_models.hpp:53](include/nav2_mppi_controller/motion_models.hpp#L53)

基类 `predict()` 在传播速度时，对相邻时间步速度变化量施加约束：

```cpp
void predict(models::State & state) {
  // 原逻辑：state.vx[:,1:] = state.cvx[:,0:-1]
  // 修改：加入加速度限制，使 |vx[t+1] - vx[t]| / dt <= a_max
  //        以及 |(a[t+1] - a[t])| / dt <= j_max（可选）

  const Eigen::Index cols = state.vx.cols();
  for (Eigen::Index t = 1; t < cols; ++t) {
    const auto vx_prev = state.vx.col(t - 1);
    auto cvx_prev = state.cvx.col(t - 1);
    auto vx_next = state.vx.col(t);

    const auto delta_v = cvx_prev - vx_prev;
    vx_next = vx_prev + delta_v.min(a_max_ * dt_).max(a_min_ * dt_);
  }
}
```

**效果**：1000 条候选轨迹的速度序列都满足加速度约束，MPPI 从满足物理约束的轨迹中选择最优，输出自然平滑，无需后处理。

**需要在 `OptimizerSettings` 中新增**：

```cpp
struct OptimizerSettings {
  // ... 原有字段 ...
  float a_lim_vx{2.0f};   // 线加速度限制 m/s²
  float a_lim_wz{4.0f};   // 角加速度限制 rad/s²
  float j_lim_vx{0.0f};   // jerk 限制，0 表示不限（可选）
};
```

#### 方案 B：SpeedProfileCritic（弧长对齐版）

基于弧长插值的 Critic，解决时间-弧长对齐问题：

```cpp
void SpeedProfileCritic::score(CriticData & data) {
  auto & traj = data.trajectories;  // [batch × time_steps × {x,y,yaw}]

  // Step 1: 计算每条轨迹每步的累积弧长 [batch × time_steps]
  models::Matrix s = models::Matrix::Zero(traj.x.rows(), traj.x.cols());
  for (Eigen::Index t = 1; t < traj.x.cols(); ++t) {
    const auto dx = traj.x.col(t) - traj.x.col(t - 1);
    const auto dy = traj.y.col(t) - traj.y.col(t - 1);
    s.col(t) = s.col(t - 1) + (dx.square() + dy.square()).sqrt();
  }

  // Step 2: 对每条轨迹每步，从 v(s) 剖面插值参考速度
  // profile_s_: [N] 弧长采样点
  // profile_v_: [N] 对应速度
  auto v_ref = interpolate2D(profile_s_, profile_v_, s);  // [batch × time_steps]

  // Step 3: 计算代价
  const auto diff = data.state.cvx - v_ref;
  data.costs += cost_weight_ * diff.square().rowwise().sum();
}
```

---

### 11.5 Apollo Piecewise Jerk Speed Optimizer 原理

#### 数学模型

Apollo 使用**二次规划（QP）**求解速度曲线，优化变量为（以弧长 s 为自变量）：

```
x = [s₀, ṡ₀, s̈₀, s₁, ṡ₁, s̈₁, ..., sₙ₋₁, ṡₙ₋₁, s̈ₙ₋₁]ᵀ
```

其中 `ṡ = ds/dt` 为速度，`s̈ = d²s/dt²` 为加速度。

**核心假设**：相邻离散点之间 jerk 恒定（`s̈` 线性变化）。

#### 代价函数

```
J = Σᵢ [wₛ(sᵢ - sᵢ,ref)² + wᵥ(ṡᵢ - ṡᵢ,ref)² + wₐs̈ᵢ² + wⱼ(s̈ᵢ₊₁ - s̈ᵢ)²]
    + wₛ,ₑₙd(sₙ₋₁ - sₑₙd)² + wᵥ,ₑₙd(ṡₙ₋₁)²
```

- `wⱼ(s̈ᵢ₊₁ - s̈ᵢ)²`：**jerk 平滑项**（关键）
- 终端约束 `ṡₙ₋₁ = 0`：强制停车

#### 约束条件

1. **运动学连续性**（等式约束，由 jerk 恒定假设推导）：
   ```
   sᵢ₊₁ = sᵢ + ṡᵢΔt + ½s̈ᵢΔt² + ⅙jᵢΔt³
   ṡᵢ₊₁ = ṡᵢ + s̈ᵢΔt + ½jᵢΔt²
   s̈ᵢ₊₁ = s̈ᵢ + jᵢΔt
   ```

2. **边界约束**（不等式约束）：
   ```
   0 ≤ ṡᵢ ≤ v_max
   a_min ≤ s̈ᵢ ≤ a_max
   j_min ≤ jᵢ ≤ j_max
   ```

**注意**：QP 输出 `v(sᵢ)`（弧长-速度），与方案 B 的弧长对齐 Critic 天然匹配。

---

### 11.6 参数配置

**新增参数（`w1_nav2_params.yaml`）**：

```yaml
MPPIController:
  # 加速度/jerk 嵌入运动模型约束（方案 A）
  a_lim_vx: 1.0          # 线加速度限制 m/s²
  a_lim_wz: 2.0          # 角加速度限制 rad/s²

  # 速度规划参数（方案 B，按需启用）
  speed_planner:
    enabled: false        # 初期不启用，Phase 2 再开
    v_max: 0.5
    a_max: 0.8
    a_min: -0.8
    j_max: 2.0
    j_min: -2.0

  # 距离自适应参数（到位精度）
  goal_dist_treshold_slow: 0.8    # 进入慢速区距离 m
  goal_dist_treshold_fine: 0.3    # 进入精确区距离 m
  goal_vx_max_slow: 0.2           # 慢速区最大速度 m/s
  goal_vx_max_fine: 0.08          # 精确区最大速度 m/s
  goal_sampling_std_fine: 0.03    # 精确区采样标准差

  # SpeedProfileCritic（方案 B 启用时）
  SpeedProfileCritic:
    enabled: false
    cost_weight: 5.0
```

---

### 11.7 难点分析

#### 难点 1：弧长插值的计算效率

**问题**：SpeedProfileCritic 需要对 `batch_size × time_steps` 个点做弧长插值，1000×56 = 56000 次插值在 Eigen Array 上可能成为瓶颈。

**解决方案**：
- `profile_s_` 均匀采样 → 查找可用整除代替二分查找 → O(1)
- 用 Eigen Array 批量计算，避免不必要的逐元素标量代码
- 预估计算量：56000 次线性插值 < 0.5ms（现代 CPU）

#### 难点 2：运动模型的 predict() 加速度约束引入隐式依赖

**问题**：在 `predict()` 中对速度变化量限幅后，候选轨迹的速度不再独立于上一步。这破坏了噪声采样的统计独立性，理论上影响 MPPI 的最优性证明。

**实践影响**：较小。因为真实机器人本来就有加速度约束，限幅使采样轨迹更接近真实可执行空间，实际效果更好而非更差。

#### 难点 3：s_goal 的动态确定

**问题**：每帧需要计算 `distance_to_goal`（目标在变换路径中的弧长），涉及路径遍历。

**解决方案**：
- `path_handler_` 已经有路径截断逻辑，可复用
- 缓存上一帧的路径弧长，增量更新

#### 难点 4：QP 求解器依赖（方案 B）

**解决方案**：
- `osqp-eigen`（ROS2 生态已有，推荐）
- 或手写三对角矩阵求解器（速度规划特殊结构，可避免通用 QP）
- 热启动（warm start）：使用上一帧解初始化，收敛更快

---

### 11.8 实施路线图

```
Phase 1（1 周）：嵌入加速度约束
├── 在 MotionModel::predict() 中加入 a_lim_vx / a_lim_wz 裁剪
├── 在 OptimizerSettings 中新增参数并声明
├── 参数调优
└── 验证抖动改善（运动更平滑，采样轨迹物理合理）

Phase 2（1 周）：距离自适应精度优化
├── 在 controller.cpp 中增加到目标距离计算
├── 接近目标时动态调用 setSpeedLimit() + setSamplingStd()
└── 验证到位精度提升

Phase 3（2-3 周，按需）：完整速度规划
├── 集成 osqp-eigen
├── 实现 SpeedPlanner（v(s) 输出）
├── 实现 SpeedProfileCritic（弧长插值版）
└── 验证减速段平滑性
```

### 11.9 预期效果

| 指标 | 当前 | Phase 1 后 | Phase 2 后 | Phase 3 后 |
|------|------|-----------|-----------|-----------|
| 加速度连续性 | 无约束 | 硬约束 | 硬约束 | 硬约束 |
| 最大 jerk | 无限制 | ≤ a_lim/dt | ≤ a_lim/dt | ≤ j_max |
| 到位精度 | ±5~10cm | ±5~10cm | ±2~3cm | ±2~3cm |
| 停车超调 | 有 | 减少 | 基本消除 | 消除 |
| 计算开销 | 基准 | +0.1ms | +0.2ms | +1ms |

---

## 12. 总结

### 12.1 MPPI + 速度规划的本质

MPPI 是**轨迹跟踪器**，给定参考路径和速度剖面，通过随机采样找到最优控制。关键认知：

```
原来：Path → MPPI → cmd_vel（速度由 MPPI 自由探索，无物理约束）
现在：Path + v(s) → MPPI（内部满足加速度约束）→ cmd_vel（物理可执行）
```

### 12.2 设计原则

1. **约束内嵌，不后处理**：jerk / 加速度约束放在 `predict()` 内，保证 MPPI 的状态反馈一致性
2. **弧长对齐，不时间对齐**：速度剖面以 `v(s)` 表示，Critic 用弧长插值
3. **渐进增强，不切换控制器**：距离自适应调整采样参数，不切换到 PID
4. **分阶段实施**：Phase 1 最小改动即可显著改善，Phase 3 按需执行

---

*文档生成时间：2026-03-20 | 评审更新：2026-03-20*
*对应源码分支：`pzc/feature/speedplan`*
