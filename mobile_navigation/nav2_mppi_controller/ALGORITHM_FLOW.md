# MPPI Controller 算法流程文档

## 目录
- [1. 概述](#1-概述)
- [2. 算法原理](#2-算法原理)
- [3. 整体架构](#3-整体架构)
- [4. 核心流程](#4-核心流程)
- [5. 关键组件](#5-关键组件)
- [6. 详细流程图](#6-详细流程图)
- [7. 数学模型](#7-数学模型)
- [8. 参数配置](#8-参数配置)

---

## 1. 概述

### 1.1 MPPI 简介
Model Predictive Path Integral (MPPI) 控制器是一种基于采样的模型预测控制 (MPC) 算法，用于移动机器人的局部轨迹规划和路径跟踪。该算法通过迭代采样和评分的方式，找到最优的控制序列。

### 1.2 主要特性
- **预测性**: 前瞻多个时间步，考虑未来状态
- **基于采样**: 使用随机采样探索控制空间
- **自适应避障**: 通过代价函数实现动态避障
- **高性能**: 纯 CPU 实现，使用向量化和张量运算优化
- **可扩展**: 基于插件的评价函数架构

### 1.3 支持的运动模型
- **Differential Drive**: 差速驱动（两轮驱动）
- **Omni**: 全向轮驱动
- **Ackermann**: 阿克曼转向（类汽车）

---

## 2. 算法原理

### 2.1 MPPI 核心思想

MPPI 算法是一种信息论方法，通过以下步骤寻找最优控制：

1. **采样**: 在上一次最优控制序列的基础上，添加高斯噪声，生成一批候选轨迹
2. **预测**: 使用运动模型将控制序列前向传播，生成轨迹
3. **评价**: 使用多个评价函数对轨迹进行打分
4. **加权**: 使用 softmax 函数根据代价对控制进行加权平均
5. **迭代**: 重复上述过程直到收敛

### 2.2 与传统 MPC 的区别

| 特性 | MPPI | 传统 MPC |
|------|------|----------|
| 求解方式 | 采样 + softmax | 非线性优化 |
| 梯度需求 | 不需要 | 需要（梯度或 Hessian） |
| 计算复杂度 | 易于并行化 | 优化求解器 |
| 约束处理 | 软约束 + 硬约束 | 显式约束 |
| 局部最优 | 随机性更强，不易陷入 | 容易陷入局部最优 |

---

## 3. 整体架构

### 3.1 类图结构

```
MPPIController
    ├── Optimizer                  # 核心优化器
    │   ├── NoiseGenerator         # 噪声生成器
    │   ├── CriticManager          # 评价函数管理器
    │   │   └── CriticFunction[]   # 多个评价函数插件
    │   ├── MotionModel            # 运动模型
    │   ├── State                  # 状态数据
    │   ├── ControlSequence        # 控制序列
    │   └── Trajectories           # 轨迹数据
    ├── PathHandler                # 路径处理器
    ├── TrajectoryVisualizer       # 可视化工具
    └── ParametersHandler          # 参数管理器
```

### 3.2 数据流

```
输入: 机器人位姿、速度、全局路径
  ↓
路径转换 (PathHandler)
  ↓
优化器 (Optimizer)
  ├─ 准备状态数据
  ├─ 迭代优化
  │   ├─ 生成噪声轨迹
  │   ├─ 评价轨迹
  │   └─ 更新控制序列
  └─ 输出控制
  ↓
速度平滑 (Savitzky-Golay 滤波)
  ↓
输出: TwistStamped 速度命令
```

---

## 4. 核心流程

### 4.1 主控制循环

**位置**: `controller.cpp::computeVelocityCommands()`

```cpp
geometry_msgs::msg::TwistStamped computeVelocityCommands(
    robot_pose, robot_speed, goal_checker)
{
    // 1. 检查重置周期
    if (当前时间 - 上次调用时间 > reset_period) {
        reset();  // 重置优化器
    }
    
    // 2. 转换路径到机器人坐标系
    transformed_plan = path_handler_.transformPath(robot_pose);
    
    // 3. 获取代价地图锁
    lock(costmap);
    
    // 4. 核心优化计算
    cmd = optimizer_.evalControl(robot_pose, robot_speed, 
                                   transformed_plan, goal_checker);
    
    // 5. 可视化（可选）
    if (visualize_) {
        visualize(transformed_plan);
    }
    
    return cmd;
}
```

### 4.2 优化器核心流程

**位置**: `optimizer.cpp::evalControl()`

```cpp
geometry_msgs::msg::TwistStamped evalControl(
    robot_pose, robot_speed, plan, goal_checker)
{
    // 1. 准备阶段：初始化状态数据
    prepare(robot_pose, robot_speed, plan, goal_checker);
    
    // 2. 优化循环：可能失败重试
    do {
        optimize();  // 执行 MPPI 优化
    } while (fallback(fail_flag));  // 失败处理
    
    // 3. 平滑滤波：Savitzky-Golay 滤波器
    savitskyGolayFilter(control_sequence_, control_history_);
    
    // 4. 提取控制命令
    control = getControlFromSequenceAsTwist(timestamp);
    
    // 5. 控制序列移位（用于下次迭代）
    if (shift_control_sequence) {
        shiftControlSequence();
    }
    
    return control;
}
```

### 4.3 MPPI 优化迭代

**位置**: `optimizer.cpp::optimize()`

```cpp
void optimize()
{
    for (i = 0; i < iteration_count; ++i) {
        // 步骤1: 生成带噪声的候选轨迹
        generateNoisedTrajectories();
        
        // 步骤2: 评价所有轨迹
        critic_manager_.evalTrajectoriesScores(critics_data_);
        
        // 步骤3: 更新控制序列（softmax 加权）
        updateControlSequence();
    }
}
```

---

## 5. 关键组件

### 5.1 噪声轨迹生成

**位置**: `optimizer.cpp::generateNoisedTrajectories()`

**流程**:
```
1. 生成噪声控制
   noise_generator_.setNoisedControls(state_, control_sequence_);
   ↓
   对每个采样 batch:
     cvx[i] = control_sequence.vx + noise_vx[i]
     cwz[i] = control_sequence.wz + noise_wz[i]
     cvy[i] = control_sequence.vy + noise_vy[i]  # 全向轮

2. 预测状态速度
   updateStateVelocities(state_);
   ↓
   - 初始化当前速度
   - 使用运动模型预测速度序列

3. 积分生成轨迹
   integrateStateVelocities(trajectories, state_);
   ↓
   对每个时间步:
     yaw[t] = yaw[t-1] + wz[t] * dt
     x[t] = x[t-1] + (vx * cos(yaw) - vy * sin(yaw)) * dt
     y[t] = y[t-1] + (vx * sin(yaw) + vy * cos(yaw)) * dt
```

**噪声参数**:
- `vx_std`: 线速度 x 的标准差
- `vy_std`: 线速度 y 的标准差（全向轮）
- `wz_std`: 角速度的标准差

### 5.2 轨迹评价

**位置**: `critic_manager.cpp::evalTrajectoriesScores()`

**流程**:
```cpp
void evalTrajectoriesScores(CriticData & data)
{
    // 顺序调用所有评价函数
    for (critic in critics_) {
        if (data.fail_flag) break;  // 如果已失败则停止
        critic->score(data);          // 累加代价到 costs_
    }
}
```

**评价函数类型**:

| 评价函数 | 作用 | 关键参数 |
|---------|------|----------|
| **GoalCritic** | 接近目标点 | cost_weight, threshold_to_consider |
| **GoalAngleCritic** | 目标方向对齐 | cost_weight, threshold_to_consider |
| **PathFollowCritic** | 跟随路径 | cost_weight, offset_from_furthest |
| **PathAlignCritic** | 路径对齐 | cost_weight, max_path_occupancy_ratio |
| **PathAngleCritic** | 路径角度对齐 | cost_weight, offset_from_furthest |
| **ObstaclesCritic** | 避障 | cost_weight, inflation_radius |
| **CostCritic** | 代价地图代价 | cost_weight, cost_power |
| **ConstraintCritic** | 约束违反惩罚 | cost_weight |
| **PreferForwardCritic** | 偏好前向运动 | cost_weight, threshold |
| **TwirlingCritic** | 惩罚旋转 | cost_weight |

**评价数据结构**:
```cpp
struct CriticData {
    const State & state;              // 机器人状态
    const Trajectories & trajectories; // 候选轨迹
    const Path & path;                 // 参考路径
    xt::xtensor<float, 1> & costs;    // 代价数组 [batch_size]
    float & model_dt;                  // 时间步长
    bool fail_flag;                    // 失败标志
    nav2_core::GoalChecker * goal_checker;
    MotionModel * motion_model;
    // ... 其他可选数据
};
```

### 5.3 控制序列更新

**位置**: `optimizer.cpp::updateControlSequence()`

**数学原理**:

MPPI 使用信息论框架，控制更新公式如下：

$$
u^* = \sum_{i=1}^{N} w_i \cdot u_i
$$

其中权重 $w_i$ 由 softmax 函数计算：

$$
w_i = \frac{\exp(-\frac{1}{\lambda} S_i)}{\sum_{j=1}^{N} \exp(-\frac{1}{\lambda} S_j)}
$$

$S_i$ 是第 $i$ 个轨迹的总代价：

$$
S_i = \Phi_i + \gamma \cdot u_i^T R^{-1} u_i
$$

- $\Phi_i$: 评价函数的总代价
- $\gamma$: 控制代价权重（能量项）
- $R$: 控制协方差矩阵（对角阵）
- $\lambda$: 温度参数（temperature）

**代码实现**:
```cpp
void updateControlSequence()
{
    // 1. 计算控制成本（能量项）
    bounded_noises_vx = state.cvx - control_sequence.vx;
    bounded_noises_wz = state.cwz - control_sequence.wz;
    
    costs += gamma / (vx_std^2) * sum(control_sequence.vx * bounded_noises_vx);
    costs += gamma / (wz_std^2) * sum(control_sequence.wz * bounded_noises_wz);
    
    // 全向轮
    if (holonomic) {
        bounded_noises_vy = state.cvy - control_sequence.vy;
        costs += gamma / (vy_std^2) * sum(control_sequence.vy * bounded_noises_vy);
    }
    
    // 2. 归一化代价
    costs_normalized = costs - min(costs);
    
    // 3. 计算 softmax 权重
    exponents = exp(-1/temperature * costs_normalized);
    softmaxes = exponents / sum(exponents);
    
    // 4. 加权平均更新控制序列
    control_sequence.vx = sum(state.cvx * softmaxes);
    control_sequence.wz = sum(state.cwz * softmaxes);
    if (holonomic) {
        control_sequence.vy = sum(state.cvy * softmaxes);
    }
    
    // 5. 应用硬约束
    applyControlSequenceConstraints();
}
```

### 5.4 运动模型

**位置**: `motion_models.hpp`

#### 5.4.1 Differential Drive (差速驱动)

```cpp
class DiffDriveMotionModel : public MotionModel {
    bool isHolonomic() { return false; }  // 不支持侧向运动
    
    void predict(State & state) {
        // 直接使用控制速度
        state.vx = state.cvx;  // 线速度
        state.wz = state.cwz;  // 角速度
    }
};
```

#### 5.4.2 Omni (全向轮)

```cpp
class OmniMotionModel : public MotionModel {
    bool isHolonomic() { return true; }  // 支持侧向运动
    
    void predict(State & state) {
        state.vx = state.cvx;  // 前向速度
        state.vy = state.cvy;  // 侧向速度
        state.wz = state.cwz;  // 角速度
    }
};
```

#### 5.4.3 Ackermann (阿克曼转向)

```cpp
class AckermannMotionModel : public MotionModel {
    bool isHolonomic() { return false; }
    
    void applyConstraints(ControlSequence & control) {
        // 最小转弯半径约束
        wz_max = vx / min_turning_r;
        control.wz = clip(control.wz, -wz_max, wz_max);
    }
};
```

### 5.5 路径处理器

**位置**: `path_handler.cpp`

**功能**:
1. **路径转换**: 将全局路径转换到机器人当前坐标系
2. **路径修剪**: 删除机器人后方的路径点
3. **反转处理**: 处理路径反转点（如倒车点）

```cpp
nav_msgs::msg::Path transformPath(robot_pose)
{
    // 1. 找到离机器人最近的路径点
    nearest_point = findNearestPathPoint(robot_pose);
    
    // 2. 修剪路径
    pruned_path = prunePath(nearest_point, prune_distance);
    
    // 3. 处理路径反转（如果启用）
    if (enforce_path_inversion) {
        pruned_path = handlePathInversion(pruned_path, robot_pose);
    }
    
    // 4. 转换到机器人坐标系
    transformed_path = transformToRobotFrame(pruned_path, robot_pose);
    
    return transformed_path;
}
```

### 5.6 速度平滑

**位置**: `tools/utils.hpp::savitskyGolayFilter()`

**作用**: 对控制序列进行平滑，减少抖动

**原理**: Savitzky-Golay 滤波器使用局部多项式回归平滑数据

```cpp
void savitskyGolayFilter(
    ControlSequence & control_sequence,
    std::array<Control, 4> & control_history)
{
    // 使用历史控制和当前控制序列的前几个点
    // 进行多项式拟合和平滑
    
    // 一般使用 5 点滤波器，系数如下:
    // [-3, 12, 17, 12, -3] / 35
    
    for (i = 0; i < filter_size; ++i) {
        smoothed[i] = sum(coefficients[j] * values[i+j]);
    }
}
```

---

## 6. 详细流程图

### 6.1 总体流程

```
开始
  ↓
[配置阶段]
  ├─ 加载参数
  ├─ 初始化优化器
  ├─ 加载评价函数插件
  └─ 初始化运动模型
  ↓
[激活阶段]
  ↓
┌─[主循环: computeVelocityCommands]─────────────────┐
│                                                     │
│  1. 检查重置周期                                    │
│     if (timeout) reset()                           │
│     ↓                                               │
│  2. 路径转换                                        │
│     transformed_plan = pathHandler.transform()     │
│     ↓                                               │
│  3. 优化计算                                        │
│     ┌─[evalControl]────────────────────────────┐  │
│     │                                           │  │
│     │  A. 准备阶段                              │  │
│     │     - 初始化状态                          │  │
│     │     - 转换路径为张量                      │  │
│     │     - 重置代价数组                        │  │
│     │     ↓                                      │  │
│     │  B. 优化循环 (可重试)                     │  │
│     │     ┌─[optimize]──────────────────────┐  │  │
│     │     │                                  │  │  │
│     │     │  迭代 iteration_count 次:        │  │  │
│     │     │  ┌───────────────────────────┐  │  │  │
│     │     │  │ 1. 生成噪声轨迹           │  │  │  │
│     │     │  │    - 添加高斯噪声         │  │  │  │
│     │     │  │    - 预测速度            │  │  │  │
│     │     │  │    - 积分得到轨迹         │  │  │  │
│     │     │  │    ↓                      │  │  │  │
│     │     │  │ 2. 评价轨迹              │  │  │  │
│     │     │  │    - 依次调用评价函数     │  │  │  │
│     │     │  │    - 累加代价            │  │  │  │
│     │     │  │    ↓                      │  │  │  │
│     │     │  │ 3. 更新控制序列          │  │  │  │
│     │     │  │    - 计算 softmax 权重   │  │  │  │
│     │     │  │    - 加权平均            │  │  │  │
│     │     │  │    - 应用约束            │  │  │  │
│     │     │  └───────────────────────────┘  │  │  │
│     │     │                                  │  │  │
│     │     └──────────────────────────────────┘  │  │
│     │     ↓                                      │  │
│     │  C. 失败重试                               │  │
│     │     if (fail_flag) {                       │  │
│     │         reset()                            │  │
│     │         retry (最多 retry_attempt_limit)   │  │
│     │     }                                       │  │
│     │     ↓                                       │  │
│     │  D. 平滑滤波                               │  │
│     │     savitskyGolayFilter()                  │  │
│     │     ↓                                       │  │
│     │  E. 提取控制                               │  │
│     │     control = getControlFromSequence()     │  │
│     │     ↓                                       │  │
│     │  F. 控制序列移位                           │  │
│     │     shiftControlSequence()                 │  │
│     │                                             │  │
│     └─────────────────────────────────────────────┘  │
│     ↓                                               │
│  4. 可视化 (可选)                                   │
│     - 候选轨迹                                      │
│     - 最优轨迹                                      │
│     ↓                                               │
│  5. 返回控制命令                                    │
│                                                     │
└─────────────────────────────────────────────────────┘
  ↓
输出: TwistStamped
```

### 6.2 优化迭代详细流程

```
┌─[MPPI 迭代]─────────────────────────────────────────┐
│                                                      │
│  输入:                                               │
│  - control_sequence: 上次最优控制序列               │
│  - robot_state: 当前机器人状态                      │
│  - path: 参考路径                                   │
│                                                      │
│  ┌────────────────────────────────────────────────┐│
│  │ 1. 噪声生成                                    ││
│  │    for i in batch_size:                        ││
│  │        noise_vx[i] ~ N(0, vx_std^2)            ││
│  │        noise_wz[i] ~ N(0, wz_std^2)            ││
│  │        noise_vy[i] ~ N(0, vy_std^2) [全向轮]   ││
│  └────────────────────────────────────────────────┘│
│  ↓                                                   │
│  ┌────────────────────────────────────────────────┐│
│  │ 2. 控制采样                                    ││
│  │    for i in batch_size:                        ││
│  │        cvx[i] = control_sequence.vx + noise_vx ││
│  │        cwz[i] = control_sequence.wz + noise_wz ││
│  │        cvy[i] = control_sequence.vy + noise_vy ││
│  └────────────────────────────────────────────────┘│
│  ↓                                                   │
│  ┌────────────────────────────────────────────────┐│
│  │ 3. 运动预测                                    ││
│  │    for i in batch_size:                        ││
│  │        for t in time_steps:                    ││
│  │            state.vx[i,t] = motion_model.predict(cvx) ││
│  │            state.wz[i,t] = motion_model.predict(cwz) ││
│  └────────────────────────────────────────────────┘│
│  ↓                                                   │
│  ┌────────────────────────────────────────────────┐│
│  │ 4. 轨迹积分                                    ││
│  │    for i in batch_size:                        ││
│  │        for t in time_steps:                    ││
│  │            yaw[t] = yaw[t-1] + wz[t] * dt      ││
│  │            x[t] = x[t-1] + vx*cos(yaw) * dt    ││
│  │            y[t] = y[t-1] + vx*sin(yaw) * dt    ││
│  └────────────────────────────────────────────────┘│
│  ↓                                                   │
│  ┌────────────────────────────────────────────────┐│
│  │ 5. 轨迹评价                                    ││
│  │    costs[i] = 0  for all i                     ││
│  │    for each critic:                            ││
│  │        for i in batch_size:                    ││
│  │            cost = critic.score(trajectory[i])  ││
│  │            costs[i] += weight * cost           ││
│  └────────────────────────────────────────────────┘│
│  ↓                                                   │
│  ┌────────────────────────────────────────────────┐│
│  │ 6. 控制能量项                                  ││
│  │    costs[i] += gamma * sum(                    ││
│  │        (cvx[i] - control_sequence.vx)^2 / vx_std^2 + ││
│  │        (cwz[i] - control_sequence.wz)^2 / wz_std^2 ) ││
│  └────────────────────────────────────────────────┘│
│  ↓                                                   │
│  ┌────────────────────────────────────────────────┐│
│  │ 7. Softmax 计算                                ││
│  │    costs_norm = costs - min(costs)             ││
│  │    exp_costs = exp(-costs_norm / temperature)  ││
│  │    weights = exp_costs / sum(exp_costs)        ││
│  └────────────────────────────────────────────────┘│
│  ↓                                                   │
│  ┌────────────────────────────────────────────────┐│
│  │ 8. 加权更新                                    ││
│  │    control_sequence.vx = sum(weights[i] * cvx[i]) ││
│  │    control_sequence.wz = sum(weights[i] * cwz[i]) ││
│  │    control_sequence.vy = sum(weights[i] * cvy[i]) ││
│  └────────────────────────────────────────────────┘│
│  ↓                                                   │
│  ┌────────────────────────────────────────────────┐│
│  │ 9. 应用约束                                    ││
│  │    vx = clip(vx, vx_min, vx_max)               ││
│  │    wz = clip(wz, -wz_max, wz_max)              ││
│  │    motion_model.applyConstraints()             ││
│  └────────────────────────────────────────────────┘│
│  ↓                                                   │
│  输出: 更新后的 control_sequence                     │
│                                                      │
└──────────────────────────────────────────────────────┘
```

---

## 7. 数学模型

### 7.1 状态空间表示

**机器人状态**:
$$
\mathbf{x}(t) = [x, y, \theta, v_x, v_y, \omega]^T
$$

**控制输入**:
$$
\mathbf{u}(t) = [u_{v_x}, u_{v_y}, u_{\omega}]^T
$$

### 7.2 运动学模型

#### Differential Drive:
$$
\begin{align}
\dot{x} &= v_x \cos(\theta) \\
\dot{y} &= v_x \sin(\theta) \\
\dot{\theta} &= \omega \\
v_x &= u_{v_x} \\
\omega &= u_{\omega}
\end{align}
$$

#### Omni:
$$
\begin{align}
\dot{x} &= v_x \cos(\theta) - v_y \sin(\theta) \\
\dot{y} &= v_x \sin(\theta) + v_y \cos(\theta) \\
\dot{\theta} &= \omega \\
v_x &= u_{v_x}, \quad v_y = u_{v_y}, \quad \omega = u_{\omega}
\end{align}
$$

#### Ackermann:
$$
\begin{align}
\dot{x} &= v_x \cos(\theta) \\
\dot{y} &= v_x \sin(\theta) \\
\dot{\theta} &= \omega \\
|\omega| &\leq \frac{|v_x|}{r_{min}}
\end{align}
$$

### 7.3 MPPI 代价函数

总代价函数由两部分组成：

$$
S(\tau_i) = \underbrace{\sum_{t=0}^{T} \sum_{k} w_k \cdot \phi_k(x_t, u_t)}_{\text{评价函数代价}} + \underbrace{\gamma \sum_{t=0}^{T} \mathbf{u}_t^T R^{-1} \mathbf{u}_t}_{\text{控制能量代价}}
$$

其中:
- $\tau_i$: 第 $i$ 个采样轨迹
- $T$: 预测时域长度
- $\phi_k$: 第 $k$ 个评价函数
- $w_k$: 评价函数权重
- $\gamma$: 能量权重参数
- $R$: 控制协方差矩阵

### 7.4 Softmax 权重计算

$$
w_i = \frac{\exp\left(-\frac{1}{\lambda} S(\tau_i)\right)}{\sum_{j=1}^{N} \exp\left(-\frac{1}{\lambda} S(\tau_j)\right)}
$$

其中:
- $N$: batch_size（采样数量）
- $\lambda$: temperature（温度参数）
  - $\lambda \to 0$: 选择最优轨迹（贪婪）
  - $\lambda \to \infty$: 均匀平均

### 7.5 控制更新公式

$$
\mathbf{u}^*_t = \sum_{i=1}^{N} w_i \cdot \mathbf{u}_{i,t}
$$

---

## 8. 参数配置

### 8.1 核心参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `batch_size` | int | 1000 | 每次迭代的采样轨迹数量，越大越精确但计算量越大 |
| `time_steps` | int | 56 | 预测时域的时间步数 |
| `model_dt` | double | 0.05 | 时间步长（秒），决定预测时域长度 = time_steps × model_dt |
| `iteration_count` | int | 1 | MPPI 迭代次数，通常保持为 1 |
| `temperature` | double | 0.3 | softmax 温度参数，控制轨迹选择的锐度 |
| `gamma` | double | 0.015 | 控制能量权重，平衡平滑度和能量消耗 |

### 8.2 速度约束

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `vx_max` | double | 0.5 | 最大前向线速度（m/s） |
| `vx_min` | double | -0.35 | 最小前向线速度（m/s），负值表示后退 |
| `vy_max` | double | 0.5 | 最大侧向线速度（m/s），仅全向轮 |
| `wz_max` | double | 1.9 | 最大角速度（rad/s） |

### 8.3 噪声参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `vx_std` | double | 0.2 | vx 采样标准差，控制前向速度的探索范围 |
| `vy_std` | double | 0.2 | vy 采样标准差，仅全向轮 |
| `wz_std` | double | 0.4 | wz 采样标准差，控制角速度的探索范围 |

### 8.4 其他参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `motion_model` | string | "DiffDrive" | 运动模型: DiffDrive / Omni / Ackermann |
| `visualize` | bool | false | 是否可视化轨迹（会显著降低性能） |
| `retry_attempt_limit` | int | 1 | 失败重试次数 |
| `reset_period` | double | 1.0 | 不活动重置周期（秒） |
| `regenerate_noises` | bool | false | 是否每次迭代重新生成噪声 |

### 8.5 路径处理参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `prune_distance` | double | 1.5 | 路径修剪距离（m） |
| `transform_tolerance` | double | 0.1 | TF 变换时间容差（秒） |
| `enforce_path_inversion` | bool | false | 是否强制路径反转点 |
| `inversion_xy_tolerance` | double | 0.2 | 反转点位置容差（m） |
| `inversion_yaw_tolerance` | double | 0.4 | 反转点角度容差（rad） |

### 8.6 评价函数参数示例

**GoalCritic**:
```yaml
GoalCritic:
  cost_weight: 5.0
  cost_power: 1
  threshold_to_consider: 1.4  # 距离目标多远开始考虑该代价
```

**ObstaclesCritic**:
```yaml
ObstaclesCritic:
  cost_weight: 5.0
  consider_footprint: true
  collision_cost: 1000000.0
  near_goal_distance: 0.5
  inflation_radius: 0.5
  inflation_scale_factor: 3.0
```

**PathFollowCritic**:
```yaml
PathFollowCritic:
  cost_weight: 10.0
  offset_from_furthest: 20  # 从最远可达点的偏移
  threshold_to_consider: 1.4
```

### 8.7 参数调优建议

#### 8.7.1 提高跟踪精度
- 增大 `batch_size` (1000 → 2000)
- 增大 `PathFollowCritic.cost_weight`
- 减小 `temperature` (0.3 → 0.2)

#### 8.7.2 提高避障能力
- 增大 `ObstaclesCritic.cost_weight`
- 增大 `inflation_radius`
- 增大 `batch_size` 以探索更多避障路径

#### 8.7.3 更平滑的运动
- 增大 `gamma` (0.015 → 0.05)
- 减小噪声标准差 `vx_std`, `wz_std`
- 启用 Savitzky-Golay 滤波（默认启用）

#### 8.7.4 更快的响应
- 减小 `time_steps` (56 → 40)
- 减小 `model_dt` (0.05 → 0.03)
- 减小 `batch_size` (仅当计算资源不足时)

#### 8.7.5 减少计算量
- 减小 `batch_size` (1000 → 500)
- 减小 `time_steps` (56 → 40)
- 关闭可视化 `visualize: false`
- 设置 `regenerate_noises: false`

---

## 9. 性能分析

### 9.1 计算复杂度

**时间复杂度**: $O(N \times T \times C)$

其中:
- $N$: `batch_size` - 采样数量
- $T$: `time_steps` - 预测时域步数
- $C$: 评价函数数量

**典型配置的计算量**:
- batch_size = 1000
- time_steps = 56
- critics = 6
- 总计算: ~336,000 次评价 + 轨迹积分

### 9.2 性能优化技术

1. **向量化计算**: 使用 xtensor 进行批量张量运算
2. **并行化**: 噪声生成在独立线程中运行
3. **内存预分配**: 避免动态内存分配
4. **代价地图缓存**: 复用代价查询结果

### 9.3 基准性能

在中等性能 CPU（Intel i5 4代）上:
- **运行频率**: 50+ Hz
- **单周期延迟**: < 20 ms

---

## 10. 常见问题与调试

### 10.1 机器人抖动

**原因**:
- temperature 太大，导致权重分布过于均匀
- 噪声标准差太大
- 控制频率与 model_dt 不匹配

**解决方案**:
```yaml
temperature: 0.2          # 降低温度
vx_std: 0.1               # 减小噪声
wz_std: 0.2
model_dt: 0.05            # 匹配控制频率 (20 Hz)
```

### 10.2 避障失败

**原因**:
- ObstaclesCritic 权重太低
- batch_size 太小，没有采样到避障轨迹
- inflation_radius 设置不当

**解决方案**:
```yaml
ObstaclesCritic:
  cost_weight: 10.0       # 增大权重
  inflation_radius: 0.6   # 适当增大膨胀半径
batch_size: 1500          # 增加采样数
```

### 10.3 路径跟踪不精确

**原因**:
- PathFollowCritic 权重太低
- temperature 太大
- prune_distance 设置不当

**解决方案**:
```yaml
PathFollowCritic:
  cost_weight: 15.0       # 增大路径跟踪权重
PathAlignCritic:
  cost_weight: 8.0
temperature: 0.25         # 降低温度，更贪婪
prune_distance: 1.2       # 调整修剪距离
```

### 10.4 优化失败 (fail_flag)

**原因**:
- 所有轨迹都与障碍物碰撞
- 评价函数设置不合理

**解决方案**:
- 增加 `retry_attempt_limit`
- 检查 `collision_cost` 是否过低
- 增大 `batch_size`
- 检查代价地图配置

### 10.5 计算延迟过高

**原因**:
- batch_size 或 time_steps 太大
- 启用了可视化
- critics 数量过多

**解决方案**:
```yaml
batch_size: 800           # 减小采样数
time_steps: 48            # 减小预测时域
visualize: false          # 关闭可视化
```

---

## 11. 扩展与定制

### 11.1 添加自定义评价函数

**步骤**:

1. 创建新的 critic 类，继承 `CriticFunction`:

```cpp
#include "nav2_mppi_controller/critic_function.hpp"

namespace mppi::critics {

class MyCustomCritic : public CriticFunction {
public:
    void initialize() override {
        auto getParam = parameters_handler_->getParamGetter(name_);
        getParam(my_param_, "my_param", 1.0);
        getParam(power_, "cost_power", 1);
        getParam(weight_, "cost_weight", 1.0);
    }
    
    void score(CriticData & data) override {
        if (!enabled_) return;
        
        // 访问数据
        auto & trajectories = data.trajectories;
        auto & costs = data.costs;
        auto & state = data.state;
        
        // 计算代价
        for (size_t i = 0; i < batch_size; ++i) {
            float cost = computeCost(trajectories.x(i), 
                                     trajectories.y(i),
                                     trajectories.yaws(i));
            costs(i) += weight_ * pow(cost, power_);
        }
    }
    
private:
    float my_param_;
    int power_;
    float weight_;
};

}  // namespace mppi::critics

// 注册插件
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    mppi::critics::MyCustomCritic,
    mppi::critics::CriticFunction)
```

2. 在 `critics.xml` 中注册:

```xml
<library path="my_custom_critic">
  <class type="mppi::critics::MyCustomCritic" 
         base_class_type="mppi::critics::CriticFunction">
    <description>My custom critic function</description>
  </class>
</library>
```

3. 在参数文件中配置:

```yaml
controller_server:
  ros__parameters:
    MPPIController:
      critics: ["GoalCritic", "ObstaclesCritic", "MyCustomCritic"]
      MyCustomCritic:
        cost_weight: 2.0
        cost_power: 1
        my_param: 1.5
```

### 11.2 自定义运动模型

继承 `MotionModel` 类并实现:
- `predict()`: 速度预测
- `isHolonomic()`: 是否全向
- `applyConstraints()`: 应用约束

### 11.3 修改采样策略

修改 `NoiseGenerator` 类可以改变:
- 噪声分布类型（当前是高斯分布）
- 采样策略（如使用协方差矩阵）
- 并行化策略

---

## 12. 参考文献

1. **MPPI 原始论文**:
   - Williams, G., Wagener, N., Goldfain, B., Drews, P., Rehg, J. M., Boots, B., & Theodorou, E. A. (2017). Information theoretic MPC for model-based reinforcement learning. In 2017 IEEE International Conference on Robotics and Automation (ICRA).

2. **Nav2 文档**:
   - [MPPI Controller Documentation](https://navigation.ros.org/configuration/packages/configuring-mppic.html)

3. **相关资源**:
   - GitHub: [nav2_mppi_controller](https://github.com/ros-planning/navigation2/tree/main/nav2_mppi_controller)
   - 作者: Aleksei Budyakov (@artofnothingness), Steve Macenski

---

## 附录

### A. 关键数据结构

**State** (`models/state.hpp`):
```cpp
struct State {
    geometry_msgs::msg::PoseStamped pose;
    geometry_msgs::msg::Twist speed;
    
    // 控制速度 [batch_size, time_steps]
    xt::xtensor<float, 2> cvx, cvy, cwz;
    
    // 实际速度 [batch_size, time_steps]
    xt::xtensor<float, 2> vx, vy, wz;
};
```

**Trajectories** (`models/trajectories.hpp`):
```cpp
struct Trajectories {
    // [batch_size, time_steps]
    xt::xtensor<float, 2> x;
    xt::xtensor<float, 2> y;
    xt::xtensor<float, 2> yaws;
};
```

**ControlSequence** (`models/control_sequence.hpp`):
```cpp
struct ControlSequence {
    // [time_steps]
    xt::xtensor<float, 1> vx;
    xt::xtensor<float, 1> vy;  // 仅全向轮
    xt::xtensor<float, 1> wz;
};
```

### B. 代码文件索引

| 文件 | 主要功能 |
|------|----------|
| `controller.cpp` | 主控制器，接口实现 |
| `optimizer.cpp` | MPPI 优化核心算法 |
| `critic_manager.cpp` | 评价函数管理 |
| `noise_generator.cpp` | 噪声生成 |
| `path_handler.cpp` | 路径处理 |
| `motion_models.hpp` | 运动模型 |
| `critics/*.cpp` | 各种评价函数实现 |
| `tools/utils.hpp` | 工具函数（含滤波） |

---

**文档版本**: v1.0  
**最后更新**: 2026-01-13  
**维护者**: Your Name
