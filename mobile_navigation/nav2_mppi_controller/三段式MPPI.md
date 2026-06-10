# 三段式 MPPI 参数说明

本文档整理当前 `nav2_mppi_controller` 中三段式 MPPI Profile 的切换规则、各阶段可覆盖参数，以及建议放入 `w1_nav2_params.yaml` 的配置模板。

代码位置：

- 控制器三阶段读取与切换：`src/controller.cpp`
- Profile 应用到速度约束：`src/optimizer.cpp`
- 路径进度和目标距离计算：`src/path_handler.cpp`
- 当前主要参数文件：`../nav2_bringup/params/w1_nav2_params.yaml`

## 1. 当前状态

当前代码已经支持三段式 Profile，并且 `profile_enabled` 代码默认值已经改为 `true`。当前 `w1_nav2_params.yaml` 的 `FollowPath` 下暂时不配置三段式参数：

```yaml
profile_enabled
profile_start_distance
profile_goal_distance
profile_1
profile_2
profile_3
```

因此如果不补充这些参数，运行时会使用代码默认值：

| 参数 | 默认值 | 影响 |
|---|---:|---|
| `profile_enabled` | `true` | 三段式默认开启 |
| `profile_start_distance` | `0.8` | 路径累计前进小于 0.8 m 时使用起步段 |
| `profile_goal_distance` | `1.0` | 距离终点小于等于 1.0 m 时使用末段 |

也就是说：**即使 YAML 不配置 `profile_enabled`，当前代码也会默认按三阶段进行切换。** 但如果 YAML 不配置 `profile_1/2/3`，三个阶段会继承 `FollowPath` 下的全局速度约束和 critic 参数，此时会有阶段切换日志，但三个阶段的实际控制参数相同。

## 2. 三阶段切换规则

每次 `computeVelocityCommands()` 时，控制器会先转换局部路径，然后从 `PathHandler` 获取两个量：

| 变量 | 含义 |
|---|---|
| `path_progress` | 沿全局路径累计前进距离 |
| `goal_distance` | 当前机器人到最终目标点的直线距离 |

切换逻辑如下：

```text
如果 goal_distance <= profile_goal_distance
    使用 profile_3：末段

否则如果 path_progress < profile_start_distance
    使用 profile_1：起步段

否则
    使用 profile_2：中间段
```

注意：**末段优先级最高**。即使机器人刚开始走，只要 `goal_distance <= profile_goal_distance`，也会进入 `profile_3`。

## 3. 三个阶段的定位

```text
三段式 MPPI
├── profile_1：起步段
│   ├── 触发条件：path_progress < profile_start_distance
│   └── 目标：起步更稳，减少刚开始时方向/路径贴合突变
│
├── profile_2：中间段
│   ├── 触发条件：不在起步段，也不在末段
│   └── 目标：正常路径跟踪，强调贴线和前向推进
│
└── profile_3：末段
    ├── 触发条件：goal_distance <= profile_goal_distance
    └── 目标：降速、收敛到目标点和目标角度，减少终点超调
```

## 4. Profile 可以覆盖哪些参数

每个 `profile_N` 当前支持覆盖两类参数。

### 4.1 速度约束

| 参数 | 含义 | 作用 |
|---|---|---|
| `vx_max` | 最大前进速度 | 限制前进速度上限 |
| `vx_min` | 最小线速度 | 负值表示允许倒车 |
| `vy_max` | 最大侧向速度 | DiffDrive 下通常不实际使用 |
| `wz_max` | 最大角速度 | 限制旋转速度上限 |

这些参数会覆盖 `Optimizer` 内部的 `base_constraints`，然后再叠加速度限制层的 `speed_limit`。

### 4.2 Critic 参数

当前支持按阶段覆盖的 critic 有：

| Critic | 可覆盖参数 |
|---|---|
| `GoalCritic` | `cost_weight`, `threshold_to_consider` |
| `GoalAngleCritic` | `cost_weight`, `threshold_to_consider` |
| `PathAngleCritic` | `cost_weight`, `threshold_to_consider`, `offset_from_furthest`, `max_angle_to_furthest`, `forward_preference` |
| `PathFollowCritic` | `cost_weight`, `threshold_to_consider`, `offset_from_furthest` |
| `PathAlignCritic` | `cost_weight`, `threshold_to_consider`, `offset_from_furthest`, `trajectory_point_step`, `max_path_occupancy_ratio` |
| `PreferForwardCritic` | `cost_weight`, `threshold_to_consider` |

当前不支持按阶段覆盖的主要参数包括：

| 类型 | 说明 |
|---|---|
| 采样参数 | `vx_std`, `vy_std`, `wz_std`, `batch_size`, `time_steps`, `model_dt` |
| 约束参数 | `ax_max`, `ax_min`, `az_max`, `jx_max`, `jz_max` |
| 障碍物相关 critic | `ObstaclesCritic`, `CostCritic` 等 |
| 死区 critic | `VelocityDeadbandCritic` |

这些参数仍然使用 `FollowPath` 下的全局配置。

## 5. 当前全局 MPPI 参数基线

当前 `w1_nav2_params.yaml` 里的 `FollowPath` 全局基线如下：

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `time_steps` | `40` | 预测步数 |
| `model_dt` | `0.1` | 每步时间，预测时域约 4.0 s |
| `batch_size` | `2000` | 采样轨迹数量 |
| `vx_std` | `0.25` | 线速度采样标准差 |
| `vy_std` | `0.2` | 侧向速度采样标准差 |
| `wz_std` | `0.3` | 角速度采样标准差 |
| `vx_max` | `0.7` | 全局最大前进速度 |
| `vx_min` | `-0.5` | 全局最小速度，允许倒车 |
| `vy_max` | `0.5` | 全局侧向速度上限 |
| `wz_max` | `0.75` | 全局角速度上限 |
| `ax_max` | `0.5` | 最大线加速度 |
| `ax_min` | `-0.5` | 最大线减速度 |
| `az_max` | `0.7` | 最大角加速度 |
| `jx_max` | `1.0` | 最大线 jerk |
| `jz_max` | `1.5` | 最大角 jerk |
| `motion_model` | `DiffDrive` | 差速模型 |
| `open_loop` | `true` | 使用上一周期输出作为速度状态 |

## 6. 建议三阶段参数

下面这组参数适合放在 `FollowPath` 下，和 `vx_max/vx_min/vy_max/wz_max` 同级。

### 6.1 阶段总开关

```yaml
profile_enabled: true
profile_start_distance: 0.8
profile_goal_distance: 1.0
```

含义：

| 参数 | 建议值 | 含义 |
|---|---:|---|
| `profile_enabled` | `true` | 开启三段式 |
| `profile_start_distance` | `0.8` | 路径累计前进小于 0.8 m 时使用起步段 |
| `profile_goal_distance` | `1.0` | 距离终点小于等于 1.0 m 时使用末段 |

### 6.2 profile_1：起步段

```yaml
profile_1:
  vx_max: 0.7
  vx_min: -0.5
  vy_max: 0.5
  wz_max: 0.75
  PathAngleCritic:
    cost_weight: 8.0
    offset_from_furthest: 6
    threshold_to_consider: 0.5
    max_angle_to_furthest: 0.4
    forward_preference: true
  PathAlignCritic:
    cost_weight: 8.0
    offset_from_furthest: 15
    trajectory_point_step: 3
    threshold_to_consider: 0.5
    max_path_occupancy_ratio: 0.05
  PathFollowCritic:
    cost_weight: 5.0
    offset_from_furthest: 5
    threshold_to_consider: 2.2
  PreferForwardCritic:
    cost_weight: 8.0
    threshold_to_consider: 0.25
```

起步段设计意图：

| 参数变化 | 作用 |
|---|---|
| `PathAngleCritic.cost_weight=8.0` | 起步时更强调朝向路径方向，减少刚起步时横着切路径 |
| `PathAngleCritic.max_angle_to_furthest=0.4` | 起步阶段对朝向偏差更严格 |
| `PreferForwardCritic.cost_weight=8.0` | 起步时更偏好前进，减少不必要倒车 |
| 速度上限保持 `0.7/0.75` | 起步段不主动降速，主要靠 critic 调整行为 |

### 6.3 profile_2：中间段

```yaml
profile_2:
  vx_max: 0.7
  vx_min: -0.5
  vy_max: 0.5
  wz_max: 0.75
  PathAngleCritic:
    cost_weight: 4.0
    offset_from_furthest: 2
    threshold_to_consider: 0.5
    max_angle_to_furthest: 1.0
    forward_preference: true
  PathAlignCritic:
    cost_weight: 18.0
    offset_from_furthest: 15
    trajectory_point_step: 3
    threshold_to_consider: 0.5
    max_path_occupancy_ratio: 0.05
  PathFollowCritic:
    cost_weight: 8.0
    offset_from_furthest: 5
    threshold_to_consider: 2.2
  PreferForwardCritic:
    cost_weight: 5.0
    threshold_to_consider: 0.25
```

中间段设计意图：

| 参数变化 | 作用 |
|---|---|
| `PathAlignCritic.cost_weight=18.0` | 中间段更强调贴合参考路径 |
| `PathFollowCritic.cost_weight=8.0` | 提高沿路径向前推进的倾向 |
| `PathAngleCritic.cost_weight=4.0` | 朝向约束保留，但弱于起步段 |
| 速度上限保持 `0.7/0.75` | 中间段保持正常通行效率 |

### 6.4 profile_3：末段

```yaml
profile_3:
  vx_max: 0.35
  vx_min: -0.20
  vy_max: 0.5
  wz_max: 0.35
  PathAngleCritic:
    cost_weight: 3.0
    offset_from_furthest: 2
    threshold_to_consider: 0.5
    max_angle_to_furthest: 1.0
    forward_preference: true
  PathAlignCritic:
    cost_weight: 8.0
    offset_from_furthest: 15
    trajectory_point_step: 3
    threshold_to_consider: 0.5
    max_path_occupancy_ratio: 0.05
  PathFollowCritic:
    cost_weight: 4.0
    offset_from_furthest: 5
    threshold_to_consider: 1.2
  PreferForwardCritic:
    cost_weight: 5.0
    threshold_to_consider: 0.25
  GoalCritic:
    cost_weight: 16.0
    threshold_to_consider: 1.2
  GoalAngleCritic:
    cost_weight: 6.0
    threshold_to_consider: 1.0
```

末段设计意图：

| 参数变化 | 作用 |
|---|---|
| `vx_max=0.35` | 接近目标时降低最大前进速度，减少超调 |
| `vx_min=-0.20` | 限制倒车速度，避免终点附近大幅反复调整 |
| `wz_max=0.35` | 限制末段旋转速度，减少终点角度抖动 |
| `GoalCritic.cost_weight=16.0` | 更强调收敛到目标位置 |
| `GoalAngleCritic.cost_weight=6.0` | 更强调收敛到目标朝向 |
| `PathFollowCritic.threshold_to_consider=1.2` | 末段减少对远处路径点的追逐，更多交给目标 critic |

## 7. 完整 YAML 片段

可以将下面片段放到 `controller_server.ros__parameters.FollowPath` 下：

```yaml
profile_enabled: true
profile_start_distance: 0.8
profile_goal_distance: 1.0

profile_1:
  vx_max: 0.7
  vx_min: -0.5
  vy_max: 0.5
  wz_max: 0.75
  PathAngleCritic:
    cost_weight: 8.0
    offset_from_furthest: 6
    threshold_to_consider: 0.5
    max_angle_to_furthest: 0.4
    forward_preference: true
  PathAlignCritic:
    cost_weight: 8.0
    offset_from_furthest: 15
    trajectory_point_step: 3
    threshold_to_consider: 0.5
    max_path_occupancy_ratio: 0.05
  PathFollowCritic:
    cost_weight: 5.0
    offset_from_furthest: 5
    threshold_to_consider: 2.2
  PreferForwardCritic:
    cost_weight: 8.0
    threshold_to_consider: 0.25

profile_2:
  vx_max: 0.7
  vx_min: -0.5
  vy_max: 0.5
  wz_max: 0.75
  PathAngleCritic:
    cost_weight: 4.0
    offset_from_furthest: 2
    threshold_to_consider: 0.5
    max_angle_to_furthest: 1.0
    forward_preference: true
  PathAlignCritic:
    cost_weight: 18.0
    offset_from_furthest: 15
    trajectory_point_step: 3
    threshold_to_consider: 0.5
    max_path_occupancy_ratio: 0.05
  PathFollowCritic:
    cost_weight: 8.0
    offset_from_furthest: 5
    threshold_to_consider: 2.2
  PreferForwardCritic:
    cost_weight: 5.0
    threshold_to_consider: 0.25

profile_3:
  vx_max: 0.35
  vx_min: -0.20
  vy_max: 0.5
  wz_max: 0.35
  PathAngleCritic:
    cost_weight: 3.0
    offset_from_furthest: 2
    threshold_to_consider: 0.5
    max_angle_to_furthest: 1.0
    forward_preference: true
  PathAlignCritic:
    cost_weight: 8.0
    offset_from_furthest: 15
    trajectory_point_step: 3
    threshold_to_consider: 0.5
    max_path_occupancy_ratio: 0.05
  PathFollowCritic:
    cost_weight: 4.0
    offset_from_furthest: 5
    threshold_to_consider: 1.2
  PreferForwardCritic:
    cost_weight: 5.0
    threshold_to_consider: 0.25
  GoalCritic:
    cost_weight: 16.0
    threshold_to_consider: 1.2
  GoalAngleCritic:
    cost_weight: 6.0
    threshold_to_consider: 1.0
```

## 8. 调试日志怎么看

三段式启用后，日志中重点看这些字段：

```text
MPPI profiles: enabled=1 start_distance=0.800 goal_distance=1.000
MPPI path state: path_progress=... goal_distance=...
MPPI profile state: active=... selected=...
MPPI profile switched: 1 -> 2, reason=middle_segment/中间段
MPPI active constraints: vx=[..., ...] vy=... wz=...
```

判断方法：

| 日志字段 | 用途 |
|---|---|
| `path_progress` | 判断是否应该离开起步段 |
| `goal_distance` | 判断是否应该进入末段 |
| `active/selected` | 判断当前阶段和即将选择的阶段是否一致 |
| `profile switched` | 确认阶段确实发生切换 |
| `active constraints` | 确认 profile 速度约束是否真正应用到 optimizer |

如果 `profile_enabled=0`，说明 YAML 显式关闭了三段式，或者参数加载结果覆盖了代码默认值。

## 9. 风险和注意事项

- 三段式切换只改变速度约束和部分 critic 参数，不会重置 optimizer，也不会重建 critic 插件。
- `profile_3` 的 `vx_max/wz_max` 降低后，终点附近更稳，但到位时间可能变长。
- `PathAlignCritic.cost_weight` 过大时，机器人可能更执着于贴线；动态障碍较多时要观察是否绕障变差。
- `GoalCritic` 和 `GoalAngleCritic` 权重过大时，末段可能更积极地冲目标；如果出现终点附近抖动，需要结合 `vx_max/wz_max`、加速度和 goal checker 容差一起调。
- 如果使用速度限制层，最终运行速度约束是：**当前 profile 速度约束 × speed limit 比例**，或按绝对限速换算后的约束。
