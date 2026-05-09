# MPPI Controller 评价函数判断条件详解

本文档详细梳理了 MPPI Controller 中所有评价函数（Critics）的判断条件含义。

---

## 目录
1. [ConstraintCritic - 约束评价](#1-constraintcritic---约束评价)
2. [CostCritic - 代价地图评价](#2-costcritic---代价地图评价)
3. [GoalCritic - 目标位置评价](#3-goalcritic---目标位置评价)
4. [GoalAngleCritic - 目标角度评价](#4-goalanglecritic---目标角度评价)
5. [ObstaclesCritic - 障碍物评价](#5-obstaclescritic---障碍物评价)
6. [PathFollowCritic - 路径跟随评价](#6-pathfollowcritic---路径跟随评价)
7. [PathAlignCritic - 路径对齐评价](#7-pathaligncritic---路径对齐评价)
8. [PathAngleCritic - 路径角度评价](#8-pathanglecritic---路径角度评价)
9. [PreferForwardCritic - 偏好前进评价](#9-preferforwardcritic---偏好前进评价)
10. [TwirlingCritic - 旋转惩罚评价](#10-twirlingcritic---旋转惩罚评价)

---

## 1. ConstraintCritic - 约束评价

### 功能
惩罚违反机器人速度约束的轨迹，确保生成的轨迹在速度限制范围内。

### 判断条件详解

#### 1.1 启用检查
```cpp
if (!enabled_) {
    return;
}
```
**含义**: 检查该评价函数是否被启用  
**判断**: 如果 `enabled_ = false`，则跳过评分

#### 1.2 速度符号计算
```cpp
auto sgn = xt::where(data.state.vx > 0.0, 1.0, -1.0);
```
**含义**: 确定速度方向（前进/后退）  
**判断**:
- `vx > 0.0` → `sgn = 1.0`（前进）
- `vx ≤ 0.0` → `sgn = -1.0`（后退）

#### 1.3 总速度计算
```cpp
auto vel_total = sgn * xt::sqrt(data.state.vx * data.state.vx + data.state.vy * data.state.vy);
```
**含义**: 计算机器人的总速度（考虑方向）  
**计算**: $v_{total} = \text{sgn}(v_x) \cdot \sqrt{v_x^2 + v_y^2}$  
**注意**: 对于全向轮机器人，包含侧向速度 $v_y$

#### 1.4 最大速度约束违反
```cpp
auto out_of_max_bounds_motion = xt::maximum(vel_total - max_vel_, 0);
```
**含义**: 检查是否超过最大速度限制  
**判断**:
- `vel_total > max_vel_` → 违反量 = `vel_total - max_vel_`
- `vel_total ≤ max_vel_` → 违反量 = 0（无违反）

**示例**:
- 如果 `max_vel_ = 0.5 m/s`，`vel_total = 0.6 m/s`
- 则 `out_of_max_bounds_motion = 0.1 m/s`

#### 1.5 最小速度约束违反
```cpp
auto out_of_min_bounds_motion = xt::maximum(min_vel_ - vel_total, 0);
```
**含义**: 检查是否低于最小速度限制（针对后退）  
**判断**:
- `vel_total < min_vel_` → 违反量 = `min_vel_ - vel_total`
- `vel_total ≥ min_vel_` → 违反量 = 0（无违反）

**示例**:
- 如果 `min_vel_ = -0.35 m/s`（允许后退），`vel_total = -0.5 m/s`
- 则 `out_of_min_bounds_motion = 0.15 m/s`

#### 1.6 Ackermann 转向半径约束（仅阿克曼模型）
```cpp
auto acker = dynamic_cast<AckermannMotionModel *>(data.motion_model.get());
if (acker != nullptr) {
    auto out_of_turning_rad_motion = xt::maximum(
      acker->getMinTurningRadius() - (xt::fabs(vx) / xt::fabs(wz)), 0.0);
}
```
**含义**: 检查是否违反最小转弯半径约束  
**判断**:
- 实际转弯半径 $r = \frac{|v_x|}{|\omega_z|}$
- 如果 $r < r_{min}$ → 违反量 = $r_{min} - r$
- 如果 $r \geq r_{min}$ → 违反量 = 0

**物理意义**: 阿克曼转向车辆（如汽车）有最小转弯半径限制，不能原地旋转

#### 1.7 代价累加
```cpp
data.costs += xt::pow(
    xt::sum(
      (out_of_max_bounds_motion + 
       out_of_min_bounds_motion + 
       out_of_turning_rad_motion) * data.model_dt, {1}, immediate) * weight_, 
    power_);
```
**含义**: 计算总约束违反代价  
**计算过程**:
1. 对每个时间步的违反量求和并乘以时间步长 `dt`（积分）
2. 乘以权重 `weight_`（默认 4.0）
3. 取幂次 `power_`（默认 1）
4. 累加到总代价 `costs`

**物理意义**: 违反约束的时间越长，代价越高

---

## 2. CostCritic - 代价地图评价

### 功能
基于代价地图对轨迹进行评分，避免高代价区域（障碍物附近）。

### 判断条件详解

#### 2.1 启用检查
```cpp
if (!enabled_) {
    return;
}
```
**含义**: 检查该评价函数是否被启用

#### 2.2 动态足迹更新
```cpp
if (consider_footprint_) {
    possibly_inscribed_cost_ = findCircumscribedCost(costmap_ros_);
}
```
**含义**: 如果使用足迹碰撞检测，更新外接圆代价值  
**判断**: `consider_footprint_ = true` 时执行

#### 2.3 接近目标判断
```cpp
bool near_goal = false;
if (utils::withinPositionGoalTolerance(near_goal_distance_, data.state.pose.pose, data.path)) {
    near_goal = true;
}
```
**含义**: 判断机器人是否接近目标点  
**判断条件**: 到目标点的距离 < `near_goal_distance_`（默认 0.5m）  
**目的**: 接近目标时，不应用排斥代价，因为目标可能靠近障碍物

**示例**:
- 目标在墙角，如果一直排斥障碍物，机器人永远无法到达目标

#### 2.4 轨迹循环 - 遍历每条候选轨迹
```cpp
for (size_t i = 0; i < data.trajectories.x.shape(0); ++i) {
    bool trajectory_collide = false;
    // ...
}
```
**含义**: 对每条采样的候选轨迹进行评价

#### 2.5 自由空间检查
```cpp
pose_cost = costAtPose(traj.x(i, j), traj.y(i, j));
if (pose_cost < 1.0f) {continue;}  // In free space
```
**含义**: 检查该点是否在自由空间中  
**判断**:
- `pose_cost < 1.0` → 自由空间，跳过（无代价）
- `pose_cost ≥ 1.0` → 可能有障碍物或膨胀区域

#### 2.6 碰撞检查
```cpp
if (inCollision(pose_cost, traj.x(i, j), traj.y(i, j), traj.yaws(i, j))) {
    trajectory_collide = true;
    break;
}
```
**含义**: 检查该点是否与障碍物碰撞  
**判断**: 如果碰撞，标记轨迹为碰撞，跳出该轨迹的循环

#### 2.7 高危险区域判断
```cpp
using namespace nav2_costmap_2d; // NOLINT
if (pose_cost >= INSCRIBED_INFLATED_OBSTACLE) {
    repulsive_cost[i] += critical_cost_;
}
```
**含义**: 检查是否在铭刻膨胀障碍物区域  
**判断**:
- `INSCRIBED_INFLATED_OBSTACLE = 253`（机器人内切圆触及障碍物）
- 如果是，则施加严重惩罚 `critical_cost_`（默认 300.0）

**物理意义**: 这个区域非常接近障碍物，虽然不碰撞但极度危险

#### 2.8 一般排斥区域判断
```cpp
else if (!near_goal) {  // Generally prefer trajectories further from obstacles
    repulsive_cost[i] += pose_cost;
}
```
**含义**: 如果不在目标附近，偏好远离障碍物的轨迹  
**判断**:
- `!near_goal` → 不接近目标
- 累加代价地图值作为排斥代价

**为什么有 near_goal 判断**:
- 当接近目标时，目标可能在障碍物附近（如停车位、门口）
- 此时不应排斥障碍物，否则无法到达目标

#### 2.9 碰撞轨迹处理
```cpp
if (!trajectory_collide) {
    all_trajectories_collide = false;
} else {
    repulsive_cost[i] = collision_cost_;
}
```
**含义**: 标记碰撞轨迹  
**判断**:
- 如果轨迹碰撞，设置极高代价 `collision_cost_`（默认 1,000,000）
- 如果所有轨迹都碰撞，设置 `fail_flag = true`

#### 2.10 inCollision() 详细判断

```cpp
bool CostCritic::inCollision(float cost, float x, float y, float theta)
{
    bool is_tracking_unknown = 
        costmap_ros_->getLayeredCostmap()->isTrackingUnknown();
    
    // 如果启用足迹检测且代价高
    if (consider_footprint_ &&
        (cost >= possibly_inscribed_cost_ || possibly_inscribed_cost_ < 1.0f))
    {
        cost = static_cast<float>(collision_checker_.footprintCostAtPose(
            x, y, theta, costmap_ros_->getRobotFootprint()));
    }
    
    switch (static_cast<unsigned char>(cost)) {
        case (LETHAL_OBSTACLE):            // 254
            return true;
        case (INSCRIBED_INFLATED_OBSTACLE): // 253
            return consider_footprint_ ? false : true;
        case (NO_INFORMATION):              // 255
            return is_tracking_unknown ? false : true;
    }
    
    return false;
}
```

**各状态含义**:

| 代价值 | 常量名 | 含义 | 碰撞判断 |
|--------|--------|------|----------|
| 254 | `LETHAL_OBSTACLE` | 致命障碍物，确定碰撞 | ✅ 总是碰撞 |
| 253 | `INSCRIBED_INFLATED_OBSTACLE` | 铭刻膨胀障碍物 | ⚠️ 取决于 `consider_footprint_` |
| 255 | `NO_INFORMATION` | 未知区域 | ⚠️ 取决于 `is_tracking_unknown` |
| 0-252 | 自由或膨胀区域 | 安全或接近障碍物 | ❌ 不碰撞 |

**判断逻辑**:

1. **LETHAL_OBSTACLE (254)**:
   - 绝对碰撞，无论什么情况

2. **INSCRIBED_INFLATED_OBSTACLE (253)**:
   - 如果 `consider_footprint_ = true`: 
     - 先用足迹精确检测，可能不碰撞
   - 如果 `consider_footprint_ = false`:
     - 按圆形碰撞检测，认为碰撞

3. **NO_INFORMATION (255)**:
   - 如果 `is_tracking_unknown = true`:
     - 认为未知区域安全，不碰撞
   - 如果 `is_tracking_unknown = false`:
     - 认为未知区域危险，碰撞

#### 2.11 失败标志设置
```cpp
data.fail_flag = all_trajectories_collide;
```
**含义**: 如果所有轨迹都碰撞，优化失败  
**后续处理**: 触发 fallback 机制，重置并重试

---

## 3. GoalCritic - 目标位置评价

### 功能
引导机器人朝向目标位置移动。

### 判断条件详解

#### 3.1 启用和距离检查
```cpp
if (!enabled_ || !utils::withinPositionGoalTolerance(
      threshold_to_consider_, data.state.pose.pose, data.path))
{
    return;
}
```
**含义**: 仅当接近目标时才应用此评价函数  
**判断条件**:
- `enabled_ = false` → 跳过
- 到目标距离 > `threshold_to_consider_`（默认 1.4m）→ 跳过

**为什么只在接近目标时应用**:
- 远离目标时，应该跟随路径，而不是直接朝向目标
- 避免穿越障碍物直接奔向目标

#### 3.2 目标点获取
```cpp
const auto goal_idx = data.path.x.shape(0) - 1;
const auto goal_x = data.path.x(goal_idx);
const auto goal_y = data.path.y(goal_idx);
```
**含义**: 获取路径的最后一个点作为目标

#### 3.3 距离计算
```cpp
auto dists = xt::sqrt(
    xt::pow(traj_x - goal_x, 2) +
    xt::pow(traj_y - goal_y, 2));
```
**含义**: 计算轨迹上所有点到目标的欧几里得距离  
**公式**: $d = \sqrt{(x_{traj} - x_{goal})^2 + (y_{traj} - y_{goal})^2}$

#### 3.4 代价计算
```cpp
data.costs += xt::pow(xt::mean(dists, {1}, immediate) * weight_, power_);
```
**含义**: 取轨迹平均距离作为代价  
**计算**:
1. 对每条轨迹的所有时间步的距离求平均
2. 乘以权重 `weight_`（默认 5.0）
3. 取幂次 `power_`（默认 1）

**效果**: 偏好轨迹终点更接近目标的候选轨迹

---

## 4. GoalAngleCritic - 目标角度评价

### 功能
引导机器人以正确的朝向到达目标。

### 判断条件详解

#### 4.1 启用和距离检查
```cpp
if (!enabled_ || !utils::withinPositionGoalTolerance(
      threshold_to_consider_, data.state.pose.pose, data.path))
{
    return;
}
```
**含义**: 仅当非常接近目标时才应用此评价函数  
**判断条件**:
- 到目标距离 > `threshold_to_consider_`（默认 0.5m）→ 跳过

**为什么阈值更小**:
- 角度对齐仅在非常接近目标时重要
- 过早对齐角度可能导致不必要的旋转

#### 4.2 目标角度获取
```cpp
const auto goal_idx = data.path.x.shape(0) - 1;
const float goal_yaw = data.path.yaws(goal_idx);
```
**含义**: 获取路径终点的目标朝向

#### 4.3 角度差计算
```cpp
data.costs += xt::pow(
    xt::mean(xt::abs(utils::shortest_angular_distance(data.trajectories.yaws, goal_yaw)), {1}) *
    weight_, power_);
```
**含义**: 计算轨迹朝向与目标朝向的角度差  
**关键函数**: `shortest_angular_distance()` - 计算最短角度差（考虑 ±π 边界）

**示例**:
- 当前朝向: 175°
- 目标朝向: -175°（或 185°）
- 最短角度差: 10°（不是 350°）

**计算**:
1. 计算每个时间步的角度差
2. 取绝对值
3. 对轨迹求平均
4. 应用权重（默认 3.0）和幂次

---

## 5. ObstaclesCritic - 障碍物评价

### 功能
避免与障碍物碰撞，并偏好远离障碍物的轨迹。

### 判断条件详解

#### 5.1 启用检查
```cpp
if (!enabled_) {
    return;
}
```

#### 5.2 动态足迹更新
```cpp
if (consider_footprint_) {
    possibly_inscribed_cost_ = findCircumscribedCost(costmap_ros_);
}
```
**含义**: 如果使用足迹碰撞检测，更新外接圆代价阈值

#### 5.3 接近目标判断
```cpp
bool near_goal = false;
if (utils::withinPositionGoalTolerance(near_goal_distance_, data.state.pose.pose, data.path)) {
    near_goal = true;
}
```
**含义**: 判断是否接近目标（默认 0.5m）  
**目的**: 接近目标时放松排斥力，允许靠近障碍物

#### 5.4 自由空间跳过
```cpp
pose_cost = costAtPose(traj.x(i, j), traj.y(i, j), traj.yaws(i, j));
if (pose_cost.cost < 1.0f) {continue;}  // In free space
```
**含义**: 完全自由的空间无需评价

#### 5.5 碰撞检查
```cpp
if (inCollision(pose_cost.cost)) {
    trajectory_collide = true;
    break;
}
```
**含义**: 检测碰撞，如果碰撞则终止该轨迹评价

##### inCollision() 详细判断
```cpp
bool ObstaclesCritic::inCollision(float cost) const
{
    bool is_tracking_unknown =
        costmap_ros_->getLayeredCostmap()->isTrackingUnknown();
    
    switch (static_cast<unsigned char>(cost)) {
        case (LETHAL_OBSTACLE):              // 254
            return true;
        case (INSCRIBED_INFLATED_OBSTACLE):  // 253
            return consider_footprint_ ? false : true;
        case (NO_INFORMATION):               // 255
            return is_tracking_unknown ? false : true;
    }
    
    return false;
}
```
**判断逻辑**:（与 CostCritic 类似）
- **254**: 致命障碍物，绝对碰撞
- **253**: 铭刻障碍物，取决于是否使用足迹
- **255**: 未知区域，取决于是否跟踪未知

#### 5.6 膨胀层参数检查
```cpp
if (inflation_radius_ == 0.0f || inflation_scale_factor_ == 0.0f) {
    continue;
}
```
**含义**: 检查膨胀层是否存在  
**判断**: 如果膨胀半径或缩放因子为 0，无法计算排斥力

#### 5.7 到障碍物距离计算
```cpp
const float dist_to_obj = distanceToObstacle(pose_cost);
```
**计算公式**:
```cpp
float dist_to_obj = (scale_factor * min_radius - log(cost) + log(253.0f)) / scale_factor;
if (!cost.using_footprint) {
    dist_to_obj -= min_radius;
}
```
**含义**: 从膨胀代价反推到障碍物的距离  
**原理**: 膨胀层使用指数衰减: $cost = 253 \cdot e^{-\lambda \cdot dist}$  
**推导**: $dist = \frac{-\ln(cost/253)}{\lambda}$

#### 5.8 碰撞边界判断
```cpp
if (dist_to_obj < collision_margin_distance_) {
    traj_cost += (collision_margin_distance_ - dist_to_obj);
}
```
**含义**: 检查是否在碰撞边界内  
**判断**:
- `dist_to_obj < collision_margin_distance_`（默认 0.1m）→ 非常危险
- 代价随距离减小线性增加

**物理意义**: 距离障碍物 10cm 以内，高度危险

#### 5.9 一般排斥力
```cpp
else if (!near_goal) {  // Generally prefer trajectories further from obstacles
    repulsive_cost[i] += (inflation_radius_ - dist_to_obj);
}
```
**含义**: 如果不接近目标，偏好远离障碍物  
**判断**:
- `!near_goal` → 不接近目标
- 代价与到障碍物距离成反比

**排斥力计算**:
- 在 `inflation_radius_` 内，距离越近，排斥力越大
- 超出 `inflation_radius_`，排斥力为 0

#### 5.10 碰撞轨迹处理
```cpp
if (!trajectory_collide) {
    all_trajectories_collide = false;
}
raw_cost[i] = trajectory_collide ? collision_cost_ : traj_cost;
```
**含义**: 碰撞轨迹赋予极高代价（默认 10,000）

#### 5.11 最终代价计算
```cpp
data.costs += xt::pow(
    (critical_weight_ * raw_cost) +
    (repulsion_weight_ * repulsive_cost / traj_len),
    power_);
data.fail_flag = all_trajectories_collide;
```
**含义**: 综合两种代价  
**计算**:
- **critical_weight** × raw_cost: 碰撞和高危险代价
- **repulsion_weight** × repulsive_cost / traj_len: 平均排斥代价
- 默认: critical_weight = 20.0, repulsion_weight = 1.5

---

## 6. PathFollowCritic - 路径跟随评价

### 功能
引导机器人跟随全局规划路径，终点尽可能接近路径上的前方点。

### 判断条件详解

#### 6.1 基本检查
```cpp
if (!enabled_ || data.path.x.shape(0) < 2 ||
    utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path))
{
    return;
}
```
**判断条件**:
1. `!enabled_` → 未启用，跳过
2. `path.x.shape(0) < 2` → 路径点少于 2 个，跳过
3. 到目标距离 < `threshold_to_consider_`（默认 1.4m）→ 接近目标，跳过

**为什么接近目标时跳过**:
- 接近目标时应该直接朝向目标，而不是跟随路径
- 由 GoalCritic 接管

#### 6.2 设置最远可达点
```cpp
utils::setPathFurthestPointIfNotSet(data);
```
**含义**: 计算机器人在当前预测时域内能到达路径上的最远点  
**目的**: 确定路径跟随的目标点位置

#### 6.3 设置路径点有效性
```cpp
utils::setPathCostsIfNotSet(data, costmap_ros_);
```
**含义**: 检查路径上每个点是否被障碍物占据  
**标记**: `data.path_pts_valid[i] = true/false`

#### 6.4 计算目标偏移点
```cpp
auto offseted_idx = std::min(
    *data.furthest_reached_path_point + offset_from_furthest_, path_size);
```
**含义**: 在最远可达点基础上再向前偏移  
**判断**:
- 偏移量 = `offset_from_furthest_`（默认 6 个路径点）
- 不超过路径末尾

**为什么偏移**:
- 引导机器人向前看得更远
- 避免只追逐当前最近点，提高跟踪性能

#### 6.5 跳过无效路径点
```cpp
bool valid = false;
while (!valid && offseted_idx < path_size - 1) {
    valid = (*data.path_pts_valid)[offseted_idx];
    if (!valid) {
        offseted_idx++;
    }
}
```
**含义**: 找到第一个有效的（未被障碍物占据的）路径点  
**判断**: `path_pts_valid[i] = false` → 被障碍物占据，跳过

**为什么跳过无效点**:
- 动态障碍物可能临时遮挡路径
- 避免引导机器人穿越障碍物

#### 6.6 目标点获取
```cpp
const auto path_x = data.path.x(offseted_idx);
const auto path_y = data.path.y(offseted_idx);
```
**含义**: 获取跟随的目标点坐标

#### 6.7 距离计算
```cpp
const auto last_x = xt::view(data.trajectories.x, xt::all(), -1);
const auto last_y = xt::view(data.trajectories.y, xt::all(), -1);

auto dists = xt::sqrt(
    xt::pow(last_x - path_x, 2) +
    xt::pow(last_y - path_y, 2));
```
**含义**: 计算每条轨迹的**终点**到目标路径点的距离  
**注意**: 只考虑轨迹终点（`-1`），不是整条轨迹

#### 6.8 代价累加
```cpp
data.costs += xt::pow(weight_ * std::move(dists), power_);
```
**含义**: 偏好终点接近路径点的轨迹  
**权重**: 默认 5.0

---

## 7. PathAlignCritic - 路径对齐评价

### 功能
确保整条轨迹（不仅终点）与参考路径对齐，提高跟踪精度。

### 判断条件详解

#### 7.1 接近目标检查
```cpp
if (!enabled_ ||
    utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path))
{
    return;
}
```
**判断**: 到目标 < `threshold_to_consider_`（默认 0.5m）→ 跳过

#### 7.2 最远点检查
```cpp
utils::setPathFurthestPointIfNotSet(data);
const size_t path_segments_count = *data.furthest_reached_path_point;
if (path_segments_count < offset_from_furthest_) {
    return;
}
```
**含义**: 确保有足够的路径段用于对齐  
**判断**: 如果可达路径点少于偏移量（默认 20），跳过

**为什么**:
- 刚启动时路径点太少，对齐无意义
- 需要有足够长的路径段来计算对齐

#### 7.3 路径占用率检查
```cpp
utils::setPathCostsIfNotSet(data, costmap_ros_);
const size_t closest_initial_path_point = utils::findPathTrajectoryInitialPoint(data);
unsigned int invalid_ctr = 0;
const float range = *data.furthest_reached_path_point - closest_initial_path_point;
for (size_t i = closest_initial_path_point; i < *data.furthest_reached_path_point; i++) {
    if (!(*data.path_pts_valid)[i]) {invalid_ctr++;}
    if (static_cast<float>(invalid_ctr) / range > max_path_occupancy_ratio_ && invalid_ctr > 2) {
        return;
    }
}
```
**含义**: 检查路径是否被障碍物大量占据  
**判断**:
- 计算无效路径点占比
- 如果占比 > `max_path_occupancy_ratio_`（默认 0.07，即 7%）且无效点 > 2
- 则跳过路径对齐

**为什么跳过**:
- 动态障碍物阻挡了路径
- 此时应该让障碍物评价函数主导，而不是强制对齐路径
- 避免机器人试图穿越障碍物去对齐路径

**示例**:
- 路径有 100 个点，其中 10 个被障碍物占据
- 占比 = 10% > 7% → 跳过对齐

#### 7.4 路径积分距离计算
```cpp
std::vector<float> path_integrated_distances(path_segments_count, 0.0f);
float dx = 0.0f, dy = 0.0f;
for (unsigned int i = 1; i != path_segments_count; i++) {
    dx = P_x(i) - P_x(i - 1);
    dy = P_y(i) - P_y(i - 1);
    float curr_dist = sqrtf(dx * dx + dy * dy);
    path_integrated_distances[i] = path_integrated_distances[i - 1] + curr_dist;
}
```
**含义**: 计算路径上每个点的累计行驶距离  
**目的**: 用于在路径上找到与轨迹对应的点

**示例**:
- 点 0: 距离 = 0m
- 点 1: 距离 = 0.1m
- 点 2: 距离 = 0.25m
- ...

#### 7.5 轨迹点采样
```cpp
for (size_t p = trajectory_point_step_; p < time_steps; p += trajectory_point_step_) {
```
**含义**: 不是每个时间步都评价，而是跳着采样  
**间隔**: `trajectory_point_step_`（默认 4）  
**目的**: 减少计算量

#### 7.6 轨迹积分距离计算
```cpp
dx = Tx - T_x(p - trajectory_point_step_);
dy = Ty - T_y(p - trajectory_point_step_);
traj_integrated_distance += sqrtf(dx * dx + dy * dy);
```
**含义**: 计算轨迹的累计行驶距离  
**目的**: 与路径积分距离匹配

#### 7.7 对应路径点查找
```cpp
path_pt = utils::findClosestPathPt(
    path_integrated_distances, traj_integrated_distance, path_pt);
```
**含义**: 根据行驶距离，在路径上找到对应的点  
**匹配原理**: 行驶相同距离的点应该是对应的

**示例**:
- 轨迹行驶了 0.5m
- 在路径上找到累计距离也约为 0.5m 的点

#### 7.8 路径点有效性检查
```cpp
if ((*data.path_pts_valid)[path_pt]) {
    // 计算对齐代价
}
```
**含义**: 只对齐到有效的（未被障碍物占据的）路径点  
**判断**: `path_pts_valid[path_pt] = false` → 跳过该点

#### 7.9 对齐距离计算
```cpp
dx = P_x(path_pt) - Tx;
dy = P_y(path_pt) - Ty;
if (use_path_orientations_) {
    dyaw = angles::shortest_angular_distance(P_yaw(path_pt), T_yaw(p));
    summed_path_dist += sqrtf(dx * dx + dy * dy + dyaw * dyaw);
} else {
    summed_path_dist += sqrtf(dx * dx + dy * dy);
}
```
**含义**: 计算轨迹点到对应路径点的距离  
**判断**: `use_path_orientations_ = true` → 也考虑角度差

**两种模式**:
1. **仅位置对齐**: 只考虑 x, y 差异
2. **位置+朝向对齐**: 同时考虑 x, y, yaw 差异

#### 7.10 平均代价计算
```cpp
if (num_samples > 0) {
    cost[t] = summed_path_dist / num_samples;
} else {
    cost[t] = 0.0f;
}
```
**含义**: 对所有采样点的偏差求平均  
**判断**: 如果没有有效采样点（都被障碍物占据），代价为 0

---

## 8. PathAngleCritic - 路径角度评价

### 功能
确保机器人朝向沿路径前进方向，避免倒退或侧身移动（非全向轮）。

### 判断条件详解

#### 8.1 初始化 - 倒退允许判断
```cpp
float vx_min;
getParentParam(vx_min, "vx_min", -0.35);
if (fabs(vx_min) < 1e-6) {  // zero
    reversing_allowed_ = false;
} else if (vx_min < 0.0) {   // reversing possible
    reversing_allowed_ = true;
}
```
**含义**: 根据最小速度判断是否允许倒退  
**判断**:
- `vx_min ≈ 0` → 不允许倒退
- `vx_min < 0` → 允许倒退

#### 8.2 前向偏好设置
```cpp
if (!reversing_allowed_) {
    forward_preference_ = true;
}
```
**含义**: 如果不允许倒退，强制前向偏好

#### 8.3 启用和目标接近检查
```cpp
if (!enabled_) {
    return;
}

if (utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path)) {
    return;
}
```
**判断**: 接近目标（< 0.5m）时跳过  
**原因**: 接近目标时可能需要原地旋转对齐角度

#### 8.4 目标点偏移
```cpp
utils::setPathFurthestPointIfNotSet(data);
auto offseted_idx = std::min(
    *data.furthest_reached_path_point + offset_from_furthest_, data.path.x.shape(0) - 1);

const float goal_x = xt::view(data.path.x, offseted_idx);
const float goal_y = xt::view(data.path.y, offseted_idx);
```
**含义**: 获取前方偏移点作为朝向目标  
**偏移量**: 默认 4 个路径点

#### 8.5 机器人与目标点角度检查
```cpp
if (utils::posePointAngle(
      data.state.pose.pose, goal_x, goal_y, forward_preference_) < max_angle_to_furthest_)
{
    return;
}
```
**含义**: 如果机器人已经大致朝向目标点，跳过评价  
**判断**:
- 角度差 < `max_angle_to_furthest_`（默认 1.2 rad ≈ 69°）→ 跳过
- 角度差 ≥ 1.2 rad → 需要评价

**为什么跳过**:
- 机器人已经在正确的大致方向
- 避免不必要的角度纠正

**posePointAngle()** 计算机器人朝向与到目标点向量的夹角

#### 8.6 目标方向计算
```cpp
auto yaws_between_points = xt::atan2(
    goal_y - data.trajectories.y,
    goal_x - data.trajectories.x);
```
**含义**: 计算轨迹上每个点指向目标的方向角  
**公式**: $\theta_{target} = \arctan2(\Delta y, \Delta x)$

#### 8.7 角度差计算
```cpp
auto yaws = xt::abs(utils::shortest_angular_distance(data.trajectories.yaws, yaws_between_points));
```
**含义**: 计算轨迹朝向与目标方向的角度差  
**使用**: 最短角度距离（考虑 ±π 边界）

#### 8.8 倒退修正（如果允许）
```cpp
if (reversing_allowed_ && !forward_preference_) {
    const auto yaws_between_points_corrected = xt::where(
      yaws < M_PI_2, yaws_between_points, utils::normalize_angles(yaws_between_points + M_PI));
    const auto corrected_yaws = xt::abs(
      utils::shortest_angular_distance(data.trajectories.yaws, yaws_between_points_corrected));
    data.costs += xt::pow(xt::mean(corrected_yaws, {1}, immediate) * weight_, power_);
}
```
**含义**: 如果允许倒退且无前向偏好，考虑倒车可能性  
**判断**:
- 如果角度差 < 90°，正向行驶更合理
- 如果角度差 ≥ 90°，倒车可能更合理（修正目标角度 +180°）

**示例**:
- 目标在正后方（180°）
- 正向: 需要转 180° → 代价高
- 倒车: 只需保持当前朝向后退 → 代价低

#### 8.9 前向模式代价
```cpp
else {
    data.costs += xt::pow(xt::mean(yaws, {1}, immediate) * weight_, power_);
}
```
**含义**: 如果不允许倒退或有前向偏好，直接惩罚角度差

---

## 9. PreferForwardCritic - 偏好前进评价

### 功能
惩罚后退行为，偏好前进运动。

### 判断条件详解

#### 9.1 启用和目标接近检查
```cpp
if (!enabled_ ||
    utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path))
{
    return;
}
```
**判断**: 接近目标（< 0.5m）时跳过  
**原因**: 接近目标时可能需要倒车精确对位

#### 9.2 后退运动识别
```cpp
auto backward_motion = xt::maximum(-data.state.vx, 0);
```
**含义**: 识别并提取后退速度  
**判断**:
- `vx < 0` → 后退，`backward_motion = -vx`（正值）
- `vx ≥ 0` → 前进，`backward_motion = 0`

**示例**:
- `vx = 0.5` → `backward_motion = max(-0.5, 0) = 0`（无惩罚）
- `vx = -0.3` → `backward_motion = max(0.3, 0) = 0.3`（有惩罚）

#### 9.3 代价累加
```cpp
data.costs += xt::pow(
    xt::sum(std::move(backward_motion) * data.model_dt, {1}, immediate) * weight_, power_);
```
**含义**: 累计后退时间作为代价  
**计算**:
1. 后退速度 × 时间步长 = 后退距离
2. 对整条轨迹求和 = 总后退距离
3. 应用权重（默认 5.0）

**效果**: 后退距离越长，代价越高

---

## 10. TwirlingCritic - 旋转惩罚评价

### 功能
惩罚过度旋转，偏好平稳的运动。

### 判断条件详解

#### 10.1 启用和目标到达检查
```cpp
if (!enabled_ ||
    utils::withinPositionGoalTolerance(data.goal_checker, data.state.pose.pose, data.path))
{
    return;
}
```
**判断**: 使用 goal_checker 判断是否已到达目标  
**注意**: 这里用的是完整的 goal checker，可能包含位置和角度容差

#### 10.2 角速度提取
```cpp
const auto wz = xt::abs(data.state.wz);
```
**含义**: 提取角速度的绝对值  
**原因**: 不区分左转还是右转，都惩罚

#### 10.3 代价计算
```cpp
data.costs += xt::pow(xt::mean(wz, {1}, immediate) * weight_, power_);
```
**含义**: 对整条轨迹的角速度求平均作为代价  
**权重**: 默认 10.0

**效果**:
- 抑制原地旋转
- 抑制过度摇摆
- 偏好平滑的弧线运动

---

## 总结表格：所有判断条件概览

| Critic | 主要判断条件 | 目的 | 关键参数 |
|--------|-------------|------|----------|
| **ConstraintCritic** | `vel_total > max_vel_`<br>`vel_total < min_vel_`<br>`r < min_turning_r` | 速度约束违反检测 | `vx_max`, `vx_min`, `min_turning_r` |
| **CostCritic** | `pose_cost < 1.0` (自由空间)<br>`pose_cost >= 253` (高危险)<br>`!near_goal` (排斥力) | 代价地图避障 | `critical_cost`, `near_goal_distance` |
| **GoalCritic** | `dist > threshold_to_consider` | 接近目标时引导 | `threshold_to_consider = 1.4m` |
| **GoalAngleCritic** | `dist > threshold_to_consider` | 非常接近时角度对齐 | `threshold_to_consider = 0.5m` |
| **ObstaclesCritic** | `pose_cost < 1.0` (自由)<br>`inCollision()` (碰撞)<br>`dist < collision_margin` (高危)<br>`!near_goal` (排斥) | 精确避障与排斥 | `collision_margin_distance`, `inflation_radius` |
| **PathFollowCritic** | `path.size < 2`<br>`dist < threshold_to_consider`<br>`!path_pts_valid[i]` | 终点跟随路径 | `offset_from_furthest`, `threshold_to_consider` |
| **PathAlignCritic** | `dist < threshold_to_consider`<br>`path_count < offset`<br>`invalid_ratio > max_ratio`<br>`path_pts_valid[i]` | 整体对齐路径 | `max_path_occupancy_ratio`, `trajectory_point_step` |
| **PathAngleCritic** | `dist < threshold_to_consider`<br>`angle < max_angle`<br>`reversing_allowed` | 朝向前进方向 | `max_angle_to_furthest`, `forward_preference` |
| **PreferForwardCritic** | `vx < 0` (后退)<br>`dist < threshold` | 偏好前进 | `threshold_to_consider = 0.5m` |
| **TwirlingCritic** | `goal_checker->isGoalReached()` | 减少旋转 | `weight = 10.0` |

---

## 常见判断条件组合的物理意义

### 1. near_goal 判断组合
多个 critic 都使用了 `near_goal` 判断：

```cpp
bool near_goal = utils::withinPositionGoalTolerance(near_goal_distance, pose, path);
```

**使用场景**:
- **CostCritic**: `if (!near_goal)` → 远离障碍物
- **ObstaclesCritic**: `if (!near_goal)` → 应用排斥力
- **PathFollowCritic**: `if (near_goal)` → 停止跟随路径，让 GoalCritic 接管
- **PathAngleCritic**: `if (near_goal)` → 停止角度约束，允许原地旋转

**为什么**:
- 目标可能在障碍物附近（停车位、门口）
- 接近目标时需要放松某些约束，允许更灵活的行为
- 不同 critics 分工明确，根据距离切换主导作用

### 2. path_pts_valid 检查
路径点有效性检查在多个 critic 中使用：

**检查流程**:
```cpp
utils::setPathCostsIfNotSet(data, costmap_ros_);
// 为每个路径点标记是否被障碍物占据

if ((*data.path_pts_valid)[path_pt]) {
    // 使用该路径点
}
```

**使用场景**:
- **PathFollowCritic**: 跳过被占据的目标点
- **PathAlignCritic**: 只对齐到有效路径点，且检查占用率

**目的**:
- 动态环境中障碍物可能临时遮挡路径
- 不应强制机器人穿越障碍物去对齐路径
- 让障碍物评价函数主导此时的决策

### 3. threshold_to_consider 的不同值
不同 critic 使用不同的距离阈值：

| Critic | threshold | 物理意义 |
|--------|-----------|----------|
| GoalCritic | 1.4m | 较远就开始考虑目标 |
| GoalAngleCritic | 0.5m | 很近才对齐角度 |
| PathFollowCritic | 1.4m | 较远就停止跟随路径 |
| PathAlignCritic | 0.5m | 很近才停止对齐 |
| PathAngleCritic | 0.5m | 很近才停止角度约束 |
| PreferForwardCritic | 0.5m | 很近才允许倒车 |

**设计原则**:
- 位置相关: 1.4m（提前准备）
- 角度相关: 0.5m（最后调整）
- 路径跟随: 1.4m（远距离跟踪）
- 精细控制: 0.5m（近距离精确）

---

## 调试技巧

### 1. 查看哪些条件被触发
在评价函数中添加日志：
```cpp
if (!enabled_) {
    RCLCPP_DEBUG(logger_, "Critic disabled");
    return;
}
if (near_goal) {
    RCLCPP_DEBUG(logger_, "Near goal, skipping repulsion");
}
```

### 2. 可视化判断条件
使用 RViz markers 可视化：
- 有效/无效路径点（绿色/红色）
- 最远可达点（蓝色球）
- 目标偏移点（黄色球）
- 碰撞轨迹（红色）vs 安全轨迹（绿色）

### 3. 参数调优顺序
1. 先调整阈值参数（`threshold_to_consider`）
2. 再调整权重（`weight`）
3. 最后调整幂次（`power`）

### 4. 常见问题诊断

**问题**: 机器人在目标附近徘徊  
**可能原因**: `near_goal_distance` 太大，过早停止路径跟随  
**解决**: 减小 `near_goal_distance`

**问题**: 机器人不避障  
**可能原因**: `ObstaclesCritic.weight` 太小  
**解决**: 增大 `critical_weight` 和 `repulsion_weight`

**问题**: 路径跟踪不精确  
**可能原因**: `PathAlignCritic` 被 `max_path_occupancy_ratio` 条件跳过  
**解决**: 增大 `max_path_occupancy_ratio`（如 0.15）

**问题**: 机器人过度旋转  
**可能原因**: `TwirlingCritic.weight` 太小  
**解决**: 增大 `TwirlingCritic.weight`（如 15.0）

---

**文档版本**: v1.0  
**最后更新**: 2026-01-13  
**作者**: MPPI Controller Analysis
