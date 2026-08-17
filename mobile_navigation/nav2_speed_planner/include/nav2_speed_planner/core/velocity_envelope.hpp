// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// 速度包络：逐点上界 + 归因，以及唯一的合并入口 applyConstraint()。
#ifndef NAV2_SPEED_PLANNER__CORE__VELOCITY_ENVELOPE_HPP_
#define NAV2_SPEED_PLANNER__CORE__VELOCITY_ENVELOPE_HPP_

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_speed_planner
{

/**
 * @enum ConstraintSource
 * @brief 把某一点的速度上界压到当前值的那条约束。
 *
 * 速度包络是多条约束逐点取严的结果，此前"谁压低了这个点"只能靠通读全部
 * 约束代码反推。本枚举随 `lowerBound()` 写入，使归因成为可读字段。
 *
 * 语义是**当前生效者**（最后一次成功下压的约束），区别于 `region_type`——
 * 后者表达该点最先写入的几何/业务语义，不保证一定下压速度。两者在同一点可能
 * 不同：例如 MPPI 绕障模式仍保留 `obstacle_safety_margin` 标注，但不产生障碍零速
 * 上界；弯道与末端蠕行区重叠时，实际生效者也可能是后者。
 */
enum class ConstraintSource
{
  kNone = 0,        ///< 未被任何约束压低（= max_speed）
  kMaxSpeed,        ///< 全局最大速度
  kSpeedHint,       ///< 路径逐点限速（区域限速的数据通道）
  kExternalLimit,   ///< 外部 /speed_limit
  kObstacle,        ///< 障碍点本身（置零）
  kObstacleSafetyMargin,  ///< 硬障碍前的目标安全余量区（停障模式下置零）
  kObstacleStopMargin,  ///< 为控制延迟和停车过冲额外预留的置零区
  kLateralAccel,    ///< 曲率 / 横向加速度限速
  kTerminalCreep,   ///< 末端蠕行区
  kEndBoundary,     ///< 末端速度边界点
  kKinematicBrake,  ///< 后向可达性：为减到下游约束而必须的限速
  kKinematicAccel,  ///< 前向可达性：从上游加不上来而形成的限速
  kCount,           ///< 仅用于约束来源统计数组长度
};

/**
 * @struct TrajPointWork
 * @brief 内部工作结构：{pose, speed_limit, curvature} + S 型扩展字段 + 语义透传字段。
 */
struct TrajPointWork
{
  geometry_msgs::msg::PoseStamped pose;
  double velocity_upper_bound{0.0};
  double curvature{0.0};
  /// 当前 speed_limit 由哪条约束决定。仅归因，不参与任何判据。
  ConstraintSource constraint_source{ConstraintSource::kNone};

  // —— 时空剖面扩展字段（恒加速度剖面默认 0，jerk 限制剖面回填）——
  double accel{0.0};
  double rel_time{0.0};
  /// 加速度对时间的变化率。各后端生成 accel/rel_time 后统一由
  /// `fillJerkFromAccel()` 回填——见该函数注释：此前该量根本不存在，
  /// 导致校验中的 jerk 断言恒真。
  double jerk{0.0};

  // —— 透传字段（来自带语义的 path；nav_msgs/Path 无此字段时置默认）——
  float speed_hint{0.0f};
  std::string region_type;
  std::string segment_id;
};

/// 一条约束对整条路径的产出：逐点上界 + 归因 + 逐点语义标注。
///
/// 约束**不再就地修改共享数组**——它们返回本结构，由唯一的聚合点
/// `applyConstraint()` 合并。这样：
///   · 每条约束可单独实例化、单独断言，不必先跑完前面几条
///   · 包络的每一次下压都经同一处代码，归因不会漏写
///   · 约束之间的先后依赖必须出现在函数签名里（如横向加速度约束需要巡航基准），
///     无法再靠调用顺序隐式传递
///
/// `region_type` 逐点表达而非整条统一：各约束对"何时标注"的规则并不相同
///（弯道主段仅在确实压低时标注，弯前弯后延伸段只要落入范围就标注），
/// 由产出方自行决定，聚合点不做推断。
struct VelocityConstraint
{
  static constexpr double kUnconstrained = std::numeric_limits<double>::max();

  std::vector<double> upper_bound;         ///< kUnconstrained = 本约束不约束该点
  std::vector<std::string> region_type;    ///< 空串 = 不标注该点
  std::vector<ConstraintSource> source;    ///< 逐点归因（同一约束内也可不同，如障碍点与停车余量）

  explicit VelocityConstraint(size_t n = 0)
  : upper_bound(n, kUnconstrained), region_type(n),
    source(n, ConstraintSource::kNone) {}

  /// 便捷：设置某点的上界与归因。
  void set(size_t i, double ub, ConstraintSource src)
  {
    upper_bound[i] = ub;
    source[i] = src;
  }
};

/// 逐点下压速度上界并记录归因。
///
/// 等价于 `p.velocity_upper_bound = std::min(p.velocity_upper_bound, value)`——`std::min(x, y)`
/// 返回 `y < x ? y : x`，与此处的 `value < p.velocity_upper_bound` 判据在非 NaN 与
/// NaN 两种情形下行为均一致，故替换**不改变任何数值**。
inline bool lowerBound(TrajPointWork & p, double value, ConstraintSource source)
{
  if (value < p.velocity_upper_bound) {
    p.velocity_upper_bound = value;
    p.constraint_source = source;
    return true;
  }
  return false;
}

/// 唯一聚合点：把一条约束合并进包络。包络的每一次变化都经由此处。
inline void applyConstraint(
  std::vector<TrajPointWork> & path, const VelocityConstraint & c)
{
  const size_t n = std::min(path.size(), c.upper_bound.size());
  for (size_t i = 0; i < n; ++i) {
    if (c.upper_bound[i] < VelocityConstraint::kUnconstrained) {
      lowerBound(path[i], c.upper_bound[i], c.source[i]);
    }
    if (!c.region_type[i].empty() && path[i].region_type.empty()) {
      path[i].region_type = c.region_type[i];
    }
  }
}

}  // namespace nav2_speed_planner

#endif  // NAV2_SPEED_PLANNER__CORE__VELOCITY_ENVELOPE_HPP_
