// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// SpeedPlannerBase 各编译单元共享的内部工具。
// 仅供 src/ 内部使用，不对外暴露。

#ifndef CONSTRAINTS__PLANNER_INTERNAL_HPP_
#define CONSTRAINTS__PLANNER_INTERNAL_HPP_

#include <algorithm>
#include <cmath>

#include "geometry_msgs/msg/quaternion.hpp"
#include "nav2_speed_planner/speed_planner_base.hpp"

namespace nav2_speed_planner
{
namespace internal
{

constexpr double kDuplicateDistanceSq = 1e-8;
constexpr double kGeometryEpsilon = 1e-9;

inline bool orientationsEquivalent(
  const geometry_msgs::msg::Quaternion & lhs,
  const geometry_msgs::msg::Quaternion & rhs)
{
  const double direct_distance_sq =
    std::pow(lhs.x - rhs.x, 2) + std::pow(lhs.y - rhs.y, 2) +
    std::pow(lhs.z - rhs.z, 2) + std::pow(lhs.w - rhs.w, 2);
  const double negated_distance_sq =
    std::pow(lhs.x + rhs.x, 2) + std::pow(lhs.y + rhs.y, 2) +
    std::pow(lhs.z + rhs.z, 2) + std::pow(lhs.w + rhs.w, 2);
  return std::min(direct_distance_sq, negated_distance_sq) <= kGeometryEpsilon;
}

inline TrajPointWork interpolatePoint(
  const TrajPointWork & from, const TrajPointWork & to, double ratio)
{
  ratio = std::clamp(ratio, 0.0, 1.0);
  TrajPointWork out = from;
  out.pose.pose.position.x =
    from.pose.pose.position.x + (to.pose.pose.position.x - from.pose.pose.position.x) * ratio;
  out.pose.pose.position.y =
    from.pose.pose.position.y + (to.pose.pose.position.y - from.pose.pose.position.y) * ratio;
  out.pose.pose.position.z =
    from.pose.pose.position.z + (to.pose.pose.position.z - from.pose.pose.position.z) * ratio;
  out.pose.pose.orientation = ratio < 0.5 ? from.pose.pose.orientation : to.pose.pose.orientation;
  out.velocity_upper_bound = std::min(from.velocity_upper_bound, to.velocity_upper_bound);
  // 插值点继承较小一侧的归因（与 speed_limit 取严口径一致）
  out.constraint_source = (to.velocity_upper_bound < from.velocity_upper_bound) ?
    to.constraint_source : from.constraint_source;
  if (from.speed_hint > 0.0f && to.speed_hint > 0.0f) {
    out.speed_hint = std::min(from.speed_hint, to.speed_hint);
  } else {
    out.speed_hint = std::max(from.speed_hint, to.speed_hint);
  }
  if (from.region_type != to.region_type) {
    out.region_type.clear();
  }
  if (from.segment_id != to.segment_id) {
    out.segment_id.clear();
  }
  return out;
}

}  // namespace internal
}  // namespace nav2_speed_planner

#endif  // CONSTRAINTS__PLANNER_INTERNAL_HPP_
