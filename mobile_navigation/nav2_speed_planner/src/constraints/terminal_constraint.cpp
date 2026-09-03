// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// A4 末端约束：稳定低速交接区。terminal_speed_cap_ 是区内上限，
// end_speed_ 是S曲线交给末端PID的稳定速度目标；最终零速由位置环完成。
//
// 本文件是 SpeedPlannerBase 的一部分（同类跨编译单元），签名与成员访问
// 与拆分前完全一致。
#include "nav2_speed_planner/speed_planner_base.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"

#include "constraints/planner_internal.hpp"

using namespace nav2_speed_planner::internal;  // NOLINT(build/namespaces)

namespace nav2_speed_planner
{

namespace
{

void insertBoundaryBeforeGoal(
  std::vector<TrajPointWork> & path, double distance_before_goal)
{
  if (path.size() < 2 || distance_before_goal <= 0.0) {
    return;
  }

  std::vector<double> arc_s(path.size(), 0.0);
  for (size_t index = 1; index < path.size(); ++index) {
    const auto & previous = path[index - 1].pose.pose.position;
    const auto & current = path[index].pose.pose.position;
    arc_s[index] = arc_s[index - 1] +
      std::hypot(current.x - previous.x, current.y - previous.y);
  }
  const double total_s = arc_s.back();
  const double boundary_s = total_s - distance_before_goal;
  if (boundary_s <= kGeometryEpsilon || boundary_s >= total_s - kGeometryEpsilon) {
    return;
  }

  constexpr double kBoundaryTolerance = 1e-4;
  auto upper = std::lower_bound(arc_s.begin(), arc_s.end(), boundary_s);
  if (upper != arc_s.end() && std::fabs(*upper - boundary_s) <= kBoundaryTolerance) {
    return;
  }
  if (upper != arc_s.begin() && std::fabs(*(upper - 1) - boundary_s) <= kBoundaryTolerance) {
    return;
  }

  const size_t upper_index = static_cast<size_t>(std::distance(arc_s.begin(), upper));
  if (upper_index == 0 || upper_index >= path.size()) {
    return;
  }
  const double segment_length = arc_s[upper_index] - arc_s[upper_index - 1];
  if (segment_length <= kGeometryEpsilon) {
    return;
  }
  const double ratio = (boundary_s - arc_s[upper_index - 1]) / segment_length;
  path.insert(
    path.begin() + static_cast<std::ptrdiff_t>(upper_index),
    interpolatePoint(path[upper_index - 1], path[upper_index], ratio));
}

}  // namespace

void SpeedPlannerBase::insertEndSpeedBoundary(std::vector<TrajPointWork> & path) const
{
  const double speed_cap = std::max(terminal_speed_cap_, 0.0);
  if (terminal_creep_distance_ <= 0.0 || speed_cap < 1e-3) {
    return;
  }
  insertBoundaryBeforeGoal(path, terminal_creep_distance_);
}

void SpeedPlannerBase::insertApproachSpeedBoundary(std::vector<TrajPointWork> & path) const
{
  if (approach_entry_distance_ <= 0.0 || approach_entry_speed_ <= 0.0) {
    return;
  }
  insertBoundaryBeforeGoal(path, approach_entry_distance_);
}

VelocityConstraint SpeedPlannerBase::produceApproachConstraint(
  const std::vector<TrajPointWork> & path) const
{
  VelocityConstraint constraint(path.size());
  if (path.size() < 2 || approach_entry_distance_ <= 0.0 || approach_entry_speed_ <= 0.0) {
    return constraint;
  }
  const std::vector<double> arc_s = computeCumulativeArcLengths(path);
  for (size_t index = path.size(); index-- > 1; ) {
    if (arc_s.back() - arc_s[index] > approach_entry_distance_ + 1e-6) {
      break;
    }
    constraint.set(
      index, approach_entry_speed_, ConstraintSource::kApproachHandoff);
    constraint.region_type[index] = "approach_handoff";
  }
  return constraint;
}


VelocityConstraint SpeedPlannerBase::produceTerminalConstraint(
  const std::vector<TrajPointWork> & path) const
{
  // 只约束未来点：首点是当前实测状态，不能被蠕行区改写。
  VelocityConstraint c(path.size());
  const double speed_cap = std::max(terminal_speed_cap_, 0.0);
  if (path.size() < 2) {
    return c;
  }
  if (terminal_creep_distance_ > 0.0 && speed_cap >= 1e-3) {
    const std::vector<double> arc_s = computeCumulativeArcLengths(path);
    for (size_t i = path.size(); i-- > 1; ) {
      if (arc_s.back() - arc_s[i] > terminal_creep_distance_ + 1e-6) {
        break;
      }
      c.set(i, speed_cap, ConstraintSource::kTerminalCreep);
      c.region_type[i] = "stop_zone";
    }
  }
  c.set(path.size() - 1, std::max(0.0, end_speed_), ConstraintSource::kEndBoundary);
  c.region_type[path.size() - 1] = "terminal_handoff";
  return c;
}

// ============================ 共享工具 ============================


}  // namespace nav2_speed_planner
