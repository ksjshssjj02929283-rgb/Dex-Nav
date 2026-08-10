// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// A4 末端约束：末端速度边界点插入 + 末端蠕行区。
// terminal_creep_distance_ 是**蠕行区长度**，不是停车距离——减速到 end_speed 在该区之前完成。
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

void SpeedPlannerBase::insertEndSpeedBoundary(std::vector<TrajPointWork> & path) const
{
  const double v_creep = std::max(end_speed_, 0.0);
  if (path.size() < 2 || terminal_creep_distance_ <= 0.0 || v_creep < 1e-3) {
    return;
  }

  const std::vector<double> arc_s = computeCumulativeArcLengths(path);
  const double total_s = arc_s.back();
  const double boundary_s = total_s - terminal_creep_distance_;
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


VelocityConstraint SpeedPlannerBase::produceTerminalConstraint(
  const std::vector<TrajPointWork> & path) const
{
  // 只约束未来点：首点是当前实测状态，不能被蠕行区改写。
  VelocityConstraint c(path.size());
  const double v_creep = std::max(end_speed_, 0.0);
  if (path.size() < 2 || terminal_creep_distance_ <= 0.0 || v_creep < 1e-3) {
    return c;
  }
  const std::vector<double> arc_s = computeCumulativeArcLengths(path);
  for (size_t i = path.size(); i-- > 1; ) {
    if (arc_s.back() - arc_s[i] > terminal_creep_distance_ + 1e-6) {
      break;
    }
    c.set(i, v_creep, ConstraintSource::kTerminalCreep);
    c.region_type[i] = "stop_zone";   // 落入蠕行区即标注，与是否压低无关
  }
  return c;
}

// ============================ 共享工具 ============================


}  // namespace nav2_speed_planner
