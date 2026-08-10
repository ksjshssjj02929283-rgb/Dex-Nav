// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// A2 巡航约束：求解理想巡航速度并施加全局上界。
// 注意 computeCruiseLimit 读取的是**障碍约束之后**的包络（跳过已置零的点），
// 这个顺序依赖是真实存在的，不是可以随意重排的。
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

double SpeedPlannerBase::computeProfileCruiseTarget(const std::vector<TrajPointWork> & path) const
{
  // A2 理想巡航上界（纯查询）：非零速点限速的最小值；安全零速点不参与求解。
  double speed_limit = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path.size(); i++) {
    if (path[i].velocity_upper_bound < 1e-3) {
      continue;
    }
    if (speed_limit > path[i].velocity_upper_bound) {
      speed_limit = path[i].velocity_upper_bound;
    }
  }
  if (!(speed_limit < std::numeric_limits<double>::max())) {
    // 全部点被安全置零（起点即阻塞）。
    return 0.0;
  }
  return std::clamp(speed_limit, min_speed_, max_speed_);
}




}  // namespace nav2_speed_planner
