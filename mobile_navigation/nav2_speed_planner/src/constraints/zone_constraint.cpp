// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// 区域限速的数据通道。基类恒返回 0（不约束）；带语义的路径类型落地后由派生类覆写。
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

float SpeedPlannerBase::pointSpeedHint(
  const nav_msgs::msg::Path & /*path*/, size_t /*index*/) const
{
  return 0.0f;   // 基类无逐点提示；P0-2 接入带语义 path 后由子类覆写
}


}  // namespace nav2_speed_planner
