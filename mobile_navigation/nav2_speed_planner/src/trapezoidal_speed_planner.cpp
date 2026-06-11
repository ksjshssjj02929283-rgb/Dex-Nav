// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// T 型（梯形速度）规划器 pluginlib 插件实现。
// 速度剖面直接复用 base 的恒加速度参考内核（三段式 + 曲率/障碍约束）。
#include "nav2_speed_planner/trapezoidal_speed_planner.hpp"

#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace nav2_speed_planner
{

void TrapezoidalSpeedPlanner::assignVelocityProfile(
  std::vector<TrajPointWork> & work, double curr_vel)
{
  // T 型 = 恒加速度参考内核。
  assignConstantAccelProfile(work, curr_vel);
}

}  // namespace nav2_speed_planner

PLUGINLIB_EXPORT_CLASS(
  nav2_speed_planner::TrapezoidalSpeedPlanner, nav2_core::SpeedPlanner)
