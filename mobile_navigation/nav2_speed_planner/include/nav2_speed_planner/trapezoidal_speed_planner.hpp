// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// T 型（梯形速度）规划器 pluginlib 插件。
// 仅 override assignVelocityProfile()，体内直接调用 base 的恒加速度参考内核
// assignConstantAccelProfile()（三段式：加速/匀速/减速 + 曲率/障碍约束）。
#ifndef NAV2_SPEED_PLANNER__TRAPEZOIDAL_SPEED_PLANNER_HPP_
#define NAV2_SPEED_PLANNER__TRAPEZOIDAL_SPEED_PLANNER_HPP_

#include <vector>

#include "nav2_speed_planner/speed_planner_base.hpp"

namespace nav2_speed_planner
{

/**
 * @class TrapezoidalSpeedPlanner
 * @brief T 型速度规划器：恒加速度三段式（dcm tcurve 移植内核）。
 */
class TrapezoidalSpeedPlanner : public SpeedPlannerBase
{
public:
  TrapezoidalSpeedPlanner() = default;
  ~TrapezoidalSpeedPlanner() override = default;

protected:
  /// T 型剖面 = 恒加速度参考内核（直接复用 base 实现）。
  void assignVelocityProfile(
    std::vector<TrajPointWork> & work, double curr_vel) override;
};

}  // namespace nav2_speed_planner

#endif  // NAV2_SPEED_PLANNER__TRAPEZOIDAL_SPEED_PLANNER_HPP_
