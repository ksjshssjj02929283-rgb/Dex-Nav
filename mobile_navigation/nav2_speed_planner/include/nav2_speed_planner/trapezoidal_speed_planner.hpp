// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// T 型（梯形/恒加速度）速度规划器 pluginlib 插件（源自 dcm tcurve）。
//
// Phase B 剖面（Phase A 包络由基类完成）：
//   可达性回退：加减速所需距离超过有效路径长度时，按当前状态解三角形剖面峰值速度；
//   B1 加速段：从当前速度按恒加速度推进（receding-horizon，每周期从实际速度重算）；
//   B2 减速段：末端减速（锚定距终点 stop_distance 处降到 end_speed，蠕行区包络由 A4 保证）
//            + 曲率前减速（向第一个小半径点）+ 零速点前减速过渡（障碍/硬停）。
//   全部写入只降不升（保留包络更严约束）。
#ifndef NAV2_SPEED_PLANNER__TRAPEZOIDAL_SPEED_PLANNER_HPP_
#define NAV2_SPEED_PLANNER__TRAPEZOIDAL_SPEED_PLANNER_HPP_

#include <vector>

#include "nav2_speed_planner/speed_planner_base.hpp"

namespace nav2_speed_planner
{

/**
 * @class TrapezoidalSpeedPlanner
 * @brief T 型速度规划器插件：恒加速度前向/后向剖面 + 短路径三角形回退。
 */
class TrapezoidalSpeedPlanner : public SpeedPlannerBase
{
public:
  TrapezoidalSpeedPlanner() = default;
  ~TrapezoidalSpeedPlanner() override = default;

protected:
  /// Phase B：可达性回退 + 加速段 + 减速段（见文件头说明）。
  void assignVelocityProfile(
    std::vector<TrajPointWork> & work, double curr_vel, double cruise_limit) override;

  /// B1 加速段：按当前速度 + 恒加速度推进（只降不升；起点安全零不抬升）。
  void computeAccelPlan(
    std::vector<TrajPointWork> & path, const double curr_vel, const double speed_limit);

  /// B2 减速段：末端 end_speed 减速 + 曲率前减速 + 零速点前减速过渡（只降不升）。
  void computeDecelPlan(
    std::vector<TrajPointWork> & path, const double curr_vel, const double speed_limit);

  /// 加速所需距离（curr_vel→speed_limit，恒加速度）。
  double computeAccelLength(const double curr_vel, const double speed_limit) const;

  /// 减速所需距离（curr_vel→min_speed，恒减速度，decel_ < 0）。
  double computeDecelLength(const double curr_vel, const double min_speed) const;

  /// 有效路径长度（遇第一个零速点截断，含该点）。
  static double computeValidSpeedPathLength(const std::vector<TrajPointWork> & path);
};

}  // namespace nav2_speed_planner

#endif  // NAV2_SPEED_PLANNER__TRAPEZOIDAL_SPEED_PLANNER_HPP_
