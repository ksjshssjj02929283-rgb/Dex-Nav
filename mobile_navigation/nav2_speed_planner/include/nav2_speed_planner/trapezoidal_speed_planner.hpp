// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// T 型（梯形/恒加速度）速度规划器 pluginlib 插件。
//
// Phase B 剖面（Phase A 包络由基类完成）——后向/前向双传播 pass：
//   B1 后向减速 pass：v[i] = min(v[i], √(v[i+1]² + 2·|decel|·Δs)) 从终点向前传播，
//     对一切低速点（曲率 cap/蠕行区/障碍零速/硬停终点）生成进入坡道；
//   B2 前向加速 pass：v[i+1] = min(v[i+1], √(v[i]² + 2·accel·Δs)) 从当前速度向后传播
//     （receding-horizon，每周期从实际速度重算），对一切低速区出口生成再加速坡道。
//   前后坡道相交即峰值（短路径三角形自动涌现）；顺序必须后向在前、前向在后。
//   全部写入只降不升（保留包络更严约束）；index 0 为当前速度状态戳，不参与设置。
#ifndef NAV2_SPEED_PLANNER__TRAPEZOIDAL_SPEED_PLANNER_HPP_
#define NAV2_SPEED_PLANNER__TRAPEZOIDAL_SPEED_PLANNER_HPP_

#include <vector>

#include "nav2_speed_planner/speed_planner_base.hpp"

namespace nav2_speed_planner
{

/**
 * @class TrapezoidalSpeedPlanner
 * @brief T 型速度规划器插件：恒加速度后向减速 + 前向加速双传播 pass。
 */
class TrapezoidalSpeedPlanner : public SpeedPlannerBase
{
public:
  TrapezoidalSpeedPlanner() = default;
  ~TrapezoidalSpeedPlanner() override = default;

protected:
  /// Phase B：减速规划 → 起点状态戳 → 加速规划（见文件头说明）。
  void assignVelocityProfile(
    std::vector<TrajPointWork> & work, double curr_vel, double cruise_limit) override;

  /// 减速规划（后向 pass）：为一切低速点生成进入坡道（min 写入，index 0 不参与）。
  void computeDecelPlan(
    std::vector<TrajPointWork> & work, const std::vector<double> & arc_s) const;

  /// 起点状态戳：写入当前速度（安全置零不抬升）。
  void stampStartVelocity(std::vector<TrajPointWork> & work, double curr_vel) const;

  /// 加速规划（前向 pass）：起步 + 一切低速区出口的再加速坡道（min 写入）。
  void computeAccelPlan(
    std::vector<TrajPointWork> & work, const std::vector<double> & arc_s) const;
};

}  // namespace nav2_speed_planner

#endif  // NAV2_SPEED_PLANNER__TRAPEZOIDAL_SPEED_PLANNER_HPP_
