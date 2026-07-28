// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// T 型（梯形/恒加速度）速度规划器实现：前向加速 pass + 后向减速 pass。
#include "nav2_speed_planner/trapezoidal_speed_planner.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace nav2_speed_planner
{

void TrapezoidalSpeedPlanner::assignVelocityProfile(
  std::vector<TrajPointWork> & work, double curr_vel, double /*curr_accel*/,
  double cruise_limit)
{
  if (work.size() < 2) {
    return;
  }
  if (cruise_limit < 1e-3) {
    // 全部点被安全置零（起点即阻塞）：保持零速剖面。
    return;
  }

  // 硬停配置（end_speed < 1e-3，无 A4 蠕行区）：终点锚定为 0，
  // 后向 pass 从终点自动生成收敛坡道。
  if (end_speed_ < 1e-3) {
    work.back().speed_limit = std::min(work.back().speed_limit, 0.0);
  }

  const std::vector<double> arc_s = computeCumulativeArcLengths(work);

  // 顺序约束：减速 pass 必须在加速 pass 之前——前向把 v[i+1] 压到可达值时
  // 结果仍 ≥ v[i]，不会破坏已生成的减速坡道；反序则加速可行性无法保证。
  // 前后坡道相交即为峰值：短路径三角形剖面自动涌现。
  computeDecelPlan(work, arc_s);        // 后向：一切低速点的进入坡道
  stampStartVelocity(work, curr_vel);   // 起点状态戳（receding-horizon）
  computeAccelPlan(work, arc_s);        // 前向：起步 + 一切低速区出口的再加速坡道
  enforceTerminalReachability(work, arc_s);

  // 逐点填充 accel 字段（局部段加速度 (v[i+1]²−v[i]²)/(2·Δs)）：恒加速度剖面
  // 内部虽是分段常数，但此前一直留空 0——消费侧（⑤ 控制器速度环）想用它做
  // 前馈抵消纯反馈的斜坡跟踪滞后时，T 型（当前默认）读到的全是 0，前馈失效。
  // 末点沿用倒数第二段的值（无下一点可差分）。
  for (size_t i = 0; i + 1 < work.size(); ++i) {
    const double ds = arc_s[i + 1] - arc_s[i];
    if (ds > 1e-6) {
      const double v0 = work[i].speed_limit;
      const double v1 = work[i + 1].speed_limit;
      work[i].accel = (v1 * v1 - v0 * v0) / (2.0 * ds);
    }
  }
  if (work.size() >= 2) {
    work.back().accel = work[work.size() - 2].accel;
  }
}

void TrapezoidalSpeedPlanner::enforceTerminalReachability(
  std::vector<TrajPointWork> & work, const std::vector<double> & arc_s) const
{
  if (decel_ >= -1e-6 || stop_distance_ <= 0.0 || end_speed_ < 1e-3) {
    return;
  }
  for (size_t i = 0; i + 1 < work.size(); ++i) {
    if (arc_s.back() - arc_s[i + 1] > stop_distance_ + 1e-6 ||
      work[i + 1].speed_limit < 1e-3)
    {
      continue;
    }
    const double ds = arc_s[i + 1] - arc_s[i];
    if (ds <= 1e-9) {
      continue;
    }
    const double v_prev = std::max(0.0, work[i].speed_limit);
    const double reachable_sq = v_prev * v_prev + 2.0 * decel_ * ds;
    const double v_min_reachable = reachable_sq > 0.0 ? std::sqrt(reachable_sq) : 0.0;
    work[i + 1].speed_limit = std::max(work[i + 1].speed_limit, v_min_reachable);
  }
}

// ============================ 减速规划（后向 pass）============================

void TrapezoidalSpeedPlanner::computeDecelPlan(
  std::vector<TrajPointWork> & work, const std::vector<double> & arc_s) const
{
  // v[i] = min(v[i], √(v[i+1]² + 2·|decel|·Δs))，从终点向前传播。
  // 对一切低速点（每个曲率 cap / 蠕行区 / 障碍零速 / 硬停终点）自动生成
  // 物理正确的进入坡道。index 0 为机器人当前速度（状态戳），不参与设置。
  for (size_t i = work.size() - 1; i-- > 1; ) {
    const double ds = arc_s[i + 1] - arc_s[i];
    const double v_next = work[i + 1].speed_limit;
    const double v_reachable = std::sqrt(v_next * v_next - 2.0 * decel_ * ds);
    if (work[i].speed_limit > v_reachable) {
      work[i].speed_limit = v_reachable;
    }
  }
}

// ============================ 起点状态戳 ============================

void TrapezoidalSpeedPlanner::stampStartVelocity(
  std::vector<TrajPointWork> & work, double curr_vel) const
{
  // 安全置零绝对优先：起点被置零（阻塞）时不得抬升。
  if (work[0].speed_limit >= 1e-3) {
    work[0].speed_limit = curr_vel;
  }
}

// ============================ 加速规划（前向 pass）============================

void TrapezoidalSpeedPlanner::computeAccelPlan(
  std::vector<TrajPointWork> & work, const std::vector<double> & arc_s) const
{
  // v[i+1] = min(v[i+1], √(v[i]² + 2·accel·Δs))，从起点状态戳向后传播。
  // 从当前速度起坡（receding-horizon，每周期从实际速度重算），并对一切
  // 低速区出口（弯道出口/障碍豁免区边界等）自动生成再加速坡道。
  for (size_t i = 0; i + 1 < work.size(); ++i) {
    const double ds = arc_s[i + 1] - arc_s[i];
    const double v_prev = work[i].speed_limit;
    const double v_reachable = std::sqrt(v_prev * v_prev + 2.0 * accel_ * ds);
    if (work[i + 1].speed_limit > v_reachable) {
      work[i + 1].speed_limit = v_reachable;
    }
  }
}

}  // namespace nav2_speed_planner

PLUGINLIB_EXPORT_CLASS(
  nav2_speed_planner::TrapezoidalSpeedPlanner, nav2_core::SpeedPlanner)
