// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// S 型（jerk 限制速度曲线）规划器 pluginlib 插件 —— 基于 Ruckig。
//
// 算法：用 Ruckig（在线 jerk 限制轨迹生成器，MIT 许可 community 版）以弧长 s 为 1-DOF，
// 在线循环步进：每步把 max_velocity 设为「当前到刹车距离前瞻窗内的 v_env 最小值」
// （接近弯/障碍时 vmax 提前下降 → Ruckig 自然 jerk 限制减速，解决 lookahead），
// 步进得 (s,v,a)，再按路径点弧长重采样。jerk/加速度由 Ruckig 保证不超限；非对称
// 加减速用 min_acceleration=-d_max 实现。顺带填 acceleration/relative_time。
//
// 背景：简单距离域两遍 min 法无法严格限 jerk（巡航入口需前瞻），故采用 Ruckig 在线生成。
#ifndef NAV2_SPEED_PLANNER__SCURVE_SPEED_PLANNER_HPP_
#define NAV2_SPEED_PLANNER__SCURVE_SPEED_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_speed_planner/speed_planner_base.hpp"

namespace nav2_speed_planner
{

/**
 * @class SCurveSpeedPlanner
 * @brief S 型（jerk 限制）速度规划器插件：复用基类包络，Ruckig 生成 jerk 限制速度剖面。
 */
class SCurveSpeedPlanner : public SpeedPlannerBase
{
public:
  SCurveSpeedPlanner() = default;
  ~SCurveSpeedPlanner() override = default;

  // 先调 base configure 再 declare 自己的 jerk / 控制周期参数。
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief 用 Ruckig 沿弧长生成 jerk 限制速度剖面（纯算法，仅依赖 ruckig，便于单测）。
   *
   * 在线循环 + 前瞻 vmax + 重采样；jerk≤max_jerk、加速度∈[-d_max, a_max] 由 Ruckig 保证。
   *
   * @param v_env   逐点速度包络上界（>=0），含曲率/障碍约束（障碍处=0，作为停止点）
   * @param arc_s   逐点累计弧长（单调增，arc_s[0]=0）
   * @param v_start 起点当前速度（前向运动，<0 视作 0）
   * @param a_max   最大加速度 (>0)
   * @param d_max   最大减速度幅值 (>0)
   * @param max_jerk jerk 上限 (>0)
   * @param ctrl_dt 在线步进控制周期 (s)，建议取控制器频率倒数
   * @param v_out   输出逐点速度
   * @param a_out   输出逐点加速度（Ruckig 输出，连续、有界）
   * @param t_out   输出逐点相对时间（单调增）
   * @param v_end   路径终点目标速度：仅当停止点为路径终点时生效；
   *                停止点为障碍（零包络点）时内部强制 0
   */
  static void generateRuckigProfile(
    const std::vector<double> & v_env,
    const std::vector<double> & arc_s,
    double v_start,
    double a_max, double d_max,
    double max_jerk, double ctrl_dt,
    std::vector<double> & v_out,
    std::vector<double> & a_out,
    std::vector<double> & t_out,
    double v_end = 0.0);

protected:
  /// Phase B：基类 Phase A 包络（含末端蠕行区）→ Ruckig jerk 限制剖面 → 回填
  /// work（速度/加速度/相对时间）。
  void assignVelocityProfile(
    std::vector<TrajPointWork> & work, double curr_vel, double cruise_limit) override;

  // —— 参数 ——
  double max_jerk_{0.5};    ///< jerk 上限 (m/s^3)
  double control_dt_{0.05}; ///< Ruckig 在线步进控制周期 (s)
};

}  // namespace nav2_speed_planner

#endif  // NAV2_SPEED_PLANNER__SCURVE_SPEED_PLANNER_HPP_
