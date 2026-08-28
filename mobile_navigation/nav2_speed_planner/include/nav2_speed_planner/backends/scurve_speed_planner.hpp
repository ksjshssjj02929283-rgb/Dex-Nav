// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// S 型（jerk 限制速度曲线）规划器 pluginlib 插件 —— 基于 Ruckig。
//
// 算法：用 Ruckig（在线 jerk 限制轨迹生成器，MIT 许可 community 版）以弧长 s 为 1-DOF，
// **目标驱动分段规划**：把包络的每个"下降沿谷底"（蠕行区界/弯心/障碍缓冲带起点）
// 作为一次硬目标 (target_position, target_velocity)——"恰好在谷底处降到谷速"由
// Ruckig 时间最优规划保证，提前减速自然产生。段内 max_velocity 取区间包络最大值。
// 步进采样 (s,v,a,t) 后按路径点弧长重采样。jerk/加速度由 Ruckig 保证不超限；
// 非对称加减速用 min_acceleration=-d_max。顺带填 acceleration/relative_time。
//
// 为什么不用 max_velocity 做提前减速：Ruckig 对"当前速度已超 max_velocity"的状态
// 没有强制减速语义（只温和收加速度不刹车）——低速区必须用 target 硬约束。
#ifndef NAV2_SPEED_PLANNER__BACKENDS__SCURVE_SPEED_PLANNER_HPP_
#define NAV2_SPEED_PLANNER__BACKENDS__SCURVE_SPEED_PLANNER_HPP_

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
   * 目标驱动分段（包络下降沿谷底为硬目标）+ 重采样；
   * jerk≤max_jerk、加速度∈[-d_max, a_max] 由 Ruckig 保证。
   *
   * @param v_env   逐点速度包络上界（>=0），含曲率/障碍约束（障碍处=0，作为停止点）
   * @param arc_s   逐点累计弧长（单调增，arc_s[0]=0）
   * @param v_start 起点当前速度（前向运动，<0 视作 0）
   * @param a_start 起点当前加速度（会钳位到配置的加减速度能力）
   * @param a_max   最大加速度 (>0)
   * @param d_max   最大减速度幅值 (>0)
   * @param max_jerk jerk 上限 (>0)
   * @param ctrl_dt 在线步进控制周期 (s)，建议取控制器频率倒数
   * @param v_out   输出逐点速度
   * @param a_out   输出逐点加速度（Ruckig 输出，连续、有界）
   * @param t_out   输出逐点相对时间（单调增）
   * @param v_end   路径终点目标速度：仅当停止点为路径终点时生效；
   *                停止点为障碍（零包络点）时内部强制 0
   * @param terminal_speed_cap low-speed-zone cap; distinct from the terminal target v_end
   * @param terminal_brake_latched true only after ControllerServer commits to final-goal braking;
   *                ordinary negative acceleration must remain free to recover and re-accelerate
   */
  static FeasibilityResult generateRuckigProfile(
    const std::vector<double> & v_env,
    const std::vector<double> & arc_s,
    double v_start,
    double a_start,
    double a_max, double d_max,
    double max_jerk, double ctrl_dt,
    std::vector<double> & v_out,
    std::vector<double> & a_out,
    std::vector<double> & t_out,
    double v_end = 0.0,
    bool terminal_brake_latched = false,
    double terminal_speed_cap = 0.0);

protected:
  /// Phase B：基类 Phase A 包络（含末端蠕行区）→ Ruckig jerk 限制剖面 → 回填
  /// work（速度/加速度/相对时间）。
  void generateProfile(
    std::vector<TrajPointWork> & work, double curr_vel, double curr_accel,
    double cruise_limit, bool terminal_brake_latched) override;

  // —— 参数 ——
  double max_jerk_{0.5};    ///< jerk 上限 (m/s^3)
  // TERMINAL_SPATIAL_STAGES_BEGIN
  // 高速制动锁存只负责在固定蠕行入口达到0.05；最终0速由独立空间锁存控制。
  bool final_stop_latched_{false};
  // 仅冻结由terminal_execution_delay换算出的命令提前距离，不含额外安全距离。
  double latched_creep_delay_distance_{-1.0};
  // TERMINAL_SPATIAL_STAGES_END
  double control_dt_{0.05};  ///< Ruckig 在线步进控制周期 (s)
  // 仅用于0.05→0最终停车距离，不再前移蠕行入口或叠加到障碍距离。
  double terminal_execution_delay_{0.0};
};

}  // namespace nav2_speed_planner

#endif  // NAV2_SPEED_PLANNER__BACKENDS__SCURVE_SPEED_PLANNER_HPP_
