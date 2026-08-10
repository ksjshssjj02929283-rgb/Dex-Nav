// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#ifndef NAV2_SPEED_PLANNER__BACKENDS__REACHABILITY_SPEED_PLANNER_HPP_
#define NAV2_SPEED_PLANNER__BACKENDS__REACHABILITY_SPEED_PLANNER_HPP_

#include <string>
#include <vector>

#include "nav2_speed_planner/speed_planner_base.hpp"

namespace nav2_speed_planner
{

/**
 * @class ReachabilitySpeedPlanner
 * @brief 纯可达性滤波后端：后向减速 → 前向加速 → 两遍均带 jerk 斜率限制。
 *
 * 与「硬路点 + 位置模式求解」后端的根本差别在于**提的问题不同**：
 *
 *   硬路点：「从 (s0,v0,a0) 出发，**恰好**在 s1 达到 v1 且 a=0，怎么走？」
 *           —— 这是求解，可行域可能为空；点间距离由路径几何给定、不可调，
 *              而每个路点施加位置+速度+零加速度三个约束，一维剖面被过定。
 *              无解时只能掉兜底，而兜底会污染整条剖面。
 *
 *   本后端：「在每一点，最快能多快而不违反任何约束？」
 *           —— 每一步都是 `min`，故：
 *              · 结果恒 ≤ 包络（不可能违反约束）
 *              · 结果恒可从前一点达到（不可能不可执行）
 *              · 单调收缩、无求解分支 → **不存在失败模式**，也就不需要兜底
 *
 * 速度包络的语义本就是**上界** `v(s) ≤ env(s)`。把谷底钉成等式
 * `v(s_i) = env(s_i)` 是把不等式当等式，可行域从一片区域塌成一个点——
 * 于是它可以是空的。本后端不做这个转换。
 *
 * 减速坡道与加速坡道是滤波**自然产生**的，无需在包络侧手工加宽弯前弯后。
 *
 * 代价：不是时间最优，比硬路点求解**更保守**（到点时间略长）；
 *       jerk 为斜率限制而非 jerk 最优（后者需在包络下解 QP）。
 */
class ReachabilitySpeedPlanner : public SpeedPlannerBase
{
public:
  ReachabilitySpeedPlanner() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

protected:
  void generateProfile(
    std::vector<TrajPointWork> & work, double curr_vel, double curr_accel,
    double cruise_limit, bool terminal_brake_latched) override;

protected:
  /// jerk 限制下的单向可达性积分——**前向与后向共用同一算子**。
  ///
  /// 制动是加速的时间反演：「从 v_hi 在 D 内减到 v_lo」与「从 v_lo 在 D 内
  /// 加到 v_hi」是同一个问题（限值取 |decel|）。故只写一遍，`forward` 决定
  /// 遍历方向，其余完全一致——包括那条决定 S 形的收手项。
  ///
  /// 状态是 (v, a)：只传播 v 而事后差分出 a，等于 jerk 根本不是任何环节的
  /// 变量，无从限制。四个转折方向（起步加速、巡航收油、制动建立、制动释放）
  /// 必须都由这个带 a 状态的积分覆盖。
  ///
  /// ⚠ **两遍是串行复合（后向就地改写包络，前向再读改写后的结果），这是刻意的。**
  ///   试过改成「各自独立跑在原始包络上 + 取 min 合并」（Autoware 的结构），
  ///   实测全面变差：straight 0.85 → 10.73、nonuniform_3mm 8.90 → 18.73、
  ///   obs_after_curve 1.07 → 13.64。原因是前向收手项里的 `a_cap`（对齐后向
  ///   曲线的局部斜率）**本身就是一个隐式 merge**——它让 a 在余量趋零时平滑
  ///   收敛到后向曲线的斜率。拆成独立两遍再取 min，等于删掉这个 merge 而没有
  ///   补上替代品，每个交汇处都退化成 C0 折角。详见 decisions.md。
  void integrateReachable(
    std::vector<TrajPointWork> & work, const std::vector<double> & arc_s,
    bool forward, double a_limit, double v_seed, double a_seed,
    ConstraintSource source) const;

  /// 后向：保证每点都减得到下游约束，且**提前踩刹**——减速度从 0 线性建立。
  /// 种子为终点包络值、加速度 0（到达时制动已完全释放）。
  void filterBackward(
    std::vector<TrajPointWork> & work, const std::vector<double> & arc_s) const;

  /// 前向：保证每点都能从上一点加速上来，且**提前收手**。
  /// 种子为实测初速与初始加速度：计划每周期重算，若每次从零起坡，
  /// 控制器采样到的永远是坡道起点，机器人升不上速度。
  void filterForward(
    std::vector<TrajPointWork> & work, const std::vector<double> & arc_s,
    double v_seed, double a_seed) const;

  /// 向下可达：把剖面抬到「物理上也只能减到这么慢」，使包络要求的不可能减速
  /// 不会变成输出里的违规加速度。**减速度本身也受 jerk 限制**——它同样要建立，
  /// 不能瞬间到 `decel`；`a_seed` 是当前实测/继承加速度，制动从它开始往负向爬。
  void enforceDecelFeasibility(
    std::vector<TrajPointWork> & work, const std::vector<double> & arc_s,
    double a_seed) const;

  /// 由最终速度剖面逐段回填加速度与相对时间。
  void fillProfileDerivatives(
    std::vector<TrajPointWork> & work, const std::vector<double> & arc_s) const;

  double max_jerk_{0.5};

  /// 控制周期。**必须与 `controller_frequency` 一致。**
  ///
  /// 用途只有一个但很关键：剖面首点的加速度是"此刻执行的那一个"，它每个控制
  /// 周期都被下一版计划替换，故 seed → 首点这一次变化只有一个控制周期去完成。
  /// 内部点之间用段通过时长 `Δs/v` 是对的（机器人确实要走那么久），但首点若也
  /// 用它，静止起步时 `Δs/v → ∞` 会让 jerk 约束形同虚设。
  double control_dt_{0.1};
};

}  // namespace nav2_speed_planner

#endif  // NAV2_SPEED_PLANNER__BACKENDS__REACHABILITY_SPEED_PLANNER_HPP_
