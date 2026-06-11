// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// S 型（jerk 限制速度曲线）规划器实现 —— 基于 Ruckig（项目 doc 8 §十一）。
#include "nav2_speed_planner/scurve_speed_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "ruckig/ruckig.hpp"

namespace nav2_speed_planner
{

namespace
{
// 前瞻 vmax：当前到刹车距离前瞻窗内的 v_env 最小值（让 Ruckig 接近弯/障碍时提前减速）。
double lookaheadVmax(
  const std::vector<double> & env, const std::vector<double> & s,
  double s_now, double v_now, double d_max)
{
  // 刹车距离（恒减速估计）+ jerk 余量系数 1.5 + 固定余量 0.1 m。
  const double brake = v_now * v_now / (2.0 * std::max(1e-3, d_max)) * 1.5 + 0.1;
  const double s_end = std::min(s_now + brake, s.back());
  double vmin = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < s.size(); ++i) {
    if (s[i] >= s_now - 1e-6 && s[i] <= s_end + 1e-6) {vmin = std::min(vmin, env[i]);}
  }
  if (!std::isfinite(vmin)) {vmin = env.back();}
  return vmin;
}
}  // namespace

void SCurveSpeedPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  // 先复用 base configure（声明全部恒加速度参数 + 取参 + 缓存 node/tf/costmap）。
  SpeedPlannerBase::configure(parent, name, tf, costmap_ros);

  auto node = parent.lock();
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_jerk", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".control_dt", rclcpp::ParameterValue(0.05));
  node->get_parameter(plugin_name_ + ".max_jerk", max_jerk_);
  node->get_parameter(plugin_name_ + ".control_dt", control_dt_);
}

// ===================== Ruckig 沿弧长生成 jerk 限制速度剖面 =====================
void SCurveSpeedPlanner::generateRuckigProfile(
  const std::vector<double> & v_env,
  const std::vector<double> & arc_s,
  double v_start,
  double a_max, double d_max,
  double max_jerk, double ctrl_dt,
  std::vector<double> & v_out,
  std::vector<double> & a_out,
  std::vector<double> & t_out)
{
  const size_t n = v_env.size();
  v_out.assign(n, 0.0);
  a_out.assign(n, 0.0);
  t_out.assign(n, 0.0);
  if (n == 0) {return;}
  a_max = std::max(1e-3, a_max);
  d_max = std::max(1e-3, d_max);
  max_jerk = std::max(1e-3, max_jerk);
  ctrl_dt = std::max(1e-3, ctrl_dt);
  if (n == 1) {
    v_out[0] = std::clamp(v_start, 0.0, std::max(0.0, v_env[0]));
    return;
  }

  // 停止点：首个零包络点（障碍）否则终点。
  size_t stop_i = n - 1;
  for (size_t i = 0; i < n; ++i) {
    if (v_env[i] < 1e-3) {stop_i = i; break;}
  }
  const double target = arc_s[stop_i];

  ruckig::Ruckig<1> otg(ctrl_dt);
  ruckig::InputParameter<1> in;
  ruckig::OutputParameter<1> out;
  in.current_position = {0.0};
  in.current_velocity = {std::clamp(v_start, 0.0, std::max(0.0, v_env[0]))};
  in.current_acceleration = {0.0};
  in.target_position = {target};
  in.target_velocity = {0.0};
  in.target_acceleration = {0.0};
  in.max_acceleration = {a_max};
  in.max_jerk = {max_jerk};
  in.min_acceleration = std::array<double, 1>{-d_max};   // 非对称：减速幅值 ≤ d_max

  // 在线循环步进，记录 (s,v,a,t)。
  std::vector<double> rs{0.0}, rv{in.current_velocity[0]}, ra{0.0}, rt{0.0};
  double tnow = 0.0;
  const int cap = 200000;
  for (int step = 0; step < cap; ++step) {
    const double s_now = in.current_position[0];
    const double v_now = in.current_velocity[0];
    const double vmax = std::max(lookaheadVmax(v_env, arc_s, s_now, v_now, d_max), 1e-3);
    in.max_velocity = {vmax};
    const ruckig::Result res = otg.update(in, out);
    if (res == ruckig::Result::Error || res == ruckig::Result::ErrorInvalidInput) {break;}
    tnow += ctrl_dt;
    rs.push_back(out.new_position[0]);
    rv.push_back(out.new_velocity[0]);
    ra.push_back(out.new_acceleration[0]);
    rt.push_back(tnow);
    out.pass_to_input(in);
    if (res == ruckig::Result::Finished) {break;}
    if (out.new_position[0] >= target - 1e-3 && out.new_velocity[0] < 1e-3) {break;}
  }

  // 重采样到路径点弧长；停止点之后置 0；最终对包络限幅消除重采样残差。
  size_t k = 0;
  for (size_t i = 0; i < n; ++i) {
    if (arc_s[i] >= target - 1e-9) {
      v_out[i] = 0.0; a_out[i] = 0.0; t_out[i] = rt.back();
      continue;
    }
    while (k + 1 < rs.size() && rs[k + 1] < arc_s[i]) {++k;}
    if (k + 1 >= rs.size()) {
      v_out[i] = rv.back(); a_out[i] = ra.back(); t_out[i] = rt.back();
    } else {
      const double denom = rs[k + 1] - rs[k];
      const double f = (denom > 1e-9) ? std::clamp((arc_s[i] - rs[k]) / denom, 0.0, 1.0) : 0.0;
      v_out[i] = rv[k] + f * (rv[k + 1] - rv[k]);
      a_out[i] = ra[k] + f * (ra[k + 1] - ra[k]);
      t_out[i] = rt[k] + f * (rt[k + 1] - rt[k]);
    }
    v_out[i] = std::min(std::max(0.0, v_out[i]), std::max(0.0, v_env[i]));
  }
}

// ===================== 插件 hook：复用包络 + Ruckig + 回填 =====================
void SCurveSpeedPlanner::assignVelocityProfile(
  std::vector<TrajPointWork> & work, double curr_vel)
{
  const size_t n = work.size();
  if (n == 0) {return;}
  if (n == 1) {
    work[0].speed_limit = std::clamp(curr_vel, 0.0, max_speed_);
    work[0].accel = 0.0;
    work[0].rel_time = 0.0;
    return;
  }

  // 1) 复用基类构建逐点速度包络（cruise + 障碍置零 + 曲率限速；base 已设 points_interval_）。
  const double cruise = getSpeedLimit(work, curr_vel);
  setPlanSpeedLimit(work, cruise);
  applyCurvatureConstraint(work, cruise);

  // 2) 累计弧长 + 包络。
  std::vector<double> v_env(n, 0.0), arc_s(n, 0.0);
  v_env[0] = std::max(0.0, work[0].speed_limit);
  for (size_t i = 1; i < n; ++i) {
    const double dx = work[i].pose.pose.position.x - work[i - 1].pose.pose.position.x;
    const double dy = work[i].pose.pose.position.y - work[i - 1].pose.pose.position.y;
    arc_s[i] = arc_s[i - 1] + std::hypot(dx, dy);
    v_env[i] = std::max(0.0, work[i].speed_limit);
  }

  // 3) Ruckig 生成 jerk 限制剖面（非对称 a_max / d_max=|decel_|）。
  std::vector<double> v_out, a_out, t_out;
  generateRuckigProfile(
    v_env, arc_s, curr_vel,
    accel_, std::fabs(decel_),
    max_jerk_, control_dt_,
    v_out, a_out, t_out);

  // 4) 回填 work（曲率已由 applyCurvatureConstraint 填好）。
  for (size_t i = 0; i < n; ++i) {
    work[i].speed_limit = v_out[i];
    work[i].accel = a_out[i];
    work[i].rel_time = t_out[i];
  }
}

}  // namespace nav2_speed_planner

PLUGINLIB_EXPORT_CLASS(
  nav2_speed_planner::SCurveSpeedPlanner, nav2_core::SpeedPlanner)
