// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// S 型（jerk 限制速度曲线）规划器实现 —— 基于 Ruckig。
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
// 包络"下降沿谷底"目标点：每个下降的谷底 (index, v) 作为一次 Ruckig 硬目标——
// target_position/target_velocity 语义保证"恰好在谷底处降到谷速"（提前减速由
// 时间最优规划自然产生）。不能用 max_velocity 做提前减速：Ruckig 对已超限的
// 当前速度没有强制减速语义（只温和收加速度，不刹车）。
struct SpeedTarget
{
  size_t index;
  double velocity;
};

std::vector<SpeedTarget> extractDescentTargets(
  const std::vector<double> & env, size_t stop_i)
{
  constexpr double kTol = 1e-3;
  std::vector<SpeedTarget> targets;
  size_t i = 1;
  while (i < stop_i) {
    if (env[i] < env[i - 1] - kTol) {
      size_t bottom = i;
      while (bottom + 1 <= stop_i && env[bottom + 1] < env[bottom] - 1e-9) {
        ++bottom;
      }
      if (bottom < stop_i) {
        targets.push_back({bottom, std::max(env[bottom], 0.0)});
      }
      i = bottom + 1;
    } else {
      ++i;
    }
  }
  return targets;
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
  double a_start,
  double a_max, double d_max,
  double max_jerk, double ctrl_dt,
  std::vector<double> & v_out,
  std::vector<double> & a_out,
  std::vector<double> & t_out,
  double v_end)
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
  a_start = std::clamp(a_start, -d_max, a_max);
  if (n == 1) {
    v_out[0] = std::clamp(v_start, 0.0, std::max(0.0, v_env[0]));
    a_out[0] = a_start;
    return;
  }

  // 停止点：首个零包络点（障碍）否则终点。障碍停止时终点速度强制 0。
  size_t stop_i = n - 1;
  bool stop_at_env_zero = false;
  for (size_t i = 0; i < n; ++i) {
    if (v_env[i] < 1e-3) {stop_i = i; stop_at_env_zero = true; break;}
  }
  v_end = stop_at_env_zero ? 0.0 : std::max(0.0, v_end);
  const double target = arc_s[stop_i];

  // 目标序列 = 包络下降沿谷底（蠕行区界/弯心/缓冲带起点）+ 最终停止点。
  // 逐目标分段规划："在谷底恰好降到谷速"由 target 硬保证；段内 max_velocity 取
  // 区间包络最大值（渐变段轻微越界由重采样钳位收尾），并抬到 ≥ 当前/目标速度
  // 保证输入合法。
  std::vector<SpeedTarget> targets = extractDescentTargets(v_env, stop_i);
  targets.push_back({stop_i, v_end});

  ruckig::Ruckig<1> otg(ctrl_dt);
  ruckig::InputParameter<1> in;
  ruckig::OutputParameter<1> out;
  in.current_position = {0.0};
  in.current_velocity = {std::clamp(v_start, 0.0, std::max(0.0, v_env[0]))};
  in.current_acceleration = {a_start};
  in.max_acceleration = {a_max};
  in.max_jerk = {max_jerk};
  in.min_acceleration = std::array<double, 1>{-d_max};   // 非对称：减速幅值 ≤ d_max

  std::vector<double> rs{0.0}, rv{in.current_velocity[0]}, ra{a_start}, rt{0.0};
  double tnow = 0.0;
  bool failed = false;
  for (const auto & tgt : targets) {
    const double s_t = arc_s[tgt.index];
    if (s_t <= in.current_position[0] + 1e-9) {
      continue;
    }
    double seg_vmax = 1e-3;
    for (size_t i = 0; i < n; ++i) {
      if (arc_s[i] >= in.current_position[0] - 1e-9 && arc_s[i] <= s_t + 1e-9) {
        seg_vmax = std::max(seg_vmax, v_env[i]);
      }
    }
    seg_vmax = std::max({seg_vmax, in.current_velocity[0], tgt.velocity, 1e-3});
    in.max_velocity = {seg_vmax};
    in.target_position = {s_t};
    in.target_velocity = {std::max(tgt.velocity, 0.0)};
    in.target_acceleration = {0.0};

    const double segment_distance = s_t - in.current_position[0];
    // Bound iterations by a conservative travel, acceleration, and jerk time
    // for this segment so an infeasible target cannot consume the global cap.
    const double min_accel_limit = std::min(a_max, d_max);
    const double duration_budget =
      2.0 + 2.0 * segment_distance / std::max(seg_vmax, 1e-3) +
      4.0 * seg_vmax / min_accel_limit + 4.0 * std::sqrt(seg_vmax / max_jerk);
    if (!std::isfinite(duration_budget) || duration_budget <= 0.0) {
      failed = true;
      break;
    }
    constexpr double kMaxRuckigSteps = 200000.0;
    int segment_steps_left = std::max(
      1, static_cast<int>(std::min(
        std::ceil(duration_budget / ctrl_dt), kMaxRuckigSteps)));
    bool segment_finished = false;
    while (segment_steps_left-- > 0) {
      const ruckig::Result res = otg.update(in, out);
      if (res != ruckig::Result::Working && res != ruckig::Result::Finished) {
        failed = true;
        break;
      }
      if (!std::isfinite(out.new_position[0]) || !std::isfinite(out.new_velocity[0]) ||
        !std::isfinite(out.new_acceleration[0]))
      {
        failed = true;
        break;
      }
      tnow += ctrl_dt;
      rs.push_back(out.new_position[0]);
      rv.push_back(out.new_velocity[0]);
      ra.push_back(out.new_acceleration[0]);
      rt.push_back(tnow);
      out.pass_to_input(in);
      if (res == ruckig::Result::Finished) {
        segment_finished = true;
        break;
      }
    }
    if (failed || !segment_finished) {
      failed = true;
      break;
    }
  }

  // 重采样到路径点弧长；停止点之后置终点速度（障碍停止时 v_end 已强制 0）；
  // 最终对包络限幅消除重采样残差。
  size_t k = 0;
  for (size_t i = 0; i < n; ++i) {
    if (arc_s[i] >= target - 1e-9) {
      v_out[i] = v_end; a_out[i] = 0.0; t_out[i] = rt.back();
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

// ===================== 插件 hook：Phase B（Ruckig jerk 限制剖面）=====================
void SCurveSpeedPlanner::assignVelocityProfile(
  std::vector<TrajPointWork> & work, double curr_vel, double curr_accel,
  double /*cruise_limit*/)
{
  const size_t n = work.size();
  if (n < 2) {
    return;
  }

  // 1) 累计弧长 + 包络（Phase A 已由基类完成：障碍/巡航/曲率/末端蠕行区，
  //    包络零点含障碍缓冲带，蠕行区已压到 end_speed）。
  std::vector<double> v_env(n, 0.0), arc_s(n, 0.0);
  v_env[0] = std::max(0.0, work[0].speed_limit);
  for (size_t i = 1; i < n; ++i) {
    const double dx = work[i].pose.pose.position.x - work[i - 1].pose.pose.position.x;
    const double dy = work[i].pose.pose.position.y - work[i - 1].pose.pose.position.y;
    arc_s[i] = arc_s[i - 1] + std::hypot(dx, dy);
    v_env[i] = std::max(0.0, work[i].speed_limit);
  }

  // 2) Ruckig 生成 jerk 限制剖面（非对称 a_max / d_max=|decel_|）。
  //    终点目标速度 = end_speed_（障碍停止时函数内部强制 0）。
  std::vector<double> v_out, a_out, t_out;
  generateRuckigProfile(
    v_env, arc_s, curr_vel, curr_accel,
    accel_, std::fabs(decel_),
    max_jerk_, control_dt_,
    v_out, a_out, t_out, end_speed_);

  // 3) 回填 work（曲率已由 Phase A3 填好）。
  for (size_t i = 0; i < n; ++i) {
    work[i].speed_limit = v_out[i];
    work[i].accel = a_out[i];
    work[i].rel_time = t_out[i];
  }
}

}  // namespace nav2_speed_planner

PLUGINLIB_EXPORT_CLASS(
  nav2_speed_planner::SCurveSpeedPlanner, nav2_core::SpeedPlanner)
