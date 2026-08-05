// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// S 型（jerk 限制速度曲线）规划器实现 —— 基于 Ruckig。
#include "nav2_speed_planner/scurve_speed_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <utility>
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

bool generateRuckigVelocityProfile(
  double v_start, double a_start, double a_max, double d_max,
  double max_jerk, double ctrl_dt, double v_end,
  std::vector<double> & position, std::vector<double> & velocity,
  std::vector<double> & acceleration, std::vector<double> & time)
{
  ruckig::Ruckig<1> otg(ctrl_dt);
  ruckig::InputParameter<1> in;
  ruckig::OutputParameter<1> out;
  in.control_interface = ruckig::ControlInterface::Velocity;
  in.current_position = {0.0};
  in.current_velocity = {std::max(v_start, 0.0)};
  in.current_acceleration = {std::clamp(a_start, -d_max, a_max)};
  in.target_velocity = {std::max(v_end, 0.0)};
  in.target_acceleration = {0.0};
  in.max_acceleration = {a_max};
  in.min_acceleration = std::array<double, 1>{-d_max};
  in.max_jerk = {max_jerk};

  position = {0.0};
  velocity = {in.current_velocity[0]};
  acceleration = {in.current_acceleration[0]};
  time = {0.0};

  constexpr int kMaxSteps = 200000;
  for (int step = 1; step <= kMaxSteps; ++step) {
    const ruckig::Result result = otg.update(in, out);
    if (result != ruckig::Result::Working && result != ruckig::Result::Finished) {
      return false;
    }
    if (!std::isfinite(out.new_position[0]) || !std::isfinite(out.new_velocity[0]) ||
      !std::isfinite(out.new_acceleration[0]) ||
      out.new_position[0] + 1e-9 < position.back())
    {
      return false;
    }
    position.push_back(out.new_position[0]);
    velocity.push_back(std::max(out.new_velocity[0], 0.0));
    acceleration.push_back(out.new_acceleration[0]);
    time.push_back(step * ctrl_dt);
    out.pass_to_input(in);
    if (result == ruckig::Result::Finished) {
      return true;
    }
  }
  return false;
}

struct JerkPhase
{
  double duration;
  double jerk;
  double end_acceleration;
};

void integrateConstantJerk(
  double duration, double jerk,
  double & position, double & velocity, double & acceleration)
{
  position += velocity * duration +
    0.5 * acceleration * duration * duration +
    jerk * duration * duration * duration / 6.0;
  velocity += acceleration * duration + 0.5 * jerk * duration * duration;
  acceleration += jerk * duration;
}

bool sampleJerkPhases(
  double initial_velocity, double initial_acceleration, double ctrl_dt,
  const std::vector<JerkPhase> & phases,
  std::vector<double> & position, std::vector<double> & velocity,
  std::vector<double> & acceleration, std::vector<double> & time)
{
  constexpr double kNumericTolerance = 1e-9;
  position = {0.0};
  velocity = {initial_velocity};
  acceleration = {initial_acceleration};
  time = {0.0};

  double total_duration = 0.0;
  for (const auto & phase : phases) {
    if (!std::isfinite(phase.duration) || phase.duration < 0.0) {
      return false;
    }
    total_duration += phase.duration;
  }
  if (!std::isfinite(total_duration)) {
    return false;
  }
  if (total_duration <= kNumericTolerance) {
    return true;
  }

  double state_position = 0.0;
  double state_velocity = initial_velocity;
  double state_acceleration = initial_acceleration;
  double elapsed = 0.0;
  size_t phase_index = 0;
  double phase_remaining = phases.front().duration;
  constexpr int kMaxSteps = 200000;
  for (int step = 0; step < kMaxSteps && elapsed < total_duration; ++step) {
    const double sample_duration = std::min(ctrl_dt, total_duration - elapsed);
    double sample_remaining = sample_duration;
    while (sample_remaining > kNumericTolerance && phase_index < phases.size()) {
      const double duration = std::min(sample_remaining, phase_remaining);
      integrateConstantJerk(
        duration, phases[phase_index].jerk,
        state_position, state_velocity, state_acceleration);
      sample_remaining -= duration;
      phase_remaining -= duration;
      if (phase_remaining <= kNumericTolerance) {
        state_acceleration = phases[phase_index].end_acceleration;
        ++phase_index;
        if (phase_index < phases.size()) {
          phase_remaining = phases[phase_index].duration;
        }
      }
    }
    if (!std::isfinite(state_position) || !std::isfinite(state_velocity) ||
      !std::isfinite(state_acceleration) ||
      state_position + kNumericTolerance < position.back())
    {
      return false;
    }
    elapsed += sample_duration;
    position.push_back(state_position);
    velocity.push_back(std::max(state_velocity, 0.0));
    acceleration.push_back(state_acceleration);
    time.push_back(elapsed);
  }
  return elapsed + kNumericTolerance >= total_duration;
}

// Ruckig velocity mode is allowed to cross v_end and accelerate back to it.
// For a non-zero terminal creep speed, construct the time-optimal monotonic
// jerk-limited braking transition explicitly instead:
//
//   a_start --(-j)--> a_peak --(hold)--> a_peak --(+j)--> 0
//
// a_peak is solved from the requested velocity loss. Therefore the final
// state reaches (v_end, 0) from above without a positive-acceleration rebound.
// If a_start is already so negative that releasing it immediately must cross
// v_end, stable_target_reachable is false and the physically continuous best
// effort is an immediate +j release.
bool generateReachableBrakingProfile(
  double v_start, double a_start, double a_max, double d_max,
  double max_jerk, double ctrl_dt, double v_end,
  std::vector<double> & position, std::vector<double> & velocity,
  std::vector<double> & acceleration, std::vector<double> & time,
  bool & stable_target_reachable)
{
  constexpr double kNumericTolerance = 1e-9;
  const double initial_velocity = std::max(v_start, 0.0);
  const double initial_acceleration = std::clamp(a_start, -d_max, a_max);
  const double target_velocity = std::max(v_end, 0.0);
  stable_target_reachable = true;

  // Preserve the established hard-stop fallback and recovery from an already
  // undershot measured speed. R40 concerns braking to a non-zero creep speed.
  if (target_velocity < 1e-3 ||
    initial_velocity < target_velocity - kNumericTolerance)
  {
    stable_target_reachable = initial_velocity >= target_velocity;
    return generateRuckigVelocityProfile(
      initial_velocity, initial_acceleration, a_max, d_max,
      max_jerk, ctrl_dt, target_velocity,
      position, velocity, acceleration, time);
  }

  const double velocity_loss = initial_velocity - target_velocity;
  const double release_loss = initial_acceleration < 0.0 ?
    initial_acceleration * initial_acceleration / (2.0 * max_jerk) : 0.0;
  std::vector<JerkPhase> phases;

  if (velocity_loss + kNumericTolerance < release_loss) {
    // No future trajectory can keep v >= v_end from this state. Do not add
    // more braking or a positive-acceleration rebound: release immediately.
    stable_target_reachable = false;
    phases.push_back(
        {
          -initial_acceleration / max_jerk, max_jerk, 0.0});
  } else {
    const double peak_magnitude = std::sqrt(
      std::max(
        0.0, max_jerk * velocity_loss +
        0.5 * initial_acceleration * initial_acceleration));
    const double peak_acceleration = -std::min(peak_magnitude, d_max);
    const double ramp_down_time =
      (initial_acceleration - peak_acceleration) / max_jerk;
    if (ramp_down_time > kNumericTolerance) {
      phases.push_back(
          {
            ramp_down_time, -max_jerk, peak_acceleration});
    }

    if (peak_magnitude > d_max + kNumericTolerance) {
      const double triangular_loss =
        d_max * d_max / max_jerk -
        initial_acceleration * initial_acceleration / (2.0 * max_jerk);
      const double hold_time =
        std::max(0.0, (velocity_loss - triangular_loss) / d_max);
      if (hold_time > kNumericTolerance) {
        phases.push_back({hold_time, 0.0, peak_acceleration});
      }
    }

    const double ramp_up_time = -peak_acceleration / max_jerk;
    if (ramp_up_time > kNumericTolerance) {
      phases.push_back({ramp_up_time, max_jerk, 0.0});
    }
  }

  if (!sampleJerkPhases(
      initial_velocity, initial_acceleration, ctrl_dt, phases,
      position, velocity, acceleration, time))
  {
    return false;
  }
  if (phases.empty()) {
    acceleration.front() = 0.0;
    if (stable_target_reachable) {
      velocity.front() = target_velocity;
    }
    return true;
  }
  acceleration.back() = 0.0;
  if (stable_target_reachable) {
    if (std::fabs(velocity.back() - target_velocity) > 1e-6) {
      return false;
    }
    velocity.back() = target_velocity;
  }
  return true;
}

double computeStableBrakingDistance(
  double v_start, double a_start, double a_max, double d_max,
  double max_jerk, double ctrl_dt, double v_end)
{
  if (v_start <= v_end + 1e-9) {
    return 0.0;
  }
  std::vector<double> position, velocity, acceleration, time;
  bool stable_target_reachable = false;
  if (!generateReachableBrakingProfile(
      v_start, a_start, a_max, d_max, max_jerk, ctrl_dt, v_end,
      position, velocity, acceleration, time, stable_target_reachable) ||
    position.empty() || !stable_target_reachable || !std::isfinite(position.back()))
  {
    return std::numeric_limits<double>::infinity();
  }
  return std::max(0.0, position.back());
}

size_t applyTerminalSpeedEnvelope(
  std::vector<TrajPointWork> & work, double anchor_s, double end_speed)
{
  constexpr double kGeometryTolerance = 1e-6;
  constexpr double kBoundaryTolerance = 1e-4;
  std::vector<double> arc_s(work.size(), 0.0);
  for (size_t index = 1; index < work.size(); ++index) {
    const auto & previous = work[index - 1].pose.pose.position;
    const auto & current = work[index].pose.pose.position;
    arc_s[index] = arc_s[index - 1] +
      std::hypot(current.x - previous.x, current.y - previous.y);
  }
  anchor_s = std::clamp(anchor_s, 0.0, arc_s.back());

  size_t anchor_index = 1;
  if (anchor_s > kGeometryTolerance) {
    auto upper = std::lower_bound(arc_s.begin() + 1, arc_s.end(), anchor_s);
    anchor_index = static_cast<size_t>(std::distance(arc_s.begin(), upper));
    if (upper == arc_s.end()) {
      anchor_index = work.size() - 1;
    } else if (std::fabs(*upper - anchor_s) > kBoundaryTolerance) {
      const size_t lower_index = anchor_index - 1;
      const double segment_length = arc_s[anchor_index] - arc_s[lower_index];
      if (segment_length > kGeometryTolerance) {
        const double ratio = (anchor_s - arc_s[lower_index]) / segment_length;
        TrajPointWork anchor = work[lower_index];
        const auto & from = work[lower_index].pose.pose.position;
        const auto & to = work[anchor_index].pose.pose.position;
        anchor.pose.pose.position.x = from.x + ratio * (to.x - from.x);
        anchor.pose.pose.position.y = from.y + ratio * (to.y - from.y);
        anchor.pose.pose.position.z = from.z + ratio * (to.z - from.z);
        anchor.pose.pose.orientation =
          ratio < 0.5 ? work[lower_index].pose.pose.orientation :
          work[anchor_index].pose.pose.orientation;
        anchor.speed_limit = std::min(
          {work[lower_index].speed_limit, work[anchor_index].speed_limit, end_speed});
        anchor.curvature = std::max(
          std::fabs(work[lower_index].curvature), std::fabs(work[anchor_index].curvature));
        if (anchor.region_type.empty()) {
          anchor.region_type = "terminal_reserve";
        }
        work.insert(
          work.begin() + static_cast<std::ptrdiff_t>(anchor_index), std::move(anchor));
      }
    }
  }

  for (size_t index = anchor_index; index < work.size(); ++index) {
    work[index].speed_limit = std::min(work[index].speed_limit, end_speed);
    if (work[index].region_type.empty()) {
      work[index].region_type = "terminal_reserve";
    }
  }
  return anchor_index;
}

// Build a stable terminal segment that reaches (distance, v_end, a=0)
// without crossing v_end. Existing acceleration is first released to zero;
// any remaining distance is spent coasting before a monotonic S-curve brake.
// This explicit release/coast reserve prevents receding-horizon replans from
// carrying a large negative acceleration into the creep-speed boundary.
bool generateStableTerminalSegment(
  double v_start, double a_start, double d_max,
  double max_jerk, double ctrl_dt, double distance, double v_end,
  std::vector<double> & position, std::vector<double> & velocity,
  std::vector<double> & acceleration, std::vector<double> & time)
{
  constexpr double kNumericTolerance = 1e-9;
  const double initial_velocity = std::max(v_start, 0.0);
  const double initial_acceleration = a_start;
  const double target_velocity = std::max(v_end, 0.0);
  if (distance <= kNumericTolerance ||
    initial_velocity < target_velocity - kNumericTolerance)
  {
    return false;
  }

  if (initial_acceleration < -kNumericTolerance) {
    const double velocity_loss = initial_velocity - target_velocity;
    const double release_loss =
      initial_acceleration * initial_acceleration / (2.0 * max_jerk);
    if (velocity_loss + kNumericTolerance < release_loss) {
      return false;
    }

    const double peak_magnitude = std::sqrt(
      std::max(
        0.0, max_jerk * velocity_loss +
        0.5 * initial_acceleration * initial_acceleration));
    const double peak_acceleration = -std::min(peak_magnitude, d_max);
    std::vector<JerkPhase> direct_phases;
    const double ramp_down_time =
      (initial_acceleration - peak_acceleration) / max_jerk;
    if (ramp_down_time > kNumericTolerance) {
      direct_phases.push_back(
          {
            ramp_down_time, -max_jerk, peak_acceleration});
    }
    if (peak_magnitude > d_max + kNumericTolerance) {
      const double triangular_loss =
        d_max * d_max / max_jerk -
        initial_acceleration * initial_acceleration / (2.0 * max_jerk);
      const double hold_time =
        std::max(0.0, (velocity_loss - triangular_loss) / d_max);
      if (hold_time > kNumericTolerance) {
        direct_phases.push_back(
            {
              hold_time, 0.0, peak_acceleration});
      }
    }
    const double ramp_up_time = -peak_acceleration / max_jerk;
    if (ramp_up_time > kNumericTolerance) {
      direct_phases.push_back(
          {
            ramp_up_time, max_jerk, 0.0});
    }

    std::vector<double> direct_s, direct_v, direct_a, direct_t;
    if (!sampleJerkPhases(
        initial_velocity, initial_acceleration, ctrl_dt, direct_phases,
        direct_s, direct_v, direct_a, direct_t))
    {
      return false;
    }
    const double remaining_distance = distance - direct_s.back();
    constexpr double kControllerPreviewSteps = 2.0;
    const double release_reserve_distance =
      std::max(initial_velocity, target_velocity) *
      kControllerPreviewSteps * ctrl_dt;
    if (remaining_distance < -1e-6) {
      return false;
    }
    if (remaining_distance <= release_reserve_distance + 1e-6) {
      if (remaining_distance > kNumericTolerance) {
        direct_phases.push_back(
            {
              remaining_distance / target_velocity, 0.0, 0.0});
      }
      if (!sampleJerkPhases(
          initial_velocity, initial_acceleration, ctrl_dt, direct_phases,
          position, velocity, acceleration, time))
      {
        return false;
      }
      if (std::fabs(position.back() - distance) > 1e-6 ||
        std::fabs(velocity.back() - target_velocity) > 1e-6 ||
        std::fabs(acceleration.back()) > 1e-6)
      {
        return false;
      }
      position.back() = distance;
      velocity.back() = target_velocity;
      acceleration.back() = 0.0;
      return true;
    }
  }

  std::vector<JerkPhase> release_phases;
  if (initial_acceleration < -kNumericTolerance) {
    release_phases.push_back(
        {
          -initial_acceleration / max_jerk, max_jerk, 0.0});
  } else if (initial_acceleration > kNumericTolerance) {
    release_phases.push_back(
        {
          initial_acceleration / max_jerk, -max_jerk, 0.0});
  }

  std::vector<double> release_s, release_v, release_a, release_t;
  if (!sampleJerkPhases(
      initial_velocity, initial_acceleration, ctrl_dt, release_phases,
      release_s, release_v, release_a, release_t))
  {
    return false;
  }
  const double released_velocity = release_v.back();
  const double released_distance = release_s.back();
  if (released_velocity < target_velocity - kNumericTolerance) {
    return false;
  }

  const double velocity_loss = released_velocity - target_velocity;
  const double peak_magnitude = std::min(
    std::sqrt(std::max(0.0, max_jerk * velocity_loss)), d_max);
  std::vector<JerkPhase> braking_phases;
  if (peak_magnitude > kNumericTolerance) {
    braking_phases.push_back(
        {
          peak_magnitude / max_jerk, -max_jerk, -peak_magnitude});
    const double triangular_loss = peak_magnitude * peak_magnitude / max_jerk;
    if (velocity_loss > triangular_loss + kNumericTolerance) {
      braking_phases.push_back(
          {
            (velocity_loss - triangular_loss) / peak_magnitude,
            0.0, -peak_magnitude});
    }
    braking_phases.push_back(
        {
          peak_magnitude / max_jerk, max_jerk, 0.0});
  }

  std::vector<double> braking_s, braking_v, braking_a, braking_t;
  if (!sampleJerkPhases(
      released_velocity, 0.0, ctrl_dt, braking_phases,
      braking_s, braking_v, braking_a, braking_t))
  {
    return false;
  }
  const double minimum_distance = released_distance + braking_s.back();
  if (minimum_distance > distance + 1e-6) {
    return false;
  }

  std::vector<JerkPhase> phases = release_phases;
  const double coast_distance = std::max(0.0, distance - minimum_distance);
  if (coast_distance > kNumericTolerance) {
    if (released_velocity <= kNumericTolerance) {
      return false;
    }
    phases.push_back(
        {
          coast_distance / released_velocity, 0.0, 0.0});
  }
  phases.insert(phases.end(), braking_phases.begin(), braking_phases.end());
  if (!sampleJerkPhases(
      initial_velocity, initial_acceleration, ctrl_dt, phases,
      position, velocity, acceleration, time))
  {
    return false;
  }
  if (std::fabs(position.back() - distance) > 1e-6 ||
    std::fabs(velocity.back() - target_velocity) > 1e-6 ||
    std::fabs(acceleration.back()) > 1e-6)
  {
    return false;
  }
  position.back() = distance;
  velocity.back() = target_velocity;
  acceleration.back() = 0.0;
  return true;
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
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_execution_delay", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".terminal_safety_margin", rclcpp::ParameterValue(0.0));
  node->get_parameter(plugin_name_ + ".max_jerk", max_jerk_);
  node->get_parameter(plugin_name_ + ".control_dt", control_dt_);
  node->get_parameter(
    plugin_name_ + ".terminal_execution_delay", terminal_execution_delay_);
  node->get_parameter(
    plugin_name_ + ".terminal_safety_margin", terminal_safety_margin_);
  terminal_execution_delay_ = std::max(0.0, terminal_execution_delay_);
  terminal_safety_margin_ = std::max(0.0, terminal_safety_margin_);
}

double SCurveSpeedPlanner::computeTerminalExecutionReserve(
  double velocity, double planned_acceleration,
  double execution_delay, double safety_margin)
{
  velocity = std::max(0.0, velocity);
  execution_delay = std::max(0.0, execution_delay);
  safety_margin = std::max(0.0, safety_margin);
  // A negative consumed target acceleration preserves jerk continuity, but it
  // does not prove that the smoother/chassis has established that braking.
  const double conservative_acceleration = std::max(0.0, planned_acceleration);
  return velocity * execution_delay +
         0.5 * conservative_acceleration * execution_delay * execution_delay +
         safety_margin;
}

// ===================== Ruckig 沿弧长生成 jerk 限制速度剖面 =====================
bool SCurveSpeedPlanner::generateRuckigProfile(
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
  if (n == 0) {return false;}
  a_max = std::max(1e-3, a_max);
  d_max = std::max(1e-3, d_max);
  max_jerk = std::max(1e-3, max_jerk);
  ctrl_dt = std::max(1e-3, ctrl_dt);
  a_start = std::clamp(a_start, -d_max, a_max);
  if (n == 1) {
    v_out[0] = std::clamp(v_start, 0.0, std::max(0.0, v_env[0]));
    a_out[0] = a_start;
    return false;
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

  size_t terminal_start_i = stop_i;
  if (!stop_at_env_zero) {
    while (terminal_start_i > 0 && v_env[terminal_start_i - 1] <= v_end + 1e-3) {
      --terminal_start_i;
    }
  }

  ruckig::Ruckig<1> otg(ctrl_dt);
  ruckig::InputParameter<1> in;
  ruckig::OutputParameter<1> out;
  const double initial_velocity = std::max(v_start, 0.0);
  const double initial_acceleration = a_start;
  in.current_position = {0.0};
  in.current_velocity = {initial_velocity};
  in.current_acceleration = {initial_acceleration};
  in.min_velocity = std::array<double, 1>{0.0};
  in.max_acceleration = {a_max};
  in.max_jerk = {max_jerk};
  in.min_acceleration = std::array<double, 1>{-d_max};   // 非对称：减速幅值 ≤ d_max

  std::vector<double> rs{0.0}, rv{in.current_velocity[0]}, ra{a_start}, rt{0.0};
  double tnow = 0.0;
  bool failed = false;
  bool terminal_infeasible = false;
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
    if (!stop_at_env_zero && v_end >= 1e-3 &&
      tgt.index >= terminal_start_i &&
      in.current_acceleration[0] < -1e-9 &&
      in.current_velocity[0] >= v_end - 1e-9)
    {
      std::vector<double> seg_s, seg_v, seg_a, seg_t;
      if (!generateStableTerminalSegment(
          in.current_velocity[0], in.current_acceleration[0],
          d_max, max_jerk, ctrl_dt, segment_distance, v_end,
          seg_s, seg_v, seg_a, seg_t))
      {
        failed = true;
        terminal_infeasible = true;
        break;
      }
      const double segment_start_s = in.current_position[0];
      for (size_t index = 1; index < seg_s.size(); ++index) {
        rs.push_back(segment_start_s + seg_s[index]);
        rv.push_back(seg_v[index]);
        ra.push_back(seg_a[index]);
        rt.push_back(tnow + seg_t[index]);
      }
      tnow += seg_t.back();
      in.current_position = {s_t};
      in.current_velocity = {v_end};
      in.current_acceleration = {0.0};
      continue;
    }

    // Bound iterations by a conservative travel, acceleration, and jerk time
    // for this segment so an infeasible target cannot consume the global cap.
    const double min_accel_limit = std::min(a_max, d_max);
    const double duration_budget =
      2.0 + 2.0 * segment_distance / std::max(seg_vmax, 1e-3) +
      4.0 * seg_vmax / min_accel_limit + 4.0 * std::sqrt(seg_vmax / max_jerk);
    if (!std::isfinite(duration_budget) || duration_budget <= 0.0) {
      failed = true;
      terminal_infeasible = !stop_at_env_zero && tgt.index >= terminal_start_i;
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
        terminal_infeasible = !stop_at_env_zero && tgt.index >= terminal_start_i;
        break;
      }
      if (!std::isfinite(out.new_position[0]) || !std::isfinite(out.new_velocity[0]) ||
        !std::isfinite(out.new_acceleration[0]))
      {
        failed = true;
        terminal_infeasible = !stop_at_env_zero && tgt.index >= terminal_start_i;
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
      terminal_infeasible = terminal_infeasible ||
        (!stop_at_env_zero && tgt.index >= terminal_start_i);
      break;
    }
  }

  bool braking_fallback = false;
  bool stable_target_reachable = true;
  if (failed) {
    braking_fallback = generateReachableBrakingProfile(
      initial_velocity, initial_acceleration, a_max, d_max,
      max_jerk, ctrl_dt, v_end, rs, rv, ra, rt,
      stable_target_reachable);
    if (!braking_fallback) {
      rs = {0.0};
      rv = {initial_velocity};
      ra = {initial_acceleration};
      rt = {0.0};
    } else if (!stop_at_env_zero) {
      constexpr double kTerminalSpeedTolerance = 1e-3;
      constexpr double kTerminalAccelerationTolerance = 1e-3;
      const bool undershot = std::any_of(
        rv.begin(), rv.end(),
        [v_end](double velocity) {
          return velocity < v_end - kTerminalSpeedTolerance;
        });
      size_t stable_index = rv.size();
      for (size_t index = 0; index < rv.size(); ++index) {
        if (std::fabs(rv[index] - v_end) <= kTerminalSpeedTolerance &&
          std::fabs(ra[index]) <= kTerminalAccelerationTolerance)
        {
          stable_index = index;
          break;
        }
      }
      const double stable_s = stable_index == rv.size() ?
        std::numeric_limits<double>::infinity() : rs[stable_index];
      constexpr double kControllerPreviewSteps = 2.0;
      // The controller consumes two Ruckig samples ahead at 10 Hz. Allow one
      // additional planner sample for the discrete stable-state boundary.
      // Dynamic reserve is an execution-lag allowance and does not replace
      // this existing numeric/resampling tolerance.
      const double stable_position_tolerance =
        (std::max(initial_velocity, v_end) * kControllerPreviewSteps +
        v_end) * ctrl_dt;
      terminal_infeasible =
        !stable_target_reachable || undershot ||
        stable_s >
        arc_s[terminal_start_i] + stable_position_tolerance + 1e-6;
    }
  }

  // 正常剖面遵守包络；位置硬目标不可达时，改用同源的 jerk 限制制动状态，
  // 不再只钳 velocity 伪造已经到达 end_speed。
  size_t k = 0;
  for (size_t i = 0; i < n; ++i) {
    if (!failed && arc_s[i] >= target - 1e-9) {
      v_out[i] = v_end; a_out[i] = 0.0; t_out[i] = rt.back();
      continue;
    }
    while (k + 1 < rs.size() && rs[k + 1] < arc_s[i]) {++k;}
    if (k + 1 >= rs.size()) {
      v_out[i] = rv.back(); a_out[i] = ra.back(); t_out[i] = rt.back();
      if (failed && rv.back() > 1e-3 && arc_s[i] > rs.back()) {
        t_out[i] += (arc_s[i] - rs.back()) / rv.back();
      }
    } else {
      const double denom = rs[k + 1] - rs[k];
      const double f = (denom > 1e-9) ? std::clamp((arc_s[i] - rs[k]) / denom, 0.0, 1.0) : 0.0;
      v_out[i] = rv[k] + f * (rv[k + 1] - rv[k]);
      a_out[i] = ra[k] + f * (ra[k + 1] - ra[k]);
      t_out[i] = rt[k] + f * (rt[k + 1] - rt[k]);
    }
    v_out[i] = std::max(0.0, v_out[i]);
    if (!failed && i > 0) {
      v_out[i] = std::min(v_out[i], std::max(0.0, v_env[i]));
    }
  }
  return terminal_infeasible;
}

// ===================== 插件 hook：Phase B（Ruckig jerk 限制剖面）=====================
void SCurveSpeedPlanner::assignVelocityProfile(
  std::vector<TrajPointWork> & work, double curr_vel, double curr_accel,
  double /*cruise_limit*/, bool terminal_brake_latched)
{
  if (work.size() < 2) {
    return;
  }

  // 1) Dynamic terminal reserve. The regular stop_distance keeps its semantic
  //    meaning; the reserve makes the profile reach stable creep speed earlier
  //    and then coast through the reserved segment and stop zone.
  const auto initial_arc_s = computeCumulativeArcLengths(work);
  const bool obstacle_or_hard_stop = std::any_of(
    work.begin() + 1, work.end(),
    [](const TrajPointWork & point) {return point.speed_limit < 1e-3;});
  if (!obstacle_or_hard_stop && end_speed_ >= 1e-3 && stop_distance_ > 0.0) {
    const double terminal_boundary_s = std::max(0.0, initial_arc_s.back() - stop_distance_);
    const double conservative_acceleration = std::max(0.0, curr_accel);
    const double braking_distance = computeStableBrakingDistance(
      std::max(0.0, curr_vel), conservative_acceleration,
      accel_, std::fabs(decel_), max_jerk_, control_dt_, end_speed_);
    const double continuity_braking_distance = computeStableBrakingDistance(
      std::max(0.0, curr_vel), curr_accel,
      accel_, std::fabs(decel_), max_jerk_, control_dt_, end_speed_);
    const double execution_reserve = computeTerminalExecutionReserve(
      curr_vel, curr_accel, terminal_execution_delay_, terminal_safety_margin_);

    last_planning_diagnostics_.terminal_execution_reserve = execution_reserve;
    last_planning_diagnostics_.terminal_braking_distance = braking_distance;
    last_planning_diagnostics_.terminal_distance_available = terminal_boundary_s;
    last_planning_diagnostics_.terminal_brake_required = terminal_brake_latched ||
      !std::isfinite(braking_distance) ||
      terminal_boundary_s <= braking_distance + execution_reserve + 1e-6;

    double anchor_s = std::max(0.0, terminal_boundary_s - execution_reserve);
    if (terminal_brake_latched && std::isfinite(continuity_braking_distance)) {
      // Same-goal path replacement may temporarily lengthen the local path.
      // Once braking is latched, the reserve has already selected the start
      // cycle: from then on, target the current shortest reachable braking
      // distance instead of subtracting the full reserve again every cycle.
      // This prevents both re-acceleration on a lengthened path and a false
      // infeasible result while the reserved distance is being consumed.
      anchor_s = std::min(terminal_boundary_s, continuity_braking_distance);
    }
    if (execution_reserve > 1e-9 || terminal_brake_latched) {
      applyTerminalSpeedEnvelope(work, anchor_s, end_speed_);
    }
  }

  const size_t n = work.size();
  // 2) 累计弧长 + 包络（Phase A 已由基类完成：障碍/巡航/曲率/末端蠕行区，
  //    包络零点含障碍缓冲带，动态 reserve 已提前稳定蠕行边界）。
  std::vector<double> v_env(n, 0.0), arc_s(n, 0.0);
  v_env[0] = std::max(0.0, work[0].speed_limit);
  for (size_t i = 1; i < n; ++i) {
    const double dx = work[i].pose.pose.position.x - work[i - 1].pose.pose.position.x;
    const double dy = work[i].pose.pose.position.y - work[i - 1].pose.pose.position.y;
    arc_s[i] = arc_s[i - 1] + std::hypot(dx, dy);
    v_env[i] = std::max(0.0, work[i].speed_limit);
  }

  // 3) Ruckig 生成 jerk 限制剖面（非对称 a_max / d_max=|decel_|）。
  //    终点目标速度 = end_speed_（障碍停止时函数内部强制 0）。
  std::vector<double> v_out, a_out, t_out;
  last_planning_diagnostics_.terminal_infeasible = generateRuckigProfile(
    v_env, arc_s, curr_vel, curr_accel,
    accel_, std::fabs(decel_),
    max_jerk_, control_dt_,
    v_out, a_out, t_out, end_speed_);

  // 4) 回填 work（曲率已由 Phase A3 填好）。
  for (size_t i = 0; i < n; ++i) {
    work[i].speed_limit = v_out[i];
    work[i].accel = a_out[i];
    work[i].rel_time = t_out[i];
  }
}

}  // namespace nav2_speed_planner

PLUGINLIB_EXPORT_CLASS(
  nav2_speed_planner::SCurveSpeedPlanner, nav2_core::SpeedPlanner)
