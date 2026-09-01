// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// S 型（jerk 限制速度曲线）规划器实现 —— 基于 Ruckig。
#include "nav2_speed_planner/backends/scurve_speed_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav2_speed_planner/core/profile_derivatives.hpp"
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

/// jerk 限制下，从 v0 出发在 dist 内能增加的最大速度（首尾加速度均为 0）。
/// 无闭式，二分求解；精度 1e-4 足够（点距约 2 cm）。时间反演后同样适用于减速：
/// 「从 v0 减到 v1」与「从 v1 加到 v0」所需距离相同（限值取 d_max）。
inline double reachableDeltaV(double v0, double dist, double a_max, double max_jerk)
{
  if (dist <= 1e-9) {return 0.0;}
  auto travel = [&](double dv) {
      const double a_p = std::min(std::sqrt(std::max(dv, 0.0) * max_jerk), a_max);
      const double dv_tri = a_p * a_p / max_jerk;   // 三角加速度段能换到的 Δv
      const double t = (dv <= dv_tri + 1e-12) ?
        2.0 * std::sqrt(std::max(dv, 0.0) / max_jerk) :
        2.0 * a_p / max_jerk + (dv - dv_tri) / a_p;
      return v0 * t + 0.5 * dv * t;                 // 平均速度 ≈ v0 + dv/2
    };
  double lo = 0.0, hi = 5.0;
  for (int k = 0; k < 60; ++k) {
    const double mid = 0.5 * (lo + hi);
    (travel(mid) <= dist) ? lo = mid : hi = mid;
  }
  return lo;
}

/// 让**包络本身**处处动力学可达——修包络，而不是事后修目标。
///
/// 梯形后端早就这么做（`computeDecelPlan` + 加速 pass，注释原文「对一切低速点
/// 自动生成物理正确的进入坡道」），**所以它从不出现本类故障**。SCurve 此前跳过
/// 这一步，把带**阶跃**的原始包络直接交给 Ruckig：曲率约束把点压到
/// `min_curvature_speed`（`curve_lead_in/out` 只加宽井、井壁仍是阶跃）→
/// `extractDescentTargets` 把每个谷底设成硬目标 → 相邻目标互不可达 → -110 →
/// 兜底 → 输出恒 `end_speed` → 全程蠕行。
///
/// **必须用恒加速度公式 `v² = v_next² + 2·a·Δs`，不能用 jerk 完整循环。**
/// 首次实现误把「加速度 0→a_p→0 的完整循环」套在每个 2 cm 点间隔上，而连续
/// 剖面里加速度是跨点延续的、不会每点归零——逐点算出的可达 Δv 被严重低估，
/// 实测导致包络过度压低：曲率井峰值 0.34~0.51 → 0.277、求解器失败 0 → 10、
/// 既有 24 场景 10 个变化、4 个单测失败。
///
/// jerk 带来的额外距离需求由 `clampTargetsToReachable` 承担——目标之间确实
/// 要求加速度首尾归零，那里用完整循环才是对的。两者分工互补：
///   包络遍历（恒加速度）消除粗粒度阶跃 → 目标 clamp（jerk 完整循环）收尾。
inline void applyEnvelopeReachability(
  std::vector<double> & env, const std::vector<double> & arc_s, size_t stop_i,
  double a_max, double d_max)
{
  if (env.size() < 2 || arc_s.size() != env.size()) {return;}
  const size_t last = std::min(stop_i, env.size() - 1);

  // 后向：保证能减速到下游约束（终点蠕行区 / 障碍零速 / 曲率谷底）
  for (size_t i = last; i-- > 1; ) {
    const double ds = arc_s[i + 1] - arc_s[i];
    if (ds <= 1e-9) {continue;}
    const double v_next = std::max(0.0, env[i + 1]);
    env[i] = std::min(env[i], std::sqrt(v_next * v_next + 2.0 * d_max * ds));
  }
  // 前向：保证能从上游加速上来（削平曲率井出口阶跃，顺带修 C4 的边界凸起）。
  // index 0 是机器人状态戳，不参与设置（同 backward pass 约定）。
  for (size_t i = 1; i < last; ++i) {
    const double ds = arc_s[i + 1] - arc_s[i];
    if (ds <= 1e-9) {continue;}
    const double v_prev = std::max(0.0, env[i]);
    env[i + 1] = std::min(env[i + 1], std::sqrt(v_prev * v_prev + 2.0 * a_max * ds));
  }
}

/// 把目标速度钳到「从上一目标出发、jerk 限制下确实到得了」的范围内。
///
/// 为什么必需：`extractDescentTargets` 把包络的**每个谷底**都设成硬目标
/// （位置 + 速度 + 加速度归零）。曲率约束会造出多个深浅不一的"井"
/// （lead_in/out 把井加宽，但井壁仍是阶跃），相邻谷底之间可能根本升不回去——
/// 实机实测：v0=0.100 → target_v=0.4148、dist=0.3375 m，而 jerk 限制下
/// 需要 0.41 m，Ruckig 返回 -110（ErrorExecutionTimeCalculation）。
/// 一次求解失败就掉进制动兜底，输出恒为 end_speed → 全程蠕行。
///
/// **包络是上界，不是必须精确命中的轨迹。** 到不了就慢一点走，永远安全；
/// 强行要求升速反而让整条计划不可行。本函数只降不升。
///
/// 升速可达性（首尾加速度均为 0 的 jerk 限制加速）：
///   Δv ≤ a_p²/j 且 a_p ≤ a_max，位移 s = v0·t + Δv·t/2，t = 2·a_p/j
/// 给定 s 反解 Δv 无闭式，用二分；精度 1e-4 足够（点距约 2 cm）。
inline void clampTargetsToReachable(
  std::vector<SpeedTarget> & targets, const std::vector<double> & arc_s,
  double v_start, double a_max, double max_jerk)
{
  auto reachable_dv = [&](double v0, double dist) {
      return reachableDeltaV(v0, dist, a_max, max_jerk);
    };

  // ① 前向：升速受限于「从上一目标能加到多少」
  double prev_v = std::max(v_start, 0.0);
  double prev_s = arc_s.empty() ? 0.0 : arc_s.front();
  for (auto & tgt : targets) {
    if (tgt.index >= arc_s.size()) {continue;}
    const double s = arc_s[tgt.index];
    if (tgt.velocity > prev_v) {
      tgt.velocity = std::min(tgt.velocity, prev_v + reachable_dv(prev_v, s - prev_s));
    }
    prev_v = tgt.velocity;
    prev_s = s;
  }

  // ② 后向：降速受限于「要减到下一目标，本目标最多能有多快」
  //    实机 R48 第二形态：0.3899 → 0.050 仅 0.0652 m（需 0.187 m）。
  //    只做前向必然漏掉这一向——单向补丁的教训。
  for (size_t k = targets.size(); k-- > 1; ) {
    if (targets[k].index >= arc_s.size() || targets[k - 1].index >= arc_s.size()) {
      continue;
    }
    const double ds = arc_s[targets[k].index] - arc_s[targets[k - 1].index];
    if (ds <= 1e-9) {continue;}
    // 时间反演：从 v_prev 减到 v_next 所需距离 == 从 v_next 加到 v_prev（限值 d_max）
    const double cap = targets[k].velocity + reachable_dv(targets[k].velocity, ds);
    targets[k - 1].velocity = std::min(targets[k - 1].velocity, cap);
  }
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
    if (target_velocity > kNumericTolerance &&
      remaining_distance <= release_reserve_distance + 1e-6)
    {
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
  node->get_parameter(plugin_name_ + ".max_jerk", max_jerk_);
  node->get_parameter(plugin_name_ + ".control_dt", control_dt_);
  node->get_parameter(
    plugin_name_ + ".terminal_execution_delay", terminal_execution_delay_);
  terminal_execution_delay_ = std::max(0.0, terminal_execution_delay_);
  RCLCPP_INFO(
    node->get_logger(),
    "%s effective terminal config: end_speed=%.3f m/s speed_cap=%.3f m/s "
    "creep_entry=%.3f m execution_delay=%.3f s",
    plugin_name_.c_str(), end_speed_, terminal_speed_cap_, terminal_creep_distance_,
    terminal_execution_delay_);
}

// ===================== Ruckig 沿弧长生成 jerk 限制速度剖面 =====================
FeasibilityResult SCurveSpeedPlanner::generateRuckigProfile(
  const std::vector<double> & v_env_in,
  const std::vector<double> & arc_s,
  double v_start,
  double a_start,
  double a_max, double d_max,
  double max_jerk, double ctrl_dt,
  std::vector<double> & v_out,
  std::vector<double> & a_out,
  std::vector<double> & t_out,
  double v_end,
  bool terminal_brake_latched,
  double terminal_speed_cap)
{
  // 包络需就地可达化（见 applyEnvelopeReachability），故取可变副本；
  // 后续全部逻辑都基于可达化后的包络，避免与目标提取口径不一致。
  std::vector<double> v_env = v_env_in;

  const size_t n = v_env.size();
  v_out.assign(n, 0.0);
  a_out.assign(n, 0.0);
  t_out.assign(n, 0.0);
  if (n == 0) {return FeasibilityResult{};}   // 空输入：不判不可行（同归一前）
  a_max = std::max(1e-3, a_max);
  d_max = std::max(1e-3, d_max);
  max_jerk = std::max(1e-3, max_jerk);
  ctrl_dt = std::max(1e-3, ctrl_dt);
  a_start = std::clamp(a_start, -d_max, a_max);
  if (n == 1) {
    v_out[0] = std::clamp(v_start, 0.0, std::max(0.0, v_env[0]));
    a_out[0] = a_start;
    return FeasibilityResult{};   // 单点：不判不可行（同归一前）
  }

  // 停止点：首个零包络点（障碍）否则终点。障碍停止时终点速度强制 0。
  size_t stop_i = n - 1;
  bool found_zero_envelope = false;
  for (size_t i = 0; i < n; ++i) {
    if (v_env[i] < 1e-3) {stop_i = i; found_zero_envelope = true; break;}
  }
  // 最后一点的零速属于 goal terminal；只有终点之前的零速才是障碍硬停。
  const bool stop_at_env_zero = found_zero_envelope && stop_i + 1 < n;
  v_end = stop_at_env_zero ? 0.0 : std::max(0.0, v_end);
  const double target = arc_s[stop_i];

  if (terminal_brake_latched && !stop_at_env_zero) {
    // 锁存后禁止回到巡航速度，但允许“低于蠕行值”的跟踪误差有限恢复到
    // terminal_speed_cap。这样实测偶然掉到0.03时仍可回到0.05；恢复上限
    // 始终是蠕行速度，最终v_end=0目标仍由剩余距离约束，不会重新加速到巡航。
    const double monotonic_cap = std::max(
      std::max(0.0, v_start), std::max(0.0, terminal_speed_cap));
    for (size_t index = 1; index < v_env.size(); ++index) {
      v_env[index] = std::min(v_env[index], monotonic_cap);
    }
    v_end = std::min(v_end, monotonic_cap);
  }

  // 目标序列 = 包络下降沿谷底（蠕行区界/弯心/缓冲带起点）+ 最终停止点。
  // 逐目标分段规划："在谷底恰好降到谷速"由 target 硬保证；段内 max_velocity 取
  // 区间包络最大值（渐变段轻微越界由重采样钳位收尾），并抬到 ≥ 当前/目标速度
  // 保证输入合法。
  // 先让包络处处可达（恒加速度口径），再提取目标——顺序不能反。
  applyEnvelopeReachability(v_env, arc_s, stop_i, a_max, d_max);
  std::vector<SpeedTarget> targets = extractDescentTargets(v_env, stop_i);
  targets.push_back({stop_i, v_end});
  // 硬目标必须彼此可达，否则一次求解失败就整条掉进兜底（实机 R48）。
  clampTargetsToReachable(targets, arc_s, v_start, a_max, max_jerk);

  size_t terminal_start_i = stop_i;
  if (!stop_at_env_zero) {
    const double terminal_zone_cap = std::max(v_end, terminal_speed_cap);
    while (terminal_start_i > 0 &&
      v_env[terminal_start_i - 1] <= terminal_zone_cap + 1e-3)
    {
      --terminal_start_i;
    }
    // 已经到达蠕行上限后，不再重复求解一个“保持同速且a=0”的中间硬目标。
    // 当前若带极小负加速度（实测约-0.003 m/s²），为了回到完全相同的0.05
    // 必然先轻微下穿再加速，单调终端段会正确拒绝，却形成无意义的
    // terminal_infeasible。跳过该冗余目标后，直接用剩余距离规划0.05->0。
    const double terminal_initial_velocity = std::max(0.0, v_start);
    if (terminal_brake_latched && v_end + 1e-6 < terminal_zone_cap &&
      terminal_initial_velocity <= terminal_zone_cap + 1e-3)
    {
      targets.erase(
        std::remove_if(
          targets.begin(), targets.end(),
          [terminal_start_i, v_end, terminal_initial_velocity](
            const SpeedTarget & speed_target) {
            return speed_target.index >= terminal_start_i &&
                   speed_target.velocity > v_end + 1e-6 &&
                   speed_target.velocity >= terminal_initial_velocity - 1e-3;
          }),
        targets.end());
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
  // 段级兜底必须继承“当前失败分段”的目标，而不是无条件跳到全局v_end。
  // 两段终端轨迹中，蠕行入口目标是0.05、最终点目标才是0；若入口段失败
  // 却直接兜底到0，滚动规划会在冻结执行余量边界（实测约终点前21cm）早停。
  double fallback_target_velocity = v_end;
  FeasibilityResult feas;   // 归一：7 处赋值收敛到此，语义逐字保持
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
    const bool intermediate_terminal_handoff =
      terminal_brake_latched && !stop_at_env_zero &&
      terminal_speed_cap > v_end + 1e-6 &&
      tgt.index >= terminal_start_i &&
      in.target_velocity[0] > v_end + 1e-6;
    fallback_target_velocity = intermediate_terminal_handoff ?
      in.target_velocity[0] : v_end;

    const double segment_distance = s_t - in.current_position[0];
    // A negative acceleration is an instantaneous motion state, not proof that final-goal
    // braking has started. Without the lifecycle latch this branch makes any mid-path slowdown
    // sticky across receding-horizon replans and prevents recovery after a dynamic obstacle.
    if (terminal_brake_latched && !stop_at_env_zero &&
      tgt.index >= terminal_start_i &&
      in.current_acceleration[0] < -1e-9 &&
      in.current_velocity[0] >= in.target_velocity[0] - 1e-9)
    {
      std::vector<double> seg_s, seg_v, seg_a, seg_t;
      if (!generateStableTerminalSegment(
          in.current_velocity[0], in.current_acceleration[0],
          d_max, max_jerk, ctrl_dt, segment_distance, in.target_velocity[0],
          seg_s, seg_v, seg_a, seg_t))
      {
        failed = true;
        feas.mark(FeasibilityResult::Reason::kStableSegmentUnreachable, true);
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
      in.current_velocity = {in.target_velocity[0]};
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
      feas.mark(
        FeasibilityResult::Reason::kDurationBudgetInvalid,
        !stop_at_env_zero && tgt.index >= terminal_start_i);
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
        feas.solver_code = static_cast<int>(res);
        feas.solver_v0 = in.current_velocity[0];
        feas.solver_a0 = in.current_acceleration[0];
        feas.solver_vmax = in.max_velocity[0];
        feas.solver_target_v = in.target_velocity[0];
        feas.solver_distance = in.target_position[0] - in.current_position[0];
        failed = true;
        feas.mark(
          FeasibilityResult::Reason::kSolverError,
          !stop_at_env_zero && tgt.index >= terminal_start_i);
        break;
      }
      if (!std::isfinite(out.new_position[0]) || !std::isfinite(out.new_velocity[0]) ||
        !std::isfinite(out.new_acceleration[0]))
      {
        failed = true;
        feas.mark(
          FeasibilityResult::Reason::kNonFiniteOutput,
          !stop_at_env_zero && tgt.index >= terminal_start_i);
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
      feas.markOr(
        FeasibilityResult::Reason::kIterationExhausted,
        !stop_at_env_zero && tgt.index >= terminal_start_i);
      break;
    }
  }

  bool braking_fallback = false;
  bool stable_target_reachable = true;
  if (failed) {
    feas.provenance = FeasibilityResult::Provenance::kBrakingFallback;
    // ── 段级兜底（2026-08-08）──────────────────────────────────────────
    //
    // 旧实现从 **原始起点状态** 重新生成整条制动剖面
    //（`generateReachableBrakingProfile(initial_velocity, initial_acceleration, …)`
    // 内部 `position = {0.0}` 丢弃全部已算结果）。后果是
    // **任何一段求解失败都会让整条计划塌成 end_speed**。
    //
    // 这是 2026-08-07 全部四种失败形态的共同放大器：曲率井 / 限速下调 /
    // 末端 5 mm 数值退化 / 特定不等距配置——触发器各不相同，后果永远是
    // "全程 end_speed 蠕行"。而实测证明触发器**追不完**：3 mm 点距比 10 mm
    // 密得多却正常，说明是配置相关的求解器边界情况，不可穷举。
    //
    // 改为保留失败点之前**已成功生成的前缀**，只从失败点续接降级剖面：
    // 前段正常速度得以保留，单次求解器边界情况不再是灾难。
    // 失败发生在首段时前缀仅剩状态点，行为自动退化为旧的全局兜底。
    const double resume_s = rs.empty() ? 0.0 : rs.back();
    const double resume_t = rt.empty() ? 0.0 : rt.back();
    const double resume_v = rv.empty() ? initial_velocity : rv.back();
    const double resume_a = ra.empty() ? initial_acceleration : ra.back();

    std::vector<double> fs, fv, fa, ft;
    braking_fallback = generateReachableBrakingProfile(
      resume_v, resume_a, a_max, d_max,
      max_jerk, ctrl_dt, fallback_target_velocity, fs, fv, fa, ft,
      stable_target_reachable);
    if (braking_fallback) {
      for (size_t k = 1; k < fs.size(); ++k) {
        rs.push_back(resume_s + fs[k]);
        rv.push_back(fv[k]);
        ra.push_back(fa[k]);
        rt.push_back(resume_t + ft[k]);
      }
    }
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
        [fallback_target_velocity](double velocity) {
          return velocity < fallback_target_velocity - kTerminalSpeedTolerance;
        });
      size_t stable_index = rv.size();
      for (size_t index = 0; index < rv.size(); ++index) {
        if (std::fabs(rv[index] - fallback_target_velocity) <= kTerminalSpeedTolerance &&
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
        (std::max(initial_velocity, fallback_target_velocity) * kControllerPreviewSteps +
        fallback_target_velocity) * ctrl_dt;
      // ⚠ **这是目前唯一能把 infeasible 清回 false 的地方**——制动兜底之后
      //   的最终判定。它承担了"退出判据"的职责，但语义与"兜底成没成功"耦合，
      //   没有独立的「已回到可行集」判断（doc8 §三 步 4 补）。
      feas.resolve(
        !stable_target_reachable || undershot ||
        stable_s >
        arc_s[terminal_start_i] + stable_position_tolerance + 1e-6,
        FeasibilityResult::Reason::kFallbackUnreachable);
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
  return feas;
}

// ===================== 插件 hook：Phase B（Ruckig jerk 限制剖面）=====================
void SCurveSpeedPlanner::generateProfile(
  std::vector<TrajPointWork> & work, double curr_vel, double curr_accel,
  double /*cruise_limit*/, bool terminal_brake_latched)
{
  if (work.size() < 2) {
    return;
  }

  // TERMINAL_SPATIAL_STAGES_BEGIN
  // 1) 终端距离只保留两条固定空间语义：
  //    · goal-terminal：在 terminal_creep_distance_（生产=5cm）边界达到0.05；
  //    · final-stop：进入动态0.05→0停车距离后才把终点速度切成0并锁存。
  // 高速执行余量不再前移5cm边界；障碍已有 clearance + zero_buffer，也不叠加
  // 另一套执行距离。这样减速可提前开始，但真正蠕行不会提前几十厘米。
  const auto initial_arc_s = computeCumulativeArcLengths(work);
  size_t hard_zero_index = work.size();
  for (size_t index = 1; index < work.size(); ++index) {
    if (work[index].velocity_upper_bound < 1e-3) {
      hard_zero_index = index;
      break;
    }
  }
  const bool obstacle_or_hard_stop =
    hard_zero_index < work.size() - 1;
  double active_end_speed = std::max(0.0, end_speed_);

  if (obstacle_or_hard_stop) {
    const double obstacle_boundary_s = initial_arc_s[hard_zero_index];
    const double conservative_acceleration = std::max(0.0, curr_accel);
    const double braking_distance = computeStableBrakingDistance(
      std::max(0.0, curr_vel), conservative_acceleration,
      accel_, std::fabs(decel_), max_jerk_, control_dt_, 0.0);
    last_planning_diagnostics_.terminal_branch =
      nav2_core::SpeedPlanningTerminalBranch::PATH_OBSTACLE_STOP;
    last_planning_diagnostics_.terminal_execution_reserve = 0.0;
    last_planning_diagnostics_.terminal_latched_reserve = 0.0;
    last_planning_diagnostics_.terminal_braking_distance = braking_distance;
    last_planning_diagnostics_.terminal_distance_available = obstacle_boundary_s;
    const double planning_cycle_guard =
      std::max(0.0, curr_vel) * control_dt_ +
      0.5 * std::max(0.0, curr_accel) * control_dt_ * control_dt_;
    last_planning_diagnostics_.terminal_brake_required =
      !std::isfinite(braking_distance) ||
      obstacle_boundary_s <= braking_distance + planning_cycle_guard + 1e-6;
    last_planning_diagnostics_.terminal_anchor_s = obstacle_boundary_s;
    active_end_speed = 0.0;
  } else if (terminal_speed_cap_ >= 1e-3 && terminal_creep_distance_ > 0.0) {
    const bool two_stage_terminal = end_speed_ + 1e-6 < terminal_speed_cap_;
    if (!terminal_brake_latched) {
      final_stop_latched_ = false;
    }
    const double terminal_boundary_s =
      std::max(0.0, initial_arc_s.back() - terminal_creep_distance_);
    const double conservative_acceleration = std::max(0.0, curr_accel);
    const double braking_distance = computeStableBrakingDistance(
      std::max(0.0, curr_vel), conservative_acceleration,
      accel_, std::fabs(decel_), max_jerk_, control_dt_, terminal_speed_cap_);
    const double planning_cycle_guard =
      std::max(0.0, curr_vel) * control_dt_ +
      0.5 * std::max(0.0, curr_accel) * control_dt_ * control_dt_;
    const double execution_delay_distance =
      std::max(0.0, curr_vel) * terminal_execution_delay_ +
      0.5 * std::max(0.0, curr_accel) *
      terminal_execution_delay_ * terminal_execution_delay_;
    if (terminal_brake_latched) {
      if (latched_creep_delay_distance_ < 0.0) {
        latched_creep_delay_distance_ = execution_delay_distance;
      }
    } else {
      latched_creep_delay_distance_ = -1.0;
    }
    const double creep_delay_distance = terminal_brake_latched ?
      latched_creep_delay_distance_ : execution_delay_distance;

    last_planning_diagnostics_.terminal_branch =
      nav2_core::SpeedPlanningTerminalBranch::GOAL_TERMINAL;
    last_planning_diagnostics_.terminal_execution_reserve = execution_delay_distance;
    last_planning_diagnostics_.terminal_latched_reserve =
      terminal_brake_latched ? latched_creep_delay_distance_ : -1.0;
    last_planning_diagnostics_.terminal_braking_distance = braking_distance;
    last_planning_diagnostics_.terminal_distance_available = terminal_boundary_s;
    last_planning_diagnostics_.terminal_brake_required = terminal_brake_latched ||
      !std::isfinite(braking_distance) ||
      terminal_boundary_s <=
      braking_distance + execution_delay_distance + planning_cycle_guard + 1e-6;
    const double creep_command_anchor = std::max(
      0.0, terminal_boundary_s - std::max(0.0, creep_delay_distance));
    last_planning_diagnostics_.terminal_anchor_s = creep_command_anchor;

    // 控制命令必须提前一个执行延迟开始收敛，实际底盘才会在固定5cm边界达到0.05。
    // 这里只有v×delay，没有旧版额外5cm safety margin；按现有路径采样点向后取整。
    for (size_t index = 1; index < work.size(); ++index) {
      if (initial_arc_s[index] + 1e-9 < creep_command_anchor) {
        continue;
      }
      lowerBound(work[index], terminal_speed_cap_, ConstraintSource::kTerminalCreep);
      if (work[index].region_type.empty()) {
        work[index].region_type = "terminal_reserve";
      }
    }

    if (two_stage_terminal) {
      const double final_braking_distance = computeStableBrakingDistance(
        std::max(std::max(0.0, curr_vel), terminal_speed_cap_), curr_accel,
        accel_, std::fabs(decel_), max_jerk_, control_dt_, end_speed_);
      const double delay_distance =
        std::max(std::max(0.0, curr_vel), terminal_speed_cap_) *
        terminal_execution_delay_;
      const double required_final_stop_distance =
        final_braking_distance + delay_distance;
      const double final_stop_entry = std::min(
        terminal_creep_distance_, required_final_stop_distance);
      if (terminal_brake_latched &&
        initial_arc_s.back() <= final_stop_entry + 1e-6)
      {
        final_stop_latched_ = true;
      }
      active_end_speed = final_stop_latched_ ?
        std::max(0.0, end_speed_) : terminal_speed_cap_;

      // Phase A把末点按end_speed_=0压低。最终停车尚未锁存时，只撤销这一条
      // EndBoundary，不抬高障碍/曲率等更严格约束。
      if (!final_stop_latched_ &&
        work.back().constraint_source == ConstraintSource::kEndBoundary)
      {
        work.back().velocity_upper_bound = terminal_speed_cap_;
        work.back().constraint_source = ConstraintSource::kTerminalCreep;
      }
    }
  } else if (!terminal_brake_latched) {
    final_stop_latched_ = false;
    latched_creep_delay_distance_ = -1.0;
  }
  // TERMINAL_SPATIAL_STAGES_END

  const size_t n = work.size();
  // 2) 累计弧长 + 包络（Phase A 已由基类完成：障碍/巡航/曲率/末端蠕行区，
  //    包络零点含障碍缓冲带，蠕行入口固定在终点前 terminal_creep_distance_）。
  std::vector<double> v_env(n, 0.0), arc_s(n, 0.0);
  v_env[0] = std::max(0.0, work[0].velocity_upper_bound);
  for (size_t i = 1; i < n; ++i) {
    const double dx = work[i].pose.pose.position.x - work[i - 1].pose.pose.position.x;
    const double dy = work[i].pose.pose.position.y - work[i - 1].pose.pose.position.y;
    arc_s[i] = arc_s[i - 1] + std::hypot(dx, dy);
    v_env[i] = std::max(0.0, work[i].velocity_upper_bound);
  }

  // 3) Ruckig 生成 jerk 限制剖面（非对称 a_max / d_max=|decel_|）。
  //    终点目标速度由空间阶段决定；障碍停止时函数内部强制 0。
  std::vector<double> v_out, a_out, t_out;
  // 已稳定进入蠕行阶段时，厘米级速度/加速度估计噪声不应被解释为必须先下穿
  // 0.05再恢复的真实制动状态；最终停车锁存后则保留真实a继续规划0速。
  double profile_start_acceleration = curr_accel;
  if (!final_stop_latched_ &&
    std::fabs(active_end_speed - terminal_speed_cap_) <= 1e-6 &&
    std::fabs(curr_vel - terminal_speed_cap_) <= 0.005 &&
    std::fabs(curr_accel) <= 0.05)
  {
    profile_start_acceleration = 0.0;
  }
  const FeasibilityResult terminal_feas = generateRuckigProfile(
    v_env, arc_s, curr_vel, profile_start_acceleration,
    accel_, std::fabs(decel_),
    max_jerk_, control_dt_,
    v_out, a_out, t_out, active_end_speed,
    terminal_brake_latched, terminal_speed_cap_);
  last_planning_diagnostics_.terminal_infeasible = terminal_feas.infeasible();
  last_planning_diagnostics_.terminal_infeasible_kind =
    static_cast<int>(terminal_feas.kind);
  last_planning_diagnostics_.terminal_infeasible_reason =
    static_cast<int>(terminal_feas.reason);
  last_planning_diagnostics_.terminal_result_provenance =
    static_cast<int>(terminal_feas.provenance);
  last_planning_diagnostics_.terminal_entry_reason =
    static_cast<int>(terminal_feas.entry_reason);
  last_planning_diagnostics_.solver_code = terminal_feas.solver_code;
  last_planning_diagnostics_.solver_v0 = terminal_feas.solver_v0;
  last_planning_diagnostics_.solver_a0 = terminal_feas.solver_a0;
  last_planning_diagnostics_.solver_vmax = terminal_feas.solver_vmax;
  last_planning_diagnostics_.solver_target_v = terminal_feas.solver_target_v;
  last_planning_diagnostics_.solver_distance = terminal_feas.solver_distance;

  // 4) 回填 work（曲率已由 Phase A3 填好）。
  for (size_t i = 0; i < n; ++i) {
    work[i].velocity_upper_bound = v_out[i];
    work[i].accel = a_out[i];
    work[i].rel_time = t_out[i];
  }
  fillJerkFromAccel(work);
}

}  // namespace nav2_speed_planner

PLUGINLIB_EXPORT_CLASS(
  nav2_speed_planner::SCurveSpeedPlanner, nav2_core::SpeedPlanner)
