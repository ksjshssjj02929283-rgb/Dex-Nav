// Copyright 2026 DexForce
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_theta_star_planner/swerve_steering_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "nav2_util/geometry_utils.hpp"
#include "tf2/utils.h"

namespace nav2_theta_star_planner
{
namespace
{

float wrapPi(float angle)
{
  while (angle > static_cast<float>(M_PI)) {
    angle -= static_cast<float>(2.0 * M_PI);
  }
  while (angle <= static_cast<float>(-M_PI)) {
    angle += static_cast<float>(2.0 * M_PI);
  }
  return angle;
}

float angleDiff(float target, float current)
{
  return wrapPi(target - current);
}

double shortestAngularDistance(double from, double to)
{
  double delta = to - from;
  while (delta > M_PI) {
    delta -= 2.0 * M_PI;
  }
  while (delta < -M_PI) {
    delta += 2.0 * M_PI;
  }
  return delta;
}

void setPathYaw(nav_msgs::msg::Path & path, size_t index, double yaw)
{
  path.poses[index].pose.orientation =
    nav2_util::geometry_utils::orientationAroundZAxis(yaw);
}

double getPathYaw(const nav_msgs::msg::Path & path, size_t index)
{
  return tf2::getYaw(path.poses[index].pose.orientation);
}

double segmentLength(const nav_msgs::msg::Path & path, size_t i)
{
  const auto & p0 = path.poses[i].pose.position;
  const auto & p1 = path.poses[i + 1].pose.position;
  return std::hypot(p1.x - p0.x, p1.y - p0.y);
}

/** 按路径点间距与标称速度，计算该段允许的最大 |Δyaw| */
double maxYawStepForSegment(double ds, const SwerveSteeringConfig & config)
{
  if (ds < 1e-6) {
    return 0.0;
  }
  const double dt = ds / std::max(config.nominal_path_speed, 1e-3);
  return config.max_body_yaw_rate * dt;
}

}  // namespace

double computeSwerveWheelTargets(
  double vx, double vy, double vw,
  const SwerveSteeringConfig & config,
  const std::array<float, 4> & prev_wheel_angle,
  std::array<float, 4> & out_wheel_angle)
{
  const float L = static_cast<float>(config.wheelbase);
  const float W = static_cast<float>(config.track);
  const float steer_limit = static_cast<float>(config.steer_limit);
  const float pi = static_cast<float>(M_PI);

  double max_jump = 0.0;

  for (int i = 0; i < 4; ++i) {
    const bool is_right = (i == 1 || i == 3);
    const bool is_front = (i == 0 || i == 1);

    const float kx = is_right ? +1.f : -1.f;
    const float ky = is_front ? +1.f : -1.f;

    const float vx_i = static_cast<float>(vx) + kx * W * static_cast<float>(vw) * 0.5f;
    const float vy_i = static_cast<float>(vy) + ky * L * static_cast<float>(vw) * 0.5f;

    const float ang = std::atan2(vy_i, vx_i);

    // π 等价两解：与 mobile_cmd_omni_region.cpp Phase 0 相同
    const float angA = wrapPi(ang);
    const float angB = wrapPi(ang + pi);

    const bool validA = (std::abs(angA) <= steer_limit);
    const bool validB = (std::abs(angB) <= steer_limit);

    const float costA = validA ?
      std::abs(angleDiff(angA, prev_wheel_angle[i])) :
      std::numeric_limits<float>::max();
    const float costB = validB ?
      std::abs(angleDiff(angB, prev_wheel_angle[i])) :
      std::numeric_limits<float>::max();

    const float chosen = (costA <= costB) ? angA : angB;
    out_wheel_angle[i] = chosen;

    max_jump = std::max(max_jump, static_cast<double>(std::abs(angleDiff(chosen, prev_wheel_angle[i]))));
  }

  return max_jump;
}

BodyTwistSample bodyTwistFromPathSegment(
  double x0, double y0, double yaw0,
  double x1, double y1, double yaw1,
  const SwerveSteeringConfig & config)
{
  BodyTwistSample twist;

  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const double ds = std::hypot(dx, dy);

  if (ds < 1e-6) {
    return twist;
  }

  const double phi = std::atan2(dy, dx);
  const double v_nominal = config.nominal_path_speed;
  const double dt = ds / std::max(v_nominal, 1e-3);

  twist.vx = v_nominal * std::cos(phi - yaw0);
  twist.vy = v_nominal * std::sin(phi - yaw0);
  twist.wz = shortestAngularDistance(yaw0, yaw1) / dt;

  return twist;
}

bool isPathSegmentSteeringFeasible(
  double x0, double y0, double yaw0,
  double x1, double y1, double yaw1,
  const SwerveSteeringConfig & config,
  SwerveSteeringState & in_out_state)
{
  const BodyTwistSample twist = bodyTwistFromPathSegment(
    x0, y0, yaw0, x1, y1, yaw1, config);

  std::array<float, 4> target{};
  const double max_jump = computeSwerveWheelTargets(
    twist.vx, twist.vy, twist.wz,
    config, in_out_state.prev_wheel_angle, target);

  in_out_state.prev_wheel_angle = target;

  return max_jump <= config.max_wheel_angle_step + 1e-6;
}

size_t enforceSteeringContinuityOnPath(
  nav_msgs::msg::Path & path,
  double goal_yaw,
  const SwerveSteeringConfig & config)
{
  const size_t n = path.poses.size();
  if (n < 2) {
    return 0;
  }

  std::vector<double> yaws(n);
  for (size_t i = 0; i < n; ++i) {
    yaws[i] = getPathYaw(path, i);
  }

  const std::vector<double> desired_yaws = yaws;

  auto segment_max_jump = [&](size_t i, const std::vector<double> & profile) -> double {
      SwerveSteeringState state;
      state.reset();
      for (size_t k = 0; k <= i; ++k) {
        if (k + 1 >= n) {
          break;
        }
        const auto & p0 = path.poses[k].pose.position;
        const auto & p1 = path.poses[k + 1].pose.position;
        const BodyTwistSample twist = bodyTwistFromPathSegment(
          p0.x, p0.y, profile[k],
          p1.x, p1.y, profile[k + 1],
          config);
        std::array<float, 4> target{};
        const double jump = computeSwerveWheelTargets(
          twist.vx, twist.vy, twist.wz,
          config, state.prev_wheel_angle, target);
        if (k == i) {
          return jump;
        }
        state.prev_wheel_angle = target;
      }
      return 0.0;
    };

  // ── 阶段 1：按 max_body_yaw_rate 做 yaw 变化率限幅（正向）────────────────
  // 目的：把起终点转角摊到全程，而不是堆在末段；比反复“压扁”profile 更平滑。
  yaws[0] = desired_yaws[0];
  for (size_t i = 0; i + 1 < n; ++i) {
    const double ds = segmentLength(path, i);
    const double max_step = maxYawStepForSegment(ds, config);
    const double target_delta = shortestAngularDistance(yaws[i], desired_yaws[i + 1]);
    if (std::abs(target_delta) <= max_step) {
      yaws[i + 1] = desired_yaws[i + 1];
    } else {
      yaws[i + 1] = yaws[i] + std::copysign(max_step, target_delta);
    }
  }

  // ── 阶段 2：锚定终点 yaw，自后向前按变化率上限回填 ─────────────────────
  yaws[n - 1] = goal_yaw;
  for (int i = static_cast<int>(n) - 2; i >= 0; --i) {
    const size_t idx = static_cast<size_t>(i);
    const double ds = segmentLength(path, idx);
    const double max_step = maxYawStepForSegment(ds, config);
    const double needed = shortestAngularDistance(yaws[idx], yaws[idx + 1]);
    if (std::abs(needed) <= max_step) {
      continue;
    }
    yaws[idx] = yaws[idx + 1] - std::copysign(max_step, needed);
  }

  // ── 阶段 3：舵轮 IK 轻量微调（仅对仍超限的段，少量迭代）────────────────
  // 不再把 profile 整体压向起点 yaw，避免“旋转迟钝 + 路径拐角仍卡舵”。
  for (int pass = 0; pass < config.max_smoothing_passes; ++pass) {
    bool changed = false;

    for (size_t i = 0; i + 1 < n; ++i) {
      const double jump = segment_max_jump(i, yaws);
      if (jump <= config.max_wheel_angle_step) {
        continue;
      }

      const double dyaw = shortestAngularDistance(yaws[i], yaws[i + 1]);
      if (std::abs(dyaw) < 1e-6) {
        continue;
      }

      const double scale = std::min(1.0, (config.max_wheel_angle_step / jump) * 0.95);
      if (i + 1 == n - 1) {
        yaws[i] = yaws[i + 1] - dyaw * scale;
      } else {
        yaws[i + 1] = yaws[i] + dyaw * scale;
      }
      changed = true;
    }

    if (!changed) {
      break;
    }
  }

  yaws[n - 1] = goal_yaw;

  for (size_t i = 0; i < n; ++i) {
    setPathYaw(path, i, yaws[i]);
  }

  return n;
}

}  // namespace nav2_theta_star_planner
