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

    const float wheel_speed = std::hypot(vx_i, vy_i);
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

    (void)wheel_speed;  // 与控制器一致保留 speed 变量，规划侧仅检查舵角跳变

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

  // 路径切线方向（世界系）与标称速度
  const double phi = std::atan2(dy, dx);
  const double v_nominal = config.nominal_path_speed;
  const double dt = ds / std::max(v_nominal, 1e-3);

  // 世界系速度投影到机体坐标系
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

  // 仿真向前推进：假定该段结束时舵角已到位（规划侧保守估计）
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

  // 可编辑 yaw 副本，避免反复读写 quaternion
  std::vector<double> yaws(n);
  for (size_t i = 0; i < n; ++i) {
    yaws[i] = getPathYaw(path, i);
  }

  const std::vector<double> desired_yaws = yaws;

  auto segment_max_jump = [&](size_t i, const std::vector<double> & profile) -> double {
      SwerveSteeringState state;
      state.reset();
      // 从路径起点重新仿真到 segment i，评估 i→i+1
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

  // ── 阶段 1：正向迭代 ───────────────────────────────────────────────────
  // 若某段舵角跳变过大，将 profile[i+1] 向 profile[i] 拉近（减小该段 Δyaw）
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

      // 按超出比例缩小该段 yaw 变化；0.85 为保守系数，避免反复震荡
      const double scale = (config.max_wheel_angle_step / jump) * 0.85;
      yaws[i + 1] = yaws[i] + dyaw * scale;
      changed = true;
    }

    if (!changed) {
      break;
    }
  }

  // ── 阶段 2：锚定终点 yaw，自后向前修正 ─────────────────────────────────
  // 保证最终仍到达 goal_yaw，同时尽量满足舵角连续性
  yaws[n - 1] = goal_yaw;

  for (int pass = 0; pass < config.max_smoothing_passes; ++pass) {
    bool changed = false;

    for (int i = static_cast<int>(n) - 2; i >= 0; --i) {
      const size_t idx = static_cast<size_t>(i);
      const double jump = segment_max_jump(idx, yaws);
      if (jump <= config.max_wheel_angle_step) {
        continue;
      }

      // 向后修正：把当前点 yaw 向“下一点”靠拢，使末段能转到位且舵角连续
      const double dyaw = shortestAngularDistance(yaws[idx], yaws[idx + 1]);
      if (std::abs(dyaw) < 1e-6) {
        continue;
      }

      yaws[idx] += dyaw * 0.25;
      changed = true;
    }

    if (!changed) {
      break;
    }
  }

  // ── 阶段 3：在可行解附近拉回 desired 曲线（可选软约束）────────────────
  // 避免偏离 assignPathOrientations 过远，同时不破坏已满足的可行性
  for (size_t i = 0; i < n; ++i) {
    const double blend = 0.15;
    yaws[i] = yaws[i] * (1.0 - blend) + desired_yaws[i] * blend;
  }
  yaws[n - 1] = goal_yaw;

  // 最后一轮正向修复
  for (int pass = 0; pass < 3; ++pass) {
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
      const double scale = (config.max_wheel_angle_step / jump) * 0.85;
      if (i + 1 == n - 1) {
        // 末点 yaw 已锚定 goal，只调整倒数第二点
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
