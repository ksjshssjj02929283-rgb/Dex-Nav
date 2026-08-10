// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#ifndef NAV2_CORE__TRAJECTORY_PREVIEW_HPP_
#define NAV2_CORE__TRAJECTORY_PREVIEW_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <limits>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"

namespace nav2_core
{

struct TrajectoryPreviewPoint
{
  double x{0.0};
  double y{0.0};
  double speed_limit{0.0};
  double accel{0.0};
  double relative_time{0.0};
};

enum class TrajectoryPreviewMode
{
  TIME_ALIGNED,
  SPATIAL_FALLBACK
};

struct TrajectoryPreview
{
  double velocity{0.0};
  double acceleration{0.0};
  double relative_time{0.0};
  TrajectoryPreviewMode mode{TrajectoryPreviewMode::SPATIAL_FALLBACK};
};

inline double durationToSeconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) + 1e-9 * static_cast<double>(duration.nanosec);
}

// R44: RPP 与 PID 曾各自内联同一行公式（`max(|configured|, control_period)`），
// 两份拷贝各自读取自己的 controller_frequency，实际前瞻量按 20/10/14 Hz 分别漂移成
// 0.050/0.100/0.071s——方向与"补偿执行延迟"的需求相反（跑得越快补偿越少）。
// 收拢到唯一实现，任何未来的口径调整（例如把控制周期改成真实测得的执行延迟）
// 只需要改这一处，两个 consumer 不会再分别漂移。
inline double computeEffectiveTimeLookahead(double configured_time_lookahead, double control_period)
{
  return std::max(std::fabs(configured_time_lookahead), control_period);
}

template<typename PointT>
bool hasValidRelativeTime(const std::vector<PointT> & points)
{
  if (points.size() < 2 || !std::isfinite(points.front().relative_time) ||
    points.front().relative_time < -1e-9)
  {
    return false;
  }

  double previous = points.front().relative_time;
  for (size_t i = 1; i < points.size(); ++i) {
    const double current = points[i].relative_time;
    if (!std::isfinite(current) || current + 1e-9 < previous) {
      return false;
    }
    previous = current;
  }
  return points.back().relative_time > points.front().relative_time + 1e-9;
}

template<typename PointT>
bool sampleTrajectoryPreview(
  const std::vector<PointT> & points,
  double pose_x, double pose_y,
  bool relative_time_valid,
  double time_lookahead,
  double spatial_fallback_distance,
  TrajectoryPreview & preview)
{
  if (points.empty() || !std::isfinite(pose_x) || !std::isfinite(pose_y)) {
    return false;
  }

  double nearest_point_distance_sq = std::numeric_limits<double>::max();
  size_t nearest_point = 0;
  for (size_t i = 0; i < points.size(); ++i) {
    if (!std::isfinite(points[i].x) || !std::isfinite(points[i].y)) {
      return false;
    }
    const double dx = points[i].x - pose_x;
    const double dy = points[i].y - pose_y;
    const double distance_sq = dx * dx + dy * dy;
    if (distance_sq < nearest_point_distance_sq) {
      nearest_point_distance_sq = distance_sq;
      nearest_point = i;
    }
  }

  if (relative_time_valid) {
    constexpr double kSegmentLengthSqEpsilon = 1e-12;
    double nearest_segment_distance_sq = std::numeric_limits<double>::max();
    double projected_time = points[nearest_point].relative_time;
    for (size_t i = 0; i + 1 < points.size(); ++i) {
      const double segment_x = points[i + 1].x - points[i].x;
      const double segment_y = points[i + 1].y - points[i].y;
      const double segment_length_sq = segment_x * segment_x + segment_y * segment_y;
      if (segment_length_sq <= kSegmentLengthSqEpsilon) {
        continue;
      }
      const double ratio = std::clamp(
        ((pose_x - points[i].x) * segment_x + (pose_y - points[i].y) * segment_y) /
        segment_length_sq, 0.0, 1.0);
      const double projected_x = points[i].x + ratio * segment_x;
      const double projected_y = points[i].y + ratio * segment_y;
      const double dx = pose_x - projected_x;
      const double dy = pose_y - projected_y;
      const double distance_sq = dx * dx + dy * dy;
      if (distance_sq < nearest_segment_distance_sq) {
        nearest_segment_distance_sq = distance_sq;
        projected_time = points[i].relative_time + ratio *
          (points[i + 1].relative_time - points[i].relative_time);
      }
    }

    const double sample_time = projected_time + std::max(0.0, time_lookahead);
    const auto upper = std::upper_bound(
      points.begin(), points.end(), sample_time,
      [](double value, const PointT & point) {return value < point.relative_time;});
    if (upper == points.begin()) {
      preview.velocity = points.front().speed_limit;
      preview.acceleration = points.front().accel;
      preview.relative_time = points.front().relative_time;
    } else if (upper == points.end()) {
      preview.velocity = points.back().speed_limit;
      preview.acceleration = points.back().accel;
      preview.relative_time = points.back().relative_time;
    } else {
      const auto & second = *upper;
      const auto & first = *std::prev(upper);
      const double duration = second.relative_time - first.relative_time;
      const double ratio = duration > 1e-9 ?
        std::clamp((sample_time - first.relative_time) / duration, 0.0, 1.0) : 1.0;
      preview.velocity = first.speed_limit + ratio * (second.speed_limit - first.speed_limit);
      preview.acceleration = first.accel + ratio * (second.accel - first.accel);
      preview.relative_time = first.relative_time + ratio * duration;
    }
    if (!std::isfinite(preview.velocity) || !std::isfinite(preview.acceleration)) {
      return false;
    }
    preview.mode = TrajectoryPreviewMode::TIME_ALIGNED;
    return true;
  }

  size_t target = nearest_point;
  if (nearest_point + 1 < points.size()) {
    target = nearest_point + 1;
    double distance = 0.0;
    for (size_t i = nearest_point + 1; i < points.size(); ++i) {
      distance += std::hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
      if (points[i].speed_limit > points[target].speed_limit) {
        target = i;
      }
      if (distance > std::max(0.0, spatial_fallback_distance)) {
        break;
      }
    }
  }
  // 逐点 accel 的语义是"离开该点"的段加速度。上面按速度极大挑出的采样点，
  // 其"离开段"按定义就是峰后的制动段——把它当成采样点的加速度交出去，消费侧
  // 拿到的 (v, a) 自相矛盾："保持峰值速度，同时以最大减速度制动"。车已静止时
  // 该负前馈会被速度环的符号感知输出界钳成零指令，且零又成为下一周期的指令
  // 基准，逐周期自复现——机器人再也起不来（R49 末端 6cm 不可达）。
  // 与采样点速度自洽的是"到达该点"的那一段：要处在 v[target]，就该用把速度
  // 抬/压到 v[target] 的那一段加速度做前馈。恒加速度剖面的匀速/制动段内相邻
  // 两段取值相同，本改动只在段边界（峰值、制动起点）改变结果。
  // 例外：index 0 是状态戳（速度=实测），段 0→1 的加速度含实测量，用作前馈
  // 等于把反馈混进前馈（超速时会与速度环重复计一次制动）。因此 target==1 时
  // 仍取该点自身的段加速度——此时若 target 恰为峰值，剩余风险由消费侧速度环
  // 的反死锁兜底承担（见 PIDController/RPP::trackLinearVelocity）。
  preview.velocity = points[target].speed_limit;
  preview.acceleration = points[target >= 2 ? target - 1 : target].accel;
  preview.relative_time = points[target].relative_time;
  preview.mode = TrajectoryPreviewMode::SPATIAL_FALLBACK;
  return std::isfinite(preview.velocity) && std::isfinite(preview.acceleration);
}

}  // namespace nav2_core

#endif  // NAV2_CORE__TRAJECTORY_PREVIEW_HPP_
