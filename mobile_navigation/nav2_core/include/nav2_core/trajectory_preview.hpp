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
  preview.velocity = points[target].speed_limit;
  preview.acceleration = points[target].accel;
  preview.relative_time = points[target].relative_time;
  preview.mode = TrajectoryPreviewMode::SPATIAL_FALLBACK;
  return std::isfinite(preview.velocity) && std::isfinite(preview.acceleration);
}

}  // namespace nav2_core

#endif  // NAV2_CORE__TRAJECTORY_PREVIEW_HPP_
