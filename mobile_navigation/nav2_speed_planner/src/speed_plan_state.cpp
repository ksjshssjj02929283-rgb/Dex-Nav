#include "nav2_speed_planner/speed_plan_state.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "angles/angles.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

namespace nav2_speed_planner
{
namespace
{

bool isFinite(const geometry_msgs::msg::Twist & velocity)
{
  return std::isfinite(velocity.linear.x) &&
         std::isfinite(velocity.linear.y) &&
         std::isfinite(velocity.linear.z) &&
         std::isfinite(velocity.angular.x) &&
         std::isfinite(velocity.angular.y) &&
         std::isfinite(velocity.angular.z);
}

bool isFinite(const geometry_msgs::msg::Point & point)
{
  return std::isfinite(point.x) &&
         std::isfinite(point.y) &&
         std::isfinite(point.z);
}

bool isFinite(const geometry_msgs::msg::Quaternion & orientation)
{
  return std::isfinite(orientation.x) &&
         std::isfinite(orientation.y) &&
         std::isfinite(orientation.z) &&
         std::isfinite(orientation.w);
}

BoundaryDecision reseed(
  const geometry_msgs::msg::Twist & measured_velocity,
  BoundaryReason reason)
{
  BoundaryDecision decision;
  decision.boundary.velocity = measured_velocity;
  decision.reason = reason;
  return decision;
}

}  // namespace

const char * boundaryReasonName(BoundaryReason reason)
{
  switch (reason) {
    case BoundaryReason::FIRST_CYCLE:
      return "first_cycle";
    case BoundaryReason::EXPIRED:
      return "expired";
    case BoundaryReason::FRAME_MISMATCH:
      return "frame_mismatch";
    case BoundaryReason::GOAL_MISMATCH:
      return "goal_mismatch";
    case BoundaryReason::BRANCH_MISMATCH:
      return "branch_mismatch";
    case BoundaryReason::SPATIAL_MISMATCH:
      return "spatial_mismatch";
    case BoundaryReason::VELOCITY_MISMATCH:
      return "velocity_mismatch";
    case BoundaryReason::INVALID_NUMERIC:
      return "invalid_numeric";
    case BoundaryReason::EXPLICIT_INVALIDATION:
      return "explicit_invalidation";
  }
  return "unknown";
}

SpeedPlanState::SpeedPlanState()
: SpeedPlanState(Config())
{
}

SpeedPlanState::SpeedPlanState(const Config & config)
: config_(config)
{
}

void SpeedPlanState::configure(const Config & config)
{
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;
  resetUnlocked(BoundaryReason::FIRST_CYCLE);
}

BoundaryDecision SpeedPlanState::boundary(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::Twist & measured_velocity,
  const rclcpp::Time & now) const
{
  if (!isFinite(measured_velocity)) {
    return reseed(measured_velocity, BoundaryReason::INVALID_NUMERIC);
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!valid_) {
    return reseed(measured_velocity, invalidation_reason_);
  }
  if (path.poses.size() < 2 || accepted_trajectory_.points.size() < 2) {
    return reseed(measured_velocity, BoundaryReason::SPATIAL_MISMATCH);
  }
  if (now.get_clock_type() != accepted_trajectory_time_.get_clock_type()) {
    return reseed(measured_velocity, BoundaryReason::INVALID_NUMERIC);
  }

  const double age = (now - accepted_trajectory_time_).seconds();
  if (!std::isfinite(age) || age < 0.0) {
    return reseed(measured_velocity, BoundaryReason::INVALID_NUMERIC);
  }
  if (age > config_.max_age) {
    return reseed(measured_velocity, BoundaryReason::EXPIRED);
  }

  const auto path_frame = path.header.frame_id.empty() ?
    path.poses.front().header.frame_id : path.header.frame_id;
  const auto trajectory_frame = accepted_trajectory_.header.frame_id.empty() ?
    accepted_trajectory_.points.front().pose.header.frame_id :
    accepted_trajectory_.header.frame_id;
  if (path_frame.empty() || trajectory_frame != path_frame) {
    return reseed(measured_velocity, BoundaryReason::FRAME_MISMATCH);
  }

  const auto & new_goal = path.poses.back().pose;
  const auto & old_goal = accepted_trajectory_.points.back().pose.pose;
  if (!isFinite(new_goal.position) || !isFinite(old_goal.position) ||
    !isFinite(new_goal.orientation) || !isFinite(old_goal.orientation))
  {
    return reseed(measured_velocity, BoundaryReason::INVALID_NUMERIC);
  }
  const double old_goal_yaw = tf2::getYaw(old_goal.orientation);
  const double new_goal_yaw = tf2::getYaw(new_goal.orientation);
  if (!std::isfinite(old_goal_yaw) || !std::isfinite(new_goal_yaw)) {
    return reseed(measured_velocity, BoundaryReason::INVALID_NUMERIC);
  }
  const double goal_position_error = std::hypot(
    new_goal.position.x - old_goal.position.x,
    new_goal.position.y - old_goal.position.y);
  const double goal_yaw_error = std::fabs(
    angles::shortest_angular_distance(old_goal_yaw, new_goal_yaw));
  if (goal_position_error > config_.goal_position_tolerance ||
    goal_yaw_error > config_.goal_yaw_tolerance)
  {
    return reseed(measured_velocity, BoundaryReason::GOAL_MISMATCH);
  }

  constexpr double kSegmentLengthSqEpsilon = 1e-12;
  const auto & origin = path.poses.front().pose.position;
  if (!isFinite(origin)) {
    return reseed(measured_velocity, BoundaryReason::INVALID_NUMERIC);
  }
  double best_distance_sq = std::numeric_limits<double>::max();
  size_t best_segment = 0;
  double best_ratio = 0.0;
  double old_heading = 0.0;
  bool projection_valid = false;
  for (size_t index = 0; index + 1 < accepted_trajectory_.points.size(); ++index) {
    const auto & first = accepted_trajectory_.points[index].pose.pose.position;
    const auto & second = accepted_trajectory_.points[index + 1].pose.pose.position;
    if (!isFinite(first) || !isFinite(second)) {
      return reseed(measured_velocity, BoundaryReason::INVALID_NUMERIC);
    }
    const double segment_x = second.x - first.x;
    const double segment_y = second.y - first.y;
    const double length_sq = segment_x * segment_x + segment_y * segment_y;
    if (length_sq <= kSegmentLengthSqEpsilon) {
      continue;
    }
    const double ratio = std::clamp(
      ((origin.x - first.x) * segment_x + (origin.y - first.y) * segment_y) /
      length_sq, 0.0, 1.0);
    const double distance_x = origin.x - (first.x + ratio * segment_x);
    const double distance_y = origin.y - (first.y + ratio * segment_y);
    const double distance_sq = distance_x * distance_x + distance_y * distance_y;
    if (distance_sq < best_distance_sq) {
      best_distance_sq = distance_sq;
      best_segment = index;
      best_ratio = ratio;
      old_heading = std::atan2(segment_y, segment_x);
      projection_valid = true;
    }
  }
  if (!projection_valid ||
    std::sqrt(best_distance_sq) > config_.max_spatial_error)
  {
    return reseed(measured_velocity, BoundaryReason::SPATIAL_MISMATCH);
  }

  double new_heading = 0.0;
  bool new_heading_valid = false;
  for (size_t index = 1; index < path.poses.size(); ++index) {
    const auto & point = path.poses[index].pose.position;
    if (!isFinite(point)) {
      return reseed(measured_velocity, BoundaryReason::INVALID_NUMERIC);
    }
    const double delta_x = point.x - origin.x;
    const double delta_y = point.y - origin.y;
    if (delta_x * delta_x + delta_y * delta_y > kSegmentLengthSqEpsilon) {
      new_heading = std::atan2(delta_y, delta_x);
      new_heading_valid = true;
      break;
    }
  }
  if (!new_heading_valid ||
    std::fabs(angles::shortest_angular_distance(old_heading, new_heading)) >
    config_.branch_heading_tolerance)
  {
    return reseed(measured_velocity, BoundaryReason::BRANCH_MISMATCH);
  }

  const auto & first = accepted_trajectory_.points[best_segment];
  const auto & second = accepted_trajectory_.points[best_segment + 1];
  if (!std::isfinite(first.longitudinal_velocity) ||
    !std::isfinite(second.longitudinal_velocity) ||
    !std::isfinite(first.acceleration) ||
    !std::isfinite(second.acceleration))
  {
    return reseed(measured_velocity, BoundaryReason::INVALID_NUMERIC);
  }
  const double sampled_velocity = first.longitudinal_velocity + best_ratio *
    (second.longitudinal_velocity - first.longitudinal_velocity);
  const double sampled_acceleration = first.acceleration + best_ratio *
    (second.acceleration - first.acceleration);
  if (!std::isfinite(sampled_velocity) || !std::isfinite(sampled_acceleration)) {
    return reseed(measured_velocity, BoundaryReason::INVALID_NUMERIC);
  }
  if (std::fabs(measured_velocity.linear.x - sampled_velocity) >
    config_.max_velocity_error)
  {
    return reseed(measured_velocity, BoundaryReason::VELOCITY_MISMATCH);
  }

  BoundaryDecision decision;
  decision.boundary.velocity = measured_velocity;
  decision.boundary.acceleration = sampled_acceleration;
  decision.boundary.acceleration_valid = true;
  decision.boundary.source_stamp = accepted_trajectory_time_;
  decision.boundary.source_age = age;
  return decision;
}

void SpeedPlanState::accept(
  const nav_interfaces::msg::RobotTrajectory & trajectory,
  const rclcpp::Time & now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (trajectory.points.empty()) {
    resetUnlocked(BoundaryReason::INVALID_NUMERIC);
    return;
  }
  accepted_trajectory_ = trajectory;
  accepted_trajectory_time_ = now;
  valid_ = true;
}

void SpeedPlanState::reset(BoundaryReason reason)
{
  std::lock_guard<std::mutex> lock(mutex_);
  resetUnlocked(reason);
}

bool SpeedPlanState::valid() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return valid_;
}

void SpeedPlanState::resetUnlocked(BoundaryReason reason)
{
  accepted_trajectory_ = nav_interfaces::msg::RobotTrajectory();
  accepted_trajectory_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  valid_ = false;
  invalidation_reason_ = reason;
}

}  // namespace nav2_speed_planner
