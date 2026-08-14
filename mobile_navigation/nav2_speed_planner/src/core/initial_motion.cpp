#include "nav2_speed_planner/core/initial_motion.hpp"

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

InitialMotionDecision reseed(
  const geometry_msgs::msg::Twist & measured_velocity,
  InitialMotionSource reason)
{
  InitialMotionDecision decision;
  decision.state.velocity = measured_velocity;
  decision.measured_velocity = measured_velocity.linear.x;
  decision.reason = reason;
  return decision;
}

}  // namespace

const char * initialMotionSourceName(InitialMotionSource reason)
{
  switch (reason) {
    case InitialMotionSource::FIRST_CYCLE:
      return "first_cycle";
    case InitialMotionSource::EXPIRED:
      return "expired";
    case InitialMotionSource::FRAME_MISMATCH:
      return "frame_mismatch";
    case InitialMotionSource::GOAL_MISMATCH:
      return "goal_mismatch";
    case InitialMotionSource::BRANCH_MISMATCH:
      return "branch_mismatch";
    case InitialMotionSource::SPATIAL_MISMATCH:
      return "spatial_mismatch";
    case InitialMotionSource::VELOCITY_MISMATCH:
      return "velocity_mismatch";
    case InitialMotionSource::VELOCITY_MISMATCH_FAILSAFE:
      return "velocity_mismatch_failsafe";
    case InitialMotionSource::INVALID_NUMERIC:
      return "invalid_numeric";
    case InitialMotionSource::EXPLICIT_INVALIDATION:
      return "explicit_invalidation";
  }
  return "unknown";
}

InitialMotionArbiter::InitialMotionArbiter()
: InitialMotionArbiter(Config())
{
}

InitialMotionArbiter::InitialMotionArbiter(const Config & config)
: config_(config)
{
}

void InitialMotionArbiter::configure(const Config & config)
{
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;
  resetUnlocked(InitialMotionSource::FIRST_CYCLE);
}

InitialMotionDecision InitialMotionArbiter::resolve(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::Twist & measured_velocity,
  const rclcpp::Time & now) const
{
  if (!isFinite(measured_velocity)) {
    return reseed(measured_velocity, InitialMotionSource::INVALID_NUMERIC);
  }

  std::lock_guard<std::mutex> lock(mutex_);
  // 结构性 reseed（frame/goal/branch/spatial/…）说明当前上下文已经改变，
  // 连续速度偏离计数不再有意义，随 reseed 一起清零。
  const auto reseed_and_reset = [&](InitialMotionSource reason) {
      consecutive_velocity_deviation_ = 0;
      return reseed(measured_velocity, reason);
    };
  if (!valid_) {
    return reseed_and_reset(invalidation_reason_);
  }
  if (path.poses.size() < 2 || accepted_trajectory_.points.size() < 2) {
    return reseed_and_reset(InitialMotionSource::SPATIAL_MISMATCH);
  }
  if (now.get_clock_type() != accepted_trajectory_time_.get_clock_type()) {
    return reseed_and_reset(InitialMotionSource::INVALID_NUMERIC);
  }

  const double age = (now - accepted_trajectory_time_).seconds();
  if (!std::isfinite(age) || age < 0.0) {
    return reseed_and_reset(InitialMotionSource::INVALID_NUMERIC);
  }
  if (age > config_.max_age) {
    return reseed_and_reset(InitialMotionSource::EXPIRED);
  }

  const auto path_frame = path.header.frame_id.empty() ?
    path.poses.front().header.frame_id : path.header.frame_id;
  const auto trajectory_frame = accepted_trajectory_.header.frame_id.empty() ?
    accepted_trajectory_.points.front().pose.header.frame_id :
    accepted_trajectory_.header.frame_id;
  if (path_frame.empty() || trajectory_frame != path_frame) {
    return reseed_and_reset(InitialMotionSource::FRAME_MISMATCH);
  }

  const auto & new_goal = path.poses.back().pose;
  const auto & old_goal = accepted_trajectory_.points.back().pose.pose;
  if (!isFinite(new_goal.position) || !isFinite(old_goal.position) ||
    !isFinite(new_goal.orientation) || !isFinite(old_goal.orientation))
  {
    return reseed_and_reset(InitialMotionSource::INVALID_NUMERIC);
  }
  const double old_goal_yaw = tf2::getYaw(old_goal.orientation);
  const double new_goal_yaw = tf2::getYaw(new_goal.orientation);
  if (!std::isfinite(old_goal_yaw) || !std::isfinite(new_goal_yaw)) {
    return reseed_and_reset(InitialMotionSource::INVALID_NUMERIC);
  }
  const double goal_position_error = std::hypot(
    new_goal.position.x - old_goal.position.x,
    new_goal.position.y - old_goal.position.y);
  const double goal_yaw_error = std::fabs(
    angles::shortest_angular_distance(old_goal_yaw, new_goal_yaw));
  if (goal_position_error > config_.goal_position_tolerance ||
    goal_yaw_error > config_.goal_yaw_tolerance)
  {
    return reseed_and_reset(InitialMotionSource::GOAL_MISMATCH);
  }

  constexpr double kSegmentLengthSqEpsilon = 1e-12;
  const auto & origin = path.poses.front().pose.position;
  if (!isFinite(origin)) {
    return reseed_and_reset(InitialMotionSource::INVALID_NUMERIC);
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
      return reseed_and_reset(InitialMotionSource::INVALID_NUMERIC);
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
    return reseed_and_reset(InitialMotionSource::SPATIAL_MISMATCH);
  }

  double new_heading = 0.0;
  bool new_heading_valid = false;
  for (size_t index = 1; index < path.poses.size(); ++index) {
    const auto & point = path.poses[index].pose.position;
    if (!isFinite(point)) {
      return reseed_and_reset(InitialMotionSource::INVALID_NUMERIC);
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
    return reseed_and_reset(InitialMotionSource::BRANCH_MISMATCH);
  }

  const auto & first = accepted_trajectory_.points[best_segment];
  const auto & second = accepted_trajectory_.points[best_segment + 1];
  if (!std::isfinite(first.longitudinal_velocity) ||
    !std::isfinite(second.longitudinal_velocity) ||
    !std::isfinite(accepted_target_velocity_) ||
    !std::isfinite(accepted_target_acceleration_))
  {
    return reseed_and_reset(InitialMotionSource::INVALID_NUMERIC);
  }
  const double sampled_velocity = first.longitudinal_velocity + best_ratio *
    (second.longitudinal_velocity - first.longitudinal_velocity);
  if (!std::isfinite(sampled_velocity)) {
    return reseed_and_reset(InitialMotionSource::INVALID_NUMERIC);
  }

  // P1-2（重构方案 §2.3）：结构性连续性（frame/goal/branch/spatial）已经全部成立。
  // 是否还要继承计划的速度值，取决于实测与计划采样值差多远：
  //   在 replan_dev 内     -> 正常态，v0/a0 都继承计划（不再无条件重锚定速度）
  //   超出但未达 failsafe  -> 保留加速度连续，但速度重锚定到实测（旧行为的折中）
  //   连续 N 周期都超出    -> 计划已系统性脱离现实，完整 reseed（v0/a0 都不再继承）
  const double velocity_deviation =
    std::fabs(measured_velocity.linear.x - sampled_velocity);
  if (velocity_deviation > config_.replan_dev) {
    if (++consecutive_velocity_deviation_ >= config_.failsafe_deviation_cycles) {
      auto failsafe = reseed_and_reset(InitialMotionSource::VELOCITY_MISMATCH_FAILSAFE);
      failsafe.measured_velocity = measured_velocity.linear.x;
      failsafe.sampled_velocity = sampled_velocity;
      failsafe.velocity_deviation = velocity_deviation;
      return failsafe;
    }
    InitialMotionDecision decision;
    decision.measured_velocity = measured_velocity.linear.x;
    decision.sampled_velocity = sampled_velocity;
    decision.velocity_deviation = velocity_deviation;
    decision.state.velocity = measured_velocity;
    decision.state.acceleration = accepted_target_acceleration_;
    decision.state.acceleration_valid = true;
    decision.state.source_stamp = accepted_trajectory_time_;
    decision.state.source_age = age;
    decision.velocity_inherited = false;
    decision.reason = InitialMotionSource::VELOCITY_MISMATCH;
    return decision;
  }
  consecutive_velocity_deviation_ = 0;

  InitialMotionDecision decision;
  decision.measured_velocity = measured_velocity.linear.x;
  decision.sampled_velocity = sampled_velocity;
  decision.velocity_deviation = velocity_deviation;
  decision.state.velocity = measured_velocity;
  decision.state.velocity.linear.x = sampled_velocity;
  // Acceleration continues from the exact preview target consumed by the
  // controller, not from a spatial resample at the next local path origin.
  decision.state.acceleration = accepted_target_acceleration_;
  decision.state.acceleration_valid = true;
  decision.state.source_stamp = accepted_trajectory_time_;
  decision.state.source_age = age;
  decision.velocity_inherited = true;
  return decision;
}

void InitialMotionArbiter::accept(
  const nav_interfaces::msg::RobotTrajectory & trajectory,
  double consumed_target_velocity,
  double consumed_target_acceleration,
  const rclcpp::Time & now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (trajectory.points.empty() ||
    !std::isfinite(consumed_target_velocity) ||
    !std::isfinite(consumed_target_acceleration))
  {
    resetUnlocked(InitialMotionSource::INVALID_NUMERIC);
    return;
  }
  accepted_trajectory_ = trajectory;
  accepted_target_velocity_ = consumed_target_velocity;
  accepted_target_acceleration_ = consumed_target_acceleration;
  accepted_trajectory_time_ = now;
  valid_ = true;
}

void InitialMotionArbiter::reset(InitialMotionSource reason)
{
  std::lock_guard<std::mutex> lock(mutex_);
  resetUnlocked(reason);
}

bool InitialMotionArbiter::valid() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return valid_;
}

void InitialMotionArbiter::resetUnlocked(InitialMotionSource reason)
{
  accepted_trajectory_ = nav_interfaces::msg::RobotTrajectory();
  accepted_target_velocity_ = 0.0;
  accepted_target_acceleration_ = 0.0;
  accepted_trajectory_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  valid_ = false;
  invalidation_reason_ = reason;
  consecutive_velocity_deviation_ = 0;
}

}  // namespace nav2_speed_planner
