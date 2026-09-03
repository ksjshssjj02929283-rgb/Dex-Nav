#include "nav2_speed_planner/core/initial_motion.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

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

double durationSeconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) + 1e-9 * static_cast<double>(duration.nanosec);
}

nav_interfaces::msg::TrajectoryPoint interpolateTrajectoryPoint(
  const nav_interfaces::msg::TrajectoryPoint & first,
  const nav_interfaces::msg::TrajectoryPoint & second,
  double ratio)
{
  ratio = std::clamp(ratio, 0.0, 1.0);
  nav_interfaces::msg::TrajectoryPoint point = ratio < 0.5 ? first : second;
  const auto interpolate = [ratio](double first_value, double second_value) {
      return first_value + ratio * (second_value - first_value);
    };
  point.pose.pose.position.x = interpolate(
    first.pose.pose.position.x, second.pose.pose.position.x);
  point.pose.pose.position.y = interpolate(
    first.pose.pose.position.y, second.pose.pose.position.y);
  point.pose.pose.position.z = interpolate(
    first.pose.pose.position.z, second.pose.pose.position.z);
  point.longitudinal_velocity = interpolate(
    first.longitudinal_velocity, second.longitudinal_velocity);
  point.angular_velocity = interpolate(first.angular_velocity, second.angular_velocity);
  point.acceleration = interpolate(first.acceleration, second.acceleration);
  point.jerk = interpolate(first.jerk, second.jerk);
  point.path_kappa = interpolate(first.path_kappa, second.path_kappa);
  const double relative_time = interpolate(
    durationSeconds(first.relative_time), durationSeconds(second.relative_time));
  point.relative_time = rclcpp::Duration::from_seconds(relative_time);
  return point;
}

struct PathProjection
{
  bool valid{false};
  size_t segment{0};
  double ratio{0.0};
  double distance{std::numeric_limits<double>::infinity()};
  geometry_msgs::msg::PoseStamped pose;
};

double pathLength(const nav_msgs::msg::Path & path)
{
  double length = 0.0;
  for (size_t index = 1; index < path.poses.size(); ++index) {
    const auto & previous = path.poses[index - 1].pose.position;
    const auto & current = path.poses[index].pose.position;
    length += std::hypot(current.x - previous.x, current.y - previous.y);
  }
  return length;
}

PathProjection projectOntoPath(
  const geometry_msgs::msg::Point & query,
  const nav_msgs::msg::Path & path)
{
  PathProjection projection;
  if (path.poses.size() < 2 || !isFinite(query)) {
    return projection;
  }
  double best_distance_sq = std::numeric_limits<double>::infinity();
  for (size_t index = 0; index + 1 < path.poses.size(); ++index) {
    const auto & first = path.poses[index].pose.position;
    const auto & second = path.poses[index + 1].pose.position;
    const double dx = second.x - first.x;
    const double dy = second.y - first.y;
    const double length_sq = dx * dx + dy * dy;
    if (length_sq <= 1e-12) {
      continue;
    }
    const double ratio = std::clamp(
      ((query.x - first.x) * dx + (query.y - first.y) * dy) / length_sq,
      0.0, 1.0);
    const double projected_x = first.x + ratio * dx;
    const double projected_y = first.y + ratio * dy;
    const double error_x = query.x - projected_x;
    const double error_y = query.y - projected_y;
    const double distance_sq = error_x * error_x + error_y * error_y;
    if (distance_sq < best_distance_sq) {
      best_distance_sq = distance_sq;
      projection.valid = true;
      projection.segment = index;
      projection.ratio = ratio;
      projection.pose = ratio < 0.5 ? path.poses[index] : path.poses[index + 1];
      projection.pose.pose.position.x = projected_x;
      projection.pose.pose.position.y = projected_y;
      projection.pose.pose.position.z = first.z + ratio * (second.z - first.z);
    }
  }
  if (projection.valid) {
    projection.distance = std::sqrt(best_distance_sq);
  }
  return projection;
}

bool buildAcceptedPrefix(
  const nav_interfaces::msg::RobotTrajectory & accepted,
  const nav_msgs::msg::Path & path,
  size_t best_segment,
  double best_ratio,
  double prefix_start_velocity,
  double consumed_acceleration,
  double consumed_relative_time,
  const InitialMotionArbiter::Config & config,
  InitialMotionDecision & decision)
{
  if (config.stitching_horizon <= 0.0 || accepted.points.size() < 2 ||
    best_segment + 1 >= accepted.points.size())
  {
    return false;
  }
  const double first_time = durationSeconds(accepted.points[best_segment].relative_time);
  const double second_time = durationSeconds(accepted.points[best_segment + 1].relative_time);
  if (!std::isfinite(first_time) || !std::isfinite(second_time) ||
    second_time <= first_time + 1e-9)
  {
    return false;
  }
  const double start_time = first_time + best_ratio * (second_time - first_time);
  if (!std::isfinite(consumed_relative_time) ||
    std::fabs(start_time - consumed_relative_time) > config.stitching_max_time_error)
  {
    return false;
  }
  // PRECISION_STITCHING_NEAR_TERMINAL_BEGIN
  // 远处保留0.2s有利于速度连续；终点前30cm则缩短到0.1s，最后10cm不再
  // 保留旧前缀。终端位置精度只有2cm，继续消费0.2s旧空间前缀会把新位置
  // 修正推迟到已经越过目标之后。
  const double remaining_path = pathLength(path);
  if (!std::isfinite(remaining_path) || remaining_path <= 0.10) {
    return false;
  }
  const double stitching_horizon = remaining_path <= 0.30 ?
    std::min(config.stitching_horizon, 0.10) : config.stitching_horizon;
  const double requested_end_time = start_time + stitching_horizon;
  // PRECISION_STITCHING_NEAR_TERMINAL_END
  const double available_end_time = durationSeconds(accepted.points.back().relative_time);
  const double end_time = std::min(requested_end_time, available_end_time);
  if (!std::isfinite(available_end_time) ||
    end_time - start_time < config.stitching_min_duration - 1e-9)
  {
    return false;
  }

  auto first_point = interpolateTrajectoryPoint(
    accepted.points[best_segment], accepted.points[best_segment + 1], best_ratio);
  first_point.pose = path.poses.front();
  // The prefix starts at the state consumed by the controller.  Spatial
  // interpolation is still used for geometry, but its velocity can be the
  // old trajectory's zero-speed state while the robot is already creeping.
  // Keeping the explicitly selected state velocity prevents the next
  // receding-horizon plan from restarting at v=0.
  first_point.longitudinal_velocity = prefix_start_velocity;
  first_point.acceleration = consumed_acceleration;
  first_point.relative_time = rclcpp::Duration::from_seconds(0.0);
  decision.stitching_prefix.header = accepted.header;
  decision.stitching_prefix.header.stamp = path.header.stamp;
  decision.stitching_prefix.points.push_back(std::move(first_point));

  size_t end_segment = best_segment;
  for (size_t index = best_segment + 1; index < accepted.points.size(); ++index) {
    const double point_time = durationSeconds(accepted.points[index].relative_time);
    if (!std::isfinite(point_time)) {
      decision.stitching_prefix = nav_interfaces::msg::RobotTrajectory();
      return false;
    }
    // Projection exactly on a segment endpoint already produced the prefix's
    // t=0 interpolation point. Do not append that same accepted point again
    // with another t=0, otherwise consumers see a zero-duration first segment.
    if (point_time <= start_time + 1e-9) {
      end_segment = index;
      continue;
    }
    if (point_time >= end_time - 1e-9) {
      end_segment = index - 1;
      break;
    }
    auto point = accepted.points[index];
    point.relative_time = rclcpp::Duration::from_seconds(point_time - start_time);
    decision.stitching_prefix.points.push_back(std::move(point));
    end_segment = index;
  }

  nav_interfaces::msg::TrajectoryPoint end_point;
  if (end_time >= available_end_time - 1e-9) {
    end_point = accepted.points.back();
  } else {
    while (end_segment + 1 < accepted.points.size() &&
      durationSeconds(accepted.points[end_segment + 1].relative_time) < end_time - 1e-9)
    {
      ++end_segment;
    }
    if (end_segment + 1 >= accepted.points.size()) {
      decision.stitching_prefix = nav_interfaces::msg::RobotTrajectory();
      return false;
    }
    const double segment_start = durationSeconds(accepted.points[end_segment].relative_time);
    const double segment_end = durationSeconds(accepted.points[end_segment + 1].relative_time);
    if (segment_end <= segment_start + 1e-9) {
      decision.stitching_prefix = nav_interfaces::msg::RobotTrajectory();
      return false;
    }
    end_point = interpolateTrajectoryPoint(
      accepted.points[end_segment], accepted.points[end_segment + 1],
      (end_time - segment_start) / (segment_end - segment_start));
  }
  end_point.relative_time = rclcpp::Duration::from_seconds(end_time - start_time);
  const double previous_time = durationSeconds(
    decision.stitching_prefix.points.back().relative_time);
  if (end_time - start_time > previous_time + 1e-9) {
    decision.stitching_prefix.points.push_back(std::move(end_point));
  } else {
    decision.stitching_prefix.points.back() = std::move(end_point);
  }
  if (decision.stitching_prefix.points.size() < 2) {
    decision.stitching_prefix = nav_interfaces::msg::RobotTrajectory();
    return false;
  }
  const auto & tail = decision.stitching_prefix.points.back();
  decision.stitching_boundary.velocity.linear.x = tail.longitudinal_velocity;
  decision.stitching_boundary.acceleration = tail.acceleration;
  decision.stitching_boundary.acceleration_valid = true;
  return std::isfinite(tail.longitudinal_velocity) && std::isfinite(tail.acceleration);
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
    case InitialMotionSource::LOW_SPEED_STARTUP_RESEED:
      return "low_speed_startup_reseed";
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
  config_.stitching_horizon = std::max(0.0, config_.stitching_horizon);
  config_.stitching_min_duration = std::clamp(
    config_.stitching_min_duration, 0.0, config_.stitching_horizon);
  config_.stitching_max_time_error = std::max(0.0, config_.stitching_max_time_error);
  config_.replan_dev = std::max(0.0, config_.replan_dev);
  config_.terminal_overspeed_replan_dev = std::max(
    0.0, config_.terminal_overspeed_replan_dev);
  resetUnlocked(InitialMotionSource::FIRST_CYCLE);
}

InitialMotionDecision InitialMotionArbiter::resolve(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::Twist & measured_velocity,
  const rclcpp::Time & now,
  bool terminal_brake_latched) const
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
    !std::isfinite(accepted_target_acceleration_) ||
    !std::isfinite(accepted_target_relative_time_))
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
  const double terminal_overspeed = measured_velocity.linear.x - sampled_velocity;
  const bool terminal_overspeed_mismatch = terminal_brake_latched &&
    terminal_overspeed > config_.terminal_overspeed_replan_dev;
  if (velocity_deviation > config_.replan_dev || terminal_overspeed_mismatch) {
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

  // The spatial sample is normally the safest cross-cycle velocity because it
  // keeps the new plan tied to the robot's path position.  At a standstill
  // startup, however, the robot can already be creeping while its pose is
  // still nearest the old trajectory's zero-speed first point.  Reusing that
  // zero creates a receding-horizon self-lock: every plan starts at v=0 and
  // the time-aligned controller keeps consuming only the first tiny ramp.
  // Use the measured velocity only in this bounded, non-terminal case.  Do
  // not apply it near the goal or after terminal braking is latched, where
  // preserving the conservative terminal profile is more important.
  constexpr double kStartupSampledVelocityEpsilon = 0.01;
  constexpr double kStartupMeasuredVelocityEpsilon = 0.001;
  constexpr double kStartupMeasuredVelocityMax = 0.05;
  constexpr double kStartupTerminalGuardDistance = 0.30;
  const double measured_velocity_x = measured_velocity.linear.x;
  const double remaining_path = pathLength(path);
  const bool low_speed_startup =
    !terminal_brake_latched &&
    std::isfinite(remaining_path) && remaining_path > kStartupTerminalGuardDistance &&
    std::fabs(sampled_velocity) <= kStartupSampledVelocityEpsilon &&
    std::fabs(measured_velocity_x) > kStartupMeasuredVelocityEpsilon &&
    std::fabs(measured_velocity_x) <= kStartupMeasuredVelocityMax;

  InitialMotionDecision decision;
  decision.measured_velocity = measured_velocity.linear.x;
  decision.sampled_velocity = sampled_velocity;
  decision.velocity_deviation = velocity_deviation;
  decision.state.velocity = measured_velocity;
  decision.state.velocity.linear.x = low_speed_startup ? measured_velocity_x : sampled_velocity;
  // Acceleration continues from the exact preview target consumed by the
  // controller, not from a spatial resample at the next local path origin.
  decision.state.acceleration = accepted_target_acceleration_;
  decision.state.acceleration_valid = true;
  decision.state.source_stamp = accepted_trajectory_time_;
  decision.state.source_age = age;
  decision.velocity_inherited = !low_speed_startup;
  if (low_speed_startup) {
    decision.reason = InitialMotionSource::LOW_SPEED_STARTUP_RESEED;
  }
  buildAcceptedPrefix(
    accepted_trajectory_, path, best_segment, best_ratio,
    decision.state.velocity.linear.x,
    accepted_target_acceleration_, accepted_target_relative_time_, config_, decision);
  return decision;
}

bool prepareTrajectoryStitching(
  const nav_msgs::msg::Path & path,
  const InitialMotionDecision & decision,
  double max_path_error,
  TrajectoryStitchingPlan & plan,
  std::string * rejection_reason)
{
  plan = TrajectoryStitchingPlan();
  const auto reject = [&](const std::string & reason) {
      if (rejection_reason != nullptr) {
        *rejection_reason = reason;
      }
      return false;
    };
  if (!decision.hasStitchingPrefix() || path.poses.size() < 2) {
    return reject("prefix_unavailable");
  }
  // PRECISION_STITCHING_NEAR_TERMINAL_BEGIN
  // 终点附近将前缀允许的空间误差收紧到不超过最终目标容差量级；远处保持
  // 原有配置，避免改变正常路径的连续性。
  max_path_error = std::max(0.0, max_path_error);
  const double remaining_path = pathLength(path);
  if (std::isfinite(remaining_path) && remaining_path <= 0.30) {
    max_path_error = std::min(max_path_error, 0.02);
  }
  // PRECISION_STITCHING_NEAR_TERMINAL_END
  double last_progress = -1.0;
  PathProjection tail_projection;
  for (const auto & point : decision.stitching_prefix.points) {
    const auto projection = projectOntoPath(point.pose.pose.position, path);
    if (!projection.valid || projection.distance > max_path_error) {
      return reject("prefix_path_mismatch");
    }
    const double progress = static_cast<double>(projection.segment) + projection.ratio;
    if (progress + 1e-6 < last_progress) {
      return reject("prefix_backtracks_on_path");
    }
    last_progress = progress;
    tail_projection = projection;
  }
  if (!tail_projection.valid || tail_projection.segment + 1 >= path.poses.size()) {
    return reject("prefix_reaches_path_end");
  }

  plan.prefix = decision.stitching_prefix;
  plan.prefix.header = path.header;
  plan.prefix.points.front().pose = path.poses.front();
  plan.prefix.points.back().pose = tail_projection.pose;
  plan.prefix.points.back().pose.header = path.header;
  plan.prefix_duration = durationSeconds(plan.prefix.points.back().relative_time);
  if (!std::isfinite(plan.prefix_duration) || plan.prefix_duration <= 0.0) {
    return reject("prefix_time_invalid");
  }

  plan.suffix_path.header = path.header;
  plan.suffix_path.poses.push_back(tail_projection.pose);
  plan.suffix_path.poses.back().header = path.header;
  for (size_t index = tail_projection.segment + 1; index < path.poses.size(); ++index) {
    const auto & previous = plan.suffix_path.poses.back().pose.position;
    const auto & current = path.poses[index].pose.position;
    if (std::hypot(current.x - previous.x, current.y - previous.y) > 1e-6 ||
      index + 1 == path.poses.size())
    {
      plan.suffix_path.poses.push_back(path.poses[index]);
    }
  }
  if (plan.suffix_path.poses.size() < 2) {
    return reject("suffix_too_short");
  }
  plan.suffix_boundary = decision.stitching_boundary;
  if (!std::isfinite(plan.suffix_boundary.velocity.linear.x) ||
    !std::isfinite(plan.suffix_boundary.acceleration))
  {
    return reject("suffix_boundary_invalid");
  }
  return true;
}

nav_interfaces::msg::RobotTrajectory concatenateStitchedTrajectory(
  const nav_interfaces::msg::RobotTrajectory & prefix,
  const nav_interfaces::msg::RobotTrajectory & suffix)
{
  if (prefix.points.empty()) {
    return suffix;
  }
  if (suffix.points.empty()) {
    return prefix;
  }
  nav_interfaces::msg::RobotTrajectory result = prefix;
  result.header = suffix.header;
  result.plan_id = suffix.plan_id;
  const double time_shift = durationSeconds(prefix.points.back().relative_time);
  for (size_t index = 1; index < suffix.points.size(); ++index) {
    auto point = suffix.points[index];
    point.relative_time = rclcpp::Duration::from_seconds(
      time_shift + durationSeconds(point.relative_time));
    result.points.push_back(std::move(point));
  }
  return result;
}

void InitialMotionArbiter::accept(
  const nav_interfaces::msg::RobotTrajectory & trajectory,
  double consumed_target_velocity,
  double consumed_target_acceleration,
  double consumed_target_relative_time,
  const rclcpp::Time & now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (trajectory.points.empty() ||
    !std::isfinite(consumed_target_velocity) ||
    !std::isfinite(consumed_target_acceleration) ||
    !std::isfinite(consumed_target_relative_time))
  {
    resetUnlocked(InitialMotionSource::INVALID_NUMERIC);
    return;
  }
  accepted_trajectory_ = trajectory;
  accepted_target_velocity_ = consumed_target_velocity;
  accepted_target_acceleration_ = consumed_target_acceleration;
  accepted_target_relative_time_ = consumed_target_relative_time;
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
  accepted_target_relative_time_ = 0.0;
  accepted_trajectory_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  valid_ = false;
  invalidation_reason_ = reason;
  consecutive_velocity_deviation_ = 0;
}

}  // namespace nav2_speed_planner
