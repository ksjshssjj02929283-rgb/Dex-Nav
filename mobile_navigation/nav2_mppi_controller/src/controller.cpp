// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#include <stdint.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include "nav2_mppi_controller/controller.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "tf2/utils.h"

// #define BENCHMARK_TESTING

namespace nav2_mppi_controller
{
namespace
{

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

double poseDistance(const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b)
{
  return std::hypot(a.position.x - b.position.x, a.position.y - b.position.y);
}

}  // namespace

void MPPIController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  costmap_ros_ = costmap_ros;
  tf_buffer_ = tf;
  name_ = name;
  parameters_handler_ = std::make_unique<ParametersHandler>(parent);

  auto node = parent_.lock();
  clock_ = node->get_clock();
  last_time_called_ = clock_->now();
  // Get high-level controller parameters
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(visualize_, "visualize", false);
  getParam(reset_period_, "reset_period", 1.0);
  getParam(tangent_phase_control_enabled_, "tangent_phase_control_enabled", false);
  getParam(tangent_rotate_kp_, "tangent_rotate_kp", 1.5);
  getParam(tangent_rotate_max_speed_, "tangent_rotate_max_speed", 0.5);
  getParam(tangent_rotate_min_speed_, "tangent_rotate_min_speed", 0.08);
  getParam(tangent_rotate_max_accel_, "tangent_rotate_max_accel", 0.7);
  getParam(tangent_start_yaw_tolerance_, "tangent_start_yaw_tolerance", 0.03);
  getParam(tangent_goal_yaw_tolerance_, "tangent_goal_yaw_tolerance", 0.03);
  getParam(tangent_goal_xy_tolerance_, "tangent_goal_xy_tolerance", 0.05);
  getParam(
    tangent_stopped_linear_velocity_, "tangent_stopped_linear_velocity", 0.03);
  getParam(
    tangent_stopped_angular_velocity_, "tangent_stopped_angular_velocity", 0.03);
  getParam(
    tangent_marker_position_tolerance_, "tangent_marker_position_tolerance", 1e-4);
  getParam(
    tangent_rotation_simulate_ahead_time_,
    "tangent_rotation_simulate_ahead_time", 0.5);

  double controller_frequency = 10.0;
  node->get_parameter("controller_frequency", controller_frequency);
  control_duration_ = 1.0 / std::max(controller_frequency, 1.0);
  tangent_command_frame_ = costmap_ros_->getBaseFrameID();
  tangent_collision_checker_ = std::make_unique<
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(
    costmap_ros_->getCostmap());

  // Configure composed objects
  optimizer_.initialize(parent_, name_, costmap_ros_, parameters_handler_.get());
  path_handler_.initialize(parent_, name_, costmap_ros_, tf_buffer_, parameters_handler_.get());
  trajectory_visualizer_.on_configure(
    parent_, name_,
    costmap_ros_->getGlobalFrameID(), parameters_handler_.get());

  RCLCPP_INFO(
    logger_, "Configured MPPI Controller: %s (tangent phase control=%s)",
    name_.c_str(), tangent_phase_control_enabled_ ? "enabled" : "disabled");
}

void MPPIController::cleanup()
{
  optimizer_.shutdown();
  trajectory_visualizer_.on_cleanup();
  tangent_collision_checker_.reset();
  tangent_phase_ = TangentPhase::DISABLED;
  parameters_handler_.reset();
  RCLCPP_INFO(logger_, "Cleaned up MPPI Controller: %s", name_.c_str());
}

void MPPIController::activate()
{
  trajectory_visualizer_.on_activate();
  parameters_handler_->start();
  RCLCPP_INFO(logger_, "Activated MPPI Controller: %s", name_.c_str());
}

void MPPIController::deactivate()
{
  trajectory_visualizer_.on_deactivate();
  RCLCPP_INFO(logger_, "Deactivated MPPI Controller: %s", name_.c_str());
}

void MPPIController::reset()
{
  optimizer_.reset();
}

// geometry_msgs::msg::TwistStamped MPPIController::computeVelocityCommands(
//   const geometry_msgs::msg::PoseStamped & robot_pose,
//   const geometry_msgs::msg::Twist & robot_speed,
//   nav2_core::GoalChecker * goal_checker)
// {
// #ifdef BENCHMARK_TESTING
//   auto start = std::chrono::system_clock::now();
// #endif

//   if (clock_->now() - last_time_called_ > rclcpp::Duration::from_seconds(reset_period_)) {
//     reset();
//   }
//   last_time_called_ = clock_->now();

//   std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());
//   nav_msgs::msg::Path transformed_plan = path_handler_.transformPath(robot_pose);

//   nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
//   std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*(costmap->getMutex()));
//   auto start_mppi = std::chrono::system_clock::now();
//   geometry_msgs::msg::TwistStamped cmd =
//     optimizer_.evalControl(robot_pose, robot_speed, transformed_plan, goal_checker);
//   auto end_mppi = std::chrono::system_clock::now();
//   auto duration_mppi =
//     std::chrono::duration_cast<std::chrono::milliseconds>(end_mppi - start_mppi).count();
//   RCLCPP_INFO(logger_, "Control evalControl time: %ld [ms]", duration_mppi);
// #ifdef BENCHMARK_TESTING
//   auto end = std::chrono::system_clock::now();
//   auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
//   RCLCPP_INFO(logger_, "Control loop execution time: %ld [ms]", duration);
// #endif

//   if (visualize_) {
//     visualize(std::move(transformed_plan));
//   }

//   return cmd;
// }
geometry_msgs::msg::TwistStamped MPPIController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  nav2_core::GoalChecker * goal_checker)
{
  constexpr double kSlowTimingWarnMs = 100.0;  // 100.0
  auto t_total_start = std::chrono::high_resolution_clock::now();

  if (clock_->now() - last_time_called_ > rclcpp::Duration::from_seconds(reset_period_)) {
    reset();
  }
  last_time_called_ = clock_->now();

  std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());

  if (auto tangent_command = computeTangentPhaseCommand(robot_pose, robot_speed)) {
    return *tangent_command;
  }

  auto t_transform_start = std::chrono::high_resolution_clock::now();
  nav_msgs::msg::Path transformed_plan = path_handler_.transformPath(robot_pose);
  auto t_transform_end = std::chrono::high_resolution_clock::now();
  double transform_ms =
    std::chrono::duration<double, std::milli>(t_transform_end - t_transform_start).count();

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();

  auto t_lock_start = std::chrono::high_resolution_clock::now();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*(costmap->getMutex()));
  auto t_lock_end = std::chrono::high_resolution_clock::now();
  double wait_lock_ms =
    std::chrono::duration<double, std::milli>(t_lock_end - t_lock_start).count();

  auto t_optimize_start = std::chrono::high_resolution_clock::now();
  geometry_msgs::msg::TwistStamped cmd =
    optimizer_.evalControl(robot_pose, robot_speed, transformed_plan, goal_checker);
  auto t_optimize_end = std::chrono::high_resolution_clock::now();
  double optimize_ms =
    std::chrono::duration<double, std::milli>(t_optimize_end - t_optimize_start).count();

  double visualize_ms = 0.0;
  if (visualize_) {
    auto t_visualize_start = std::chrono::high_resolution_clock::now();
    visualize(std::move(transformed_plan));
    auto t_visualize_end = std::chrono::high_resolution_clock::now();
    visualize_ms =
      std::chrono::duration<double, std::milli>(t_visualize_end - t_visualize_start).count();
  }

  auto t_total_end = std::chrono::high_resolution_clock::now();
  double total_ms =
    std::chrono::duration<double, std::milli>(t_total_end - t_total_start).count();

  if (
    total_ms >= kSlowTimingWarnMs || transform_ms >= kSlowTimingWarnMs ||
    wait_lock_ms >= kSlowTimingWarnMs || optimize_ms >= kSlowTimingWarnMs ||
    visualize_ms >= kSlowTimingWarnMs)
  {
    RCLCPP_WARN(
      logger_,
      "MPPI timing (ms): transform=%.3f wait_costmap_lock=%.3f optimize=%.3f visualize=%.3f "
      "total=%.3f",
      transform_ms, wait_lock_ms, optimize_ms, visualize_ms, total_ms);
  }

  return cmd;
}


void MPPIController::visualize(nav_msgs::msg::Path transformed_plan)
{
  trajectory_visualizer_.add(optimizer_.getGeneratedTrajectories(), "Candidate Trajectories");
  trajectory_visualizer_.add(optimizer_.getOptimizedTrajectory(), "Optimal Trajectory");
  trajectory_visualizer_.visualize(std::move(transformed_plan));
}

void MPPIController::setPlan(const nav_msgs::msg::Path & path)
{
  nav_msgs::msg::Path cruise_path;
  if (configureTangentPlan(path, cruise_path)) {
    path_handler_.setPath(cruise_path);
  } else {
    tangent_phase_ = TangentPhase::DISABLED;
    path_handler_.setPath(path);
  }
}

bool MPPIController::configureTangentPlan(
  const nav_msgs::msg::Path & path, nav_msgs::msg::Path & cruise_path)
{
  if (path.poses.size() < 4) {
    return false;
  }

  const auto & first = path.poses.front().pose;
  const auto & cruise_first = path.poses[1].pose;
  const auto & cruise_last = path.poses[path.poses.size() - 2].pose;
  const auto & last = path.poses.back().pose;
  if (poseDistance(first, cruise_first) > tangent_marker_position_tolerance_ ||
    poseDistance(cruise_last, last) > tangent_marker_position_tolerance_)
  {
    return false;
  }

  const bool same_task =
    tangent_phase_ != TangentPhase::DISABLED && tangent_phase_ != TangentPhase::COMPLETE &&
    poseDistance(tangent_goal_pose_, last) <= tangent_goal_xy_tolerance_ &&
    std::abs(
    shortestAngularDistance(
      tf2::getYaw(tangent_goal_pose_.orientation),
      tf2::getYaw(last.orientation))) <= tangent_goal_yaw_tolerance_;

  tangent_start_pose_ = first;
  tangent_cruise_start_pose_ = cruise_first;
  tangent_goal_pose_ = last;

  cruise_path = path;
  cruise_path.poses.erase(cruise_path.poses.begin());
  cruise_path.poses.pop_back();

  if (!tangent_phase_control_enabled_) {
    // Marker generation and phase control are independently opt-in. If only
    // the planner side was enabled, strip the markers and retain legacy MPPI
    // behavior with a strict final goal yaw.
    cruise_path.poses.back().pose.orientation = last.orientation;
    tangent_phase_ = TangentPhase::DISABLED;
    return true;
  }

  if (!same_task) {
    tangent_phase_ = TangentPhase::START_STOP;
    optimizer_.reset();
    RCLCPP_INFO(
      logger_,
      "Tangent phase task started: start_yaw=%.3f tangent_yaw=%.3f goal_yaw=%.3f",
      tf2::getYaw(tangent_start_pose_.orientation),
      tf2::getYaw(tangent_cruise_start_pose_.orientation),
      tf2::getYaw(tangent_goal_pose_.orientation));
  } else {
    RCLCPP_DEBUG(
      logger_, "Tangent replan received; preserving phase %s",
      tangentPhaseName(tangent_phase_));
  }

  return true;
}

std::optional<geometry_msgs::msg::TwistStamped>
MPPIController::computeTangentPhaseCommand(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed)
{
  if (tangent_phase_ == TangentPhase::DISABLED) {
    return std::nullopt;
  }

  auto transition = [this](TangentPhase next) {
      if (tangent_phase_ != next) {
        RCLCPP_INFO(
          logger_, "Tangent phase: %s -> %s",
          tangentPhaseName(tangent_phase_), tangentPhaseName(next));
        tangent_phase_ = next;
      }
    };

  const double linear_speed = std::hypot(robot_speed.linear.x, robot_speed.linear.y);
  const double angular_speed = std::abs(robot_speed.angular.z);
  const double current_yaw = tf2::getYaw(robot_pose.pose.orientation);

  // Multiple zero-cost phase transitions may be taken in one controller tick.
  for (int transition_count = 0; transition_count < 6; ++transition_count) {
    switch (tangent_phase_) {
      case TangentPhase::START_STOP:
        if (linear_speed > tangent_stopped_linear_velocity_ ||
          angular_speed > tangent_stopped_angular_velocity_)
        {
          return makeStoppedCommand(robot_pose);
        }
        transition(TangentPhase::START_YAW_ALIGN);
        break;

      case TangentPhase::START_YAW_ALIGN: {
          const double error = shortestAngularDistance(
            current_yaw, tf2::getYaw(tangent_start_pose_.orientation));
          if (std::abs(error) > tangent_start_yaw_tolerance_) {
            return makeRotationCommand(error, robot_pose, robot_speed);
          }
          transition(TangentPhase::START_TANGENT_ALIGN);
          break;
        }

      case TangentPhase::START_TANGENT_ALIGN: {
          const double error = shortestAngularDistance(
            current_yaw, tf2::getYaw(tangent_cruise_start_pose_.orientation));
          if (std::abs(error) > tangent_start_yaw_tolerance_) {
            return makeRotationCommand(error, robot_pose, robot_speed);
          }
          transition(TangentPhase::START_SETTLE);
          break;
        }

      case TangentPhase::START_SETTLE:
        if (linear_speed > tangent_stopped_linear_velocity_ ||
          angular_speed > tangent_stopped_angular_velocity_)
        {
          return makeStoppedCommand(robot_pose);
        }
        transition(TangentPhase::TRACK);
        return makeStoppedCommand(robot_pose);

      case TangentPhase::TRACK:
        if (poseDistance(robot_pose.pose, tangent_goal_pose_) <= tangent_goal_xy_tolerance_) {
          transition(TangentPhase::GOAL_STOP);
          return makeStoppedCommand(robot_pose);
        }
        return std::nullopt;

      case TangentPhase::GOAL_STOP:
        if (linear_speed > tangent_stopped_linear_velocity_ ||
          angular_speed > tangent_stopped_angular_velocity_)
        {
          return makeStoppedCommand(robot_pose);
        }
        transition(TangentPhase::GOAL_YAW_ALIGN);
        break;

      case TangentPhase::GOAL_YAW_ALIGN: {
          const double error = shortestAngularDistance(
            current_yaw, tf2::getYaw(tangent_goal_pose_.orientation));
          if (std::abs(error) > tangent_goal_yaw_tolerance_) {
            return makeRotationCommand(error, robot_pose, robot_speed);
          }
          transition(TangentPhase::COMPLETE);
          return makeStoppedCommand(robot_pose);
        }

      case TangentPhase::COMPLETE:
        return makeStoppedCommand(robot_pose);

      case TangentPhase::DISABLED:
        return std::nullopt;
    }
  }

  return makeStoppedCommand(robot_pose);
}

geometry_msgs::msg::TwistStamped MPPIController::makeStoppedCommand(
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = clock_->now();
  command.header.frame_id = tangent_command_frame_;
  (void)robot_pose;
  return command;
}

geometry_msgs::msg::TwistStamped MPPIController::makeRotationCommand(
  double yaw_error,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed)
{
  geometry_msgs::msg::TwistStamped command = makeStoppedCommand(robot_pose);
  double desired = std::clamp(
    tangent_rotate_kp_ * yaw_error,
    -tangent_rotate_max_speed_, tangent_rotate_max_speed_);
  if (std::abs(desired) < tangent_rotate_min_speed_) {
    desired = std::copysign(tangent_rotate_min_speed_, yaw_error);
  }

  const double max_delta = tangent_rotate_max_accel_ * control_duration_;
  command.twist.angular.z = std::clamp(
    desired,
    robot_speed.angular.z - max_delta,
    robot_speed.angular.z + max_delta);
  ensureRotationIsCollisionFree(robot_pose, command, yaw_error);
  return command;
}

void MPPIController::ensureRotationIsCollisionFree(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::TwistStamped & command,
  double yaw_error) const
{
  const double angular_speed = std::abs(command.twist.angular.z);
  if (!tangent_collision_checker_ || angular_speed < 1e-6) {
    return;
  }

  const double rotation_time = std::abs(yaw_error) / angular_speed;
  const double simulate_time = std::min(
    tangent_rotation_simulate_ahead_time_, rotation_time);
  const double initial_yaw = tf2::getYaw(robot_pose.pose.orientation);

  auto * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
  for (double time = control_duration_; time <= simulate_time + 1e-9;
    time += control_duration_)
  {
    const double yaw = initial_yaw + command.twist.angular.z * time;
    const double footprint_cost = tangent_collision_checker_->footprintCostAtPose(
      robot_pose.pose.position.x, robot_pose.pose.position.y, yaw,
      costmap_ros_->getRobotFootprint());
    if ((footprint_cost == static_cast<double>(nav2_costmap_2d::NO_INFORMATION) &&
      costmap_ros_->getLayeredCostmap()->isTrackingUnknown()) ||
      footprint_cost >= static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE))
    {
      throw nav2_core::PlannerException(
              "Tangent in-place rotation is not collision free");
    }
  }
}

const char * MPPIController::tangentPhaseName(TangentPhase phase)
{
  switch (phase) {
    case TangentPhase::DISABLED: return "disabled";
    case TangentPhase::START_STOP: return "start_stop";
    case TangentPhase::START_YAW_ALIGN: return "start_yaw_align";
    case TangentPhase::START_TANGENT_ALIGN: return "start_tangent_align";
    case TangentPhase::START_SETTLE: return "start_settle";
    case TangentPhase::TRACK: return "track";
    case TangentPhase::GOAL_STOP: return "goal_stop";
    case TangentPhase::GOAL_YAW_ALIGN: return "goal_yaw_align";
    case TangentPhase::COMPLETE: return "complete";
  }
  return "unknown";
}

void MPPIController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  optimizer_.setSpeedLimit(speed_limit, percentage);
}

}  // namespace nav2_mppi_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_mppi_controller::MPPIController, nav2_core::Controller)
