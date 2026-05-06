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
#include <chrono>
#include "nav2_mppi_controller/controller.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

// #define BENCHMARK_TESTING

namespace nav2_mppi_controller
{

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

  // Configure composed objects
  optimizer_.initialize(parent_, name_, costmap_ros_, parameters_handler_.get());
  path_handler_.initialize(parent_, name_, costmap_ros_, tf_buffer_, parameters_handler_.get());
  trajectory_visualizer_.on_configure(
    parent_, name_,
    costmap_ros_->getGlobalFrameID(), parameters_handler_.get());

  RCLCPP_INFO(logger_, "Configured MPPI Controller: %s", name_.c_str());
}

void MPPIController::cleanup()
{
  optimizer_.shutdown();
  trajectory_visualizer_.on_cleanup();
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
//   auto duration_mppi = std::chrono::duration_cast<std::chrono::milliseconds>(end_mppi - start_mppi).count();
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
  constexpr double kSlowTimingWarnMs = 100.0;
  auto t_total_start = std::chrono::high_resolution_clock::now();

  if (clock_->now() - last_time_called_ > rclcpp::Duration::from_seconds(reset_period_)) {
    reset();
  }
  last_time_called_ = clock_->now();

  std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());

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
  path_handler_.setPath(path);
}

void MPPIController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  optimizer_.setSpeedLimit(speed_limit, percentage);
}

}  // namespace nav2_mppi_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_mppi_controller::MPPIController, nav2_core::Controller)
