// Copyright (c) 2024 Guo Linfeng
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

#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <string>

#include "nav2_high_precision_goal_checker/high_precision_goal_checker.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "angles/angles.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_high_precision_goal_checker
{

HighPrecisionGoalChecker::HighPrecisionGoalChecker()
: lat_goal_tolerance_(0.25),
  lon_goal_tolerance_(0.25),
  yaw_goal_tolerance_(0.25),
  stateful_(true),
  check_xy_(true),
  rot_stopped_velocity_(0.01),
  trans_stopped_velocity_(0.01)
{
}

void HighPrecisionGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/)
{
  plugin_name_ = plugin_name;
  auto node = parent.lock();

  logger_ = node->get_logger();

  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".lon_goal_tolerance", rclcpp::ParameterValue(0.02));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".lat_goal_tolerance", rclcpp::ParameterValue(0.04));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.02));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".rot_stopped_velocity", rclcpp::ParameterValue(0.01));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".trans_stopped_velocity", rclcpp::ParameterValue(0.01));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".stateful", rclcpp::ParameterValue(true));

  node->get_parameter(plugin_name + ".lon_goal_tolerance", lon_goal_tolerance_);
  node->get_parameter(plugin_name + ".lat_goal_tolerance", lat_goal_tolerance_);
  node->get_parameter(plugin_name + ".yaw_goal_tolerance", yaw_goal_tolerance_);
  node->get_parameter(plugin_name + ".rot_stopped_velocity", rot_stopped_velocity_);
  node->get_parameter(plugin_name + ".trans_stopped_velocity", trans_stopped_velocity_);
  node->get_parameter(plugin_name + ".stateful", stateful_);
}

void HighPrecisionGoalChecker::reset()
{
  check_xy_ = true;
}

bool HighPrecisionGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & velocity)
{
  tf2::Transform T_map_base, T_map_goal;
  tf2::fromMsg(goal_pose, T_map_goal);
  tf2::fromMsg(query_pose, T_map_base);
  // goal in base
  tf2::Transform T_base_goal = T_map_base.inverse() * T_map_goal;
  double lon_err = T_base_goal.getOrigin().x();
  double lat_err = T_base_goal.getOrigin().y();

  // 先判断距离
  if (check_xy_) {
    if (fabs(lon_err) > 0.1) {
      longitude_errors_.clear();
      return false;
    }

    longitude_errors_.push_back(std::fabs(lon_err));
    while (longitude_errors_.size() > 20) {
      longitude_errors_.pop_front();
    }

    double longitude_mean = std::numeric_limits<double>::max();
    double longitude_var = std::numeric_limits<double>::max();
    if (longitude_errors_.size() >= 20) {
      longitude_mean = CalculateMean(longitude_errors_);
      longitude_var = CalculateVariance(longitude_errors_, longitude_mean);
    }
    (void)longitude_var;

    if (fabs(lon_err) > lon_goal_tolerance_ ||
      fabs(lat_err) > lat_goal_tolerance_ ||
      fabs(longitude_mean) > lon_goal_tolerance_)
    {
      return false;
    }
    // We are within the window
    // If we are stateful, change the state.
    // 如果是 stateful，则在距离满足要求后，就不再判断距离是否还满足要求
    if (stateful_) {
      check_xy_ = false;
    }
  }

  double dyaw = angles::shortest_angular_distance(
    tf2::getYaw(query_pose.orientation),
    tf2::getYaw(goal_pose.orientation));

  if (fabs(dyaw) > 0.1) {
    theta_errors_.clear();
    return false;
  }

  theta_errors_.push_back(std::fabs(dyaw));
  while (theta_errors_.size() > 20) {
    theta_errors_.pop_front();
  }
  // 计算角度
  double theta_mean = std::numeric_limits<double>::max();
  double theta_var = std::numeric_limits<double>::max();
  if (theta_errors_.size() >= 20) {
    theta_mean = CalculateMean(theta_errors_);
    theta_var = CalculateVariance(theta_errors_, theta_mean);
  }
  (void)theta_var;

  if (!(fabs(dyaw) < yaw_goal_tolerance_) ||
    !(fabs(theta_mean) < yaw_goal_tolerance_))
  {
    return false;
  }

  if (fabs(velocity.angular.z) > rot_stopped_velocity_ ||
    hypot(velocity.linear.x, velocity.linear.y) > trans_stopped_velocity_)
  {
    return false;
  }

  RCLCPP_INFO(logger_, "HighPrecisionGoalChecker: Goal reached!");
  RCLCPP_INFO(
    logger_, "goal reached error: lon_err: %.3f, lat_err: %.3f, yaw_err: %.3f",
    lon_err, lat_err, dyaw);

  return true;
}

bool HighPrecisionGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  double invalid_field = std::numeric_limits<double>::lowest();

  pose_tolerance.position.x = lon_goal_tolerance_;
  pose_tolerance.position.y = lat_goal_tolerance_;
  pose_tolerance.position.z = invalid_field;
  pose_tolerance.orientation =
    nav2_util::geometry_utils::orientationAroundZAxis(yaw_goal_tolerance_);

  vel_tolerance.linear.x = trans_stopped_velocity_;
  vel_tolerance.linear.y = invalid_field;
  vel_tolerance.linear.z = invalid_field;

  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = rot_stopped_velocity_;

  return true;
}

double HighPrecisionGoalChecker::CalculateMean(const std::deque<double> & data)
{
  double sum = 0.0;
  for (const auto & value : data) {
    sum += value;
  }
  double avg = sum / static_cast<double>(data.size());
  return avg;
}

double HighPrecisionGoalChecker::CalculateVariance(const std::deque<double> & data, double mean)
{
  double sumSquaredDifferences = 0.0;
  for (const auto & value : data) {
    double difference = value - mean;
    sumSquaredDifferences += difference * difference;
  }
  double variance = sumSquaredDifferences / static_cast<double>(data.size());
  return variance;
}

}  // namespace nav2_high_precision_goal_checker

PLUGINLIB_EXPORT_CLASS(
  nav2_high_precision_goal_checker::HighPrecisionGoalChecker,
  nav2_core::GoalChecker)
