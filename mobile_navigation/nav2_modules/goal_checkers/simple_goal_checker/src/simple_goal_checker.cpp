// Copyright (c) 2017 Locus Robotics
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
#include <memory>
#include <string>
#include <limits>

#include "simple_goal_checker/simple_goal_checker.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "angles/angles.h"
#include "tf2/utils.h"

namespace simple_goal_checker
{

SimpleGoalChecker::SimpleGoalChecker()
: xy_goal_tolerance_(0.25),
  yaw_goal_tolerance_(0.25),
  stateful_(true),
  check_xy_(true),
  xy_goal_tolerance_sq_(0.0625)
{
}

void SimpleGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/)
{
  plugin_name_ = plugin_name;
  auto node = parent.lock();

  logger_ = node->get_logger();

  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".xy_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".stateful", rclcpp::ParameterValue(true));

  node->get_parameter(plugin_name + ".xy_goal_tolerance", xy_goal_tolerance_);
  node->get_parameter(plugin_name + ".yaw_goal_tolerance", yaw_goal_tolerance_);
  node->get_parameter(plugin_name + ".stateful", stateful_);

  xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;
}

void SimpleGoalChecker::reset()
{
  check_xy_ = true;
}

bool SimpleGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist &)
{
  double error_sq = 0.0;
  // 先判断距离
  if (check_xy_) {
    double dx = query_pose.position.x - goal_pose.position.x,
      dy = query_pose.position.y - goal_pose.position.y;

    error_sq = dx * dx + dy * dy;
    if (error_sq > xy_goal_tolerance_sq_) {
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

  if (fabs(dyaw) >= yaw_goal_tolerance_) {
    return false;
  }

  double position_error = sqrt(error_sq);

  RCLCPP_INFO(logger_, "SimpleGoalChecker: Goal reached!");
  RCLCPP_INFO(
    logger_, "goal reached error: position_err: %.3f, yaw_error: %.3f",
    position_error, fabs(dyaw));

  return true;
}

bool SimpleGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  double invalid_field = std::numeric_limits<double>::lowest();

  pose_tolerance.position.x = xy_goal_tolerance_;
  pose_tolerance.position.y = xy_goal_tolerance_;
  pose_tolerance.position.z = invalid_field;
  pose_tolerance.orientation =
    nav2_util::geometry_utils::orientationAroundZAxis(yaw_goal_tolerance_);

  vel_tolerance.linear.x = invalid_field;
  vel_tolerance.linear.y = invalid_field;
  vel_tolerance.linear.z = invalid_field;

  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = invalid_field;

  return true;
}

}  // namespace simple_goal_checker

PLUGINLIB_EXPORT_CLASS(simple_goal_checker::SimpleGoalChecker, nav2_core::GoalChecker)
