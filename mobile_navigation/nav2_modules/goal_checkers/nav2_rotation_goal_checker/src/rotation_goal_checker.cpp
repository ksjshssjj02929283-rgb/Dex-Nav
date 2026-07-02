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
#include <memory>
#include <string>
#include <limits>

#include "nav2_rotation_goal_checker/rotation_goal_checker.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "angles/angles.h"
#include "tf2/utils.h"

namespace nav2_rotation_goal_checker
{

RotationGoalChecker::RotationGoalChecker()
: yaw_goal_tolerance_(0.25),
  rot_stopped_velocity_(0.05)
{
}

void RotationGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/)
{
  plugin_name_ = plugin_name;
  auto node = parent.lock();

  logger_ = node->get_logger();

  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".rot_stopped_velocity", rclcpp::ParameterValue(0.05));

  node->get_parameter(plugin_name + ".yaw_goal_tolerance", yaw_goal_tolerance_);
  node->get_parameter(plugin_name + ".rot_stopped_velocity", rot_stopped_velocity_);
}

void RotationGoalChecker::reset()
{
}

bool RotationGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & velocity)
{
  // 仅判断偏航容差，忽略位置
  double dyaw = angles::shortest_angular_distance(
    tf2::getYaw(query_pose.orientation),
    tf2::getYaw(goal_pose.orientation));

  if (!(std::fabs(dyaw) < yaw_goal_tolerance_)) {
    return false;
  }

  if (std::fabs(velocity.angular.z) > rot_stopped_velocity_) {
    return false;
  }

  RCLCPP_INFO(logger_, "RotationGoalChecker: Goal reached!");
  RCLCPP_INFO(logger_, "goal reached error: yaw_err: %.3f", std::fabs(dyaw));

  return true;
}

bool RotationGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  double invalid_field = std::numeric_limits<double>::lowest();

  pose_tolerance.position.x = invalid_field;
  pose_tolerance.position.y = invalid_field;
  pose_tolerance.position.z = invalid_field;
  pose_tolerance.orientation =
    nav2_util::geometry_utils::orientationAroundZAxis(yaw_goal_tolerance_);

  vel_tolerance.linear.x = invalid_field;
  vel_tolerance.linear.y = invalid_field;
  vel_tolerance.linear.z = invalid_field;

  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = rot_stopped_velocity_;

  return true;
}

}  // namespace nav2_rotation_goal_checker

PLUGINLIB_EXPORT_CLASS(nav2_rotation_goal_checker::RotationGoalChecker, nav2_core::GoalChecker)
