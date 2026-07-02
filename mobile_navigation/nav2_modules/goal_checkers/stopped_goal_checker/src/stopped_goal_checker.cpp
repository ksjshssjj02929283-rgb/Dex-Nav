// Copyright (c) 2024 Guo Linfeng
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
#include <functional>
#include <memory>
#include <string>
#include <limits>
#include <vector>

#include "stopped_goal_checker/stopped_goal_checker.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"

namespace stopped_goal_checker
{

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

StoppedGoalChecker::StoppedGoalChecker()
: SimpleGoalChecker(),
  rot_stopped_velocity_(0.25),
  trans_stopped_velocity_(0.25)
{
}

void StoppedGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  plugin_name_ = plugin_name;
  simple_goal_checker::SimpleGoalChecker::initialize(parent, plugin_name, costmap_ros);

  auto node = parent.lock();

  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".rot_stopped_velocity", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".trans_stopped_velocity", rclcpp::ParameterValue(0.05));

  node->get_parameter(plugin_name + ".rot_stopped_velocity", rot_stopped_velocity_);
  node->get_parameter(plugin_name + ".trans_stopped_velocity", trans_stopped_velocity_);

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&StoppedGoalChecker::dynamicParametersCallback, this, _1));
}

bool StoppedGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & velocity)
{
  bool ret = simple_goal_checker::SimpleGoalChecker::isGoalReached(query_pose, goal_pose, velocity);
  if (!ret) {
    return ret;
  }

  if (fabs(velocity.angular.z) > rot_stopped_velocity_ ||
    hypot(velocity.linear.x, velocity.linear.y) > trans_stopped_velocity_)
  {
    return false;
  }

  RCLCPP_INFO(logger_, "StoppedGoalChecker: Goal reached!");

  return true;
}

bool StoppedGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  double invalid_field = std::numeric_limits<double>::lowest();

  // populate the poses
  bool rtn = SimpleGoalChecker::getTolerances(pose_tolerance, vel_tolerance);

  // override the velocities
  vel_tolerance.linear.x = trans_stopped_velocity_;
  vel_tolerance.linear.y = trans_stopped_velocity_;
  vel_tolerance.linear.z = invalid_field;

  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = rot_stopped_velocity_;

  return true && rtn;
}

rcl_interfaces::msg::SetParametersResult
StoppedGoalChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".rot_stopped_velocity") {
        rot_stopped_velocity_ = parameter.as_double();
      } else if (name == plugin_name_ + ".trans_stopped_velocity") {
        trans_stopped_velocity_ = parameter.as_double();
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace stopped_goal_checker

PLUGINLIB_EXPORT_CLASS(stopped_goal_checker::StoppedGoalChecker, nav2_core::GoalChecker)
