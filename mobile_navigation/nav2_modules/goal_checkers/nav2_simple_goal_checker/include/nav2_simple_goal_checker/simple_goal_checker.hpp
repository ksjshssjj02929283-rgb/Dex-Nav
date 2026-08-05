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

#ifndef SIMPLE_GOAL_CHECKER__SIMPLE_GOAL_CHECKER_HPP_
#define SIMPLE_GOAL_CHECKER__SIMPLE_GOAL_CHECKER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_simple_goal_checker
{

/**
 * @class SimpleGoalChecker
 * @brief Goal Checker plugin that only checks the position and orientation difference.
 *
 * Reaches the goal once the XY position is within the (squared) xy tolerance and the
 * yaw difference is within the yaw tolerance. When stateful, the XY check is latched off
 * after it is first satisfied so only the orientation needs to keep being satisfied.
 */
class SimpleGoalChecker : public nav2_core::GoalChecker
{
public:
  SimpleGoalChecker();

  // Standard GoalChecker Interface
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void reset() override;

  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;

  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override;

protected:
  double xy_goal_tolerance_, yaw_goal_tolerance_;
  bool stateful_, check_xy_;
  // Cached squared xy goal tolerance to avoid a sqrt on every check
  double xy_goal_tolerance_sq_;
  std::string plugin_name_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  rclcpp::Logger logger_{rclcpp::get_logger("SimpleGoalChecker")};

  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};

}  // namespace nav2_simple_goal_checker

#endif  // SIMPLE_GOAL_CHECKER__SIMPLE_GOAL_CHECKER_HPP_
