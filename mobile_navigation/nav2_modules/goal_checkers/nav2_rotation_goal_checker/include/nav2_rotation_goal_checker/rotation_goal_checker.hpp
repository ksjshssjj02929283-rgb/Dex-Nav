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

#ifndef NAV2_ROTATION_GOAL_CHECKER__ROTATION_GOAL_CHECKER_HPP_
#define NAV2_ROTATION_GOAL_CHECKER__ROTATION_GOAL_CHECKER_HPP_

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

namespace nav2_rotation_goal_checker
{

/**
 * @class RotationGoalChecker
 * @brief Goal Checker plugin that only checks the orientation (yaw) difference,
 * ignoring the XY position. The goal is reached once the yaw error is within the
 * yaw tolerance and the angular velocity is below the rotational stopped velocity.
 */
class RotationGoalChecker : public nav2_core::GoalChecker
{
public:
  RotationGoalChecker();

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
  double yaw_goal_tolerance_;
  double rot_stopped_velocity_;
  std::string plugin_name_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  rclcpp::Logger logger_{rclcpp::get_logger("RotationGoalChecker")};

  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};

}  // namespace nav2_rotation_goal_checker

#endif  // NAV2_ROTATION_GOAL_CHECKER__ROTATION_GOAL_CHECKER_HPP_
