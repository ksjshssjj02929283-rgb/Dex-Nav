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

#ifndef LONGITUDINAL_GOAL_CHECKER__LONGITUDINAL_GOAL_CHECKER_HPP_
#define LONGITUDINAL_GOAL_CHECKER__LONGITUDINAL_GOAL_CHECKER_HPP_

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

namespace longitudinal_goal_checker
{

/**
 * @class LongitudinalGoalChecker
 * @brief Goal Checker plugin that checks goal reaching in the robot heading frame.
 *
 * The XY error is projected into the query pose (robot) frame, splitting it into a
 * longitudinal error (along heading, X) and a lateral error (Y), each with an
 * independent tolerance. Additionally, the goal is considered reached only once the
 * yaw error is within tolerance and the robot has stopped (rotational and translational
 * velocities below their thresholds). When stateful, the position (lon/lat) check is
 * latched off after it is first satisfied so only the orientation/velocity conditions
 * keep being checked.
 */
class LongitudinalGoalChecker : public nav2_core::GoalChecker
{
public:
  LongitudinalGoalChecker();

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
  double lat_goal_tolerance_, lon_goal_tolerance_;
  double yaw_goal_tolerance_;
  bool stateful_, check_xy_;
  double rot_stopped_velocity_, trans_stopped_velocity_;
  std::string plugin_name_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  rclcpp::Logger logger_{rclcpp::get_logger("LongitudinalGoalChecker")};

  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};

}  // namespace longitudinal_goal_checker

#endif  // LONGITUDINAL_GOAL_CHECKER__LONGITUDINAL_GOAL_CHECKER_HPP_
