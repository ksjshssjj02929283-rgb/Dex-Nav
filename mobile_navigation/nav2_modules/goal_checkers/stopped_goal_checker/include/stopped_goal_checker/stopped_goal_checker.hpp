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

#ifndef STOPPED_GOAL_CHECKER__STOPPED_GOAL_CHECKER_HPP_
#define STOPPED_GOAL_CHECKER__STOPPED_GOAL_CHECKER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "simple_goal_checker/simple_goal_checker.hpp"

namespace stopped_goal_checker
{

/**
 * @class StoppedGoalChecker
 * @brief Goal Checker plugin that requires the robot to be (nearly) stopped at the goal.
 *
 * Extends SimpleGoalChecker: on top of the XY position and yaw tolerance check, the goal
 * is only considered reached when both the angular and translational velocities are below
 * the configured stopped-velocity thresholds.
 */
class StoppedGoalChecker : public simple_goal_checker::SimpleGoalChecker
{
public:
  StoppedGoalChecker();

  // Standard GoalChecker Interface
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;

  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override;

protected:
  double rot_stopped_velocity_, trans_stopped_velocity_;
};

}  // namespace stopped_goal_checker

#endif  // STOPPED_GOAL_CHECKER__STOPPED_GOAL_CHECKER_HPP_
