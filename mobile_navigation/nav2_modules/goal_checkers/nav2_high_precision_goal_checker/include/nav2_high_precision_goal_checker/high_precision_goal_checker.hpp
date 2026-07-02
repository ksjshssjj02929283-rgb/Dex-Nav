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

#ifndef HIGH_PRECISION_GOAL_CHECKER__HIGH_PRECISION_GOAL_CHECKER_HPP_
#define HIGH_PRECISION_GOAL_CHECKER__HIGH_PRECISION_GOAL_CHECKER_HPP_

#include <deque>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_high_precision_goal_checker
{

/**
 * @class HighPrecisionGoalChecker
 * @brief Goal Checker plugin enforcing tight tolerances for high-precision docking.
 *
 * Differs from SimpleGoalChecker by transforming the goal into the robot base frame
 * to separate longitudinal (x) and lateral (y) errors, and by smoothing the
 * longitudinal and yaw errors over a sliding window (deque of 20 samples) before
 * declaring the goal reached. Also requires the robot to be (nearly) stopped.
 */
class HighPrecisionGoalChecker : public nav2_core::GoalChecker
{
public:
  HighPrecisionGoalChecker();

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
  double CalculateMean(const std::deque<double> & data);
  double CalculateVariance(const std::deque<double> & data, double mean);

  double lat_goal_tolerance_, lon_goal_tolerance_;
  double yaw_goal_tolerance_;
  bool stateful_, check_xy_;
  double rot_stopped_velocity_, trans_stopped_velocity_;
  std::string plugin_name_;

  std::deque<double> theta_errors_;
  std::deque<double> longitude_errors_;

  rclcpp::Logger logger_{rclcpp::get_logger("HighPrecisionGoalChecker")};
};

}  // namespace nav2_high_precision_goal_checker

#endif  // HIGH_PRECISION_GOAL_CHECKER__HIGH_PRECISION_GOAL_CHECKER_HPP_
