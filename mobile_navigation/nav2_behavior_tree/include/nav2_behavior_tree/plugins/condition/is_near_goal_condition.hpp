// Copyright (c) 2024 Mobile Navigation Team
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_NEAR_GOAL_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_NEAR_GOAL_CONDITION_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "tf2_ros/buffer.h"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ConditionNode that returns SUCCESS when the robot is within
 *        a specified distance from the goal, and FAILURE otherwise.
 *
 *  Typical usage: wrap RateController with Inverter + IsNearGoal so that
 *  periodic global re-planning is suppressed when approaching the goal.
 *
 *        Sequence
 *         ├── Inverter
 *         │    └── IsNearGoal  goal="{goal}" distance_threshold="1.5"
 *         └── RateController ...
 */
class IsNearGoalCondition : public BT::ConditionNode
{
public:
  IsNearGoalCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsNearGoalCondition() = delete;

  ~IsNearGoalCondition() override;

  BT::NodeStatus tick() override;

  void initialize();

  /**
   * @brief BT ports:
   *   goal               – destination pose (from blackboard)
   *   distance_threshold – metres; SUCCESS when closer than this (default 1.5)
   *   global_frame       – (default "map")
   *   robot_base_frame   – (default "base_link")
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination"),
      BT::InputPort<double>("distance_threshold", 1.5, "Distance threshold in metres"),
      BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
      BT::InputPort<std::string>(
        "robot_base_frame", std::string("base_link"), "Robot base frame"),
    };
  }

protected:
  void cleanup() {}

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  bool initialized_;
  double distance_threshold_;
  std::string global_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_;

  geometry_msgs::msg::PoseStamped last_goal_;
  bool have_last_goal_{false};
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_NEAR_GOAL_CONDITION_HPP_
