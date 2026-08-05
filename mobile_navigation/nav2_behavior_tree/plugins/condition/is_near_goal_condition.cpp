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

#include <string>
#include <memory>
#include <cmath>

#include "nav2_util/robot_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/node_utils.hpp"

#include "nav2_behavior_tree/plugins/condition/is_near_goal_condition.hpp"

namespace nav2_behavior_tree
{

IsNearGoalCondition::IsNearGoalCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  initialized_(false),
  distance_threshold_(1.5),
  global_frame_("map"),
  robot_base_frame_("base_link"),
  have_last_goal_(false)
{
  getInput("distance_threshold", distance_threshold_);
  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);
}

IsNearGoalCondition::~IsNearGoalCondition()
{
  cleanup();
}

void IsNearGoalCondition::initialize()
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_   = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_->get_parameter("transform_tolerance", transform_tolerance_);
  initialized_ = true;
}

BT::NodeStatus IsNearGoalCondition::tick()
{
  if (!initialized_) {
    initialize();
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_DEBUG(node_->get_logger(), "[IsNearGoal] Cannot get current robot pose.");
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped goal;
  if (!getInput("goal", goal)) {
    RCLCPP_DEBUG(node_->get_logger(), "[IsNearGoal] goal port not set.");
    return BT::NodeStatus::FAILURE;
  }

  if (!have_last_goal_ ||
      goal.pose.position.x != last_goal_.pose.position.x ||
      goal.pose.position.y != last_goal_.pose.position.y ||
      goal.pose.position.z != last_goal_.pose.position.z ||
      goal.pose.orientation.x != last_goal_.pose.orientation.x ||
      goal.pose.orientation.y != last_goal_.pose.orientation.y ||
      goal.pose.orientation.z != last_goal_.pose.orientation.z ||
      goal.pose.orientation.w != last_goal_.pose.orientation.w)
  {
    last_goal_ = goal;
    have_last_goal_ = true;
    RCLCPP_INFO(
      node_->get_logger(),
      "[IsNearGoal] Detected new goal or first tick. Forcing global planning first.");
    return BT::NodeStatus::FAILURE;
  }

  double dx = goal.pose.position.x - current_pose.pose.position.x;
  double dy = goal.pose.position.y - current_pose.pose.position.y;
  double dist = std::sqrt(dx * dx + dy * dy);

  if (dist <= distance_threshold_) {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "[IsNearGoal] %.3f m from goal (< %.2f m) — skipping global replan.",
      dist, distance_threshold_);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsNearGoalCondition>("IsNearGoal");
}
