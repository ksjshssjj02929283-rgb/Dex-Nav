// Copyright (c) 2018 Intel Corporation
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

#include "nav2_behavior_tree/plugins/action/compute_path_to_pose_action.hpp"

namespace nav2_behavior_tree
{

namespace
{

bool isSameGoal(
  const geometry_msgs::msg::PoseStamped & a,
  const geometry_msgs::msg::PoseStamped & b)
{
  constexpr double kEpsilon = 1e-6;
  return a.header.frame_id == b.header.frame_id &&
         std::abs(a.pose.position.x - b.pose.position.x) < kEpsilon &&
         std::abs(a.pose.position.y - b.pose.position.y) < kEpsilon &&
         std::abs(a.pose.position.z - b.pose.position.z) < kEpsilon &&
         std::abs(a.pose.orientation.x - b.pose.orientation.x) < kEpsilon &&
         std::abs(a.pose.orientation.y - b.pose.orientation.y) < kEpsilon &&
         std::abs(a.pose.orientation.z - b.pose.orientation.z) < kEpsilon &&
         std::abs(a.pose.orientation.w - b.pose.orientation.w) < kEpsilon;
}

}  // namespace

ComputePathToPoseAction::ComputePathToPoseAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::ComputePathToPose>(xml_tag_name, action_name, conf)
{
}

void ComputePathToPoseAction::on_tick()
{
  getInput("goal", goal_.goal);
  getInput("planner_id", goal_.planner_id);
  goal_.use_start = false;
  if (getInput("start", goal_.start)) {
    goal_.use_start = true;
  }

  if (!have_last_goal_ || !isSameGoal(last_goal_, goal_.goal)) {
    nav_msgs::msg::Path empty_path;
    setOutput("path", empty_path);
    RCLCPP_INFO(
      node_->get_logger(),
      "Global path cleared: reason=new_goal, frame='%s', goal=(%.3f, %.3f).",
      goal_.goal.header.frame_id.c_str(), goal_.goal.pose.position.x, goal_.goal.pose.position.y);
    last_goal_ = goal_.goal;
    have_last_goal_ = true;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Global planning started: planner='%s', goal=(%.3f, %.3f), use_start=%s.",
    goal_.planner_id.c_str(), goal_.goal.pose.position.x, goal_.goal.pose.position.y,
    goal_.use_start ? "true" : "false");
}

BT::NodeStatus ComputePathToPoseAction::on_success()
{
  if (result_.result->path.poses.empty()) {
    nav_msgs::msg::Path empty_path;
    setOutput("path", empty_path);
    RCLCPP_WARN(
      node_->get_logger(),
      "Global planning failed: planner='%s', reason=empty_path; path cleared.",
      goal_.planner_id.c_str());
    return BT::NodeStatus::FAILURE;
  }

  setOutput("path", result_.result->path);
  RCLCPP_INFO(
    node_->get_logger(),
    "Global planning succeeded: planner='%s', poses=%zu.",
    goal_.planner_id.c_str(), result_.result->path.poses.size());
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputePathToPoseAction::on_aborted()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  RCLCPP_WARN(
    node_->get_logger(),
    "Global planning aborted: planner='%s'; path cleared.",
    goal_.planner_id.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputePathToPoseAction::on_cancelled()
{
  RCLCPP_INFO(
    node_->get_logger(),
    "Global planning canceled: planner='%s'; previous path kept.",
    goal_.planner_id.c_str());
  return BT::NodeStatus::FAILURE;
}

void ComputePathToPoseAction::halt()
{
  BtActionNode::halt();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::ComputePathToPoseAction>(
        name, "compute_path_to_pose", config);
    };

  factory.registerBuilder<nav2_behavior_tree::ComputePathToPoseAction>(
    "ComputePathToPose", builder);
}
