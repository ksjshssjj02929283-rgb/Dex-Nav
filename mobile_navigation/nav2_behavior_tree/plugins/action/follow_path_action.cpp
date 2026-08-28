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

#include <memory>
#include <string>

#include "nav2_behavior_tree/plugins/action/follow_path_action.hpp"

namespace nav2_behavior_tree
{

namespace
{

const char * displayId(const std::string & id, const char * default_source)
{
  return id.empty() ? default_source : id.c_str();
}

}  // namespace

FollowPathAction::FollowPathAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::FollowPath>(xml_tag_name, action_name, conf)
{
}

void FollowPathAction::on_tick()
{
  getInput("path", goal_.path);
  getInput("controller_id", goal_.controller_id);
  getInput("goal_checker_id", goal_.goal_checker_id);

  RCLCPP_INFO(
    node_->get_logger(),
    "FollowPath requested: controller='%s', goal_checker='%s', poses=%zu.",
    displayId(goal_.controller_id, "<current_controller>"),
    displayId(goal_.goal_checker_id, "<current_goal_checker>"), goal_.path.poses.size());
}

BT::NodeStatus FollowPathAction::tick()
{
  nav_msgs::msg::Path path;
  getInput("path", path);

  const bool no_action_in_flight = !goal_handle_ && !future_goal_handle_;
  if (path.poses.empty() &&
    (status() == BT::NodeStatus::IDLE || waiting_for_valid_path_ || no_action_in_flight))
  {
    if (!waiting_for_valid_path_) {
      RCLCPP_WARN(
        node_->get_logger(),
        "FollowPath waiting: path is empty.");
    }
    waiting_for_valid_path_ = true;
    setStatus(BT::NodeStatus::RUNNING);
    return BT::NodeStatus::RUNNING;
  }

  if (waiting_for_valid_path_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "FollowPath resumed: poses=%zu.", path.poses.size());
    waiting_for_valid_path_ = false;
    BtActionNode<nav2_msgs::action::FollowPath>::halt();
  }

  return BtActionNode<nav2_msgs::action::FollowPath>::tick();
}

void FollowPathAction::halt()
{
  is_halting_ = true;
  BtActionNode<nav2_msgs::action::FollowPath>::halt();
  is_halting_ = false;
  waiting_for_valid_path_ = false;
}

void FollowPathAction::on_wait_for_result(
  std::shared_ptr<const nav2_msgs::action::FollowPath::Feedback>/*feedback*/)
{
  // Grab the new path
  nav_msgs::msg::Path new_path;
  getInput("path", new_path);

  // Check if it is not same with the current one
  if (goal_.path != new_path) {
    if (new_path.poses.empty()) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "FollowPath update ignored: new path is empty, active_poses=%zu.",
        goal_.path.poses.size());
      return;
    }

    // the action server on the next loop iteration
    RCLCPP_INFO(
      node_->get_logger(),
      "FollowPath path updated: old_poses=%zu, new_poses=%zu.",
      goal_.path.poses.size(), new_path.poses.size());
    goal_.path = new_path;
    goal_updated_ = true;
  }

  std::string new_controller_id;
  getInput("controller_id", new_controller_id);

  if (goal_.controller_id != new_controller_id) {
    RCLCPP_INFO(
      node_->get_logger(),
      "FollowPath controller updated: '%s' -> '%s'.",
      goal_.controller_id.c_str(), new_controller_id.c_str());
    goal_.controller_id = new_controller_id;
    goal_updated_ = true;
  }

  std::string new_goal_checker_id;
  getInput("goal_checker_id", new_goal_checker_id);

  if (goal_.goal_checker_id != new_goal_checker_id) {
    RCLCPP_INFO(
      node_->get_logger(),
      "FollowPath goal checker updated: '%s' -> '%s'.",
      goal_.goal_checker_id.c_str(), new_goal_checker_id.c_str());
    goal_.goal_checker_id = new_goal_checker_id;
    goal_updated_ = true;
  }
}

BT::NodeStatus FollowPathAction::on_success()
{
  RCLCPP_INFO(
    node_->get_logger(),
    "FollowPath succeeded: controller='%s', poses=%zu.",
    displayId(goal_.controller_id, "<current_controller>"), goal_.path.poses.size());
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus FollowPathAction::on_aborted()
{
  RCLCPP_WARN(
    node_->get_logger(),
    "FollowPath aborted: controller='%s', poses=%zu.",
    displayId(goal_.controller_id, "<current_controller>"), goal_.path.poses.size());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus FollowPathAction::on_cancelled()
{
  if (is_halting_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "FollowPath canceled: behavior tree halted.");
    return BT::NodeStatus::RUNNING;
  }

  nav_msgs::msg::Path path;
  getInput("path", path);

  // 取消只表示当前控制目标已终止，不能作为导航到达；下一次 tick 必须重新提交有效路径。
  waiting_for_valid_path_ = true;
  if (path.poses.empty()) {
    RCLCPP_INFO(
      node_->get_logger(),
      "FollowPath canceled: path is empty; waiting for replan.");
    return BT::NodeStatus::RUNNING;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "FollowPath canceled: valid path is ready; restarting with %zu poses.",
    path.poses.size());
  return BT::NodeStatus::RUNNING;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::FollowPathAction>(
        name, "follow_path", config);
    };

  factory.registerBuilder<nav2_behavior_tree::FollowPathAction>(
    "FollowPath", builder);
}
