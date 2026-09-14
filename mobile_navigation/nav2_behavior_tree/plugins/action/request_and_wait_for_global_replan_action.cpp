// Copyright (c) 2026 kuawei
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

#include "nav2_behavior_tree/plugins/action/request_and_wait_for_global_replan_action.hpp"

#include <cinttypes>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"

namespace nav2_behavior_tree
{

RequestAndWaitForGlobalReplan::RequestAndWaitForGlobalReplan(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  context_ = getOrCreateGlobalReplanContext(config().blackboard);
}

BT::NodeStatus RequestAndWaitForGlobalReplan::onStart()
{
  request_token_.reset();

  std::string reason;
  if (!getInput("reason", reason) || !parseGlobalReplanReason(reason, reason_)) {
    RCLCPP_ERROR(
      node_->get_logger(), "Global replan request rejected: invalid reason='%s'.",
      reason.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const GlobalReplanMode mode = context_->mode();
  if (mode == GlobalReplanMode::DISABLED) {
    return BT::NodeStatus::SUCCESS;
  }
  if (mode == GlobalReplanMode::OBSERVE) {
    RCLCPP_INFO(node_->get_logger(), "Global replan observed: reason=%s.", toString(reason_));
    return BT::NodeStatus::SUCCESS;
  }

  request_token_ = context_->request(reason_);
  RCLCPP_INFO(
    node_->get_logger(),
    "Global replan requested: id=%" PRIu64 ", reason=%s.",
    request_token_->request_id, toString(reason_));
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RequestAndWaitForGlobalReplan::onRunning()
{
  if (!request_token_) {
    RCLCPP_ERROR(node_->get_logger(), "Global replan wait failed: request token is missing.");
    return BT::NodeStatus::FAILURE;
  }

  if (context_->isCompleted(*request_token_)) {
    return BT::NodeStatus::SUCCESS;
  }
  if (context_->isCanceled(*request_token_)) {
    RCLCPP_WARN(
      node_->get_logger(), "Global replan wait stopped: id=%" PRIu64 " was canceled.",
      request_token_->request_id);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void RequestAndWaitForGlobalReplan::onHalted()
{
  cancelPendingRequest();
}

void RequestAndWaitForGlobalReplan::cancelPendingRequest()
{
  if (request_token_ && context_->cancel(*request_token_)) {
    RCLCPP_INFO(
      node_->get_logger(), "Global replan canceled: id=%" PRIu64 ".",
      request_token_->request_id);
  }
  request_token_.reset();
}

}  // namespace nav2_behavior_tree

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<nav2_behavior_tree::RequestAndWaitForGlobalReplan>(
    "RequestAndWaitForGlobalReplan");
}
