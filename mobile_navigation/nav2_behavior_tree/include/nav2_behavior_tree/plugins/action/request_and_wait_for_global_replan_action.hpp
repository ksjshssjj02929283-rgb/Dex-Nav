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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REQUEST_AND_WAIT_FOR_GLOBAL_REPLAN_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REQUEST_AND_WAIT_FOR_GLOBAL_REPLAN_ACTION_HPP_

#include <memory>
#include <optional>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_behavior_tree/global_replan_context.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief 请求全局重规划，并在对应新路径生成前保持 RUNNING
 *
 * RecoveryNode 会在恢复子树成功后立即重试主任务，因此该节点必须等待规划完成，
 * 不能只写入一个请求标志后直接返回 SUCCESS。
 */
class RequestAndWaitForGlobalReplan : public BT::StatefulActionNode
{
public:
  RequestAndWaitForGlobalReplan(const std::string & name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>(
        "reason",
        "Recovery reason: follow_path_failed or back_up_succeeded")};
  }

private:
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
  void cancelPendingRequest();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<GlobalReplanContext> context_;
  std::optional<GlobalReplanToken> request_token_;
  GlobalReplanReason reason_{GlobalReplanReason::FOLLOW_PATH_FAILED};
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REQUEST_AND_WAIT_FOR_GLOBAL_REPLAN_ACTION_HPP_
