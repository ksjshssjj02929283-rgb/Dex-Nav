// Copyright (c) 2022 Neobotix GmbH
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

#include "std_msgs/msg/string.hpp"

#include "nav2_behavior_tree/plugins/action/controller_cancel_node.hpp"

namespace nav2_behavior_tree
{

ControllerCancel::ControllerCancel(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtCancelActionNode<nav2_msgs::action::FollowPath>(xml_tag_name, action_name, conf)
{
}

BT::NodeStatus ControllerCancel::tick()
{
  BtScopedNodeTimer tick_timer(tick_timing_, name());
  RCLCPP_WARN(
    node_->get_logger(),
    "-------------------- [BT Recovery Action][CancelControl] -------------------- "
    "全局规划连续失败，取消当前 FollowPath，停止沿用旧路径继续局部控制；"
    "等待后续重规划成功再重新跟踪新路径");
  return tickCancelActionWithoutTiming();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::ControllerCancel>(
        name, "follow_path", config);
    };

  factory.registerBuilder<nav2_behavior_tree::ControllerCancel>(
    "CancelControl", builder);
}
