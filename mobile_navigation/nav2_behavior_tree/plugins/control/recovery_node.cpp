// Copyright (c) 2019 Intel Corporation
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
#include "nav2_behavior_tree/plugins/control/recovery_node.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

RecoveryNode::RecoveryNode(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ControlNode::ControlNode(name, conf),
  current_child_idx_(0),
  number_of_retries_(1),
  retry_count_(0),
  primary_running_logged_(false),
  recovery_running_logged_(false)
{
  getInput("number_of_retries", number_of_retries_);
}

BT::NodeStatus RecoveryNode::tick()
{
  const unsigned children_count = children_nodes_.size();

  if (children_count != 2) {
    throw BT::BehaviorTreeException("Recovery Node '" + name() + "' must only have 2 children.");
  }

  setStatus(BT::NodeStatus::RUNNING);

  while (current_child_idx_ < children_count && retry_count_ <= number_of_retries_) {
    TreeNode * child_node = children_nodes_[current_child_idx_];
    const BT::NodeStatus child_status = child_node->executeTick();

    if (current_child_idx_ == 0) {
      switch (child_status) {
        case BT::NodeStatus::SUCCESS:
          {
            // reset node and return success when first child returns success
            if (retry_count_ > 0) {
              RCLCPP_INFO(
                rclcpp::get_logger("nav2_behavior_tree"),
                "Recovery succeeded: node='%s', primary='%s', recoveries=%u/%u.",
                name().c_str(), child_node->name().c_str(), retry_count_,
                number_of_retries_);
            }
            retry_count_ = 0;
            primary_running_logged_ = false;
            recovery_running_logged_ = false;
            halt();
            return BT::NodeStatus::SUCCESS;
          }

        case BT::NodeStatus::FAILURE:
          {
            if (retry_count_ < number_of_retries_) {
              // halt first child and tick second child in next iteration
              RCLCPP_WARN(
                rclcpp::get_logger("nav2_behavior_tree"),
                "Recovery started: node='%s', primary='%s', action='%s', attempt=%u/%u.",
                name().c_str(), child_node->name().c_str(),
                children_nodes_[1]->name().c_str(), retry_count_ + 1,
                number_of_retries_);
              ControlNode::haltChild(0);
              current_child_idx_++;
              primary_running_logged_ = false;
              recovery_running_logged_ = false;
              break;
            } else {
              // reset node and return failure when max retries has been exceeded
              RCLCPP_ERROR(
                rclcpp::get_logger("nav2_behavior_tree"),
                "Recovery exhausted: node='%s', primary='%s', attempts=%u/%u.",
                name().c_str(), child_node->name().c_str(), retry_count_,
                number_of_retries_);
              halt();
              return BT::NodeStatus::FAILURE;
            }
          }

        case BT::NodeStatus::RUNNING:
          {
            if (!primary_running_logged_) {
              RCLCPP_DEBUG(
                rclcpp::get_logger("nav2_behavior_tree"),
                "Recovery waiting: node='%s', primary='%s' is running.",
                name().c_str(), child_node->name().c_str());
              primary_running_logged_ = true;
            }
            return BT::NodeStatus::RUNNING;
          }

        default:
          {
            throw BT::LogicError("A child node must never return IDLE");
          }
      }  // end switch

    } else if (current_child_idx_ == 1) {
      switch (child_status) {
        case BT::NodeStatus::SUCCESS:
          {
            // halt second child, increment recovery count, and tick first child in next iteration
            RCLCPP_INFO(
              rclcpp::get_logger("nav2_behavior_tree"),
              "Recovery action succeeded: node='%s', action='%s', attempt=%u/%u, "
              "retrying='%s'.",
              name().c_str(), child_node->name().c_str(), retry_count_ + 1,
              number_of_retries_, children_nodes_[0]->name().c_str());
            ControlNode::haltChild(1);
            retry_count_++;
            current_child_idx_--;
            primary_running_logged_ = false;
            recovery_running_logged_ = false;
          }
          break;

        case BT::NodeStatus::FAILURE:
          {
            // reset node and return failure if second child fails
            RCLCPP_ERROR(
              rclcpp::get_logger("nav2_behavior_tree"),
              "Recovery action failed: node='%s', action='%s', attempt=%u/%u.",
              name().c_str(), child_node->name().c_str(), retry_count_ + 1,
              number_of_retries_);
            halt();
            return BT::NodeStatus::FAILURE;
          }

        case BT::NodeStatus::RUNNING:
          {
            if (!recovery_running_logged_) {
              RCLCPP_DEBUG(
                rclcpp::get_logger("nav2_behavior_tree"),
                "Recovery action running: node='%s', action='%s', primary='%s'.",
                name().c_str(), child_node->name().c_str(),
                children_nodes_[0]->name().c_str());
              recovery_running_logged_ = true;
            }
            return BT::NodeStatus::RUNNING;
          }

        default:
          {
            throw BT::LogicError("A child node must never return IDLE");
          }
      }  // end switch
    }
  }  // end while loop

  // reset node and return failure
  halt();
  return BT::NodeStatus::FAILURE;
}

void RecoveryNode::halt()
{
  ControlNode::halt();
  retry_count_ = 0;
  current_child_idx_ = 0;
  primary_running_logged_ = false;
  recovery_running_logged_ = false;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RecoveryNode>("RecoveryNode");
}
