// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0

#include "nav2_bt_navigator/navigation_session_manager.hpp"

#include <chrono>
#include <cinttypes>

#include "nav2_behavior_tree/global_replan_context.hpp"
#include "nav2_util/navigation_protocol.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_bt_navigator
{

NavigationSessionManager::NavigationSessionManager(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
: logger_(node->get_logger()),
  next_session_id_(
    static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count()))
{
  session_pub_ = node->create_publisher<std_msgs::msg::UInt64>(
    nav2_util::navigation_protocol::kPrepareSessionTopic,
    nav2_util::navigation_protocol::stateTopicQoS());
}

void NavigationSessionManager::activate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!session_pub_->is_activated()) {
    session_pub_->on_activate();
  }

  // 激活时明确发布当前状态，晚加入的Controller可立即同步任务边界。
  publishSession(current_session_id_);
}

void NavigationSessionManager::deactivate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (session_pub_->is_activated()) {
    session_pub_->on_deactivate();
  }
}

uint64_t NavigationSessionManager::beginSession(const BT::Blackboard::Ptr & blackboard)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // 0 保留为“无活动任务”，计数溢出时跳过该值。
  ++next_session_id_;
  if (next_session_id_ == 0) {
    ++next_session_id_;
  }

  if (active_blackboard_ && active_blackboard_ != blackboard) {
    active_blackboard_->set<uint64_t>(nav2_behavior_tree::kPrepareNavigationSessionKey, 0);
  }

  current_session_id_ = next_session_id_;
  active_blackboard_ = blackboard;
  active_blackboard_->set<uint64_t>(
    nav2_behavior_tree::kPrepareNavigationSessionKey, current_session_id_);
  publishSession(current_session_id_);

  RCLCPP_INFO(
    logger_, "Navigation session started: session=%" PRIu64 ".", current_session_id_);
  return current_session_id_;
}

bool NavigationSessionManager::endSession(uint64_t session_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (session_id == 0 || session_id != current_session_id_) {
    return false;
  }

  if (active_blackboard_) {
    active_blackboard_->set<uint64_t>(nav2_behavior_tree::kPrepareNavigationSessionKey, 0);
  }
  current_session_id_ = 0;
  active_blackboard_.reset();
  publishSession(0);

  RCLCPP_INFO(logger_, "Navigation session ended: session=%" PRIu64 ".", session_id);
  return true;
}

void NavigationSessionManager::endCurrentSession()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (current_session_id_ == 0) {
    return;
  }

  const uint64_t ended_session_id = current_session_id_;
  if (active_blackboard_) {
    active_blackboard_->set<uint64_t>(nav2_behavior_tree::kPrepareNavigationSessionKey, 0);
  }
  current_session_id_ = 0;
  active_blackboard_.reset();
  publishSession(0);

  RCLCPP_INFO(logger_, "Navigation session ended: session=%" PRIu64 ".", ended_session_id);
}

uint64_t NavigationSessionManager::currentSessionId() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_session_id_;
}

void NavigationSessionManager::publishSession(uint64_t session_id)
{
  if (!session_pub_->is_activated()) {
    return;
  }

  std_msgs::msg::UInt64 message;
  message.data = session_id;
  session_pub_->publish(message);
}

}  // namespace nav2_bt_navigator
