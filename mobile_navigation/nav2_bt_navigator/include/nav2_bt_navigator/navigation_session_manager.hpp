// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0

#ifndef NAV2_BT_NAVIGATOR__NAVIGATION_SESSION_MANAGER_HPP_
#define NAV2_BT_NAVIGATOR__NAVIGATION_SESSION_MANAGER_HPP_

#include <cstdint>
#include <memory>
#include <mutex>

#include "behaviortree_cpp_v3/blackboard.h"
#include "rclcpp/logger.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/u_int64.hpp"

namespace nav2_bt_navigator
{

/**
 * @brief 统一维护 BtNavigator 当前导航任务代次。
 *
 * 该对象不是 ROS Node，不创建线程或定时器。单点、多点导航共享同一个实例，
 * 从而保证 Session Topic 只有一个发布者。
 */
class NavigationSessionManager
{
public:
  explicit NavigationSessionManager(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);

  /// BtNavigator激活后启用发布器，并发布当前任务状态。
  void activate();

  /// 结束当前任务后停用发布器；重复调用无副作用。
  void deactivate();

  /// 接受新任务后生成代次并同步到 Blackboard 和 Session Topic。
  uint64_t beginSession(const BT::Blackboard::Ptr & blackboard);

  /// 仅当 session_id 仍是当前任务时结束，避免迟到回调清除更新的任务。
  bool endSession(uint64_t session_id);

  /// Lifecycle 退出时幂等清理当前任务。
  void endCurrentSession();

  uint64_t currentSessionId() const;

private:
  void publishSession(uint64_t session_id);

  rclcpp::Logger logger_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt64>::SharedPtr session_pub_;
  mutable std::mutex mutex_;
  uint64_t next_session_id_{0};
  uint64_t current_session_id_{0};
  BT::Blackboard::Ptr active_blackboard_;
};

}  // namespace nav2_bt_navigator

#endif  // NAV2_BT_NAVIGATOR__NAVIGATION_SESSION_MANAGER_HPP_
