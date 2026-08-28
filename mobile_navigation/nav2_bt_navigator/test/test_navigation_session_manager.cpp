// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>

#include "behaviortree_cpp_v3/blackboard.h"
#include "nav2_behavior_tree/global_replan_context.hpp"
#include "nav2_bt_navigator/navigation_session_manager.hpp"
#include "nav2_util/navigation_protocol.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/u_int64.hpp"

namespace
{

using namespace std::chrono_literals;

class NavigationSessionManagerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

TEST_F(NavigationSessionManagerTest, ProtocolUsesRetainedSingleValueQos)
{
  const auto profile =
    nav2_util::navigation_protocol::stateTopicQoS().get_rmw_qos_profile();

  EXPECT_EQ(profile.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  EXPECT_EQ(profile.depth, 1u);
  EXPECT_EQ(profile.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(profile.durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
}

TEST_F(NavigationSessionManagerTest, OldCompletionCannotClearNewSession)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("session_manager_test");
  nav2_bt_navigator::NavigationSessionManager manager(node);
  manager.activate();
  auto first_blackboard = BT::Blackboard::create();
  auto second_blackboard = BT::Blackboard::create();

  const uint64_t first_session = manager.beginSession(first_blackboard);
  const uint64_t second_session = manager.beginSession(second_blackboard);

  EXPECT_NE(first_session, 0u);
  EXPECT_NE(second_session, first_session);
  EXPECT_FALSE(manager.endSession(first_session));
  EXPECT_EQ(manager.currentSessionId(), second_session);

  uint64_t blackboard_session = first_session;
  ASSERT_TRUE(first_blackboard->get(
    nav2_behavior_tree::kPrepareNavigationSessionKey, blackboard_session));
  EXPECT_EQ(blackboard_session, 0u);

  ASSERT_TRUE(second_blackboard->get(
    nav2_behavior_tree::kPrepareNavigationSessionKey, blackboard_session));
  EXPECT_EQ(blackboard_session, second_session);

  EXPECT_TRUE(manager.endSession(second_session));
  EXPECT_EQ(manager.currentSessionId(), 0u);
  ASSERT_TRUE(second_blackboard->get(
    nav2_behavior_tree::kPrepareNavigationSessionKey, blackboard_session));
  EXPECT_EQ(blackboard_session, 0u);
}

TEST_F(NavigationSessionManagerTest, LateSubscriberReceivesCurrentSession)
{
  auto navigator_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "session_manager_publisher", "/session_test");
  nav2_bt_navigator::NavigationSessionManager manager(navigator_node);
  manager.activate();
  auto blackboard = BT::Blackboard::create();
  const uint64_t session_id = manager.beginSession(blackboard);

  auto observer = std::make_shared<rclcpp::Node>("session_manager_observer", "/session_test");
  std::atomic<uint64_t> received_session{0};
  auto subscription = observer->create_subscription<std_msgs::msg::UInt64>(
    nav2_util::navigation_protocol::kPrepareSessionTopic,
    nav2_util::navigation_protocol::stateTopicQoS(),
    [&received_session](const std_msgs::msg::UInt64::SharedPtr message) {
      received_session.store(message->data);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(observer);
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (received_session.load() != session_id && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  EXPECT_EQ(received_session.load(), session_id);
  manager.endSession(session_id);
  (void)subscription;
}

}  // namespace
