// Copyright (c) 2020 Samsung Research America
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

#include <gtest/gtest.h>
#include <atomic>
#include <initializer_list>
#include <memory>
#include <chrono>
#include <string>
#include <thread>
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_thread.hpp"
#include "nav2_lifecycle_manager/lifecycle_manager.hpp"
#include "nav2_lifecycle_manager/lifecycle_manager_client.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using lifecycle_msgs::msg::State;

// minimal lifecycle node implementing bond as in rest of navigation servers
class TestLifecycleNode : public nav2_util::LifecycleNode
{
public:
  TestLifecycleNode(bool bond, std::string name, int bond_attempt = 1)
  : nav2_util::LifecycleNode(name)
  {
    state = "";
    enable_bond = bond;
    bond_attempt_ = bond_attempt;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State & /*state*/) override
  {
    RCLCPP_INFO(get_logger(), "Lifecycle Test node is Configured!");
    state = "configured";
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & /*state*/) override
  {
    ++activation_count;
    RCLCPP_INFO(get_logger(), "Lifecycle Test node is Activated!");
    state = "activated";
    // 前几轮不创建 bond，模拟首次建连超时。
    if (enable_bond && activation_count.load() >= bond_attempt_) {
      createBond();
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*state*/) override
  {
    // 模拟停用回调迟迟不返回，覆盖客户端响应超时路径。
    std::this_thread::sleep_for(std::chrono::milliseconds(deactivate_delay_ms.load()));
    if (!allow_deactivate.load()) {
      return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(get_logger(), "Lifecycle Test node is Deactivated!");
    state = "deactivated";
    if (bond_) {
      destroyBond();
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /*state*/) override
  {
    // 清理被拒绝时保留 inactive，覆盖 RESUME 可能绕过待清理检查的场景。
    if (!allow_cleanup.load()) {
      return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(get_logger(), "Lifecycle Test node is Cleanup!");
    state = "cleaned up";
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*state*/) override
  {
    RCLCPP_INFO(get_logger(), "Lifecycle Test node is Shutdown!");
    state = "shut down";
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State & /*state*/) override
  {
    RCLCPP_INFO(get_logger(), "Lifecycle Test node is encountered an error!");
    state = "errored";
    return CallbackReturn::SUCCESS;
  }

  bool bondAllocated()
  {
    return bond_ ? true : false;
  }

  void breakBond()
  {
    bond_->breakBond();
  }

  std::string getState()
  {
    return state;
  }

  bool isBondEnabled()
  {
    return enable_bond;
  }

  bool isBondConnected()
  {
    return bondAllocated() ? !bond_->isBroken() : false;
  }

  std::string state;
  bool enable_bond;
  int bond_attempt_;
  std::atomic<int> activation_count{0};
  std::atomic<bool> allow_deactivate{true};
  std::atomic<bool> allow_cleanup{true};
  std::atomic<int> deactivate_delay_ms{0};
};

class TestFixture
{
public:
  TestFixture(bool bond, std::string node_name, int bond_attempt = 1)
  {
    lf_node_ = std::make_shared<TestLifecycleNode>(bond, node_name, bond_attempt);
    lf_thread_ = std::make_unique<nav2_util::NodeThread>(lf_node_->get_node_base_interface());
    // 分别覆盖超时节点之前已激活、之后仅配置的节点。
    before_ = std::make_shared<TestLifecycleNode>(true, "bond_before");
    after_ = std::make_shared<TestLifecycleNode>(true, "bond_after");
    before_thread_ = std::make_unique<nav2_util::NodeThread>(before_->get_node_base_interface());
    after_thread_ = std::make_unique<nav2_util::NodeThread>(after_->get_node_base_interface());
  }

  std::shared_ptr<TestLifecycleNode> lf_node_;
  std::unique_ptr<nav2_util::NodeThread> lf_thread_;
  std::shared_ptr<TestLifecycleNode> before_;
  std::shared_ptr<TestLifecycleNode> after_;
  std::unique_ptr<nav2_util::NodeThread> before_thread_;
  std::unique_ptr<nav2_util::NodeThread> after_thread_;
};

TEST(LifecycleBondTest, POSITIVE)
{
  // let the lifecycle server come up
  rclcpp::Rate(1).sleep();

  auto node = std::make_shared<rclcpp::Node>("lifecycle_manager_test_service_client");
  nav2_lifecycle_manager::LifecycleManagerClient client("lifecycle_manager_test", node);

  // create node, should be up now
  auto fixture = TestFixture(true, "bond_tester");
  auto bond_tester = fixture.lf_node_;

  EXPECT_TRUE(client.startup());

  // check if bond is connected after being activated
  rclcpp::Rate(5).sleep();
  EXPECT_TRUE(bond_tester->isBondConnected());
  EXPECT_EQ(bond_tester->getState(), "activated");

  bond_tester->breakBond();

  // bond should be disconnected now and lifecycle manager should know and react to reset
  rclcpp::Rate(5).sleep();
  EXPECT_EQ(
    nav2_lifecycle_manager::SystemStatus::INACTIVE,
    client.is_active(std::chrono::nanoseconds(1000000000)));
  EXPECT_FALSE(bond_tester->isBondConnected());
  EXPECT_EQ(bond_tester->getState(), "cleaned up");

  // check that bringing up again is OK
  EXPECT_TRUE(client.startup());
  EXPECT_EQ(bond_tester->getState(), "activated");
  EXPECT_TRUE(bond_tester->isBondConnected());
  EXPECT_EQ(
    nav2_lifecycle_manager::SystemStatus::ACTIVE,
    client.is_active(std::chrono::nanoseconds(1000000000)));

  // clean state for next test.
  EXPECT_TRUE(client.reset());
  EXPECT_FALSE(bond_tester->isBondConnected());
  EXPECT_EQ(bond_tester->getState(), "cleaned up");
}

TEST(LifecycleBondTest, NEGATIVE)
{
  auto node = std::make_shared<rclcpp::Node>("lifecycle_manager_test_service_client");
  nav2_lifecycle_manager::LifecycleManagerClient client("lifecycle_manager_test", node);

  // create node, now without bond setup to connect to. Should fail because no bond
  auto fixture = TestFixture(false, "bond_tester");
  auto bond_tester = fixture.lf_node_;
  EXPECT_FALSE(client.startup());
  EXPECT_EQ(bond_tester->activation_count.load(), 1);
  EXPECT_EQ(bond_tester->getState(), "cleaned up");
  EXPECT_EQ(fixture.before_->getState(), "cleaned up");
  EXPECT_EQ(fixture.after_->getState(), "cleaned up");
  EXPECT_FALSE(fixture.before_->bondAllocated());
  // 模拟外层再次请求，确认没有残留的激活状态或旧 bond。
  EXPECT_FALSE(client.startup());
  EXPECT_EQ(bond_tester->activation_count.load(), 2);
  EXPECT_EQ(fixture.before_->getState(), "cleaned up");
  EXPECT_EQ(fixture.after_->getState(), "cleaned up");
  EXPECT_FALSE(bond_tester->isBondEnabled());
  EXPECT_EQ(
    nav2_lifecycle_manager::SystemStatus::INACTIVE,
    client.is_active(std::chrono::nanoseconds(1000000000)));
}

TEST(LifecycleBondTest, EXTERNAL_RETRY_SUCCEEDS)
{
  auto node = std::make_shared<rclcpp::Node>("lifecycle_manager_test_service_client");
  nav2_lifecycle_manager::LifecycleManagerClient client("lifecycle_manager_test", node);
  auto fixture = TestFixture(true, "bond_tester", 2);

  EXPECT_FALSE(client.startup());
  EXPECT_EQ(fixture.lf_node_->activation_count.load(), 1);
  EXPECT_EQ(fixture.after_->activation_count.load(), 0);
  EXPECT_TRUE(client.startup());
  EXPECT_EQ(fixture.lf_node_->activation_count.load(), 2);
  EXPECT_EQ(fixture.before_->activation_count.load(), 2);
  EXPECT_EQ(fixture.after_->activation_count.load(), 1);
  EXPECT_TRUE(fixture.lf_node_->isBondConnected());
  EXPECT_TRUE(client.reset());
}

TEST(LifecycleBondTest, CLEANUP_FAILURE_BLOCKS_STARTUP)
{
  auto node = std::make_shared<rclcpp::Node>("lifecycle_manager_test_service_client");
  nav2_lifecycle_manager::LifecycleManagerClient client("lifecycle_manager_test", node);
  auto fixture = TestFixture(false, "bond_tester");
  fixture.lf_node_->allow_deactivate.store(false);

  EXPECT_FALSE(client.startup());
  EXPECT_EQ(fixture.lf_node_->activation_count.load(), 1);
  EXPECT_EQ(fixture.before_->getState(), "cleaned up");
  EXPECT_EQ(fixture.after_->getState(), "cleaned up");
  // 清理仍失败时，只重做清理，不得再次配置或激活。
  EXPECT_FALSE(client.startup());
  EXPECT_EQ(fixture.lf_node_->activation_count.load(), 1);
  EXPECT_EQ(fixture.before_->activation_count.load(), 1);
  fixture.lf_node_->allow_deactivate.store(true);
  // 清理恢复后允许新一轮启动；本节点仍不发 bond，因此再次返回失败。
  EXPECT_FALSE(client.startup());
  EXPECT_EQ(fixture.lf_node_->activation_count.load(), 2);
  EXPECT_EQ(fixture.lf_node_->getState(), "cleaned up");
}

TEST(LifecycleBondTest, CLEANUP_FAILURE_BLOCKS_RESUME)
{
  auto node = std::make_shared<rclcpp::Node>("lifecycle_manager_test_service_client");
  nav2_lifecycle_manager::LifecycleManagerClient client("lifecycle_manager_test", node);
  auto fixture = TestFixture(true, "bond_tester", 2);
  const auto nodes = {fixture.before_, fixture.lf_node_, fixture.after_};
  for (const auto & lifecycle_node : nodes) {
    lifecycle_node->allow_cleanup.store(false);
  }

  // 首次建连失败后停用成功、清理失败，所有节点仍为 inactive。
  EXPECT_FALSE(client.startup());
  for (const auto & lifecycle_node : nodes) {
    EXPECT_EQ(lifecycle_node->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
  }
  EXPECT_EQ(
    nav2_lifecycle_manager::SystemStatus::INACTIVE,
    client.is_active(std::chrono::seconds(1)));

  // 即使下一次激活已可建立 bond，也必须先完成清理，不能通过 RESUME 激活。
  EXPECT_FALSE(client.resume());
  EXPECT_EQ(fixture.before_->activation_count.load(), 1);
  EXPECT_EQ(fixture.lf_node_->activation_count.load(), 1);
  EXPECT_EQ(fixture.after_->activation_count.load(), 0);
  EXPECT_EQ(
    nav2_lifecycle_manager::SystemStatus::INACTIVE,
    client.is_active(std::chrono::seconds(1)));

  // 清理仍失败时再次 STARTUP，不得错误报告 active。
  EXPECT_FALSE(client.startup());
  for (const auto & lifecycle_node : nodes) {
    EXPECT_EQ(lifecycle_node->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);
  }
  EXPECT_EQ(
    nav2_lifecycle_manager::SystemStatus::INACTIVE,
    client.is_active(std::chrono::seconds(1)));

  for (const auto & lifecycle_node : nodes) {
    lifecycle_node->allow_cleanup.store(true);
  }
  EXPECT_TRUE(client.startup());
  for (const auto & lifecycle_node : nodes) {
    EXPECT_EQ(lifecycle_node->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);
    EXPECT_TRUE(lifecycle_node->isBondConnected());
  }
  EXPECT_EQ(
    nav2_lifecycle_manager::SystemStatus::ACTIVE,
    client.is_active(std::chrono::seconds(1)));

  // 清理完成并正常启动后，原有暂停、恢复流程仍可使用。
  EXPECT_TRUE(client.pause());
  EXPECT_TRUE(client.resume());
  EXPECT_EQ(
    nav2_lifecycle_manager::SystemStatus::ACTIVE,
    client.is_active(std::chrono::seconds(1)));
  EXPECT_TRUE(client.reset());
}

TEST(LifecycleBondTest, CLEANUP_TIMEOUT_CONTINUES_OTHER_NODES)
{
  auto node = std::make_shared<rclcpp::Node>("lifecycle_manager_test_service_client");
  nav2_lifecycle_manager::LifecycleManagerClient client("lifecycle_manager_test", node);
  auto fixture = TestFixture(false, "bond_tester");
  fixture.lf_node_->deactivate_delay_ms.store(6000);

  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(client.startup());
  EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::seconds(20));
  EXPECT_EQ(fixture.before_->getState(), "cleaned up");
  EXPECT_EQ(fixture.after_->getState(), "cleaned up");
  EXPECT_EQ(fixture.lf_node_->activation_count.load(), 1);

  // 迟到的停用完成后，下次请求先核对清理状态，再允许重新启动。
  fixture.lf_node_->deactivate_delay_ms.store(0);
  EXPECT_FALSE(client.startup());
  EXPECT_EQ(fixture.lf_node_->activation_count.load(), 2);
  EXPECT_EQ(fixture.lf_node_->getState(), "cleaned up");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
