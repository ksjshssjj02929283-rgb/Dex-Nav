// Copyright (c) 2026 Dexforce
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
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "bond/msg/status.hpp"
#include "bondcpp/bond.hpp"
#include "nav2_lifecycle_manager/lifecycle_manager.hpp"
#include "nav2_util/node_thread.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

// 复用真实建连函数，只暴露被测入口，不改变生产接口或替换 bond 实现。
class BondConnectionManager : public nav2_lifecycle_manager::LifecycleManager
{
public:
  explicit BondConnectionManager(const rclcpp::NodeOptions & options)
  : LifecycleManager(options) {}

  bool connect(const std::string & name, std::chrono::milliseconds timeout)
  {
    bond_timeout_ = timeout;
    return createBondConnection(name);
  }

  bool connectionFailed() const
  {
    return bond_connection_failed_;
  }
};

class BondConnectionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    context_ = std::make_shared<rclcpp::Context>();
    context_->init(0, nullptr);
    options_.context(context_);
    options_.arguments({"--ros-args", "-r", "__ns:=/bond_connection_test"});
    options_.parameter_overrides({rclcpp::Parameter("node_names", std::vector<std::string>{})});
    manager_ = std::make_shared<BondConnectionManager>(options_);
    manager_thread_ = std::make_unique<nav2_util::NodeThread>(manager_);
  }

  void TearDown() override
  {
    // 管理器注册了持有 this 的停机回调，必须在销毁实例前关闭其独立 Context。
    context_->shutdown("Bond connection test complete.");
    manager_thread_.reset();
    manager_.reset();
  }

  rclcpp::Context::SharedPtr context_;
  rclcpp::NodeOptions options_;
  std::shared_ptr<BondConnectionManager> manager_;
  std::unique_ptr<nav2_util::NodeThread> manager_thread_;
};

TEST_F(BondConnectionTest, MissingPeerAtTenSecondsReturnsFailure)
{
  EXPECT_FALSE(manager_->connect("missing_peer", 10s))
    << "没有对端时，10 秒连接边界不能返回成功。";
  EXPECT_TRUE(manager_->connectionFailed())
    << "建连失败必须保留标志，使 STARTUP 进入既有清理流程。";
}

TEST_F(BondConnectionTest, PeerReportingDeathDuringWaitReturnsFailure)
{
  auto peer_options = options_;
  peer_options.parameter_overrides({});
  auto peer_node = std::make_shared<rclcpp::Node>("dead_bond_peer", peer_options);
  auto publisher = peer_node->create_publisher<bond::msg::Status>("bond", 10);
  std::atomic<int> published{0};
  // 持续发送对端死亡状态，确定性覆盖离开等待态但从未成功连接的分支。
  auto timer = peer_node->create_wall_timer(100ms, [publisher, &published]() {
      bond::msg::Status message;
      message.id = "dead_peer";
      message.instance_id = "dead_peer_instance";
      message.active = false;
      publisher->publish(message);
      ++published;
    });
  nav2_util::NodeThread peer_thread(peer_node);

  const auto begin = std::chrono::steady_clock::now();
  EXPECT_FALSE(manager_->connect("dead_peer", 5s))
    << "对端已死亡时，即使 waitUntilFormed 返回 true 也必须判为失败。";
  EXPECT_GT(published.load(), 0) << "必须实际注入对端死亡消息。";
  EXPECT_LT(std::chrono::steady_clock::now() - begin, 4s)
    << "本例应由死亡消息触发失败，不能靠等待满 5 秒让用例误通过。";
  EXPECT_TRUE(manager_->connectionFailed());
}

TEST_F(BondConnectionTest, MissingPeerBeyondDefaultConnectTimeoutReturnsFailure)
{
  const auto begin = std::chrono::steady_clock::now();
  EXPECT_FALSE(manager_->connect("missing_peer", 12s))
    << "超过 bond 默认 10 秒窗口时，仍不能把底层断开当成建连成功。";
  EXPECT_GE(std::chrono::steady_clock::now() - begin, 11s)
    << "底层连接超时必须跟随配置，不能在默认 10 秒提前结束。";
  EXPECT_TRUE(manager_->connectionFailed());
}

TEST_F(BondConnectionTest, PeerAppearingAfterTenSecondsCanStillConnect)
{
  auto peer_options = options_;
  peer_options.parameter_overrides({});
  auto peer_node = std::make_shared<rclcpp::Node>("delayed_bond_peer", peer_options);
  auto peer = std::make_shared<bond::Bond>("bond", "delayed_peer", peer_node);
  peer->setHeartbeatPeriod(0.10);
  nav2_util::NodeThread peer_thread(peer_node);

  // 对端在默认 10 秒之后出现；future 析构会等待任务，异常退出也不会遗留线程。
  const auto begin = std::chrono::steady_clock::now();
  auto delayed_start = std::async(std::launch::async, [peer]() {
      std::this_thread::sleep_for(11s);
      peer->start();
    });
  EXPECT_TRUE(manager_->connect("delayed_peer", 14s))
    << "配置窗口内迟到的真实对端应能连接，不能被默认 10 秒超时提前断开。";
  EXPECT_GE(std::chrono::steady_clock::now() - begin, 11s)
    << "管理器必须等待真实对端出现，不能在 10 秒时提前报告成功。";
  delayed_start.get();
  EXPECT_TRUE(peer->waitUntilFormed(rclcpp::Duration(2s)));
  EXPECT_FALSE(peer->isBroken()) << "必须确认真实连接仍然存活。";
  EXPECT_FALSE(manager_->connectionFailed());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
