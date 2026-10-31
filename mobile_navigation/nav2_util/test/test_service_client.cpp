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

#include <memory>
#include <string>
#include "nav2_util/lifecycle_service_client.hpp"
#include "nav2_util/node_thread.hpp"
#include "nav2_util/service_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/empty.hpp"
#include "gtest/gtest.h"

using nav2_util::ServiceClient;
using std::string;

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class TestServiceClient : public ServiceClient<std_srvs::srv::Empty>
{
public:
  TestServiceClient(
    const std::string & name,
    const rclcpp::Node::SharedPtr & provided_node = rclcpp::Node::SharedPtr())
  : ServiceClient(name, provided_node) {}

  string name() {return node_->get_name();}
  const rclcpp::Node::SharedPtr & getNode() {return node_;}
  size_t prunePendingRequests() {return client_->prune_pending_requests();}
};

TEST(ServiceClient, can_ServiceClient_use_passed_in_node)
{
  auto node = rclcpp::Node::make_shared("test_node");
  TestServiceClient t("bar", node);
  ASSERT_EQ(t.getNode(), node);
  ASSERT_EQ(t.name(), "test_node");
}

TEST(ServiceClient, can_ServiceClient_invoke_in_callback)
{
  int a = 0;
  auto service_node = rclcpp::Node::make_shared("service_node");
  auto service = service_node->create_service<std_srvs::srv::Empty>(
    "empty_srv",
    [&a](std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr) {
      a = 1;
    });
  auto srv_thread = std::thread([&]() {rclcpp::spin(service_node);});

  auto pub_node = rclcpp::Node::make_shared("pub_node");
  auto pub = pub_node->create_publisher<std_msgs::msg::Empty>(
    "empty_topic",
    rclcpp::QoS(1).transient_local());
  auto pub_thread = std::thread([&]() {rclcpp::spin(pub_node);});

  auto sub_node = rclcpp::Node::make_shared("sub_node");
  ServiceClient<std_srvs::srv::Empty> client("empty_srv", sub_node);
  auto sub = sub_node->create_subscription<std_msgs::msg::Empty>(
    "empty_topic",
    rclcpp::QoS(1),
    [&client](std_msgs::msg::Empty::SharedPtr) {
      auto req = std::make_shared<std_srvs::srv::Empty::Request>();
      auto res = client.invoke(req);
    });

  pub->publish(std_msgs::msg::Empty());
  rclcpp::spin_some(sub_node);

  rclcpp::shutdown();
  srv_thread.join();
  pub_thread.join();
  ASSERT_EQ(a, 1);
}

TEST(ServiceClient, can_ServiceClient_timeout)
{
  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("test_node");
  TestServiceClient t("bar", node);
  rclcpp::spin_some(node);
  bool ready = t.wait_for_service(std::chrono::milliseconds(10));
  rclcpp::shutdown();
  ASSERT_EQ(ready, false);
}

class BoundedServiceClientTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override {rclcpp::shutdown();}
};

TEST_F(BoundedServiceClientTest, DiscoveryTimeout)
{
  auto node = rclcpp::Node::make_shared("bounded_discovery_client");
  TestServiceClient client("missing_bounded_service", node);
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  const auto start = std::chrono::steady_clock::now();
  EXPECT_THROW(
    client.invoke_with_timeout(request, std::chrono::milliseconds(100)), std::runtime_error);
  EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::seconds(2));
}

TEST_F(BoundedServiceClientTest, ResponseTimeoutAndLateReply)
{
  auto server = rclcpp::Node::make_shared("bounded_response_server");
  auto service = server->create_service<std_srvs::srv::Empty>(
    "bounded_response_service",
    [](std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr) {});
  auto node = rclcpp::Node::make_shared("bounded_response_client");
  TestServiceClient client("bounded_response_service", node);
  ASSERT_TRUE(client.wait_for_service(std::chrono::seconds(3)));
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();

  // 服务已被发现，但不执行服务回调，模拟请求没有回复。
  const auto start = std::chrono::steady_clock::now();
  EXPECT_THROW(
    client.invoke_with_timeout(request, std::chrono::milliseconds(100)), std::runtime_error);
  EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::seconds(2));
  EXPECT_EQ(client.prunePendingRequests(), 0u);

  // 恢复服务处理后，迟到的回复不能干扰下一次请求。
  nav2_util::NodeThread server_thread(server);
  EXPECT_NE(client.invoke_with_timeout(request, std::chrono::seconds(2)), nullptr);
}

TEST_F(BoundedServiceClientTest, RejectNonPositiveTimeout)
{
  auto node = rclcpp::Node::make_shared("bounded_invalid_timeout_client");
  TestServiceClient client("bounded_invalid_timeout_service", node);
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  EXPECT_THROW(
    client.invoke_with_timeout(request, std::chrono::nanoseconds(0)), std::invalid_argument);
  EXPECT_THROW(
    client.invoke_with_timeout(request, std::chrono::nanoseconds(-1)), std::invalid_argument);
}

TEST_F(BoundedServiceClientTest, LifecycleResponseFailure)
{
  auto server = rclcpp::Node::make_shared("bounded_lifecycle_server");
  auto get_state = server->create_service<lifecycle_msgs::srv::GetState>(
    "bounded_lifecycle/get_state",
    [](lifecycle_msgs::srv::GetState::Request::SharedPtr,
    lifecycle_msgs::srv::GetState::Response::SharedPtr response) {
      response->current_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    });
  auto change_state = server->create_service<lifecycle_msgs::srv::ChangeState>(
    "bounded_lifecycle/change_state",
    [](lifecycle_msgs::srv::ChangeState::Request::SharedPtr,
    lifecycle_msgs::srv::ChangeState::Response::SharedPtr response) {
      response->success = false;
    });
  nav2_util::NodeThread server_thread(server);
  nav2_util::LifecycleServiceClient client("bounded_lifecycle");
  EXPECT_FALSE(client.change_state_with_timeout(
      lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, std::chrono::seconds(2)));
  EXPECT_EQ(client.get_state_with_timeout(std::chrono::seconds(2)),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
}
