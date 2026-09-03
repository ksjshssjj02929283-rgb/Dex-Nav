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

#include <gtest/gtest.h>

#include <memory>

#include "../../test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/global_replan_context.hpp"
#include "nav2_behavior_tree/plugins/action/request_and_wait_for_global_replan_action.hpp"

class RequestAndWaitForGlobalReplanTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  void SetUp() override
  {
    config_->blackboard = BT::Blackboard::create();
    config_->blackboard->set<rclcpp::Node::SharedPtr>("node", node_);
    config_->input_ports["reason"] = "follow_path_failed";

    context_ = nav2_behavior_tree::getOrCreateGlobalReplanContext(config_->blackboard);
    context_->setMode(nav2_behavior_tree::GlobalReplanMode::ACTIVE);
    bt_node_ =
      std::make_shared<nav2_behavior_tree::RequestAndWaitForGlobalReplan>(
      "request_global_replan",
      *config_);
  }

  void TearDown() override
  {
    bt_node_.reset();
    context_.reset();
    config_->input_ports.clear();
  }

protected:
  std::shared_ptr<nav2_behavior_tree::GlobalReplanContext> context_;
  std::shared_ptr<nav2_behavior_tree::RequestAndWaitForGlobalReplan> bt_node_;
};

TEST_F(RequestAndWaitForGlobalReplanTestFixture, waitsForMatchingRequestCompletion)
{
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  const auto request = context_->latestPendingRequest();
  ASSERT_TRUE(request);

  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  const auto repeated_tick_request = context_->latestPendingRequest();
  ASSERT_TRUE(repeated_tick_request);
  EXPECT_EQ(repeated_tick_request->token.request_id, request->token.request_id);

  EXPECT_TRUE(context_->complete(request->token));
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
}

TEST_F(RequestAndWaitForGlobalReplanTestFixture, haltCancelsPendingRequest)
{
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  const auto request = context_->latestPendingRequest();
  ASSERT_TRUE(request);

  bt_node_->halt();

  EXPECT_FALSE(context_->latestPendingRequest());
  EXPECT_TRUE(context_->isCanceled(request->token));
  EXPECT_FALSE(context_->isCompleted(request->token));
}

TEST_F(RequestAndWaitForGlobalReplanTestFixture, canceledRequestNeverBecomesCompleted)
{
  const auto canceled = context_->request(
    nav2_behavior_tree::GlobalReplanReason::FOLLOW_PATH_FAILED);
  ASSERT_TRUE(context_->cancel(canceled));

  const auto completed = context_->request(
    nav2_behavior_tree::GlobalReplanReason::BACK_UP_SUCCEEDED);
  ASSERT_TRUE(context_->complete(completed));

  EXPECT_TRUE(context_->isCanceled(canceled));
  EXPECT_FALSE(context_->isCompleted(canceled));
  EXPECT_TRUE(context_->isCompleted(completed));
}

TEST_F(RequestAndWaitForGlobalReplanTestFixture, canceledRequestReturnsFailure)
{
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  const auto request = context_->latestPendingRequest();
  ASSERT_TRUE(request);
  ASSERT_TRUE(context_->cancel(request->token));

  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
}

TEST_F(RequestAndWaitForGlobalReplanTestFixture, disabledModeSkipsRecoveryReplanRequest)
{
  context_->setMode(nav2_behavior_tree::GlobalReplanMode::DISABLED);

  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(context_->latestPendingRequest());
}

TEST_F(RequestAndWaitForGlobalReplanTestFixture, observeModeOnlyRecordsRecoveryEvent)
{
  context_->setMode(nav2_behavior_tree::GlobalReplanMode::OBSERVE);

  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(context_->latestPendingRequest());
}

TEST_F(RequestAndWaitForGlobalReplanTestFixture, newerRequestIsNotCompletedByOlderToken)
{
  const auto first = context_->request(nav2_behavior_tree::GlobalReplanReason::FOLLOW_PATH_FAILED);
  const auto second = context_->request(nav2_behavior_tree::GlobalReplanReason::BACK_UP_SUCCEEDED);

  EXPECT_TRUE(context_->complete(first));
  const auto pending = context_->latestPendingRequest();
  ASSERT_TRUE(pending);
  EXPECT_EQ(pending->token.request_id, second.request_id);
  EXPECT_EQ(pending->reason, nav2_behavior_tree::GlobalReplanReason::BACK_UP_SUCCEEDED);
}

TEST_F(RequestAndWaitForGlobalReplanTestFixture, latestPlanCompletesEarlierPendingRequests)
{
  const auto first = context_->request(nav2_behavior_tree::GlobalReplanReason::FOLLOW_PATH_FAILED);
  const auto second = context_->request(nav2_behavior_tree::GlobalReplanReason::BACK_UP_SUCCEEDED);

  EXPECT_TRUE(context_->complete(second));
  EXPECT_TRUE(context_->isCompleted(first));
  EXPECT_TRUE(context_->isCompleted(second));
  EXPECT_FALSE(context_->latestPendingRequest());
}

TEST_F(RequestAndWaitForGlobalReplanTestFixture, backUpReasonCreatesRequest)
{
  config_->input_ports["reason"] = "back_up_succeeded";
  bt_node_ =
    std::make_shared<nav2_behavior_tree::RequestAndWaitForGlobalReplan>(
    "request_global_replan_after_back_up", *config_);

  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  const auto request = context_->latestPendingRequest();
  ASSERT_TRUE(request);
  EXPECT_EQ(request->reason, nav2_behavior_tree::GlobalReplanReason::BACK_UP_SUCCEEDED);
  bt_node_->halt();
}

TEST_F(RequestAndWaitForGlobalReplanTestFixture, invalidReasonReturnsFailure)
{
  config_->input_ports["reason"] = "invalid_reason";
  bt_node_ =
    std::make_shared<nav2_behavior_tree::RequestAndWaitForGlobalReplan>(
    "request_global_replan_invalid_reason", *config_);

  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
  EXPECT_FALSE(context_->latestPendingRequest());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const bool all_successful = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return all_successful;
}
