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
#include <chrono>
#include <limits>
#include <memory>
#include <set>
#include <thread>
#include <utility>
#include <vector>

#include "../../test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/plugins/action/request_and_wait_for_global_replan_action.hpp"
#include "nav2_behavior_tree/plugins/control/pipeline_sequence.hpp"
#include "nav2_behavior_tree/plugins/decorator/adaptive_replanning_decorator.hpp"
#include "nav2_util/navigation_protocol.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "nav2_util/node_thread.hpp"
#include "std_msgs/msg/u_int64.hpp"

using namespace std::chrono_literals;

// 定义带有计数器的 Mock 子节点，规避 Decorator 节点自动 haltChild() 重置状态导致状态断言不准的问题
class TestDummyNode : public BT::ActionNodeBase
{
public:
  TestDummyNode()
  : BT::ActionNodeBase("test_dummy", {}),
    tick_count_(0),
    return_status_(BT::NodeStatus::SUCCESS)
  {}

  BT::NodeStatus executeTick() override
  {
    return tick();
  }

  BT::NodeStatus tick() override
  {
    tick_count_++;
    return return_status_;
  }

  void halt() override {}

  int getTickCount() const {return tick_count_;}
  void resetCount() {tick_count_ = 0;}
  void setReturnStatus(BT::NodeStatus status) {return_status_ = status;}

private:
  int tick_count_;
  BT::NodeStatus return_status_;
};

class PathValidityTestServer
{
public:
  explicit PathValidityTestServer(std::vector<int32_t> invalid_indices)
  : invalid_indices_(std::move(invalid_indices)),
    node_(std::make_shared<rclcpp::Node>("adaptive_replanning_path_validity_server"))
  {
    service_ = node_->create_service<nav2_msgs::srv::IsPathValid>(
      "is_path_valid",
      [this](
        const std::shared_ptr<nav2_msgs::srv::IsPathValid::Request>,
        std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> response)
      {
        response->invalid_pose_indices = invalid_indices_;
        response->is_valid = invalid_indices_.empty();
      });
    thread_ = std::make_unique<nav2_util::NodeThread>(node_->get_node_base_interface());
  }

private:
  std::vector<int32_t> invalid_indices_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<nav2_msgs::srv::IsPathValid>::SharedPtr service_;
  std::unique_ptr<nav2_util::NodeThread> thread_;
};

class AdaptiveReplanningTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  void SetUp() override
  {
    // 基础 fixture 的 blackboard 是静态共享的；每个用例使用独立实例，避免 goal/path 串扰。
    config_->blackboard = BT::Blackboard::create();
    config_->blackboard->set<rclcpp::Node::SharedPtr>("node", node_);
    config_->blackboard->set<std::shared_ptr<tf2_ros::Buffer>>(
      "tf_buffer", transform_handler_->getBuffer());
    config_->blackboard->set<uint64_t>(
      nav2_behavior_tree::kPrepareNavigationSessionKey, 1001);

    // 配置节点输入端口映射到 blackboard 变量
    config_->input_ports.insert(std::make_pair("goal", "{goal}"));
    config_->input_ports.insert(std::make_pair("path", "{path}"));
    config_->input_ports.insert(std::make_pair("distance_threshold", "0.3"));
    config_->input_ports.insert(std::make_pair("max_deviation", "0.5"));
    config_->input_ports.insert(std::make_pair("hz", "10.0"));
    config_->input_ports.insert(std::make_pair("global_frame", "map"));
    config_->input_ports.insert(std::make_pair("robot_base_frame", "base_link"));
    config_->input_ports.insert(std::make_pair("planning_timeout_s", "0.0"));
    config_->input_ports.insert(std::make_pair("recovery_replan_mode", "active"));

    global_replan_context_ = nav2_behavior_tree::getOrCreateGlobalReplanContext(
      config_->blackboard);
    bt_node_ = std::make_shared<nav2_behavior_tree::AdaptiveReplanning>(
      "adaptive_replanning", *config_);
    dummy_node_ = std::make_shared<TestDummyNode>();
    bt_node_->setChild(dummy_node_.get());
  }

  void TearDown() override
  {
    dummy_node_.reset();
    bt_node_.reset();
    global_replan_context_.reset();
    config_->input_ports.clear();
  }

protected:
  std::shared_ptr<nav2_behavior_tree::AdaptiveReplanning> bt_node_;
  std::shared_ptr<TestDummyNode> dummy_node_;
  std::shared_ptr<nav2_behavior_tree::GlobalReplanContext> global_replan_context_;
};

TEST_F(AdaptiveReplanningTestFixture, test_rate_limiting)
{
  // 准备常规目标点（较远，2.0m）和不偏离的路径（x=0m，机器人在 0m，偏离为 0m）
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 2.0;
  config_->blackboard->set("goal", goal_pose);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = 0.0;
  path.poses.push_back(p);
  config_->blackboard->set("path", path);

  EXPECT_EQ(bt_node_->status(), BT::NodeStatus::IDLE);

  // 1. 首次规划：应该被放行，子节点被 tick
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 1);

  // 2. 紧接着第二次 tick（频限为 10Hz，即周期 0.1s）：应直接返回 SUCCESS，
  // 且不再 tick 子节点。
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 1);  // 运行计数依然是 1

  // 3. 延时 120ms 超过频限周期
  std::this_thread::sleep_for(120ms);

  // 4. 第三次 tick：应该放行，子节点被重新 tick
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 2);  // 运行计数变为 2
}

TEST_F(AdaptiveReplanningTestFixture, test_planning_timeout_fails_task)
{
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 2.0;
  config_->blackboard->set("goal", goal_pose);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.emplace_back();
  config_->blackboard->set("path", path);

  const auto active_request = global_replan_context_->request(
    nav2_behavior_tree::GlobalReplanReason::FOLLOW_PATH_FAILED);

  bt_node_.reset();
  config_->input_ports["planning_timeout_s"] = "0.05";
  bt_node_ = std::make_shared<nav2_behavior_tree::AdaptiveReplanning>(
    "adaptive_replanning_timeout", *config_);
  bt_node_->setChild(dummy_node_.get());

  dummy_node_->setReturnStatus(BT::NodeStatus::RUNNING);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(dummy_node_->getTickCount(), 1);

  std::this_thread::sleep_for(120ms);

  // 超时后取消当前请求、不再 tick 子节点，而是 halt 并向上返回 FAILURE。
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(dummy_node_->getTickCount(), 1);
  EXPECT_TRUE(global_replan_context_->isCanceled(active_request));
}

TEST_F(AdaptiveReplanningTestFixture, test_zero_planning_timeout_disables_timeout)
{
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 2.0;
  config_->blackboard->set("goal", goal_pose);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.emplace_back();
  config_->blackboard->set("path", path);

  bt_node_.reset();
  config_->input_ports["planning_timeout_s"] = "0.0";
  bt_node_ = std::make_shared<nav2_behavior_tree::AdaptiveReplanning>(
    "adaptive_replanning_timeout_disabled", *config_);
  bt_node_->setChild(dummy_node_.get());

  dummy_node_->setReturnStatus(BT::NodeStatus::RUNNING);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(dummy_node_->getTickCount(), 1);

  std::this_thread::sleep_for(120ms);

  // 0 表示关闭超时，即使规划持续时间明显超过测试时限也必须继续运行。
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(dummy_node_->getTickCount(), 2);

  dummy_node_->setReturnStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
}

TEST_F(AdaptiveReplanningTestFixture, test_negative_planning_timeout_uses_safe_default)
{
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 2.0;
  config_->blackboard->set("goal", goal_pose);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.emplace_back();
  config_->blackboard->set("path", path);

  bt_node_.reset();
  config_->input_ports["planning_timeout_s"] = "-1.0";
  bt_node_ = std::make_shared<nav2_behavior_tree::AdaptiveReplanning>(
    "adaptive_replanning_invalid_timeout", *config_);
  bt_node_->setChild(dummy_node_.get());

  dummy_node_->setReturnStatus(BT::NodeStatus::RUNNING);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(dummy_node_->getTickCount(), 1);

  std::this_thread::sleep_for(120ms);

  // 非法负数不能产生“立即超时”，应回退到默认 30s。
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(dummy_node_->getTickCount(), 2);
}

TEST_F(AdaptiveReplanningTestFixture, test_planning_timeout_restarts_for_next_attempt)
{
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 2.0;
  config_->blackboard->set("goal", goal_pose);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.emplace_back();
  config_->blackboard->set("path", path);

  bt_node_.reset();
  config_->input_ports["planning_timeout_s"] = "0.2";
  bt_node_ = std::make_shared<nav2_behavior_tree::AdaptiveReplanning>(
    "adaptive_replanning_timeout_restart", *config_);
  bt_node_->setChild(dummy_node_.get());

  dummy_node_->setReturnStatus(BT::NodeStatus::RUNNING);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  std::this_thread::sleep_for(250ms);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(dummy_node_->getTickCount(), 1);

  // 失败后的下一次 tick 应开启新规划窗口，而不是沿用上一轮的开始时间。
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(dummy_node_->getTickCount(), 2);

  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(dummy_node_->getTickCount(), 3);

  std::this_thread::sleep_for(150ms);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(dummy_node_->getTickCount(), 3);
}

TEST_F(AdaptiveReplanningTestFixture, test_global_replanning_disabled_keeps_initial_and_goal_plans)
{
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 2.0;
  config_->blackboard->set("goal", goal_pose);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.emplace_back();
  config_->blackboard->set("path", path);

  bt_node_.reset();
  config_->input_ports["enable_global_replanning"] = "false";
  bt_node_ = std::make_shared<nav2_behavior_tree::AdaptiveReplanning>(
    "adaptive_replanning_disabled", *config_);
  bt_node_->setChild(dummy_node_.get());

  EXPECT_EQ(global_replan_context_->mode(), nav2_behavior_tree::GlobalReplanMode::DISABLED);

  // Initial planning is always required, even with replanning disabled.
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 1);

  // Periodic and deviation replanning remain disabled after the first valid path.
  std::this_thread::sleep_for(120ms);
  path.poses.front().pose.position.x = 10.0;
  config_->blackboard->set("path", path);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 1);

  // A changed goal invalidates the old path and must still be planned once.
  goal_pose.pose.position.x = 3.0;
  config_->blackboard->set("goal", goal_pose);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 2);
}

TEST_F(AdaptiveReplanningTestFixture, test_near_goal_skip)
{
  // 准备近终点目标点 (0.1m，小于距离阈值 0.3m)
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 0.1;
  config_->blackboard->set("goal", goal_pose);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = 0.0;
  path.poses.push_back(p);
  config_->blackboard->set("path", path);

  // 1. 首次全局规划优先于近终点判断，必须放行一次以生成有效路径
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 1);

  // 2. 已完成首次规划且仍在近终点范围内，跳过后续重规划
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 1);  // 子节点没有再次执行
}

TEST_F(AdaptiveReplanningTestFixture, test_failed_first_planning_keeps_first_planning_priority)
{
  // 即使已在近终点范围内，首次规划失败后也必须继续放行恢复重试。
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 0.1;
  config_->blackboard->set("goal", goal_pose);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = 0.0;
  path.poses.push_back(p);
  config_->blackboard->set("path", path);

  dummy_node_->setReturnStatus(BT::NodeStatus::FAILURE);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(dummy_node_->getTickCount(), 1);

  // 首次规划尚未成功，近目标门控不能拦截第二次尝试。
  dummy_node_->setReturnStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 2);

  // 已经成功生成首条路径，之后才允许近目标门控跳过重规划。
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 2);
}

TEST_F(AdaptiveReplanningTestFixture, test_running_first_planning_keeps_first_planning_priority)
{
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 0.1;
  config_->blackboard->set("goal", goal_pose);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = 0.0;
  path.poses.push_back(p);
  config_->blackboard->set("path", path);

  dummy_node_->setReturnStatus(BT::NodeStatus::RUNNING);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(dummy_node_->getTickCount(), 1);

  // RUNNING 不是有效首条路径；下一次 tick 仍应执行规划子节点。
  dummy_node_->setReturnStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 2);

  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 2);
}

TEST_F(AdaptiveReplanningTestFixture, test_deviation_force_replan)
{
  // 准备常规较远目标点
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 2.0;
  config_->blackboard->set("goal", goal_pose);

  // 准备偏离较远的路径 (x=1.0m，偏离 1.0m > 0.5m 最大限制)
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = 1.0;
  path.poses.push_back(p);
  config_->blackboard->set("path", path);

  // 1. 首次规划放行
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 1);

  // 2. 紧接着第二次 tick (未到频限周期)：由于偏离路径，必须立刻强行重规划！
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 2);  // 再次被放行并 tick，计数变为 2
}

TEST_F(AdaptiveReplanningTestFixture, test_deviation_below_threshold_does_not_replan)
{
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 2.0;
  config_->blackboard->set("goal", goal_pose);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = 0.49;
  path.poses.push_back(p);
  config_->blackboard->set("path", path);

  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 1);
  // 0.49 m < 0.5 m，且频率周期未到，不应重规划。
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 1);
}

TEST_F(AdaptiveReplanningTestFixture, test_deviation_equal_to_threshold_does_not_replan)
{
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 2.0;
  config_->blackboard->set("goal", goal_pose);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = 0.5;
  path.poses.push_back(p);
  config_->blackboard->set("path", path);

  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 1);
  // 判定使用严格大于号，恰好 0.5 m 不应触发偏离重规划。
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 1);
}

TEST_F(AdaptiveReplanningTestFixture, test_deviation_just_above_threshold_replans)
{
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 2.0;
  config_->blackboard->set("goal", goal_pose);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = 0.51;
  path.poses.push_back(p);
  config_->blackboard->set("path", path);

  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 1);
  // 0.51 m > 0.5 m，应绕过频率限制立即重规划。
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 2);
}

TEST_F(AdaptiveReplanningTestFixture, test_near_goal_suppresses_deviation_after_first_success)
{
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 0.1;
  config_->blackboard->set("goal", goal_pose);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = 1.0;
  path.poses.push_back(p);
  config_->blackboard->set("path", path);

  // 首次规划不受近目标或路径偏离门控。
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 1);
  // 首次成功后，近目标与偏离同时成立时，近目标门控优先，不再规划。
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 1);
}

TEST_F(AdaptiveReplanningTestFixture, test_goal_changed_reset)
{
  // 1. 设定常规初始目标点 (2.0m)
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 2.0;
  config_->blackboard->set("goal", goal_pose);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = 0.0;
  path.poses.push_back(p);
  config_->blackboard->set("path", path);

  // 首次规划，应该放行
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 1);

  // 2. 突然收到新目标点 (3.0m)，此时目标改变应该使现有路径失效并立刻重新规划。
  goal_pose.pose.position.x = 3.0;
  config_->blackboard->set("goal", goal_pose);

  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 2);  // 成功进行第二次重规划
}

TEST_F(AdaptiveReplanningTestFixture, test_empty_path_skip_deviation_check)
{
  // 1. 设定常规较远目标点 (2.0m)
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 2.0;
  config_->blackboard->set("goal", goal_pose);

  // 不向 blackboard 设置任何路径信息，使其保持为空

  // 首次规划放行
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 1);

  // 2. 紧接着第二次 tick。虽然尚未到达重规划周期，但路径为空时应跳过偏离校验，
  // 继续由常规频限控制拦截。
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 1);  // 计数依旧是 1，未因空路径产生异常崩溃
}

TEST_F(AdaptiveReplanningTestFixture, test_persistent_large_path_obstacle_triggers_once)
{
  PathValidityTestServer validity_server({2, 3, 4});
  bt_node_.reset();
  config_->input_ports["hz"] = "0.001";
  config_->input_ports["path_obstacle_check_hz"] = "100.0";
  config_->input_ports["invalid_path_confirmations"] = "2";
  config_->input_ports["min_blocked_path_poses"] = "3";
  bt_node_ = std::make_shared<nav2_behavior_tree::AdaptiveReplanning>(
    "adaptive_replanning_large_obstacle", *config_);
  bt_node_->setChild(dummy_node_.get());

  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 2.0;
  config_->blackboard->set("goal", goal_pose);
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (int i = 0; i < 8; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = 0.1 * i;
    path.poses.push_back(pose);
  }
  config_->blackboard->set("path", path);

  ASSERT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(dummy_node_->getTickCount(), 1);

  // halt() 会要求下一次执行重新生成首条路径，因此这里只连续 tick，不在中间 halt。
  for (int i = 0; i < 30 && dummy_node_->getTickCount() < 2; ++i) {
    std::this_thread::sleep_for(10ms);
    EXPECT_NE(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
  }
  EXPECT_EQ(dummy_node_->getTickCount(), 2)
    << "Two confirmed checks of three consecutive blocked poses must trigger one replan";

  // The successful replan clears the latch, so a following BT tick cannot retrigger immediately.
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 2);
}

TEST_F(AdaptiveReplanningTestFixture, test_obstacle_replan_cooldown_delays_revalidation)
{
  PathValidityTestServer validity_server({2, 3, 4});
  bt_node_.reset();
  config_->input_ports["hz"] = "0.001";
  config_->input_ports["path_obstacle_check_hz"] = "100.0";
  config_->input_ports["invalid_path_confirmations"] = "1";
  config_->input_ports["min_blocked_path_poses"] = "3";
  config_->input_ports["obstacle_replan_cooldown"] = "0.15";
  bt_node_ = std::make_shared<nav2_behavior_tree::AdaptiveReplanning>(
    "adaptive_replanning_cooldown", *config_);
  bt_node_->setChild(dummy_node_.get());

  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 2.0;
  config_->blackboard->set("goal", goal_pose);
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  path.poses.assign(8, pose);
  config_->blackboard->set("path", path);

  ASSERT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(dummy_node_->getTickCount(), 1);

  for (int i = 0; i < 30 && dummy_node_->getTickCount() < 2; ++i) {
    std::this_thread::sleep_for(10ms);
    EXPECT_NE(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
  }
  ASSERT_EQ(dummy_node_->getTickCount(), 2);

  // The obstacle remains present, but the replacement path must not be checked during cooldown.
  for (int i = 0; i < 10; ++i) {
    std::this_thread::sleep_for(10ms);
    EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  }
  EXPECT_EQ(dummy_node_->getTickCount(), 2);

  // Once cooldown expires, persistent blockage is eligible to trigger another replan.
  for (int i = 0; i < 20 && dummy_node_->getTickCount() < 3; ++i) {
    std::this_thread::sleep_for(10ms);
    EXPECT_NE(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
  }
  EXPECT_EQ(dummy_node_->getTickCount(), 3);
}

TEST_F(AdaptiveReplanningTestFixture, test_path_validity_keeps_polling_without_external_halt)
{
  PathValidityTestServer validity_server({3});
  bt_node_.reset();
  config_->input_ports["hz"] = "0.001";
  config_->input_ports["path_obstacle_check_hz"] = "100.0";
  config_->input_ports["invalid_path_confirmations"] = "2";
  config_->input_ports["min_blocked_path_poses"] = "1";
  bt_node_ = std::make_shared<nav2_behavior_tree::AdaptiveReplanning>(
    "adaptive_replanning_continuous_poll", *config_);
  bt_node_->setChild(dummy_node_.get());

  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 2.0;
  config_->blackboard->set("goal", goal_pose);
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  path.poses.assign(8, pose);
  config_->blackboard->set("path", path);

  ASSERT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(dummy_node_->getTickCount(), 1);

  // ReactiveSequence re-ticks this successful decorator while FollowPath is RUNNING;
  // it does not halt the decorator between ticks. Reproduce that exact runtime pattern.
  for (int i = 0; i < 40 && dummy_node_->getTickCount() < 2; ++i) {
    std::this_thread::sleep_for(10ms);
    EXPECT_NE(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
  }
  EXPECT_EQ(dummy_node_->getTickCount(), 2)
    << "Path validity polling must continue while the downstream controller is RUNNING";
}

TEST_F(AdaptiveReplanningTestFixture, test_single_blocked_path_pose_does_not_trigger)
{
  PathValidityTestServer validity_server({3});
  bt_node_.reset();
  config_->input_ports["hz"] = "0.001";
  config_->input_ports["path_obstacle_check_hz"] = "100.0";
  config_->input_ports["invalid_path_confirmations"] = "1";
  config_->input_ports["min_blocked_path_poses"] = "3";
  bt_node_ = std::make_shared<nav2_behavior_tree::AdaptiveReplanning>(
    "adaptive_replanning_single_obstacle", *config_);
  bt_node_->setChild(dummy_node_.get());

  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 2.0;
  config_->blackboard->set("goal", goal_pose);
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  path.poses.assign(8, pose);
  config_->blackboard->set("path", path);

  ASSERT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  for (int i = 0; i < 10; ++i) {
    std::this_thread::sleep_for(10ms);
    EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  }
  EXPECT_EQ(dummy_node_->getTickCount(), 1)
    << "A single occupied path pose is below the configured large-obstacle threshold";
}

TEST_F(AdaptiveReplanningTestFixture, test_missing_goal_falls_back_to_planning)
{
  // 目标端口不可读时采用保守策略：不拦截规划子树。
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 1);
}

TEST_F(AdaptiveReplanningTestFixture, test_unavailable_robot_pose_falls_back_to_planning)
{
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 2.0;
  config_->blackboard->set("goal", goal_pose);

  // 使用不存在的全局坐标系，强制 getCurrentPose() 失败。
  config_->input_ports["global_frame"] = "unavailable_map";
  bt_node_ = std::make_shared<nav2_behavior_tree::AdaptiveReplanning>(
    "adaptive_replanning_unavailable_pose", *config_);
  bt_node_->setChild(dummy_node_.get());

  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 1);

  // 位姿仍不可用时继续放行，避免门控节点阻塞全局规划恢复。
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 2);
}

TEST_F(AdaptiveReplanningTestFixture, test_recovery_request_bypasses_near_goal_gate)
{
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 0.1;
  config_->blackboard->set("goal", goal_pose);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.emplace_back();
  config_->blackboard->set("path", path);

  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  const auto token = global_replan_context_->request(
    nav2_behavior_tree::GlobalReplanReason::FOLLOW_PATH_FAILED);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);

  EXPECT_EQ(dummy_node_->getTickCount(), 2);
  EXPECT_TRUE(global_replan_context_->isCompleted(token));
}

TEST_F(AdaptiveReplanningTestFixture, test_prepare_backup_requests_one_replan_per_goal_session)
{
  constexpr uint64_t kSessionId = 1001;

  auto request_pub = node_->create_publisher<std_msgs::msg::UInt64>(
    nav2_util::navigation_protocol::kPrepareReplanTopic,
    nav2_util::navigation_protocol::stateTopicQoS());
  ASSERT_NE(request_pub, nullptr);

  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 0.1;
  config_->blackboard->set("goal", goal_pose);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.emplace_back();
  config_->blackboard->set("path", path);

  // 会话由顶层 Navigator 写入黑板；装饰器只消费与当前会话匹配的请求。
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 1);

  std_msgs::msg::UInt64 request;
  request.data = kSessionId;
  request_pub->publish(request);
  ASSERT_TRUE(request_pub->wait_for_all_acked(1s));

  // 只执行行为树 tick，不手动 spin BT node；装饰器必须自行处理订阅回调。
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 2);

  // 相同会话的重复事件不能造成重复规划。
  request_pub->publish(request);
  ASSERT_TRUE(request_pub->wait_for_all_acked(1s));
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 2);
}

TEST_F(AdaptiveReplanningTestFixture, test_prepare_request_during_running_plan_needs_next_plan)
{
  constexpr uint64_t kSessionId = 1002;
  config_->blackboard->set<uint64_t>(
    nav2_behavior_tree::kPrepareNavigationSessionKey, kSessionId);

  auto request_pub = node_->create_publisher<std_msgs::msg::UInt64>(
    nav2_util::navigation_protocol::kPrepareReplanTopic,
    nav2_util::navigation_protocol::stateTopicQoS());
  ASSERT_NE(request_pub, nullptr);

  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 2.0;
  config_->blackboard->set("goal", goal_pose);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.emplace_back();
  config_->blackboard->set("path", path);

  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);

  // 先启动一轮普通异步规划，再让 PREPARE 请求在该规划运行期间到达。
  path.poses.front().pose.position.x = 1.0;
  config_->blackboard->set("path", path);
  dummy_node_->setReturnStatus(BT::NodeStatus::RUNNING);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);

  std_msgs::msg::UInt64 request;
  request.data = kSessionId;
  request_pub->publish(request);
  ASSERT_TRUE(request_pub->wait_for_all_acked(1s));

  dummy_node_->setReturnStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 3);

  // 运行中的旧规划不能满足后来到达的 PREPARE 请求，下一次 tick 必须再规划。
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 4);
}

TEST_F(AdaptiveReplanningTestFixture, test_prepare_replan_supports_multi_pose_goal)
{
  constexpr uint64_t kSessionId = 1003;
  config_->blackboard->set<uint64_t>(
    nav2_behavior_tree::kPrepareNavigationSessionKey, kSessionId);

  bt_node_.reset();
  config_->input_ports.erase("goal");
  config_->input_ports.insert(std::make_pair("goals", "{goals}"));

  std::vector<geometry_msgs::msg::PoseStamped> goals(2);
  goals.back().header.frame_id = "map";
  goals.back().pose.position.x = 2.0;
  config_->blackboard->set("goals", goals);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.emplace_back();
  config_->blackboard->set("path", path);

  bt_node_ = std::make_shared<nav2_behavior_tree::AdaptiveReplanning>(
    "adaptive_replanning_multi_pose", *config_);
  bt_node_->setChild(dummy_node_.get());

  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 1);

  auto request_pub = node_->create_publisher<std_msgs::msg::UInt64>(
    nav2_util::navigation_protocol::kPrepareReplanTopic,
    nav2_util::navigation_protocol::stateTopicQoS());
  std_msgs::msg::UInt64 request;
  request.data = kSessionId;
  request_pub->publish(request);
  ASSERT_TRUE(request_pub->wait_for_all_acked(1s));

  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 2);
}

TEST_F(AdaptiveReplanningTestFixture, test_new_session_replans_same_goal)
{
  bt_node_.reset();
  config_->input_ports["hz"] = "0.001";

  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 2.0;
  config_->blackboard->set("goal", goal_pose);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.emplace_back();
  config_->blackboard->set("path", path);

  bt_node_ = std::make_shared<nav2_behavior_tree::AdaptiveReplanning>(
    "adaptive_replanning_new_session", *config_);
  bt_node_->setChild(dummy_node_.get());

  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 1);

  // 目标坐标相同也属于新导航任务，session 变化必须重新生成首条路径。
  config_->blackboard->set<uint64_t>(
    nav2_behavior_tree::kPrepareNavigationSessionKey, 1002);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 2);
}

TEST_F(AdaptiveReplanningTestFixture, test_failed_recovery_plan_keeps_request_pending)
{
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 0.1;
  config_->blackboard->set("goal", goal_pose);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.emplace_back();
  config_->blackboard->set("path", path);

  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  const auto token = global_replan_context_->request(
    nav2_behavior_tree::GlobalReplanReason::FOLLOW_PATH_FAILED);
  dummy_node_->setReturnStatus(BT::NodeStatus::FAILURE);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);

  EXPECT_FALSE(global_replan_context_->isCompleted(token));
  ASSERT_TRUE(global_replan_context_->latestPendingRequest());
}

TEST_F(AdaptiveReplanningTestFixture, test_request_during_running_plan_needs_next_plan)
{
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 2.0;
  config_->blackboard->set("goal", goal_pose);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped path_pose;
  path_pose.pose.position.x = 0.0;
  path.poses.push_back(path_pose);
  config_->blackboard->set("path", path);

  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  // 先通过路径偏离启动一次与恢复请求无关的异步规划。
  path.poses.front().pose.position.x = 1.0;
  config_->blackboard->set("path", path);
  dummy_node_->setReturnStatus(BT::NodeStatus::RUNNING);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);

  const auto token = global_replan_context_->request(
    nav2_behavior_tree::GlobalReplanReason::FOLLOW_PATH_FAILED);
  dummy_node_->setReturnStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);

  // 请求发生在规划开始之后，旧规划成功不能确认这个请求。
  EXPECT_FALSE(global_replan_context_->isCompleted(token));
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(global_replan_context_->isCompleted(token));
  EXPECT_EQ(dummy_node_->getTickCount(), 4);
}

TEST_F(AdaptiveReplanningTestFixture, test_halt_requires_initial_plan_for_same_goal)
{
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 0.1;
  config_->blackboard->set("goal", goal_pose);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.emplace_back();
  config_->blackboard->set("path", path);

  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 1);

  // 行为树结束后重新执行同一目标，也必须生成以当前位置为起点的新路径。
  bt_node_->halt();
  config_->blackboard->set("path", nav_msgs::msg::Path());
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 2);
}

TEST_F(AdaptiveReplanningTestFixture, test_pipeline_waits_for_requested_plan)
{
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 0.1;
  config_->blackboard->set("goal", goal_pose);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.emplace_back();
  config_->blackboard->set("path", path);

  // 先完成首条路径，后续请求用于模拟FollowPath失败后的恢复重规划。
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);

  BT::NodeConfiguration request_config;
  request_config.blackboard = config_->blackboard;
  request_config.input_ports["reason"] = "follow_path_failed";
  auto request_node =
    std::make_shared<nav2_behavior_tree::RequestAndWaitForGlobalReplan>(
    "request_global_replan", request_config);

  BT::NodeConfiguration pipeline_config;
  pipeline_config.blackboard = config_->blackboard;
  auto pipeline = std::make_shared<nav2_behavior_tree::PipelineSequence>(
    "replanning_pipeline", pipeline_config);
  pipeline->addChild(bt_node_.get());
  pipeline->addChild(request_node.get());

  // 第一次tick只创建请求；下一次tick才由前置规划分支观察并处理请求。
  EXPECT_EQ(pipeline->executeTick(), BT::NodeStatus::RUNNING);
  const auto request = global_replan_context_->latestPendingRequest();
  ASSERT_TRUE(request);
  EXPECT_EQ(dummy_node_->getTickCount(), 1);

  dummy_node_->setReturnStatus(BT::NodeStatus::RUNNING);
  EXPECT_EQ(pipeline->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_FALSE(global_replan_context_->isCompleted(request->token));
  EXPECT_EQ(dummy_node_->getTickCount(), 2);

  dummy_node_->setReturnStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(pipeline->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(global_replan_context_->isCompleted(request->token));
  EXPECT_FALSE(global_replan_context_->latestPendingRequest());
  EXPECT_EQ(dummy_node_->getTickCount(), 3);
}

TEST_F(AdaptiveReplanningTestFixture, test_invalid_mode_falls_back_to_disabled)
{
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 0.1;
  config_->blackboard->set("goal", goal_pose);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.emplace_back();
  config_->blackboard->set("path", path);

  // 非法静态配置必须安全回退disabled，不能意外开启恢复重规划。
  bt_node_.reset();
  config_->input_ports["recovery_replan_mode"] = "invalid_mode";
  bt_node_ = std::make_shared<nav2_behavior_tree::AdaptiveReplanning>(
    "adaptive_replanning_invalid_mode", *config_);
  bt_node_->setChild(dummy_node_.get());

  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  const auto token = global_replan_context_->request(
    nav2_behavior_tree::GlobalReplanReason::FOLLOW_PATH_FAILED);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);

  EXPECT_EQ(dummy_node_->getTickCount(), 1);
  EXPECT_FALSE(global_replan_context_->isCompleted(token));

  // 即使新一轮任务必须规划，disabled也不能顺带确认恢复请求。
  bt_node_->halt();
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 2);
  EXPECT_FALSE(global_replan_context_->isCompleted(token));
}

TEST_F(AdaptiveReplanningTestFixture, test_nonfinite_distance_threshold_uses_default)
{
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 0.4;
  config_->blackboard->set("goal", goal_pose);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.emplace_back();
  config_->blackboard->set("path", path);

  bt_node_.reset();
  config_->input_ports["distance_threshold"] = "{invalid_distance_threshold}";
  config_->blackboard->set(
    "invalid_distance_threshold", std::numeric_limits<double>::infinity());
  bt_node_ = std::make_shared<nav2_behavior_tree::AdaptiveReplanning>(
    "adaptive_replanning_invalid_distance", *config_);
  bt_node_->setChild(dummy_node_.get());

  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  std::this_thread::sleep_for(120ms);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);

  // 默认阈值为 0.3 m；0.4 m 不属于近目标，应在频率周期到达后重新规划。
  EXPECT_EQ(dummy_node_->getTickCount(), 2);
}

TEST_F(AdaptiveReplanningTestFixture, test_negative_max_deviation_uses_default)
{
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 2.0;
  config_->blackboard->set("goal", goal_pose);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.emplace_back();
  path.poses.front().pose.position.x = 0.1;
  config_->blackboard->set("path", path);

  bt_node_.reset();
  config_->input_ports["max_deviation"] = "-0.1";
  bt_node_ = std::make_shared<nav2_behavior_tree::AdaptiveReplanning>(
    "adaptive_replanning_invalid_deviation", *config_);
  bt_node_->setChild(dummy_node_.get());

  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);

  // 默认阈值为 0.5 m；0.1 m 偏差不能绕过频率门控触发重规划。
  EXPECT_EQ(dummy_node_->getTickCount(), 1);
}

TEST_F(AdaptiveReplanningTestFixture, test_nonfinite_frequency_uses_default)
{
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 2.0;
  config_->blackboard->set("goal", goal_pose);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.emplace_back();
  config_->blackboard->set("path", path);

  bt_node_.reset();
  config_->input_ports["hz"] = "{invalid_hz}";
  config_->blackboard->set("invalid_hz", std::numeric_limits<double>::quiet_NaN());
  bt_node_ = std::make_shared<nav2_behavior_tree::AdaptiveReplanning>(
    "adaptive_replanning_invalid_frequency", *config_);
  bt_node_->setChild(dummy_node_.get());

  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);

  // 非有限频率回退默认 0.02 Hz，连续两个 tick 之间不应立即重复规划。
  EXPECT_EQ(dummy_node_->getTickCount(), 1);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  bool all_successful = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return all_successful;
}
