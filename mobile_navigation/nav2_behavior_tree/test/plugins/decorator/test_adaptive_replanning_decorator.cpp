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
#include <memory>
#include <set>
#include <thread>
#include <utility>
#include <vector>

#include "../../test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/plugins/decorator/adaptive_replanning_decorator.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "nav2_util/node_thread.hpp"

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

  int getTickCount() const { return tick_count_; }
  void resetCount() { tick_count_ = 0; }
  void setReturnStatus(BT::NodeStatus status) { return_status_ = status; }

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

    // 配置节点输入端口映射到 blackboard 变量
    config_->input_ports.insert(std::make_pair("goal", "{goal}"));
    config_->input_ports.insert(std::make_pair("path", "{path}"));
    config_->input_ports.insert(std::make_pair("distance_threshold", "0.3"));
    config_->input_ports.insert(std::make_pair("max_deviation", "0.5"));
    config_->input_ports.insert(std::make_pair("hz", "10.0"));
    config_->input_ports.insert(std::make_pair("global_frame", "map"));
    config_->input_ports.insert(std::make_pair("robot_base_frame", "base_link"));

    bt_node_ = std::make_shared<nav2_behavior_tree::AdaptiveReplanning>(
      "adaptive_replanning", *config_);
    dummy_node_ = std::make_shared<TestDummyNode>();
    bt_node_->setChild(dummy_node_.get());
  }

  void TearDown() override
  {
    dummy_node_.reset();
    bt_node_.reset();
    config_->input_ports.clear();
  }

protected:
  std::shared_ptr<nav2_behavior_tree::AdaptiveReplanning> bt_node_;
  std::shared_ptr<TestDummyNode> dummy_node_;
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

  // 在进行多次手动单步测试前，通过 halt() 将 Decorator 节点及其子节点重置为 IDLE 状态
  bt_node_->halt();

  // 2. 频限等待期间返回 SUCCESS，让外层 ReactiveSequence 继续执行 FollowPath。
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 1); // 运行计数依然是 1

  bt_node_->halt();

  // 3. 延时 120ms 超过频限周期
  std::this_thread::sleep_for(120ms);

  // 4. 第三次 tick：应该放行，子节点被重新 tick
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 2); // 运行计数变为 2
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

  bt_node_->halt();

  // 2. 已完成首次规划且仍在近终点范围内，跳过后续重规划
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 1); // 子节点没有再次执行
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

  bt_node_->halt();

  // 首次规划尚未成功，近目标门控不能拦截第二次尝试。
  dummy_node_->setReturnStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 2);

  bt_node_->halt();

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

  bt_node_->halt();

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

  bt_node_->halt();

  // 2. 紧接着第二次 tick (未到频限周期)：由于偏离路径，必须立刻强行重规划！
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 2); // 再次被放行并 tick，计数变为 2
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
  bt_node_->halt();

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
  bt_node_->halt();

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
  bt_node_->halt();

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
  bt_node_->halt();

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

  bt_node_->halt();

  // 2. 突然收到新目标点 (3.0m)，此时目标改变应该重置 is_first_planning_ 并立刻重新规划
  goal_pose.pose.position.x = 3.0;
  config_->blackboard->set("goal", goal_pose);

  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 2); // 成功进行第二次重规划
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

  bt_node_->halt();

  // 2. 紧接着第二次 tick。虽然时间上未到频限限制，但因为路径为空，应该跳过偏离校验，走常规频限控制，被成功拦截
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 1); // 计数依旧是 1，未因空路径产生异常崩溃
}

TEST_F(AdaptiveReplanningTestFixture, test_persistent_large_path_obstacle_triggers_once)
{
  PathValidityTestServer validity_server({2, 3, 4});
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
  bt_node_->halt();

  for (int i = 0; i < 30 && dummy_node_->getTickCount() < 2; ++i) {
    std::this_thread::sleep_for(10ms);
    EXPECT_NE(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
    bt_node_->halt();
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
  bt_node_->halt();
  for (int i = 0; i < 10; ++i) {
    std::this_thread::sleep_for(10ms);
    EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
    bt_node_->halt();
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

  bt_node_->halt();

  // 位姿仍不可用时继续放行，避免门控节点阻塞全局规划恢复。
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 2);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  bool all_successful = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return all_successful;
}
