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

#include "../../test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/plugins/decorator/adaptive_replanning_decorator.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

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

  // 2. 紧接着第二次 tick (频限限制在 10Hz，即周期 0.1s)：应该被频限拦截，直接返回 SUCCESS 且不 tick 子节点
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 1); // 运行计数依然是 1

  bt_node_->halt();

  // 3. 延时 120ms 超过频限周期
  std::this_thread::sleep_for(120ms);

  // 4. 第三次 tick：应该放行，子节点被重新 tick
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->getTickCount(), 2); // 运行计数变为 2
}

TEST_F(AdaptiveReplanningTestFixture, test_timing_diagnostics)
{
  nav2_behavior_tree::BtTickTiming tick_timing;
  config_->blackboard->set<nav2_behavior_tree::BtTickTiming *>(
    "bt_tick_timing", &tick_timing);
  bt_node_ = std::make_shared<nav2_behavior_tree::AdaptiveReplanning>(
    "adaptive_replanning_with_timing", *config_);
  bt_node_->setChild(dummy_node_.get());

  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = 2.0;
  config_->blackboard->set("goal", goal_pose);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.resize(2);
  config_->blackboard->set("path", path);

  // 首次规划用于验证child分段已接通。
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(tick_timing.adaptive.valid);
  EXPECT_GE(tick_timing.adaptive.total_ms, tick_timing.adaptive.child_ms);

  bt_node_->halt();
  tick_timing.reset();

  // 第二轮会读取并扫描已有路径，用路径点数验证path_copy/path_scan分段已执行。
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(tick_timing.adaptive.valid);
  EXPECT_EQ(tick_timing.adaptive.path_poses, 2u);
  const double known_ms =
    tick_timing.adaptive.ports_ms + tick_timing.adaptive.goal_update_ms +
    tick_timing.adaptive.tf_ms + tick_timing.adaptive.path_copy_ms +
    tick_timing.adaptive.path_scan_ms + tick_timing.adaptive.decision_ms +
    tick_timing.adaptive.child_ms;
  EXPECT_NEAR(
    tick_timing.adaptive.total_ms,
    known_ms + tick_timing.adaptive.other_ms, 1e-6);
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
