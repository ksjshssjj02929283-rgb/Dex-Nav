// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Sarthak Mittal
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
#include <memory>
#include <set>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"

#include "../../test_service.hpp"
#include "nav2_behavior_tree/plugins/action/clear_costmap_service.hpp"

class ClearEntireCostmapService : public TestService<nav2_msgs::srv::ClearEntireCostmap>
{
public:
  ClearEntireCostmapService()
  : TestService("clear_entire_costmap")
  {}

  void setDelay(std::chrono::milliseconds delay) {delay_ = delay.count();}
  void resetCompletedCount() {completed_count_ = 0;}
  int completedCount() const {return completed_count_;}

protected:
  void handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Request> request,
    const std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Response> response) override
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_.load()));
    TestService<nav2_msgs::srv::ClearEntireCostmap>::handle_service(
      request_header, request, response);
    ++completed_count_;
  }

private:
  std::atomic<int64_t> delay_{0};
  std::atomic<int> completed_count_{0};
};

class ClearEntireCostmapServiceTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("clear_entire_costmap_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set<rclcpp::Node::SharedPtr>(
      "node",
      node_);
    config_->blackboard->set<std::chrono::milliseconds>(
      "server_timeout",
      std::chrono::milliseconds(20));
    config_->blackboard->set<std::chrono::milliseconds>(
      "bt_loop_duration",
      std::chrono::milliseconds(10));
    config_->blackboard->set<bool>("initial_pose_received", false);
    config_->blackboard->set<int>("number_recoveries", 0);

    factory_->registerNodeType<nav2_behavior_tree::ClearEntireCostmapService>("ClearEntireCostmap");
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    server_.reset();
    factory_.reset();
  }

  void SetUp() override
  {
    config_->blackboard->set("number_recoveries", 0);
    server_->setDelay(std::chrono::milliseconds(0));
    server_->resetCompletedCount();
  }

  void TearDown() override
  {
    tree_.reset();
  }

  static std::shared_ptr<ClearEntireCostmapService> server_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr ClearEntireCostmapServiceTestFixture::node_ = nullptr;
std::shared_ptr<ClearEntireCostmapService> ClearEntireCostmapServiceTestFixture::server_ = nullptr;
BT::NodeConfiguration * ClearEntireCostmapServiceTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> ClearEntireCostmapServiceTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> ClearEntireCostmapServiceTestFixture::tree_ = nullptr;

TEST_F(ClearEntireCostmapServiceTestFixture, test_tick)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <ClearEntireCostmap service_name="clear_entire_costmap"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(config_->blackboard->get<int>("number_recoveries"), 0);
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(config_->blackboard->get<int>("number_recoveries"), 1);
}

TEST_F(ClearEntireCostmapServiceTestFixture, test_timeout_drains_before_failure)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <ClearEntireCostmap service_name="clear_entire_costmap"/>
        </BehaviorTree>
      </root>)";

  config_->blackboard->set<std::chrono::milliseconds>(
    "server_timeout", std::chrono::milliseconds(20));
  config_->blackboard->set<std::chrono::milliseconds>(
    "bt_loop_duration", std::chrono::milliseconds(10));
  server_->setDelay(std::chrono::milliseconds(80));
  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  BT::NodeStatus result = BT::NodeStatus::RUNNING;
  int ticks = 0;
  rclcpp::WallRate loop_rate(std::chrono::milliseconds(5));
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING && ticks < 100) {
    result = tree_->tickRoot();
    ++ticks;
    if (result != BT::NodeStatus::RUNNING) {
      EXPECT_EQ(server_->completedCount(), 1);
    }
    loop_rate.sleep();
  }

  EXPECT_EQ(result, BT::NodeStatus::FAILURE);
  EXPECT_EQ(server_->completedCount(), 1);
  // Recovery accounting happens when the single request is sent. Draining the
  // timed-out response must not send a duplicate request or increment it again.
  EXPECT_EQ(config_->blackboard->get<int>("number_recoveries"), 1);
  EXPECT_GT(ticks, 2);
}

class ClearCostmapExceptRegionService : public TestService<nav2_msgs::srv::ClearCostmapExceptRegion>
{
public:
  ClearCostmapExceptRegionService()
  : TestService("clear_costmap_except_region")
  {}
};

class ClearCostmapExceptRegionServiceTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("clear_costmap_except_region_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set<rclcpp::Node::SharedPtr>(
      "node",
      node_);
    config_->blackboard->set<std::chrono::milliseconds>(
      "server_timeout",
      std::chrono::milliseconds(10));
    config_->blackboard->set<std::chrono::milliseconds>(
      "bt_loop_duration",
      std::chrono::milliseconds(10));
    config_->blackboard->set<bool>("initial_pose_received", false);
    config_->blackboard->set<int>("number_recoveries", 0);

    factory_->registerNodeType<nav2_behavior_tree::ClearCostmapExceptRegionService>(
      "ClearCostmapExceptRegion");
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    server_.reset();
    factory_.reset();
  }

  void SetUp() override
  {
    config_->blackboard->set("number_recoveries", 0);
  }

  void TearDown() override
  {
    tree_.reset();
  }

  static std::shared_ptr<ClearCostmapExceptRegionService> server_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr
ClearCostmapExceptRegionServiceTestFixture::node_ = nullptr;
std::shared_ptr<ClearCostmapExceptRegionService>
ClearCostmapExceptRegionServiceTestFixture::server_ = nullptr;
BT::NodeConfiguration
* ClearCostmapExceptRegionServiceTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory>
ClearCostmapExceptRegionServiceTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree>
ClearCostmapExceptRegionServiceTestFixture::tree_ = nullptr;

TEST_F(ClearCostmapExceptRegionServiceTestFixture, test_tick)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <ClearCostmapExceptRegion service_name="clear_costmap_except_region"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(config_->blackboard->get<int>("number_recoveries"), 0);
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(config_->blackboard->get<int>("number_recoveries"), 1);
}
//******************************************
class ClearCostmapAroundRobotService : public TestService<nav2_msgs::srv::ClearCostmapAroundRobot>
{
public:
  ClearCostmapAroundRobotService()
  : TestService("clear_costmap_around_robot")
  {}
};

class ClearCostmapAroundRobotServiceTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("clear_costmap_around_robot_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set<rclcpp::Node::SharedPtr>(
      "node",
      node_);
    config_->blackboard->set<std::chrono::milliseconds>(
      "server_timeout",
      std::chrono::milliseconds(10));
    config_->blackboard->set<std::chrono::milliseconds>(
      "bt_loop_duration",
      std::chrono::milliseconds(10));
    config_->blackboard->set<bool>("initial_pose_received", false);
    config_->blackboard->set<int>("number_recoveries", 0);

    factory_->registerNodeType<nav2_behavior_tree::ClearCostmapAroundRobotService>(
      "ClearCostmapAroundRobot");
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    server_.reset();
    factory_.reset();
  }

  void SetUp() override
  {
    config_->blackboard->set("number_recoveries", 0);
  }

  void TearDown() override
  {
    tree_.reset();
  }

  static std::shared_ptr<ClearCostmapAroundRobotService> server_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr
ClearCostmapAroundRobotServiceTestFixture::node_ = nullptr;
std::shared_ptr<ClearCostmapAroundRobotService>
ClearCostmapAroundRobotServiceTestFixture::server_ = nullptr;
BT::NodeConfiguration
* ClearCostmapAroundRobotServiceTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory>
ClearCostmapAroundRobotServiceTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree>
ClearCostmapAroundRobotServiceTestFixture::tree_ = nullptr;

TEST_F(ClearCostmapAroundRobotServiceTestFixture, test_tick)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <ClearCostmapAroundRobot service_name="clear_costmap_around_robot"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(config_->blackboard->get<int>("number_recoveries"), 0);
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(config_->blackboard->get<int>("number_recoveries"), 1);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  // initialize service and spin on new thread
  ClearEntireCostmapServiceTestFixture::server_ = std::make_shared<ClearEntireCostmapService>();
  std::thread server_thread([]() {
      rclcpp::spin(ClearEntireCostmapServiceTestFixture::server_);
    });

  ClearCostmapExceptRegionServiceTestFixture::server_ =
    std::make_shared<ClearCostmapExceptRegionService>();
  std::thread server_thread_except_region([]() {
      rclcpp::spin(ClearCostmapExceptRegionServiceTestFixture::server_);
    });

  ClearCostmapAroundRobotServiceTestFixture::server_ =
    std::make_shared<ClearCostmapAroundRobotService>();
  std::thread server_thread_around_robot([]() {
      rclcpp::spin(ClearCostmapAroundRobotServiceTestFixture::server_);
    });

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();
  server_thread.join();
  server_thread_except_region.join();
  server_thread_around_robot.join();

  return all_successful;
}
