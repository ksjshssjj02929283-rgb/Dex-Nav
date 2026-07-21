// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "nav2_controller/controller_server.hpp"
#include "nav2_core/local_planner.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

class FakeLocalPlanner : public nav2_core::LocalPlanner
{
public:
  explicit FakeLocalPlanner(bool success, bool return_empty = false)
  : success_(success), return_empty_(return_empty) {}

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    std::string, std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}
  void setPlan(const nav_msgs::msg::Path & path) override {plan_ = path;}

  bool computeLocalPlan(
    const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::Twist &,
    nav_msgs::msg::Path & local_path) override
  {
    if (!return_empty_) {
      local_path = plan_;
    }
    return success_;
  }

private:
  bool success_;
  bool return_empty_;
  nav_msgs::msg::Path plan_;
};

class ControllerShim : public nav2_controller::ControllerServer
{
public:
  ControllerShim()
  : nav2_controller::ControllerServer(rclcpp::NodeOptions()) {}

  void disableLocalPlanner()
  {
    current_local_planner_.clear();
    local_planners_.clear();
  }

  void setLocalPlanner(const nav2_core::LocalPlanner::Ptr & planner)
  {
    current_local_planner_ = "test_local_planner";
    local_planners_.clear();
    local_planners_.insert({current_local_planner_, planner});
  }

  bool runLocalStage(const nav_msgs::msg::Path & input, nav_msgs::msg::Path & output)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = input.header.frame_id;
    geometry_msgs::msg::Twist velocity;
    return computeLocalPath(input, pose, velocity, output);
  }
};

struct RclcppFixture
{
  RclcppFixture() {rclcpp::init(0, nullptr);}
  ~RclcppFixture() {rclcpp::shutdown();}
};

RclcppFixture g_rclcpp_fixture;

nav_msgs::msg::Path makePath()
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (double x : {0.0, 1.0}) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = x;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

TEST(LocalPlanningStage, DisabledPlannerPreservesGlobalPathPassthrough)
{
  auto controller = std::make_shared<ControllerShim>();
  controller->disableLocalPlanner();
  const auto input = makePath();
  nav_msgs::msg::Path output;

  EXPECT_TRUE(controller->runLocalStage(input, output));
  EXPECT_EQ(output, input);
}

TEST(LocalPlanningStage, ConfiguredFailureIsFailClosed)
{
  auto controller = std::make_shared<ControllerShim>();
  auto planner = std::make_shared<FakeLocalPlanner>(false);
  planner->setPlan(makePath());
  controller->setLocalPlanner(planner);
  nav_msgs::msg::Path output;

  EXPECT_FALSE(controller->runLocalStage(makePath(), output));
  EXPECT_TRUE(output.poses.empty());
}

TEST(LocalPlanningStage, ConfiguredEmptyOutputIsFailClosed)
{
  auto controller = std::make_shared<ControllerShim>();
  auto planner = std::make_shared<FakeLocalPlanner>(true, true);
  planner->setPlan(makePath());
  controller->setLocalPlanner(planner);
  nav_msgs::msg::Path output;

  EXPECT_FALSE(controller->runLocalStage(makePath(), output));
  EXPECT_TRUE(output.poses.empty());
}

}  // namespace
