// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

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

  void seedAcceptedTrajectory(
    const nav_interfaces::msg::RobotTrajectory & trajectory, double age)
  {
    speed_plan_state_.accept(trajectory, now() - rclcpp::Duration::from_seconds(age));
  }

  nav2_core::SpeedPlanningBoundary boundaryFor(
    const nav_msgs::msg::Path & path, double measured_velocity)
  {
    geometry_msgs::msg::Twist velocity;
    velocity.linear.x = measured_velocity;
    return buildSpeedPlanningBoundary(path, velocity);
  }

  void setCurrentTrajectory(const nav_interfaces::msg::RobotTrajectory & trajectory)
  {
    current_trajectory_ = trajectory;
  }

  void updateContinuity(ControllerPhase phase, bool succeeded, bool consumed)
  {
    controller_phase_ = phase;
    updateSpeedPlanningContinuityAfterControl(succeeded, consumed);
  }

  void resetStages() {resetStagedControlState();}
  bool continuityValid() const {return speed_plan_state_.valid();}
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

nav_msgs::msg::Path makePath(const std::vector<std::pair<double, double>> & points)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (const auto & [x, y] : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = path.header.frame_id;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

nav_interfaces::msg::RobotTrajectory makeTrajectory()
{
  nav_interfaces::msg::RobotTrajectory trajectory;
  trajectory.header.frame_id = "map";
  for (size_t i = 0; i < 3; ++i) {
    nav_interfaces::msg::TrajectoryPoint point;
    point.pose.header.frame_id = trajectory.header.frame_id;
    point.pose.pose.position.x = static_cast<double>(i);
    point.pose.pose.orientation.w = 1.0;
    point.longitudinal_velocity = 0.2 + 0.2 * static_cast<double>(i);
    point.acceleration = 0.1 + 0.1 * static_cast<double>(i);
    trajectory.points.push_back(point);
  }
  return trajectory;
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

TEST(SpeedPlanningBoundary, InterpolatesAcceptedTrajectoryState)
{
  auto controller = std::make_shared<ControllerShim>();
  controller->seedAcceptedTrajectory(makeTrajectory(), 0.1);

  const auto boundary = controller->boundaryFor(
    makePath({{0.5, 0.0}, {1.0, 0.0}, {2.0, 0.0}}), 0.31);

  ASSERT_TRUE(boundary.acceleration_valid);
  EXPECT_NEAR(boundary.acceleration, 0.15, 1e-9);
  EXPECT_NEAR(boundary.source_age, 0.1, 0.03);
  EXPECT_NEAR(boundary.velocity.linear.x, 0.31, 1e-9);
}

TEST(SpeedPlanningBoundary, RejectsIncompatibleContinuity)
{
  auto controller = std::make_shared<ControllerShim>();
  const auto normal_path = makePath({{0.5, 0.0}, {1.0, 0.0}, {2.0, 0.0}});

  controller->seedAcceptedTrajectory(makeTrajectory(), 0.6);
  EXPECT_FALSE(controller->boundaryFor(normal_path, 0.3).acceleration_valid);

  controller->seedAcceptedTrajectory(makeTrajectory(), 0.1);
  auto wrong_frame = normal_path;
  wrong_frame.header.frame_id = "odom";
  for (auto & pose : wrong_frame.poses) {
    pose.header.frame_id = "odom";
  }
  EXPECT_FALSE(controller->boundaryFor(wrong_frame, 0.3).acceleration_valid);

  auto wrong_goal = normal_path;
  wrong_goal.poses.back().pose.position.x = 2.2;
  EXPECT_FALSE(controller->boundaryFor(wrong_goal, 0.3).acceleration_valid);

  const auto wrong_branch = makePath({{0.5, 0.0}, {0.5, 0.5}, {2.0, 0.0}});
  EXPECT_FALSE(controller->boundaryFor(wrong_branch, 0.3).acceleration_valid);

  const auto spatial_mismatch = makePath({{0.5, 0.3}, {1.0, 0.3}, {2.0, 0.0}});
  EXPECT_FALSE(controller->boundaryFor(spatial_mismatch, 0.3).acceleration_valid);

  EXPECT_FALSE(controller->boundaryFor(normal_path, 0.7).acceleration_valid);
}

TEST(SpeedPlanningBoundary, AcceptsOnlyConsumedTrackingTrajectoryAndResets)
{
  auto controller = std::make_shared<ControllerShim>();
  const auto trajectory = makeTrajectory();

  controller->setCurrentTrajectory(trajectory);
  controller->updateContinuity(
    nav2_controller::ControllerServer::ControllerPhase::TRACK, true, true);
  EXPECT_TRUE(controller->continuityValid());

  controller->resetStages();
  EXPECT_FALSE(controller->continuityValid());

  controller->setCurrentTrajectory(trajectory);
  controller->updateContinuity(
    nav2_controller::ControllerServer::ControllerPhase::PREPARE, true, true);
  EXPECT_FALSE(controller->continuityValid());

  controller->setCurrentTrajectory(trajectory);
  controller->updateContinuity(
    nav2_controller::ControllerServer::ControllerPhase::APPROACH, true, false);
  EXPECT_FALSE(controller->continuityValid());
}

}  // namespace
