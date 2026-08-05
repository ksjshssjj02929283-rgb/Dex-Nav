// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <limits>
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

class FakeGoalChecker : public nav2_core::GoalChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    const std::string &,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}

  void reset() override {++reset_count_;}

  bool isGoalReached(
    const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::Twist &) override
  {
    return false;
  }

  bool getTolerances(
    geometry_msgs::msg::Pose &,
    geometry_msgs::msg::Twist &) override
  {
    return false;
  }

  size_t resetCount() const {return reset_count_;}

private:
  size_t reset_count_{0};
};

class ControllerShim : public nav2_controller::ControllerServer
{
public:
  ControllerShim()
  : nav2_controller::ControllerServer(rclcpp::NodeOptions()),
    fake_goal_checker_(std::make_shared<FakeGoalChecker>())
  {
    current_goal_checker_ = "test_goal_checker";
    goal_checkers_.insert({current_goal_checker_, fake_goal_checker_});
  }

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
    const nav_interfaces::msg::RobotTrajectory & trajectory, double age,
    double consumed_target_velocity = 0.35,
    double consumed_target_acceleration = -0.2)
  {
    speed_plan_state_.accept(
      trajectory, consumed_target_velocity, consumed_target_acceleration,
      now() - rclcpp::Duration::from_seconds(age));
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

  void updateContinuity(
    ControllerPhase phase, bool succeeded, bool consumed,
    double consumed_target_velocity = 0.35,
    double consumed_target_acceleration = -0.2)
  {
    controller_phase_ = phase;
    updateSpeedPlanningContinuityAfterControl(
      succeeded, consumed, consumed_target_velocity, consumed_target_acceleration);
  }

  void resetStages() {resetStagedControlState();}
  bool continuityValid() const {return speed_plan_state_.valid();}

  void setTerminalStateForTest()
  {
    controller_phase_ = ControllerPhase::APPROACH;
    near_goal_progress_acceptance_reached_ = true;
    near_goal_pose_hold_active_ = true;
    near_goal_hold_outside_accept_count_ = 2;
  }

  void latchTerminalBrakeForTest()
  {
    nav2_core::SpeedPlanningDiagnostics diagnostics;
    diagnostics.terminal_brake_required = true;
    diagnostics.terminal_distance_available = 0.5;
    diagnostics.terminal_braking_distance = 0.4;
    diagnostics.terminal_execution_reserve = 0.1;
    updateTerminalBrakeLatch(diagnostics);
  }

  void commitPathForTest(
    const nav_msgs::msg::Path & path,
    PathUpdateType update_type)
  {
    commitPlannerPathState(path, update_type);
  }

  ControllerPhase phase() const {return controller_phase_;}
  bool nearGoalAcceptanceReached() const {return near_goal_progress_acceptance_reached_;}
  bool nearGoalPoseHoldActive() const {return near_goal_pose_hold_active_;}
  size_t nearGoalOutsideCount() const {return near_goal_hold_outside_accept_count_;}
  size_t goalCheckerResetCount() const {return fake_goal_checker_->resetCount();}
  bool terminalBrakeLatched() const {return terminal_brake_latched_;}

private:
  std::shared_ptr<FakeGoalChecker> fake_goal_checker_;
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

TEST(SpeedPlanningBoundary, InheritsConsumedTrajectoryTargetState)
{
  auto controller = std::make_shared<ControllerShim>();
  controller->seedAcceptedTrajectory(makeTrajectory(), 0.1);

  const auto boundary = controller->boundaryFor(
    makePath({{0.5, 0.0}, {1.0, 0.0}, {2.0, 0.0}}), 0.31);

  ASSERT_TRUE(boundary.acceleration_valid);
  EXPECT_NEAR(boundary.acceleration, -0.2, 1e-9);
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
    nav2_controller::ControllerServer::ControllerPhase::TRACK, true, true, 0.31, -0.18);
  EXPECT_TRUE(controller->continuityValid());
  EXPECT_NEAR(
    controller->boundaryFor(
      makePath({{0.5, 0.0}, {1.0, 0.0}, {2.0, 0.0}}), 0.31).acceleration,
    -0.18, 1e-9);

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

  controller->setCurrentTrajectory(trajectory);
  controller->updateContinuity(
    nav2_controller::ControllerServer::ControllerPhase::TRACK, true, true,
    0.31, std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(controller->continuityValid());
}

TEST(SpeedPlanningBoundary, SameGoalGeometryUpdatePreservesTerminalState)
{
  auto controller = std::make_shared<ControllerShim>();
  controller->seedAcceptedTrajectory(makeTrajectory(), 0.1, 0.31, -0.18);
  controller->setTerminalStateForTest();
  controller->latchTerminalBrakeForTest();

  const auto updated_path = makePath({{0.5, 0.0}, {1.1, 0.05}, {2.0, 0.0}});
  controller->commitPathForTest(
    updated_path,
    nav2_controller::ControllerServer::PathUpdateType::SAME_GOAL_GEOMETRY);

  EXPECT_EQ(
    controller->phase(),
    nav2_controller::ControllerServer::ControllerPhase::APPROACH);
  EXPECT_TRUE(controller->continuityValid());
  EXPECT_TRUE(controller->nearGoalAcceptanceReached());
  EXPECT_TRUE(controller->nearGoalPoseHoldActive());
  EXPECT_EQ(controller->nearGoalOutsideCount(), 2u);
  EXPECT_TRUE(controller->terminalBrakeLatched());
  EXPECT_NEAR(
    controller->boundaryFor(updated_path, 0.31).acceleration, -0.18, 1e-9);
  EXPECT_TRUE(controller->boundaryFor(updated_path, 0.31).terminal_brake_latched);
}

TEST(SpeedPlanningBoundary, NewGoalPathResetsTerminalBookkeeping)
{
  auto controller = std::make_shared<ControllerShim>();
  controller->setTerminalStateForTest();
  controller->latchTerminalBrakeForTest();

  const auto new_goal_path = makePath({{0.5, 0.0}, {1.5, 0.0}, {2.5, 0.0}});
  controller->commitPathForTest(
    new_goal_path,
    nav2_controller::ControllerServer::PathUpdateType::NEW_GOAL);

  EXPECT_EQ(
    controller->phase(),
    nav2_controller::ControllerServer::ControllerPhase::TRACK);
  EXPECT_FALSE(controller->nearGoalAcceptanceReached());
  EXPECT_FALSE(controller->nearGoalPoseHoldActive());
  EXPECT_EQ(controller->nearGoalOutsideCount(), 0u);
  EXPECT_EQ(controller->goalCheckerResetCount(), 1u);
  EXPECT_FALSE(controller->terminalBrakeLatched());
}

}  // namespace
