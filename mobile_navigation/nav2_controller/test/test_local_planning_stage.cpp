// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
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

constexpr auto kNoTerminalBranch = nav2_core::SpeedPlanningTerminalBranch::NONE;
constexpr auto kObstacleTerminalBranch =
  nav2_core::SpeedPlanningTerminalBranch::PATH_OBSTACLE_STOP;

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

class FakeController : public nav2_core::Controller
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    std::string, std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}
  void setPlan(const nav_msgs::msg::Path &) override {}
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::Twist &,
    nav2_core::GoalChecker *) override
  {
    return geometry_msgs::msg::TwistStamped();
  }
  void setSpeedLimit(const double &, const bool &) override {}
};

class FakeAvoidingController
  : public FakeController,
  public nav2_core::PathObstacleAvoidanceCapability
{
public:
  bool canAvoidPathObstacles() const override {return true;}
};

class FakeEndLocalizationController
  : public FakeController,
  public nav2_core::EndLocalizationCapability
{
public:
  void resetEndLocalization(uint64_t goal_generation) override
  {
    snapshot_ = nav2_core::EndLocalizationSnapshot();
    snapshot_.goal_generation = goal_generation;
    ++reset_count_;
  }

  void setEndLocalizationGoal(
    const geometry_msgs::msg::PoseStamped & live_goal,
    uint64_t goal_generation) override
  {
    snapshot_.goal_generation = goal_generation;
    snapshot_.live_goal = live_goal;
    snapshot_.live_goal_valid = true;
  }

  nav2_core::EndLocalizationSnapshot getEndLocalizationSnapshot() const override
  {
    return snapshot_;
  }

  void seedPositionLatch(uint64_t goal_generation)
  {
    snapshot_.goal_generation = goal_generation;
    snapshot_.enabled = true;
    snapshot_.effective_goal_valid = true;
    snapshot_.position_goal_latched = true;
    snapshot_.stage = nav2_core::EndLocalizationStage::ODOM_POSITION;
  }

  void seedFinalYawStage(
    uint64_t goal_generation,
    nav2_core::EndLocalizationStage stage)
  {
    snapshot_.goal_generation = goal_generation;
    snapshot_.enabled = true;
    snapshot_.effective_goal_valid = true;
    snapshot_.position_goal_latched = false;
    snapshot_.stage = stage;
  }

  void setEffectiveGoalValid(bool valid)
  {
    snapshot_.effective_goal_valid = valid;
  }

  void seedEffectiveGoal(
    uint64_t goal_generation,
    const geometry_msgs::msg::PoseStamped & effective_goal)
  {
    seedPositionLatch(goal_generation);
    snapshot_.effective_goal = effective_goal;
  }

  size_t resetCount() const {return reset_count_;}

private:
  nav2_core::EndLocalizationSnapshot snapshot_;
  size_t reset_count_{0};
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

class FakeProgressChecker : public nav2_core::ProgressChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    const std::string &) override {}

  bool check(geometry_msgs::msg::PoseStamped &) override
  {
    ++check_count_;
    return check_result_;
  }

  void reset() override {++reset_count_;}
  void setCheckResult(bool result) {check_result_ = result;}
  size_t checkCount() const {return check_count_;}
  size_t resetCount() const {return reset_count_;}

private:
  bool check_result_{true};
  size_t check_count_{0};
  size_t reset_count_{0};
};

class ControllerShim : public nav2_controller::ControllerServer
{
public:
  ControllerShim()
  : nav2_controller::ControllerServer(rclcpp::NodeOptions()),
    fake_goal_checker_(std::make_shared<FakeGoalChecker>()),
    fake_progress_checker_(std::make_shared<FakeProgressChecker>())
  {
    current_goal_checker_ = "test_goal_checker";
    goal_checkers_.insert({current_goal_checker_, fake_goal_checker_});
    progress_checker_ = fake_progress_checker_;
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
    initial_motion_arbiter_.accept(
      trajectory, consumed_target_velocity, consumed_target_acceleration,
      now() - rclcpp::Duration::from_seconds(age));
  }

  nav2_core::InitialMotionState boundaryFor(
    const nav_msgs::msg::Path & path, double measured_velocity)
  {
    geometry_msgs::msg::Twist velocity;
    velocity.linear.x = measured_velocity;
    return buildInitialMotionState(path, velocity);
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
  bool continuityValid() const {return initial_motion_arbiter_.valid();}

  void setTerminalStateForTest()
  {
    controller_phase_ = ControllerPhase::APPROACH;
  }

  void latchTerminalBrakeForTest()
  {
    nav2_core::SpeedPlanningDiagnostics diagnostics;
    diagnostics.terminal_brake_required = true;
    diagnostics.terminal_distance_available = 0.5;
    diagnostics.terminal_braking_distance = 0.4;
    diagnostics.terminal_execution_reserve = 0.1;
    updateTerminalBrakeLatch(diagnostics, true);
  }

  void updateTerminalBrakeForTest(
    const nav2_core::SpeedPlanningDiagnostics & diagnostics,
    bool longitudinal_continuity_valid)
  {
    updateTerminalBrakeLatch(diagnostics, longitudinal_continuity_valid);
  }

  void commitPathForTest(
    const nav_msgs::msg::Path & path,
    PathUpdateType update_type)
  {
    commitPlannerPathState(path, update_type);
  }

  void setProgressPlanningContext(
    const nav_interfaces::msg::RobotTrajectory & trajectory,
    nav2_core::SpeedPlanningTerminalBranch terminal_branch,
    bool terminal_infeasible = false)
  {
    current_trajectory_ = trajectory;
    speed_plan_cycle_ = SpeedPlanCycleContext();
    speed_plan_cycle_.active = true;
    speed_plan_cycle_.planning_succeeded = true;
    speed_plan_cycle_.terminal_valid = true;
    speed_plan_cycle_.terminal.terminal_branch = terminal_branch;
    speed_plan_cycle_.terminal.terminal_infeasible = terminal_infeasible;
    speed_plan_cycle_.terminal.first_obstacle_arc = 1.0;
  }

  bool checkProgressForTest(geometry_msgs::msg::PoseStamped pose)
  {
    return checkProgress(pose);
  }

  void setProgressCheckResult(bool result)
  {
    fake_progress_checker_->setCheckResult(result);
  }

  // 设置碰撞等待状态，供状态转换测试使用。
  void setCollisionStopActiveForTest(bool active)
  {
    is_collision_stop_active_ = active;
  }

  void setControllerForObstaclePolicy(const nav2_core::Controller::Ptr & controller)
  {
    staged_control_enabled_ = false;
    current_controller_ = "test_controller";
    controllers_.clear();
    controllers_.insert({current_controller_, controller});
  }

  void setEndLocalizationControllerForTest(
    const std::shared_ptr<FakeEndLocalizationController> & controller)
  {
    controllers_.clear();
    controllers_.insert({"PIDController", controller});
    approach_controller_ = "PIDController";
  }

  bool shouldUseApproachForTest(
    double distance_to_goal,
    bool & end_localization_active)
  {
    return shouldUseApproachController(distance_to_goal, end_localization_active);
  }

  bool applyEndLocalizationGoalForTest(geometry_msgs::msg::PoseStamped & goal_pose)
  {
    return applyEndLocalizationGoalToGoalChecker(approach_controller_, goal_pose);
  }

  bool activeControllerCanAvoidForTest()
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.orientation.w = 1.0;
    return activeControllerCanAvoidPathObstacles(pose);
  }

  ControllerPhase phase() const {return controller_phase_;}
  size_t goalCheckerResetCount() const {return fake_goal_checker_->resetCount();}
  bool terminalBrakeLatched() const {return terminal_brake_latched_;}
  // 读取碰撞等待状态，供测试断言使用。
  bool isCollisionStopActive() const {return is_collision_stop_active_;}
  bool isProgressCheckPaused() const
  {
    return is_progress_check_paused_for_obstacle_stop_;
  }
  size_t progressCheckCount() const {return fake_progress_checker_->checkCount();}
  size_t progressResetCount() const {return fake_progress_checker_->resetCount();}
  uint64_t goalGeneration() const {return goal_generation_;}

private:
  std::shared_ptr<FakeGoalChecker> fake_goal_checker_;
  std::shared_ptr<FakeProgressChecker> fake_progress_checker_;
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

nav_interfaces::msg::RobotTrajectory makeStoppedTrajectory()
{
  auto trajectory = makeTrajectory();
  for (auto & point : trajectory.points) {
    point.longitudinal_velocity = 0.0;
    point.acceleration = 0.0;
  }
  return trajectory;
}

geometry_msgs::msg::PoseStamped makePose(double x = 0.0, double y = 0.0)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.w = 1.0;
  return pose;
}

TEST(ControllerObstaclePolicy, UnknownCapabilityDefaultsToStop)
{
  auto controller = std::make_shared<ControllerShim>();
  controller->setControllerForObstaclePolicy(std::make_shared<FakeController>());

  EXPECT_FALSE(controller->activeControllerCanAvoidForTest());
}

TEST(ControllerObstaclePolicy, ExplicitAvoidanceCapabilityDisablesPathStop)
{
  auto controller = std::make_shared<ControllerShim>();
  controller->setControllerForObstaclePolicy(std::make_shared<FakeAvoidingController>());

  EXPECT_TRUE(controller->activeControllerCanAvoidForTest());
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

TEST(InitialMotionState, InheritsConsumedTrajectoryTargetState)
{
  auto controller = std::make_shared<ControllerShim>();
  controller->seedAcceptedTrajectory(makeTrajectory(), 0.1);

  const auto initial_motion = controller->boundaryFor(
    makePath({{0.5, 0.0}, {1.0, 0.0}, {2.0, 0.0}}), 0.31);

  ASSERT_TRUE(initial_motion.acceleration_valid);
  EXPECT_NEAR(initial_motion.acceleration, -0.2, 1e-9);
  EXPECT_NEAR(initial_motion.source_age, 0.1, 0.03);
  EXPECT_NEAR(initial_motion.velocity.linear.x, 0.31, 1e-9);
}

TEST(InitialMotionState, RejectsIncompatibleContinuity)
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

TEST(InitialMotionState, AcceptsOnlyConsumedTrackingTrajectoryAndResets)
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

TEST(InitialMotionState, SameGoalGeometryUpdatePreservesTerminalState)
{
  auto controller = std::make_shared<ControllerShim>();
  controller->seedAcceptedTrajectory(makeTrajectory(), 0.1, 0.31, -0.18);
  controller->setTerminalStateForTest();
  controller->latchTerminalBrakeForTest();
  controller->setCollisionStopActiveForTest(true);

  const auto updated_path = makePath({{0.5, 0.0}, {1.1, 0.05}, {2.0, 0.0}});
  controller->commitPathForTest(
    updated_path,
    nav2_controller::ControllerServer::PathUpdateType::SAME_GOAL_GEOMETRY);

  EXPECT_EQ(
    controller->phase(),
    nav2_controller::ControllerServer::ControllerPhase::APPROACH);
  EXPECT_TRUE(controller->continuityValid());
  EXPECT_TRUE(controller->terminalBrakeLatched());
  EXPECT_TRUE(controller->isCollisionStopActive());
  EXPECT_NEAR(
    controller->boundaryFor(updated_path, 0.31).acceleration, -0.18, 1e-9);
  EXPECT_TRUE(controller->boundaryFor(updated_path, 0.31).terminal_brake_latched);
}

TEST(InitialMotionState, NewGoalPathResetsTerminalBookkeeping)
{
  auto controller = std::make_shared<ControllerShim>();
  controller->setTerminalStateForTest();
  controller->latchTerminalBrakeForTest();
  controller->setCollisionStopActiveForTest(true);

  const auto new_goal_path = makePath({{0.5, 0.0}, {1.5, 0.0}, {2.5, 0.0}});
  controller->commitPathForTest(
    new_goal_path,
    nav2_controller::ControllerServer::PathUpdateType::NEW_GOAL);

  EXPECT_EQ(
    controller->phase(),
    nav2_controller::ControllerServer::ControllerPhase::TRACK);
  EXPECT_EQ(controller->goalCheckerResetCount(), 1u);
  EXPECT_FALSE(controller->terminalBrakeLatched());
  EXPECT_FALSE(controller->isCollisionStopActive());
}

TEST(EndLocalizationBridge, SameGoalPreservesLatchAndLifecycleResetClearsIt)
{
  auto server = std::make_shared<ControllerShim>();
  auto pid = std::make_shared<FakeEndLocalizationController>();
  server->setEndLocalizationControllerForTest(pid);
  pid->seedPositionLatch(server->goalGeneration());

  const auto same_goal_path = makePath({{0.5, 0.0}, {1.0, 0.0}, {2.0, 0.0}});
  server->commitPathForTest(
    same_goal_path,
    nav2_controller::ControllerServer::PathUpdateType::SAME_GOAL_GEOMETRY);
  EXPECT_EQ(pid->resetCount(), 0u);
  EXPECT_TRUE(pid->getEndLocalizationSnapshot().position_goal_latched);

  const auto new_goal_path = makePath({{0.5, 0.0}, {1.5, 0.0}, {2.5, 0.0}});
  server->commitPathForTest(
    new_goal_path,
    nav2_controller::ControllerServer::PathUpdateType::NEW_GOAL);
  EXPECT_EQ(pid->resetCount(), 1u);
  EXPECT_EQ(pid->getEndLocalizationSnapshot().goal_generation, server->goalGeneration());
  EXPECT_FALSE(pid->getEndLocalizationSnapshot().position_goal_latched);

  pid->seedPositionLatch(server->goalGeneration());
  server->resetStages();
  EXPECT_EQ(pid->resetCount(), 2u);
  EXPECT_FALSE(pid->getEndLocalizationSnapshot().position_goal_latched);
}

TEST(EndLocalizationBridge, TemporaryInvalidGoalKeepsApproachAfterPositionLatch)
{
  auto server = std::make_shared<ControllerShim>();
  auto pid = std::make_shared<FakeEndLocalizationController>();
  server->setEndLocalizationControllerForTest(pid);
  pid->seedPositionLatch(server->goalGeneration());
  pid->setEffectiveGoalValid(false);

  bool end_localization_active = false;
  const bool use_approach = server->shouldUseApproachForTest(
    std::numeric_limits<double>::infinity(), end_localization_active);

  EXPECT_TRUE(end_localization_active);
  EXPECT_TRUE(use_approach);
}

TEST(EndLocalizationBridge, FinalYawStageKeepsApproachAfterPositionLatchReleased)
{
  auto server = std::make_shared<ControllerShim>();
  auto pid = std::make_shared<FakeEndLocalizationController>();
  server->setEndLocalizationControllerForTest(pid);

  for (const auto stage : {
      nav2_core::EndLocalizationStage::MAP_YAW_COARSE,
      nav2_core::EndLocalizationStage::ODOM_YAW_FINE})
  {
    pid->seedFinalYawStage(server->goalGeneration(), stage);
    pid->setEffectiveGoalValid(false);
    bool end_localization_active = false;
    const bool use_approach = server->shouldUseApproachForTest(
      std::numeric_limits<double>::infinity(), end_localization_active);

    EXPECT_TRUE(end_localization_active);
    EXPECT_TRUE(use_approach);
    EXPECT_FALSE(pid->getEndLocalizationSnapshot().position_goal_latched);
  }
}

TEST(EndLocalizationBridge, GoalCheckerUsesSameEffectiveLatchedGoal)
{
  auto server = std::make_shared<ControllerShim>();
  auto pid = std::make_shared<FakeEndLocalizationController>();
  server->setEndLocalizationControllerForTest(pid);

  auto latched_goal = makePose(1.25, -0.35);
  latched_goal.pose.orientation.z = 0.3;
  latched_goal.pose.orientation.w = std::sqrt(1.0 - 0.3 * 0.3);
  pid->seedEffectiveGoal(server->goalGeneration(), latched_goal);

  auto goal_checker_target = makePose(9.0, 8.0);
  ASSERT_TRUE(server->applyEndLocalizationGoalForTest(goal_checker_target));
  EXPECT_EQ(goal_checker_target.header.frame_id, latched_goal.header.frame_id);
  EXPECT_DOUBLE_EQ(goal_checker_target.pose.position.x, latched_goal.pose.position.x);
  EXPECT_DOUBLE_EQ(goal_checker_target.pose.position.y, latched_goal.pose.position.y);
  EXPECT_DOUBLE_EQ(goal_checker_target.pose.orientation.z, latched_goal.pose.orientation.z);
  EXPECT_DOUBLE_EQ(goal_checker_target.pose.orientation.w, latched_goal.pose.orientation.w);
}

TEST(InitialMotionState, TerminalInfeasibleOutsideDynamicGuardDoesNotLatch)
{
  auto controller = std::make_shared<ControllerShim>();
  nav2_core::SpeedPlanningDiagnostics diagnostics;
  diagnostics.terminal_infeasible = true;
  diagnostics.terminal_brake_required = false;
  diagnostics.terminal_distance_available = 3.24;
  diagnostics.terminal_braking_distance = 0.0;

  controller->updateTerminalBrakeForTest(diagnostics, false);

  EXPECT_FALSE(controller->terminalBrakeLatched());
}

TEST(InitialMotionState, StationaryTerminalConditionWaitsForLongitudinalContinuity)
{
  auto controller = std::make_shared<ControllerShim>();
  nav2_core::SpeedPlanningDiagnostics diagnostics;
  diagnostics.terminal_brake_required = true;
  diagnostics.terminal_distance_available = 0.02;
  diagnostics.terminal_braking_distance = 0.0;

  controller->updateTerminalBrakeForTest(diagnostics, false);
  EXPECT_FALSE(controller->terminalBrakeLatched());

  controller->updateTerminalBrakeForTest(diagnostics, true);
  EXPECT_TRUE(controller->terminalBrakeLatched());
}

TEST(InitialMotionState, MovingTerminalConditionLatchesWithoutPriorReceipt)
{
  auto controller = std::make_shared<ControllerShim>();
  nav2_core::SpeedPlanningDiagnostics diagnostics;
  diagnostics.terminal_brake_required = true;
  diagnostics.terminal_distance_available = 0.35;
  diagnostics.terminal_braking_distance = 0.4;

  controller->updateTerminalBrakeForTest(diagnostics, false);

  EXPECT_TRUE(controller->terminalBrakeLatched());
}

TEST(InitialMotionState, ObstacleBrakeDoesNotLatchGoalTerminalState)
{
  auto controller = std::make_shared<ControllerShim>();
  nav2_core::SpeedPlanningDiagnostics diagnostics;
  diagnostics.terminal_branch = kObstacleTerminalBranch;
  diagnostics.terminal_brake_required = true;
  diagnostics.terminal_distance_available = 0.35;
  diagnostics.terminal_braking_distance = 0.4;

  controller->updateTerminalBrakeForTest(diagnostics, true);

  EXPECT_FALSE(controller->terminalBrakeLatched());
}

TEST(ProgressCheckerObstacleStop, FeasiblePlannedStopPausesProgressCheck)
{
  auto controller = std::make_shared<ControllerShim>();
  controller->setProgressPlanningContext(
    makeStoppedTrajectory(), kObstacleTerminalBranch, false);
  controller->setProgressCheckResult(false);

  EXPECT_TRUE(controller->checkProgressForTest(makePose()));
  EXPECT_TRUE(controller->isProgressCheckPaused());
  EXPECT_EQ(controller->progressCheckCount(), 0u);
  EXPECT_EQ(controller->progressResetCount(), 1u);

  // 同一个停障事件不得重复reset或重复打印进入日志。
  EXPECT_TRUE(controller->checkProgressForTest(makePose()));
  EXPECT_EQ(controller->progressCheckCount(), 0u);
  EXPECT_EQ(controller->progressResetCount(), 1u);
}

TEST(ProgressCheckerObstacleStop, MovingOrInfeasiblePlanDoesNotPause)
{
  auto moving_controller = std::make_shared<ControllerShim>();
  auto accelerating_trajectory = makeTrajectory();
  accelerating_trajectory.points.front().longitudinal_velocity = 0.0;
  moving_controller->setProgressPlanningContext(
    accelerating_trajectory, kObstacleTerminalBranch, false);
  moving_controller->setProgressCheckResult(false);

  EXPECT_FALSE(moving_controller->checkProgressForTest(makePose()));
  EXPECT_FALSE(moving_controller->isProgressCheckPaused());
  EXPECT_EQ(moving_controller->progressCheckCount(), 1u);

  auto infeasible_controller = std::make_shared<ControllerShim>();
  infeasible_controller->setProgressPlanningContext(
    makeStoppedTrajectory(), kObstacleTerminalBranch, true);
  infeasible_controller->setProgressCheckResult(false);

  EXPECT_FALSE(infeasible_controller->checkProgressForTest(makePose()));
  EXPECT_FALSE(infeasible_controller->isProgressCheckPaused());
  EXPECT_EQ(infeasible_controller->progressCheckCount(), 1u);
}

TEST(ProgressCheckerObstacleStop, ObstacleClearResetsBaselineBeforeNormalCheck)
{
  auto controller = std::make_shared<ControllerShim>();
  controller->setProgressPlanningContext(
    makeStoppedTrajectory(), kObstacleTerminalBranch, false);
  ASSERT_TRUE(controller->checkProgressForTest(makePose()));

  controller->setProgressPlanningContext(makeTrajectory(), kNoTerminalBranch, false);
  controller->setProgressCheckResult(true);
  EXPECT_TRUE(controller->checkProgressForTest(makePose()));
  EXPECT_FALSE(controller->isProgressCheckPaused());
  EXPECT_EQ(controller->progressCheckCount(), 1u);
  EXPECT_EQ(controller->progressResetCount(), 2u);
}

TEST(ProgressCheckerObstacleStop, HardStopWaitPausesUntilCollisionRiskClears)
{
  auto controller = std::make_shared<ControllerShim>();
  controller->setProgressPlanningContext(makeTrajectory(), kNoTerminalBranch, false);
  controller->setProgressCheckResult(false);
  controller->setCollisionStopActiveForTest(true);

  EXPECT_TRUE(controller->checkProgressForTest(makePose()));
  EXPECT_TRUE(controller->isProgressCheckPaused());
  EXPECT_EQ(controller->progressCheckCount(), 0u);
  EXPECT_EQ(controller->progressResetCount(), 1u);

  EXPECT_TRUE(controller->checkProgressForTest(makePose()));
  EXPECT_EQ(controller->progressCheckCount(), 0u);
  EXPECT_EQ(controller->progressResetCount(), 1u);

  controller->setCollisionStopActiveForTest(false);
  controller->setProgressCheckResult(true);
  EXPECT_TRUE(controller->checkProgressForTest(makePose()));
  EXPECT_FALSE(controller->isProgressCheckPaused());
  EXPECT_EQ(controller->progressCheckCount(), 1u);
  EXPECT_EQ(controller->progressResetCount(), 2u);

  controller->setCollisionStopActiveForTest(true);
  controller->resetStages();
  EXPECT_FALSE(controller->isCollisionStopActive());
  EXPECT_FALSE(controller->isProgressCheckPaused());
}

}  // namespace
