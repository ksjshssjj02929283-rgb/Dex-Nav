#include <gtest/gtest.h>

#include <limits>
#include <utility>
#include <vector>

#include "nav2_speed_planner/core/initial_motion.hpp"
#include "rclcpp/duration.hpp"

namespace
{

using nav2_speed_planner::InitialMotionDecision;
using nav2_speed_planner::InitialMotionSource;
using nav2_speed_planner::InitialMotionArbiter;

nav_msgs::msg::Path makePath(
  const std::vector<std::pair<double, double>> & points =
  {{0.5, 0.0}, {1.0, 0.0}, {2.0, 0.0}})
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (const auto & point : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = path.header.frame_id;
    pose.pose.position.x = point.first;
    pose.pose.position.y = point.second;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

nav_interfaces::msg::RobotTrajectory makeTrajectory()
{
  nav_interfaces::msg::RobotTrajectory trajectory;
  trajectory.header.frame_id = "map";
  for (size_t index = 0; index < 3; ++index) {
    nav_interfaces::msg::TrajectoryPoint point;
    point.pose.header.frame_id = trajectory.header.frame_id;
    point.pose.pose.position.x = static_cast<double>(index);
    point.pose.pose.orientation.w = 1.0;
    point.longitudinal_velocity = 0.2 + 0.2 * static_cast<double>(index);
    point.acceleration = 0.1 + 0.1 * static_cast<double>(index);
    trajectory.points.push_back(point);
  }
  return trajectory;
}

nav_interfaces::msg::RobotTrajectory makeTimedTrajectory()
{
  nav_interfaces::msg::RobotTrajectory trajectory;
  trajectory.header.frame_id = "map";
  for (size_t index = 0; index <= 10; ++index) {
    nav_interfaces::msg::TrajectoryPoint point;
    point.pose.header.frame_id = trajectory.header.frame_id;
    point.pose.pose.position.x = 0.1 * static_cast<double>(index);
    point.pose.pose.orientation.w = 1.0;
    point.longitudinal_velocity = 0.3;
    point.acceleration = -0.1;
    point.relative_time = rclcpp::Duration::from_seconds(
      0.1 * static_cast<double>(index));
    trajectory.points.push_back(point);
  }
  return trajectory;
}

nav_interfaces::msg::RobotTrajectory makeStartupTrajectory()
{
  nav_interfaces::msg::RobotTrajectory trajectory;
  trajectory.header.frame_id = "map";
  const std::vector<double> speeds{0.0, 0.005, 0.03, 0.10};
  for (size_t index = 0; index < speeds.size(); ++index) {
    nav_interfaces::msg::TrajectoryPoint point;
    point.pose.header.frame_id = trajectory.header.frame_id;
    point.pose.pose.position.x = 0.2 * static_cast<double>(index);
    point.pose.pose.orientation.w = 1.0;
    point.longitudinal_velocity = speeds[index];
    point.acceleration = 0.12;
    point.relative_time = rclcpp::Duration::from_seconds(0.1 * static_cast<double>(index));
    trajectory.points.push_back(point);
  }
  return trajectory;
}

nav_msgs::msg::Path makeDensePath(double start = 0.2)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (double x = start; x <= 1.0 + 1e-9; x += 0.1) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = path.header.frame_id;
    pose.pose.position.x = x;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

geometry_msgs::msg::Twist makeVelocity(double longitudinal_velocity = 0.3)
{
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = longitudinal_velocity;
  return velocity;
}

rclcpp::Time testTime()
{
  return rclcpp::Time(10, 0, RCL_ROS_TIME);
}

void acceptRecentTrajectory(
  InitialMotionArbiter & state,
  double consumed_target_velocity = 0.35,
  double consumed_target_acceleration = -0.2)
{
  state.accept(
    makeTrajectory(), consumed_target_velocity, consumed_target_acceleration,
    0.0,
    testTime() - rclcpp::Duration::from_seconds(0.1));
}

void acceptStartupTrajectory(InitialMotionArbiter & state)
{
  state.accept(
    makeStartupTrajectory(), 0.005, 0.12, 0.1,
    testTime() - rclcpp::Duration::from_seconds(0.1));
}

void expectReason(const InitialMotionDecision & decision, InitialMotionSource reason)
{
  EXPECT_FALSE(decision.inherited());
  ASSERT_TRUE(decision.reason.has_value());
  EXPECT_EQ(*decision.reason, reason);
}

TEST(InitialMotionArbiterTest, ReportsFirstCycleBeforeAnyAcceptedTrajectory)
{
  InitialMotionArbiter state;

  expectReason(
    state.resolve(makePath(), makeVelocity(), testTime()),
    InitialMotionSource::FIRST_CYCLE);
}

TEST(InitialMotionArbiterTest, ReportsExplicitInvalidationAfterReset)
{
  InitialMotionArbiter state;
  acceptRecentTrajectory(state);
  state.reset();

  expectReason(
    state.resolve(makePath(), makeVelocity(), testTime()),
    InitialMotionSource::EXPLICIT_INVALIDATION);
}

TEST(InitialMotionArbiterTest, RejectsExpiredTrajectory)
{
  InitialMotionArbiter state;
  state.accept(
    makeTrajectory(), 0.35, -0.2,
    0.0,
    testTime() - rclcpp::Duration::from_seconds(0.6));

  expectReason(
    state.resolve(makePath(), makeVelocity(), testTime()),
    InitialMotionSource::EXPIRED);
}

TEST(InitialMotionArbiterTest, RejectsFrameMismatch)
{
  InitialMotionArbiter state;
  acceptRecentTrajectory(state);
  auto path = makePath();
  path.header.frame_id = "odom";
  for (auto & pose : path.poses) {
    pose.header.frame_id = path.header.frame_id;
  }

  expectReason(
    state.resolve(path, makeVelocity(), testTime()),
    InitialMotionSource::FRAME_MISMATCH);
}

TEST(InitialMotionArbiterTest, RejectsGoalMismatch)
{
  InitialMotionArbiter state;
  acceptRecentTrajectory(state);
  auto path = makePath();
  path.poses.back().pose.position.x = 2.2;

  expectReason(
    state.resolve(path, makeVelocity(), testTime()),
    InitialMotionSource::GOAL_MISMATCH);
}

TEST(InitialMotionArbiterTest, RejectsBranchMismatch)
{
  InitialMotionArbiter state;
  acceptRecentTrajectory(state);

  expectReason(
    state.resolve(
      makePath({{0.5, 0.0}, {0.5, 0.5}, {2.0, 0.0}}),
      makeVelocity(), testTime()),
    InitialMotionSource::BRANCH_MISMATCH);
}

TEST(InitialMotionArbiterTest, RejectsSpatialMismatch)
{
  InitialMotionArbiter state;
  acceptRecentTrajectory(state);

  expectReason(
    state.resolve(
      makePath({{0.5, 0.3}, {1.0, 0.3}, {2.0, 0.0}}),
      makeVelocity(), testTime()),
    InitialMotionSource::SPATIAL_MISMATCH);
}

// P1-2（重构方案 §2.3）：结构性门全部通过、但速度偏离超出 replan_dev 时，
// 不再整体拒绝——加速度仍然继承（inherited()==true），只有速度重锚定到实测。
TEST(InitialMotionArbiterTest, PartiallyReseedsOnVelocityDeviationKeepingAcceleration)
{
  InitialMotionArbiter state;
  acceptRecentTrajectory(state);  // consumed accel = -0.2

  const auto decision = state.resolve(makePath(), makeVelocity(0.7), testTime());

  ASSERT_TRUE(decision.reason.has_value());
  EXPECT_EQ(*decision.reason, InitialMotionSource::VELOCITY_MISMATCH);
  EXPECT_TRUE(decision.inherited());               // 加速度仍继承
  EXPECT_FALSE(decision.velocity_inherited);        // 速度未继承
  EXPECT_NEAR(decision.state.acceleration, -0.2, 1e-9);
  EXPECT_NEAR(decision.state.velocity.linear.x, 0.7, 1e-9);  // 重锚定到实测，不是采样值
}

// 偏离在 replan_dev 内：不仅加速度，速度也直接继承计划采样值（不再无条件取实测）。
TEST(InitialMotionArbiterTest, InheritsPlannedVelocityWithinReplanDev)
{
  InitialMotionArbiter state;
  acceptRecentTrajectory(state);  // 路径投影处的计划采样速度 = 0.3

  // 实测 0.35 与采样 0.3 相差 0.05 < 默认 replan_dev(0.10)。
  const auto decision = state.resolve(makePath(), makeVelocity(0.35), testTime());

  EXPECT_TRUE(decision.inherited());
  EXPECT_TRUE(decision.velocity_inherited);
  EXPECT_FALSE(decision.reason.has_value());
  EXPECT_NEAR(decision.state.velocity.linear.x, 0.3, 1e-9);  // 取计划值，不是实测 0.35
  EXPECT_NEAR(decision.state.acceleration, -0.2, 1e-9);
}

// 174118 L3复现：终端真实0.125、旧轨迹采样约0.071，绝对偏差仍小于
// 巡航replan_dev=0.10。终端制动不能继续继承偏低速度，否则会把制动距离
// 估小；只重锚定速度到实测，计划加速度连续性仍保留。
TEST(InitialMotionArbiterTest, TerminalOverspeedUsesTighterReplanThreshold)
{
  InitialMotionArbiter state;
  InitialMotionArbiter::Config config;
  config.replan_dev = 0.10;
  config.terminal_overspeed_replan_dev = 0.02;
  state.configure(config);
  acceptRecentTrajectory(state);

  // makePath起点x=0.5，对makeTrajectory的空间采样速度为0.3。
  const auto normal = state.resolve(makePath(), makeVelocity(0.355), testTime(), false);
  EXPECT_TRUE(normal.velocity_inherited);
  EXPECT_NEAR(normal.state.velocity.linear.x, 0.3, 1e-9);

  const auto terminal = state.resolve(makePath(), makeVelocity(0.355), testTime(), true);
  ASSERT_TRUE(terminal.reason.has_value());
  EXPECT_EQ(*terminal.reason, InitialMotionSource::VELOCITY_MISMATCH);
  EXPECT_FALSE(terminal.velocity_inherited);
  EXPECT_TRUE(terminal.inherited());
  EXPECT_NEAR(terminal.state.velocity.linear.x, 0.355, 1e-9);
  EXPECT_NEAR(terminal.state.acceleration, -0.2, 1e-9);
}

// 连续 failsafe_deviation_cycles 次都命中 VELOCITY_MISMATCH，第 N 次升级为
// FAILSAFE：加速度也不再继承，防止计划持续脱离现实（打滑/卡阻）时被无限期原谅。
TEST(InitialMotionArbiterTest, EscalatesToFailsafeAfterConsecutiveVelocityDeviation)
{
  InitialMotionArbiter state;
  InitialMotionArbiter::Config config;
  config.failsafe_deviation_cycles = 3;
  state.configure(config);
  acceptRecentTrajectory(state);

  for (int cycle = 0; cycle < 2; ++cycle) {
    const auto decision = state.resolve(makePath(), makeVelocity(0.7), testTime());
    ASSERT_TRUE(decision.reason.has_value());
    EXPECT_EQ(*decision.reason, InitialMotionSource::VELOCITY_MISMATCH) << "cycle=" << cycle;
    EXPECT_TRUE(decision.inherited()) << "cycle=" << cycle;
  }

  const auto failsafe_decision = state.resolve(makePath(), makeVelocity(0.7), testTime());
  ASSERT_TRUE(failsafe_decision.reason.has_value());
  EXPECT_EQ(*failsafe_decision.reason, InitialMotionSource::VELOCITY_MISMATCH_FAILSAFE);
  EXPECT_FALSE(failsafe_decision.inherited());
}

// 连续偏离计数在正常（继承）周期后清零：偏离两次、正常一次、再偏离两次，
// 不应该在第二轮偏离的第二次就触发 failsafe（若计数未清零则会）。
TEST(InitialMotionArbiterTest, ResetsDeviationCounterAfterInheritedCycle)
{
  InitialMotionArbiter state;
  InitialMotionArbiter::Config config;
  config.failsafe_deviation_cycles = 3;
  state.configure(config);
  acceptRecentTrajectory(state);

  state.resolve(makePath(), makeVelocity(0.7), testTime());
  state.resolve(makePath(), makeVelocity(0.7), testTime());
  const auto good_cycle = state.resolve(makePath(), makeVelocity(0.3), testTime());
  EXPECT_TRUE(good_cycle.velocity_inherited);

  const auto after_reset = state.resolve(makePath(), makeVelocity(0.7), testTime());
  ASSERT_TRUE(after_reset.reason.has_value());
  EXPECT_EQ(*after_reset.reason, InitialMotionSource::VELOCITY_MISMATCH)
    << "counter should have reset after the inherited cycle, not escalated to failsafe";
}

TEST(InitialMotionArbiterTest, RejectsInvalidNumericInput)
{
  InitialMotionArbiter state;
  acceptRecentTrajectory(state);
  auto velocity = makeVelocity();
  velocity.linear.x = std::numeric_limits<double>::quiet_NaN();

  expectReason(
    state.resolve(makePath(), velocity, testTime()),
    InitialMotionSource::INVALID_NUMERIC);
}

TEST(InitialMotionArbiterTest, InheritsConsumedTargetAccelerationForCompatiblePath)
{
  InitialMotionArbiter state;
  acceptRecentTrajectory(state);

  const auto decision = state.resolve(makePath(), makeVelocity(), testTime());

  EXPECT_TRUE(decision.inherited());
  EXPECT_TRUE(decision.velocity_inherited);
  EXPECT_FALSE(decision.reason.has_value());
  EXPECT_NEAR(decision.state.acceleration, -0.2, 1e-9);
  EXPECT_NEAR(decision.state.source_age, 0.1, 1e-9);
  EXPECT_NEAR(decision.state.velocity.linear.x, 0.3, 1e-9);
}

TEST(InitialMotionArbiterTest, LowSpeedStartupUsesMeasuredVelocityInsteadOfZeroSpatialSample)
{
  InitialMotionArbiter state;
  acceptStartupTrajectory(state);

  nav_msgs::msg::Path path = makePath({{0.0, 0.0}, {0.3, 0.0}, {0.6, 0.0}});
  const auto decision = state.resolve(path, makeVelocity(0.005), testTime());

  ASSERT_TRUE(decision.reason.has_value());
  EXPECT_EQ(*decision.reason, InitialMotionSource::LOW_SPEED_STARTUP_RESEED);
  EXPECT_FALSE(decision.velocity_inherited);
  EXPECT_NEAR(decision.sampled_velocity, 0.0, 1e-9);
  EXPECT_NEAR(decision.state.velocity.linear.x, 0.005, 1e-9);
  EXPECT_TRUE(decision.hasStitchingPrefix());
  EXPECT_NEAR(decision.stitching_prefix.points.front().longitudinal_velocity, 0.005, 1e-9);
}

TEST(InitialMotionArbiterTest, LowSpeedStartupRecoveryIsDisabledForTerminalBrake)
{
  InitialMotionArbiter state;
  acceptStartupTrajectory(state);

  nav_msgs::msg::Path path = makePath({{0.0, 0.0}, {0.3, 0.0}, {0.6, 0.0}});
  const auto decision = state.resolve(path, makeVelocity(0.005), testTime(), true);

  EXPECT_FALSE(decision.reason.has_value());
  EXPECT_TRUE(decision.velocity_inherited);
  EXPECT_NEAR(decision.state.velocity.linear.x, 0.0, 1e-9);
}

TEST(InitialMotionArbiterTest, PreservesTimedFutureTrajectoryPrefix)
{
  InitialMotionArbiter state;
  InitialMotionArbiter::Config config;
  config.stitching_horizon = 0.2;
  config.stitching_min_duration = 0.1;
  state.configure(config);
  state.accept(makeTimedTrajectory(), 0.3, -0.1, 0.2, testTime());

  const auto decision = state.resolve(makeDensePath(), makeVelocity(0.3), testTime());

  ASSERT_TRUE(decision.hasStitchingPrefix());
  ASSERT_GE(decision.stitching_prefix.points.size(), 3u);
  EXPECT_NEAR(decision.stitching_prefix.points.front().pose.pose.position.x, 0.2, 1e-9);
  EXPECT_NEAR(decision.stitching_prefix.points.front().acceleration, -0.1, 1e-9);
  EXPECT_NEAR(
    rclcpp::Duration(decision.stitching_prefix.points.front().relative_time).seconds(),
    0.0, 1e-9);
  EXPECT_NEAR(
    rclcpp::Duration(decision.stitching_prefix.points.back().relative_time).seconds(),
    0.2, 1e-9);
  EXPECT_NEAR(decision.stitching_prefix.points.back().pose.pose.position.x, 0.4, 1e-9);
  EXPECT_NEAR(decision.stitching_boundary.velocity.linear.x, 0.3, 1e-9);
  EXPECT_NEAR(decision.stitching_boundary.acceleration, -0.1, 1e-9);
}

TEST(InitialMotionArbiterTest, BuildsSuffixPathAtPrefixTailAndConcatenatesTime)
{
  InitialMotionArbiter state;
  state.accept(makeTimedTrajectory(), 0.3, -0.1, 0.2, testTime());
  const auto path = makeDensePath();
  const auto decision = state.resolve(path, makeVelocity(0.3), testTime());
  ASSERT_TRUE(decision.hasStitchingPrefix());

  nav2_speed_planner::TrajectoryStitchingPlan stitching;
  std::string rejection_reason;
  ASSERT_TRUE(
    nav2_speed_planner::prepareTrajectoryStitching(
      path, decision, 0.02, stitching, &rejection_reason)) << rejection_reason;
  ASSERT_GE(stitching.suffix_path.poses.size(), 2u);
  EXPECT_NEAR(stitching.suffix_path.poses.front().pose.position.x, 0.4, 1e-9);
  EXPECT_NEAR(stitching.suffix_path.poses.back().pose.position.x, 1.0, 1e-9);

  nav_interfaces::msg::RobotTrajectory suffix;
  suffix.header.frame_id = "map";
  for (size_t index = 0; index < 3; ++index) {
    nav_interfaces::msg::TrajectoryPoint point;
    point.pose.header.frame_id = "map";
    point.pose.pose.position.x = 0.4 + 0.1 * static_cast<double>(index);
    point.pose.pose.orientation.w = 1.0;
    point.longitudinal_velocity = 0.3;
    point.acceleration = -0.1;
    point.relative_time = rclcpp::Duration::from_seconds(
      0.1 * static_cast<double>(index));
    suffix.points.push_back(point);
  }
  const auto combined = nav2_speed_planner::concatenateStitchedTrajectory(
    stitching.prefix, suffix);
  ASSERT_EQ(combined.points.size(), stitching.prefix.points.size() + 2u);
  for (size_t index = 1; index < combined.points.size(); ++index) {
    EXPECT_GT(
      rclcpp::Duration(combined.points[index].relative_time).seconds(),
      rclcpp::Duration(combined.points[index - 1].relative_time).seconds());
  }
  EXPECT_NEAR(
    rclcpp::Duration(combined.points.back().relative_time).seconds(), 0.4, 1e-9);
}

TEST(InitialMotionArbiterTest, RejectsPrefixThatLeavesCurrentPath)
{
  InitialMotionArbiter state;
  state.accept(makeTimedTrajectory(), 0.3, -0.1, 0.2, testTime());
  const auto path = makeDensePath();
  auto decision = state.resolve(path, makeVelocity(0.3), testTime());
  ASSERT_TRUE(decision.hasStitchingPrefix());
  decision.stitching_prefix.points.back().pose.pose.position.y = 0.2;

  nav2_speed_planner::TrajectoryStitchingPlan stitching;
  std::string rejection_reason;
  EXPECT_FALSE(
    nav2_speed_planner::prepareTrajectoryStitching(
      path, decision, 0.05, stitching, &rejection_reason));
  EXPECT_EQ(rejection_reason, "prefix_path_mismatch");
}

TEST(InitialMotionArbiterTest, VelocityMismatchDisablesTrajectoryPrefix)
{
  InitialMotionArbiter state;
  state.accept(makeTimedTrajectory(), 0.3, -0.1, 0.2, testTime());

  const auto decision = state.resolve(makeDensePath(), makeVelocity(0.6), testTime());

  ASSERT_TRUE(decision.reason.has_value());
  EXPECT_EQ(*decision.reason, InitialMotionSource::VELOCITY_MISMATCH);
  EXPECT_FALSE(decision.hasStitchingPrefix());
}

TEST(InitialMotionArbiterTest, TimeMismatchDisablesTrajectoryPrefixButKeepsStateContinuity)
{
  InitialMotionArbiter state;
  InitialMotionArbiter::Config config;
  config.stitching_max_time_error = 0.05;
  state.configure(config);
  // Spatial projection is t=0.2, but the controller receipt claims it
  // consumed t=0.6. Keep v/a inheritance, reject only the future prefix.
  state.accept(makeTimedTrajectory(), 0.3, -0.1, 0.6, testTime());

  const auto decision = state.resolve(makeDensePath(), makeVelocity(0.3), testTime());

  EXPECT_TRUE(decision.velocity_inherited);
  EXPECT_TRUE(decision.inherited());
  EXPECT_FALSE(decision.hasStitchingPrefix());
}

TEST(InitialMotionArbiterTest, EmptyAcceptanceFailsClosed)
{
  InitialMotionArbiter state;
  state.accept(nav_interfaces::msg::RobotTrajectory(), 0.35, -0.2, 0.0, testTime());

  EXPECT_FALSE(state.valid());
  expectReason(
    state.resolve(makePath(), makeVelocity(), testTime()),
    InitialMotionSource::INVALID_NUMERIC);
}

TEST(InitialMotionArbiterTest, InvalidConsumedTargetFailsClosed)
{
  InitialMotionArbiter state;
  state.accept(
    makeTrajectory(), 0.35, std::numeric_limits<double>::quiet_NaN(), 0.0, testTime());

  EXPECT_FALSE(state.valid());
  expectReason(
    state.resolve(makePath(), makeVelocity(), testTime()),
    InitialMotionSource::INVALID_NUMERIC);
}

TEST(InitialMotionArbiterTest, ConfiguringStateClearsAcceptedTrajectory)
{
  InitialMotionArbiter state;
  acceptRecentTrajectory(state);
  ASSERT_TRUE(state.valid());

  InitialMotionArbiter::Config config;
  config.max_age = 1.0;
  state.configure(config);

  EXPECT_FALSE(state.valid());
  expectReason(
    state.resolve(makePath(), makeVelocity(), testTime()),
    InitialMotionSource::FIRST_CYCLE);
}

}  // namespace
