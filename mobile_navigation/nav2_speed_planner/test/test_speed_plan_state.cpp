#include <gtest/gtest.h>

#include <limits>
#include <utility>
#include <vector>

#include "nav2_speed_planner/speed_plan_state.hpp"
#include "rclcpp/duration.hpp"

namespace
{

using nav2_speed_planner::BoundaryDecision;
using nav2_speed_planner::BoundaryReason;
using nav2_speed_planner::SpeedPlanState;

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

void acceptRecentTrajectory(SpeedPlanState & state)
{
  state.accept(
    makeTrajectory(), testTime() - rclcpp::Duration::from_seconds(0.1));
}

void expectReason(const BoundaryDecision & decision, BoundaryReason reason)
{
  EXPECT_FALSE(decision.inherited());
  ASSERT_TRUE(decision.reason.has_value());
  EXPECT_EQ(*decision.reason, reason);
}

TEST(SpeedPlanStateTest, ReportsFirstCycleBeforeAnyAcceptedTrajectory)
{
  SpeedPlanState state;

  expectReason(
    state.boundary(makePath(), makeVelocity(), testTime()),
    BoundaryReason::FIRST_CYCLE);
}

TEST(SpeedPlanStateTest, ReportsExplicitInvalidationAfterReset)
{
  SpeedPlanState state;
  acceptRecentTrajectory(state);
  state.reset();

  expectReason(
    state.boundary(makePath(), makeVelocity(), testTime()),
    BoundaryReason::EXPLICIT_INVALIDATION);
}

TEST(SpeedPlanStateTest, RejectsExpiredTrajectory)
{
  SpeedPlanState state;
  state.accept(
    makeTrajectory(), testTime() - rclcpp::Duration::from_seconds(0.6));

  expectReason(
    state.boundary(makePath(), makeVelocity(), testTime()),
    BoundaryReason::EXPIRED);
}

TEST(SpeedPlanStateTest, RejectsFrameMismatch)
{
  SpeedPlanState state;
  acceptRecentTrajectory(state);
  auto path = makePath();
  path.header.frame_id = "odom";
  for (auto & pose : path.poses) {
    pose.header.frame_id = path.header.frame_id;
  }

  expectReason(
    state.boundary(path, makeVelocity(), testTime()),
    BoundaryReason::FRAME_MISMATCH);
}

TEST(SpeedPlanStateTest, RejectsGoalMismatch)
{
  SpeedPlanState state;
  acceptRecentTrajectory(state);
  auto path = makePath();
  path.poses.back().pose.position.x = 2.2;

  expectReason(
    state.boundary(path, makeVelocity(), testTime()),
    BoundaryReason::GOAL_MISMATCH);
}

TEST(SpeedPlanStateTest, RejectsBranchMismatch)
{
  SpeedPlanState state;
  acceptRecentTrajectory(state);

  expectReason(
    state.boundary(
      makePath({{0.5, 0.0}, {0.5, 0.5}, {2.0, 0.0}}),
      makeVelocity(), testTime()),
    BoundaryReason::BRANCH_MISMATCH);
}

TEST(SpeedPlanStateTest, RejectsSpatialMismatch)
{
  SpeedPlanState state;
  acceptRecentTrajectory(state);

  expectReason(
    state.boundary(
      makePath({{0.5, 0.3}, {1.0, 0.3}, {2.0, 0.0}}),
      makeVelocity(), testTime()),
    BoundaryReason::SPATIAL_MISMATCH);
}

TEST(SpeedPlanStateTest, RejectsVelocityMismatch)
{
  SpeedPlanState state;
  acceptRecentTrajectory(state);

  expectReason(
    state.boundary(makePath(), makeVelocity(0.7), testTime()),
    BoundaryReason::VELOCITY_MISMATCH);
}

TEST(SpeedPlanStateTest, RejectsInvalidNumericInput)
{
  SpeedPlanState state;
  acceptRecentTrajectory(state);
  auto velocity = makeVelocity();
  velocity.linear.x = std::numeric_limits<double>::quiet_NaN();

  expectReason(
    state.boundary(makePath(), velocity, testTime()),
    BoundaryReason::INVALID_NUMERIC);
}

TEST(SpeedPlanStateTest, InterpolatesAcceptedAccelerationForCompatiblePath)
{
  SpeedPlanState state;
  acceptRecentTrajectory(state);

  const auto decision = state.boundary(makePath(), makeVelocity(), testTime());

  EXPECT_TRUE(decision.inherited());
  EXPECT_FALSE(decision.reason.has_value());
  EXPECT_NEAR(decision.boundary.acceleration, 0.15, 1e-9);
  EXPECT_NEAR(decision.boundary.source_age, 0.1, 1e-9);
  EXPECT_NEAR(decision.boundary.velocity.linear.x, 0.3, 1e-9);
}

TEST(SpeedPlanStateTest, EmptyAcceptanceFailsClosed)
{
  SpeedPlanState state;
  state.accept(nav_interfaces::msg::RobotTrajectory(), testTime());

  EXPECT_FALSE(state.valid());
  expectReason(
    state.boundary(makePath(), makeVelocity(), testTime()),
    BoundaryReason::INVALID_NUMERIC);
}

TEST(SpeedPlanStateTest, ConfiguringStateClearsAcceptedTrajectory)
{
  SpeedPlanState state;
  acceptRecentTrajectory(state);
  ASSERT_TRUE(state.valid());

  SpeedPlanState::Config config;
  config.max_age = 1.0;
  state.configure(config);

  EXPECT_FALSE(state.valid());
  expectReason(
    state.boundary(makePath(), makeVelocity(), testTime()),
    BoundaryReason::FIRST_CYCLE);
}

}  // namespace
