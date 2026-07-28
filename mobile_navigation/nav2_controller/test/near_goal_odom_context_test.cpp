// Copyright (c) 2026 Dexory
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include <cmath>

#include "gtest/gtest.h"
#include "near_goal_odom_context_impl.hpp"

namespace nav2_controller
{
namespace
{

geometry_msgs::msg::Quaternion yawQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(yaw * 0.5);
  quaternion.w = std::cos(yaw * 0.5);
  return quaternion;
}

NearGoalOdomInput makeInput(
  int64_t nanoseconds,
  double robot_x,
  double robot_y,
  double robot_yaw,
  double map_position_error,
  double map_yaw_error,
  double actual_vx = 0.0,
  double actual_wz = 0.0)
{
  NearGoalOdomInput input;
  input.stamp = rclcpp::Time(nanoseconds, RCL_ROS_TIME);
  input.evaluation_stamp = input.stamp;
  input.tf_valid = true;
  input.robot_in_odom.position.x = robot_x;
  input.robot_in_odom.position.y = robot_y;
  input.robot_in_odom.orientation = yawQuaternion(robot_yaw);
  input.live_goal_in_odom.orientation = yawQuaternion(0.0);
  input.live_map_position_error = map_position_error;
  input.live_map_lon_error = robot_x;
  input.live_map_lat_error = robot_y;
  input.live_map_yaw_error = map_yaw_error;
  input.actual_vx = actual_vx;
  input.actual_wz = actual_wz;
  return input;
}

NearGoalOdomParameters testParameters()
{
  NearGoalOdomParameters parameters;
  parameters.mode = NearGoalOdomMode::OBSERVE;
  parameters.entry_confirm_cycles = 1;
  parameters.position_confirm_cycles = 1;
  parameters.fine_yaw_entry_confirm_cycles = 1;
  parameters.stop_confirm_cycles = 2;
  return parameters;
}

TEST(NearGoalOdomContextTest, AdvancesCompleteObservedStateMachine)
{
  NearGoalOdomContext context(testParameters());
  context.reset(7, NearGoalTransitionReason::GOAL_CHANGED);

  context.update(makeInput(100000000, 0.05, 0.0, 0.20, 0.05, 0.20));
  EXPECT_EQ(context.snapshot().stage, NearGoalOdomStage::ODOM_POSITION);

  context.update(makeInput(200000000, 0.0, 0.0, 0.20, 0.0, 0.20));
  EXPECT_EQ(context.snapshot().stage, NearGoalOdomStage::MAP_YAW_COARSE);

  context.update(makeInput(300000000, 0.0, 0.0, 0.05, 0.0, 0.05));
  EXPECT_EQ(context.snapshot().stage, NearGoalOdomStage::ODOM_YAW_FINE);

  context.update(makeInput(400000000, 0.0, 0.0, 0.01, 0.0, 0.01));
  EXPECT_EQ(context.snapshot().stage, NearGoalOdomStage::STOP_CONFIRM);
  EXPECT_EQ(
    context.snapshot().control_directive,
    NearGoalControlDirective::HOLD_ZERO);

  context.update(makeInput(500000000, 0.0, 0.0, 0.01, 0.0, 0.01));
  EXPECT_EQ(context.snapshot().stage, NearGoalOdomStage::SUCCESS);
  EXPECT_EQ(context.snapshot().goal_generation, 7u);
}

TEST(NearGoalOdomContextTest, ReportsOdomPositionErrorOnEntryLatchCycle)
{
  NearGoalOdomContext context(testParameters());
  context.reset(21, NearGoalTransitionReason::GOAL_CHANGED);

  auto input = makeInput(100000000, 0.95, 0.10, 0.0, 0.05, 0.0);
  input.live_goal_in_odom.position.x = 1.0;
  input.live_goal_in_odom.position.y = 0.20;
  const auto snapshot = context.update(input);

  ASSERT_EQ(snapshot.stage, NearGoalOdomStage::ODOM_POSITION);
  ASSERT_TRUE(snapshot.position_goal_latched);
  EXPECT_NEAR(snapshot.odom_lon_error, -0.05, 1e-9);
  EXPECT_NEAR(snapshot.odom_lat_error, -0.10, 1e-9);
}

TEST(NearGoalOdomContextTest, BrakesBeforeLatchingPositionWhileMoving)
{
  auto parameters = testParameters();
  parameters.mode = NearGoalOdomMode::CONTROL;
  parameters.position_confirm_cycles = 2;
  NearGoalOdomContext context(parameters);
  context.reset(22, NearGoalTransitionReason::GOAL_CHANGED);

  context.update(makeInput(100000000, 0.02, 0.0, 0.20, 0.02, 0.20));
  context.update(makeInput(200000000, 0.01, 0.0, 0.20, 0.01, 0.20, 0.04, 0.0));
  const auto braking_snapshot =
    context.update(makeInput(300000000, 0.0, 0.0, 0.20, 0.0, 0.20, 0.04, 0.0));

  EXPECT_EQ(braking_snapshot.stage, NearGoalOdomStage::ODOM_POSITION);
  EXPECT_EQ(
    braking_snapshot.control_directive,
    NearGoalControlDirective::HOLD_ZERO);
  EXPECT_EQ(
    braking_snapshot.transition_reason,
    NearGoalTransitionReason::POSITION_SETTLING);
  EXPECT_FALSE(braking_snapshot.position_reached_latched);

  const auto stopped_snapshot =
    context.update(makeInput(400000000, 0.0, 0.0, 0.20, 0.0, 0.20));
  EXPECT_EQ(stopped_snapshot.stage, NearGoalOdomStage::MAP_YAW_COARSE);
  EXPECT_EQ(stopped_snapshot.control_directive, NearGoalControlDirective::NORMAL);
  EXPECT_TRUE(stopped_snapshot.position_reached_latched);
}

TEST(NearGoalOdomContextTest, HandsOffSameDirectionRotationWithoutFullStop)
{
  auto parameters = testParameters();
  parameters.mode = NearGoalOdomMode::CONTROL;
  parameters.position_confirm_cycles = 2;
  NearGoalOdomContext context(parameters);
  context.reset(24, NearGoalTransitionReason::GOAL_CHANGED);

  context.update(makeInput(100000000, 0.02, 0.0, 0.20, 0.02, 0.20));
  context.update(makeInput(200000000, 0.01, 0.0, 0.20, 0.01, 0.20, 0.0, -0.15));
  const auto snapshot =
    context.update(makeInput(300000000, 0.0, 0.0, 0.20, 0.0, 0.20, 0.0, -0.15));

  EXPECT_EQ(snapshot.stage, NearGoalOdomStage::MAP_YAW_COARSE);
  EXPECT_EQ(snapshot.control_directive, NearGoalControlDirective::NORMAL);
  EXPECT_TRUE(snapshot.position_reached_latched);
}

TEST(NearGoalOdomContextTest, StopsOppositeRotationBeforeYawHandoff)
{
  auto parameters = testParameters();
  parameters.mode = NearGoalOdomMode::CONTROL;
  parameters.position_confirm_cycles = 2;
  NearGoalOdomContext context(parameters);
  context.reset(25, NearGoalTransitionReason::GOAL_CHANGED);

  context.update(makeInput(100000000, 0.02, 0.0, 0.20, 0.02, 0.20));
  context.update(makeInput(200000000, 0.01, 0.0, 0.20, 0.01, 0.20, 0.0, 0.15));
  const auto braking_snapshot =
    context.update(makeInput(300000000, 0.0, 0.0, 0.20, 0.0, 0.20, 0.0, 0.15));

  EXPECT_EQ(braking_snapshot.stage, NearGoalOdomStage::ODOM_POSITION);
  EXPECT_EQ(
    braking_snapshot.control_directive,
    NearGoalControlDirective::HOLD_ZERO);
  EXPECT_EQ(
    braking_snapshot.transition_reason,
    NearGoalTransitionReason::POSITION_SETTLING);

  const auto stopped_snapshot =
    context.update(makeInput(400000000, 0.0, 0.0, 0.20, 0.0, 0.20));
  EXPECT_EQ(stopped_snapshot.stage, NearGoalOdomStage::MAP_YAW_COARSE);
  EXPECT_EQ(stopped_snapshot.control_directive, NearGoalControlDirective::NORMAL);
}

TEST(NearGoalOdomContextTest, KeepsBrakingIfTranslationMovesDuringSameDirectionRotation)
{
  auto parameters = testParameters();
  parameters.mode = NearGoalOdomMode::CONTROL;
  parameters.position_confirm_cycles = 2;
  NearGoalOdomContext context(parameters);
  context.reset(26, NearGoalTransitionReason::GOAL_CHANGED);

  context.update(makeInput(100000000, 0.02, 0.0, 0.20, 0.02, 0.20));
  context.update(makeInput(200000000, 0.01, 0.0, 0.20, 0.01, 0.20, 0.04, -0.15));
  const auto snapshot =
    context.update(makeInput(300000000, 0.0, 0.0, 0.20, 0.0, 0.20, 0.04, -0.15));

  EXPECT_EQ(snapshot.stage, NearGoalOdomStage::ODOM_POSITION);
  EXPECT_EQ(snapshot.control_directive, NearGoalControlDirective::HOLD_ZERO);
  EXPECT_FALSE(snapshot.position_reached_latched);
}

TEST(NearGoalOdomContextTest, ResumesPositioningIfErrorLeavesToleranceWhileBraking)
{
  auto parameters = testParameters();
  parameters.mode = NearGoalOdomMode::CONTROL;
  parameters.position_confirm_cycles = 2;
  NearGoalOdomContext context(parameters);
  context.reset(23, NearGoalTransitionReason::GOAL_CHANGED);

  context.update(makeInput(100000000, 0.02, 0.0, 0.20, 0.02, 0.20));
  context.update(makeInput(200000000, 0.01, 0.0, 0.20, 0.01, 0.20, 0.04, 0.0));
  context.update(makeInput(300000000, 0.0, 0.0, 0.20, 0.0, 0.20, 0.04, 0.0));
  ASSERT_EQ(
    context.snapshot().control_directive,
    NearGoalControlDirective::HOLD_ZERO);

  const auto snapshot =
    context.update(makeInput(400000000, 0.03, 0.0, 0.20, 0.03, 0.20));
  EXPECT_EQ(snapshot.stage, NearGoalOdomStage::ODOM_POSITION);
  EXPECT_EQ(snapshot.control_directive, NearGoalControlDirective::NORMAL);
  EXPECT_FALSE(snapshot.position_reached_latched);
}

TEST(NearGoalOdomContextTest, BrakesBeforeReturningToPosition)
{
  auto parameters = testParameters();
  NearGoalOdomContext context(parameters);
  context.reset(8, NearGoalTransitionReason::GOAL_CHANGED);

  context.update(makeInput(100000000, 0.02, 0.0, 1.0, 0.02, 1.0));
  context.update(makeInput(200000000, 0.0, 0.0, 1.0, 0.0, 1.0));
  ASSERT_EQ(context.snapshot().stage, NearGoalOdomStage::MAP_YAW_COARSE);

  context.update(makeInput(300000000, 0.03, 0.0, 1.0, 0.03, 1.0, 0.0, 0.1));
  EXPECT_EQ(context.snapshot().stage, NearGoalOdomStage::MAP_YAW_COARSE);
  EXPECT_EQ(
    context.snapshot().control_directive,
    NearGoalControlDirective::HOLD_ZERO);

  context.update(makeInput(400000000, 0.03, 0.0, 1.0, 0.03, 1.0));
  EXPECT_EQ(context.snapshot().stage, NearGoalOdomStage::ODOM_POSITION);
  EXPECT_EQ(
    context.snapshot().control_directive,
    NearGoalControlDirective::NORMAL);
}

TEST(NearGoalOdomContextTest, LargeGoalYawIsNotGlobalLocalizationFailure)
{
  auto parameters = testParameters();
  NearGoalOdomContext context(parameters);
  context.reset(9, NearGoalTransitionReason::GOAL_CHANGED);

  context.update(makeInput(100000000, 0.02, 0.0, 1.57, 0.02, 1.57));
  context.update(makeInput(200000000, 0.0, 0.0, 1.57, 0.0, 1.57));
  const auto snapshot = context.snapshot();
  EXPECT_EQ(snapshot.stage, NearGoalOdomStage::MAP_YAW_COARSE);
  EXPECT_TRUE(snapshot.global_sanity_ok);
}

TEST(NearGoalOdomContextTest, IgnoresTfFailureBeforeTerminalModeEntry)
{
  auto parameters = testParameters();
  parameters.mode = NearGoalOdomMode::CONTROL;
  parameters.tf_failure_timeout = 0.1;
  NearGoalOdomContext context(parameters);
  context.reset(4, NearGoalTransitionReason::GOAL_CHANGED);

  auto input = makeInput(100000000, 0.0, 0.0, 0.0, 0.5, 0.0);
  input.tf_valid = false;
  context.update(input);
  input.stamp = rclcpp::Time(2, 0, RCL_ROS_TIME);
  input.evaluation_stamp = input.stamp;
  const auto snapshot = context.update(input);

  EXPECT_EQ(snapshot.stage, NearGoalOdomStage::INACTIVE);
  EXPECT_EQ(snapshot.control_directive, NearGoalControlDirective::NORMAL);
}

TEST(NearGoalOdomContextTest, HoldsThenTerminatesOnTfTimeoutAfterEntry)
{
  auto parameters = testParameters();
  parameters.mode = NearGoalOdomMode::CONTROL;
  parameters.tf_failure_timeout = 0.1;
  NearGoalOdomContext context(parameters);
  context.reset(11, NearGoalTransitionReason::GOAL_CHANGED);
  context.update(makeInput(100000000, 0.05, 0.0, 0.0, 0.05, 0.0));
  ASSERT_EQ(context.snapshot().stage, NearGoalOdomStage::ODOM_POSITION);

  auto input = makeInput(200000000, 0.05, 0.0, 0.0, 0.05, 0.0);
  input.tf_valid = false;
  auto snapshot = context.update(input);
  EXPECT_EQ(snapshot.stage, NearGoalOdomStage::ODOM_POSITION);
  EXPECT_EQ(snapshot.control_directive, NearGoalControlDirective::HOLD_ZERO);
  EXPECT_EQ(snapshot.transition_reason, NearGoalTransitionReason::TF_FAILURE);

  // 模拟定位时间戳停滞；故障超时仍应按控制周期时间正常推进。
  input.evaluation_stamp = rclcpp::Time(int64_t{400000000}, RCL_ROS_TIME);
  snapshot = context.update(input);
  EXPECT_EQ(snapshot.stage, NearGoalOdomStage::INVALID);
  EXPECT_EQ(snapshot.control_directive, NearGoalControlDirective::TERMINATE);
}

TEST(NearGoalOdomContextTest, ResetsEntryConfirmationOnOdomJump)
{
  auto parameters = testParameters();
  parameters.entry_confirm_cycles = 2;
  NearGoalOdomContext context(parameters);
  context.reset(5, NearGoalTransitionReason::GOAL_CHANGED);

  context.update(makeInput(100000000, 0.0, 0.0, 0.0, 0.05, 0.0));
  const auto jump_snapshot =
    context.update(makeInput(200000000, 0.50, 0.0, 0.0, 0.05, 0.0));
  EXPECT_EQ(jump_snapshot.stage, NearGoalOdomStage::INACTIVE);
  EXPECT_EQ(jump_snapshot.control_directive, NearGoalControlDirective::NORMAL);

  context.update(makeInput(300000000, 0.0, 0.0, 0.0, 0.05, 0.0));
  EXPECT_EQ(context.snapshot().stage, NearGoalOdomStage::INACTIVE);
  context.update(makeInput(400000000, 0.0, 0.0, 0.0, 0.05, 0.0));
  EXPECT_EQ(context.snapshot().stage, NearGoalOdomStage::INACTIVE);
  context.update(makeInput(500000000, 0.0, 0.0, 0.0, 0.05, 0.0));
  EXPECT_EQ(context.snapshot().stage, NearGoalOdomStage::ODOM_POSITION);
}

TEST(NearGoalOdomContextTest, InvalidatesOdomJumpAfterTerminalEntry)
{
  NearGoalOdomContext context(testParameters());
  context.reset(12, NearGoalTransitionReason::GOAL_CHANGED);
  context.update(makeInput(100000000, 0.05, 0.0, 0.0, 0.05, 0.0));
  ASSERT_EQ(context.snapshot().stage, NearGoalOdomStage::ODOM_POSITION);

  const auto snapshot =
    context.update(makeInput(200000000, 0.50, 0.0, 0.0, 0.05, 0.0));
  EXPECT_EQ(snapshot.stage, NearGoalOdomStage::INVALID);
  EXPECT_EQ(snapshot.control_directive, NearGoalControlDirective::TERMINATE);
  EXPECT_EQ(snapshot.transition_reason, NearGoalTransitionReason::ODOM_JUMP);
}

TEST(NearGoalOdomContextTest, KeepsLatchedPositionAcrossLiveMapCorrection)
{
  NearGoalOdomContext context(testParameters());
  context.reset(13, NearGoalTransitionReason::GOAL_CHANGED);

  auto input = makeInput(100000000, 0.95, 0.0, 0.0, 0.05, 0.0);
  input.live_goal_in_odom.position.x = 1.0;
  context.update(input);
  ASSERT_EQ(context.snapshot().stage, NearGoalOdomStage::ODOM_POSITION);

  input = makeInput(200000000, 0.95, 0.0, 0.0, 0.07, 0.0);
  input.live_goal_in_odom.position.x = 1.02;
  const auto snapshot = context.update(input);
  EXPECT_DOUBLE_EQ(snapshot.goal_in_odom.position.x, 1.0);
  EXPECT_NEAR(snapshot.odom_lon_error, -0.05, 1e-9);
}

TEST(NearGoalOdomContextTest, KeepsPositionReachedAfterSmallMapCorrection)
{
  NearGoalOdomContext context(testParameters());
  context.reset(18, NearGoalTransitionReason::GOAL_CHANGED);

  auto input = makeInput(100000000, 0.98, 0.0, 0.20, 0.02, 0.20);
  input.live_goal_in_odom.position.x = 1.0;
  context.update(input);
  input = makeInput(200000000, 1.0, 0.0, 0.20, 0.0, 0.20);
  input.live_goal_in_odom.position.x = 1.0;
  context.update(input);
  ASSERT_EQ(context.snapshot().stage, NearGoalOdomStage::MAP_YAW_COARSE);
  ASSERT_TRUE(context.snapshot().position_reached_latched);

  input = makeInput(300000000, 1.0, 0.0, 0.20, 0.08, 0.20);
  input.live_goal_in_odom.position.x = 1.08;
  input.map_to_odom_x = 0.08;
  const auto snapshot = context.update(input);
  EXPECT_EQ(snapshot.stage, NearGoalOdomStage::MAP_YAW_COARSE);
  EXPECT_TRUE(snapshot.position_reached_latched);
  EXPECT_DOUBLE_EQ(snapshot.goal_in_odom.position.x, 1.0);
  EXPECT_NEAR(snapshot.odom_lon_error, 0.0, 1e-9);
}

TEST(NearGoalOdomContextTest, AcceptsRepeatedPoseTimestampWithoutFalseJump)
{
  auto parameters = testParameters();
  parameters.entry_confirm_cycles = 2;
  NearGoalOdomContext context(parameters);
  context.reset(6, NearGoalTransitionReason::GOAL_CHANGED);

  context.update(makeInput(100000000, 0.0, 0.0, 0.0, 0.05, 0.0));
  const auto snapshot =
    context.update(makeInput(100000000, 0.0, 0.0, 0.0, 0.05, 0.0));

  EXPECT_TRUE(snapshot.odom_continuous);
  EXPECT_EQ(snapshot.stage, NearGoalOdomStage::ODOM_POSITION);
}

TEST(NearGoalOdomContextTest, ReturnsToYawFineAfterStoppedAngleRebound)
{
  NearGoalOdomContext context(testParameters());
  context.reset(10, NearGoalTransitionReason::GOAL_CHANGED);

  context.update(makeInput(100000000, 0.01, 0.0, 0.20, 0.01, 0.20));
  context.update(makeInput(200000000, 0.0, 0.0, 0.20, 0.0, 0.20));
  context.update(makeInput(300000000, 0.0, 0.0, 0.05, 0.0, 0.05));
  context.update(makeInput(400000000, 0.0, 0.0, 0.01, 0.0, 0.01));
  ASSERT_EQ(context.snapshot().stage, NearGoalOdomStage::STOP_CONFIRM);

  const auto snapshot =
    context.update(makeInput(500000000, 0.0, 0.0, 0.03, 0.0, 0.03));
  EXPECT_EQ(snapshot.stage, NearGoalOdomStage::ODOM_YAW_FINE);
  EXPECT_EQ(snapshot.control_directive, NearGoalControlDirective::NORMAL);
  EXPECT_TRUE(snapshot.yaw_goal_latched);
}

TEST(NearGoalOdomContextTest, ChangesControlRevisionOnlyWhenDirectiveChanges)
{
  NearGoalOdomContext context(testParameters());
  context.reset(16, NearGoalTransitionReason::GOAL_CHANGED);

  context.update(makeInput(100000000, 0.01, 0.0, 0.20, 0.01, 0.20));
  context.update(makeInput(200000000, 0.0, 0.0, 0.20, 0.0, 0.20));
  context.update(makeInput(300000000, 0.0, 0.0, 0.05, 0.0, 0.05));
  const auto fine_snapshot = context.snapshot();
  ASSERT_EQ(fine_snapshot.stage, NearGoalOdomStage::ODOM_YAW_FINE);

  const auto stop_snapshot =
    context.update(makeInput(400000000, 0.0, 0.0, 0.01, 0.0, 0.01));
  ASSERT_EQ(stop_snapshot.stage, NearGoalOdomStage::STOP_CONFIRM);
  EXPECT_GT(stop_snapshot.control_revision, fine_snapshot.control_revision);

  const auto success_snapshot =
    context.update(makeInput(500000000, 0.0, 0.0, 0.01, 0.0, 0.01));
  EXPECT_EQ(success_snapshot.stage, NearGoalOdomStage::SUCCESS);
  EXPECT_EQ(success_snapshot.control_revision, stop_snapshot.control_revision);
}

TEST(NearGoalOdomContextTest, SupportsSingleCycleStopConfirmation)
{
  auto parameters = testParameters();
  parameters.stop_confirm_cycles = 1;
  NearGoalOdomContext context(parameters);
  context.reset(20, NearGoalTransitionReason::GOAL_CHANGED);

  context.update(makeInput(100000000, 0.01, 0.0, 0.20, 0.01, 0.20));
  context.update(makeInput(200000000, 0.0, 0.0, 0.20, 0.0, 0.20));
  context.update(makeInput(300000000, 0.0, 0.0, 0.05, 0.0, 0.05));
  const auto snapshot =
    context.update(makeInput(400000000, 0.0, 0.0, 0.01, 0.0, 0.01));

  EXPECT_EQ(snapshot.stage, NearGoalOdomStage::SUCCESS);
  EXPECT_EQ(snapshot.control_directive, NearGoalControlDirective::HOLD_ZERO);
}

TEST(NearGoalOdomContextTest, InvalidatesLargeGlobalYawCorrectionAfterYawLatch)
{
  auto parameters = testParameters();
  parameters.global_yaw_correction_abort_tolerance = 0.10;
  NearGoalOdomContext context(parameters);
  context.reset(17, NearGoalTransitionReason::GOAL_CHANGED);

  context.update(makeInput(100000000, 0.01, 0.0, 0.20, 0.01, 0.20));
  context.update(makeInput(200000000, 0.0, 0.0, 0.20, 0.0, 0.20));
  context.update(makeInput(300000000, 0.0, 0.0, 0.05, 0.0, 0.05));
  ASSERT_EQ(context.snapshot().stage, NearGoalOdomStage::ODOM_YAW_FINE);

  const auto snapshot =
    context.update(makeInput(400000000, 0.0, 0.0, 0.04, 0.0, 0.25));
  EXPECT_EQ(snapshot.stage, NearGoalOdomStage::INVALID);
  EXPECT_EQ(snapshot.control_directive, NearGoalControlDirective::TERMINATE);
  EXPECT_EQ(
    snapshot.transition_reason,
    NearGoalTransitionReason::GLOBAL_CORRECTION);
}

TEST(NearGoalOdomContextTest, NormalizesFineYawErrorAcrossPiBoundary)
{
  NearGoalOdomContext context(testParameters());
  context.reset(14, NearGoalTransitionReason::GOAL_CHANGED);
  constexpr double kRobotYaw = -3.131592653589793;
  constexpr double kGoalYaw = 3.141592653589793;

  auto input = makeInput(100000000, 0.01, 0.0, kRobotYaw, 0.01, 0.01);
  input.live_goal_in_odom.orientation = yawQuaternion(kGoalYaw);
  context.update(input);
  input = makeInput(200000000, 0.0, 0.0, kRobotYaw, 0.0, 0.01);
  input.live_goal_in_odom.orientation = yawQuaternion(kGoalYaw);
  context.update(input);
  input.stamp = rclcpp::Time(int64_t{300000000}, RCL_ROS_TIME);
  const auto snapshot = context.update(input);

  EXPECT_EQ(snapshot.stage, NearGoalOdomStage::ODOM_YAW_FINE);
  EXPECT_NEAR(snapshot.odom_yaw_error, 0.01, 1e-9);
}

TEST(NearGoalOdomContextTest, SanitizesInvalidNumericParameters)
{
  auto parameters = testParameters();
  parameters.entry_distance = -1.0;
  parameters.entry_confirm_cycles = 0;
  NearGoalOdomContext context(parameters);
  context.reset(15, NearGoalTransitionReason::GOAL_CHANGED);

  const auto snapshot =
    context.update(makeInput(100000000, 0.0, 0.0, 0.0, 0.0, 0.0));
  EXPECT_EQ(snapshot.stage, NearGoalOdomStage::ODOM_POSITION);
}

TEST(NearGoalOdomContextTest, ResetsLatchedStateWhenModeChanges)
{
  auto parameters = testParameters();
  NearGoalOdomContext context(parameters);
  context.reset(19, NearGoalTransitionReason::GOAL_CHANGED);
  context.update(makeInput(100000000, 0.01, 0.0, 0.20, 0.01, 0.20));
  ASSERT_TRUE(context.snapshot().position_goal_latched);

  parameters.mode = NearGoalOdomMode::CONTROL;
  context.setParameters(parameters);
  const auto snapshot = context.snapshot();
  EXPECT_EQ(snapshot.stage, NearGoalOdomStage::INACTIVE);
  EXPECT_FALSE(snapshot.position_goal_latched);
  EXPECT_FALSE(snapshot.position_reached_latched);
  EXPECT_FALSE(snapshot.yaw_goal_latched);
  EXPECT_EQ(snapshot.transition_reason, NearGoalTransitionReason::MODE_CHANGED);
  EXPECT_EQ(snapshot.goal_generation, 19u);
}

}  // namespace
}  // namespace nav2_controller
