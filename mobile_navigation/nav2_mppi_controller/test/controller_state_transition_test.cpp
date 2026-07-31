// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#include "gtest/gtest.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>

#include "nav2_mppi_controller/controller.hpp"

#include "utils/utils.hpp"

class TangentTestableMPPIController : public nav2_mppi_controller::MPPIController
{
public:
  using nav2_mppi_controller::MPPIController::computeTangentStartCommand;
  using nav2_mppi_controller::MPPIController::computeFinalGoalYawCommand;
  using nav2_mppi_controller::MPPIController::configureStartPoseInControllerFrame;
  using nav2_mppi_controller::MPPIController::configurePlanForControl;
  using nav2_mppi_controller::MPPIController::isSameNavigationGoal;
  using nav2_mppi_controller::MPPIController::makeRotationCommand;
  using nav2_mppi_controller::MPPIController::PathControlMode;
  using nav2_mppi_controller::MPPIController::shouldRunStartControl;
  using nav2_mppi_controller::MPPIController::enforceFixedCruiseYaw;
  using nav2_mppi_controller::MPPIController::enforceAutoSteeringContinuity;
  using nav2_mppi_controller::MPPIController::alignAutoTranslationWithPath;
  using nav2_mppi_controller::MPPIController::applyPersistentAutoYawProfile;
  using nav2_mppi_controller::MPPIController::liftTerminalTranslationCommand;
  using nav2_mppi_controller::MPPIController::makeAutoPathAlignedTranslation;
  using nav2_mppi_controller::MPPIController::filterAutoLinearSpeedMagnitude;
  using nav2_mppi_controller::MPPIController::filterAutoAngularVelocity;
  using nav2_mppi_controller::MPPIController::limitAutoPathDirectionChange;
  using nav2_mppi_controller::MPPIController::blendTangentTrackEntryCommand;
  using nav2_mppi_controller::MPPIController::projectAutoCommandForSteering;
  using nav2_mppi_controller::MPPIController::makeDeterministicTerminalTranslation;
  using nav2_mppi_controller::MPPIController::usesDeterministicTerminalPositionControl;

  void initializeTangentTestState()
  {
    tangent_phase_control_enabled_ = false;
    tangent_command_frame_ = "base_link";
    clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  }

  void setStartStopPhaseForTest()
  {
    tangent_phase_ = TangentPhase::START_STOP;
  }

  void setTrackPhaseForTest()
  {
    tangent_phase_ = TangentPhase::TRACK;
  }

  void setTangentTargetsForTest(double tangent_yaw, double goal_yaw)
  {
    tangent_cruise_start_pose_.orientation.z = std::sin(tangent_yaw * 0.5);
    tangent_cruise_start_pose_.orientation.w = std::cos(tangent_yaw * 0.5);
    tangent_goal_pose_.orientation.z = std::sin(goal_yaw * 0.5);
    tangent_goal_pose_.orientation.w = std::cos(goal_yaw * 0.5);
    tangent_goal_pose_.position.x = 1.0;
  }

  void setRotationDynamicsForTest()
  {
    tangent_rotate_kp_ = 1.2;
    tangent_rotate_max_speed_ = 0.4;
    tangent_rotate_min_speed_ = 0.08;
    tangent_rotate_max_accel_ = 0.5;
    tangent_start_yaw_tolerance_ = 0.05;
    tangent_stopped_angular_velocity_ = 0.03;
    control_duration_ = 0.1;
  }

  void setStartYawLookaheadForTest(double distance)
  {
    start_yaw_lookahead_distance_ = distance;
  }

  void setPathControlModeForTest(PathControlMode mode)
  {
    current_path_control_mode_ = mode;
  }

  void enableTangentStartControlForTest()
  {
    tangent_phase_control_enabled_ = true;
  }

  void rememberNavigationGoalForTest(const geometry_msgs::msg::Pose & goal)
  {
    tangent_goal_pose_ = goal;
    navigation_goal_initialized_ = true;
  }

  void configureAutoFilterWithoutCostmapForTest()
  {
    initializeTangentTestState();
    current_path_control_mode_ = PathControlMode::AUTO;
    auto_command_filter_initialized_ = true;
    last_auto_filtered_command_.linear.x = 0.20;
    last_auto_filter_robot_yaw_ = 0.0;
    auto_chassis_length_ = 0.460;
    auto_chassis_width_ = 0.345;
    auto_max_angular_speed_ = 0.45;
    auto_max_angular_acceleration_ = 0.7;
    auto_max_wheel_steer_step_ = 0.08;
    auto_steering_filter_simulate_ahead_time_ = 0.5;
    costmap_snapshot_initialized_ = false;
  }

  double startYaw() const {return tf2::getYaw(tangent_cruise_start_pose_.orientation);}
};

class StageTestableMPPIController : public nav2_mppi_controller::MPPIController
{
public:
  using nav2_mppi_controller::MPPIController::PathControlMode;

  void setApproachState(size_t substage)
  {
    active_stage_ = nav2_mppi_controller::STAGE_APPROACH;
    approach_substage_ = substage;
  }

  void setApproachTolerances(double enter, double exit)
  {
    final_yaw_enter_lon_tolerance_ = enter;
    final_yaw_enter_lat_tolerance_ = enter;
    final_yaw_exit_lon_tolerance_ = exit;
    final_yaw_exit_lat_tolerance_ = exit;
  }

  void updateApproach(double lon, double lat)
  {
    updateApproachSubstage(lon, lat);
  }

  void setApproachMode(size_t mode)
  {
    approach_mode_ = mode;
    approach_mode_locked_ = true;
  }

  void setPositionFirstYawThreshold(double threshold)
  {
    position_first_yaw_threshold_ = threshold;
  }

  void setPositionFirstLateralGeometry(double threshold, double ratio)
  {
    position_first_lateral_error_threshold_ = threshold;
    position_first_lateral_ratio_ = ratio;
  }

  void enterApproachFrom(
    size_t previous_stage, double lon_error, double lat_error, double yaw_error)
  {
    active_stage_ = nav2_mppi_controller::STAGE_APPROACH;
    updateApproachMode(
      previous_stage, nav2_mppi_controller::STAGE_APPROACH,
      lon_error, lat_error, yaw_error);
  }

  size_t approachMode() const {return approach_mode_;}
  size_t approachSubstage() const {return approach_substage_;}

  size_t selectStageForTest(double goal_distance) const
  {
    return selectStage(goal_distance);
  }

  void applySubstageProfile(
    const mppi::MppiProfileSettings & profile, size_t substage,
    float final_vx_max = 0.03f, float final_vy_max = 0.03f,
    float final_wz_max = 0.4f)
  {
    active_stage_ = nav2_mppi_controller::STAGE_APPROACH;
    approach_substage_ = substage;
    applied_profile_ = profile;
    stages_[nav2_mppi_controller::STAGE_APPROACH] = profile;
    final_yaw_vx_max_ = final_vx_max;
    final_yaw_vy_max_ = final_vy_max;
    final_yaw_wz_max_ = final_wz_max;
    applyApproachSubstageProfile();
  }

  void filterFinalYawCommand(geometry_msgs::msg::TwistStamped & command)
  {
    clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    final_yaw_lock_translation_ = true;
    final_yaw_cmd_wz_slew_rate_ = 1.2f;
    geometry_msgs::msg::Twist speed;
    smoothFinalYawCommand(command, 0.5, speed);
  }

  void applyBrakingEnvelopeForTest(
    const mppi::VelocityProfile & track,
    const mppi::VelocityProfile & approach,
    double goal_distance, double approach_distance = 1.0,
    double braking_deceleration = 0.5)
  {
    stages_[nav2_mppi_controller::STAGE_TRACK].velocity = track;
    stages_[nav2_mppi_controller::STAGE_APPROACH].velocity = approach;
    applied_profile_.velocity = track;
    approach_distance_ = approach_distance;
    approach_braking_deceleration_ = braking_deceleration;
    applyApproachBrakingEnvelope(goal_distance);
  }

  void applyTerminalGoalAxisWeightsForTest(
    const mppi::MppiProfileSettings & track,
    const mppi::MppiProfileSettings & terminal,
    double goal_distance, double terminal_distance = 0.30,
    PathControlMode mode = PathControlMode::AUTO,
    double tangent_yaw_hold_distance = 0.20)
  {
    applied_profile_ = track;
    stages_[nav2_mppi_controller::STAGE_APPROACH] = terminal;
    terminal_translation_assist_distance_ = terminal_distance;
    current_path_control_mode_ = mode;
    tangent_terminal_yaw_hold_distance_ = tangent_yaw_hold_distance;
    applyTerminalGoalAxisWeights(goal_distance);
  }

  const mppi::MppiProfileSettings & appliedProfile() const {return applied_profile_;}
};

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

TEST(ControllerStateTransitionTest, ApproachSubstageUsesPositionHysteresis)
{
  StageTestableMPPIController controller;
  controller.setApproachTolerances(0.015, 0.02);
  controller.setApproachState(nav2_mppi_controller::APPROACH_POSITIONING);

  controller.updateApproach(0.014, -0.015);
  EXPECT_EQ(controller.approachSubstage(), nav2_mppi_controller::APPROACH_FINAL_YAW);

  controller.updateApproach(0.019, -0.019);
  EXPECT_EQ(controller.approachSubstage(), nav2_mppi_controller::APPROACH_FINAL_YAW);

  controller.updateApproach(0.021, 0.0);
  EXPECT_EQ(controller.approachSubstage(), nav2_mppi_controller::APPROACH_POSITIONING);
}

TEST(ControllerStateTransitionTest, PositionMppiAlwaysUsesTrackProfile)
{
  StageTestableMPPIController controller;
  EXPECT_EQ(controller.selectStageForTest(5.0), nav2_mppi_controller::STAGE_TRACK);
  EXPECT_EQ(controller.selectStageForTest(0.5), nav2_mppi_controller::STAGE_TRACK);
  EXPECT_EQ(controller.selectStageForTest(0.021), nav2_mppi_controller::STAGE_TRACK);
}

TEST(ControllerStateTransitionTest, ApproachSubstageAppliesOmniFinalYawProfile)
{
  mppi::MppiProfileSettings profile;
  profile.velocity = {0.5f, -0.1f, 0.4f, 0.75f};
  profile.critics.goal_angle.weight = 75.0f;
  profile.critics.path_angle.weight = 2.0f;
  profile.critics.prefer_forward.weight = 5.0f;
  profile.critics.linear_velocity.weight = 10.0f;
  profile.critics.angular_velocity.weight = 5.0f;
  profile.critics.path_align.use_path_orientations = true;

  StageTestableMPPIController controller;
  controller.setApproachMode(nav2_mppi_controller::APPROACH_POSITION_FIRST);
  controller.applySubstageProfile(profile, nav2_mppi_controller::APPROACH_POSITIONING);
  // 距离尚未进入位置容差时，不得提前追最终 yaw；同时保留角速度惩罚防止原地转身。
  EXPECT_FLOAT_EQ(controller.appliedProfile().critics.goal_angle.weight, 0.0f);
  EXPECT_FLOAT_EQ(controller.appliedProfile().critics.angular_velocity.weight, 5.0f);
  EXPECT_FLOAT_EQ(controller.appliedProfile().critics.path_angle.weight, 2.0f);
  EXPECT_FALSE(controller.appliedProfile().critics.path_align.use_path_orientations);
  EXPECT_FALSE(controller.appliedProfile().critics.angular_velocity.track_goal_yaw);

  controller.applySubstageProfile(profile, nav2_mppi_controller::APPROACH_FINAL_YAW);
  EXPECT_FLOAT_EQ(controller.appliedProfile().velocity.vx_max, 0.03f);
  EXPECT_FLOAT_EQ(controller.appliedProfile().velocity.vx_min, -0.03f);
  EXPECT_FLOAT_EQ(controller.appliedProfile().velocity.vy_max, 0.03f);
  EXPECT_FLOAT_EQ(controller.appliedProfile().velocity.wz_max, 0.4f);
  EXPECT_FLOAT_EQ(controller.appliedProfile().critics.goal_angle.weight, 75.0f);
  EXPECT_FLOAT_EQ(controller.appliedProfile().critics.path_angle.weight, 0.0f);
  EXPECT_FLOAT_EQ(controller.appliedProfile().critics.prefer_forward.weight, 0.0f);
  EXPECT_FLOAT_EQ(controller.appliedProfile().critics.linear_velocity.weight, 0.0f);
  EXPECT_FLOAT_EQ(controller.appliedProfile().critics.angular_velocity.weight, 5.0f);
  EXPECT_FALSE(controller.appliedProfile().critics.path_align.use_path_orientations);
  EXPECT_TRUE(controller.appliedProfile().critics.angular_velocity.track_goal_yaw);
}

TEST(ControllerStateTransitionTest, ApproachBrakingEnvelopeIsContinuousAtBoundary)
{
  StageTestableMPPIController controller;
  const mppi::VelocityProfile track{1.0f, -0.5f, 1.0f, 0.75f};
  const mppi::VelocityProfile approach{0.25f, -0.18f, 0.25f, 0.5f};

  // 单套 profile 直接面向 2 cm 位置门槛制动；远处保持 TRACK 上限。
  controller.applyBrakingEnvelopeForTest(track, approach, 1.5);
  EXPECT_FLOAT_EQ(controller.appliedProfile().velocity.vx_max, 1.0f);
  EXPECT_FLOAT_EQ(controller.appliedProfile().velocity.vy_max, 1.0f);

  // 12 cm 处按 v²=2as 连续收紧，但不更换 critic/profile。
  controller.applyBrakingEnvelopeForTest(track, approach, 0.12);
  const float expected = std::sqrt(2.0f * 0.5f * (0.12f - 0.015f));
  EXPECT_NEAR(controller.appliedProfile().velocity.vx_max, expected, 1e-6);
  EXPECT_NEAR(controller.appliedProfile().velocity.vx_min, -expected, 1e-6);
  EXPECT_NEAR(controller.appliedProfile().velocity.vy_max, expected, 1e-6);
}

TEST(ControllerStateTransitionTest, TerminalGoalAxesPreserveOrientationMode)
{
  mppi::MppiProfileSettings track;
  track.velocity.wz_max = 0.75f;
  track.sampling_std.wz = 0.15f;
  track.critics.path_align.use_path_orientations = true;
  track.critics.goal_angle.weight = 9.0f;
  track.critics.path_angle.weight = 7.0f;
  track.critics.angular_velocity.weight = 5.0f;
  track.critics.angular_velocity.track_goal_yaw = true;
  track.critics.goal_lateral.weight = 0.0f;
  track.critics.goal_longitudinal.weight = 0.0f;

  mppi::MppiProfileSettings terminal = track;
  terminal.critics.goal_lateral.weight = 40.0f;
  terminal.critics.goal_longitudinal.weight = 25.0f;

  StageTestableMPPIController controller;
  controller.applyTerminalGoalAxisWeightsForTest(track, terminal, 0.10);

  EXPECT_FLOAT_EQ(controller.appliedProfile().velocity.wz_max, 0.75f);
  EXPECT_FLOAT_EQ(controller.appliedProfile().sampling_std.wz, 0.15f);
  EXPECT_TRUE(controller.appliedProfile().critics.path_align.use_path_orientations);
  EXPECT_FLOAT_EQ(controller.appliedProfile().critics.goal_angle.weight, 9.0f);
  EXPECT_FLOAT_EQ(controller.appliedProfile().critics.path_angle.weight, 7.0f);
  EXPECT_FLOAT_EQ(controller.appliedProfile().critics.angular_velocity.weight, 5.0f);
  EXPECT_TRUE(controller.appliedProfile().critics.angular_velocity.track_goal_yaw);
  EXPECT_FLOAT_EQ(controller.appliedProfile().critics.goal_lateral.weight, 40.0f);
  EXPECT_FLOAT_EQ(controller.appliedProfile().critics.goal_longitudinal.weight, 25.0f);
}

TEST(ControllerStateTransitionTest, TerminalGoalAxesStayOffOutsideAssistDistance)
{
  mppi::MppiProfileSettings track;
  track.velocity.wz_max = 0.75f;
  track.sampling_std.wz = 0.15f;
  track.critics.path_align.use_path_orientations = true;

  mppi::MppiProfileSettings terminal = track;
  terminal.critics.goal_lateral.weight = 40.0f;
  terminal.critics.goal_longitudinal.weight = 25.0f;

  StageTestableMPPIController controller;
  controller.applyTerminalGoalAxisWeightsForTest(track, terminal, 0.31);

  EXPECT_FLOAT_EQ(controller.appliedProfile().velocity.wz_max, 0.75f);
  EXPECT_FLOAT_EQ(controller.appliedProfile().sampling_std.wz, 0.15f);
  EXPECT_TRUE(controller.appliedProfile().critics.path_align.use_path_orientations);
  EXPECT_FLOAT_EQ(controller.appliedProfile().critics.goal_lateral.weight, 0.0f);
  EXPECT_FLOAT_EQ(controller.appliedProfile().critics.goal_longitudinal.weight, 0.0f);
}

TEST(ControllerStateTransitionTest, TangentTerminalHoldDampsSpuriousYaw)
{
  mppi::MppiProfileSettings track;
  track.velocity.wz_max = 0.75f;
  track.sampling_std.wz = 0.15f;
  track.critics.path_align.use_path_orientations = true;
  track.critics.angular_velocity.weight = 0.0f;
  track.critics.angular_velocity.track_goal_yaw = true;

  mppi::MppiProfileSettings terminal = track;
  terminal.critics.goal_lateral.weight = 40.0f;
  terminal.critics.goal_longitudinal.weight = 25.0f;
  terminal.critics.angular_velocity.weight = 15.0f;

  StageTestableMPPIController controller;
  controller.applyTerminalGoalAxisWeightsForTest(
    track, terminal, 0.10, 0.30,
    StageTestableMPPIController::PathControlMode::TANGENT, 0.20);

  // 保留 tangent 路径 yaw 评分，只把无意义的 wz 压向零。
  EXPECT_FLOAT_EQ(controller.appliedProfile().velocity.wz_max, 0.75f);
  EXPECT_FLOAT_EQ(controller.appliedProfile().sampling_std.wz, 0.15f);
  EXPECT_TRUE(controller.appliedProfile().critics.path_align.use_path_orientations);
  EXPECT_FLOAT_EQ(controller.appliedProfile().critics.angular_velocity.weight, 15.0f);
  EXPECT_FALSE(controller.appliedProfile().critics.angular_velocity.track_goal_yaw);
}

TEST(ControllerStateTransitionTest, TangentYawDampingStaysOffBeforeHoldRegion)
{
  mppi::MppiProfileSettings track;
  track.critics.angular_velocity.weight = 0.0f;
  track.critics.angular_velocity.track_goal_yaw = true;

  mppi::MppiProfileSettings terminal = track;
  terminal.critics.goal_lateral.weight = 40.0f;
  terminal.critics.goal_longitudinal.weight = 25.0f;
  terminal.critics.angular_velocity.weight = 15.0f;

  StageTestableMPPIController controller;
  controller.applyTerminalGoalAxisWeightsForTest(
    track, terminal, 0.25, 0.30,
    StageTestableMPPIController::PathControlMode::TANGENT, 0.20);

  EXPECT_FLOAT_EQ(controller.appliedProfile().critics.goal_lateral.weight, 40.0f);
  EXPECT_FLOAT_EQ(controller.appliedProfile().critics.goal_longitudinal.weight, 25.0f);
  EXPECT_FLOAT_EQ(controller.appliedProfile().critics.angular_velocity.weight, 0.0f);
  EXPECT_TRUE(controller.appliedProfile().critics.angular_velocity.track_goal_yaw);
}

TEST(ControllerStateTransitionTest, FinalYawLocksBothTranslationAxes)
{
  StageTestableMPPIController controller;
  controller.setApproachState(nav2_mppi_controller::APPROACH_FINAL_YAW);

  geometry_msgs::msg::TwistStamped command;
  command.twist.linear.x = 0.02;
  command.twist.linear.y = -0.02;
  command.twist.angular.z = 0.1;
  controller.filterFinalYawCommand(command);

  EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command.twist.linear.y, 0.0);
}

TEST(ControllerStateTransitionTest, ApproachModeLocksFromEntryGeometry)
{
  StageTestableMPPIController controller;
  controller.setPositionFirstYawThreshold(2.094);
  controller.setPositionFirstLateralGeometry(0.05, 1.0);

  controller.enterApproachFrom(nav2_mppi_controller::STAGE_TRACK, 0.4, 0.02, 0.5);
  EXPECT_EQ(controller.approachMode(), nav2_mppi_controller::APPROACH_NORMAL);

  controller.enterApproachFrom(nav2_mppi_controller::STAGE_TRACK, 0.02, -0.45, 1.5);
  EXPECT_EQ(controller.approachMode(), nav2_mppi_controller::APPROACH_POSITION_FIRST);

  controller.enterApproachFrom(nav2_mppi_controller::STAGE_TRACK, 0.4, 0.02, -2.8);
  EXPECT_EQ(controller.approachMode(), nav2_mppi_controller::APPROACH_POSITION_FIRST);
}

TEST(ControllerStateTransitionTest, DeterministicTerminalCommandUsesStableGoalDirection)
{
  TangentTestableMPPIController controller;
  geometry_msgs::msg::TwistStamped command;
  command.twist.linear.x = -0.08;
  command.twist.linear.y = 0.03;
  command.twist.angular.z = 0.12;
  mppi::models::ControlConstraints constraints{};
  constraints.vx_max = 1.0f;
  constraints.vx_min = -0.5f;
  constraints.vy = 1.0f;

  ASSERT_TRUE(
    controller.makeDeterministicTerminalTranslation(
      command, 0.0, 0.10, 0.10, 0.018,
      0.05f, 0.20f, 0.8f, constraints));

  // 输入方向背离目标，改写后必须严格沿目标方向；0.8*(0.10-0.018)=0.0656。
  EXPECT_NEAR(command.twist.linear.x, 0.0, 1e-6);
  EXPECT_NEAR(command.twist.linear.y, 0.0656, 1e-6);
  // helper 只负责平移；TANGENT/START 的 wz=0 由带碰撞检查的调用层执行。
  EXPECT_NEAR(command.twist.angular.z, 0.12, 1e-6);
}

TEST(ControllerStateTransitionTest, DeterministicTerminalCommandStartsContinuouslyFromZero)
{
  TangentTestableMPPIController controller;
  geometry_msgs::msg::TwistStamped command;
  mppi::models::ControlConstraints constraints{};
  constraints.vx_max = 1.0f;
  constraints.vx_min = -0.5f;
  constraints.vy = 1.0f;

  ASSERT_TRUE(
    controller.makeDeterministicTerminalTranslation(
      command, 0.10, 0.0, 0.10, 0.018,
      0.05f, 0.20f, 0.8f, constraints));
  EXPECT_NEAR(command.twist.linear.x, 0.0656, 1e-6);
  EXPECT_DOUBLE_EQ(command.twist.linear.y, 0.0);
}

TEST(ControllerStateTransitionTest, DeterministicTerminalControlAlsoCoversAuto)
{
  using Mode = TangentTestableMPPIController::PathControlMode;
  EXPECT_TRUE(
    TangentTestableMPPIController::usesDeterministicTerminalPositionControl(Mode::TANGENT));
  EXPECT_TRUE(
    TangentTestableMPPIController::usesDeterministicTerminalPositionControl(Mode::START));
  EXPECT_FALSE(
    TangentTestableMPPIController::usesDeterministicTerminalPositionControl(Mode::GOAL));
  EXPECT_TRUE(
    TangentTestableMPPIController::usesDeterministicTerminalPositionControl(Mode::AUTO));
  EXPECT_FALSE(
    TangentTestableMPPIController::usesDeterministicTerminalPositionControl(Mode::NONE));
}

TEST(ControllerStateTransitionTest, DeterministicTerminalSpeedChangesContinuouslyWithDistance)
{
  TangentTestableMPPIController controller;
  mppi::models::ControlConstraints constraints{};
  constraints.vx_max = 1.0f;
  constraints.vx_min = -0.5f;
  constraints.vy = 1.0f;

  geometry_msgs::msg::TwistStamped below_cap;
  geometry_msgs::msg::TwistStamped above_cap;
  ASSERT_TRUE(
    controller.makeDeterministicTerminalTranslation(
      below_cap, 0.267, 0.0, 0.267, 0.018,
      0.05f, 0.20f, 0.8f, constraints));
  ASSERT_TRUE(
    controller.makeDeterministicTerminalTranslation(
      above_cap, 0.269, 0.0, 0.269, 0.018,
      0.05f, 0.20f, 0.8f, constraints));

  EXPECT_NEAR(below_cap.twist.linear.x, 0.1992, 1e-6);
  EXPECT_NEAR(above_cap.twist.linear.x, 0.20, 1e-6);
  EXPECT_LT(above_cap.twist.linear.x - below_cap.twist.linear.x, 0.001);
}

TEST(ControllerStateTransitionTest, DeterministicTerminalCommandRespectsZeroSpeedLimit)
{
  TangentTestableMPPIController controller;
  geometry_msgs::msg::TwistStamped command;
  mppi::models::ControlConstraints constraints{};

  EXPECT_FALSE(
    controller.makeDeterministicTerminalTranslation(
      command, 0.10, 0.0, 0.10, 0.018,
      0.05f, 0.20f, 0.8f, constraints));
  EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command.twist.linear.y, 0.0);
}

TEST(ControllerStateTransitionTest, AutoYawRateIsIndependentOfLinearSpeed)
{
  geometry_msgs::msg::Twist command;
  command.linear.x = 0.10;
  command.angular.z = 0.30;
  geometry_msgs::msg::Twist previous;

  EXPECT_FALSE(
    TangentTestableMPPIController::projectAutoCommandForSteering(
      command, previous, false, 0.0, 0.0, 0.460, 0.345, 0.45, 0.08));
  EXPECT_NEAR(command.linear.x, 0.10, 1e-9);
  EXPECT_NEAR(command.angular.z, 0.30, 1e-9);
}

TEST(ControllerStateTransitionTest, AutoPureRotationIsAllowedAtZeroLinearSpeed)
{
  geometry_msgs::msg::Twist command;
  command.angular.z = 0.30;
  geometry_msgs::msg::Twist previous;

  EXPECT_FALSE(
    TangentTestableMPPIController::projectAutoCommandForSteering(
      command, previous, false, 0.0, 0.0, 0.460, 0.345, 0.45, 0.08));
  EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command.linear.y, 0.0);
  EXPECT_NEAR(command.angular.z, 0.30, 1e-9);
}

TEST(ControllerStateTransitionTest, AutoProjectionKeepsStraightWorldVelocityWhileTurning)
{
  // 10 Hz、约 0.2 rad/s 时相邻周期车身 yaw 变化约 0.02 rad。
  constexpr double current_yaw = 0.02;
  geometry_msgs::msg::Twist previous;
  previous.linear.x = 0.20;
  previous.angular.z = 0.20;
  geometry_msgs::msg::Twist command;
  // 车身转过 0.02 rad 后，本体速度必须反向旋转，世界速度才能继续沿 +X 直线。
  command.linear.x = 0.20 * std::cos(current_yaw);
  command.linear.y = -0.20 * std::sin(current_yaw);
  command.angular.z = 0.40;

  TangentTestableMPPIController::projectAutoCommandForSteering(
    command, previous, true, 0.0, current_yaw, 0.460, 0.345, 0.45, 0.08);
  const double world_x =
    std::cos(current_yaw) * command.linear.x - std::sin(current_yaw) * command.linear.y;
  const double world_y =
    std::sin(current_yaw) * command.linear.x + std::cos(current_yaw) * command.linear.y;
  EXPECT_NEAR(world_x, 0.20, 1e-6);
  EXPECT_NEAR(world_y, 0.0, 1e-6);
  EXPECT_GT(command.angular.z, previous.angular.z);
  EXPECT_LE(command.angular.z, 0.40);
}

TEST(ControllerStateTransitionTest, AutoProjectionNeverInterpolatesWorldDirection)
{
  geometry_msgs::msg::Twist previous;
  previous.linear.x = 0.20;
  geometry_msgs::msg::Twist command;
  command.linear.y = 0.20;

  TangentTestableMPPIController::projectAutoCommandForSteering(
    command, previous, true, 0.0, 0.0, 0.460, 0.345, 0.45, 0.08);
  const double filtered_direction = std::atan2(command.linear.y, command.linear.x);
  // 90° 路径突变无法由 0.08 rad 软限制连续执行时，交给底盘停车对舵；上层也
  // 不能重新生成介于 +X 和 +Y 之间的斜向速度，否则世界轨迹必然被走成弧线。
  EXPECT_NEAR(filtered_direction, M_PI_2, 1e-9);
  EXPECT_NEAR(command.linear.x, 0.0, 1e-9);
  EXPECT_GT(command.linear.y, 0.0);
}

TEST(ControllerStateTransitionTest, AutoPathReferenceKeepsMppiSpeedAndUsesLookaheadDirection)
{
  geometry_msgs::msg::Twist command;
  command.linear.x = 0.12;
  command.linear.y = -0.16;
  command.angular.z = 0.30;

  geometry_msgs::msg::Pose robot_pose;
  robot_pose.orientation = tf2::toMsg(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

  nav_msgs::msg::Path path;
  path.poses.resize(4);
  // 起点接入的 1 cm 反向短段不得成为 AUTO 世界平移方向。
  path.poses[0].pose.position.x = 0.0;
  path.poses[1].pose.position.x = -0.01;
  path.poses[2].pose.position.x = 0.50;
  path.poses[3].pose.position.x = 1.00;

  ASSERT_TRUE(
    TangentTestableMPPIController::makeAutoPathAlignedTranslation(
      command, robot_pose, path, 0.40, 0.005));
  EXPECT_NEAR(command.linear.x, 0.20, 1e-6);
  EXPECT_NEAR(command.linear.y, 0.0, 1e-6);
  EXPECT_NEAR(command.angular.z, 0.30, 1e-9);
}

TEST(ControllerStateTransitionTest, AutoPathReferenceCompensatesCurrentRobotYaw)
{
  geometry_msgs::msg::Twist command;
  command.linear.x = 0.20;

  geometry_msgs::msg::Pose robot_pose;
  tf2::Quaternion robot_orientation;
  robot_orientation.setRPY(0.0, 0.0, M_PI_2);
  robot_pose.orientation = tf2::toMsg(robot_orientation);

  nav_msgs::msg::Path path;
  path.poses.resize(2);
  path.poses[0].pose.position.x = 0.0;
  path.poses[1].pose.position.x = 1.0;

  ASSERT_TRUE(
    TangentTestableMPPIController::makeAutoPathAlignedTranslation(
      command, robot_pose, path, 0.40, 0.005));
  // 车身朝 +Y 时，要沿世界 +X 行驶，本体命令应为 -Y；合速度大小保持不变。
  EXPECT_NEAR(command.linear.x, 0.0, 1e-6);
  EXPECT_NEAR(command.linear.y, -0.20, 1e-6);
}

TEST(ControllerStateTransitionTest, AutoPathReferenceDoesNotForceStartFromZeroCommand)
{
  geometry_msgs::msg::Twist command;
  command.angular.z = 0.30;
  geometry_msgs::msg::Pose robot_pose;
  robot_pose.orientation.w = 1.0;
  nav_msgs::msg::Path path;
  path.poses.resize(2);
  path.poses[1].pose.position.x = 1.0;

  EXPECT_FALSE(
    TangentTestableMPPIController::makeAutoPathAlignedTranslation(
      command, robot_pose, path, 0.40, 0.005));
  EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command.linear.y, 0.0);
  EXPECT_NEAR(command.angular.z, 0.30, 1e-9);
}

TEST(ControllerStateTransitionTest, TangentTrackEntryKeepsFirstWheelGeometryBeforeMoving)
{
  geometry_msgs::msg::Twist entry_reference;
  entry_reference.linear.x = 0.10;
  entry_reference.angular.z = 0.05;

  geometry_msgs::msg::Twist command;
  command.linear.x = 0.20;
  command.angular.z = -0.10;

  TangentTestableMPPIController::blendTangentTrackEntryCommand(
    command, entry_reference, 0.0);
  // 当前速度大小可以随 MPPI 提升，但 wz/vx 比例保持首帧的 0.5，舵轮目标不变。
  EXPECT_NEAR(command.linear.x, 0.20, 1e-9);
  EXPECT_NEAR(command.linear.y, 0.0, 1e-9);
  EXPECT_NEAR(command.angular.z, 0.10, 1e-9);

  geometry_msgs::msg::Twist blended;
  blended.linear.x = 0.20;
  blended.angular.z = -0.10;
  TangentTestableMPPIController::blendTangentTrackEntryCommand(
    blended, entry_reference, 0.5);
  EXPECT_NEAR(blended.linear.x, 0.20, 1e-9);
  EXPECT_NEAR(blended.angular.z, 0.0, 1e-9);
}

TEST(ControllerStateTransitionTest, AutoPathDirectionChangesGraduallyInWorldFrame)
{
  geometry_msgs::msg::Twist command;
  // 机器人 yaw=0 时目标世界方向为 +0.60 rad，模拟本包约 33° 的重规划跳变。
  command.linear.x = 0.30 * std::cos(0.60);
  command.linear.y = 0.30 * std::sin(0.60);
  double selected_world_yaw = 0.0;

  EXPECT_TRUE(
    TangentTestableMPPIController::limitAutoPathDirectionChange(
      command, 0.0, 0.0, true, 0.08, 1.047, selected_world_yaw));
  EXPECT_NEAR(selected_world_yaw, 0.08, 1e-9);
  EXPECT_NEAR(std::hypot(command.linear.x, command.linear.y), 0.30, 1e-9);
  EXPECT_NEAR(std::atan2(command.linear.y, command.linear.x), 0.08, 1e-9);
}

TEST(ControllerStateTransitionTest, AutoPathDirectionKeepsEmergencyReplanImmediate)
{
  geometry_msgs::msg::Twist command;
  command.linear.y = 0.30;
  double selected_world_yaw = 0.0;

  EXPECT_FALSE(
    TangentTestableMPPIController::limitAutoPathDirectionChange(
      command, 0.0, 0.0, true, 0.08, 1.0, selected_world_yaw));
  EXPECT_NEAR(selected_world_yaw, M_PI_2, 1e-9);
  EXPECT_NEAR(command.linear.x, 0.0, 1e-9);
  EXPECT_NEAR(command.linear.y, 0.30, 1e-9);
}

TEST(ControllerStateTransitionTest, AutoProjectionReducesYawWithoutRotatingPathDirection)
{
  geometry_msgs::msg::Twist previous;
  previous.linear.x = 0.20;
  geometry_msgs::msg::Twist command;
  command.linear.x = 0.20;
  command.angular.z = 0.45;

  ASSERT_TRUE(
    TangentTestableMPPIController::projectAutoCommandForSteering(
      command, previous, true, 0.0, 0.0, 0.460, 0.345, 0.45, 0.08));
  EXPECT_GT(command.linear.x, 0.0);
  EXPECT_NEAR(command.linear.y, 0.0, 1e-9);
  EXPECT_GE(command.angular.z, 0.0);
  EXPECT_LT(command.angular.z, 0.45);
}

TEST(ControllerStateTransitionTest, AutoLinearSpeedFilterPreservesDirectionAndLimitsRise)
{
  geometry_msgs::msg::Twist previous;
  previous.linear.x = 0.20;
  geometry_msgs::msg::Twist command;
  command.linear.x = 0.30;
  command.linear.y = 0.40;

  EXPECT_TRUE(
    TangentTestableMPPIController::filterAutoLinearSpeedMagnitude(
      command, previous, true, 0.35, 0.06, 0.08));

  // 目标合速度 0.50 m/s，上一速度 0.20 m/s；低通结果 0.305 仍受单周期
  // 0.06 m/s 加速上限约束为 0.26 m/s，且 3:4 的路径方向保持不变。
  EXPECT_NEAR(std::hypot(command.linear.x, command.linear.y), 0.26, 1e-9);
  EXPECT_NEAR(command.linear.x / command.linear.y, 0.75, 1e-9);
}

TEST(ControllerStateTransitionTest, AutoLinearSpeedFilterLimitsBrakeWithoutResidualAtZero)
{
  geometry_msgs::msg::Twist previous;
  previous.linear.x = 0.40;
  geometry_msgs::msg::Twist command;
  command.linear.y = 0.10;

  EXPECT_TRUE(
    TangentTestableMPPIController::filterAutoLinearSpeedMagnitude(
      command, previous, true, 0.35, 0.06, 0.08));
  // 低通目标为 0.295 m/s，先受单周期 0.08 m/s 制动上限约束为 0.32 m/s。
  EXPECT_NEAR(command.linear.x, 0.0, 1e-9);
  EXPECT_NEAR(command.linear.y, 0.32, 1e-9);

  geometry_msgs::msg::Twist stopped;
  EXPECT_FALSE(
    TangentTestableMPPIController::filterAutoLinearSpeedMagnitude(
      stopped, previous, true, 0.35, 0.06, 0.08));
  EXPECT_DOUBLE_EQ(stopped.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(stopped.linear.y, 0.0);
}

TEST(ControllerStateTransitionTest, AutoAngularFilterCannotInstantlyReverseAcceleration)
{
  double acceleration_state = 0.70;
  const double filtered = TangentTestableMPPIController::filterAutoAngularVelocity(
    -0.40, 0.10, true, 1.0, 0.70, 3.0, 0.10, acceleration_state);

  // 目标突然反向时，角加速度每周期最多变化 jerk*dt=0.30；不会从 +0.70
  // 直接跳到 -0.70，因而不再产生旧版每帧 ±0.07 rad/s 的锯齿。
  EXPECT_NEAR(acceleration_state, 0.40, 1e-9);
  EXPECT_NEAR(filtered, 0.14, 1e-9);
}

TEST(ControllerStateTransitionTest, AutoAngularFilterStartsWithoutStaleAcceleration)
{
  double acceleration_state = 0.50;
  const double filtered = TangentTestableMPPIController::filterAutoAngularVelocity(
    0.30, 0.0, false, 0.45, 0.70, 3.0, 0.10, acceleration_state);
  EXPECT_NEAR(filtered, 0.30, 1e-9);
  EXPECT_DOUBLE_EQ(acceleration_state, 0.0);
}

TEST(ControllerStateTransitionTest, AutoSteeringFilterDoesNotChangeOtherModes)
{
  TangentTestableMPPIController controller;
  const std::array<TangentTestableMPPIController::PathControlMode, 4> modes{
    TangentTestableMPPIController::PathControlMode::TANGENT,
    TangentTestableMPPIController::PathControlMode::START,
    TangentTestableMPPIController::PathControlMode::GOAL,
    TangentTestableMPPIController::PathControlMode::NONE};
  for (const auto mode : modes) {
    controller.setPathControlModeForTest(mode);
    geometry_msgs::msg::TwistStamped command;
    command.twist.linear.x = 0.10;
    command.twist.linear.y = -0.05;
    command.twist.angular.z = 0.30;

    geometry_msgs::msg::PoseStamped robot_pose;
    geometry_msgs::msg::Twist robot_speed;
    controller.enforceAutoSteeringContinuity(command, robot_pose, robot_speed);

    EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.10);
    EXPECT_DOUBLE_EQ(command.twist.linear.y, -0.05);
    EXPECT_DOUBLE_EQ(command.twist.angular.z, 0.30);
  }
}

TEST(ControllerStateTransitionTest, AutoPathAlignmentDoesNotChangeOtherModes)
{
  TangentTestableMPPIController controller;
  const std::array<TangentTestableMPPIController::PathControlMode, 4> modes{
    TangentTestableMPPIController::PathControlMode::TANGENT,
    TangentTestableMPPIController::PathControlMode::START,
    TangentTestableMPPIController::PathControlMode::GOAL,
    TangentTestableMPPIController::PathControlMode::NONE};
  nav_msgs::msg::Path path;
  path.poses.resize(2);
  path.poses[1].pose.position.y = 1.0;
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.pose.orientation.w = 1.0;

  for (const auto mode : modes) {
    controller.setPathControlModeForTest(mode);
    geometry_msgs::msg::TwistStamped command;
    command.twist.linear.x = 0.20;
    command.twist.angular.z = 0.30;

    EXPECT_FALSE(controller.alignAutoTranslationWithPath(command, robot_pose, path));
    EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.20);
    EXPECT_DOUBLE_EQ(command.twist.linear.y, 0.0);
    EXPECT_DOUBLE_EQ(command.twist.angular.z, 0.30);
  }
}

TEST(ControllerStateTransitionTest, AutoFilterFallsBackWhenCollisionCheckUnavailable)
{
  TangentTestableMPPIController controller;
  controller.configureAutoFilterWithoutCostmapForTest();
  geometry_msgs::msg::TwistStamped command;
  command.twist.linear.y = 0.20;
  geometry_msgs::msg::PoseStamped robot_pose;
  geometry_msgs::msg::Twist robot_speed;

  controller.enforceAutoSteeringContinuity(command, robot_pose, robot_speed);

  // 没有有效代价地图快照时不得发布插值命令，必须保留 MPPI 原命令。
  EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command.twist.linear.y, 0.20);
  EXPECT_DOUBLE_EQ(command.twist.angular.z, 0.0);
}

TEST(ControllerStateTransitionTest, AutoYawProfileKeepsTaskProgressAcrossReplans)
{
  nav_msgs::msg::Path replanned_path;
  replanned_path.poses.resize(3);
  replanned_path.poses[0].pose.position.x = 0.0;
  replanned_path.poses[1].pose.position.x = 0.25;
  replanned_path.poses[2].pose.position.x = 0.50;

  TangentTestableMPPIController::applyPersistentAutoYawProfile(
    replanned_path, 0.0, M_PI_2, 0.50, 1.0, 0.20);

  const double u = 0.50 / 0.80;
  const double expected_first_yaw = M_PI_2 * u * u * (3.0 - 2.0 * u);
  // 重规划首点应继续原任务约 50 cm 的 yaw 进度，不能重新回到当前规划的起点 yaw。
  EXPECT_NEAR(tf2::getYaw(replanned_path.poses.front().pose.orientation), expected_first_yaw, 1e-6);
  EXPECT_NEAR(tf2::getYaw(replanned_path.poses.back().pose.orientation), M_PI_2, 1e-6);
}

TEST(ControllerStateTransitionTest, AutoYawProfileFinishesBeforeTerminalRegion)
{
  nav_msgs::msg::Path path;
  path.poses.resize(4);
  path.poses[0].pose.position.x = 0.0;
  path.poses[1].pose.position.x = 0.40;
  path.poses[2].pose.position.x = 0.80;
  path.poses[3].pose.position.x = 1.00;

  TangentTestableMPPIController::applyPersistentAutoYawProfile(
    path, 0.0, M_PI_2, 0.0, 1.0, 0.20);

  EXPECT_NEAR(tf2::getYaw(path.poses.front().pose.orientation), 0.0, 1e-6);
  EXPECT_NEAR(tf2::getYaw(path.poses[2].pose.orientation), M_PI_2, 1e-6);
  EXPECT_NEAR(tf2::getYaw(path.poses.back().pose.orientation), M_PI_2, 1e-6);
}

// Tests basic transition from configure->active->process->deactive->cleanup

TEST(ControllerStateTransitionTest, ControllerNotFail)
{
  const bool visualize = true;
  TestCostmapSettings costmap_settings{};

  // Node Options
  rclcpp::NodeOptions options;
  std::vector<rclcpp::Parameter> params;
  setUpControllerParams(visualize, params);
  options.parameter_overrides(params);

  auto node = getDummyNode(options);
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap_ros = getDummyCostmapRos(costmap_settings);
  costmap_ros->setRobotFootprint(getDummySquareFootprint(0.01));

  auto controller = getDummyController(node, tf_buffer, costmap_ros);

  TestPose start_pose = costmap_settings.getCenterPose();
  const double path_step = costmap_settings.resolution;
  TestPathSettings path_settings{start_pose, 8, path_step, path_step};

  // evalControl args
  auto pose = getDummyPointStamped(node, start_pose);
  auto velocity = getDummyTwist();
  auto path = getIncrementalDummyPath(node, path_settings);

  controller->setPlan(path);

  EXPECT_NO_THROW(controller->computeVelocityCommands(pose, velocity, {}));

  controller->setSpeedLimit(0.5, true);
  controller->setSpeedLimit(0.5, false);
  controller->setSpeedLimit(1.0, true);
  controller->deactivate();
  controller->cleanup();
}

TEST(ControllerStateTransitionTest, TangentMarkerContract)
{
  TangentTestableMPPIController controller;
  controller.initializeTangentTestState();

  nav_msgs::msg::Path marked_path;
  marked_path.poses.resize(5);
  marked_path.poses[0].pose.position.x = 0.0;
  marked_path.poses[1].pose.position.x = 0.0;
  marked_path.poses[2].pose.position.x = 0.5;
  marked_path.poses[3].pose.position.x = 1.0;
  marked_path.poses[4].pose.position.x = 1.0;
  marked_path.poses[4].pose.orientation.z = std::sin(0.5);
  marked_path.poses[4].pose.orientation.w = std::cos(0.5);

  nav_msgs::msg::Path cruise_path;
  EXPECT_EQ(
    controller.configurePlanForControl(marked_path, cruise_path),
    TangentTestableMPPIController::PathControlMode::TANGENT);
  ASSERT_EQ(cruise_path.poses.size(), 3u);
  EXPECT_DOUBLE_EQ(cruise_path.poses.front().pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(cruise_path.poses.back().pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(
    cruise_path.poses.back().pose.orientation.z,
    marked_path.poses.back().pose.orientation.z);
  EXPECT_DOUBLE_EQ(
    cruise_path.poses.back().pose.orientation.w,
    marked_path.poses.back().pose.orientation.w);

  nav_msgs::msg::Path ordinary_path = cruise_path;
  EXPECT_EQ(
    controller.configurePlanForControl(ordinary_path, cruise_path),
    TangentTestableMPPIController::PathControlMode::AUTO);

  nav_msgs::msg::Path start_path = ordinary_path;
  start_path.poses.insert(start_path.poses.begin(), start_path.poses.front());
  start_path.poses.insert(start_path.poses.begin(), start_path.poses.front());
  EXPECT_EQ(
    controller.configurePlanForControl(start_path, cruise_path),
    TangentTestableMPPIController::PathControlMode::START);
  ASSERT_EQ(cruise_path.poses.size(), ordinary_path.poses.size());

  nav_msgs::msg::Path goal_path = ordinary_path;
  goal_path.poses.insert(goal_path.poses.begin(), goal_path.poses.front());
  EXPECT_EQ(
    controller.configurePlanForControl(goal_path, cruise_path),
    TangentTestableMPPIController::PathControlMode::GOAL);

  nav_msgs::msg::Path none_path = ordinary_path;
  none_path.poses.push_back(none_path.poses.back());
  EXPECT_EQ(
    controller.configurePlanForControl(none_path, cruise_path),
    TangentTestableMPPIController::PathControlMode::NONE);
}

TEST(ControllerStateTransitionTest, StartAlignmentOnlyRunsForTangentAndGoal)
{
  TangentTestableMPPIController controller;
  controller.initializeTangentTestState();
  controller.enableTangentStartControlForTest();

  using Mode = TangentTestableMPPIController::PathControlMode;
  EXPECT_TRUE(controller.shouldRunStartControl(Mode::TANGENT));
  EXPECT_TRUE(controller.shouldRunStartControl(Mode::GOAL));
  EXPECT_FALSE(controller.shouldRunStartControl(Mode::AUTO));
  EXPECT_FALSE(controller.shouldRunStartControl(Mode::START));
  EXPECT_FALSE(controller.shouldRunStartControl(Mode::NONE));
}

TEST(ControllerStateTransitionTest, StartAndGoalCruiseNeverOutputAngularVelocity)
{
  TangentTestableMPPIController controller;
  geometry_msgs::msg::TwistStamped command;
  command.twist.angular.z = 0.42;

  controller.setPathControlModeForTest(
    TangentTestableMPPIController::PathControlMode::START);
  controller.enforceFixedCruiseYaw(command);
  EXPECT_DOUBLE_EQ(command.twist.angular.z, 0.0);

  command.twist.angular.z = -0.31;
  controller.setPathControlModeForTest(
    TangentTestableMPPIController::PathControlMode::GOAL);
  controller.enforceFixedCruiseYaw(command);
  EXPECT_DOUBLE_EQ(command.twist.angular.z, 0.0);
}

TEST(ControllerStateTransitionTest, TerminalTranslationLiftPreservesVectorDirection)
{
  mppi::models::ControlConstraints constraints{};
  constraints.vx_max = 1.0f;
  constraints.vx_min = -0.5f;
  constraints.vy = 1.0f;

  geometry_msgs::msg::TwistStamped command;
  command.twist.linear.x = 0.01;
  command.twist.linear.y = -0.02;
  const double original_ratio = command.twist.linear.y / command.twist.linear.x;

  EXPECT_TRUE(
    TangentTestableMPPIController::liftTerminalTranslationCommand(
      command, 1.0, -2.0, 0.05f, 0.005f, 0.5f, constraints));
  EXPECT_NEAR(std::hypot(command.twist.linear.x, command.twist.linear.y), 0.05, 1e-6);
  EXPECT_NEAR(command.twist.linear.y / command.twist.linear.x, original_ratio, 1e-6);
}

TEST(ControllerStateTransitionTest, TerminalTranslationLiftRejectsZeroAndWrongDirection)
{
  mppi::models::ControlConstraints constraints{};
  constraints.vx_max = 1.0f;
  constraints.vx_min = -0.5f;
  constraints.vy = 1.0f;

  geometry_msgs::msg::TwistStamped zero_command;
  EXPECT_FALSE(
    TangentTestableMPPIController::liftTerminalTranslationCommand(
      zero_command, 1.0, 0.0, 0.05f, 0.005f, 0.5f, constraints));

  geometry_msgs::msg::TwistStamped reverse_command;
  reverse_command.twist.linear.x = -0.02;
  EXPECT_FALSE(
    TangentTestableMPPIController::liftTerminalTranslationCommand(
      reverse_command, 1.0, 0.0, 0.05f, 0.005f, 0.5f, constraints));
  EXPECT_DOUBLE_EQ(reverse_command.twist.linear.x, -0.02);
}

TEST(ControllerStateTransitionTest, TerminalTranslationLiftRespectsActiveSpeedLimit)
{
  mppi::models::ControlConstraints constraints{};
  constraints.vx_max = 0.03f;
  constraints.vx_min = -0.015f;
  constraints.vy = 0.03f;

  geometry_msgs::msg::TwistStamped command;
  command.twist.linear.x = 0.01;
  EXPECT_TRUE(
    TangentTestableMPPIController::liftTerminalTranslationCommand(
      command, 1.0, 0.0, 0.05f, 0.005f, 0.5f, constraints));
  EXPECT_NEAR(command.twist.linear.x, 0.03, 1e-6);
}

TEST(ControllerStateTransitionTest, GoalStartYawUsesGoalOrientationNotPathTangent)
{
  TangentTestableMPPIController controller;
  controller.initializeTangentTestState();
  controller.setPathControlModeForTest(
    TangentTestableMPPIController::PathControlMode::GOAL);

  nav_msgs::msg::Path path;
  path.poses.resize(3);
  path.poses[1].pose.position.x = 0.5;
  path.poses[2].pose.position.x = 1.0;
  path.poses[2].pose.orientation.z = std::sin(0.6);
  path.poses[2].pose.orientation.w = std::cos(0.6);

  ASSERT_TRUE(controller.configureStartPoseInControllerFrame(path));
  EXPECT_NEAR(controller.startYaw(), 1.2, 1e-6);
}

TEST(ControllerStateTransitionTest, ControllerFrameStartYawUsesArcLengthLookahead)
{
  TangentTestableMPPIController controller;
  controller.initializeTangentTestState();
  controller.setStartYawLookaheadForTest(0.20);

  nav_msgs::msg::Path path;
  path.poses.resize(5);
  path.poses[0].pose.orientation.w = 1.0;
  path.poses[1].pose.position.x = 0.0;  // 重复 start 点必须跳过
  path.poses[1].pose.position.y = 0.0;
  // 两个近点错误地朝 +X；只有弧长前视到更后面的点才能得到真实 +Y 出发方向。
  path.poses[2].pose.position.x = 0.05;
  path.poses[3].pose.position.x = 0.10;
  path.poses[4].pose.position.x = 0.0;
  path.poses[4].pose.position.y = 0.20;

  ASSERT_TRUE(controller.configureStartPoseInControllerFrame(path));
  EXPECT_NEAR(controller.startYaw(), M_PI_2, 1e-6);
}

TEST(ControllerStateTransitionTest, RecoveryReplanKeepsSameNavigationTask)
{
  TangentTestableMPPIController controller;
  controller.initializeTangentTestState();

  geometry_msgs::msg::Pose goal;
  goal.position.x = 1.2;
  goal.position.y = -0.4;
  goal.orientation.z = std::sin(0.3);
  goal.orientation.w = std::cos(0.3);
  controller.rememberNavigationGoalForTest(goal);

  // 是否同一任务只由目标位姿判断，不能因恢复等待超过 reset_period 而改变。
  EXPECT_TRUE(controller.isSameNavigationGoal(goal));

  goal.position.x += 0.10;
  EXPECT_FALSE(controller.isSameNavigationGoal(goal));
}

TEST(ControllerStateTransitionTest, StartRotationCommandRampsWithoutOdomResponse)
{
  TangentTestableMPPIController controller;
  controller.initializeTangentTestState();
  controller.setRotationDynamicsForTest();

  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.pose.orientation.w = 1.0;
  geometry_msgs::msg::Twist stopped_odom;

  const auto first = controller.makeRotationCommand(1.0, robot_pose, stopped_odom, 0.05);
  const auto second = controller.makeRotationCommand(1.0, robot_pose, stopped_odom, 0.05);
  EXPECT_NEAR(first.twist.angular.z, 0.05, 1e-6);
  // 里程计仍为零也必须继续按 0.5 rad/s² 爬升，不能每周期卡回 0.05。
  EXPECT_NEAR(second.twist.angular.z, 0.10, 1e-6);
}

TEST(ControllerStateTransitionTest, FinalYawWaitsOnceThenRotatesContinuously)
{
  TangentTestableMPPIController controller;
  controller.initializeTangentTestState();
  controller.setRotationDynamicsForTest();

  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.pose.orientation.w = 1.0;

  // 刚进入 final_yaw 时若上一动作仍有角速度，先明确停稳一次。
  geometry_msgs::msg::Twist moving_speed;
  moving_speed.angular.z = 0.2;
  auto command = controller.computeFinalGoalYawCommand(robot_pose, moving_speed, -1.0);
  EXPECT_DOUBLE_EQ(command.twist.angular.z, 0.0);

  // 停稳后开始旋转并锁存状态。
  geometry_msgs::msg::Twist stopped_speed;
  command = controller.computeFinalGoalYawCommand(robot_pose, stopped_speed, -1.0);
  EXPECT_GT(command.twist.angular.z, 0.0);

  // 旋转已开始后，实测角速度超过停稳阈值也必须连续转，不能输出零脉冲。
  command = controller.computeFinalGoalYawCommand(robot_pose, moving_speed, -0.8);
  EXPECT_GT(command.twist.angular.z, 0.0);
}

TEST(ControllerStateTransitionTest, TangentAlignmentCommandsNeverTranslate)
{
  TangentTestableMPPIController controller;
  controller.initializeTangentTestState();
  controller.setTangentTargetsForTest(0.5, -1.0);

  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.pose.orientation.w = 1.0;
  geometry_msgs::msg::Twist robot_speed;

  controller.setStartStopPhaseForTest();
  auto command = controller.computeTangentStartCommand(robot_pose, robot_speed);
  ASSERT_TRUE(command.has_value());
  EXPECT_DOUBLE_EQ(command->twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command->twist.linear.y, 0.0);
  EXPECT_GT(command->twist.angular.z, 0.0);

  // 起点对齐完成进入 TRACK 后必须交还控制权，不能再在终点附近独立停车旋转。
  controller.setTrackPhaseForTest();
  command = controller.computeTangentStartCommand(robot_pose, robot_speed);
  EXPECT_FALSE(command.has_value());
}

TEST(ControllerStateTransitionTest, TangentAlignmentBrakesBeforeYawTarget)
{
  TangentTestableMPPIController controller;
  controller.initializeTangentTestState();
  controller.setRotationDynamicsForTest();

  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.pose.orientation.w = 1.0;
  geometry_msgs::msg::Twist robot_speed;
  robot_speed.angular.z = 0.2;

  // 只剩 0.01 rad 有效制动距离时，应继续同向减速，不能直接反向找 yaw。
  const auto command = controller.makeRotationCommand(0.06, robot_pose, robot_speed, 0.05);
  EXPECT_GT(command.twist.angular.z, 0.0);
  EXPECT_LT(command.twist.angular.z, robot_speed.angular.z);
  EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command.twist.linear.y, 0.0);
}
