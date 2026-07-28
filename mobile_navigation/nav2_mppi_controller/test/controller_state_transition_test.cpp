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

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

class TestableMPPIController : public nav2_mppi_controller::MPPIController
{
public:
  void setApproachState(nav2_mppi_controller::ApproachSubstage substage)
  {
    active_stage_ = nav2_mppi_controller::Stage::APPROACH;
    approach_substage_ = substage;
  }

  void setApproachTolerances(double enter, double exit)
  {
    final_yaw_enter_lon_tolerance_ = enter;
    final_yaw_enter_lat_tolerance_ = enter;
    final_yaw_exit_lon_tolerance_ = exit;
    final_yaw_exit_lat_tolerance_ = exit;
  }

  void setFinalYawExitViolationThreshold(size_t threshold)
  {
    final_yaw_exit_violation_threshold_ = threshold;
  }

  void updateApproach(double lon, double lat)
  {
    updateApproachSubstage(lon, lat);
  }

  void setApproachMode(nav2_mppi_controller::ApproachMode mode)
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
    nav2_mppi_controller::Stage previous_stage,
    double lon_error, double lat_error, double yaw_error)
  {
    active_stage_ = nav2_mppi_controller::Stage::APPROACH;
    updateApproachMode(
      previous_stage, nav2_mppi_controller::Stage::APPROACH,
      lon_error, lat_error, yaw_error);
  }

  nav2_mppi_controller::ApproachMode approachMode() const
  {
    return approach_mode_;
  }

  nav2_mppi_controller::ApproachSubstage approachSubstage() const
  {
    return approach_substage_;
  }

  void applySubstageProfile(
    const mppi::MppiProfileSettings & profile,
    nav2_mppi_controller::ApproachSubstage substage,
    float final_vx_max = 0.03f, float final_wz_max = 0.4f)
  {
    active_stage_ = nav2_mppi_controller::Stage::APPROACH;
    approach_substage_ = substage;
    applied_profile_ = profile;
    profileFor(nav2_mppi_controller::Stage::APPROACH) = profile;
    final_yaw_vx_max_ = final_vx_max;
    final_yaw_wz_max_ = final_wz_max;
    applyApproachSubstageProfile();
  }

  const mppi::MppiProfileSettings & appliedProfile() const
  {
    return applied_profile_;
  }

  void configureFinalYawEnvelopeBrakeTest()
  {
    clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    active_stage_ = nav2_mppi_controller::Stage::APPROACH;
    approach_substage_ = nav2_mppi_controller::ApproachSubstage::FINAL_YAW;
    applied_profile_.velocity.wz_max = 0.6f;
    final_yaw_cmd_wz_slew_rate_ = 0.8f;
    final_yaw_cmd_stop_yaw_tolerance_ = 0.02f;
    final_yaw_cmd_direction_deadband_ = 0.08f;
    final_yaw_cmd_actual_decel_ = 0.35f;
    final_yaw_cmd_delay_ = 0.15f;
    final_yaw_cmd_brake_safety_margin_ = 0.004f;
    final_yaw_dynamic_brake_min_wz_ = 0.10f;
    final_yaw_internal_tolerance_ = 0.02f;
    final_yaw_envelope_brake_enter_margin_ = 0.02f;
    final_yaw_envelope_brake_exit_margin_ = 0.005f;
  }

  geometry_msgs::msg::TwistStamped runFinalYawCommand(
    double yaw_error, double actual_wz, bool reset_command_filter = false)
  {
    if (reset_command_filter) {
      final_yaw_cmd_filter_active_ = false;
    }
    geometry_msgs::msg::TwistStamped cmd;
    cmd.twist.angular.z = actual_wz;
    geometry_msgs::msg::Twist robot_speed;
    robot_speed.angular.z = actual_wz;
    smoothFinalYawCommand(cmd, yaw_error, robot_speed);
    return cmd;
  }

  bool envelopeBrakingActive() const
  {
    return final_yaw_envelope_braking_active_;
  }
};

TEST(ControllerStateTransitionTest, ApproachSubstageUsesPositionHysteresis)
{
  TestableMPPIController controller;
  controller.setApproachTolerances(0.015, 0.02);
  controller.setFinalYawExitViolationThreshold(2);
  controller.setApproachState(nav2_mppi_controller::ApproachSubstage::POSITIONING);

  controller.updateApproach(0.014, -0.015);
  EXPECT_EQ(controller.approachSubstage(), nav2_mppi_controller::ApproachSubstage::FINAL_YAW);

  controller.updateApproach(0.019, -0.019);
  EXPECT_EQ(controller.approachSubstage(), nav2_mppi_controller::ApproachSubstage::FINAL_YAW);

  controller.updateApproach(0.021, 0.0);
  EXPECT_EQ(controller.approachSubstage(), nav2_mppi_controller::ApproachSubstage::FINAL_YAW);

  controller.updateApproach(0.021, 0.0);
  EXPECT_EQ(controller.approachSubstage(), nav2_mppi_controller::ApproachSubstage::POSITIONING);
}

TEST(ControllerStateTransitionTest, EnvelopeBrakeWaitsUntilActualReturnsInsideSafeEnvelope)
{
  TestableMPPIController controller;
  controller.configureFinalYawEnvelopeBrakeTest();

  const auto braking_cmd = controller.runFinalYawCommand(-0.10, 0.30);
  EXPECT_TRUE(controller.envelopeBrakingActive());
  EXPECT_DOUBLE_EQ(braking_cmd.twist.angular.z, 0.0);

  const auto still_braking_cmd = controller.runFinalYawCommand(-0.04, 0.09, true);
  EXPECT_TRUE(controller.envelopeBrakingActive());
  EXPECT_DOUBLE_EQ(still_braking_cmd.twist.angular.z, 0.0);

  const auto resumed_cmd = controller.runFinalYawCommand(-0.04, 0.03, true);
  EXPECT_FALSE(controller.envelopeBrakingActive());
  EXPECT_GT(resumed_cmd.twist.angular.z, 0.0);
}

TEST(ControllerStateTransitionTest, ApproachSubstageAppliesSeparateProfiles)
{
  mppi::MppiProfileSettings profile;
  profile.velocity = {0.5f, -0.1f, 0.0f, 0.75f};
  profile.critics.goal_angle.weight = 75.0f;
  profile.critics.path_angle.weight = 2.0f;
  profile.critics.prefer_forward.weight = 5.0f;
  profile.critics.linear_velocity.weight = 10.0f;
  profile.critics.angular_velocity.weight = 5.0f;

  TestableMPPIController controller;
  controller.setApproachMode(nav2_mppi_controller::ApproachMode::POSITION_FIRST);
  controller.applySubstageProfile(profile, nav2_mppi_controller::ApproachSubstage::POSITIONING);
  EXPECT_FLOAT_EQ(controller.appliedProfile().critics.goal_angle.weight, 10.0f);
  EXPECT_FLOAT_EQ(controller.appliedProfile().critics.angular_velocity.weight, 0.0f);
  EXPECT_FLOAT_EQ(controller.appliedProfile().critics.path_angle.weight, 2.0f);

  controller.applySubstageProfile(profile, nav2_mppi_controller::ApproachSubstage::FINAL_YAW);
  EXPECT_FLOAT_EQ(controller.appliedProfile().velocity.vx_max, 0.03f);
  EXPECT_FLOAT_EQ(controller.appliedProfile().velocity.vx_min, -0.03f);
  EXPECT_FLOAT_EQ(controller.appliedProfile().velocity.wz_max, 0.4f);
  EXPECT_FLOAT_EQ(controller.appliedProfile().critics.goal_angle.weight, 75.0f);
  EXPECT_FLOAT_EQ(controller.appliedProfile().critics.path_angle.weight, 0.0f);
  EXPECT_FLOAT_EQ(controller.appliedProfile().critics.prefer_forward.weight, 0.0f);
  EXPECT_FLOAT_EQ(controller.appliedProfile().critics.linear_velocity.weight, 0.0f);
  EXPECT_FLOAT_EQ(controller.appliedProfile().critics.angular_velocity.weight, 5.0f);
}

TEST(ControllerStateTransitionTest, ApproachModeLocksFromEntryYawError)
{
  TestableMPPIController controller;
  controller.setPositionFirstYawThreshold(2.094);
  controller.setPositionFirstLateralGeometry(0.05, 1.0);

  controller.enterApproachFrom(nav2_mppi_controller::Stage::TRACK, 0.4, 0.02, 0.5);
  EXPECT_EQ(controller.approachMode(), nav2_mppi_controller::ApproachMode::NORMAL);

  controller.enterApproachFrom(nav2_mppi_controller::Stage::TRACK, 0.4, 0.02, 1.8);
  EXPECT_EQ(controller.approachMode(), nav2_mppi_controller::ApproachMode::NORMAL);

  controller.enterApproachFrom(nav2_mppi_controller::Stage::TRACK, 0.4, 0.02, -2.8);
  EXPECT_EQ(controller.approachMode(), nav2_mppi_controller::ApproachMode::POSITION_FIRST);
}

TEST(ControllerStateTransitionTest, ApproachModeUsesPositionFirstForLateralDominantGoal)
{
  TestableMPPIController controller;
  controller.setPositionFirstYawThreshold(2.094);
  controller.setPositionFirstLateralGeometry(0.05, 1.0);

  controller.enterApproachFrom(nav2_mppi_controller::Stage::TRACK, 0.02, -0.45, 1.5);
  EXPECT_EQ(controller.approachMode(), nav2_mppi_controller::ApproachMode::POSITION_FIRST);

  controller.enterApproachFrom(nav2_mppi_controller::Stage::TRACK, 0.30, 0.04, 1.5);
  EXPECT_EQ(controller.approachMode(), nav2_mppi_controller::ApproachMode::NORMAL);
}

TEST(ControllerStateTransitionTest, NormalApproachKeepsFullYawControlWhilePositioning)
{
  mppi::MppiProfileSettings profile;
  profile.velocity = {0.5f, -0.1f, 0.0f, 0.75f};
  profile.critics.goal_angle.weight = 75.0f;
  profile.critics.angular_velocity.weight = 5.0f;

  TestableMPPIController controller;
  controller.setApproachMode(nav2_mppi_controller::ApproachMode::NORMAL);
  controller.applySubstageProfile(profile, nav2_mppi_controller::ApproachSubstage::POSITIONING);

  EXPECT_FLOAT_EQ(controller.appliedProfile().critics.goal_angle.weight, 75.0f);
  EXPECT_FLOAT_EQ(controller.appliedProfile().critics.angular_velocity.weight, 5.0f);
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
