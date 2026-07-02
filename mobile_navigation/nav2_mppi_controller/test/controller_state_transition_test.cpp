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

  void enterApproachFrom(size_t previous_stage, double yaw_error)
  {
    active_stage_ = nav2_mppi_controller::STAGE_APPROACH;
    updateApproachMode(previous_stage, nav2_mppi_controller::STAGE_APPROACH, yaw_error);
  }

  size_t approachMode() const
  {
    return approach_mode_;
  }

  size_t approachSubstage() const
  {
    return approach_substage_;
  }

  void applySubstageProfile(
    const mppi::MppiProfileSettings & profile, size_t substage,
    float final_vx_max = 0.03f, float final_wz_max = 0.4f)
  {
    active_stage_ = nav2_mppi_controller::STAGE_APPROACH;
    approach_substage_ = substage;
    applied_profile_ = profile;
    stages_[nav2_mppi_controller::STAGE_APPROACH] = profile;
    final_yaw_vx_max_ = final_vx_max;
    final_yaw_wz_max_ = final_wz_max;
    applyApproachSubstageProfile();
  }

  const mppi::MppiProfileSettings & appliedProfile() const
  {
    return applied_profile_;
  }
};

TEST(ControllerStateTransitionTest, ApproachSubstageUsesPositionHysteresis)
{
  TestableMPPIController controller;
  controller.setApproachTolerances(0.015, 0.02);
  controller.setApproachState(nav2_mppi_controller::APPROACH_POSITIONING);

  controller.updateApproach(0.014, -0.015);
  EXPECT_EQ(controller.approachSubstage(), nav2_mppi_controller::APPROACH_FINAL_YAW);

  controller.updateApproach(0.019, -0.019);
  EXPECT_EQ(controller.approachSubstage(), nav2_mppi_controller::APPROACH_FINAL_YAW);

  controller.updateApproach(0.021, 0.0);
  EXPECT_EQ(controller.approachSubstage(), nav2_mppi_controller::APPROACH_POSITIONING);
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
  controller.setApproachMode(nav2_mppi_controller::APPROACH_POSITION_FIRST);
  controller.applySubstageProfile(profile, nav2_mppi_controller::APPROACH_POSITIONING);
  EXPECT_FLOAT_EQ(controller.appliedProfile().critics.goal_angle.weight, 10.0f);
  EXPECT_FLOAT_EQ(controller.appliedProfile().critics.angular_velocity.weight, 0.0f);
  EXPECT_FLOAT_EQ(controller.appliedProfile().critics.path_angle.weight, 2.0f);

  controller.applySubstageProfile(profile, nav2_mppi_controller::APPROACH_FINAL_YAW);
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

  controller.enterApproachFrom(nav2_mppi_controller::STAGE_TRACK, 0.5);
  EXPECT_EQ(controller.approachMode(), nav2_mppi_controller::APPROACH_NORMAL);

  controller.enterApproachFrom(nav2_mppi_controller::STAGE_TRACK, 1.8);
  EXPECT_EQ(controller.approachMode(), nav2_mppi_controller::APPROACH_NORMAL);

  controller.enterApproachFrom(nav2_mppi_controller::STAGE_TRACK, -2.8);
  EXPECT_EQ(controller.approachMode(), nav2_mppi_controller::APPROACH_POSITION_FIRST);
}

TEST(ControllerStateTransitionTest, NormalApproachKeepsFullYawControlWhilePositioning)
{
  mppi::MppiProfileSettings profile;
  profile.velocity = {0.5f, -0.1f, 0.0f, 0.75f};
  profile.critics.goal_angle.weight = 75.0f;
  profile.critics.angular_velocity.weight = 5.0f;

  TestableMPPIController controller;
  controller.setApproachMode(nav2_mppi_controller::APPROACH_NORMAL);
  controller.applySubstageProfile(profile, nav2_mppi_controller::APPROACH_POSITIONING);

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
