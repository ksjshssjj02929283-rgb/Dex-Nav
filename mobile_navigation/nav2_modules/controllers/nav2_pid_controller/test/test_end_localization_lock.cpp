// Copyright (c) 2026 dexforce
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

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include "angles/angles.h"
#include "gtest/gtest.h"
#include "nav2_pid_controller/end_localization_lock.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace
{

constexpr double kPositionSwitchDistance = 0.05;
constexpr double kYawSwitchAngle = 0.075;
constexpr double kTestTolerance = 1e-9;
constexpr uint64_t kFirstGoalGeneration = 1;
constexpr uint64_t kSecondGoalGeneration = 2;

geometry_msgs::msg::PoseStamped makePose(
  double x, double y, double yaw, const std::string & frame = "odom")
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, yaw);
  pose.pose.orientation = tf2::toMsg(orientation);
  return pose;
}

void configureEnabledLock(
  nav2_pid_controller::EndLocalizationLock & lock,
  bool should_keep_position_lock_in_final_yaw = true)
{
  nav2_pid_controller::EndLocalizationLockConfig config;
  config.switch_distance_m = kPositionSwitchDistance;
  config.switch_angle_rad = kYawSwitchAngle;
  config.should_keep_position_lock_in_final_yaw = should_keep_position_lock_in_final_yaw;
  lock.configure(config);
  lock.reset(kFirstGoalGeneration);
}

}  // namespace

TEST(EndLocalizationLockConfig, KeepsPositionLockInFinalYawByDefault)
{
  const nav2_pid_controller::EndLocalizationLockConfig config;
  EXPECT_TRUE(config.should_keep_position_lock_in_final_yaw);
}

TEST(EndLocalizationLock, PositionLatchPreservesOdomTargetAcrossSameGoalUpdates)
{
  nav2_pid_controller::EndLocalizationLock lock;
  configureEnabledLock(lock);
  lock.setLiveGoal(makePose(1.0, 0.0, 0.2), kFirstGoalGeneration);

  const auto latched = lock.updatePositionReference(makePose(0.96, 0.0, 0.0));
  ASSERT_TRUE(latched.position_goal_latched);
  EXPECT_EQ(latched.stage, nav2_core::EndLocalizationStage::ODOM_POSITION);
  EXPECT_NEAR(latched.effective_goal.pose.position.x, 1.0, kTestTolerance);
  // 锁存发生的同一周期必须立即给出真实误差，不能短暂输出默认零值。
  EXPECT_NEAR(latched.effective_position_error, 0.04, kTestTolerance);

  // 同目标重规划或map修正改变实时目标时，Odom位置参考保持不动。
  lock.setLiveGoal(makePose(1.1, 0.0, 0.25), kFirstGoalGeneration);
  const auto updated = lock.updatePositionReference(makePose(0.97, 0.0, 0.0));
  ASSERT_TRUE(updated.position_goal_latched);
  EXPECT_NEAR(updated.live_goal.pose.position.x, 1.1, kTestTolerance);
  EXPECT_NEAR(updated.effective_goal.pose.position.x, 1.0, kTestTolerance);
  EXPECT_NEAR(updated.position_correction_after_latch, 0.1, kTestTolerance);
  EXPECT_EQ(updated.revision, latched.revision);
}

TEST(EndLocalizationLock, FinalYawUsesLivePositionAndLatchesOnlyYawNearGoal)
{
  nav2_pid_controller::EndLocalizationLock lock;
  configureEnabledLock(lock, false);
  lock.setLiveGoal(makePose(1.0, 0.0, 0.2), kFirstGoalGeneration);
  ASSERT_TRUE(
    lock.updatePositionReference(makePose(0.96, 0.0, 0.0)).position_goal_latched);

  const auto coarse = lock.updateYawReference(makePose(0.96, 0.0, 0.0), true);
  EXPECT_FALSE(coarse.position_goal_latched);
  ASSERT_FALSE(coarse.yaw_goal_latched);
  EXPECT_EQ(coarse.stage, nav2_core::EndLocalizationStage::MAP_YAW_COARSE);
  EXPECT_NEAR(coarse.effective_goal.pose.position.x, 1.0, kTestTolerance);
  EXPECT_NEAR(tf2::getYaw(coarse.effective_goal.pose.orientation), 0.2, kTestTolerance);

  // 最终转角的后续控制周期仍处于5 cm门限内，也不能重新锁存目标位置。
  const auto next_cycle = lock.updatePositionReference(makePose(0.96, 0.0, 0.0));
  EXPECT_FALSE(next_cycle.position_goal_latched);
  EXPECT_EQ(next_cycle.stage, nav2_core::EndLocalizationStage::MAP_YAW_COARSE);

  lock.setLiveGoal(makePose(1.1, 0.0, 0.07), kFirstGoalGeneration);
  const auto fine = lock.updateYawReference(makePose(0.96, 0.0, 0.0), true);
  EXPECT_FALSE(fine.position_goal_latched);
  ASSERT_TRUE(fine.yaw_goal_latched);
  EXPECT_EQ(fine.stage, nav2_core::EndLocalizationStage::ODOM_YAW_FINE);
  EXPECT_NEAR(fine.effective_goal.pose.position.x, 1.1, kTestTolerance);
  EXPECT_NEAR(fine.effective_yaw_error, 0.07, kTestTolerance);

  lock.setLiveGoal(makePose(1.2, 0.0, 0.16), kFirstGoalGeneration);
  const auto corrected = lock.updateYawReference(makePose(0.96, 0.0, 0.0), true);
  EXPECT_FALSE(corrected.position_goal_latched);
  EXPECT_NEAR(corrected.effective_goal.pose.position.x, 1.2, kTestTolerance);
  EXPECT_NEAR(tf2::getYaw(corrected.effective_goal.pose.orientation), 0.07, kTestTolerance);
  EXPECT_NEAR(corrected.yaw_correction_after_latch, 0.09, kTestTolerance);
}

TEST(EndLocalizationLock, FinalYawKeepsOdomPositionWhenConfigured)
{
  nav2_pid_controller::EndLocalizationLock lock;
  configureEnabledLock(lock, true);
  lock.setLiveGoal(makePose(1.0, 0.0, 0.2), kFirstGoalGeneration);
  ASSERT_TRUE(
    lock.updatePositionReference(makePose(0.96, 0.0, 0.0)).position_goal_latched);

  // Map目标变化后进入最终转角，位置仍使用进入前锁存的Odom参考。
  lock.setLiveGoal(makePose(1.1, 0.0, 0.2), kFirstGoalGeneration);
  const auto coarse = lock.updateYawReference(makePose(0.96, 0.0, 0.0), true);
  ASSERT_TRUE(coarse.position_goal_latched);
  EXPECT_FALSE(coarse.yaw_goal_latched);
  EXPECT_EQ(coarse.stage, nav2_core::EndLocalizationStage::MAP_YAW_COARSE);
  EXPECT_NEAR(coarse.live_goal.pose.position.x, 1.1, kTestTolerance);
  EXPECT_NEAR(coarse.effective_goal.pose.position.x, 1.0, kTestTolerance);

  const auto next_cycle = lock.updatePositionReference(makePose(0.97, 0.0, 0.0));
  ASSERT_TRUE(next_cycle.position_goal_latched);
  EXPECT_NEAR(next_cycle.effective_goal.pose.position.x, 1.0, kTestTolerance);

  lock.setLiveGoal(makePose(1.2, 0.0, 0.07), kFirstGoalGeneration);
  const auto fine = lock.updateYawReference(makePose(0.97, 0.0, 0.0), true);
  EXPECT_TRUE(fine.position_goal_latched);
  EXPECT_TRUE(fine.yaw_goal_latched);
  EXPECT_EQ(fine.stage, nav2_core::EndLocalizationStage::ODOM_YAW_FINE);
  EXPECT_NEAR(fine.effective_goal.pose.position.x, 1.0, kTestTolerance);
  EXPECT_NEAR(tf2::getYaw(fine.effective_goal.pose.orientation), 0.07, kTestTolerance);
}

TEST(EndLocalizationLock, LeavingFinalYawClearsYawThenPositionRelatchesNextCycle)
{
  nav2_pid_controller::EndLocalizationLock lock;
  configureEnabledLock(lock, false);
  lock.setLiveGoal(makePose(1.0, 0.0, 0.07), kFirstGoalGeneration);
  lock.updatePositionReference(makePose(0.96, 0.0, 0.0));
  ASSERT_TRUE(lock.updateYawReference(makePose(0.96, 0.0, 0.0), true).yaw_goal_latched);

  lock.setLiveGoal(makePose(1.0, 0.0, 0.15), kFirstGoalGeneration);
  const auto fallback = lock.updateYawReference(makePose(0.96, 0.0, 0.0), false);
  EXPECT_FALSE(fallback.position_goal_latched);
  EXPECT_FALSE(fallback.yaw_goal_latched);
  EXPECT_EQ(fallback.stage, nav2_core::EndLocalizationStage::INACTIVE);
  EXPECT_NEAR(fallback.effective_goal.pose.position.x, 1.0, kTestTolerance);
  EXPECT_NEAR(tf2::getYaw(fallback.effective_goal.pose.orientation), 0.15, kTestTolerance);

  const auto approaching = lock.updatePositionReference(makePose(0.96, 0.0, 0.0));
  EXPECT_TRUE(approaching.position_goal_latched);
  EXPECT_EQ(approaching.stage, nav2_core::EndLocalizationStage::ODOM_POSITION);
}

TEST(EndLocalizationLock, NewGoalGenerationDropsPreviousLatch)
{
  nav2_pid_controller::EndLocalizationLock lock;
  configureEnabledLock(lock);
  lock.setLiveGoal(makePose(1.0, 0.0, 0.0), kFirstGoalGeneration);
  ASSERT_TRUE(lock.updatePositionReference(makePose(0.96, 0.0, 0.0)).position_goal_latched);

  lock.setLiveGoal(makePose(2.0, 0.0, 0.0), kSecondGoalGeneration);
  const auto new_goal = lock.updatePositionReference(makePose(0.96, 0.0, 0.0));
  EXPECT_EQ(new_goal.goal_generation, kSecondGoalGeneration);
  EXPECT_FALSE(new_goal.position_goal_latched);
  EXPECT_FALSE(new_goal.yaw_goal_latched);
  EXPECT_NEAR(new_goal.effective_goal.pose.position.x, 2.0, kTestTolerance);
}

TEST(EndLocalizationLock, YawThresholdHandlesPiBoundary)
{
  nav2_pid_controller::EndLocalizationLock lock;
  configureEnabledLock(lock);
  const double robot_yaw = M_PI - 0.02;
  const double goal_yaw = -M_PI + 0.02;
  lock.setLiveGoal(makePose(0.0, 0.0, goal_yaw), kFirstGoalGeneration);

  const auto snapshot = lock.updateYawReference(
    makePose(0.0, 0.0, robot_yaw), true);
  ASSERT_TRUE(snapshot.yaw_goal_latched);
  EXPECT_NEAR(
    snapshot.effective_yaw_error,
    angles::shortest_angular_distance(robot_yaw, goal_yaw),
    kTestTolerance);
}

TEST(EndLocalizationLock, RejectsMismatchedFrames)
{
  nav2_pid_controller::EndLocalizationLock lock;
  configureEnabledLock(lock);
  lock.setLiveGoal(makePose(1.0, 0.0, 0.0, "odom"), kFirstGoalGeneration);
  const auto snapshot = lock.updatePositionReference(makePose(0.96, 0.0, 0.0, "map"));
  EXPECT_FALSE(snapshot.effective_goal_valid);
  EXPECT_FALSE(snapshot.position_goal_latched);
}

TEST(EndLocalizationLock, DisabledConfigurationPreservesLegacyPath)
{
  nav2_pid_controller::EndLocalizationLock lock;
  lock.configure(nav2_pid_controller::EndLocalizationLockConfig());
  lock.reset(kFirstGoalGeneration);
  lock.setLiveGoal(makePose(1.0, 0.0, 0.0), kFirstGoalGeneration);

  const auto snapshot = lock.updatePositionReference(makePose(0.99, 0.0, 0.0));
  EXPECT_FALSE(snapshot.enabled);
  EXPECT_FALSE(snapshot.effective_goal_valid);
  EXPECT_FALSE(snapshot.position_goal_latched);
  EXPECT_FALSE(snapshot.yaw_goal_latched);
}

TEST(EndLocalizationLock, InvalidThresholdsAreDisabled)
{
  nav2_pid_controller::EndLocalizationLock lock;
  nav2_pid_controller::EndLocalizationLockConfig config;
  config.switch_distance_m = -kPositionSwitchDistance;
  config.switch_angle_rad = std::numeric_limits<double>::quiet_NaN();
  lock.configure(config);
  lock.reset(kFirstGoalGeneration);
  lock.setLiveGoal(makePose(1.0, 0.0, 0.0), kFirstGoalGeneration);

  const auto snapshot = lock.updatePositionReference(makePose(0.99, 0.0, 0.0));
  EXPECT_FALSE(snapshot.enabled);
  EXPECT_FALSE(snapshot.position_goal_latched);
}
