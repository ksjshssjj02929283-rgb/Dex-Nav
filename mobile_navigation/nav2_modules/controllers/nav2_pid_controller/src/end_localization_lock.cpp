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

#include "nav2_pid_controller/end_localization_lock.hpp"

#include <cmath>

#include "angles/angles.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace nav2_pid_controller
{

void EndLocalizationLock::configure(const EndLocalizationLockConfig & config)
{
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;
  // 独立状态机也做边界保护，避免绕过PID参数入口时带入非法阈值。
  if (!std::isfinite(config_.switch_distance_m) || config_.switch_distance_m < 0.0) {
    config_.switch_distance_m = 0.0;
  }
  if (!std::isfinite(config_.switch_angle_rad) || config_.switch_angle_rad < 0.0) {
    config_.switch_angle_rad = 0.0;
  }
  resetLocked(snapshot_.goal_generation);
}

void EndLocalizationLock::reset(uint64_t goal_generation)
{
  std::lock_guard<std::mutex> lock(mutex_);
  resetLocked(goal_generation);
}

void EndLocalizationLock::resetLocked(uint64_t goal_generation)
{
  snapshot_ = nav2_core::EndLocalizationSnapshot();
  snapshot_.goal_generation = goal_generation;
  snapshot_.enabled =
    config_.switch_distance_m > 0.0 || config_.switch_angle_rad > 0.0;
  locked_position_ = geometry_msgs::msg::Point();
  locked_orientation_ = geometry_msgs::msg::Quaternion();
  previous_final_yaw_control_ = false;
}

void EndLocalizationLock::setLiveGoal(
  const geometry_msgs::msg::PoseStamped & live_goal,
  uint64_t goal_generation)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (goal_generation != snapshot_.goal_generation) {
    // generation变化代表新目标，必须在接收新数据前丢弃上一目标的锁存。
    resetLocked(goal_generation);
  }
  snapshot_.live_goal = live_goal;
  snapshot_.live_goal_valid = !live_goal.header.frame_id.empty();
  // 新实时目标尚未结合本周期机器人位姿重算，不能把上一周期的有效标志继续对外暴露。
  snapshot_.effective_goal_valid = false;
}

bool EndLocalizationLock::inputValidLocked(
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  if (!snapshot_.enabled || !snapshot_.live_goal_valid ||
    robot_pose.header.frame_id.empty() ||
    robot_pose.header.frame_id != snapshot_.live_goal.header.frame_id)
  {
    return false;
  }

  const auto & goal = snapshot_.live_goal.pose;
  const auto & robot = robot_pose.pose;
  return
    std::isfinite(goal.position.x) && std::isfinite(goal.position.y) &&
    std::isfinite(robot.position.x) && std::isfinite(robot.position.y) &&
    std::isfinite(tf2::getYaw(goal.orientation)) &&
    std::isfinite(tf2::getYaw(robot.orientation));
}

void EndLocalizationLock::tryLatchPositionLocked(
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  if (snapshot_.position_goal_latched || config_.switch_distance_m <= 0.0) {
    return;
  }

  const double dx =
    snapshot_.live_goal.pose.position.x - robot_pose.pose.position.x;
  const double dy =
    snapshot_.live_goal.pose.position.y - robot_pose.pose.position.y;
  if (std::hypot(dx, dy) > config_.switch_distance_m) {
    return;
  }

  snapshot_.position_goal_latched = true;
  locked_position_ = snapshot_.live_goal.pose.position;
  ++snapshot_.revision;
}

void EndLocalizationLock::releasePositionLatchLocked()
{
  if (!snapshot_.position_goal_latched) {
    return;
  }

  snapshot_.position_goal_latched = false;
  locked_position_ = geometry_msgs::msg::Point();
  ++snapshot_.revision;
}

void EndLocalizationLock::rebuildEffectiveGoalLocked(
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  snapshot_.effective_goal = snapshot_.live_goal;
  if (snapshot_.position_goal_latched) {
    snapshot_.effective_goal.pose.position = locked_position_;
  }
  if (snapshot_.yaw_goal_latched) {
    snapshot_.effective_goal.pose.orientation = locked_orientation_;
  }

  const auto & live = snapshot_.live_goal.pose;
  const auto & effective = snapshot_.effective_goal.pose;
  const auto & robot = robot_pose.pose;
  snapshot_.live_position_error = std::hypot(
    live.position.x - robot.position.x,
    live.position.y - robot.position.y);
  snapshot_.effective_position_error = std::hypot(
    effective.position.x - robot.position.x,
    effective.position.y - robot.position.y);
  snapshot_.live_yaw_error = angles::shortest_angular_distance(
    tf2::getYaw(robot.orientation), tf2::getYaw(live.orientation));
  snapshot_.effective_yaw_error = angles::shortest_angular_distance(
    tf2::getYaw(robot.orientation), tf2::getYaw(effective.orientation));

  snapshot_.position_correction_after_latch = snapshot_.position_goal_latched ?
    std::hypot(
    live.position.x - locked_position_.x,
    live.position.y - locked_position_.y) : 0.0;
  snapshot_.yaw_correction_after_latch = snapshot_.yaw_goal_latched ?
    angles::shortest_angular_distance(
    tf2::getYaw(locked_orientation_), tf2::getYaw(live.orientation)) : 0.0;
  snapshot_.effective_goal_valid = true;
}

nav2_core::EndLocalizationSnapshot EndLocalizationLock::updatePositionReference(
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!inputValidLocked(robot_pose)) {
    snapshot_.effective_goal_valid = false;
    return snapshot_;
  }

  if (previous_final_yaw_control_) {
    // 最终转角期间只保持进入阶段前已有的锁存，不在转角过程中重新触发位置锁存。
    if (!config_.should_keep_position_lock_in_final_yaw) {
      releasePositionLatchLocked();
    }
    snapshot_.stage = snapshot_.yaw_goal_latched ?
      nav2_core::EndLocalizationStage::ODOM_YAW_FINE :
      nav2_core::EndLocalizationStage::MAP_YAW_COARSE;
  } else {
    tryLatchPositionLocked(robot_pose);
    snapshot_.stage = snapshot_.position_goal_latched ?
      nav2_core::EndLocalizationStage::ODOM_POSITION :
      nav2_core::EndLocalizationStage::INACTIVE;
  }

  rebuildEffectiveGoalLocked(robot_pose);
  return snapshot_;
}

nav2_core::EndLocalizationSnapshot EndLocalizationLock::updateYawReference(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  bool final_yaw_control)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!inputValidLocked(robot_pose)) {
    snapshot_.effective_goal_valid = false;
    return snapshot_;
  }

  if (!final_yaw_control) {
    const bool leaving_final_yaw = previous_final_yaw_control_;
    // 从最终转角退回位置调整时重新以实时map航向粗调，防止沿用旧的精调锁存。
    if (leaving_final_yaw && snapshot_.yaw_goal_latched) {
      snapshot_.yaw_goal_latched = false;
      locked_orientation_ = geometry_msgs::msg::Quaternion();
      ++snapshot_.revision;
    }
    previous_final_yaw_control_ = false;
    // 关闭保留开关时，退回位置调整后由下一周期按门限重新锁存；开启时沿用原锁存。
    // 本周期不重新触发锁存，避免目标点和速度误差计算完成后再改变位置参考。
    snapshot_.stage = snapshot_.position_goal_latched ?
      nav2_core::EndLocalizationStage::ODOM_POSITION :
      nav2_core::EndLocalizationStage::INACTIVE;
    rebuildEffectiveGoalLocked(robot_pose);
    return snapshot_;
  }

  previous_final_yaw_control_ = true;
  // 开关只决定已有位置锁存是否跨越最终转角；位置与航向锁存仍独立维护。
  if (!config_.should_keep_position_lock_in_final_yaw) {
    releasePositionLatchLocked();
  }
  const double live_yaw_error = angles::shortest_angular_distance(
    tf2::getYaw(robot_pose.pose.orientation),
    tf2::getYaw(snapshot_.live_goal.pose.orientation));
  if (!snapshot_.yaw_goal_latched && config_.switch_angle_rad > 0.0 &&
    std::abs(live_yaw_error) <= config_.switch_angle_rad)
  {
    snapshot_.yaw_goal_latched = true;
    locked_orientation_ = snapshot_.live_goal.pose.orientation;
    ++snapshot_.revision;
  }

  snapshot_.stage = snapshot_.yaw_goal_latched ?
    nav2_core::EndLocalizationStage::ODOM_YAW_FINE :
    nav2_core::EndLocalizationStage::MAP_YAW_COARSE;
  // 锁存发生后立即重算同周期误差，不能把默认零值发布给控制器。
  rebuildEffectiveGoalLocked(robot_pose);
  return snapshot_;
}

nav2_core::EndLocalizationSnapshot EndLocalizationLock::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_;
}

}  // namespace nav2_pid_controller
