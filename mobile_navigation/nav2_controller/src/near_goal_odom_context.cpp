// Copyright (c) 2026 Dexory
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "near_goal_odom_context_impl.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "angles/angles.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace nav2_controller
{
namespace
{

double poseDistance(
  const geometry_msgs::msg::Pose & lhs,
  const geometry_msgs::msg::Pose & rhs)
{
  return std::hypot(
    lhs.position.x - rhs.position.x,
    lhs.position.y - rhs.position.y);
}

NearGoalOdomParameters sanitizeParameters(NearGoalOdomParameters parameters)
{
  parameters.entry_distance = std::max(0.0, parameters.entry_distance);
  parameters.entry_confirm_cycles = std::max(1, parameters.entry_confirm_cycles);
  parameters.position_lon_tolerance = std::max(0.0, parameters.position_lon_tolerance);
  parameters.position_lat_tolerance = std::max(0.0, parameters.position_lat_tolerance);
  parameters.position_confirm_cycles = std::max(1, parameters.position_confirm_cycles);
  parameters.fine_yaw_entry_error = std::max(0.0, parameters.fine_yaw_entry_error);
  parameters.fine_yaw_entry_confirm_cycles =
    std::max(1, parameters.fine_yaw_entry_confirm_cycles);
  parameters.yaw_latch_max_error_jump =
    std::max(0.0, parameters.yaw_latch_max_error_jump);
  parameters.final_yaw_tolerance = std::max(0.0, parameters.final_yaw_tolerance);
  parameters.stop_confirm_cycles = std::max(1, parameters.stop_confirm_cycles);
  parameters.odom_position_drift_tolerance =
    std::max(0.0, parameters.odom_position_drift_tolerance);
  parameters.position_hold_lon_tolerance =
    std::max(0.0, parameters.position_hold_lon_tolerance);
  parameters.position_hold_lat_tolerance =
    std::max(0.0, parameters.position_hold_lat_tolerance);
  parameters.global_position_abort_tolerance =
    std::max(0.0, parameters.global_position_abort_tolerance);
  parameters.global_yaw_correction_abort_tolerance =
    std::max(0.0, parameters.global_yaw_correction_abort_tolerance);
  parameters.odom_jump_position_tolerance =
    std::max(0.0, parameters.odom_jump_position_tolerance);
  parameters.odom_jump_yaw_tolerance =
    std::max(0.0, parameters.odom_jump_yaw_tolerance);
  parameters.tf_failure_timeout = std::max(0.0, parameters.tf_failure_timeout);
  parameters.trans_stopped_velocity = std::max(0.0, parameters.trans_stopped_velocity);
  parameters.rot_stopped_velocity = std::max(0.0, parameters.rot_stopped_velocity);
  return parameters;
}

}  // namespace

NearGoalOdomContext::NearGoalOdomContext(const NearGoalOdomParameters & parameters)
: parameters_(sanitizeParameters(parameters))
{
  snapshot_.mode = parameters_.mode;
}

void NearGoalOdomContext::setParameters(const NearGoalOdomParameters & parameters)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto sanitized_parameters = sanitizeParameters(parameters);
  const bool mode_changed = parameters_.mode != sanitized_parameters.mode;
  parameters_ = sanitized_parameters;
  if (mode_changed) {
    resetStateLocked(snapshot_.goal_generation, NearGoalTransitionReason::MODE_CHANGED);
  }
  snapshot_.mode = parameters_.mode;
}

void NearGoalOdomContext::reset(
  uint64_t goal_generation,
  NearGoalTransitionReason reason)
{
  std::lock_guard<std::mutex> lock(mutex_);
  resetStateLocked(goal_generation, reason);
}

void NearGoalOdomContext::resetStateLocked(
  uint64_t goal_generation,
  NearGoalTransitionReason reason)
{
  const auto mode = parameters_.mode;
  NearGoalOdomSnapshot reset_snapshot;
  snapshot_ = std::move(reset_snapshot);
  snapshot_.goal_generation = goal_generation;
  snapshot_.mode = mode;
  snapshot_.transition_reason = reason;
  entry_confirm_count_ = 0;
  position_confirm_count_ = 0;
  fine_yaw_confirm_count_ = 0;
  stop_confirm_count_ = 0;
  yaw_latch_failure_count_ = 0;
  braking_to_position_ = false;
  previous_odom_valid_ = false;
  previous_map_to_odom_valid_ = false;
  tf_failure_active_ = false;
  tf_failure_start_ = rclcpp::Time{};
}

void NearGoalOdomContext::transitionToLocked(
  NearGoalOdomStage stage,
  NearGoalTransitionReason reason)
{
  if (!transitionAllowedLocked(stage)) {
    snapshot_.stage = NearGoalOdomStage::INVALID;
    snapshot_.transition_reason = NearGoalTransitionReason::ILLEGAL_TRANSITION;
    return;
  }
  snapshot_.stage = stage;
  snapshot_.transition_reason = reason;
}

bool NearGoalOdomContext::transitionAllowedLocked(NearGoalOdomStage stage) const
{
  // 只允许方案定义的单向推进和受控回退；意外转换统一收敛到INVALID。
  if (stage == NearGoalOdomStage::INVALID) {
    return snapshot_.stage != NearGoalOdomStage::INVALID;
  }

  switch (snapshot_.stage) {
    case NearGoalOdomStage::INACTIVE:
      return stage == NearGoalOdomStage::ODOM_POSITION;
    case NearGoalOdomStage::ODOM_POSITION:
      return stage == NearGoalOdomStage::MAP_YAW_COARSE;
    case NearGoalOdomStage::MAP_YAW_COARSE:
      return
        stage == NearGoalOdomStage::ODOM_POSITION ||
        stage == NearGoalOdomStage::ODOM_YAW_FINE;
    case NearGoalOdomStage::ODOM_YAW_FINE:
      return
        stage == NearGoalOdomStage::ODOM_POSITION ||
        stage == NearGoalOdomStage::STOP_CONFIRM;
    case NearGoalOdomStage::STOP_CONFIRM:
      return
        stage == NearGoalOdomStage::ODOM_POSITION ||
        stage == NearGoalOdomStage::ODOM_YAW_FINE ||
        stage == NearGoalOdomStage::SUCCESS;
    case NearGoalOdomStage::SUCCESS:
    case NearGoalOdomStage::INVALID:
      return false;
  }
  return false;
}

void NearGoalOdomContext::setControlDirectiveLocked(NearGoalControlDirective directive)
{
  if (snapshot_.control_directive != directive) {
    snapshot_.control_directive = directive;
    ++snapshot_.control_revision;
  }
}

bool NearGoalOdomContext::robotStoppedLocked() const
{
  return
    std::abs(snapshot_.actual_vx) <= parameters_.trans_stopped_velocity &&
    std::abs(snapshot_.actual_wz) <= parameters_.rot_stopped_velocity;
}

bool NearGoalOdomContext::positionHoldValidLocked() const
{
  return
    snapshot_.odom_drift_since_position_reached <=
    parameters_.odom_position_drift_tolerance &&
    std::abs(snapshot_.odom_lon_error) <= parameters_.position_hold_lon_tolerance &&
    std::abs(snapshot_.odom_lat_error) <= parameters_.position_hold_lat_tolerance;
}

bool NearGoalOdomContext::odomContinuousLocked(const NearGoalOdomInput & input)
{
  if (!previous_odom_valid_) {
    return true;
  }

  const double dt = (input.stamp - previous_stamp_).seconds();
  if (dt < 0.0) {
    return false;
  }

  const double position_delta =
    poseDistance(input.robot_in_odom, previous_robot_in_odom_);
  const double yaw_delta = std::abs(angles::shortest_angular_distance(
      tf2::getYaw(previous_robot_in_odom_.orientation),
      tf2::getYaw(input.robot_in_odom.orientation)));
  const double allowed_position = std::max(
    parameters_.odom_jump_position_tolerance,
    2.0 * (std::abs(input.actual_vx) + 0.10) * dt);
  const double allowed_yaw = std::max(
    parameters_.odom_jump_yaw_tolerance,
    2.0 * (std::abs(input.actual_wz) + 0.20) * dt);
  return position_delta <= allowed_position && yaw_delta <= allowed_yaw;
}

void NearGoalOdomContext::updateErrorsLocked(const NearGoalOdomInput & input)
{
  snapshot_.live_map_position_error = input.live_map_position_error;
  snapshot_.live_map_lon_error = input.live_map_lon_error;
  snapshot_.live_map_lat_error = input.live_map_lat_error;
  snapshot_.live_map_yaw_error = input.live_map_yaw_error;
  snapshot_.actual_vx = input.actual_vx;
  snapshot_.actual_wz = input.actual_wz;
  snapshot_.tf_age = input.tf_age;
  snapshot_.snapshot_stamp = input.stamp;

  if (previous_map_to_odom_valid_) {
    snapshot_.map_to_odom_delta_x =
      input.map_to_odom_x - previous_map_to_odom_x_;
    snapshot_.map_to_odom_delta_y =
      input.map_to_odom_y - previous_map_to_odom_y_;
    snapshot_.map_to_odom_delta_yaw = angles::shortest_angular_distance(
      previous_map_to_odom_yaw_, input.map_to_odom_yaw);
  } else {
    snapshot_.map_to_odom_delta_x = 0.0;
    snapshot_.map_to_odom_delta_y = 0.0;
    snapshot_.map_to_odom_delta_yaw = 0.0;
  }

  if (snapshot_.position_goal_latched) {
    updateOdomPositionErrorsLocked(input.robot_in_odom);
  }

  if (snapshot_.yaw_goal_latched) {
    snapshot_.odom_yaw_error = angles::shortest_angular_distance(
      snapshot_.yaw_goal_in_odom,
      tf2::getYaw(input.robot_in_odom.orientation));
  }

  if (snapshot_.position_reached_latched) {
    snapshot_.odom_drift_since_position_reached = poseDistance(
      input.robot_in_odom, snapshot_.position_reached_pose_in_odom);
  }
}

void NearGoalOdomContext::updateOdomPositionErrorsLocked(
  const geometry_msgs::msg::Pose & robot_in_odom)
{
  const double goal_yaw = tf2::getYaw(snapshot_.goal_in_odom.orientation);
  const double dx =
    robot_in_odom.position.x - snapshot_.goal_in_odom.position.x;
  const double dy =
    robot_in_odom.position.y - snapshot_.goal_in_odom.position.y;
  snapshot_.odom_lon_error =
    std::cos(goal_yaw) * dx + std::sin(goal_yaw) * dy;
  snapshot_.odom_lat_error =
    -std::sin(goal_yaw) * dx + std::cos(goal_yaw) * dy;
}

NearGoalOdomSnapshot NearGoalOdomContext::update(const NearGoalOdomInput & input)
{
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.mode = parameters_.mode;
  snapshot_.transition_reason = NearGoalTransitionReason::NONE;

  if (parameters_.mode == NearGoalOdomMode::DISABLED) {
    snapshot_.stage = NearGoalOdomStage::INACTIVE;
    setControlDirectiveLocked(NearGoalControlDirective::NORMAL);
    return snapshot_;
  }

  snapshot_.tf_valid = input.tf_valid;
  snapshot_.odom_continuous = false;
  snapshot_.actual_vx = input.actual_vx;
  snapshot_.actual_wz = input.actual_wz;
  snapshot_.tf_age = input.tf_age;
  snapshot_.snapshot_stamp = input.stamp;

  if (!input.tf_valid) {
    snapshot_.global_sanity_ok = false;
    if (snapshot_.stage == NearGoalOdomStage::INACTIVE) {
      entry_confirm_count_ = 0;
      previous_odom_valid_ = false;
      previous_map_to_odom_valid_ = false;
      tf_failure_active_ = false;
      tf_failure_start_ = rclcpp::Time{};
      setControlDirectiveLocked(NearGoalControlDirective::NORMAL);
      return snapshot_;
    }
    setControlDirectiveLocked(NearGoalControlDirective::HOLD_ZERO);
    snapshot_.transition_reason = NearGoalTransitionReason::TF_FAILURE;
    if (!tf_failure_active_) {
      tf_failure_active_ = true;
      tf_failure_start_ = input.evaluation_stamp;
    } else if (
      (input.evaluation_stamp - tf_failure_start_).seconds() >
      parameters_.tf_failure_timeout)
    {
      transitionToLocked(NearGoalOdomStage::INVALID, NearGoalTransitionReason::TF_FAILURE);
      setControlDirectiveLocked(NearGoalControlDirective::TERMINATE);
    }
    return snapshot_;
  }
  snapshot_.odom_continuous = odomContinuousLocked(input);
  updateErrorsLocked(input);
  tf_failure_active_ = false;
  tf_failure_start_ = rclcpp::Time(0, 0, input.stamp.get_clock_type());

  if (!snapshot_.odom_continuous) {
    snapshot_.global_sanity_ok = false;
    if (snapshot_.stage == NearGoalOdomStage::INACTIVE) {
      entry_confirm_count_ = 0;
      previous_robot_in_odom_ = input.robot_in_odom;
      previous_stamp_ = input.stamp;
      previous_odom_valid_ = true;
      previous_map_to_odom_x_ = input.map_to_odom_x;
      previous_map_to_odom_y_ = input.map_to_odom_y;
      previous_map_to_odom_yaw_ = input.map_to_odom_yaw;
      previous_map_to_odom_valid_ = true;
      setControlDirectiveLocked(NearGoalControlDirective::NORMAL);
      return snapshot_;
    }
    transitionToLocked(NearGoalOdomStage::INVALID, NearGoalTransitionReason::ODOM_JUMP);
    setControlDirectiveLocked(NearGoalControlDirective::TERMINATE);
    return snapshot_;
  }

  bool global_sanity_ok =
    input.live_map_position_error <= parameters_.global_position_abort_tolerance ||
    snapshot_.stage == NearGoalOdomStage::INACTIVE;
  if (snapshot_.stage == NearGoalOdomStage::MAP_YAW_COARSE) {
    global_sanity_ok =
      global_sanity_ok &&
      std::abs(snapshot_.map_to_odom_delta_yaw) <=
      parameters_.global_yaw_correction_abort_tolerance;
  } else if (
    snapshot_.stage == NearGoalOdomStage::ODOM_YAW_FINE ||
    snapshot_.stage == NearGoalOdomStage::STOP_CONFIRM ||
    snapshot_.stage == NearGoalOdomStage::SUCCESS)
  {
    const double yaw_difference = angles::normalize_angle(
      input.live_map_yaw_error - snapshot_.odom_yaw_error);
    snapshot_.global_yaw_consistency_delta = angles::normalize_angle(
      yaw_difference - yaw_consistency_at_latch_);
    global_sanity_ok =
      global_sanity_ok &&
      std::abs(snapshot_.global_yaw_consistency_delta) <=
      parameters_.global_yaw_correction_abort_tolerance;
  }
  snapshot_.global_sanity_ok = global_sanity_ok;
  if (!global_sanity_ok) {
    transitionToLocked(
      NearGoalOdomStage::INVALID, NearGoalTransitionReason::GLOBAL_CORRECTION);
    setControlDirectiveLocked(NearGoalControlDirective::TERMINATE);
    return snapshot_;
  }

  if (braking_to_position_) {
    setControlDirectiveLocked(NearGoalControlDirective::HOLD_ZERO);
    if (robotStoppedLocked()) {
      snapshot_.position_reached_latched = false;
      snapshot_.position_reached_pose_in_odom = geometry_msgs::msg::Pose{};
      snapshot_.odom_drift_since_position_reached = 0.0;
      snapshot_.yaw_goal_latched = false;
      snapshot_.yaw_goal_in_odom = 0.0;
      snapshot_.odom_yaw_error = 0.0;
      braking_to_position_ = false;
      position_confirm_count_ = 0;
      fine_yaw_confirm_count_ = 0;
      transitionToLocked(
        NearGoalOdomStage::ODOM_POSITION,
        NearGoalTransitionReason::POSITION_DRIFT);
      setControlDirectiveLocked(NearGoalControlDirective::NORMAL);
    }
  } else {
    switch (snapshot_.stage) {
      case NearGoalOdomStage::INACTIVE:
        setControlDirectiveLocked(NearGoalControlDirective::NORMAL);
        if (input.live_map_position_error <= parameters_.entry_distance) {
          ++entry_confirm_count_;
        } else {
          entry_confirm_count_ = 0;
        }
        if (entry_confirm_count_ >= parameters_.entry_confirm_cycles) {
          snapshot_.goal_in_odom = input.live_goal_in_odom;
          snapshot_.position_goal_latched = true;
          updateOdomPositionErrorsLocked(input.robot_in_odom);
          transitionToLocked(
            NearGoalOdomStage::ODOM_POSITION,
            NearGoalTransitionReason::ENTRY_STABLE);
        }
        break;

      case NearGoalOdomStage::ODOM_POSITION:
        setControlDirectiveLocked(NearGoalControlDirective::NORMAL);
        if (
          std::abs(snapshot_.odom_lon_error) <= parameters_.position_lon_tolerance &&
          std::abs(snapshot_.odom_lat_error) <= parameters_.position_lat_tolerance)
        {
          ++position_confirm_count_;
        } else {
          position_confirm_count_ = 0;
        }
        if (position_confirm_count_ >= parameters_.position_confirm_cycles) {
          snapshot_.position_reached_latched = true;
          snapshot_.position_reached_pose_in_odom = input.robot_in_odom;
          transitionToLocked(
            NearGoalOdomStage::MAP_YAW_COARSE,
            NearGoalTransitionReason::POSITION_STABLE);
        }
        break;

      case NearGoalOdomStage::MAP_YAW_COARSE:
        setControlDirectiveLocked(NearGoalControlDirective::NORMAL);
        if (!positionHoldValidLocked()) {
          braking_to_position_ = true;
          setControlDirectiveLocked(NearGoalControlDirective::HOLD_ZERO);
          snapshot_.transition_reason = NearGoalTransitionReason::POSITION_DRIFT;
          break;
        }
        if (std::abs(input.live_map_yaw_error) <= parameters_.fine_yaw_entry_error) {
          ++fine_yaw_confirm_count_;
        } else {
          fine_yaw_confirm_count_ = 0;
        }
        if (fine_yaw_confirm_count_ >= parameters_.fine_yaw_entry_confirm_cycles) {
          snapshot_.yaw_error_before_latch = input.live_map_yaw_error;
          snapshot_.yaw_goal_in_odom = tf2::getYaw(input.live_goal_in_odom.orientation);
          snapshot_.yaw_error_after_latch = angles::shortest_angular_distance(
            snapshot_.yaw_goal_in_odom,
            tf2::getYaw(input.robot_in_odom.orientation));
          const double latch_jump = std::abs(angles::normalize_angle(
              snapshot_.yaw_error_before_latch - snapshot_.yaw_error_after_latch));
          if (latch_jump <= parameters_.yaw_latch_max_error_jump) {
            snapshot_.yaw_goal_latched = true;
            snapshot_.odom_yaw_error = snapshot_.yaw_error_after_latch;
            yaw_consistency_at_latch_ = angles::normalize_angle(
              input.live_map_yaw_error - snapshot_.odom_yaw_error);
            yaw_latch_failure_count_ = 0;
            transitionToLocked(
              NearGoalOdomStage::ODOM_YAW_FINE,
              NearGoalTransitionReason::FINE_YAW_STABLE);
          } else {
            ++yaw_latch_failure_count_;
            fine_yaw_confirm_count_ = 0;
            setControlDirectiveLocked(NearGoalControlDirective::HOLD_ZERO);
            snapshot_.transition_reason =
              NearGoalTransitionReason::YAW_LATCH_DISCONTINUITY;
            if (yaw_latch_failure_count_ >= 2) {
              transitionToLocked(
                NearGoalOdomStage::INVALID,
                NearGoalTransitionReason::YAW_LATCH_DISCONTINUITY);
              setControlDirectiveLocked(NearGoalControlDirective::TERMINATE);
            }
          }
        }
        break;

      case NearGoalOdomStage::ODOM_YAW_FINE:
        setControlDirectiveLocked(NearGoalControlDirective::NORMAL);
        if (!positionHoldValidLocked()) {
          braking_to_position_ = true;
          setControlDirectiveLocked(NearGoalControlDirective::HOLD_ZERO);
          snapshot_.transition_reason = NearGoalTransitionReason::POSITION_DRIFT;
        } else if (
          std::abs(snapshot_.odom_yaw_error) <= parameters_.final_yaw_tolerance &&
          robotStoppedLocked())
        {
          stop_confirm_count_ = 1;
          transitionToLocked(
            NearGoalOdomStage::STOP_CONFIRM,
            NearGoalTransitionReason::STOP_STABLE);
          setControlDirectiveLocked(NearGoalControlDirective::HOLD_ZERO);
          if (stop_confirm_count_ >= parameters_.stop_confirm_cycles) {
            transitionToLocked(
              NearGoalOdomStage::SUCCESS,
              NearGoalTransitionReason::STOP_STABLE);
          }
        }
        break;

      case NearGoalOdomStage::STOP_CONFIRM:
        setControlDirectiveLocked(NearGoalControlDirective::HOLD_ZERO);
        if (!positionHoldValidLocked()) {
          braking_to_position_ = true;
          stop_confirm_count_ = 0;
          snapshot_.transition_reason = NearGoalTransitionReason::POSITION_DRIFT;
        } else if (
          std::abs(snapshot_.odom_yaw_error) <= parameters_.final_yaw_tolerance &&
          robotStoppedLocked())
        {
          ++stop_confirm_count_;
          if (stop_confirm_count_ >= parameters_.stop_confirm_cycles) {
            transitionToLocked(
              NearGoalOdomStage::SUCCESS,
              NearGoalTransitionReason::STOP_STABLE);
          }
        } else {
          stop_confirm_count_ = 0;
          if (robotStoppedLocked()) {
            transitionToLocked(
              NearGoalOdomStage::ODOM_YAW_FINE,
              NearGoalTransitionReason::FINE_YAW_STABLE);
            setControlDirectiveLocked(NearGoalControlDirective::NORMAL);
          }
        }
        break;

      case NearGoalOdomStage::SUCCESS:
        setControlDirectiveLocked(NearGoalControlDirective::HOLD_ZERO);
        break;

      case NearGoalOdomStage::INVALID:
        setControlDirectiveLocked(NearGoalControlDirective::TERMINATE);
        break;
    }
  }

  previous_robot_in_odom_ = input.robot_in_odom;
  previous_stamp_ = input.stamp;
  previous_odom_valid_ = true;
  previous_map_to_odom_x_ = input.map_to_odom_x;
  previous_map_to_odom_y_ = input.map_to_odom_y;
  previous_map_to_odom_yaw_ = input.map_to_odom_yaw;
  previous_map_to_odom_valid_ = true;
  return snapshot_;
}

NearGoalOdomSnapshot NearGoalOdomContext::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_;
}

}  // namespace nav2_controller
