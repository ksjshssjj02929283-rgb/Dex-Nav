// ----------------------------------------------------------------------------
// Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
//
// All rights reserved.
// ----------------------------------------------------------------------------

#include "mobile_perception/mobile_control/goal_checker.hpp"

#include <algorithm>
#include <cmath>

namespace mobile_perception::mobile_control {

namespace {

double nonNegativeFiniteOrZero(double value) { return std::isfinite(value) ? std::max(0.0, value) : 0.0; }

}  // namespace

PreciseGoalChecker::PreciseGoalChecker(const GoalCheckerConfig& config) : config_(config) {
    config_.longitudinal_tolerance = nonNegativeFiniteOrZero(config_.longitudinal_tolerance);
    config_.lateral_tolerance = nonNegativeFiniteOrZero(config_.lateral_tolerance);
    config_.yaw_tolerance = nonNegativeFiniteOrZero(config_.yaw_tolerance);
    config_.stopped_linear_velocity = nonNegativeFiniteOrZero(config_.stopped_linear_velocity);
    config_.stopped_angular_velocity = nonNegativeFiniteOrZero(config_.stopped_angular_velocity);
    config_.confirm_duration = nonNegativeFiniteOrZero(config_.confirm_duration);
}

GoalCheckResult PreciseGoalChecker::update(const GoalCheckInput& input) {
    GoalCheckResult result;
    const bool mode_valid = input.mode == GoalCheckMode::Translation || input.mode == GoalCheckMode::Rotation;
    result.position_required = input.mode == GoalCheckMode::Translation;
    if (!std::isfinite(input.longitudinal_error) || !std::isfinite(input.lateral_error) ||
        !std::isfinite(input.yaw_error) || !std::isfinite(input.linear_velocity) ||
        !std::isfinite(input.angular_velocity) || !std::isfinite(input.dt) || input.dt <= 0.0 || !mode_valid) {
        stable_duration_ = 0.0;
        return result;
    }

    const double normalized_yaw_error = std::atan2(std::sin(input.yaw_error), std::cos(input.yaw_error));
    result.longitudinal_reached = std::fabs(input.longitudinal_error) <= config_.longitudinal_tolerance;
    result.lateral_reached = std::fabs(input.lateral_error) <= config_.lateral_tolerance;
    result.position_reached = result.longitudinal_reached && result.lateral_reached;
    result.yaw_reached = std::fabs(normalized_yaw_error) <= config_.yaw_tolerance;
    result.pose_reached = result.yaw_reached && (!result.position_required || result.position_reached);
    result.speed_reached = std::fabs(input.linear_velocity) <= config_.stopped_linear_velocity &&
                           std::fabs(input.angular_velocity) <= config_.stopped_angular_velocity;

    if (result.pose_reached && result.speed_reached) {
        stable_duration_ += input.dt;
    } else {
        stable_duration_ = 0.0;
    }

    result.stable_duration = stable_duration_;
    result.goal_reached =
        result.pose_reached && result.speed_reached && stable_duration_ + 1e-9 >= config_.confirm_duration;
    return result;
}

GoalControlState selectGoalControlState(const GoalCheckResult& result) {
    if (result.goal_reached) {
        return GoalControlState::Succeeded;
    }
    if (result.pose_reached && result.speed_reached) {
        return GoalControlState::HoldZero;
    }
    return GoalControlState::TrackTrajectory;
}

void PreciseGoalChecker::reset() { stable_duration_ = 0.0; }

const GoalCheckerConfig& PreciseGoalChecker::config() const { return config_; }

}  // namespace mobile_perception::mobile_control
