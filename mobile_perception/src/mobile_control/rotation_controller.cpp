// ----------------------------------------------------------------------------
// Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
//
// All rights reserved.
// ----------------------------------------------------------------------------

#include "mobile_perception/mobile_control/rotation_controller.hpp"

#include <algorithm>
#include <cmath>

namespace mobile_perception::mobile_control {

double unwrapRotationError(double wrapped_error, double previous_unwrapped_error) {
    if (!std::isfinite(wrapped_error) || !std::isfinite(previous_unwrapped_error)) {
        return 0.0;
    }
    return previous_unwrapped_error + std::atan2(std::sin(wrapped_error - previous_unwrapped_error),
                                                 std::cos(wrapped_error - previous_unwrapped_error));
}

RotationController::RotationController(const RotationControllerConfig& config)
    : config_(config), angular_velocity_pid_(config.angular_velocity_pid) {
    config_.yaw_kp = std::isfinite(config_.yaw_kp) ? std::max(0.0, config_.yaw_kp) : 0.0;
    config_.yaw_tolerance = std::isfinite(config_.yaw_tolerance) ? std::max(0.0, config_.yaw_tolerance) : 0.0;
    config_.min_angular_velocity =
        std::isfinite(config_.min_angular_velocity) ? std::max(0.0, config_.min_angular_velocity) : 0.0;
    config_.terminal_slowdown_angle = std::isfinite(config_.terminal_slowdown_angle)
                                          ? std::max(config_.yaw_tolerance, config_.terminal_slowdown_angle)
                                          : config_.yaw_tolerance;
    config_.terminal_angular_velocity =
        std::isfinite(config_.terminal_angular_velocity) ? std::max(0.0, config_.terminal_angular_velocity) : 0.0;
    config_.max_angular_velocity =
        std::isfinite(config_.max_angular_velocity) ? std::max(0.0, config_.max_angular_velocity) : 0.0;
    config_.min_angular_velocity = std::min(config_.min_angular_velocity, config_.max_angular_velocity);
    config_.terminal_angular_velocity =
        std::clamp(config_.terminal_angular_velocity, config_.min_angular_velocity, config_.max_angular_velocity);
    config_.max_angular_acceleration =
        std::isfinite(config_.max_angular_acceleration) ? std::max(0.0, config_.max_angular_acceleration) : 0.0;
}

RotationControlCommand RotationController::computeCommand(const RotationControlInput& input) {
    RotationControlCommand command;
    if (!std::isfinite(input.reference_angular_velocity) || !std::isfinite(input.reference_angular_acceleration) ||
        !std::isfinite(input.tracking_yaw_error) || !std::isfinite(input.goal_yaw_error) ||
        !std::isfinite(input.current_angular_velocity) || !std::isfinite(input.dt) || input.dt <= 0.0) {
        return command;
    }

    const double normalized_tracking_error =
        std::atan2(std::sin(input.tracking_yaw_error), std::cos(input.tracking_yaw_error));
    // The Action node unwraps goal_yaw_error against the previous control
    // cycle. Wrapping it again here would make exact 180-degree goals switch
    // direction at the +/-pi branch.
    const double goal_error = input.goal_yaw_error;
    const double raw_outer_loop_target =
        std::clamp(input.reference_angular_velocity + config_.yaw_kp * normalized_tracking_error,
                   -config_.max_angular_velocity, config_.max_angular_velocity);

    // The S-curve velocity is the distance-feasible envelope, not an additive
    // baseline that the yaw loop may exceed. Sampling this envelope by actual
    // angular progress makes its acceleration and deceleration phases follow
    // remaining angle instead of wall time.
    command.planned_angular_velocity_limit =
        std::min(std::fabs(input.reference_angular_velocity), config_.max_angular_velocity);
    double planned_limited_target = raw_outer_loop_target;
    if (command.planned_angular_velocity_limit > 1e-9) {
        planned_limited_target = std::clamp(raw_outer_loop_target, -command.planned_angular_velocity_limit,
                                            command.planned_angular_velocity_limit);
        if (planned_limited_target * input.reference_angular_velocity < 0.0) {
            planned_limited_target = 0.0;
        }
    } else {
        planned_limited_target =
            std::clamp(config_.yaw_kp * goal_error, -config_.max_angular_velocity, config_.max_angular_velocity);
    }

    // Independent physical stopping envelope protects against localization or
    // tracking transients. The earlier jerk-limited deceleration still comes
    // from the S profile; this is only the final safety bound.
    command.braking_angular_velocity_limit = std::min(
        config_.max_angular_velocity, std::sqrt(2.0 * config_.max_angular_acceleration * std::fabs(goal_error)));
    const double envelope_limited_target = std::clamp(planned_limited_target, -command.braking_angular_velocity_limit,
                                                      command.braking_angular_velocity_limit);
    command.braking_limited = std::fabs(envelope_limited_target - planned_limited_target) > 1e-12;
    const bool inside_terminal_slowdown = std::fabs(goal_error) <= config_.terminal_slowdown_angle;

    if (std::fabs(goal_error) <= config_.yaw_tolerance) {
        command.target_angular_velocity = 0.0;
        command.braking_limited = true;
    } else if (inside_terminal_slowdown) {
        const double terminal_limit =
            std::min(config_.terminal_angular_velocity, command.braking_angular_velocity_limit);
        command.target_angular_velocity = std::clamp(envelope_limited_target, -terminal_limit, terminal_limit);
        if (command.target_angular_velocity * goal_error < 0.0) {
            command.target_angular_velocity = 0.0;
        }
        command.terminal_velocity_applied =
            std::fabs(command.target_angular_velocity - envelope_limited_target) > 1e-12;
    } else if (goal_error > 0.0) {
        command.target_angular_velocity = std::max(0.0, envelope_limited_target);
    } else {
        command.target_angular_velocity = std::min(0.0, envelope_limited_target);
    }

    if (!inside_terminal_slowdown && std::fabs(goal_error) > config_.yaw_tolerance &&
        std::fabs(command.target_angular_velocity) < config_.min_angular_velocity) {
        command.target_angular_velocity =
            std::copysign(std::min(config_.min_angular_velocity, command.braking_angular_velocity_limit), goal_error);
        command.minimum_velocity_applied = true;
    }
    command.braking_limited =
        command.braking_limited || std::fabs(command.target_angular_velocity - planned_limited_target) > 1e-12;

    const double angular_velocity_error = command.target_angular_velocity - input.current_angular_velocity;
    command.angular_velocity_pid_output = angular_velocity_pid_.compute(angular_velocity_error, input.dt);

    double feedforward_acceleration = input.reference_angular_acceleration;
    const bool target_overrides_profile =
        command.terminal_velocity_applied || command.minimum_velocity_applied ||
        command.braking_angular_velocity_limit + 1e-9 < command.planned_angular_velocity_limit ||
        command.planned_angular_velocity_limit <= 1e-9;
    if (target_overrides_profile) {
        feedforward_acceleration =
            std::clamp((command.target_angular_velocity - last_command_angular_velocity_) / input.dt,
                       -config_.max_angular_acceleration, config_.max_angular_acceleration);
    }
    const double requested_acceleration = feedforward_acceleration + command.angular_velocity_pid_output;
    command.angular_acceleration_command =
        std::clamp(requested_acceleration, -config_.max_angular_acceleration, config_.max_angular_acceleration);

    // The inner PID follows the same feedforward-plus-feedback acceleration
    // semantics as the control module speed loop. Integrate once from the last
    // command and keep measured angular velocity as feedback only.
    const double max_delta = config_.max_angular_acceleration * input.dt;
    command.angular_z = last_command_angular_velocity_ + command.angular_acceleration_command * input.dt;
    command.angular_z = std::clamp(command.angular_z, -config_.max_angular_velocity, config_.max_angular_velocity);

    // The target chassis has an observed angular command dead zone around
    // 0.02 rad/s. Outside the terminal slowdown zone, reach the effective
    // floor through the same acceleration-bounded interval instead of waiting
    // for a tiny PID acceleration to integrate for several seconds.
    if (!inside_terminal_slowdown && std::fabs(goal_error) > config_.yaw_tolerance &&
        command.angular_z * goal_error >= 0.0 && std::fabs(command.angular_z) < config_.min_angular_velocity) {
        const double effective_command = std::copysign(config_.min_angular_velocity, goal_error);
        command.angular_z = std::clamp(effective_command, last_command_angular_velocity_ - max_delta,
                                       last_command_angular_velocity_ + max_delta);
        command.minimum_velocity_applied = true;
    }
    command.angular_acceleration_command = (command.angular_z - last_command_angular_velocity_) / input.dt;
    last_command_angular_velocity_ = command.angular_z;
    return command;
}

void RotationController::reset(double initial_command_angular_velocity) {
    angular_velocity_pid_.reset();
    last_command_angular_velocity_ =
        std::isfinite(initial_command_angular_velocity)
            ? std::clamp(initial_command_angular_velocity, -config_.max_angular_velocity, config_.max_angular_velocity)
            : 0.0;
}

}  // namespace mobile_perception::mobile_control
