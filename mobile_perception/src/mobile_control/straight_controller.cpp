// ----------------------------------------------------------------------------
// Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
//
// All rights reserved.
// ----------------------------------------------------------------------------

#include "mobile_perception/mobile_control/straight_controller.hpp"

#include <algorithm>
#include <cmath>

namespace mobile_perception::mobile_control {

StraightController::StraightController(const StraightControllerConfig& config)
    : config_(config),
      position_pid_(config.position_pid),
      speed_pid_(config.speed_pid),
      lateral_pid_(config.lateral_pid),
      heading_pid_(config.heading_pid) {
    config_.max_linear_velocity =
        std::isfinite(config_.max_linear_velocity) ? std::max(0.0, config_.max_linear_velocity) : 0.0;
    config_.terminal_slowdown_distance =
        std::isfinite(config_.terminal_slowdown_distance) ? std::max(0.0, config_.terminal_slowdown_distance) : 0.0;
    config_.terminal_linear_velocity =
        std::isfinite(config_.terminal_linear_velocity)
            ? std::clamp(config_.terminal_linear_velocity, 0.0, config_.max_linear_velocity)
            : 0.0;
}

StraightHeadingCheckResult checkStraightHeadingAlignment(double robot_yaw,
                                                         double goal_yaw,
                                                         double path_body_yaw,
                                                         double tolerance) {
    StraightHeadingCheckResult result;
    if (!std::isfinite(robot_yaw) || !std::isfinite(goal_yaw) || !std::isfinite(path_body_yaw) ||
        !std::isfinite(tolerance) || tolerance < 0.0) {
        return result;
    }

    const auto normalize = [](double angle) { return std::atan2(std::sin(angle), std::cos(angle)); };
    result.goal_to_robot_error = normalize(goal_yaw - robot_yaw);
    result.path_to_robot_error = normalize(path_body_yaw - robot_yaw);
    result.goal_to_path_error = normalize(goal_yaw - path_body_yaw);
    result.accepted = std::fabs(result.goal_to_robot_error) <= tolerance &&
                      std::fabs(result.path_to_robot_error) <= tolerance &&
                      std::fabs(result.goal_to_path_error) <= tolerance;
    return result;
}

double computeStraightProgress(double initial_motion, double goal_longitudinal_error, bool reverse) {
    if (!std::isfinite(initial_motion) || initial_motion < 0.0 || !std::isfinite(goal_longitudinal_error)) {
        return 0.0;
    }
    const double direction = reverse ? -1.0 : 1.0;
    const double remaining_motion = direction * goal_longitudinal_error;
    return std::clamp(initial_motion - remaining_motion, 0.0, initial_motion);
}

ControlCommand StraightController::computeCommand(const StraightControlInput& input) {
    ControlCommand command;
    if (input.dt <= 0.0 || !std::isfinite(input.dt) || !std::isfinite(input.reference_linear_velocity) ||
        !std::isfinite(input.reference_linear_acceleration) || !std::isfinite(input.tracking_longitudinal_error) ||
        (input.goal_longitudinal_error_valid && !std::isfinite(input.goal_longitudinal_error)) ||
        !std::isfinite(input.tracking_lateral_error) || !std::isfinite(input.heading_error) ||
        !std::isfinite(input.current_linear_velocity) || !std::isfinite(input.current_angular_velocity)) {
        return command;
    }

    // Distance loop: the signed x error to the sampled trajectory point
    // produces a velocity correction. While the S profile is active, its
    // velocity is a hard envelope rather than an additive speed request: a
    // position correction may slow the robot down, but must not consume the
    // distance reserved by the planned deceleration. At the zero-speed
    // endpoint the same PID is allowed to make the small signed correction
    // needed by the GoalChecker.
    command.position_pid_output = position_pid_.compute(input.tracking_longitudinal_error, input.dt);
    command.planned_linear_velocity_limit =
        std::min(std::fabs(input.reference_linear_velocity), config_.max_linear_velocity);
    double active_velocity_limit = command.planned_linear_velocity_limit;
    const bool inside_terminal_slowdown =
        input.goal_longitudinal_error_valid &&
        std::fabs(input.goal_longitudinal_error) <= config_.terminal_slowdown_distance;
    if (inside_terminal_slowdown) {
        const double unconstrained_limit =
            active_velocity_limit > 1e-9 ? active_velocity_limit : config_.max_linear_velocity;
        active_velocity_limit = std::min(unconstrained_limit, config_.terminal_linear_velocity);
        command.terminal_velocity_applied = active_velocity_limit + 1e-9 < unconstrained_limit;
    }
    const double raw_target_velocity = input.reference_linear_velocity + command.position_pid_output;
    if (command.planned_linear_velocity_limit > 1e-9) {
        command.target_linear_velocity = std::clamp(raw_target_velocity, -active_velocity_limit, active_velocity_limit);
        if (command.target_linear_velocity * input.reference_linear_velocity < 0.0) {
            command.target_linear_velocity = 0.0;
        }
    } else {
        const double endpoint_limit =
            inside_terminal_slowdown ? config_.terminal_linear_velocity : config_.max_linear_velocity;
        command.target_linear_velocity = std::clamp(command.position_pid_output, -endpoint_limit, endpoint_limit);
        command.terminal_velocity_applied =
            inside_terminal_slowdown && endpoint_limit + 1e-9 < config_.max_linear_velocity;
    }

    // Speed loop: consume the complete S-curve state. The planned acceleration
    // is feedforward (including the negative deceleration phase); PID only
    // corrects velocity tracking residuals. This matches the ControllerServer
    // trajectory -> controller contract and avoids waiting until the endpoint
    // to brake.
    const double speed_error = command.target_linear_velocity - input.current_linear_velocity;
    command.speed_pid_output = speed_pid_.compute(speed_error, input.dt);
    double acceleration_command = input.reference_linear_acceleration + command.speed_pid_output;
    const double motion_direction = std::fabs(input.reference_linear_velocity) > 1e-9
                                        ? std::copysign(1.0, input.reference_linear_velocity)
                                        : (std::fabs(command.target_linear_velocity) > 1e-9
                                               ? std::copysign(1.0, command.target_linear_velocity)
                                               : (std::fabs(input.current_linear_velocity) > 1e-9
                                                      ? std::copysign(1.0, input.current_linear_velocity)
                                                      : 1.0));
    const double minimum_acceleration =
        motion_direction >= 0.0 ? -config_.max_linear_deceleration : -config_.max_linear_acceleration;
    const double maximum_acceleration =
        motion_direction >= 0.0 ? config_.max_linear_acceleration : config_.max_linear_deceleration;
    acceleration_command = std::clamp(acceleration_command, minimum_acceleration, maximum_acceleration);

    if (!command_initialized_) {
        last_linear_command_ = input.current_linear_velocity;
        command_initialized_ = true;
    }
    const double previous_linear_command = last_linear_command_;
    double linear_output = last_linear_command_ + acceleration_command * input.dt;
    if (!std::isfinite(linear_output)) {
        linear_output = 0.0;
    }
    linear_output = std::clamp(linear_output, -config_.max_linear_velocity, config_.max_linear_velocity);
    last_linear_command_ = linear_output;
    command.linear_acceleration_command = (linear_output - previous_linear_command) / input.dt;

    // A point on the planned path is expressed in base_link. Positive y is to
    // the robot's left. Reversing inverts the steering response required to
    // drive the same lateral error toward zero.
    const double steering_error = input.reverse ? -input.tracking_lateral_error : input.tracking_lateral_error;
    const double lateral_correction = lateral_pid_.compute(steering_error, input.dt);
    const double normalized_heading_error = std::atan2(std::sin(input.heading_error), std::cos(input.heading_error));
    const double heading_correction = heading_pid_.compute(normalized_heading_error, input.dt);
    const double desired_angular_velocity = std::clamp(lateral_correction + heading_correction,
                                                       -config_.max_angular_velocity, config_.max_angular_velocity);
    const double max_angular_delta = config_.max_angular_acceleration * input.dt;
    const double angular_output =
        std::clamp(desired_angular_velocity, input.current_angular_velocity - max_angular_delta,
                   input.current_angular_velocity + max_angular_delta);

    command.linear_x = linear_output;
    command.angular_z = std::clamp(angular_output, -config_.max_angular_velocity, config_.max_angular_velocity);
    return command;
}

void StraightController::reset() {
    position_pid_.reset();
    speed_pid_.reset();
    lateral_pid_.reset();
    heading_pid_.reset();
    last_linear_command_ = 0.0;
    command_initialized_ = false;
}

}  // namespace mobile_perception::mobile_control
