// ----------------------------------------------------------------------------
// Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
//
// All rights reserved.
// ----------------------------------------------------------------------------

#pragma once

#include "mobile_perception/mobile_control/pid_controller.hpp"

namespace mobile_perception::mobile_control {

double unwrapRotationError(double wrapped_error, double previous_unwrapped_error);

struct RotationControllerConfig {
    double yaw_kp{2.0};
    double yaw_tolerance{0.008};
    double min_angular_velocity{0.02};
    double terminal_slowdown_angle{0.30};
    double terminal_angular_velocity{0.01};
    double max_angular_velocity{0.75};
    double max_angular_acceleration{1.0};
    PidConfig angular_velocity_pid;
};

struct RotationControlInput {
    double reference_angular_velocity{0.0};
    double reference_angular_acceleration{0.0};
    double tracking_yaw_error{0.0};
    double goal_yaw_error{0.0};
    double current_angular_velocity{0.0};
    double dt{0.0};
};

struct RotationControlCommand {
    double angular_z{0.0};
    double target_angular_velocity{0.0};
    double planned_angular_velocity_limit{0.0};
    double braking_angular_velocity_limit{0.0};
    double angular_velocity_pid_output{0.0};
    double angular_acceleration_command{0.0};
    bool braking_limited{false};
    bool minimum_velocity_applied{false};
    bool terminal_velocity_applied{false};
};

class RotationController {
public:
    explicit RotationController(const RotationControllerConfig& config);

    RotationControlCommand computeCommand(const RotationControlInput& input);
    void reset(double initial_command_angular_velocity = 0.0);

private:
    RotationControllerConfig config_;
    PidController angular_velocity_pid_;
    double last_command_angular_velocity_{0.0};
};

}  // namespace mobile_perception::mobile_control
