// ----------------------------------------------------------------------------
// Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
//
// All rights reserved.
// ----------------------------------------------------------------------------

#pragma once

#include "mobile_perception/mobile_control/pid_controller.hpp"

namespace mobile_perception::mobile_control {

struct StraightControllerConfig {
    double max_linear_velocity{0.35};
    double max_angular_velocity{1.0};
    double max_linear_acceleration{0.5};
    double max_linear_deceleration{0.5};
    double max_angular_acceleration{1.57};
    double terminal_slowdown_distance{0.05};
    double terminal_linear_velocity{0.03};
    PidConfig position_pid;
    PidConfig speed_pid;
    PidConfig lateral_pid;
    PidConfig heading_pid;
};

struct StraightControlInput {
    double reference_linear_velocity{0.0};
    double tracking_longitudinal_error{0.0};
    double tracking_lateral_error{0.0};
    // Current v1 passes goal yaw error; the future three-stage line phase will pass path heading error.
    double heading_error{0.0};
    double current_linear_velocity{0.0};
    double current_angular_velocity{0.0};
    double dt{0.0};
    bool reverse{false};
    double reference_linear_acceleration{0.0};
    double goal_longitudinal_error{0.0};
    bool goal_longitudinal_error_valid{false};
};

double computeStraightProgress(double initial_motion, double goal_longitudinal_error, bool reverse);

struct StraightHeadingCheckResult {
    bool accepted{false};
    double goal_to_robot_error{0.0};
    double path_to_robot_error{0.0};
    double goal_to_path_error{0.0};
};

StraightHeadingCheckResult checkStraightHeadingAlignment(double robot_yaw,
                                                         double goal_yaw,
                                                         double path_body_yaw,
                                                         double tolerance);

class StraightController {
public:
    explicit StraightController(const StraightControllerConfig& config);

    ControlCommand computeCommand(const StraightControlInput& input);
    void reset();

private:
    StraightControllerConfig config_;
    PidController position_pid_;
    PidController speed_pid_;
    PidController lateral_pid_;
    PidController heading_pid_;
    double last_linear_command_{0.0};
    bool command_initialized_{false};
};

}  // namespace mobile_perception::mobile_control
