// ----------------------------------------------------------------------------
// Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
//
// All rights reserved.
// ----------------------------------------------------------------------------

#pragma once

#include <algorithm>
#include <cmath>

namespace mobile_perception::mobile_control {

struct PidConfig {
    bool integrator_enable{true};
    double integrator_saturation_level{0.3};
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};
};

class PidController {
public:
    explicit PidController(const PidConfig& config = PidConfig()) : config_(config) {}

    void reset() {
        integral_ = 0.0;
        previous_error_ = 0.0;
        has_previous_error_ = false;
    }

    double compute(double error, double dt) {
        if (dt <= 0.0 || !std::isfinite(dt) || !std::isfinite(error)) {
            return 0.0;
        }

        if (config_.integrator_enable) {
            integral_ += error * dt;
            const double saturation = std::fabs(config_.integrator_saturation_level);
            integral_ = std::clamp(integral_, -saturation, saturation);
        } else {
            integral_ = 0.0;
        }

        double derivative = 0.0;
        if (has_previous_error_) {
            derivative = (error - previous_error_) / dt;
        }

        previous_error_ = error;
        has_previous_error_ = true;
        return config_.kp * error + config_.ki * integral_ + config_.kd * derivative;
    }

private:
    PidConfig config_;
    double integral_{0.0};
    double previous_error_{0.0};
    bool has_previous_error_{false};
};

struct ControlCommand {
    double linear_x{0.0};
    double angular_z{0.0};
    double target_linear_velocity{0.0};
    double planned_linear_velocity_limit{0.0};
    double position_pid_output{0.0};
    double speed_pid_output{0.0};
    double linear_acceleration_command{0.0};
    bool terminal_velocity_applied{false};
};

}  // namespace mobile_perception::mobile_control
