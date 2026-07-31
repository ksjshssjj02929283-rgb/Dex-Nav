// ----------------------------------------------------------------------------
// Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
//
// All rights reserved.
// ----------------------------------------------------------------------------

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <unordered_map>

#include "mobile_perception/mobile_control/rotation_controller.hpp"
#include "mobile_perception/mobile_control/s_curve_speed_profile_planner.hpp"
#include "mobile_perception/mobile_control/straight_controller.hpp"

namespace {

using mobile_perception::mobile_control::RotationControlInput;
using mobile_perception::mobile_control::RotationController;
using mobile_perception::mobile_control::RotationControllerConfig;
using mobile_perception::mobile_control::SCurveSpeedProfilePlanner;
using mobile_perception::mobile_control::SpeedProfile;
using mobile_perception::mobile_control::SpeedProfileRequest;
using mobile_perception::mobile_control::StraightControlInput;
using mobile_perception::mobile_control::StraightController;
using mobile_perception::mobile_control::StraightControllerConfig;
using mobile_perception::mobile_control::TerminalSlowdownConstraint;

struct Options {
    std::string mode;
    SpeedProfileRequest request;
    double slowdown_motion{0.0};
    double slowdown_speed{0.0};
    double goal_tolerance{0.0};
    double min_velocity{0.0};
    double preview_time{0.05};
};

void printUsage(const char* program) {
    std::cerr << "Usage: " << program
              << " --mode linear|angular --distance VALUE --max-velocity VALUE --max-acceleration VALUE"
                 " --max-deceleration VALUE --max-jerk VALUE"
                 " --slowdown-motion VALUE --slowdown-speed VALUE"
                 " [--goal-tolerance VALUE] [--min-velocity VALUE]"
                 " [--preview-time VALUE] [--sample-period VALUE] [--max-duration VALUE]\n";
}

bool parseDouble(const std::string& text, double& value) {
    errno = 0;
    char* end = nullptr;
    const double parsed = std::strtod(text.c_str(), &end);
    if (errno != 0 || end == text.c_str() || *end != '\0' || !std::isfinite(parsed)) {
        return false;
    }
    value = parsed;
    return true;
}

bool parseArguments(int argc, char** argv, Options& options) {
    std::unordered_map<std::string, double*> double_options{
        {"--distance", &options.request.distance},
        {"--max-velocity", &options.request.max_velocity},
        {"--max-acceleration", &options.request.max_acceleration},
        {"--max-deceleration", &options.request.max_deceleration},
        {"--max-jerk", &options.request.max_jerk},
        {"--slowdown-motion", &options.slowdown_motion},
        {"--slowdown-speed", &options.slowdown_speed},
        {"--goal-tolerance", &options.goal_tolerance},
        {"--min-velocity", &options.min_velocity},
        {"--preview-time", &options.preview_time},
        {"--sample-period", &options.request.sample_period},
        {"--max-duration", &options.request.max_duration},
    };
    std::unordered_map<std::string, bool> required{
        {"--distance", false},         {"--max-velocity", false}, {"--max-acceleration", false},
        {"--max-deceleration", false}, {"--max-jerk", false},     {"--slowdown-motion", false},
        {"--slowdown-speed", false},
    };
    bool mode_present = false;

    for (int index = 1; index < argc; ++index) {
        const std::string argument(argv[index]);
        if (argument == "--help" || argument == "-h") {
            printUsage(argv[0]);
            return false;
        }
        if (argument == "--mode") {
            if (index + 1 >= argc) {
                std::cerr << "Missing value for --mode\n";
                return false;
            }
            options.mode = argv[++index];
            mode_present = true;
            continue;
        }
        const auto option = double_options.find(argument);
        if (option == double_options.end() || index + 1 >= argc) {
            std::cerr << "Unknown option or missing value: " << argument << '\n';
            return false;
        }
        if (!parseDouble(argv[++index], *option->second)) {
            std::cerr << "Invalid numeric value for " << argument << ": " << argv[index] << '\n';
            return false;
        }
        const auto required_option = required.find(argument);
        if (required_option != required.end()) {
            required_option->second = true;
        }
    }

    for (const auto& [name, present] : required) {
        if (!present) {
            std::cerr << "Missing required option: " << name << '\n';
            return false;
        }
    }
    if (!mode_present || (options.mode != "linear" && options.mode != "angular")) {
        std::cerr << "--mode must be linear or angular\n";
        return false;
    }
    return true;
}

struct RuntimeReference {
    double target_velocity{0.0};
    bool slowdown_applied{false};
    bool braking_limited{false};
    bool minimum_velocity_applied{false};
};

RuntimeReference computeStraightReference(const Options& options,
                                          StraightController& controller,
                                          const mobile_perception::mobile_control::SpeedProfileSample& reference,
                                          double remaining_motion) {
    StraightControlInput input;
    input.reference_linear_velocity = reference.velocity;
    input.reference_linear_acceleration = reference.acceleration;
    input.goal_longitudinal_error = remaining_motion;
    input.goal_longitudinal_error_valid = true;
    input.current_linear_velocity = reference.velocity;
    input.dt = options.request.sample_period;
    const auto command = controller.computeCommand(input);
    return RuntimeReference{command.target_linear_velocity, command.terminal_velocity_applied, false, false};
}

RuntimeReference computeRotationReference(const Options& options,
                                          RotationController& controller,
                                          const mobile_perception::mobile_control::SpeedProfileSample& reference,
                                          double remaining_motion) {
    RotationControlInput input;
    input.reference_angular_velocity = reference.velocity;
    input.reference_angular_acceleration = reference.acceleration;
    input.goal_yaw_error = remaining_motion;
    input.current_angular_velocity = reference.velocity;
    input.dt = options.request.sample_period;
    const auto command = controller.computeCommand(input);
    return RuntimeReference{command.target_angular_velocity, command.terminal_velocity_applied, command.braking_limited,
                            command.minimum_velocity_applied};
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    options.request.initial_velocity = 0.0;
    options.request.initial_acceleration = 0.0;
    options.request.target_velocity = 0.0;
    options.request.sample_period = 0.02;
    options.request.max_duration = 30.0;

    if (!parseArguments(argc, argv, options)) {
        printUsage(argv[0]);
        return 2;
    }

    SCurveSpeedProfilePlanner planner;
    SpeedProfile profile;
    std::string error;
    const double effective_slowdown_speed =
        options.mode == "angular" ? std::max(options.slowdown_speed, options.min_velocity) : options.slowdown_speed;
    const TerminalSlowdownConstraint slowdown{options.slowdown_motion, effective_slowdown_speed};
    if (!mobile_perception::mobile_control::planSpeedProfileWithTerminalSlowdown(planner, options.request, slowdown,
                                                                                 profile, error)) {
        std::cerr << "S-curve planning failed: " << error << '\n';
        return 1;
    }

    StraightControllerConfig straight_config;
    straight_config.max_linear_velocity = options.request.max_velocity;
    straight_config.max_linear_acceleration = options.request.max_acceleration;
    straight_config.max_linear_deceleration = options.request.max_deceleration;
    straight_config.terminal_slowdown_distance = options.slowdown_motion;
    straight_config.terminal_linear_velocity = options.slowdown_speed;
    StraightController straight_controller(straight_config);

    RotationControllerConfig rotation_config;
    rotation_config.yaw_tolerance = options.goal_tolerance;
    rotation_config.min_angular_velocity = options.min_velocity;
    rotation_config.terminal_slowdown_angle = options.slowdown_motion;
    rotation_config.terminal_angular_velocity = options.slowdown_speed;
    rotation_config.max_angular_velocity = options.request.max_velocity;
    rotation_config.max_angular_acceleration = options.request.max_acceleration;
    RotationController rotation_controller(rotation_config);

    std::cout << std::setprecision(std::numeric_limits<double>::max_digits10);
    std::cout << "time,progress_position,profile_velocity,reference_position,remaining_motion,planned_velocity,"
                 "runtime_target_velocity,"
                 "planned_acceleration,planned_jerk,slowdown_zone,slowdown_applied,"
                 "braking_limited,minimum_velocity_applied\n";
    for (const auto& point : profile.points) {
        const double remaining_motion = std::max(0.0, options.request.distance - point.position);
        const auto sampled_reference = mobile_perception::mobile_control::sampleSpeedProfileForTracking(
            profile, point.position, point.relative_time + options.preview_time, options.preview_time);
        const RuntimeReference runtime_reference =
            options.mode == "linear"
                ? computeStraightReference(options, straight_controller, sampled_reference, remaining_motion)
                : computeRotationReference(options, rotation_controller, sampled_reference, remaining_motion);
        const bool slowdown_zone = remaining_motion <= options.slowdown_motion;
        std::cout << point.relative_time << ',' << point.position << ',' << point.velocity << ','
                  << sampled_reference.position << ',' << remaining_motion << ',' << sampled_reference.velocity << ','
                  << runtime_reference.target_velocity << ',' << sampled_reference.acceleration << ','
                  << sampled_reference.jerk << ',' << static_cast<int>(slowdown_zone) << ','
                  << static_cast<int>(runtime_reference.slowdown_applied) << ','
                  << static_cast<int>(runtime_reference.braking_limited) << ','
                  << static_cast<int>(runtime_reference.minimum_velocity_applied) << '\n';
    }
    return 0;
}
