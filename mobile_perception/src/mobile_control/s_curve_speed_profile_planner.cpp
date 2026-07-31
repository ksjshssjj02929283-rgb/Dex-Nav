// ----------------------------------------------------------------------------
// Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
//
// All rights reserved.
// ----------------------------------------------------------------------------

#include "mobile_perception/mobile_control/s_curve_speed_profile_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <sstream>
#include <utility>

#include "ruckig/ruckig.hpp"

namespace mobile_perception::mobile_control {

namespace {

constexpr double kTolerance = 1e-9;

bool isPositiveFinite(double value) { return std::isfinite(value) && value > 0.0; }

}  // namespace

bool SCurveSpeedProfilePlanner::plan(const SpeedProfileRequest& request, SpeedProfile& profile, std::string& error) {
    profile = SpeedProfile{};
    error.clear();

    if (!std::isfinite(request.distance) || request.distance < 0.0 || !std::isfinite(request.initial_velocity) ||
        request.initial_velocity < -kTolerance || !std::isfinite(request.initial_acceleration) ||
        !std::isfinite(request.target_velocity) || request.target_velocity < -kTolerance ||
        !isPositiveFinite(request.max_velocity) || !isPositiveFinite(request.max_acceleration) ||
        !isPositiveFinite(request.max_deceleration) || !isPositiveFinite(request.max_jerk) ||
        !isPositiveFinite(request.sample_period) || !isPositiveFinite(request.max_duration)) {
        error = "invalid or non-finite S-curve request";
        return false;
    }

    if (request.initial_velocity > request.max_velocity + kTolerance ||
        request.target_velocity > request.max_velocity + kTolerance ||
        request.initial_acceleration > request.max_acceleration + kTolerance ||
        request.initial_acceleration < -request.max_deceleration - kTolerance) {
        error = "initial/target state exceeds configured motion limits";
        return false;
    }

    if (request.distance <= kTolerance) {
        if (request.initial_velocity > kTolerance || request.target_velocity > kTolerance ||
            std::fabs(request.initial_acceleration) > kTolerance) {
            error = "zero-distance profile cannot absorb a moving initial state";
            return false;
        }
        profile.points.push_back(SpeedProfilePoint{});
        return true;
    }

    ruckig::Ruckig<1> trajectory_generator(request.sample_period);
    ruckig::InputParameter<1> input;
    ruckig::OutputParameter<1> output;

    input.control_interface = ruckig::ControlInterface::Position;
    input.current_position = {0.0};
    input.current_velocity = {std::max(0.0, request.initial_velocity)};
    input.current_acceleration = {request.initial_acceleration};
    input.target_position = {request.distance};
    input.target_velocity = {request.target_velocity};
    input.target_acceleration = {0.0};
    input.max_velocity = {request.max_velocity};
    input.max_acceleration = {request.max_acceleration};
    input.min_acceleration = std::array<double, 1>{-request.max_deceleration};
    input.max_jerk = {request.max_jerk};

    profile.points.push_back({0.0, input.current_velocity[0], input.current_acceleration[0], 0.0, 0.0});

    const int max_steps = std::max(1, static_cast<int>(std::ceil(request.max_duration / request.sample_period)));
    double previous_acceleration = input.current_acceleration[0];

    for (int step = 1; step <= max_steps; ++step) {
        const ruckig::Result result = trajectory_generator.update(input, output);
        if (result != ruckig::Result::Working && result != ruckig::Result::Finished) {
            std::ostringstream stream;
            stream << "Ruckig failed with result " << static_cast<int>(result);
            error = stream.str();
            profile = SpeedProfile{};
            return false;
        }

        double position = output.new_position[0];
        double velocity = output.new_velocity[0];
        double acceleration = output.new_acceleration[0];
        double jerk = (acceleration - previous_acceleration) / request.sample_period;
        const double relative_time =
            result == ruckig::Result::Finished ? output.trajectory.get_duration() : step * request.sample_period;
        if (result == ruckig::Result::Finished) {
            // With a non-zero target velocity, Ruckig propagates the finished
            // target state through the remainder of the fixed update period.
            // Store the requested boundary state itself so a following segment
            // starts at exactly the configured slowdown point.
            position = request.distance;
            velocity = request.target_velocity;
            acceleration = 0.0;
            jerk = 0.0;
        }

        if (!std::isfinite(position) || !std::isfinite(velocity) || !std::isfinite(acceleration) ||
            !std::isfinite(jerk) || position + 1e-7 < profile.points.back().position || position < -1e-7 ||
            position > request.distance + 1e-6 || velocity < -1e-7 || velocity > request.max_velocity + 1e-6 ||
            acceleration > request.max_acceleration + 1e-6 || acceleration < -request.max_deceleration - 1e-6 ||
            std::fabs(jerk) > request.max_jerk + 1e-5) {
            std::ostringstream stream;
            stream << "Ruckig generated a non-monotonic or out-of-limit S-curve"
                   << " at step=" << step << ", result=" << static_cast<int>(result) << ", position=" << position
                   << ", previous_position=" << profile.points.back().position << ", velocity=" << velocity
                   << ", acceleration=" << acceleration << ", jerk=" << jerk;
            error = stream.str();
            profile = SpeedProfile{};
            return false;
        }

        profile.points.push_back({position, std::max(0.0, velocity), acceleration, jerk, relative_time});
        previous_acceleration = acceleration;

        if (result == ruckig::Result::Finished) {
            auto& endpoint = profile.points.back();
            endpoint.position = request.distance;
            endpoint.velocity = request.target_velocity;
            endpoint.acceleration = 0.0;
            endpoint.jerk = 0.0;
            profile.duration = endpoint.relative_time;
            return true;
        }

        output.pass_to_input(input);
    }

    error = "S-curve planning exceeded max_duration";
    profile = SpeedProfile{};
    return false;
}

bool planSpeedProfileWithTerminalSlowdown(SpeedProfilePlanner& planner,
                                          const SpeedProfileRequest& request,
                                          const TerminalSlowdownConstraint& slowdown,
                                          SpeedProfile& profile,
                                          std::string& error) {
    profile = SpeedProfile{};
    error.clear();

    if (!std::isfinite(slowdown.motion) || slowdown.motion < 0.0 || !std::isfinite(slowdown.velocity) ||
        slowdown.velocity < 0.0) {
        error = "invalid or non-finite terminal slowdown constraint";
        return false;
    }
    if (request.distance <= kTolerance || slowdown.motion <= kTolerance) {
        return planner.plan(request, profile, error);
    }
    if (slowdown.velocity <= kTolerance) {
        error = "terminal slowdown velocity must be positive when slowdown motion is enabled";
        return false;
    }
    if (slowdown.velocity > request.max_velocity + kTolerance ||
        request.target_velocity > slowdown.velocity + kTolerance) {
        error = "terminal slowdown velocity is incompatible with profile velocity limits";
        return false;
    }

    const double terminal_motion = std::min(request.distance, slowdown.motion);
    const double main_motion = request.distance - terminal_motion;
    if (main_motion <= kTolerance) {
        auto terminal_request = request;
        terminal_request.max_velocity = std::min(
            request.max_velocity, std::max({slowdown.velocity, request.initial_velocity, request.target_velocity}));
        if (!planner.plan(terminal_request, profile, error)) {
            error = "terminal slowdown profile failed: " + error;
            return false;
        }
        return true;
    }

    auto main_request = request;
    main_request.distance = main_motion;
    main_request.target_velocity = slowdown.velocity;

    SpeedProfile main_profile;
    if (!planner.plan(main_request, main_profile, error)) {
        error = "main profile before terminal slowdown failed: " + error;
        return false;
    }
    if (main_profile.points.empty() || main_profile.duration >= request.max_duration - kTolerance) {
        error = "main profile leaves no duration for terminal slowdown";
        return false;
    }

    auto terminal_request = request;
    terminal_request.distance = terminal_motion;
    terminal_request.initial_velocity = slowdown.velocity;
    terminal_request.initial_acceleration = 0.0;
    terminal_request.max_velocity = slowdown.velocity;
    terminal_request.max_duration = request.max_duration - main_profile.duration;

    SpeedProfile terminal_profile;
    if (!planner.plan(terminal_request, terminal_profile, error)) {
        error = "terminal slowdown profile failed: " + error;
        return false;
    }
    if (terminal_profile.points.empty()) {
        error = "terminal slowdown profile is empty";
        return false;
    }

    profile = std::move(main_profile);
    profile.points.reserve(profile.points.size() + terminal_profile.points.size() - 1);
    for (auto point = std::next(terminal_profile.points.begin()); point != terminal_profile.points.end(); ++point) {
        point->position += main_motion;
        point->relative_time += profile.duration;
        profile.points.push_back(*point);
    }
    profile.duration += terminal_profile.duration;
    return true;
}

SpeedProfileSample sampleSpeedProfileByPosition(const SpeedProfile& profile, double position) {
    SpeedProfileSample sample;
    if (profile.points.empty() || !std::isfinite(position)) {
        return sample;
    }

    if (position <= profile.points.front().position) {
        const auto& point = profile.points.front();
        return {point.position, point.velocity, point.acceleration, point.jerk, point.relative_time};
    }
    if (position >= profile.points.back().position) {
        const auto& point = profile.points.back();
        return {point.position, point.velocity, point.acceleration, point.jerk, point.relative_time};
    }

    const auto upper =
        std::lower_bound(profile.points.begin(), profile.points.end(), position,
                         [](const SpeedProfilePoint& point, double query) { return point.position < query; });
    const auto lower = std::prev(upper);
    const double span = upper->position - lower->position;
    const double ratio = span > kTolerance ? (position - lower->position) / span : 0.0;
    const auto interpolate = [ratio](double start, double finish) { return start + ratio * (finish - start); };

    sample.position = interpolate(lower->position, upper->position);
    sample.velocity = interpolate(lower->velocity, upper->velocity);
    sample.acceleration = interpolate(lower->acceleration, upper->acceleration);
    sample.jerk = interpolate(lower->jerk, upper->jerk);
    sample.relative_time = interpolate(lower->relative_time, upper->relative_time);
    return sample;
}

SpeedProfileSample sampleSpeedProfileByTime(const SpeedProfile& profile, double relative_time) {
    SpeedProfileSample sample;
    if (profile.points.empty() || !std::isfinite(relative_time)) {
        return sample;
    }

    if (relative_time <= profile.points.front().relative_time) {
        const auto& point = profile.points.front();
        return {point.position, point.velocity, point.acceleration, point.jerk, point.relative_time};
    }
    if (relative_time >= profile.points.back().relative_time) {
        const auto& point = profile.points.back();
        return {point.position, point.velocity, point.acceleration, point.jerk, point.relative_time};
    }

    const auto upper =
        std::lower_bound(profile.points.begin(), profile.points.end(), relative_time,
                         [](const SpeedProfilePoint& point, double query) { return point.relative_time < query; });
    const auto lower = std::prev(upper);
    const double span = upper->relative_time - lower->relative_time;
    const double ratio = span > kTolerance ? (relative_time - lower->relative_time) / span : 0.0;
    const auto interpolate = [ratio](double start, double finish) { return start + ratio * (finish - start); };

    sample.position = interpolate(lower->position, upper->position);
    sample.velocity = interpolate(lower->velocity, upper->velocity);
    sample.acceleration = interpolate(lower->acceleration, upper->acceleration);
    sample.jerk = interpolate(lower->jerk, upper->jerk);
    sample.relative_time = interpolate(lower->relative_time, upper->relative_time);
    return sample;
}

SpeedProfileSample sampleSpeedProfileForTracking(const SpeedProfile& profile,
                                                 double position,
                                                 double elapsed_time,
                                                 double max_preview_time) {
    const auto spatial_reference = sampleSpeedProfileByPosition(profile, position);
    if (profile.points.empty() || !std::isfinite(elapsed_time) || !std::isfinite(max_preview_time)) {
        return spatial_reference;
    }

    const double preview_time = std::max(0.0, max_preview_time);
    const double lower_time = spatial_reference.relative_time;
    const double upper_time = std::min(profile.duration, lower_time + preview_time);
    const double tracking_time = std::clamp(elapsed_time, lower_time, upper_time);
    return sampleSpeedProfileByTime(profile, tracking_time);
}

}  // namespace mobile_perception::mobile_control
