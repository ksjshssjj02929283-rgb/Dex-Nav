// ----------------------------------------------------------------------------
// Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
//
// All rights reserved.
// ----------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "mobile_perception/mobile_control/goal_checker.hpp"
#include "mobile_perception/mobile_control/rotation_controller.hpp"
#include "mobile_perception/mobile_control/s_curve_speed_profile_planner.hpp"
#include "mobile_perception/mobile_control/short_trajectory_builder.hpp"
#include "mobile_perception/mobile_control/straight_controller.hpp"

namespace {

using namespace mobile_perception::mobile_control;

constexpr double kPi = 3.14159265358979323846;

PidConfig proportionalPid(double kp) {
    PidConfig config;
    config.integrator_enable = false;
    config.kp = kp;
    return config;
}

StraightControllerConfig makeStraightConfig() {
    StraightControllerConfig config;
    config.max_linear_velocity = 0.35;
    config.max_angular_velocity = 1.0;
    config.max_linear_acceleration = 0.5;
    config.max_linear_deceleration = 0.6;
    config.max_angular_acceleration = 1.0;
    config.terminal_slowdown_distance = 0.05;
    config.terminal_linear_velocity = 0.03;
    config.position_pid = proportionalPid(1.0);
    config.speed_pid = proportionalPid(1.0);
    config.lateral_pid = proportionalPid(1.0);
    config.heading_pid = proportionalPid(1.0);
    return config;
}

RotationControllerConfig makeRotationConfig() {
    RotationControllerConfig config;
    config.yaw_kp = 1.0;
    config.yaw_tolerance = 0.008;
    config.min_angular_velocity = 0.02;
    config.terminal_slowdown_angle = 0.30;
    config.terminal_angular_velocity = 0.01;
    config.max_angular_velocity = 0.75;
    config.max_angular_acceleration = 1.0;
    config.angular_velocity_pid.integrator_enable = true;
    config.angular_velocity_pid.integrator_saturation_level = 0.5;
    config.angular_velocity_pid.kp = 1.0;
    config.angular_velocity_pid.ki = 0.01;
    config.angular_velocity_pid.kd = 0.10;
    return config;
}

SpeedProfileRequest makeProfileRequest(double distance) {
    SpeedProfileRequest request;
    request.distance = distance;
    request.max_velocity = 0.35;
    request.max_acceleration = 0.5;
    request.max_deceleration = 0.5;
    request.max_jerk = 1.0;
    request.sample_period = 0.01;
    request.max_duration = 30.0;
    return request;
}

TEST(SCurveSpeedProfilePlannerTest, GeneratesMonotonicBoundedStopProfile) {
    SCurveSpeedProfilePlanner planner;
    SpeedProfile profile;
    std::string error;
    const auto request = makeProfileRequest(0.75);

    ASSERT_TRUE(planner.plan(request, profile, error)) << error;
    ASSERT_GT(profile.points.size(), 2U);
    EXPECT_DOUBLE_EQ(profile.points.front().position, 0.0);
    EXPECT_NEAR(profile.points.back().position, request.distance, 1e-9);
    EXPECT_NEAR(profile.points.back().velocity, 0.0, 1e-9);
    EXPECT_NEAR(profile.points.back().acceleration, 0.0, 1e-9);
    for (size_t index = 1; index < profile.points.size(); ++index) {
        EXPECT_GE(profile.points[index].position + 1e-8, profile.points[index - 1].position);
        EXPECT_GE(profile.points[index].velocity, -1e-8);
        EXPECT_LE(profile.points[index].velocity, request.max_velocity + 1e-6);
        EXPECT_LE(profile.points[index].acceleration, request.max_acceleration + 1e-6);
        EXPECT_GE(profile.points[index].acceleration, -request.max_deceleration - 1e-6);
        EXPECT_LE(std::fabs(profile.points[index].jerk), request.max_jerk + 1e-5);
    }
}

TEST(SCurveSpeedProfilePlannerTest, SupportsShortPathAndSpatialSampling) {
    SCurveSpeedProfilePlanner planner;
    SpeedProfile profile;
    std::string error;
    ASSERT_TRUE(planner.plan(makeProfileRequest(0.08), profile, error)) << error;

    const auto middle = sampleSpeedProfileByPosition(profile, 0.04);
    EXPECT_GT(middle.velocity, 0.0);
    EXPECT_GT(middle.relative_time, 0.0);
    const auto end = sampleSpeedProfileByPosition(profile, 1.0);
    EXPECT_DOUBLE_EQ(end.velocity, 0.0);

    double peak_velocity = 0.0;
    bool has_acceleration = false;
    bool has_deceleration = false;
    for (const auto& point : profile.points) {
        peak_velocity = std::max(peak_velocity, point.velocity);
        has_acceleration = has_acceleration || point.acceleration > 1e-6;
        has_deceleration = has_deceleration || point.acceleration < -1e-6;
    }
    EXPECT_LT(peak_velocity, makeProfileRequest(0.08).max_velocity);
    EXPECT_TRUE(has_acceleration);
    EXPECT_TRUE(has_deceleration);
}

TEST(SCurveSpeedProfilePlannerTest, TrackingPreviewBootstrapsAStationaryProfile) {
    SCurveSpeedProfilePlanner planner;
    SpeedProfile profile;
    std::string error;
    ASSERT_TRUE(planner.plan(makeProfileRequest(0.20), profile, error)) << error;

    const auto no_preview = sampleSpeedProfileForTracking(profile, 0.0, 0.05, 0.0);
    EXPECT_DOUBLE_EQ(no_preview.velocity, 0.0);
    EXPECT_DOUBLE_EQ(no_preview.acceleration, 0.0);

    const auto preview = sampleSpeedProfileForTracking(profile, 0.0, 0.05, 0.05);
    EXPECT_GT(preview.position, no_preview.position);
    EXPECT_GT(preview.velocity, 0.0);
    EXPECT_GT(preview.acceleration, 0.0);
    EXPECT_NEAR(preview.relative_time, 0.05, 1e-12);
}

TEST(SCurveSpeedProfilePlannerTest, TrackingPreviewIsBoundedFromSpatialReference) {
    SCurveSpeedProfilePlanner planner;
    SpeedProfile profile;
    std::string error;
    ASSERT_TRUE(planner.plan(makeProfileRequest(0.20), profile, error)) << error;

    const auto spatial = sampleSpeedProfileByPosition(profile, 0.02);
    const auto preview = sampleSpeedProfileForTracking(profile, 0.02, 10.0, 0.05);
    EXPECT_LE(preview.relative_time, spatial.relative_time + 0.05 + 1e-12);
    EXPECT_LE(preview.relative_time, profile.duration);
}

TEST(SCurveSpeedProfilePlannerTest, RejectsImpossibleZeroDistanceMovingState) {
    SCurveSpeedProfilePlanner planner;
    SpeedProfile profile;
    std::string error;
    auto request = makeProfileRequest(0.0);
    request.initial_velocity = 0.1;

    EXPECT_FALSE(planner.plan(request, profile, error));
    EXPECT_FALSE(error.empty());
}

TEST(SCurveSpeedProfilePlannerTest, PlansTerminalSlowdownBeforeTheBoundaryWithoutVelocityTruncation) {
    SCurveSpeedProfilePlanner planner;
    SpeedProfile profile;
    std::string error;
    const auto request = makeProfileRequest(0.20);
    const TerminalSlowdownConstraint slowdown{0.05, 0.03};

    ASSERT_TRUE(planSpeedProfileWithTerminalSlowdown(planner, request, slowdown, profile, error)) << error;
    const double slowdown_point = request.distance - slowdown.motion;
    const auto boundary = sampleSpeedProfileByPosition(profile, slowdown_point);
    EXPECT_NEAR(boundary.position, slowdown_point, 1e-9);
    EXPECT_NEAR(boundary.velocity, slowdown.velocity, 1e-9);
    EXPECT_NEAR(boundary.acceleration, 0.0, 1e-9);
    EXPECT_NEAR(profile.points.back().position, request.distance, 1e-9);
    EXPECT_NEAR(profile.points.back().velocity, request.target_velocity, 1e-9);

    bool found_boundary = false;
    for (size_t index = 0; index < profile.points.size(); ++index) {
        const auto& point = profile.points[index];
        if (std::fabs(point.position - slowdown_point) <= 1e-9) {
            found_boundary = true;
            if (index + 1 < profile.points.size()) {
                EXPECT_LE(std::fabs(profile.points[index + 1].velocity - point.velocity),
                          request.max_acceleration * request.sample_period + 1e-6);
            }
        }
        if (point.position + 1e-9 >= slowdown_point) {
            EXPECT_LE(point.velocity, slowdown.velocity + 1e-6);
        }
    }
    EXPECT_TRUE(found_boundary);
}

TEST(SCurveSpeedProfilePlannerTest, UsesSlowdownVelocityAsShortMotionMaximum) {
    SCurveSpeedProfilePlanner planner;
    SpeedProfile profile;
    std::string error;
    const auto request = makeProfileRequest(0.04);
    const TerminalSlowdownConstraint slowdown{0.05, 0.03};

    ASSERT_TRUE(planSpeedProfileWithTerminalSlowdown(planner, request, slowdown, profile, error)) << error;
    EXPECT_NEAR(profile.points.back().position, request.distance, 1e-9);
    EXPECT_NEAR(profile.points.back().velocity, 0.0, 1e-9);
    for (const auto& point : profile.points) {
        EXPECT_LE(point.velocity, slowdown.velocity + 1e-6);
    }

    auto invalid_request = request;
    invalid_request.initial_velocity = invalid_request.max_velocity + 0.01;
    EXPECT_FALSE(planSpeedProfileWithTerminalSlowdown(planner, invalid_request, slowdown, profile, error));
}

TEST(PreciseGoalCheckerTest, RequiresPoseVelocityAndConfirmDuration) {
    GoalCheckerConfig config;
    config.longitudinal_tolerance = 0.05;
    config.lateral_tolerance = 0.05;
    config.yaw_tolerance = 0.08;
    config.stopped_linear_velocity = 0.01;
    config.stopped_angular_velocity = 0.02;
    PreciseGoalChecker checker(config);

    auto result = checker.update({0.049, 0.049, 0.079, 0.02, 0.0, 0.1});
    EXPECT_TRUE(result.pose_reached);
    EXPECT_FALSE(result.speed_reached);
    EXPECT_FALSE(result.goal_reached);
    EXPECT_EQ(selectGoalControlState(result), GoalControlState::TrackTrajectory);

    result = checker.update({0.049, 0.049, 0.079, 0.005, 0.01, 0.1});
    EXPECT_FALSE(result.goal_reached);
    EXPECT_EQ(selectGoalControlState(result), GoalControlState::HoldZero);
    result = checker.update({0.049, 0.049, 0.079, 0.005, 0.01, 0.1});
    EXPECT_TRUE(result.goal_reached);
    EXPECT_EQ(selectGoalControlState(result), GoalControlState::Succeeded);
}

TEST(PreciseGoalCheckerTest, ChecksLongitudinalLateralAndYawSeparately) {
    GoalCheckerConfig config;
    config.longitudinal_tolerance = 0.05;
    config.lateral_tolerance = 0.05;
    config.yaw_tolerance = 0.08;
    PreciseGoalChecker checker(config);

    auto result = checker.update({0.051, 0.04, 0.04, 0.0, 0.0, 0.1});
    EXPECT_FALSE(result.longitudinal_reached);
    EXPECT_TRUE(result.lateral_reached);
    EXPECT_TRUE(result.yaw_reached);
    EXPECT_FALSE(result.pose_reached);

    result = checker.update({0.04, 0.051, 0.04, 0.0, 0.0, 0.1});
    EXPECT_TRUE(result.longitudinal_reached);
    EXPECT_FALSE(result.lateral_reached);
    EXPECT_TRUE(result.yaw_reached);
    EXPECT_FALSE(result.pose_reached);

    result = checker.update({0.04, 0.04, 0.081, 0.0, 0.0, 0.1});
    EXPECT_TRUE(result.longitudinal_reached);
    EXPECT_TRUE(result.lateral_reached);
    EXPECT_FALSE(result.yaw_reached);
    EXPECT_FALSE(result.pose_reached);
}

TEST(PreciseGoalCheckerTest, ResetsConfirmationWhenToleranceIsLost) {
    GoalCheckerConfig config;
    config.longitudinal_tolerance = 0.05;
    config.lateral_tolerance = 0.05;
    config.yaw_tolerance = 0.08;
    PreciseGoalChecker checker(config);
    checker.update({0.04, 0.04, 0.04, 0.0, 0.0, 0.1});
    auto result = checker.update({0.051, 0.04, 0.04, 0.0, 0.0, 0.1});
    EXPECT_FALSE(result.pose_reached);
    EXPECT_DOUBLE_EQ(result.stable_duration, 0.0);
}

TEST(PreciseGoalCheckerTest, RotationIgnoresPositionButRequiresYawVelocityAndDwell) {
    GoalCheckerConfig config;
    config.longitudinal_tolerance = 0.05;
    config.lateral_tolerance = 0.05;
    config.yaw_tolerance = 0.08;
    config.stopped_linear_velocity = 0.01;
    config.stopped_angular_velocity = 0.02;
    config.confirm_duration = 0.2;
    PreciseGoalChecker checker(config);

    auto result = checker.update({1.0, 1.0, 0.081, 0.0, 0.0, 0.1, GoalCheckMode::Rotation});
    EXPECT_FALSE(result.position_required);
    EXPECT_FALSE(result.position_reached);
    EXPECT_FALSE(result.yaw_reached);
    EXPECT_FALSE(result.pose_reached);

    result = checker.update({1.0, 1.0, 0.04, 0.0, 0.03, 0.1, GoalCheckMode::Rotation});
    EXPECT_TRUE(result.pose_reached);
    EXPECT_FALSE(result.speed_reached);
    EXPECT_EQ(selectGoalControlState(result), GoalControlState::TrackTrajectory);

    result = checker.update({1.0, 1.0, 0.04, 0.0, 0.0, 0.1, GoalCheckMode::Rotation});
    EXPECT_EQ(selectGoalControlState(result), GoalControlState::HoldZero);
    result = checker.update({2.0, 2.0, 0.04, 0.0, 0.0, 0.1, GoalCheckMode::Rotation});
    EXPECT_TRUE(result.goal_reached);
    EXPECT_EQ(selectGoalControlState(result), GoalControlState::Succeeded);
}

TEST(StraightControllerTest, CascadesForwardSCurveReferenceThroughSpeedPid) {
    auto config = makeStraightConfig();
    StraightController controller(config);

    const auto command = controller.computeCommand({0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, false});
    EXPECT_NEAR(command.target_linear_velocity, 0.2, 1e-12);
    EXPECT_NEAR(command.linear_x, 0.02, 1e-12);
    EXPECT_DOUBLE_EQ(command.angular_z, 0.0);
}

TEST(StraightControllerTest, TracksReverseSCurveReference) {
    StraightController controller(makeStraightConfig());

    const auto command = controller.computeCommand({-0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, true});
    EXPECT_NEAR(command.target_linear_velocity, -0.2, 1e-12);
    EXPECT_NEAR(command.linear_x, -0.02, 1e-12);
}

TEST(StraightControllerTest, UsesSCurveVelocityAsEnvelopeAndConsumesDeceleration) {
    auto config = makeStraightConfig();
    config.speed_pid = proportionalPid(0.0);
    StraightController controller(config);

    const auto accelerating = controller.computeCommand({0.05, 0.20, 0.0, 0.0, 0.0, 0.0, 0.1, false, 0.5});
    EXPECT_NEAR(accelerating.target_linear_velocity, 0.05, 1e-12);
    EXPECT_NEAR(accelerating.linear_acceleration_command, 0.5, 1e-12);
    EXPECT_NEAR(accelerating.linear_x, 0.05, 1e-12);

    const auto decelerating = controller.computeCommand({0.04, 0.20, 0.0, 0.0, 0.05, 0.0, 0.1, false, -0.5});
    EXPECT_NEAR(decelerating.target_linear_velocity, 0.04, 1e-12);
    EXPECT_NEAR(decelerating.linear_acceleration_command, -0.5, 1e-12);
    EXPECT_NEAR(decelerating.linear_x, 0.0, 1e-12);
}

TEST(StraightControllerTest, CapsTargetSpeedInsideConfiguredStopDistance) {
    auto config = makeStraightConfig();
    config.speed_pid = proportionalPid(0.0);
    StraightController controller(config);

    const auto outside = controller.computeCommand({0.10, 0.20, 0.0, 0.0, 0.0, 0.0, 0.1, false, 0.5, 0.051, true});
    EXPECT_NEAR(outside.target_linear_velocity, 0.10, 1e-12);
    EXPECT_FALSE(outside.terminal_velocity_applied);

    controller.reset();
    const auto inside = controller.computeCommand({0.10, 0.20, 0.0, 0.0, 0.0, 0.0, 0.1, false, 0.5, 0.049, true});
    EXPECT_NEAR(inside.target_linear_velocity, 0.03, 1e-12);
    EXPECT_TRUE(inside.terminal_velocity_applied);

    controller.reset();
    const auto endpoint_correction =
        controller.computeCommand({0.0, -0.20, 0.0, 0.0, 0.0, 0.0, 0.1, false, 0.0, -0.049, true});
    EXPECT_NEAR(endpoint_correction.target_linear_velocity, -0.03, 1e-12);
    EXPECT_TRUE(endpoint_correction.terminal_velocity_applied);
}

TEST(StraightControllerTest, SteersFromTrackingPointLateralError) {
    StraightController controller(makeStraightConfig());
    const auto command = controller.computeCommand({0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.1, false});
    EXPECT_NEAR(command.angular_z, 0.1, 1e-12);
}

TEST(StraightControllerTest, InvertsLateralSteeringWhenReversing) {
    StraightController controller(makeStraightConfig());
    const auto command = controller.computeCommand({0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.1, true});
    EXPECT_NEAR(command.angular_z, -0.1, 1e-12);
}

TEST(StraightControllerTest, CombinesLateralAndHeadingCorrections) {
    auto config = makeStraightConfig();
    config.max_angular_acceleration = 10.0;
    StraightController controller(config);
    const auto command = controller.computeCommand({0.0, 0.0, 0.2, 0.3, 0.0, 0.0, 0.1, false});
    EXPECT_NEAR(command.angular_z, 0.5, 1e-12);
}

TEST(StraightControllerTest, RejectsInvalidControlPeriod) {
    StraightController controller(makeStraightConfig());
    const auto command = controller.computeCommand({0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false});
    EXPECT_DOUBLE_EQ(command.linear_x, 0.0);
    EXPECT_DOUBLE_EQ(command.angular_z, 0.0);
}

TEST(StraightControllerTest, ComputesCurrentSignedProgressWithoutStickyMaximum) {
    EXPECT_NEAR(computeStraightProgress(0.4, 0.0081, false), 0.3919, 1e-12);
    EXPECT_NEAR(computeStraightProgress(0.4, -0.0142, true), 0.3858, 1e-12);
    EXPECT_DOUBLE_EQ(computeStraightProgress(0.4, -0.0081, false), 0.4);
    EXPECT_DOUBLE_EQ(computeStraightProgress(0.4, 0.0081, true), 0.4);
}

TEST(StraightControllerTest, PositionPidCorrectsTerminalErrorInEitherDirection) {
    StraightController controller(makeStraightConfig());

    const auto goal_ahead = controller.computeCommand({0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.05, true});
    EXPECT_NEAR(goal_ahead.position_pid_output, 0.01, 1e-12);
    EXPECT_NEAR(goal_ahead.target_linear_velocity, 0.01, 1e-12);
    EXPECT_GT(goal_ahead.linear_x, 0.0);

    controller.reset();
    const auto goal_behind = controller.computeCommand({0.0, -0.01, 0.0, 0.0, 0.0, 0.0, 0.05, false});
    EXPECT_NEAR(goal_behind.position_pid_output, -0.01, 1e-12);
    EXPECT_NEAR(goal_behind.target_linear_velocity, -0.01, 1e-12);
    EXPECT_LT(goal_behind.linear_x, 0.0);
}

TEST(StraightControllerTest, SpeedPidCommandIntegratorCrossesDeadbandWithoutMinimumSpeedBranch) {
    auto config = makeStraightConfig();
    config.position_pid = proportionalPid(1.5);
    config.speed_pid = proportionalPid(1.0);
    StraightController controller(config);

    constexpr double dt = 0.05;
    constexpr double deadband = 0.01;
    double measured_velocity = 0.0;
    double command_velocity = 0.0;
    double max_command_velocity = 0.0;
    for (int step = 0; step < 100; ++step) {
        const auto command = controller.computeCommand({0.0, 0.005, 0.0, 0.0, measured_velocity, 0.0, dt, false});
        command_velocity = command.linear_x;
        max_command_velocity = std::max(max_command_velocity, command_velocity);
        measured_velocity = std::fabs(command_velocity) >= deadband ? command_velocity : 0.0;
    }

    EXPECT_GE(max_command_velocity, deadband);
    EXPECT_LT(command_velocity, config.max_linear_velocity);
}

TEST(StraightControllerTest, SettlesSignedLongitudinalErrorAfterOvershoot) {
    auto config = makeStraightConfig();
    config.position_pid = proportionalPid(2.0);
    config.speed_pid = proportionalPid(4.0);
    StraightController controller(config);

    constexpr double dt = 0.05;
    double goal_error = 0.10;
    double velocity = 0.0;
    for (int step = 0; step < 200; ++step) {
        const auto command = controller.computeCommand({0.0, goal_error, 0.0, 0.0, velocity, 0.0, dt, true});
        velocity = command.linear_x;
        goal_error -= velocity * dt;
    }

    EXPECT_LT(std::fabs(goal_error), 0.005);
    EXPECT_LT(std::fabs(velocity), 0.005);
}

TEST(StraightControllerTest, TracksDistanceFeasibleSCurveWithPlannedDeceleration) {
    constexpr double dt = 0.05;
    constexpr double motion = 0.50;
    constexpr double direction = -1.0;
    constexpr double plant_time_constant = 0.15;

    SCurveSpeedProfilePlanner planner;
    SpeedProfile profile;
    std::string error;
    auto request = makeProfileRequest(motion);
    request.sample_period = dt;
    ASSERT_TRUE(planner.plan(request, profile, error)) << error;

    StraightController controller(makeStraightConfig());
    double pose_x = 0.0;
    double measured_velocity = 0.0;
    double maximum_overshoot = 0.0;
    bool settled = false;
    for (int step = 0; step < 400; ++step) {
        const double goal_longitudinal_error = direction * motion - pose_x;
        const double progress = computeStraightProgress(motion, goal_longitudinal_error, true);
        const auto reference = sampleSpeedProfileForTracking(profile, progress, step * dt, 0.05);
        const double tracking_longitudinal_error = direction * reference.position - pose_x;
        const auto command = controller.computeCommand(
            {direction * reference.velocity, tracking_longitudinal_error, 0.0, 0.0, measured_velocity, 0.0, dt, true,
             direction * reference.acceleration, goal_longitudinal_error, true});

        measured_velocity += (command.linear_x - measured_velocity) * std::min(1.0, dt / plant_time_constant);
        pose_x += measured_velocity * dt;
        maximum_overshoot = std::max(maximum_overshoot, std::max(0.0, -motion - pose_x));

        if (std::fabs(goal_longitudinal_error) <= 0.005 && std::fabs(measured_velocity) <= 0.005) {
            settled = true;
            break;
        }
    }

    EXPECT_TRUE(settled);
    EXPECT_LT(maximum_overshoot, 0.02);
}

TEST(StraightControllerTest, AcceptsAlignedGoalRobotAndPathHeadings) {
    constexpr double tolerance = 5.0 * kPi / 180.0;
    const auto result = checkStraightHeadingAlignment(0.0, 3.0 * kPi / 180.0, 4.0 * kPi / 180.0, tolerance);
    EXPECT_TRUE(result.accepted);
}

TEST(StraightControllerTest, RejectsAnyHeadingPairOutsidePrecontrolTolerance) {
    constexpr double tolerance = 5.0 * kPi / 180.0;
    const auto goal_outside = checkStraightHeadingAlignment(0.0, 6.0 * kPi / 180.0, 4.0 * kPi / 180.0, tolerance);
    EXPECT_FALSE(goal_outside.accepted);

    const auto path_outside = checkStraightHeadingAlignment(0.0, 4.0 * kPi / 180.0, -4.0 * kPi / 180.0, tolerance);
    EXPECT_FALSE(path_outside.accepted);
    EXPECT_GT(std::fabs(path_outside.goal_to_path_error), tolerance);
}

TEST(RotationControllerTest, CombinesSCurveReferenceAndYawOuterLoop) {
    auto config = makeRotationConfig();
    config.angular_velocity_pid = proportionalPid(0.0);
    RotationController controller(config);

    const auto command = controller.computeCommand({0.2, 0.4, 0.05, 1.0, 0.0, 0.1});
    EXPECT_NEAR(command.planned_angular_velocity_limit, 0.2, 1e-12);
    EXPECT_NEAR(command.target_angular_velocity, 0.2, 1e-12);
    EXPECT_DOUBLE_EQ(command.angular_velocity_pid_output, 0.0);
    EXPECT_NEAR(command.angular_acceleration_command, 0.4, 1e-12);
    EXPECT_NEAR(command.angular_z, 0.04, 1e-12);
}

TEST(RotationControllerTest, UsesSCurveVelocityAsDistanceFeasibleEnvelope) {
    auto config = makeRotationConfig();
    config.terminal_slowdown_angle = config.yaw_tolerance;
    config.angular_velocity_pid = proportionalPid(0.0);
    RotationController controller(config);

    const auto command = controller.computeCommand({0.10, 0.50, 0.30, 1.0, 0.0, 0.1});
    EXPECT_NEAR(command.planned_angular_velocity_limit, 0.10, 1e-12);
    EXPECT_NEAR(command.target_angular_velocity, 0.10, 1e-12);
    EXPECT_NEAR(command.angular_acceleration_command, 0.50, 1e-12);
}

TEST(RotationControllerTest, UnwrapsPiBoundaryWithoutChangingPlannedDirection) {
    EXPECT_NEAR(unwrapRotationError(-kPi + 0.001, kPi), kPi + 0.001, 1e-12);
    EXPECT_NEAR(unwrapRotationError(kPi - 0.001, -kPi), -kPi - 0.001, 1e-12);
    EXPECT_NEAR(unwrapRotationError(-0.01, 0.01), -0.01, 1e-12);
}

TEST(RotationControllerTest, AngularVelocityPidTracksOuterLoopTarget) {
    auto config = makeRotationConfig();
    config.yaw_kp = 0.0;
    config.angular_velocity_pid = proportionalPid(1.0);
    RotationController controller(config);
    controller.reset(0.1);

    const auto command = controller.computeCommand({0.2, 0.0, 0.0, 1.0, 0.1, 0.1});
    EXPECT_NEAR(command.target_angular_velocity, 0.2, 1e-12);
    EXPECT_NEAR(command.angular_velocity_pid_output, 0.1, 1e-12);
    EXPECT_NEAR(command.angular_acceleration_command, 0.1, 1e-12);
    EXPECT_NEAR(command.angular_z, 0.11, 1e-12);
}

TEST(RotationControllerTest, LimitsAngularVelocityAndAccelerationSymmetrically) {
    auto config = makeRotationConfig();
    config.angular_velocity_pid = proportionalPid(0.0);
    RotationController controller(config);

    const auto ccw = controller.computeCommand({0.75, 2.0, 0.0, 1.0, 0.5, 0.1});
    EXPECT_NEAR(ccw.angular_z, 0.1, 1e-12);

    controller.reset();
    const auto cw = controller.computeCommand({-0.75, -2.0, 0.0, -1.0, -0.5, 0.1});
    EXPECT_NEAR(cw.angular_z, -0.1, 1e-12);

    controller.reset(0.7);
    const auto saturated = controller.computeCommand({0.75, 2.0, 1.0, 1.0, 0.7, 0.1});
    EXPECT_NEAR(saturated.angular_z, 0.75, 1e-12);
}

TEST(RotationControllerTest, UsesPreviousCommandInsteadOfMeasuredVelocityAsRateLimitState) {
    auto config = makeRotationConfig();
    config.yaw_kp = 0.0;
    config.angular_velocity_pid = proportionalPid(0.0);
    RotationController controller(config);

    const auto command = controller.computeCommand({0.75, 1.0, 0.0, 1.0, 0.5, 0.1});
    EXPECT_NEAR(command.angular_z, 0.1, 1e-12);
}

TEST(RotationControllerTest, BrakesInsideGoalStoppingEnvelope) {
    auto config = makeRotationConfig();
    config.terminal_slowdown_angle = config.yaw_tolerance;
    config.angular_velocity_pid = proportionalPid(0.0);
    RotationController controller(config);
    controller.reset(0.75);

    const auto command = controller.computeCommand({0.75, 0.0, 0.0, 0.02, 0.75, 0.1});
    EXPECT_NEAR(command.braking_angular_velocity_limit, 0.2, 1e-12);
    EXPECT_NEAR(command.target_angular_velocity, 0.2, 1e-12);
    EXPECT_TRUE(command.braking_limited);
    EXPECT_NEAR(command.angular_z, 0.65, 1e-12);
    EXPECT_NEAR(command.angular_acceleration_command, -1.0, 1e-12);
}

TEST(RotationControllerTest, RaisesTerminalVelocityToDeadbandCompensatedMinimum) {
    auto config = makeRotationConfig();
    config.angular_velocity_pid = proportionalPid(0.0);
    RotationController controller(config);
    controller.reset(0.75);

    const auto command = controller.computeCommand({0.75, 0.0, 0.0, 0.25, 0.75, 0.1});
    EXPECT_NEAR(command.target_angular_velocity, 0.02, 1e-12);
    EXPECT_TRUE(command.terminal_velocity_applied);
    EXPECT_FALSE(command.minimum_velocity_applied);
    EXPECT_TRUE(command.braking_limited);
    EXPECT_NEAR(command.angular_z, 0.65, 1e-12);
    EXPECT_NEAR(command.angular_acceleration_command, -1.0, 1e-12);
}

TEST(RotationControllerTest, KeepsRequestedTerminalVelocityWhenAboveMinimum) {
    auto config = makeRotationConfig();
    config.min_angular_velocity = 0.005;
    config.terminal_angular_velocity = 0.01;
    config.angular_velocity_pid = proportionalPid(0.0);
    RotationController controller(config);

    const auto command = controller.computeCommand({0.1, 0.0, 0.0, 0.25, 0.0, 0.1});
    EXPECT_NEAR(command.target_angular_velocity, 0.01, 1e-12);
    EXPECT_TRUE(command.terminal_velocity_applied);
}

TEST(RotationControllerTest, FollowsPlannedDecelerationBelowTerminalSpeedWithoutRaisingIt) {
    auto config = makeRotationConfig();
    config.angular_velocity_pid = proportionalPid(0.0);
    RotationController controller(config);

    const auto command = controller.computeCommand({0.005, 0.0, 0.0, 0.02, 0.005, 0.1});
    EXPECT_NEAR(command.target_angular_velocity, 0.005, 1e-12);
    EXPECT_FALSE(command.terminal_velocity_applied);
    EXPECT_FALSE(command.minimum_velocity_applied);
}

TEST(RotationControllerTest, ProducesEffectiveVelocityCorrectionWhenStoppedOutsideTolerance) {
    auto config = makeRotationConfig();
    config.terminal_slowdown_angle = config.yaw_tolerance;
    RotationController controller(config);

    const auto command = controller.computeCommand({0.0, 0.0, 0.012, 0.012, 0.0, 0.05});
    EXPECT_NEAR(command.target_angular_velocity, 0.02, 1e-12);
    EXPECT_TRUE(command.minimum_velocity_applied);
    EXPECT_GE(command.angular_z, 0.02);
}

TEST(RotationControllerTest, DoesNotTargetMotionAwayAfterOvershoot) {
    auto config = makeRotationConfig();
    config.terminal_slowdown_angle = config.yaw_tolerance;
    config.angular_velocity_pid = proportionalPid(0.0);
    RotationController controller(config);
    controller.reset(0.1);

    const auto command = controller.computeCommand({0.1, 0.0, -0.02, -0.02, 0.1, 0.1});
    EXPECT_NEAR(command.target_angular_velocity, -0.02, 1e-12);
    EXPECT_TRUE(command.braking_limited);
    EXPECT_NEAR(command.angular_z, 0.0, 1e-12);
}

TEST(RotationControllerTest, RejectsNonFiniteInputAndInvalidPeriod) {
    RotationController controller(makeRotationConfig());

    EXPECT_DOUBLE_EQ(controller.computeCommand({std::nan(""), 0.0, 0.0, 0.0, 0.0, 0.1}).angular_z, 0.0);
    EXPECT_DOUBLE_EQ(controller.computeCommand({0.1, std::nan(""), 0.0, 0.0, 0.0, 0.1}).angular_z, 0.0);
    EXPECT_DOUBLE_EQ(controller.computeCommand({0.1, 0.0, std::nan(""), 0.0, 0.0, 0.1}).angular_z, 0.0);
    EXPECT_DOUBLE_EQ(controller.computeCommand({0.1, 0.0, 0.0, std::nan(""), 0.0, 0.1}).angular_z, 0.0);
    EXPECT_DOUBLE_EQ(controller.computeCommand({0.1, 0.0, 0.0, 0.0, std::nan(""), 0.1}).angular_z, 0.0);
    EXPECT_DOUBLE_EQ(controller.computeCommand({0.1, 0.0, 0.0, 0.0, 0.0, 0.0}).angular_z, 0.0);
}

TEST(RotationControllerTest, SCurveCascadeSettlesInBothDirectionsWithStrictThresholds) {
    constexpr double dt = 0.05;
    constexpr double motion = kPi / 2.0;
    auto request = makeProfileRequest(motion);
    request.max_velocity = 0.75;
    request.max_acceleration = 1.0;
    request.max_deceleration = 1.0;
    request.max_jerk = 2.0;
    request.sample_period = dt;
    SCurveSpeedProfilePlanner planner;
    SpeedProfile profile;
    std::string error;
    ASSERT_TRUE(planner.plan(request, profile, error)) << error;

    for (const double direction : {-1.0, 1.0}) {
        RotationController controller(makeRotationConfig());
        double goal_yaw_error = direction * motion;
        double angular_velocity = 0.0;
        bool settled = false;
        for (int step = 0; step < 800; ++step) {
            const auto reference = sampleSpeedProfileByTime(profile, step * dt);
            const double tracking_yaw_error =
                std::atan2(std::sin(goal_yaw_error - direction * (motion - reference.position)),
                           std::cos(goal_yaw_error - direction * (motion - reference.position)));
            const auto command =
                controller.computeCommand({direction * reference.velocity, direction * reference.acceleration,
                                           tracking_yaw_error, goal_yaw_error, angular_velocity, dt});
            angular_velocity = command.angular_z;
            goal_yaw_error = std::atan2(std::sin(goal_yaw_error - angular_velocity * dt),
                                        std::cos(goal_yaw_error - angular_velocity * dt));
            if (std::fabs(goal_yaw_error) <= 0.008 && std::fabs(angular_velocity) <= 0.01) {
                settled = true;
                break;
            }
        }

        EXPECT_TRUE(settled) << "direction=" << direction;
        EXPECT_LE(std::fabs(goal_yaw_error), 0.008);
        EXPECT_LE(std::fabs(angular_velocity), 0.01);
    }
}

TEST(RotationControllerTest, SpatialSCurveBrakesBeforeGoalWithPlantLag) {
    constexpr double dt = 0.05;
    constexpr double motion = kPi / 2.0;
    constexpr double plant_time_constant = 0.15;
    auto request = makeProfileRequest(motion);
    request.max_velocity = 0.75;
    request.max_acceleration = 1.0;
    request.max_deceleration = 1.0;
    request.max_jerk = 2.0;
    request.sample_period = dt;
    SCurveSpeedProfilePlanner planner;
    SpeedProfile profile;
    std::string error;
    ASSERT_TRUE(planner.plan(request, profile, error)) << error;

    for (const double direction : {-1.0, 1.0}) {
        RotationController controller(makeRotationConfig());
        double goal_yaw_error = direction * motion;
        double measured_angular_velocity = 0.0;
        double maximum_overshoot = 0.0;
        bool settled = false;
        for (int step = 0; step < 800; ++step) {
            const double progress = std::clamp(motion - direction * goal_yaw_error, 0.0, motion);
            const auto reference = sampleSpeedProfileForTracking(profile, progress, step * dt, 0.05);
            const double tracking_yaw_error =
                std::atan2(std::sin(goal_yaw_error - direction * (motion - reference.position)),
                           std::cos(goal_yaw_error - direction * (motion - reference.position)));
            const auto command =
                controller.computeCommand({direction * reference.velocity, direction * reference.acceleration,
                                           tracking_yaw_error, goal_yaw_error, measured_angular_velocity, dt});

            measured_angular_velocity +=
                (command.angular_z - measured_angular_velocity) * std::min(1.0, dt / plant_time_constant);
            goal_yaw_error -= measured_angular_velocity * dt;
            maximum_overshoot = std::max(maximum_overshoot, std::max(0.0, -direction * goal_yaw_error));

            if (std::fabs(goal_yaw_error) <= 0.008 && std::fabs(measured_angular_velocity) <= 0.01) {
                settled = true;
                break;
            }
        }

        EXPECT_TRUE(settled) << "direction=" << direction;
        EXPECT_LT(maximum_overshoot, 0.02) << "direction=" << direction;
    }
}

TEST(ShortTrajectoryBuilderTest, ProjectsSCurveToStraightRobotTrajectory) {
    SpeedProfile profile;
    profile.points = {
        {0.0, 0.0, 0.2, 1.0, 0.0},
        {1.0, 0.0, 0.0, 0.0, 2.0},
    };
    geometry_msgs::msg::PoseStamped start;
    start.header.frame_id = "map";
    start.pose.orientation.w = 1.0;
    geometry_msgs::msg::PoseStamped goal = start;
    goal.pose.position.x = 1.0;

    const auto output = ShortTrajectoryBuilder::build(start, goal, profile, MotionMode::Straight, 1.0, "test-plan");
    ASSERT_EQ(output.path.poses.size(), 2U);
    ASSERT_EQ(output.trajectory.points.size(), 2U);
    EXPECT_EQ(output.trajectory.plan_id, "test-plan");
    EXPECT_NEAR(output.trajectory.points.back().pose.pose.position.x, 1.0, 1e-12);
}

TEST(ShortTrajectoryBuilderTest, SamplesInterpolatedStraightTrackingPointByMotion) {
    SpeedProfile profile;
    profile.points = {
        {0.0, 0.0, 0.2, 1.0, 0.0},
        {1.0, 0.0, 0.0, 0.0, 2.0},
    };
    geometry_msgs::msg::PoseStamped start;
    start.header.frame_id = "map";
    start.pose.orientation.w = 1.0;
    geometry_msgs::msg::PoseStamped goal = start;
    goal.pose.position.x = 1.0;
    goal.pose.position.y = 0.5;

    const auto trajectory =
        ShortTrajectoryBuilder::build(start, goal, profile, MotionMode::Straight, 1.0, "sample-test");
    geometry_msgs::msg::PoseStamped tracking_pose;
    ASSERT_TRUE(ShortTrajectoryBuilder::samplePoseByMotion(trajectory, 0.25, tracking_pose));
    EXPECT_NEAR(tracking_pose.pose.position.x, 0.25, 1e-12);
    EXPECT_NEAR(tracking_pose.pose.position.y, 0.125, 1e-12);
}

TEST(ShortTrajectoryBuilderTest, RotationTrajectoryKeepsPositionFixed) {
    SpeedProfile profile;
    profile.points = {
        {0.0, 0.0, 0.0, 0.0, 0.0},
        {0.5, 0.0, 0.0, 0.0, 1.0},
    };
    geometry_msgs::msg::PoseStamped start;
    start.header.frame_id = "map";
    start.pose.position.x = 2.0;
    start.pose.orientation.w = 1.0;
    auto goal = start;
    goal.pose.position.x = 2.02;

    const auto output = ShortTrajectoryBuilder::build(start, goal, profile, MotionMode::Rotation, 1.0, "rotation");
    EXPECT_NEAR(output.trajectory.points.back().pose.pose.position.x, 2.0, 1e-12);
    EXPECT_DOUBLE_EQ(output.trajectory.points.back().longitudinal_velocity, 0.0);
    EXPECT_DOUBLE_EQ(output.trajectory.points.front().angular_velocity, 0.0);
    EXPECT_DOUBLE_EQ(output.trajectory.points.back().angular_velocity, 0.0);
}

}  // namespace
