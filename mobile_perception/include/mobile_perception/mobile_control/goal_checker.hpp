// ----------------------------------------------------------------------------
// Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
//
// All rights reserved.
// ----------------------------------------------------------------------------

#pragma once

namespace mobile_perception::mobile_control {

struct GoalCheckerConfig {
    double longitudinal_tolerance{0.005};
    double lateral_tolerance{0.005};
    double yaw_tolerance{0.008};
    double stopped_linear_velocity{0.005};
    double stopped_angular_velocity{0.01};
    double confirm_duration{0.2};
};

enum class GoalCheckMode { Translation, Rotation };

struct GoalCheckInput {
    double longitudinal_error{0.0};
    double lateral_error{0.0};
    double yaw_error{0.0};
    double linear_velocity{0.0};
    double angular_velocity{0.0};
    double dt{0.0};
    GoalCheckMode mode{GoalCheckMode::Translation};
};

struct GoalCheckResult {
    bool position_required{true};
    bool longitudinal_reached{false};
    bool lateral_reached{false};
    bool position_reached{false};
    bool yaw_reached{false};
    bool pose_reached{false};
    bool speed_reached{false};
    bool goal_reached{false};
    double stable_duration{0.0};
};

enum class GoalControlState { TrackTrajectory, HoldZero, Succeeded };

GoalControlState selectGoalControlState(const GoalCheckResult& result);

class PreciseGoalChecker {
public:
    explicit PreciseGoalChecker(const GoalCheckerConfig& config);

    GoalCheckResult update(const GoalCheckInput& input);
    void reset();
    const GoalCheckerConfig& config() const;

private:
    GoalCheckerConfig config_;
    double stable_duration_{0.0};
};

}  // namespace mobile_perception::mobile_control
