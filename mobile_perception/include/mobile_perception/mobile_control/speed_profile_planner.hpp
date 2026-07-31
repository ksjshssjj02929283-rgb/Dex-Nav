// ----------------------------------------------------------------------------
// Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
//
// All rights reserved.
// ----------------------------------------------------------------------------

#pragma once

#include <string>
#include <vector>

namespace mobile_perception::mobile_control {

struct SpeedProfileRequest {
    double distance{0.0};
    double initial_velocity{0.0};
    double initial_acceleration{0.0};
    double target_velocity{0.0};
    double max_velocity{0.35};
    double max_acceleration{0.5};
    double max_deceleration{0.5};
    double max_jerk{1.0};
    double sample_period{0.05};
    double max_duration{30.0};
};

struct SpeedProfilePoint {
    double position{0.0};
    double velocity{0.0};
    double acceleration{0.0};
    double jerk{0.0};
    double relative_time{0.0};
};

struct SpeedProfile {
    std::vector<SpeedProfilePoint> points;
    double duration{0.0};
};

struct SpeedProfileSample {
    double position{0.0};
    double velocity{0.0};
    double acceleration{0.0};
    double jerk{0.0};
    double relative_time{0.0};
};

struct TerminalSlowdownConstraint {
    double motion{0.0};
    double velocity{0.0};
};

class SpeedProfilePlanner {
public:
    virtual ~SpeedProfilePlanner() = default;

    virtual bool plan(const SpeedProfileRequest& request, SpeedProfile& profile, std::string& error) = 0;
};

bool planSpeedProfileWithTerminalSlowdown(SpeedProfilePlanner& planner,
                                          const SpeedProfileRequest& request,
                                          const TerminalSlowdownConstraint& slowdown,
                                          SpeedProfile& profile,
                                          std::string& error);

SpeedProfileSample sampleSpeedProfileByPosition(const SpeedProfile& profile, double position);

SpeedProfileSample sampleSpeedProfileByTime(const SpeedProfile& profile, double relative_time);

SpeedProfileSample sampleSpeedProfileForTracking(const SpeedProfile& profile,
                                                 double position,
                                                 double elapsed_time,
                                                 double max_preview_time);

}  // namespace mobile_perception::mobile_control
