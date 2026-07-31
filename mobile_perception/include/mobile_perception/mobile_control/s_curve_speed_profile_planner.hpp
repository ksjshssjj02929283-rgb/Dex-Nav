// ----------------------------------------------------------------------------
// Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
//
// All rights reserved.
// ----------------------------------------------------------------------------

#pragma once

#include "mobile_perception/mobile_control/speed_profile_planner.hpp"

namespace mobile_perception::mobile_control {

class SCurveSpeedProfilePlanner : public SpeedProfilePlanner {
public:
    bool plan(const SpeedProfileRequest& request, SpeedProfile& profile, std::string& error) override;
};

}  // namespace mobile_perception::mobile_control
