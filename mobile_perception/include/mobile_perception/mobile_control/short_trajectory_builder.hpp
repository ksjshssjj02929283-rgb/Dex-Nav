// ----------------------------------------------------------------------------
// Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
//
// All rights reserved.
// ----------------------------------------------------------------------------

#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_interfaces/msg/robot_trajectory.hpp>
#include <nav_msgs/msg/path.hpp>
#include <string>

#include "mobile_perception/mobile_control/speed_profile_planner.hpp"

namespace mobile_perception::mobile_control {

enum class MotionMode { Straight, Rotation };

struct ShortTrajectory {
    nav_msgs::msg::Path path;
    nav_interfaces::msg::RobotTrajectory trajectory;
    SpeedProfile motion_profile;
    MotionMode mode{MotionMode::Straight};
    double direction{1.0};
    double total_motion{0.0};
};

class ShortTrajectoryBuilder {
public:
    static ShortTrajectory build(const geometry_msgs::msg::PoseStamped& start,
                                 const geometry_msgs::msg::PoseStamped& goal,
                                 const SpeedProfile& profile,
                                 MotionMode mode,
                                 double direction,
                                 const std::string& plan_id);

    static bool samplePoseByMotion(const ShortTrajectory& trajectory,
                                   double motion_position,
                                   geometry_msgs::msg::PoseStamped& pose);
};

}  // namespace mobile_perception::mobile_control
