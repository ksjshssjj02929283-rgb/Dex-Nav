// ----------------------------------------------------------------------------
// Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
//
// All rights reserved.
// ----------------------------------------------------------------------------

#include "mobile_perception/mobile_control/short_trajectory_builder.hpp"

#include <tf2/utils.h>

#include <algorithm>
#include <builtin_interfaces/msg/duration.hpp>
#include <cmath>
#include <cstdint>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace mobile_perception::mobile_control {

namespace {

builtin_interfaces::msg::Duration toDuration(double seconds) {
    const auto nanoseconds = static_cast<int64_t>(std::llround(std::max(0.0, seconds) * 1e9));
    builtin_interfaces::msg::Duration duration;
    duration.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
    duration.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
    return duration;
}

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw) {
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw);
    return tf2::toMsg(quaternion);
}

double normalizedAngle(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

}  // namespace

ShortTrajectory ShortTrajectoryBuilder::build(const geometry_msgs::msg::PoseStamped& start,
                                              const geometry_msgs::msg::PoseStamped& goal,
                                              const SpeedProfile& profile,
                                              MotionMode mode,
                                              double direction,
                                              const std::string& plan_id) {
    ShortTrajectory output;
    output.mode = mode;
    output.direction = direction < 0.0 ? -1.0 : 1.0;
    output.motion_profile = profile;
    output.total_motion = profile.points.empty() ? 0.0 : profile.points.back().position;

    output.path.header = start.header;
    output.trajectory.header = start.header;
    output.trajectory.plan_id = plan_id;

    geometry_msgs::msg::PoseStamped path_goal = goal;
    path_goal.header = start.header;
    if (mode == MotionMode::Rotation) {
        path_goal.pose.position = start.pose.position;
    }
    output.path.poses = {start, path_goal};

    const double start_yaw = tf2::getYaw(start.pose.orientation);
    const double goal_yaw = tf2::getYaw(goal.pose.orientation);
    const double straight_yaw_delta = normalizedAngle(goal_yaw - start_yaw);
    const double total_motion = std::max(output.total_motion, 1e-12);

    output.trajectory.points.reserve(profile.points.size());
    for (const auto& profile_point : profile.points) {
        const double ratio = std::clamp(profile_point.position / total_motion, 0.0, 1.0);
        nav_interfaces::msg::TrajectoryPoint point;
        point.pose.header = start.header;

        if (mode == MotionMode::Straight) {
            point.pose.pose.position.x = start.pose.position.x + ratio * (goal.pose.position.x - start.pose.position.x);
            point.pose.pose.position.y = start.pose.position.y + ratio * (goal.pose.position.y - start.pose.position.y);
            point.pose.pose.position.z = start.pose.position.z + ratio * (goal.pose.position.z - start.pose.position.z);
            point.pose.pose.orientation = quaternionFromYaw(start_yaw + ratio * straight_yaw_delta);
            point.longitudinal_velocity = output.direction * profile_point.velocity;
            point.acceleration = output.direction * profile_point.acceleration;
            point.jerk = output.direction * profile_point.jerk;
        } else {
            point.pose.pose.position = start.pose.position;
            point.pose.pose.orientation = quaternionFromYaw(start_yaw + output.direction * profile_point.position);
            point.angular_velocity = output.direction * profile_point.velocity;
        }

        point.speed_hint = static_cast<float>(profile_point.velocity);
        point.relative_time = toDuration(profile_point.relative_time);
        output.trajectory.points.push_back(std::move(point));
    }

    return output;
}

bool ShortTrajectoryBuilder::samplePoseByMotion(const ShortTrajectory& trajectory,
                                                double motion_position,
                                                geometry_msgs::msg::PoseStamped& pose) {
    const auto& profile_points = trajectory.motion_profile.points;
    const auto& trajectory_points = trajectory.trajectory.points;
    if (!std::isfinite(motion_position) || profile_points.empty() ||
        profile_points.size() != trajectory_points.size()) {
        return false;
    }

    if (motion_position <= profile_points.front().position || profile_points.size() == 1U) {
        pose = trajectory_points.front().pose;
        return true;
    }
    if (motion_position >= profile_points.back().position) {
        pose = trajectory_points.back().pose;
        return true;
    }

    const auto upper =
        std::upper_bound(profile_points.begin(), profile_points.end(), motion_position,
                         [](double value, const SpeedProfilePoint& point) { return value < point.position; });
    const auto upper_index = static_cast<size_t>(std::distance(profile_points.begin(), upper));
    const auto lower_index = upper_index - 1U;
    const double motion_delta = profile_points[upper_index].position - profile_points[lower_index].position;
    const double ratio =
        motion_delta > 1e-12 ? (motion_position - profile_points[lower_index].position) / motion_delta : 0.0;

    const auto& lower_pose = trajectory_points[lower_index].pose;
    const auto& upper_pose = trajectory_points[upper_index].pose;
    pose = lower_pose;
    pose.pose.position.x += ratio * (upper_pose.pose.position.x - lower_pose.pose.position.x);
    pose.pose.position.y += ratio * (upper_pose.pose.position.y - lower_pose.pose.position.y);
    pose.pose.position.z += ratio * (upper_pose.pose.position.z - lower_pose.pose.position.z);
    const double lower_yaw = tf2::getYaw(lower_pose.pose.orientation);
    const double upper_yaw = tf2::getYaw(upper_pose.pose.orientation);
    pose.pose.orientation = quaternionFromYaw(lower_yaw + ratio * normalizedAngle(upper_yaw - lower_yaw));
    return true;
}

}  // namespace mobile_perception::mobile_control
