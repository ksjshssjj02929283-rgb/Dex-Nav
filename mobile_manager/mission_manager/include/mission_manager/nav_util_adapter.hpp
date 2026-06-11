#ifndef MISSION_MANAGER__NAV_UTIL_ADAPTER_HPP_
#define MISSION_MANAGER__NAV_UTIL_ADAPTER_HPP_

#include <chrono>
#include <cmath>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tf2/utils.h"

namespace nav_util {

/**
 * @brief Simple TicToc timer class for measuring elapsed time
 * This is a lightweight adapter to replace the original nav_util::TicToc
 */
class TicToc {
public:
    TicToc() { tic(); }

    void tic() { start_ = std::chrono::steady_clock::now(); }

    /**
     * @brief Returns elapsed time in microseconds
     * @return double elapsed time in microseconds
     */
    double toc() const {
        auto end = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start_);
        return static_cast<double>(duration.count());
    }

private:
    std::chrono::steady_clock::time_point start_;
};

/**
 * @brief Get the euclidean distance between two poses
 * This wraps nav2_util::geometry_utils::euclidean_distance
 */
inline double euclidean_distance(const geometry_msgs::msg::PoseStamped& pos1,
                                 const geometry_msgs::msg::PoseStamped& pos2,
                                 const bool is_3d = false) {
    return nav2_util::geometry_utils::euclidean_distance(pos1, pos2, is_3d);
}

/**
 * @brief Normalize angle to [-pi, pi]
 */
inline double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

/**
 * @brief Check if two poses are the same within given thresholds
 * @param pose1 First pose
 * @param pose2 Second pose
 * @param dist_threshold Distance threshold in meters
 * @param yaw_threshold Yaw angle threshold in radians
 * @return true if poses are considered the same
 */
inline bool is_same_pose(const geometry_msgs::msg::PoseStamped& pose1,
                         const geometry_msgs::msg::PoseStamped& pose2,
                         double dist_threshold = 0.1,
                         double yaw_threshold = 0.1) {
    // Check distance using nav2_util
    double dist = nav2_util::geometry_utils::euclidean_distance(pose1, pose2, false);
    if (dist > dist_threshold) {
        return false;
    }

    // Check yaw angle
    double yaw1 = tf2::getYaw(pose1.pose.orientation);
    double yaw2 = tf2::getYaw(pose2.pose.orientation);
    double yaw_diff = std::abs(normalize_angle(yaw1 - yaw2));

    if (yaw_diff > yaw_threshold) {
        return false;
    }

    return true;
}

}  // namespace nav_util

#endif  // MISSION_MANAGER__NAV_UTIL_ADAPTER_HPP_
