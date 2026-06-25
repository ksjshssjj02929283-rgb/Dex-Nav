// Copyright (c) 2024 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DOCKING_BASE__UTILS_HPP_
#define DOCKING_BASE__UTILS_HPP_

// DEC-014：移除 parseDockFile / parseDockParams / getDockPoseStamped（依赖已删的 Dock/DockMap，
// 属 DockDatabase 辅助）。仅保留几何工具 l2Norm。

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "angles/angles.h"
#include "tf2/utils.h"

namespace utils
{

inline double l2Norm(const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b)
{
  double angle_a = tf2::getYaw(a.orientation);
  double angle_b = tf2::getYaw(b.orientation);
  double delta_angle = angles::shortest_angular_distance(angle_a, angle_b);
  return sqrt(
    (a.position.x - b.position.x) * (a.position.x - b.position.x) +
    (a.position.y - b.position.y) * (a.position.y - b.position.y) +
    delta_angle * delta_angle);
}

}  // namespace utils

#endif  // DOCKING_BASE__UTILS_HPP_
