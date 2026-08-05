// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// 平滑器插件共享工具：平滑改变点位后，姿态需按新几何重算。
#ifndef NAV2_PATH_SMOOTHERS__SMOOTHER_UTILS_HPP_
#define NAV2_PATH_SMOOTHERS__SMOOTHER_UTILS_HPP_

#include <cmath>

#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_path_smoothers
{

/// 按切向重算路径姿态：中间点取前后点连线方向，首点取首段方向；
/// 末点保留原姿态（goal 朝向语义不可被平滑改写）。
inline void recomputeOrientations(nav_msgs::msg::Path & path)
{
  const size_t n = path.poses.size();
  if (n < 2) {
    return;
  }
  for (size_t i = 0; i + 1 < n; ++i) {
    const auto & a = path.poses[i == 0 ? 0 : i - 1].pose.position;
    const auto & b = path.poses[i + 1].pose.position;
    const double yaw = std::atan2(b.y - a.y, b.x - a.x);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    path.poses[i].pose.orientation = tf2::toMsg(q);
  }
}

}  // namespace nav2_path_smoothers

#endif  // NAV2_PATH_SMOOTHERS__SMOOTHER_UTILS_HPP_
