// Copyright 2026 DEXE Robot Engineer
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

#ifndef ROBOT_STORYTELLING__STORY_EVALUATOR_HPP_
#define ROBOT_STORYTELLING__STORY_EVALUATOR_HPP_

#include <vector>

#include "nav_interfaces/msg/area_info.hpp"
#include "nav_interfaces/msg/room_info.hpp"
#include "nav_interfaces/msg/story.hpp"

namespace robot_storytelling
{

// Evaluate semantic events triggered by the robot at (rx, ry).
// MVP three tellers: IN_ROOM / IN_SPEED_LIMIT / CLOSE_TO_CHARGER.
// Pure function with no ROS node dependency, convenient for unit tests.
std::vector<nav_interfaces::msg::Story> evaluateStories(
  double rx, double ry,
  const std::vector<nav_interfaces::msg::AreaInfo> & areas,
  const std::vector<nav_interfaces::msg::RoomInfo> & rooms,
  double charger_radius);

// Point-in-2D-polygon test (ray casting). polygon is geometry_msgs/Pose2D[].
bool pointInPolygon(
  double px, double py,
  const std::vector<geometry_msgs::msg::Pose2D> & polygon);

}  // namespace robot_storytelling

#endif  // ROBOT_STORYTELLING__STORY_EVALUATOR_HPP_
