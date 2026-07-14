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

#include "robot_storytelling/story_evaluator.hpp"

#include <cstddef>
#include <string>

namespace robot_storytelling
{

bool pointInPolygon(
  double px, double py,
  const std::vector<geometry_msgs::msg::Pose2D> & polygon)
{
  bool inside = false;
  const std::size_t n = polygon.size();
  if (n < 3) {
    return false;
  }
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    const double xi = polygon[i].x;
    const double yi = polygon[i].y;
    const double xj = polygon[j].x;
    const double yj = polygon[j].y;
    const bool intersect =
      ((yi > py) != (yj > py)) &&
      (px < (xj - xi) * (py - yi) / (yj - yi) + xi);
    if (intersect) {
      inside = !inside;
    }
  }
  return inside;
}

namespace
{
void centroid(
  const std::vector<geometry_msgs::msg::Pose2D> & polygon,
  double & cx, double & cy)
{
  cx = 0.0;
  cy = 0.0;
  if (polygon.empty()) {
    return;
  }
  for (const auto & p : polygon) {
    cx += p.x;
    cy += p.y;
  }
  cx /= static_cast<double>(polygon.size());
  cy /= static_cast<double>(polygon.size());
}
}  // namespace

std::vector<nav_interfaces::msg::Story> evaluateStories(
  double rx, double ry,
  const std::vector<nav_interfaces::msg::AreaInfo> & areas,
  const std::vector<nav_interfaces::msg::RoomInfo> & rooms,
  double charger_radius)
{
  std::vector<nav_interfaces::msg::Story> out;

  for (const auto & room : rooms) {
    if (pointInPolygon(rx, ry, room.polygon)) {
      nav_interfaces::msg::Story s;
      s.event_type = "IN_ROOM";
      s.source_id = room.name;
      s.confidence = 1.0f;
      out.push_back(s);
    }
  }

  for (const auto & area : areas) {
    // 引用生成的 msg 常量（SSOT），避免硬编码枚举值与 .msg 失同步
    if (area.type == nav_interfaces::msg::AreaInfo::SPEED_LIMIT_AREA &&
      pointInPolygon(rx, ry, area.polygon))
    {
      nav_interfaces::msg::Story s;
      s.event_type = "IN_SPEED_LIMIT";
      s.source_id = area.name;
      s.confidence = 1.0f;
      out.push_back(s);
    }
    if (area.type == nav_interfaces::msg::AreaInfo::CHARGING_AREA) {
      double cx = 0.0;
      double cy = 0.0;
      centroid(area.polygon, cx, cy);
      const double dx = cx - rx;
      const double dy = cy - ry;
      if (dx * dx + dy * dy < charger_radius * charger_radius) {
        nav_interfaces::msg::Story s;
        s.event_type = "CLOSE_TO_CHARGER";
        s.source_id = area.name;
        s.confidence = 1.0f;
        out.push_back(s);
      }
    }
  }

  return out;
}

}  // namespace robot_storytelling
