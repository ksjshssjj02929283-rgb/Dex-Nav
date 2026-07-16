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

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "robot_storytelling/area_attribute_teller.hpp"

using robot_storytelling::evaluateAreaStories;
using robot_storytelling::pointInPolygon;

namespace
{
geometry_msgs::msg::Pose2D p2d(double x, double y)
{
  geometry_msgs::msg::Pose2D p;
  p.x = x;
  p.y = y;
  return p;
}

std::vector<geometry_msgs::msg::Pose2D> rect(double x0, double y0, double x1, double y1)
{
  return {p2d(x0, y0), p2d(x1, y0), p2d(x1, y1), p2d(x0, y1)};
}
}  // namespace

TEST(AreaAttributeTeller, PointInPolygonBasic)
{
  auto sq = rect(0.0, 0.0, 2.0, 2.0);
  EXPECT_TRUE(pointInPolygon(1.0, 1.0, sq));
  EXPECT_FALSE(pointInPolygon(3.0, 1.0, sq));
}

TEST(AreaAttributeTeller, InRoomEmitsStory)
{
  nav_interfaces::msg::RoomInfo room;
  room.name = "meeting_3";
  room.polygon = rect(0.0, 0.0, 2.0, 2.0);

  auto stories = evaluateAreaStories(1.0, 1.0, {}, {room}, 1.0);

  ASSERT_EQ(stories.size(), 1u);
  EXPECT_EQ(stories[0].event_type, "IN_ROOM");
  EXPECT_EQ(stories[0].source_id, "meeting_3");
}

TEST(AreaAttributeTeller, InSpeedLimitAreaEmitsStory)
{
  nav_interfaces::msg::AreaInfo area;
  area.name = "slow_zone";
  area.type = 3;
  area.polygon = rect(0.0, 0.0, 2.0, 2.0);

  auto stories = evaluateAreaStories(1.0, 1.0, {area}, {}, 1.0);

  ASSERT_EQ(stories.size(), 1u);
  EXPECT_EQ(stories[0].event_type, "IN_SPEED_LIMIT");
  EXPECT_EQ(stories[0].source_id, "slow_zone");
}

TEST(AreaAttributeTeller, CloseToChargerByDistance)
{
  nav_interfaces::msg::AreaInfo area;
  area.name = "dock_a";
  area.type = 7;
  area.polygon = rect(0.0, 0.0, 2.0, 2.0);

  auto near = evaluateAreaStories(1.5, 1.0, {area}, {}, 1.0);
  ASSERT_EQ(near.size(), 1u);
  EXPECT_EQ(near[0].event_type, "CLOSE_TO_CHARGER");
  EXPECT_EQ(near[0].source_id, "dock_a");

  auto far = evaluateAreaStories(10.0, 10.0, {area}, {}, 1.0);
  EXPECT_TRUE(far.empty());
}

TEST(AreaAttributeTeller, NoAreasNoRoomsYieldsEmpty)
{
  EXPECT_TRUE(evaluateAreaStories(1.0, 1.0, {}, {}, 1.0).empty());
}
