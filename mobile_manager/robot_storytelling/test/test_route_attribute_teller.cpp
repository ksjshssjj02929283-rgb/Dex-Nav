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

#include "robot_storytelling/route_attribute_teller.hpp"

using robot_storytelling::evaluateBezier;
using robot_storytelling::findNearestEdge;
using robot_storytelling::pointToPolylineDistance;
using robot_storytelling::RouteAttributeTeller;
using robot_storytelling::sampleEdge;

namespace
{
geometry_msgs::msg::Point pt(double x, double y)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  return p;
}

// 一条从 (x0,y0) 到 (x1,y1) 的直线边（LINE：2 个 control_points）。
nav_interfaces::msg::RouteEdge lineEdge(
  const std::string & id, double x0, double y0, double x1, double y1,
  float max_speed = 1.0f)
{
  nav_interfaces::msg::RouteEdge e;
  e.id = id;
  e.edge_type = "LINE";
  e.max_speed = max_speed;
  e.obs_policy = "STOP";
  e.control_points = {pt(x0, y0), pt(x1, y1)};
  return e;
}
}  // namespace

// ── 几何纯函数 ──────────────────────────────────────────────────────

TEST(RouteAttributeTellerGeometry, BezierDegenerateToLineLerp)
{
  std::vector<geometry_msgs::msg::Point> cps = {pt(0.0, 0.0), pt(10.0, 0.0)};
  auto mid = evaluateBezier(cps, 0.5);
  EXPECT_NEAR(mid.x, 5.0, 1e-9);
  EXPECT_NEAR(mid.y, 0.0, 1e-9);
}

TEST(RouteAttributeTellerGeometry, BezierQuadraticMidpoint)
{
  // 三点二次 Bezier：(0,0) (5,10) (10,0)，t=0.5 应在 (5,5)。
  std::vector<geometry_msgs::msg::Point> cps = {pt(0.0, 0.0), pt(5.0, 10.0), pt(10.0, 0.0)};
  auto mid = evaluateBezier(cps, 0.5);
  EXPECT_NEAR(mid.x, 5.0, 1e-9);
  EXPECT_NEAR(mid.y, 5.0, 1e-9);
}

TEST(RouteAttributeTellerGeometry, SampleEdgeTooFewControlPointsYieldsEmpty)
{
  nav_interfaces::msg::RouteEdge e;
  e.control_points = {pt(0.0, 0.0)};
  EXPECT_TRUE(sampleEdge(e, 10).empty());
}

TEST(RouteAttributeTellerGeometry, PointToPolylineDistanceBasic)
{
  std::vector<std::pair<double, double>> polyline = {{0.0, 0.0}, {10.0, 0.0}};
  EXPECT_NEAR(pointToPolylineDistance(5.0, 3.0, polyline), 3.0, 1e-6);
  EXPECT_NEAR(pointToPolylineDistance(-2.0, 0.0, polyline), 2.0, 1e-6);  // 端点外，退化到端点距离
}

TEST(RouteAttributeTellerGeometry, FindNearestEdgeWithinThreshold)
{
  auto e1 = lineEdge("e1", 0.0, 0.0, 10.0, 0.0);
  auto e2 = lineEdge("e2", 0.0, 5.0, 10.0, 5.0);
  auto nearest = findNearestEdge(5.0, 0.2, {e1, e2}, 0.5);
  ASSERT_TRUE(nearest.has_value());
  EXPECT_EQ(*nearest, 0u);  // e1 更近
}

TEST(RouteAttributeTellerGeometry, FindNearestEdgeOutsideThresholdReturnsNullopt)
{
  auto e1 = lineEdge("e1", 0.0, 0.0, 10.0, 0.0);
  auto nearest = findNearestEdge(5.0, 3.0, {e1}, 0.5);  // 距 e1 有 3m，阈值 0.5m
  EXPECT_FALSE(nearest.has_value());
}

TEST(RouteAttributeTellerGeometry, FindNearestEdgeEmptyGraphReturnsNullopt)
{
  EXPECT_FALSE(findNearestEdge(0.0, 0.0, {}, 1.0).has_value());
}

// ── 带滞后状态机的 updateWithEdges ──────────────────────────────────

TEST(RouteAttributeTeller, EmptyGraphYieldsNoStory)
{
  RouteAttributeTeller teller;
  EXPECT_FALSE(teller.updateWithEdges(0.0, 0.0, {}).has_value());
}

TEST(RouteAttributeTeller, ColdStartAcceptsNearestEdgeImmediately)
{
  RouteAttributeTeller teller;
  auto e1 = lineEdge("e1", 0.0, 0.0, 10.0, 0.0, 0.8f);
  auto story = teller.updateWithEdges(5.0, 0.1, {e1});
  ASSERT_TRUE(story.has_value());
  EXPECT_EQ(story->event_type, "CURRENT_EDGE");
  EXPECT_EQ(story->source_id, "e1");
}

TEST(RouteAttributeTeller, OutOfThresholdClearsCurrentEdge)
{
  RouteAttributeTeller teller;
  auto e1 = lineEdge("e1", 0.0, 0.0, 10.0, 0.0);
  ASSERT_TRUE(teller.updateWithEdges(5.0, 0.1, {e1}).has_value());
  // 机器人离开图（如避障绕远），距 e1 超出默认阈值 0.5m
  EXPECT_FALSE(teller.updateWithEdges(5.0, 5.0, {e1}).has_value());
}

TEST(RouteAttributeTeller, SwitchRequiresConsecutiveHysteresisFrames)
{
  RouteAttributeTeller teller;
  auto e1 = lineEdge("e1", 0.0, 0.0, 10.0, 0.0);
  auto e2 = lineEdge("e2", 0.0, 0.3, 10.0, 0.3);  // 与 e1 平行，间距 0.3m

  // 冷启动确认在 e1 上。
  auto first = teller.updateWithEdges(5.0, 0.05, {e1, e2});
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->source_id, "e1");

  // 单帧噪声瞬间更靠近 e2：默认 hysteresis=3，1 帧不足以切换，仍应报告 e1。
  auto noisy = teller.updateWithEdges(5.0, 0.25, {e1, e2});
  ASSERT_TRUE(noisy.has_value());
  EXPECT_EQ(noisy->source_id, "e1");

  // 再来两帧持续更靠近 e2（累计 3 帧）——达到阈值，切换到 e2。
  teller.updateWithEdges(5.0, 0.25, {e1, e2});
  auto switched = teller.updateWithEdges(5.0, 0.25, {e1, e2});
  ASSERT_TRUE(switched.has_value());
  EXPECT_EQ(switched->source_id, "e2");
}

TEST(RouteAttributeTeller, StoryTagsCarryMaxSpeedAndObsPolicy)
{
  RouteAttributeTeller teller;
  auto e1 = lineEdge("e1", 0.0, 0.0, 10.0, 0.0, 1.25f);
  auto story = teller.updateWithEdges(5.0, 0.0, {e1});
  ASSERT_TRUE(story.has_value());
  ASSERT_EQ(story->tags.size(), 2u);
  EXPECT_NE(story->tags[0].find("max_speed"), std::string::npos);
  EXPECT_NE(story->tags[1].find("obs_policy"), std::string::npos);
}
