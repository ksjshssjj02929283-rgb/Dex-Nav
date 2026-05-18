// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cmath>
#include <utility>
#include <vector>

#include "nav2_route_planner/astar_searcher.hpp"
#include "nav2_route_planner/path_interpolator.hpp"
#include "nav2_route_planner/start_goal_projector.hpp"
#include "nav2_route_planner/topo_graph.hpp"

using nav2_route_planner::AStarSearcher;
using nav2_route_planner::PathInterpolator;
using nav2_route_planner::StartGoalProjector;
using nav2_route_planner::TopoGraph;
using nav2_route_planner::TopoNode;
using nav2_route_planner::TopoEdge;
using nav2_route_planner::EdgeGeometry;
using nav2_route_planner::NodeKind;

namespace
{
geometry_msgs::msg::PoseStamped pose(double x, double y, double yaw = 0.0)
{
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.w = std::cos(yaw / 2.0);
  p.pose.orientation.z = std::sin(yaw / 2.0);
  return p;
}
}  // namespace

TEST(PathInterpolatorTest, BernsteinEndpointsExact)
{
  auto pts = PathInterpolator::sampleBezier(
    {{0, 0}, {1, 1}, {3, 1}, {4, 0}}, 50);
  ASSERT_EQ(pts.size(), 51u);
  EXPECT_NEAR(pts.front().first, 0.0, 1e-6);
  EXPECT_NEAR(pts.front().second, 0.0, 1e-6);
  EXPECT_NEAR(pts.back().first, 4.0, 1e-6);
  EXPECT_NEAR(pts.back().second, 0.0, 1e-6);
}

TEST(PathInterpolatorTest, EndToEndStraightLine)
{
  TopoGraph g;
  std::vector<TopoNode> ns;
  TopoNode a; a.id = "A"; a.kind = NodeKind::NAV_POINT; a.x = 0; a.y = 0; ns.push_back(a);
  TopoNode b; b.id = "B"; b.kind = NodeKind::NAV_POINT; b.x = 10; b.y = 0; ns.push_back(b);
  std::vector<TopoEdge> es;
  TopoEdge e; e.id = "e"; e.from_id = "A"; e.to_id = "B";
  e.geometry = EdgeGeometry::LINE; e.bidirectional = true;
  es.push_back(e);
  g.rebuildFrom(std::move(ns), std::move(es));

  auto lock = g.lockShared();

  // Search A → B
  AStarSearcher::Request req;
  req.start.type = StartGoalProjector::ResultType::ON_NODE;
  req.start.id = "A";
  req.goal_node_id = "B";
  auto sr = AStarSearcher().search(req, g);
  ASSERT_TRUE(sr.success);

  // Interpolate
  PathInterpolator::Params pp;
  pp.waypoints_per_meter = 10;  // 10 samples / m for a snappier test
  PathInterpolator interp(pp);
  StartGoalProjector::Result start_proj;
  start_proj.type = StartGoalProjector::ResultType::ON_NODE;
  start_proj.id = "A";
  StartGoalProjector::Result goal_proj;
  goal_proj.type = StartGoalProjector::ResultType::ON_NODE;
  goal_proj.id = "B";
  auto plan = interp.interpolate(pose(0, 0), pose(10, 0),
      start_proj, goal_proj, sr, g);

  // Should have a transition seg + edge seg + goal alignment seg.
  EXPECT_GE(plan.segments.size(), 2u);
  // The flattened path should run roughly from (0,0) to (10,0).
  auto path = plan.toNavPath("map", rclcpp::Time(0, 0, RCL_ROS_TIME), false);
  ASSERT_FALSE(path.poses.empty());
  EXPECT_NEAR(path.poses.front().pose.position.x, 0.0, 1e-3);
  EXPECT_NEAR(path.poses.back().pose.position.x, 10.0, 1e-3);
  EXPECT_GE(path.poses.size(), 100u);  // density check
}

TEST(PathInterpolatorTest, ToMsgRoundtrip)
{
  TopoGraph g;
  std::vector<TopoNode> ns;
  TopoNode a; a.id = "A"; a.kind = NodeKind::NAV_POINT; a.x = 0; a.y = 0;
  a.floor = "1"; ns.push_back(a);
  TopoNode b; b.id = "B"; b.kind = NodeKind::NAV_POINT; b.x = 5; b.y = 0;
  b.floor = "1"; ns.push_back(b);
  std::vector<TopoEdge> es;
  TopoEdge e; e.id = "e"; e.from_id = "A"; e.to_id = "B";
  e.geometry = EdgeGeometry::LINE; e.bidirectional = true;
  e.lane_subtype = "walkway"; e.max_speed = 0.8f; e.floor = "1";
  es.push_back(e);
  g.rebuildFrom(std::move(ns), std::move(es));

  auto lock = g.lockShared();
  AStarSearcher::Request req;
  req.start.type = StartGoalProjector::ResultType::ON_NODE;
  req.start.id = "A";
  req.goal_node_id = "B";
  auto sr = AStarSearcher().search(req, g);
  ASSERT_TRUE(sr.success);

  PathInterpolator interp(PathInterpolator::Params{});
  StartGoalProjector::Result sp;
  sp.type = StartGoalProjector::ResultType::ON_NODE;
  sp.id = "A";
  StartGoalProjector::Result gp;
  gp.type = StartGoalProjector::ResultType::ON_NODE;
  gp.id = "B";
  auto plan = interp.interpolate(pose(0, 0), pose(5, 0), sp, gp, sr, g);
  auto msg = plan.toMsg("map", rclcpp::Time(0, 0, RCL_ROS_TIME));
  EXPECT_EQ(msg.header.frame_id, "map");
  EXPECT_GE(msg.segments.size(), 2u);
  // At least one segment carries the walkway subtype + speed.
  bool found = false;
  for (const auto & s : msg.segments) {
    if (s.lane_subtype == "walkway" && s.max_speed > 0.0f) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
