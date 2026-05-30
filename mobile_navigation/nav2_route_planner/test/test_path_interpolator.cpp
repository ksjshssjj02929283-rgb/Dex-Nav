// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav2_route_planner/astar_searcher.hpp"
#include "nav2_route_planner/graph_augmenter.hpp"
#include "nav2_route_planner/path_interpolator.hpp"
#include "nav2_route_planner/start_goal_projector.hpp"
#include "nav2_route_planner/topo_graph.hpp"

using nav2_route_planner::AStarSearcher;
using nav2_route_planner::buildAugmentedGraph;
using nav2_route_planner::EdgeGeometry;
using nav2_route_planner::NodeKind;
using nav2_route_planner::PathInterpolator;
using nav2_route_planner::StartGoalProjector;
using nav2_route_planner::TopoEdge;
using nav2_route_planner::TopoGraph;
using nav2_route_planner::TopoNode;

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

StartGoalProjector::Result onEdgeLine(
  const std::string & id, double px, double py)
{
  StartGoalProjector::Result r;
  r.type = StartGoalProjector::ResultType::ON_EDGE;
  r.id = id;
  r.projected_xy = {px, py};
  r.edge_sample_index = 0;
  r.edge_total_samples = 2;
  return r;
}

// Run plain node-to-node A* between two ids on `g`.
AStarSearcher::Result search(
  const TopoGraph & g, const std::string & from, const std::string & to)
{
  AStarSearcher::Request req;
  req.start_node_id = from;
  req.goal_node_id = to;
  return AStarSearcher().search(req, g);
}

double distToEndpoints(
  double x, double y, double ax, double ay, double bx, double by)
{
  return std::min(std::hypot(x - ax, y - ay), std::hypot(x - bx, y - by));
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
  e.geometry = EdgeGeometry::LINE; e.bidirectional = true; es.push_back(e);
  g.rebuildFrom(std::move(ns), std::move(es));
  auto lock = g.lockShared();

  auto sr = search(g, "A", "B");
  ASSERT_TRUE(sr.success);

  PathInterpolator::Params pp; pp.waypoints_per_meter = 10;
  PathInterpolator interp(pp);
  auto plan = interp.interpolate(pose(0, 0), pose(10, 0), sr, g);

  EXPECT_GE(plan.segments.size(), 2u);
  auto path = plan.toNavPath("map", rclcpp::Time(0, 0, RCL_ROS_TIME));
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

  auto sr = search(g, "A", "B");
  ASSERT_TRUE(sr.success);

  PathInterpolator interp(PathInterpolator::Params{});
  auto plan = interp.interpolate(pose(0, 0), pose(5, 0), sr, g);
  auto msg = plan.toMsg("map", rclcpp::Time(0, 0, RCL_ROS_TIME));
  EXPECT_EQ(msg.header.frame_id, "map");
  EXPECT_GE(msg.segments.size(), 2u);
  // The edge segment carries the walkway subtype + speed.
  bool found = false;
  for (const auto & s : msg.segments) {
    if (s.lane_subtype == "walkway" && s.max_speed > 0.0f) {found = true;}
  }
  EXPECT_TRUE(found);
}

// Same-edge end-to-end (augment → A* → interpolate): start and goal on one
// LINE edge must produce a path that runs DIRECTLY between the projections
// and never detours out to an endpoint NavPoint.
TEST(PathInterpolatorTest, SameEdgeLineNoDetour)
{
  auto g = std::make_shared<TopoGraph>();
  std::vector<TopoNode> ns;
  TopoNode a; a.id = "A"; a.kind = NodeKind::NAV_POINT; a.x = 0; a.y = 0; ns.push_back(a);
  TopoNode b; b.id = "B"; b.kind = NodeKind::NAV_POINT; b.x = 10; b.y = 0; ns.push_back(b);
  std::vector<TopoEdge> es;
  TopoEdge e; e.id = "e"; e.from_id = "A"; e.to_id = "B";
  e.geometry = EdgeGeometry::LINE; e.bidirectional = true;
  e.lane_subtype = "corridor"; es.push_back(e);
  g->rebuildFrom(std::move(ns), std::move(es));
  auto lock = g->lockShared();

  auto aug = buildAugmentedGraph(
    g, onEdgeLine("e", 2.0, 0.0), onEdgeLine("e", 8.0, 0.0), 40);
  auto sr = search(*aug.graph, aug.start_id, aug.goal_id);
  ASSERT_TRUE(sr.success) << sr.failure_reason;

  PathInterpolator::Params pp; pp.waypoints_per_meter = 10;
  PathInterpolator interp(pp);
  auto plan = interp.interpolate(pose(2, 0.5), pose(8, -0.5), sr, *aug.graph);
  auto path = plan.toNavPath("map", rclcpp::Time(0, 0, RCL_ROS_TIME));
  ASSERT_FALSE(path.poses.empty());
  EXPECT_NEAR(path.poses.front().pose.position.x, 2.0, 1e-2);
  EXPECT_NEAR(path.poses.back().pose.position.x, 8.0, 1e-2);
  for (const auto & p : path.poses) {
    EXPECT_GE(p.pose.position.x, 2.0 - 1e-2);
    EXPECT_LE(p.pose.position.x, 8.0 + 1e-2);
  }
}

// Same-edge on a BEZIER edge: the interpolated path follows the curve and
// never approaches either endpoint NavPoint.
TEST(PathInterpolatorTest, SameEdgeBezierFollowsCurve)
{
  const int kSamples = 20;
  auto g = std::make_shared<TopoGraph>();
  std::vector<TopoNode> ns;
  TopoNode a; a.id = "A"; a.kind = NodeKind::NAV_POINT; a.x = 0; a.y = 0; ns.push_back(a);
  TopoNode b; b.id = "B"; b.kind = NodeKind::NAV_POINT; b.x = 10; b.y = 0; ns.push_back(b);
  std::vector<TopoEdge> es;
  TopoEdge e; e.id = "e"; e.from_id = "A"; e.to_id = "B";
  e.geometry = EdgeGeometry::BEZIER; e.bidirectional = true;
  e.control_points = {{0, 0}, {5, 5}, {10, 0}};
  es.push_back(e);
  g->rebuildFrom(std::move(ns), std::move(es));
  auto lock = g->lockShared();

  StartGoalProjector::Params projp; projp.bezier_samples_per_edge = kSamples;
  StartGoalProjector projector(projp);
  auto start = pose(2.5, 2.2);
  auto goal = pose(7.5, 2.2);
  auto sp = projector.projectPreferEdge(start, *g);
  auto gp = projector.projectPreferEdge(goal, *g);
  ASSERT_EQ(sp.type, StartGoalProjector::ResultType::ON_EDGE);
  ASSERT_EQ(gp.type, StartGoalProjector::ResultType::ON_EDGE);

  auto aug = buildAugmentedGraph(g, sp, gp, kSamples);
  auto sr = search(*aug.graph, aug.start_id, aug.goal_id);
  ASSERT_TRUE(sr.success) << sr.failure_reason;

  PathInterpolator::Params pp;
  pp.waypoints_per_meter = 10; pp.bezier_samples_per_edge = kSamples;
  PathInterpolator interp(pp);
  auto plan = interp.interpolate(start, goal, sr, *aug.graph);
  auto path = plan.toNavPath("map", rclcpp::Time(0, 0, RCL_ROS_TIME));
  ASSERT_GE(path.poses.size(), 2u);
  for (const auto & p : path.poses) {
    EXPECT_GT(distToEndpoints(
        p.pose.position.x, p.pose.position.y, 0, 0, 10, 0), 0.8);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
