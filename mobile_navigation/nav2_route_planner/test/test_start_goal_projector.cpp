// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <memory>
#include <utility>
#include <vector>

#include "nav2_route_planner/start_goal_projector.hpp"
#include "nav2_route_planner/topo_graph.hpp"

using nav2_route_planner::StartGoalProjector;
using nav2_route_planner::TopoGraph;
using nav2_route_planner::TopoNode;
using nav2_route_planner::TopoEdge;
using nav2_route_planner::EdgeGeometry;
using nav2_route_planner::NodeKind;

namespace
{
geometry_msgs::msg::PoseStamped pose(double x, double y)
{
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.w = 1.0;
  return p;
}

std::unique_ptr<TopoGraph> buildSimple()
{
  auto g = std::make_unique<TopoGraph>();
  std::vector<TopoNode> nodes;
  TopoNode a; a.id = "A"; a.kind = NodeKind::NAV_POINT; a.x = 0.0; a.y = 0.0; nodes.push_back(a);
  TopoNode b; b.id = "B"; b.kind = NodeKind::NAV_POINT; b.x = 10.0; b.y = 0.0; nodes.push_back(b);
  std::vector<TopoEdge> edges;
  TopoEdge e; e.id = "e"; e.from_id = "A"; e.to_id = "B";
  e.geometry = EdgeGeometry::LINE;
  edges.push_back(e);
  g->rebuildFrom(std::move(nodes), std::move(edges));
  return g;
}
}  // namespace

TEST(ProjectorTest, VerticalProjectInsideSegment)
{
  bool on_seg = false;
  auto p = StartGoalProjector::verticalProject(3.0, 1.0, 0.0, 0.0, 10.0, 0.0, on_seg);
  EXPECT_TRUE(on_seg);
  EXPECT_NEAR(p.first, 3.0, 1e-6);
  EXPECT_NEAR(p.second, 0.0, 1e-6);
}

TEST(ProjectorTest, VerticalProjectBeforeSegment)
{
  bool on_seg = true;
  auto p = StartGoalProjector::verticalProject(-1.0, 1.0, 0.0, 0.0, 10.0, 0.0, on_seg);
  EXPECT_FALSE(on_seg);
  EXPECT_NEAR(p.first, 0.0, 1e-6);
}

TEST(ProjectorTest, OnNodeWhenStartCloseToNavPoint)
{
  auto g = buildSimple();
  auto lock = g->lockShared();
  StartGoalProjector::Params pr;
  StartGoalProjector proj(pr);
  auto r = proj.project(pose(0.05, 0.05), *g);
  EXPECT_EQ(r.type, StartGoalProjector::ResultType::ON_NODE);
  EXPECT_EQ(r.id, "A");
}

TEST(ProjectorTest, OnEdgeWhenBetweenNodes)
{
  auto g = buildSimple();
  auto lock = g->lockShared();
  StartGoalProjector::Params pr;
  StartGoalProjector proj(pr);
  auto r = proj.project(pose(5.0, 0.3), *g);
  EXPECT_EQ(r.type, StartGoalProjector::ResultType::ON_EDGE);
  EXPECT_EQ(r.id, "e");
  EXPECT_NEAR(r.projected_xy.first, 5.0, 1e-6);
  EXPECT_NEAR(r.projected_xy.second, 0.0, 1e-6);
}

TEST(ProjectorTest, InvalidWhenTooFar)
{
  auto g = buildSimple();
  auto lock = g->lockShared();
  StartGoalProjector::Params pr;
  pr.node_threshold_m = 0.5;
  pr.edge_threshold_m = 1.0;
  StartGoalProjector proj(pr);
  auto r = proj.project(pose(5.0, 5.0), *g);
  EXPECT_EQ(r.type, StartGoalProjector::ResultType::INVALID);
}

TEST(ProjectorTest, ProjectToNodeRequiresClose)
{
  auto g = buildSimple();
  auto lock = g->lockShared();
  StartGoalProjector::Params pr;
  StartGoalProjector proj(pr);
  EXPECT_TRUE(proj.projectToNode(pose(0.05, 0.05), *g).has_value());
  EXPECT_EQ(*proj.projectToNode(pose(10.05, 0.05), *g), "B");
  EXPECT_FALSE(proj.projectToNode(pose(5.0, 0.0), *g).has_value());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
