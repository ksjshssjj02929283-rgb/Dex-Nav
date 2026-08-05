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

// Junctions must be eligible as projection targets — both for `project`
// (start) and `projectToNode` (goal). Regression for: "Goal pose does
// not match any NavPoint within 0.50m" when a Junction was the nearest
// node.
TEST(ProjectorTest, JunctionEligibleAsStartAndGoal)
{
  auto g = std::make_unique<TopoGraph>();
  std::vector<TopoNode> nodes;
  // A NavPoint and a Junction, both within node_threshold of the query.
  // The Junction is the closer one, so it must win.
  TopoNode a; a.id = "A"; a.kind = NodeKind::NAV_POINT;
  a.x = 0.0; a.y = 0.0;
  nodes.push_back(a);
  TopoNode j; j.id = "J"; j.kind = NodeKind::JUNCTION;
  j.x = 4.64; j.y = -0.19;
  nodes.push_back(j);
  std::vector<TopoEdge> edges;
  TopoEdge e; e.id = "e"; e.from_id = "A"; e.to_id = "J";
  e.geometry = EdgeGeometry::LINE;
  edges.push_back(e);
  g->rebuildFrom(std::move(nodes), std::move(edges));

  auto lock = g->lockShared();
  StartGoalProjector::Params pr;
  StartGoalProjector proj(pr);

  // Goal at the junction's own location → projectToNode returns "J".
  auto goal_id = proj.projectToNode(pose(4.64, -0.19), *g);
  ASSERT_TRUE(goal_id.has_value());
  EXPECT_EQ(*goal_id, "J");

  // project() (start path) should also pick the junction, since it's
  // the nearest node.
  auto start = proj.project(pose(4.64, -0.19), *g);
  EXPECT_EQ(start.type, StartGoalProjector::ResultType::ON_NODE);
  EXPECT_EQ(start.id, "J");
}

// ─── projectUnlimited — fair NODE/EDGE competition, no threshold ──────

// (1) Far from everything: bounded `project` returns INVALID; `projectUnlimited`
// must still return the globally nearest projection (here: EDGE, since the
// vertical projection onto a long horizontal edge beats the distance to
// either endpoint).
TEST(ProjectorUnlimitedTest, FarFromGraphPicksNearestEdge)
{
  auto g = buildSimple();
  auto lock = g->lockShared();
  StartGoalProjector::Params pr;
  StartGoalProjector proj(pr);

  // (5, 100) is 100m off the edge but 100m above the midpoint.
  // node distance = √(5² + 100²) ≈ 100.125
  // edge distance = 100.0  (vertical drop onto the segment)
  auto r = proj.project(pose(5.0, 100.0), *g);
  EXPECT_EQ(r.type, StartGoalProjector::ResultType::INVALID);

  auto u = proj.projectUnlimited(pose(5.0, 100.0), *g);
  EXPECT_EQ(u.type, StartGoalProjector::ResultType::ON_EDGE);
  EXPECT_EQ(u.id, "e");
  EXPECT_NEAR(u.projected_xy.first, 5.0, 1e-6);
  EXPECT_NEAR(u.projected_xy.second, 0.0, 1e-6);
  EXPECT_NEAR(u.distance, 100.0, 1e-6);
  // Diagnostic distances should reflect actual best-on-each-side scan.
  EXPECT_NEAR(u.best_edge_distance, 100.0, 1e-6);
  EXPECT_GT(u.best_node_distance, 100.0);
}

// (2) NODE strictly closer than any EDGE projection: NODE wins.
// We need an isolated node (not an endpoint of the edge), otherwise the
// edge's segment-best collapses to the endpoint distance and NODE/EDGE
// tie — in which case EDGE wins by the DEC-025 tie-break.
TEST(ProjectorUnlimitedTest, FarFromGraphPicksNearestNode)
{
  auto g = std::make_unique<TopoGraph>();
  std::vector<TopoNode> nodes;
  TopoNode a; a.id = "A"; a.kind = NodeKind::NAV_POINT;
  a.x = 0.0; a.y = 0.0; nodes.push_back(a);
  TopoNode b; b.id = "B"; b.kind = NodeKind::NAV_POINT;
  b.x = 10.0; b.y = 0.0; nodes.push_back(b);
  TopoNode c; c.id = "C"; c.kind = NodeKind::NAV_POINT;
  c.x = 100.0; c.y = 0.0; nodes.push_back(c);  // isolated, no edges
  std::vector<TopoEdge> edges;
  TopoEdge e; e.id = "e"; e.from_id = "A"; e.to_id = "B";
  e.geometry = EdgeGeometry::LINE;
  edges.push_back(e);
  g->rebuildFrom(std::move(nodes), std::move(edges));

  auto lock = g->lockShared();
  StartGoalProjector::Params pr;
  StartGoalProjector proj(pr);

  // pose (105, 0): distance to C(100,0) = 5; distance to edge e_ab
  // (clipped projection at B(10,0)) = 95. NODE strictly wins.
  auto u = proj.projectUnlimited(pose(105.0, 0.0), *g);
  EXPECT_EQ(u.type, StartGoalProjector::ResultType::ON_NODE);
  EXPECT_EQ(u.id, "C");
  EXPECT_NEAR(u.distance, 5.0, 1e-6);
}

// (3) Empty graph → unlimited still returns INVALID (only failure mode).
TEST(ProjectorUnlimitedTest, EmptyGraphReturnsInvalid)
{
  auto g = std::make_unique<TopoGraph>();
  g->rebuildFrom({}, {});
  auto lock = g->lockShared();
  StartGoalProjector::Params pr;
  StartGoalProjector proj(pr);

  auto u = proj.projectUnlimited(pose(0.0, 0.0), *g);
  EXPECT_EQ(u.type, StartGoalProjector::ResultType::INVALID);
}

// (4) projectToNearestNode never returns nullopt for non-empty graphs.
TEST(ProjectorUnlimitedTest, ProjectToNearestNodeAlwaysReturnsForNonEmpty)
{
  auto g = buildSimple();
  auto lock = g->lockShared();
  StartGoalProjector::Params pr;
  StartGoalProjector proj(pr);

  // Far away from everything: bounded fails, unlimited succeeds.
  EXPECT_FALSE(proj.projectToNode(pose(50.0, 50.0), *g).has_value());
  auto id = proj.projectToNearestNode(pose(50.0, 50.0), *g);
  ASSERT_TRUE(id.has_value());
  // (50,50): node A distance ≈ 70.71, node B distance ≈ 64.03 → B wins.
  EXPECT_EQ(*id, "B");
}

// (5) nearerEndpointOfEdge picks correctly given the goal pose.
TEST(ProjectorUnlimitedTest, NearerEndpointOfEdge)
{
  auto g = buildSimple();
  auto lock = g->lockShared();
  StartGoalProjector::Params pr;
  StartGoalProjector proj(pr);

  // pose at (8, 50) — far from the edge, but B(10,0) is nearer than A(0,0).
  auto u = proj.projectUnlimited(pose(8.0, 50.0), *g);
  ASSERT_EQ(u.type, StartGoalProjector::ResultType::ON_EDGE);
  auto endpoint = StartGoalProjector::nearerEndpointOfEdge(
    u, pose(8.0, 50.0), *g);
  ASSERT_TRUE(endpoint.has_value());
  EXPECT_EQ(*endpoint, "B");
}

// R43 regression: a goal exactly at a node shared by multiple incident edges
// must remain ON_NODE. Start-side edge preference is preserved independently.
TEST(ProjectorUnlimitedTest, GoalAtSharedNodePrefersNodeOnTie)
{
  auto g = std::make_unique<TopoGraph>();
  std::vector<TopoNode> nodes;
  TopoNode a; a.id = "A"; a.kind = NodeKind::NAV_POINT;
  a.x = 0.0; a.y = 0.0; nodes.push_back(a);
  TopoNode shared; shared.id = "shared"; shared.kind = NodeKind::NAV_POINT;
  shared.x = 1.0; shared.y = 0.0; nodes.push_back(shared);
  TopoNode c; c.id = "C"; c.kind = NodeKind::NAV_POINT;
  c.x = 1.0; c.y = -1.0; nodes.push_back(c);

  std::vector<TopoEdge> edges;
  TopoEdge main; main.id = "main"; main.from_id = "A"; main.to_id = "shared";
  main.geometry = EdgeGeometry::LINE; edges.push_back(main);
  TopoEdge branch; branch.id = "branch"; branch.from_id = "C";
  branch.to_id = "shared"; branch.geometry = EdgeGeometry::LINE;
  edges.push_back(branch);
  g->rebuildFrom(std::move(nodes), std::move(edges));

  auto lock = g->lockShared();
  StartGoalProjector::Params pr;
  StartGoalProjector proj(pr);

  const auto exact_goal = pose(1.0, 0.0);
  const auto start_semantics = proj.projectPreferEdge(exact_goal, *g);
  EXPECT_EQ(start_semantics.type, StartGoalProjector::ResultType::ON_EDGE);

  const auto goal_semantics = proj.projectGoalUnlimited(exact_goal, *g);
  EXPECT_EQ(goal_semantics.type, StartGoalProjector::ResultType::ON_NODE);
  EXPECT_EQ(goal_semantics.id, "shared");
  EXPECT_NEAR(goal_semantics.distance, 0.0, 1e-9);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
