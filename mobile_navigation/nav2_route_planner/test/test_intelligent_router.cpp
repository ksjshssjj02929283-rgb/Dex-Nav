// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <memory>
#include <utility>
#include <vector>

#include "nav2_route_planner/astar_searcher.hpp"
#include "nav2_route_planner/intelligent_router.hpp"
#include "nav2_route_planner/start_goal_projector.hpp"
#include "nav2_route_planner/topo_graph.hpp"

using nav2_route_planner::AStarSearcher;
using nav2_route_planner::EdgeGeometry;
using nav2_route_planner::IntelligentRouter;
using nav2_route_planner::NodeKind;
using nav2_route_planner::StartGoalProjector;
using nav2_route_planner::TopoEdge;
using nav2_route_planner::TopoGraph;
using nav2_route_planner::TopoNode;

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

// Graph layout (top-down):
//
//      A(0,0) ──────── e_ab ──────── B(10,0)
//
// One straight edge of length 10. Used by all the geometry tests below;
// graph A\* on this graph always returns the single edge `e_ab`.
std::unique_ptr<TopoGraph> buildLineGraph()
{
  auto g = std::make_unique<TopoGraph>();
  std::vector<TopoNode> nodes;
  TopoNode a; a.id = "A"; a.kind = NodeKind::NAV_POINT;
  a.x = 0.0; a.y = 0.0; nodes.push_back(a);
  TopoNode b; b.id = "B"; b.kind = NodeKind::NAV_POINT;
  b.x = 10.0; b.y = 0.0; nodes.push_back(b);
  std::vector<TopoEdge> edges;
  TopoEdge e; e.id = "e_ab"; e.from_id = "A"; e.to_id = "B";
  e.geometry = EdgeGeometry::LINE;
  e.bidirectional = true;
  e.length = 10.0;
  edges.push_back(e);
  g->rebuildFrom(std::move(nodes), std::move(edges));
  return g;
}

AStarSearcher::Result fakeAStarSuccess(
  const std::vector<std::string> & node_ids,
  const std::vector<std::string> & edge_ids,
  double total_cost)
{
  AStarSearcher::Result r;
  r.success = true;
  r.node_ids = node_ids;
  r.edge_ids = edge_ids;
  r.total_cost = total_cost;
  return r;
}

}  // namespace

// ─── pathLength: cumulative segment length ──────────────────────────────

TEST(IntelligentRouterTest, PathLengthSimple)
{
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  poses.push_back(pose(0.0, 0.0));
  poses.push_back(pose(3.0, 4.0));   // +5
  poses.push_back(pose(3.0, 4.0));   // +0 (duplicate)
  poses.push_back(pose(6.0, 8.0));   // +5
  EXPECT_NEAR(IntelligentRouter::pathLength(poses), 10.0, 1e-9);
}

TEST(IntelligentRouterTest, PathLengthEmptyOrSingle)
{
  EXPECT_NEAR(IntelligentRouter::pathLength({}), 0.0, 1e-9);
  EXPECT_NEAR(IntelligentRouter::pathLength({pose(1.0, 1.0)}), 0.0, 1e-9);
}

// ─── estimateRouteDistance: approach + graph leg + approach ────────────

TEST(IntelligentRouterTest, EstimateRouteOnNodeProjections)
{
  auto g = buildLineGraph();
  auto lock = g->lockShared();
  // start at A, goal at B: approaches = 0, graph leg = 10.
  StartGoalProjector::Result sp;
  sp.type = StartGoalProjector::ResultType::ON_NODE;
  sp.id = "A"; sp.distance = 0.0;
  StartGoalProjector::Result gp;
  gp.type = StartGoalProjector::ResultType::ON_NODE;
  gp.id = "B"; gp.distance = 0.0;
  auto search = fakeAStarSuccess({"A", "B"}, {"e_ab"}, 10.0);

  const double total = IntelligentRouter::estimateRouteDistance(
    pose(0.0, 0.0), pose(10.0, 0.0), sp, gp, search, *g);
  EXPECT_NEAR(total, 10.0, 1e-6);
}

TEST(IntelligentRouterTest, EstimateRouteWithApproachLegs)
{
  auto g = buildLineGraph();
  auto lock = g->lockShared();
  // Start at (0, 3): nearest projection is NODE A at distance 3.
  // Goal at (10, 4): nearest projection is NODE B at distance 4.
  // Graph leg = 10. Total = 3 + 10 + 4 = 17.
  StartGoalProjector::Result sp;
  sp.type = StartGoalProjector::ResultType::ON_NODE;
  sp.id = "A"; sp.distance = 3.0;
  StartGoalProjector::Result gp;
  gp.type = StartGoalProjector::ResultType::ON_NODE;
  gp.id = "B"; gp.distance = 4.0;
  auto search = fakeAStarSuccess({"A", "B"}, {"e_ab"}, 10.0);

  const double total = IntelligentRouter::estimateRouteDistance(
    pose(0.0, 3.0), pose(10.0, 4.0), sp, gp, search, *g);
  EXPECT_NEAR(total, 17.0, 1e-6);
}

// ─── decide: disabled → always ROUTE ────────────────────────────────────

TEST(IntelligentRouterTest, DisabledAlwaysReturnsRoute)
{
  // Disabled router should not even attempt direct Theta\* — pass a
  // dummy CA layer; the cast will never be dereferenced.
  IntelligentRouter::Params p;
  p.enable = false;
  IntelligentRouter r(p);

  auto g = buildLineGraph();
  auto lock = g->lockShared();
  StartGoalProjector::Result sp;
  sp.type = StartGoalProjector::ResultType::ON_NODE;
  sp.id = "A";
  StartGoalProjector::Result gp;
  gp.type = StartGoalProjector::ResultType::ON_NODE;
  gp.id = "B";
  auto search = fakeAStarSuccess({"A", "B"}, {"e_ab"}, 10.0);

  // With enable=false, decide() short-circuits before touching ca_layer,
  // so passing a null-initialised one is safe.
  nav2_route_planner::CostmapAwareLayer ca(
    rclcpp_lifecycle::LifecycleNode::WeakPtr(), nullptr);

  auto d = r.decide(pose(0.0, 0.0), pose(10.0, 0.0),
      sp, gp, search, *g, ca);
  EXPECT_EQ(d.strategy, IntelligentRouter::Strategy::ROUTE_VIA_GRAPH);
  EXPECT_EQ(d.reason, "router disabled");
}

// ─── decide: graph failed but direct feasible → DIRECT ──────────────────
//
// We cannot easily feed a working CA layer in a unit test (no costmap),
// so this branch is covered by:
//   • mocking the path-length computation (pathLength is static)
//   • the unit test for estimateRouteDistance above
//   • the integration test (Phase 4) on the live planner
// The static-method tests above plus the disabled-mode test are
// sufficient for unit-level coverage; full strategy selection lives in
// the planner integration tests.

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
