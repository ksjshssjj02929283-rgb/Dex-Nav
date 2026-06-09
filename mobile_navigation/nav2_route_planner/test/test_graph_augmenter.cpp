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
#include "nav2_route_planner/start_goal_projector.hpp"
#include "nav2_route_planner/topo_graph.hpp"

using nav2_route_planner::AStarSearcher;
using nav2_route_planner::AugmentedGraph;
using nav2_route_planner::buildAugmentedGraph;
using nav2_route_planner::EdgeGeometry;
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

StartGoalProjector::Result onNode(const std::string & id)
{
  StartGoalProjector::Result r;
  r.type = StartGoalProjector::ResultType::ON_NODE;
  r.id = id;
  return r;
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
}  // namespace

// Neither endpoint projects onto an edge → the base graph is returned
// aliased (no copy) and the ids pass through unchanged.
TEST(GraphAugmenterTest, NoSplitWhenBothOnNode)
{
  auto g = std::make_shared<TopoGraph>();
  std::vector<TopoNode> ns;
  TopoNode a; a.id = "A"; a.x = 0; a.y = 0; ns.push_back(a);
  TopoNode b; b.id = "B"; b.x = 10; b.y = 0; ns.push_back(b);
  std::vector<TopoEdge> es;
  TopoEdge e; e.id = "e"; e.from_id = "A"; e.to_id = "B";
  e.geometry = EdgeGeometry::LINE; e.bidirectional = true; es.push_back(e);
  g->rebuildFrom(std::move(ns), std::move(es));
  auto lock = g->lockShared();

  auto aug = buildAugmentedGraph(g, onNode("A"), onNode("B"), 40);
  EXPECT_EQ(aug.graph.get(), g.get());  // aliased, no copy
  EXPECT_EQ(aug.start_id, "A");
  EXPECT_EQ(aug.goal_id, "B");
}

// A LINE edge with an ON_EDGE start: a virtual node is inserted at the
// projection and the edge is split into two sub-edges whose lengths sum to
// the original.
TEST(GraphAugmenterTest, LineSplitInsertsVirtualNode)
{
  auto g = std::make_shared<TopoGraph>();
  std::vector<TopoNode> ns;
  TopoNode a; a.id = "A"; a.x = 0; a.y = 0; ns.push_back(a);
  TopoNode b; b.id = "B"; b.x = 10; b.y = 0; ns.push_back(b);
  std::vector<TopoEdge> es;
  TopoEdge e; e.id = "e"; e.from_id = "A"; e.to_id = "B";
  e.geometry = EdgeGeometry::LINE; e.bidirectional = true;
  e.lane_subtype = "corridor"; es.push_back(e);
  g->rebuildFrom(std::move(ns), std::move(es));
  auto lock = g->lockShared();

  auto aug = buildAugmentedGraph(g, onEdgeLine("e", 3.0, 0.0), onNode("B"), 40);
  ASSERT_NE(aug.graph.get(), g.get());  // a fresh copy
  EXPECT_EQ(aug.start_id, "__vstart__");
  EXPECT_EQ(aug.goal_id, "B");

  auto v = aug.graph->getNode("__vstart__");
  ASSERT_TRUE(v.has_value());
  EXPECT_NEAR(v->x, 3.0, 1e-6);
  EXPECT_NEAR(v->y, 0.0, 1e-6);

  // Original edge gone, replaced by two sub-edges summing to length 10.
  EXPECT_FALSE(aug.graph->getEdge("e").has_value());
  auto s0 = aug.graph->getEdge("e#0");
  auto s1 = aug.graph->getEdge("e#1");
  ASSERT_TRUE(s0.has_value());
  ASSERT_TRUE(s1.has_value());
  EXPECT_NEAR(s0->length + s1->length, 10.0, 1e-3);
  // Sub-edges inherit semantics.
  EXPECT_EQ(s0->lane_subtype, "corridor");
  EXPECT_EQ(s1->lane_subtype, "corridor");
}

// Start and goal both project onto the SAME edge → the edge is split in
// three, and A* between the two virtual nodes walks ONLY the middle piece
// (never detours to an endpoint NavPoint).
TEST(GraphAugmenterTest, SameEdgeSplitsInThreeNoDetour)
{
  auto g = std::make_shared<TopoGraph>();
  std::vector<TopoNode> ns;
  TopoNode a; a.id = "A"; a.x = 0; a.y = 0; ns.push_back(a);
  TopoNode b; b.id = "B"; b.x = 10; b.y = 0; ns.push_back(b);
  std::vector<TopoEdge> es;
  TopoEdge e; e.id = "e"; e.from_id = "A"; e.to_id = "B";
  e.geometry = EdgeGeometry::LINE; e.bidirectional = true; es.push_back(e);
  g->rebuildFrom(std::move(ns), std::move(es));
  auto lock = g->lockShared();

  // start at x=3, goal at x=7 — both on edge e.
  auto aug = buildAugmentedGraph(
    g, onEdgeLine("e", 3.0, 0.0), onEdgeLine("e", 7.0, 0.0), 40);
  EXPECT_EQ(aug.start_id, "__vstart__");
  EXPECT_EQ(aug.goal_id, "__vgoal__");
  // Three sub-edges (#0 A→V, #1 V→V, #2 V→B).
  EXPECT_TRUE(aug.graph->getEdge("e#0").has_value());
  EXPECT_TRUE(aug.graph->getEdge("e#1").has_value());
  EXPECT_TRUE(aug.graph->getEdge("e#2").has_value());

  AStarSearcher s;
  AStarSearcher::Request req;
  req.start_node_id = aug.start_id;
  req.goal_node_id = aug.goal_id;
  auto r = s.search(req, *aug.graph);
  ASSERT_TRUE(r.success) << r.failure_reason;
  // Direct: __vstart__ → __vgoal__, no endpoint A/B on the path.
  EXPECT_EQ(r.node_ids.front(), "__vstart__");
  EXPECT_EQ(r.node_ids.back(), "__vgoal__");
  for (const auto & n : r.node_ids) {
    EXPECT_NE(n, "A");
    EXPECT_NE(n, "B");
  }
}

// Regression (was test_astar_searcher StartOnEdgeNoDetourToFarEndpoint):
// robot on edge F→T near the T endpoint, goal at G. The augmented A* must
// enter the graph at the near endpoint (T side), never loop back to the far
// junction F.
TEST(GraphAugmenterTest, StartOnEdgeNoDetourToFarJunction)
{
  auto g = std::make_shared<TopoGraph>();
  std::vector<TopoNode> ns;
  TopoNode f; f.id = "F"; f.kind = NodeKind::JUNCTION; f.x = 0; f.y = 0; ns.push_back(f);
  TopoNode t; t.id = "T"; t.kind = NodeKind::NAV_POINT; t.x = 10; t.y = 0; ns.push_back(t);
  TopoNode gg; gg.id = "G"; gg.kind = NodeKind::NAV_POINT; gg.x = 10; gg.y = -1; ns.push_back(gg);
  auto mk = [](const std::string & id, const std::string & a, const std::string & b) {
      TopoEdge e; e.id = id; e.from_id = a; e.to_id = b;
      e.geometry = EdgeGeometry::LINE; e.bidirectional = true; return e;
    };
  std::vector<TopoEdge> es{mk("E1", "F", "T"), mk("Etg", "T", "G"), mk("Efg", "F", "G")};
  g->rebuildFrom(std::move(ns), std::move(es));
  auto lock = g->lockShared();

  // robot projects onto E1 at (9,0) — near T.
  auto aug = buildAugmentedGraph(
    g, onEdgeLine("E1", 9.0, 0.0), onNode("G"), 40);

  AStarSearcher s;
  AStarSearcher::Request req;
  req.start_node_id = aug.start_id;
  req.goal_node_id = aug.goal_id;
  auto r = s.search(req, *aug.graph);
  ASSERT_TRUE(r.success) << r.failure_reason;
  EXPECT_EQ(r.node_ids.front(), "__vstart__");
  for (const auto & n : r.node_ids) {
    EXPECT_NE(n, "F") << "path detoured back to the far junction (loop)";
  }
}

// A BEZIER edge split keeps the curve: the virtual node lies ON the curve,
// and the concatenated sub-edge geometry closely matches the original.
TEST(GraphAugmenterTest, BezierSplitPreservesGeometry)
{
  const int kSamples = 40;
  auto g = std::make_shared<TopoGraph>();
  std::vector<TopoNode> ns;
  TopoNode a; a.id = "A"; a.x = 0; a.y = 0; ns.push_back(a);
  TopoNode b; b.id = "B"; b.x = 10; b.y = 0; ns.push_back(b);
  std::vector<TopoEdge> es;
  TopoEdge e; e.id = "e"; e.from_id = "A"; e.to_id = "B";
  e.geometry = EdgeGeometry::BEZIER; e.bidirectional = true;
  e.control_points = {{0, 0}, {5, 5}, {10, 0}};
  es.push_back(e);
  g->rebuildFrom(std::move(ns), std::move(es));
  auto lock = g->lockShared();

  // Use the real projector so edge_sample_index matches the sampling.
  StartGoalProjector::Params pp; pp.bezier_samples_per_edge = kSamples;
  StartGoalProjector projector(pp);
  auto sp = projector.projectPreferEdge(pose(5.0, 2.2), *g);
  ASSERT_EQ(sp.type, StartGoalProjector::ResultType::ON_EDGE);

  auto aug = buildAugmentedGraph(g, sp, onNode("B"), kSamples);
  auto v = aug.graph->getNode("__vstart__");
  ASSERT_TRUE(v.has_value());

  // Virtual node must sit on the original curve (closest original sample
  // within a small tolerance).
  auto orig = g->sampleEdge(e, kSamples);
  double best = 1e9;
  for (const auto & p : orig) {
    best = std::min(best, std::hypot(p.first - v->x, p.second - v->y));
  }
  EXPECT_LT(best, 0.3) << "virtual node drifted off the curve";

  // Concatenated sub-edge length ≈ original arc length.
  auto s0 = aug.graph->getEdge("e#0");
  auto s1 = aug.graph->getEdge("e#1");
  ASSERT_TRUE(s0.has_value() && s1.has_value());
  const double orig_len = g->computeEdgeLength(e, kSamples);
  EXPECT_NEAR(s0->length + s1->length, orig_len, 0.2);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
