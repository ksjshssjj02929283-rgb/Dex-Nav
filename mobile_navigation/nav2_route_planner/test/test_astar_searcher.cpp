// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <memory>
#include <utility>
#include <vector>

#include "nav2_route_planner/astar_searcher.hpp"
#include "nav2_route_planner/start_goal_projector.hpp"
#include "nav2_route_planner/topo_graph.hpp"

using nav2_route_planner::AStarSearcher;
using nav2_route_planner::StartGoalProjector;
using nav2_route_planner::TopoGraph;
using nav2_route_planner::TopoNode;
using nav2_route_planner::TopoEdge;
using nav2_route_planner::EdgeGeometry;
using nav2_route_planner::NodeKind;
using nav2_route_planner::CostWeights;
using nav2_route_planner::UserPrefs;

namespace
{
std::unique_ptr<TopoGraph> buildDiamond()
{
  // A ─e_ab─→ B ─e_bd─→ D
  // A ─e_ac─→ C ─e_cd─→ D     (C path is longer)
  auto g = std::make_unique<TopoGraph>();
  std::vector<TopoNode> ns;
  for (auto p : std::vector<std::tuple<std::string, double, double>>{
        {"A", 0, 0}, {"B", 5, 1}, {"C", 5, -3}, {"D", 10, 0}})
  {
    TopoNode n;
    n.id = std::get<0>(p);
    n.kind = NodeKind::NAV_POINT;
    n.x = std::get<1>(p);
    n.y = std::get<2>(p);
    ns.push_back(n);
  }
  std::vector<TopoEdge> es;
  for (auto e : std::vector<std::tuple<std::string, std::string, std::string>>{
        {"e_ab", "A", "B"}, {"e_bd", "B", "D"},
        {"e_ac", "A", "C"}, {"e_cd", "C", "D"}})
  {
    TopoEdge te;
    te.id = std::get<0>(e);
    te.from_id = std::get<1>(e);
    te.to_id = std::get<2>(e);
    te.geometry = EdgeGeometry::LINE;
    es.push_back(te);
  }
  g->rebuildFrom(std::move(ns), std::move(es));
  return g;
}

StartGoalProjector::Result onNode(const std::string & id)
{
  StartGoalProjector::Result r;
  r.type = StartGoalProjector::ResultType::ON_NODE;
  r.id = id;
  return r;
}
}  // namespace

TEST(AStarTest, FindsShortestPath)
{
  auto g = buildDiamond();
  auto lock = g->lockShared();
  AStarSearcher s;
  AStarSearcher::Request req;
  req.start = onNode("A");
  req.goal_node_id = "D";
  auto r = s.search(req, *g);
  ASSERT_TRUE(r.success) << r.failure_reason;
  // Expect A→B→D (top branch is shorter)
  ASSERT_EQ(r.node_ids.size(), 3u);
  EXPECT_EQ(r.node_ids[0], "A");
  EXPECT_EQ(r.node_ids[1], "B");
  EXPECT_EQ(r.node_ids[2], "D");
  EXPECT_EQ(r.edge_ids.size(), 2u);
  EXPECT_EQ(r.edge_ids[0], "e_ab");
  EXPECT_EQ(r.edge_ids[1], "e_bd");
}

TEST(AStarTest, RespectsBlacklist)
{
  auto g = buildDiamond();
  auto lock = g->lockShared();
  AStarSearcher s;
  AStarSearcher::Request req;
  req.start = onNode("A");
  req.goal_node_id = "D";
  req.blacklisted_edge_ids.insert("e_ab");
  auto r = s.search(req, *g);
  ASSERT_TRUE(r.success);
  EXPECT_EQ(r.node_ids[1], "C");  // forced through C
}

TEST(AStarTest, FailsWhenGoalUnknown)
{
  auto g = buildDiamond();
  auto lock = g->lockShared();
  AStarSearcher s;
  AStarSearcher::Request req;
  req.start = onNode("A");
  req.goal_node_id = "ZZZ";
  auto r = s.search(req, *g);
  EXPECT_FALSE(r.success);
}

TEST(AStarTest, StartOnEdgePicksClosestEntry)
{
  auto g = buildDiamond();
  auto lock = g->lockShared();
  AStarSearcher s;
  AStarSearcher::Request req;
  // Start projected halfway on e_ab — should still reach D.
  req.start.type = StartGoalProjector::ResultType::ON_EDGE;
  req.start.id = "e_ab";
  req.start.edge_sample_index = 1;
  req.start.edge_total_samples = 2;
  req.goal_node_id = "D";
  auto r = s.search(req, *g);
  ASSERT_TRUE(r.success);
  EXPECT_EQ(r.node_ids.back(), "D");
}

// Regression for the "loop back to junction" bug: when the robot is ON an
// edge, the partial-edge cost must reflect its TRUE position along the edge.
// A 2-sample LINE edge always reports edge_sample_index == 0 (FLOOR
// semantic), so the old cost `full * (index/(N-1))` collapsed to
// from-cost = 0 / to-cost = full — i.e. A* treated the robot as sitting at
// the `from` endpoint regardless of where it actually was. When `from` is a
// junction, A* rooted the path there and the robot was sent back to the
// junction before proceeding. The fix uses projected_xy's chord position.
TEST(AStarTest, StartOnEdgeNoDetourToFarEndpoint)
{
  //  F ──E1──→ T       robot is on E1 near T
  //  T ──Etg─→ G       short approach to the goal (len 1)
  //  F ──Efg─→ G       long detour back through F (len ~10)
  auto g = std::make_unique<TopoGraph>();
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

  AStarSearcher s;
  AStarSearcher::Request req;
  req.start.type = StartGoalProjector::ResultType::ON_EDGE;
  req.start.id = "E1";
  req.start.projected_xy = {9.0, 0.0};  // robot near the T endpoint
  req.start.edge_sample_index = 0;       // degenerate for any 2-sample LINE
  req.start.edge_total_samples = 2;
  req.goal_node_id = "G";

  auto r = s.search(req, *g);
  ASSERT_TRUE(r.success) << r.failure_reason;
  EXPECT_EQ(r.node_ids.front(), "T");    // enters the graph at the near endpoint
  for (const auto & n : r.node_ids) {
    EXPECT_NE(n, "F") << "path detoured back to the far junction (loop)";
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
