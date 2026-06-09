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

}  // namespace

TEST(AStarTest, FindsShortestPath)
{
  auto g = buildDiamond();
  auto lock = g->lockShared();
  AStarSearcher s;
  AStarSearcher::Request req;
  req.start_node_id = "A";
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
  req.start_node_id = "A";
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
  req.start_node_id = "A";
  req.goal_node_id = "ZZZ";
  auto r = s.search(req, *g);
  EXPECT_FALSE(r.success);
}

TEST(AStarTest, FailsWhenStartUnknown)
{
  auto g = buildDiamond();
  auto lock = g->lockShared();
  AStarSearcher s;
  AStarSearcher::Request req;
  req.start_node_id = "ZZZ";
  req.goal_node_id = "D";
  auto r = s.search(req, *g);
  EXPECT_FALSE(r.success);
}

// Note: the former StartOnEdge* tests moved to test_graph_augmenter.cpp —
// edge-projected starts are now modelled as virtual nodes, so A* only ever
// sees node-to-node requests. The "no detour back to the far junction"
// regression lives there as StartOnEdgeNoDetourToFarJunction.

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
