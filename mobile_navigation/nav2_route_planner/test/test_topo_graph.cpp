// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <memory>
#include <utility>
#include <vector>

#include "nav2_route_planner/topo_graph.hpp"

using nav2_route_planner::TopoGraph;
using nav2_route_planner::TopoNode;
using nav2_route_planner::TopoEdge;
using nav2_route_planner::NodeKind;
using nav2_route_planner::EdgeGeometry;

namespace
{

// Build a simple "L" graph:  A --(e1)--> B --(e2,bidir)--> C
std::unique_ptr<TopoGraph> buildLGraph()
{
  auto g = std::make_unique<TopoGraph>();
  std::vector<TopoNode> nodes;
  TopoNode a; a.id = "A"; a.x = 0.0; a.y = 0.0; nodes.push_back(a);
  TopoNode b; b.id = "B"; b.x = 5.0; b.y = 0.0; nodes.push_back(b);
  TopoNode c; c.id = "C"; c.x = 5.0; c.y = 5.0; nodes.push_back(c);

  std::vector<TopoEdge> edges;
  TopoEdge e1; e1.id = "e1"; e1.from_id = "A"; e1.to_id = "B";
  e1.geometry = EdgeGeometry::LINE; e1.bidirectional = false;
  edges.push_back(e1);
  TopoEdge e2; e2.id = "e2"; e2.from_id = "B"; e2.to_id = "C";
  e2.geometry = EdgeGeometry::LINE; e2.bidirectional = true;
  edges.push_back(e2);

  g->rebuildFrom(std::move(nodes), std::move(edges));
  return g;
}

}  // namespace

TEST(TopoGraphTest, RebuildAndQuery)
{
  auto g = buildLGraph();
  EXPECT_TRUE(g->isReady());
  EXPECT_EQ(g->nodeCount(), 3u);
  EXPECT_EQ(g->edgeCount(), 2u);

  auto na = g->getNode("A");
  ASSERT_TRUE(na.has_value());
  EXPECT_DOUBLE_EQ(na->x, 0.0);

  auto e1 = g->getEdge("e1");
  ASSERT_TRUE(e1.has_value());
  EXPECT_NEAR(e1->length, 5.0, 1e-6);

  auto e2 = g->getEdge("e2");
  ASSERT_TRUE(e2.has_value());
  EXPECT_NEAR(e2->length, 5.0, 1e-6);
}

TEST(TopoGraphTest, OutgoingRespectsBidirectional)
{
  auto g = buildLGraph();
  auto lock = g->lockShared();
  auto from_a = g->outgoing("A");
  EXPECT_EQ(from_a.size(), 1u);
  EXPECT_EQ(from_a.front()->id, "e1");

  auto from_c = g->outgoing("C");
  EXPECT_EQ(from_c.size(), 1u);
  EXPECT_EQ(from_c.front()->id, "e2");  // bidirectional

  auto from_b = g->outgoing("B");
  EXPECT_EQ(from_b.size(), 1u);  // e2 forward
  EXPECT_EQ(from_b.front()->id, "e2");
}

TEST(TopoGraphTest, FindNearestNode)
{
  auto g = buildLGraph();
  auto lock = g->lockShared();
  auto hits = g->findNearestNodes(0.1, 0.1, 1.0, 5);
  ASSERT_FALSE(hits.empty());
  EXPECT_EQ(hits.front(), "A");

  auto miss = g->findNearestNodes(100.0, 100.0, 1.0, 5);
  EXPECT_TRUE(miss.empty());
}

TEST(TopoGraphTest, SampleEdgeLineHasTwoEndpoints)
{
  auto g = buildLGraph();
  auto lock = g->lockShared();
  auto e1 = g->getEdge("e1");
  ASSERT_TRUE(e1);
  auto pts = g->sampleEdge(*e1, 40);
  ASSERT_EQ(pts.size(), 2u);
  EXPECT_DOUBLE_EQ(pts[0].first, 0.0);
  EXPECT_DOUBLE_EQ(pts[1].first, 5.0);
}

TEST(TopoGraphTest, SampleBezierEdge)
{
  TopoGraph g;
  std::vector<TopoNode> ns;
  TopoNode n0; n0.id = "n0"; n0.x = 0.0; n0.y = 0.0; ns.push_back(n0);
  TopoNode n1; n1.id = "n1"; n1.x = 4.0; n1.y = 0.0; ns.push_back(n1);
  TopoEdge e; e.id = "be"; e.from_id = "n0"; e.to_id = "n1";
  e.geometry = EdgeGeometry::BEZIER;
  e.control_points = {{0, 0}, {1, 2}, {3, 2}, {4, 0}};
  std::vector<TopoEdge> es{e};
  g.rebuildFrom(std::move(ns), std::move(es));

  auto lock = g.lockShared();
  auto e1 = g.getEdge("be");
  ASSERT_TRUE(e1);
  auto pts = g.sampleEdge(*e1, 40);
  EXPECT_EQ(pts.size(), 41u);
  // Bernstein endpoints must coincide with first/last control points.
  EXPECT_NEAR(pts.front().first, 0.0, 1e-6);
  EXPECT_NEAR(pts.front().second, 0.0, 1e-6);
  EXPECT_NEAR(pts.back().first, 4.0, 1e-6);
  EXPECT_NEAR(pts.back().second, 0.0, 1e-6);
  // Length should be > straight-line 4.0 because the curve bulges up.
  EXPECT_GT(e1->length, 4.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
