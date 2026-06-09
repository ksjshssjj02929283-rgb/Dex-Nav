// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#ifndef NAV2_ROUTE_PLANNER__GRAPH_AUGMENTER_HPP_
#define NAV2_ROUTE_PLANNER__GRAPH_AUGMENTER_HPP_

#include <memory>
#include <string>

#include "nav2_route_planner/start_goal_projector.hpp"
#include "nav2_route_planner/topo_graph.hpp"

namespace nav2_route_planner
{

/// Output of `buildAugmentedGraph`: the graph A* should search, plus the
/// node ids to search between.
struct AugmentedGraph
{
  /// Either the base graph itself (aliased, when no projection needed a
  /// split) or a freshly-built copy with virtual nodes inserted.
  std::shared_ptr<TopoGraph> graph;
  std::string start_id;   // node id to start A* from
  std::string goal_id;    // node id to end A* at
};

/// Reserved ids for inserted virtual elements.
inline constexpr const char * kVirtualStartId = "__vstart__";
inline constexpr const char * kVirtualGoalId = "__vgoal__";

/// Turn an ON_EDGE projection into a first-class graph node so the rest of
/// the planner can stay uniform.
///
/// For each projection that landed ON an edge, a virtual NavPoint is placed
/// at the projection and the host edge is split into sub-edges meeting at
/// that node. After this:
///   • A* runs plain node-to-node (no partial-edge start cost),
///   • the interpolator walks a uniform edge sequence (no `__start_edge_in__`
///     / `__goal_edge_out__` / goal-edge truncation),
///   • the "start & goal share one edge" case needs no special handling —
///     the edge is simply split in three and A* walks the middle piece.
///
/// Geometry: LINE edges split exactly (sub-edges are straight, defined by
/// their node endpoints). BEZIER edges split via de Casteljau so each
/// sub-edge carries the true partial curve; the virtual node sits on the
/// curve at the split parameter. Sub-edges inherit the host edge's
/// lane_subtype / floor / max_speed / bidirectional / regulatory ids; their
/// length is recomputed from geometry. Sub-edge ids are synthetic
/// (`<edge>#0`, `#1`, `#2`) — no downstream consumer keys off edge_id.
///
/// When neither projection is ON_EDGE the base graph is returned aliased
/// (no copy) with the projected node ids.
///
/// Caller must hold `base->lockShared()` for the duration of the call.
AugmentedGraph buildAugmentedGraph(
  const std::shared_ptr<TopoGraph> & base,
  const StartGoalProjector::Result & start_proj,
  const StartGoalProjector::Result & goal_proj,
  int bezier_samples);

}  // namespace nav2_route_planner

#endif  // NAV2_ROUTE_PLANNER__GRAPH_AUGMENTER_HPP_
