// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#ifndef NAV2_ROUTE_PLANNER__ASTAR_SEARCHER_HPP_
#define NAV2_ROUTE_PLANNER__ASTAR_SEARCHER_HPP_

#include <string>
#include <unordered_set>
#include <vector>

#include "nav2_route_planner/cost_function.hpp"
#include "nav2_route_planner/start_goal_projector.hpp"
#include "nav2_route_planner/topo_graph.hpp"

namespace nav2_route_planner
{

/// A walked sub-arc of one edge between two fractional positions
/// (t_begin → t_end along from→to). For middle full edges t_begin=0,
/// t_end=1 (or reversed). For first/last arcs t_begin or t_end may
/// reflect the start/goal projection fraction.
struct RouteArc
{
  std::string edge_id;
  double t_begin{0.0};
  double t_end{1.0};
};

/// Forward A* search on the road graph.
///
/// Nodes are TopoNode ids; edges are TopoEdge instances. Start may be either
/// at a node (ON_NODE) or partway along an edge (ON_EDGE) — in the latter
/// case both endpoints of the edge are treated as virtual entry points with
/// partial-edge cost on top of the regular A* graph.
class AStarSearcher
{
public:
  struct Request
  {
    StartGoalProjector::Result start;
    std::string goal_node_id;
    CostWeights weights;
    UserPrefs prefs;
    std::unordered_set<std::string> blacklisted_edge_ids;

    /// New virtual-node interface (DEC-001). When `goal_proj.type` is
    /// ON_EDGE, search terminates at a virtual node sitting on goal_proj.
    /// Falls back to `goal_node_id` only when `goal_proj.type == INVALID`.
    StartGoalProjector::Result goal_proj;
  };

  struct Result
  {
    bool success{false};
    std::vector<std::string> node_ids;   // size N+1
    std::vector<std::string> edge_ids;   // size N (edge from node_ids[i] to node_ids[i+1])
    double total_cost{0.0};
    std::size_t expanded_nodes{0};       // for diagnostics
    std::string failure_reason;

    /// Sequence of edge-arcs from start projection to goal projection.
    /// Empty if search did not run the virtual-node path.
    std::vector<RouteArc> arcs;
  };

  /// Search the graph. Caller must hold graph.lockShared() during the call.
  Result search(const Request & req, const TopoGraph & graph) const;
};

}  // namespace nav2_route_planner

#endif  // NAV2_ROUTE_PLANNER__ASTAR_SEARCHER_HPP_
