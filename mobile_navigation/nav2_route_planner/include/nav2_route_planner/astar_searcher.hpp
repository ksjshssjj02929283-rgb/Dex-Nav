// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#ifndef NAV2_ROUTE_PLANNER__ASTAR_SEARCHER_HPP_
#define NAV2_ROUTE_PLANNER__ASTAR_SEARCHER_HPP_

#include <string>
#include <unordered_set>
#include <vector>

#include "nav2_route_planner/cost_function.hpp"
#include "nav2_route_planner/topo_graph.hpp"

namespace nav2_route_planner
{

/// Forward A* search on the road graph.
///
/// Plain node-to-node search: both start and goal are TopoNode ids. When a
/// pose projects partway along an edge, the caller inserts a virtual node
/// there (see graph_augmenter) and passes that node's id — A* itself never
/// deals with partial-edge starts.
class AStarSearcher
{
public:
  struct Request
  {
    std::string start_node_id;
    std::string goal_node_id;
    CostWeights weights;
    UserPrefs prefs;
    std::unordered_set<std::string> blacklisted_edge_ids;
  };

  struct Result
  {
    bool success{false};
    std::vector<std::string> node_ids;   // size N+1
    std::vector<std::string> edge_ids;   // size N (edge from node_ids[i] to node_ids[i+1])
    double total_cost{0.0};
    std::size_t expanded_nodes{0};       // for diagnostics
    std::string failure_reason;
  };

  /// Search the graph. Caller must hold graph.lockShared() during the call.
  Result search(const Request & req, const TopoGraph & graph) const;
};

}  // namespace nav2_route_planner

#endif  // NAV2_ROUTE_PLANNER__ASTAR_SEARCHER_HPP_
