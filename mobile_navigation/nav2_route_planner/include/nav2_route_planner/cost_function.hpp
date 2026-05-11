// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#ifndef NAV2_ROUTE_PLANNER__COST_FUNCTION_HPP_
#define NAV2_ROUTE_PLANNER__COST_FUNCTION_HPP_

#include "nav2_route_planner/topo_graph.hpp"

namespace nav2_route_planner
{

/// Edge-cost weights used by both A* search and downstream tooling.
///
/// `floor_penalty` is added once per cross-floor traversal (large constant).
/// `door` / `elevator` / `stair_factor` are inferred from edge.lane_subtype
/// and applied even before a full RegulatoryElement plugin layer exists.
struct CostWeights
{
  double w_distance{1.0};
  double w_floor_penalty{100.0};
  double w_door_penalty{5.0};
  double w_elevator_penalty{30.0};
  double w_stair_factor{0.5};
  double w_user_pref_mult{10.0};
};

struct UserPrefs
{
  bool prefer_no_stairs{false};
  bool prefer_no_elevator{false};
};

/// Compute the cost of traversing an edge in the from→to direction.
/// `from_node` and `to_node` are passed so the function can look at floor
/// transitions without taking another graph lock.
double computeEdgeCost(
  const TopoEdge & edge,
  const TopoNode & from_node,
  const TopoNode & to_node,
  const CostWeights & w,
  const UserPrefs & prefs);

/// A* heuristic: planar Euclidean distance plus a soft cross-floor term.
/// Keeps admissibility as long as `w_floor_penalty` outweighs the realistic
/// minimum cross-floor traversal cost in the actual graph.
double computeHeuristic(
  const TopoNode & a,
  const TopoNode & b,
  const CostWeights & w);

}  // namespace nav2_route_planner

#endif  // NAV2_ROUTE_PLANNER__COST_FUNCTION_HPP_
