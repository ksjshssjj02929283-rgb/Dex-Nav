// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#include "nav2_route_planner/cost_function.hpp"

#include <cmath>

namespace nav2_route_planner
{

double computeEdgeCost(
  const TopoEdge & edge,
  const TopoNode & from_node,
  const TopoNode & to_node,
  const CostWeights & w,
  const UserPrefs & prefs)
{
  double cost = w.w_distance * std::max(edge.length, 0.01);

  // Cross-floor edges incur a flat penalty in addition to length.
  if (from_node.floor != to_node.floor) {
    cost += w.w_floor_penalty;
  }

  // Subtype-driven penalties (proxy for RegulatoryElement until plugin layer
  // is in place — see project doc §3.2.2).
  const std::string & st = edge.lane_subtype;
  if (st == "door" || st == "door_zone") {
    cost += w.w_door_penalty;
  } else if (st == "elevator" || st == "elevator_zone") {
    cost += w.w_elevator_penalty;
    if (prefs.prefer_no_elevator) {
      cost *= w.w_user_pref_mult;
    }
  } else if (st == "stair" || st == "stairway") {
    cost += w.w_stair_factor * std::max(edge.length, 0.01);
    if (prefs.prefer_no_stairs) {
      cost *= w.w_user_pref_mult;
    }
  }
  return cost;
}

double computeHeuristic(
  const TopoNode & a,
  const TopoNode & b,
  const CostWeights & w)
{
  const double d = std::hypot(b.x - a.x, b.y - a.y);
  // Soft cross-floor term — admissible only if w_floor_penalty <= the
  // smallest realistic cross-floor edge cost in the graph. Documented.
  double floor_term = 0.0;
  if (!a.floor.empty() && !b.floor.empty() && a.floor != b.floor) {
    floor_term = w.w_floor_penalty * 0.5;  // discount keeps admissibility
  }
  return w.w_distance * d + floor_term;
}

}  // namespace nav2_route_planner
