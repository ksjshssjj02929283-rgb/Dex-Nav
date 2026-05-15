// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#include "nav2_route_planner/intelligent_router.hpp"

#include <cmath>
#include <sstream>
#include <utility>

namespace nav2_route_planner
{

namespace
{

/// Approach distance for a projection result. ON_NODE → distance to that
/// node; ON_EDGE → distance to the projected_xy point; INVALID → 0.0
/// (caller should already have rejected this case).
double approachDistance(
  const geometry_msgs::msg::PoseStamped & pose,
  const StartGoalProjector::Result & proj,
  const TopoGraph & graph)
{
  const double px = pose.pose.position.x;
  const double py = pose.pose.position.y;
  if (proj.type == StartGoalProjector::ResultType::ON_NODE) {
    auto n_opt = graph.getNode(proj.id);
    if (!n_opt) {return 0.0;}
    return std::hypot(n_opt->x - px, n_opt->y - py);
  }
  if (proj.type == StartGoalProjector::ResultType::ON_EDGE) {
    return std::hypot(proj.projected_xy.first - px,
        proj.projected_xy.second - py);
  }
  return 0.0;
}

}  // namespace

double IntelligentRouter::pathLength(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  double total = 0.0;
  for (std::size_t i = 1; i < poses.size(); ++i) {
    total += std::hypot(
      poses[i].pose.position.x - poses[i - 1].pose.position.x,
      poses[i].pose.position.y - poses[i - 1].pose.position.y);
  }
  return total;
}

double IntelligentRouter::estimateRouteDistance(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const StartGoalProjector::Result & start_proj,
  const StartGoalProjector::Result & goal_proj,
  const AStarSearcher::Result & graph_search,
  const TopoGraph & graph)
{
  // Approach legs: start → start_proj, goal_proj → goal. Both are
  // pure Euclidean — Theta\* may shorten or lengthen the actual leg
  // (around obstacles), but the user spec compares straight-line
  // distances. Using Theta\* lengths here would penalise the route
  // for obstacles that *aren't even on the chosen path*.
  const double start_approach = approachDistance(start, start_proj, graph);
  const double goal_approach = approachDistance(goal, goal_proj, graph);

  // Graph leg: sum of edge.length (geometric, NOT A\* cost which carries
  // floor/elevator/door penalties). When start_proj is ON_EDGE the A\*
  // entered partway along the edge — `length` overstates the actual
  // distance walked. We compensate by subtracting the unwalked portion
  // of that edge using `edge_sample_index / edge_total_samples`.
  double graph_leg = 0.0;
  for (const auto & eid : graph_search.edge_ids) {
    auto e_opt = graph.getEdge(eid);
    if (!e_opt) {continue;}
    graph_leg += e_opt->length;
  }

  if (start_proj.type == StartGoalProjector::ResultType::ON_EDGE &&
      !graph_search.edge_ids.empty() &&
      graph_search.edge_ids.front() == start_proj.id)
  {
    auto e_opt = graph.getEdge(start_proj.id);
    if (e_opt && start_proj.edge_total_samples > 1) {
      // A\* picks whichever endpoint of the projected edge is cheaper,
      // and `edge_sample_index` measures position along the from→to
      // direction. The walked portion of the first edge equals the
      // distance from the projection point to the chosen entry node;
      // the unwalked portion is the rest.
      const double total = static_cast<double>(
        start_proj.edge_total_samples - 1);
      const double t_from = static_cast<double>(start_proj.edge_sample_index) /
        total;
      // If the entry node is the edge's from-side, the walked part is
      // `t_from * length`. We can't tell from here which side A\* picked
      // without scanning node_ids[0], so be conservative: subtract
      // min(t_from, 1 - t_from) * length — this overestimates the route
      // (charges full edge even when only the short side was walked),
      // biasing the decision toward DIRECT which is exactly what the
      // user spec asks for when distances are close.
      const double unwalked_fraction = std::min(t_from, 1.0 - t_from);
      graph_leg -= unwalked_fraction * e_opt->length;
      if (graph_leg < 0.0) {graph_leg = 0.0;}
    }
  }

  return start_approach + graph_leg + goal_approach;
}

IntelligentRouter::Decision IntelligentRouter::decide(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const StartGoalProjector::Result & start_proj,
  const StartGoalProjector::Result & goal_proj,
  const AStarSearcher::Result & graph_search,
  const TopoGraph & graph,
  CostmapAwareLayer & ca_layer) const
{
  Decision d;

  // Master switch — when the router is disabled, behave like the v1
  // planner: always go through the graph. Caller is responsible for
  // ensuring `graph_search.success`; we don't second-guess them here.
  if (!p_.enable) {
    d.strategy = Strategy::ROUTE_VIA_GRAPH;
    d.reason = "router disabled";
    return d;
  }

  // Step 1 — attempt the direct Theta\* shot once.
  auto direct = ca_layer.planWithThetaStar(start, goal);
  if (direct && !direct->empty()) {
    d.direct_feasible = true;
    d.direct_distance = pathLength(*direct);
    d.direct_path = std::move(*direct);
  } else {
    d.direct_feasible = false;
  }

  // Step 2 — estimate the graph route (only if A\* succeeded).
  if (graph_search.success) {
    d.route_distance = estimateRouteDistance(
      start, goal, start_proj, goal_proj, graph_search, graph);
  } else {
    d.route_distance = 0.0;
  }

  // Step 3 — pick strategy.
  std::ostringstream why;
  if (!graph_search.success && d.direct_feasible) {
    d.strategy = Strategy::DIRECT_THETA_STAR;
    why << "graph A* failed, direct Theta* succeeded (" <<
      d.direct_distance << "m)";
    d.reason = why.str();
    return d;
  }
  if (!d.direct_feasible && graph_search.success) {
    d.strategy = Strategy::ROUTE_VIA_GRAPH;
    why << "direct Theta* infeasible, falling back to graph (" <<
      d.route_distance << "m)";
    d.reason = why.str();
    return d;
  }
  if (!graph_search.success && !d.direct_feasible) {
    // Both failed — let the caller handle this by surfacing an empty
    // plan. Pick ROUTE so the caller's existing error path fires.
    d.strategy = Strategy::ROUTE_VIA_GRAPH;
    d.reason = "both direct Theta* and graph A* failed";
    return d;
  }

  // Step 4 — both feasible, do the race.
  if (p_.min_graph_route_m > 0.0 &&
      d.route_distance > 0.0 &&
      d.route_distance < p_.min_graph_route_m)
  {
    d.strategy = Strategy::DIRECT_THETA_STAR;
    why << "graph route too short (" << d.route_distance <<
      "m < min " << p_.min_graph_route_m <<
      "m); direct=" << d.direct_distance << "m";
    d.reason = why.str();
    return d;
  }

  const double effective_direct =
    d.direct_distance * p_.graph_preference_ratio;
  if (effective_direct < d.route_distance) {
    d.strategy = Strategy::DIRECT_THETA_STAR;
    why << "direct=" << d.direct_distance << "m * ratio " <<
      p_.graph_preference_ratio << " < route=" << d.route_distance << "m";
  } else {
    d.strategy = Strategy::ROUTE_VIA_GRAPH;
    why << "direct=" << d.direct_distance << "m * ratio " <<
      p_.graph_preference_ratio << " >= route=" << d.route_distance << "m";
  }
  d.reason = why.str();
  return d;
}

}  // namespace nav2_route_planner
