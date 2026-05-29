// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#include "nav2_route_planner/astar_searcher.hpp"

#include <algorithm>
#include <limits>
#include <queue>
#include <unordered_map>

namespace nav2_route_planner
{

namespace
{

struct OpenEntry
{
  double f;
  std::string node_id;
};

struct OpenCmp
{
  bool operator()(const OpenEntry & a, const OpenEntry & b) const
  {
    return a.f > b.f;  // min-heap
  }
};

struct CameFrom
{
  std::string parent_node;
  std::string parent_edge;
};

}  // namespace

AStarSearcher::Result AStarSearcher::search(
  const Request & req, const TopoGraph & graph) const
{
  Result out;

  // Resolve goal node.
  auto goal_opt = [&]() -> const TopoNode * {
      auto it = graph.nodes().find(req.goal_node_id);
      return it != graph.nodes().end() ? &it->second : nullptr;
    }();
  if (!goal_opt) {
    out.failure_reason = "Goal node not found: " + req.goal_node_id;
    return out;
  }
  const TopoNode & goal_node = *goal_opt;

  // Build the initial frontier from the start projection.
  std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenCmp> open;
  std::unordered_map<std::string, double> g_score;
  std::unordered_map<std::string, CameFrom> came_from;

  auto pushFrontier = [&](const std::string & node_id, double g) {
      auto it = graph.nodes().find(node_id);
      if (it == graph.nodes().end()) {return;}
      double h = computeHeuristic(it->second, goal_node, req.weights);
      g_score[node_id] = g;
      open.push({g + h, node_id});
    };

  if (req.start.type == StartGoalProjector::ResultType::ON_NODE) {
    pushFrontier(req.start.id, 0.0);
  } else if (req.start.type == StartGoalProjector::ResultType::ON_EDGE) {
    auto eit = graph.edges().find(req.start.id);
    if (eit == graph.edges().end()) {
      out.failure_reason = "Start projection edge not found: " + req.start.id;
      return out;
    }
    const TopoEdge & e = eit->second;
    auto from_it = graph.nodes().find(e.from_id);
    auto to_it = graph.nodes().find(e.to_id);
    if (from_it == graph.nodes().end() || to_it == graph.nodes().end()) {
      out.failure_reason = "Start projection edge has dangling endpoints";
      return out;
    }
    // Partial-edge cost proportional to the robot's TRUE position along
    // the edge. We measure it from `projected_xy` via the chord fraction,
    // NOT from `edge_sample_index`: a 2-sample LINE edge reports index 0
    // for any interior projection, which would make t_to_from = 0 and
    // anchor the robot at `from` (the "loop back to the junction" bug).
    const double t_to_from = StartGoalProjector::fractionAlongChord(
      req.start.projected_xy,
      from_it->second.x, from_it->second.y,
      to_it->second.x, to_it->second.y);
    const double t_to_to = 1.0 - t_to_from;
    double full = computeEdgeCost(e, from_it->second, to_it->second,
        req.weights, req.prefs);
    pushFrontier(e.from_id, full * t_to_from);
    if (e.bidirectional || true) {
      // We always allow joining either endpoint of the projected edge —
      // direction enforcement happens on subsequent edges.
      pushFrontier(e.to_id, full * t_to_to);
    }
  } else {
    out.failure_reason = "Invalid start projection";
    return out;
  }

  // Standard A* main loop.
  while (!open.empty()) {
    OpenEntry top = open.top();
    open.pop();
    ++out.expanded_nodes;

    if (top.node_id == req.goal_node_id) {
      // Reconstruct path.
      out.success = true;
      out.total_cost = g_score[top.node_id];
      std::vector<std::string> rev_nodes;
      std::vector<std::string> rev_edges;
      rev_nodes.push_back(top.node_id);
      auto cur = top.node_id;
      while (came_from.count(cur)) {
        const auto & cf = came_from[cur];
        rev_edges.push_back(cf.parent_edge);
        rev_nodes.push_back(cf.parent_node);
        cur = cf.parent_node;
      }
      std::reverse(rev_nodes.begin(), rev_nodes.end());
      std::reverse(rev_edges.begin(), rev_edges.end());
      out.node_ids = std::move(rev_nodes);
      out.edge_ids = std::move(rev_edges);
      return out;
    }

    auto it_g = g_score.find(top.node_id);
    if (it_g == g_score.end()) {continue;}
    const double current_g = it_g->second;

    auto cur_it = graph.nodes().find(top.node_id);
    if (cur_it == graph.nodes().end()) {continue;}

    // Expand outgoing edges.
    for (const auto & eid : cur_it->second.outgoing_edge_ids) {
      if (req.blacklisted_edge_ids.count(eid)) {continue;}
      auto eit = graph.edges().find(eid);
      if (eit == graph.edges().end() || !eit->second.enabled) {continue;}
      const TopoEdge & e = eit->second;
      const std::string neighbor =
        (e.from_id == top.node_id) ? e.to_id : e.from_id;
      auto nit = graph.nodes().find(neighbor);
      if (nit == graph.nodes().end()) {continue;}

      // Direction check: a non-bidirectional edge can only be used
      // forward, i.e. from edge.from_id to edge.to_id.
      if (!e.bidirectional && e.from_id != top.node_id) {continue;}

      const double step =
        computeEdgeCost(e, cur_it->second, nit->second, req.weights, req.prefs);
      const double tentative_g = current_g + step;

      auto gn_it = g_score.find(neighbor);
      if (gn_it == g_score.end() || tentative_g < gn_it->second) {
        g_score[neighbor] = tentative_g;
        came_from[neighbor] = {top.node_id, eid};
        const double h = computeHeuristic(nit->second, goal_node, req.weights);
        open.push({tentative_g + h, neighbor});
      }
    }
  }

  out.failure_reason = "No path found";
  return out;
}

}  // namespace nav2_route_planner
