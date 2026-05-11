// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#include "nav2_route_planner/start_goal_projector.hpp"

#include <cmath>
#include <limits>

namespace nav2_route_planner
{

std::pair<double, double> StartGoalProjector::verticalProject(
  double px, double py,
  double x1, double y1, double x2, double y2,
  bool & on_segment)
{
  const double dx = x2 - x1;
  const double dy = y2 - y1;
  const double d2 = dx * dx + dy * dy;
  if (d2 < 1e-9) {
    on_segment = true;
    return {x1, y1};
  }
  const double t = ((px - x1) * dx + (py - y1) * dy) / d2;
  if (t <= 0.0) {
    on_segment = false;
    return {x1, y1};
  }
  if (t >= 1.0) {
    on_segment = false;
    return {x2, y2};
  }
  on_segment = true;
  return {x1 + t * dx, y1 + t * dy};
}

StartGoalProjector::Result StartGoalProjector::project(
  const geometry_msgs::msg::PoseStamped & pose,
  const TopoGraph & graph) const
{
  Result r;
  const double px = pose.pose.position.x;
  const double py = pose.pose.position.y;

  // Step 1: nearest NavPoint within node_threshold.
  double best_d = p_.node_threshold_m;
  for (const auto & [id, n] : graph.nodes()) {
    if (n.kind != NodeKind::NAV_POINT) {continue;}  // goal/start prefer NavPoints
    const double d = std::hypot(n.x - px, n.y - py);
    if (d < best_d) {
      best_d = d;
      r.type = ResultType::ON_NODE;
      r.id = id;
      r.distance = d;
    }
  }
  if (r.type == ResultType::ON_NODE) {
    return r;
  }

  // Step 2: nearest edge within edge_threshold.
  best_d = p_.edge_threshold_m;
  for (const auto & [id, e] : graph.edges()) {
    if (!e.enabled) {continue;}
    auto samples = graph.sampleEdge(e, p_.bezier_samples_per_edge);
    double seg_best = std::numeric_limits<double>::infinity();
    std::size_t best_idx = 0;
    std::pair<double, double> best_xy{0, 0};
    if (e.geometry == EdgeGeometry::LINE && samples.size() == 2) {
      bool on_seg = false;
      auto proj = verticalProject(px, py,
          samples[0].first, samples[0].second,
          samples[1].first, samples[1].second,
          on_seg);
      const double d = std::hypot(proj.first - px, proj.second - py);
      seg_best = d;
      best_xy = proj;
      // Index 0 = at from end, 1 = at to end. With on_seg true, store
      // as half-step (1) so interpolator knows we're past the start.
      best_idx = on_seg ? 1u : 0u;
    } else {
      for (std::size_t i = 0; i < samples.size(); ++i) {
        const double d = std::hypot(samples[i].first - px, samples[i].second - py);
        if (d < seg_best) {
          seg_best = d;
          best_idx = i;
          best_xy = samples[i];
        }
      }
    }
    if (seg_best < best_d) {
      best_d = seg_best;
      r.type = ResultType::ON_EDGE;
      r.id = id;
      r.distance = seg_best;
      r.projected_xy = best_xy;
      r.edge_sample_index = best_idx;
      r.edge_total_samples = samples.size();
    }
  }

  return r;  // INVALID by default if no edge match within threshold
}

std::optional<std::string> StartGoalProjector::projectToNode(
  const geometry_msgs::msg::PoseStamped & pose,
  const TopoGraph & graph) const
{
  const double px = pose.pose.position.x;
  const double py = pose.pose.position.y;
  std::string best_id;
  double best_d = p_.node_threshold_m;
  for (const auto & [id, n] : graph.nodes()) {
    if (n.kind != NodeKind::NAV_POINT) {continue;}
    const double d = std::hypot(n.x - px, n.y - py);
    if (d < best_d) {
      best_d = d;
      best_id = id;
    }
  }
  if (best_id.empty()) {return std::nullopt;}
  return best_id;
}

}  // namespace nav2_route_planner
