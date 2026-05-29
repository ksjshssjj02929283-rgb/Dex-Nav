// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#include "nav2_route_planner/start_goal_projector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace nav2_route_planner
{

namespace
{

constexpr double kSnapEpsilon = 1e-6;

struct EdgeProjection
{
  std::string id;
  double distance{std::numeric_limits<double>::infinity()};
  std::pair<double, double> xy{0.0, 0.0};
  std::size_t sample_index{0};
  std::size_t sample_total{0};
};

/// Compute the best projection of (px, py) onto a single edge's sampled
/// polyline. Returns distance, projected_xy, and `sample_index` in the
/// FLOOR semantic: the index of the last sample BEFORE projected_xy
/// along the from→to direction.
///
/// LINE 2-sample edges:
///   • interior projection (on_seg=true) → idx = 0
///   • snapped to from-end                → idx = 0
///   • snapped to to-end                  → idx = 1
///
/// BEZIER N-sample edges (per-segment vertical scan):
///   • projected_xy in segment [i, i+1]   → idx = i
///   • snapped to samples[i+1] precisely  → idx = i + 1
///
/// This single helper replaced 4 near-identical scattered blocks in
/// scanGraph (bounded/unlimited) and projectPreferEdge re-scan. Floor-
/// index correctness is the foundation of `interpolator`'s walking
/// loop — getting it wrong here was the root cause of DEC-005~014 (a
/// 0.67 m straight-line approach turning into 8 m of U-turn).
EdgeProjection bestProjectionOnEdge(
  const std::vector<std::pair<double, double>> & samples,
  EdgeGeometry geometry,
  double px, double py)
{
  EdgeProjection r;
  r.sample_total = samples.size();
  if (samples.size() < 2) {return r;}

  if (geometry == EdgeGeometry::LINE && samples.size() == 2) {
    bool on_seg = false;
    auto proj = StartGoalProjector::verticalProject(px, py,
        samples[0].first, samples[0].second,
        samples[1].first, samples[1].second,
        on_seg);
    r.distance = std::hypot(proj.first - px, proj.second - py);
    r.xy = proj;
    if (on_seg) {
      r.sample_index = 0;
    } else {
      const double d_from = std::hypot(
        proj.first - samples[0].first,
        proj.second - samples[0].second);
      r.sample_index = (d_from < kSnapEpsilon) ? 0u : 1u;
    }
    return r;
  }

  // BEZIER / multi-sample: scan every adjacent sample pair as a
  // line segment, keep the globally closest projection.
  for (std::size_t i = 0; i + 1 < samples.size(); ++i) {
    bool on_seg = false;
    auto proj = StartGoalProjector::verticalProject(px, py,
        samples[i].first, samples[i].second,
        samples[i + 1].first, samples[i + 1].second,
        on_seg);
    const double d = std::hypot(proj.first - px, proj.second - py);
    if (d < r.distance) {
      r.distance = d;
      r.xy = proj;
      if (on_seg) {
        r.sample_index = i;
      } else {
        const double d_to_to = std::hypot(
          proj.first - samples[i + 1].first,
          proj.second - samples[i + 1].second);
        r.sample_index = (d_to_to < kSnapEpsilon) ? (i + 1) : i;
      }
    }
  }
  return r;
}

/// Distance-only variant: just the closest-distance value, used by
/// diagnostic scans that don't need the index / projected_xy.
double bestProjectionDistance(
  const std::vector<std::pair<double, double>> & samples,
  EdgeGeometry geometry,
  double px, double py)
{
  return bestProjectionOnEdge(samples, geometry, px, py).distance;
}

}  // namespace

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

double StartGoalProjector::fractionAlongChord(
  const std::pair<double, double> & p,
  double x1, double y1, double x2, double y2)
{
  const double dx = x2 - x1;
  const double dy = y2 - y1;
  const double d2 = dx * dx + dy * dy;
  if (d2 < 1e-9) {return 0.0;}
  const double t = ((p.first - x1) * dx + (p.second - y1) * dy) / d2;
  return std::clamp(t, 0.0, 1.0);
}

StartGoalProjector::Result StartGoalProjector::scanGraph(
  const geometry_msgs::msg::PoseStamped & pose,
  const TopoGraph & graph,
  bool apply_node_threshold,
  bool apply_edge_threshold) const
{
  Result r;
  const double px = pose.pose.position.x;
  const double py = pose.pose.position.y;

  // Bounded mode (apply_node_threshold && apply_edge_threshold) keeps
  // the legacy "NODE first" semantics for backward compatibility — both
  // nav points and junctions get a chance to win within `node_threshold`,
  // and only if NONE qualify do we fall through to the edge scan. This
  // is the behaviour the v1 GTests pin (e.g. OnNodeWhenStartCloseToNavPoint).
  //
  // Unlimited mode (both flags false) drops the thresholds entirely and
  // does an honest distance race between the best NODE and the best
  // EDGE — used by the 2026-05 "arbitrary start/goal" extension.

  if (apply_node_threshold && apply_edge_threshold) {
    // ── Legacy path: NODE first within node_threshold ────────────────
    double best_d = p_.node_threshold_m;
    for (const auto & [id, n] : graph.nodes()) {
      const double d = std::hypot(n.x - px, n.y - py);
      if (d < best_d) {
        best_d = d;
        r.type = ResultType::ON_NODE;
        r.id = id;
        r.distance = d;
      }
      if (d < r.best_node_distance) {r.best_node_distance = d;}
    }
    if (r.type == ResultType::ON_NODE) {
      // Still scan edges so `best_edge_distance` is reported correctly
      // (diagnostic), but do not change the result.
      for (const auto & [id, e] : graph.edges()) {
        if (!e.enabled) {continue;}
        auto samples = graph.sampleEdge(e, p_.bezier_samples_per_edge);
        if (samples.size() < 2) {continue;}
        const double d = bestProjectionDistance(samples, e.geometry, px, py);
        if (d < r.best_edge_distance) {r.best_edge_distance = d;}
      }
      return r;
    }

    // No node within threshold — try edges within edge_threshold.
    EdgeProjection best;
    best.distance = p_.edge_threshold_m;
    for (const auto & [id, e] : graph.edges()) {
      if (!e.enabled) {continue;}
      auto samples = graph.sampleEdge(e, p_.bezier_samples_per_edge);
      auto proj = bestProjectionOnEdge(samples, e.geometry, px, py);
      if (proj.sample_total < 2) {continue;}
      if (proj.distance < r.best_edge_distance) {
        r.best_edge_distance = proj.distance;
      }
      if (proj.distance < best.distance) {
        best.distance = proj.distance;
        best.id = id;
        best.xy = proj.xy;
        best.sample_index = proj.sample_index;
        best.sample_total = proj.sample_total;
      }
    }
    if (!best.id.empty()) {
      r.type = ResultType::ON_EDGE;
      r.id = best.id;
      r.distance = best.distance;
      r.projected_xy = best.xy;
      r.edge_sample_index = best.sample_index;
      r.edge_total_samples = best.sample_total;
    }
    return r;
  }

  // ── Unlimited mode: full scan, fair NODE-vs-EDGE distance race ─────
  std::string best_node_id;
  double best_node_d = std::numeric_limits<double>::infinity();
  for (const auto & [id, n] : graph.nodes()) {
    const double d = std::hypot(n.x - px, n.y - py);
    if (d < best_node_d) {
      best_node_d = d;
      best_node_id = id;
    }
  }
  r.best_node_distance = best_node_d;

  EdgeProjection best_edge;
  for (const auto & [id, e] : graph.edges()) {
    if (!e.enabled) {continue;}
    auto samples = graph.sampleEdge(e, p_.bezier_samples_per_edge);
    auto proj = bestProjectionOnEdge(samples, e.geometry, px, py);
    if (proj.sample_total < 2) {continue;}
    if (proj.distance < best_edge.distance) {
      best_edge.distance = proj.distance;
      best_edge.id = id;
      best_edge.xy = proj.xy;
      best_edge.sample_index = proj.sample_index;
      best_edge.sample_total = proj.sample_total;
    }
  }
  r.best_edge_distance = best_edge.distance;

  const bool node_avail = !best_node_id.empty();
  const bool edge_avail = !best_edge.id.empty();
  if (!node_avail && !edge_avail) {
    return r;  // empty graph
  }
  // Smaller distance wins. On exact tie EDGE wins (it carries direction
  // info, prevents start-side U-turn).
  if (node_avail && (!edge_avail || best_node_d < best_edge.distance)) {
    r.type = ResultType::ON_NODE;
    r.id = best_node_id;
    r.distance = best_node_d;
  } else {
    r.type = ResultType::ON_EDGE;
    r.id = best_edge.id;
    r.distance = best_edge.distance;
    r.projected_xy = best_edge.xy;
    r.edge_sample_index = best_edge.sample_index;
    r.edge_total_samples = best_edge.sample_total;
  }
  return r;
}

StartGoalProjector::Result StartGoalProjector::project(
  const geometry_msgs::msg::PoseStamped & pose,
  const TopoGraph & graph) const
{
  return scanGraph(pose, graph, true, true);
}

StartGoalProjector::Result StartGoalProjector::projectUnlimited(
  const geometry_msgs::msg::PoseStamped & pose,
  const TopoGraph & graph) const
{
  return scanGraph(pose, graph, false, false);
}

StartGoalProjector::Result StartGoalProjector::projectPreferEdge(
  const geometry_msgs::msg::PoseStamped & pose,
  const TopoGraph & graph) const
{
  // Start from the unlimited scan (NODE + EDGE both populated). If the
  // graph has any enabled edge, force the result to ON_EDGE regardless
  // of which side won the fair distance race — keeps the plan glued to
  // the road geometry. Only fall back to ON_NODE when there is no
  // enabled edge at all (graph is just isolated nodes / junctions).
  Result r = scanGraph(pose, graph, false, false);
  if (r.type == ResultType::ON_EDGE ||
      !std::isfinite(r.best_edge_distance))
  {
    return r;
  }
  // Some enabled edge exists but the NODE happened to win — re-scan
  // for the best edge so we get its id / projected_xy / sample_index.
  const double px = pose.pose.position.x;
  const double py = pose.pose.position.y;
  EdgeProjection best_edge;
  for (const auto & [id, e] : graph.edges()) {
    if (!e.enabled) {continue;}
    auto samples = graph.sampleEdge(e, p_.bezier_samples_per_edge);
    auto proj = bestProjectionOnEdge(samples, e.geometry, px, py);
    if (proj.sample_total < 2) {continue;}
    if (proj.distance < best_edge.distance) {
      best_edge.distance = proj.distance;
      best_edge.id = id;
      best_edge.xy = proj.xy;
      best_edge.sample_index = proj.sample_index;
      best_edge.sample_total = proj.sample_total;
    }
  }
  if (!best_edge.id.empty()) {
    r.type = ResultType::ON_EDGE;
    r.id = best_edge.id;
    r.distance = best_edge.distance;
    r.projected_xy = best_edge.xy;
    r.edge_sample_index = best_edge.sample_index;
    r.edge_total_samples = best_edge.sample_total;
  }
  return r;
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
    const double d = std::hypot(n.x - px, n.y - py);
    if (d < best_d) {
      best_d = d;
      best_id = id;
    }
  }
  if (best_id.empty()) {return std::nullopt;}
  return best_id;
}

std::optional<std::string> StartGoalProjector::projectToNearestNode(
  const geometry_msgs::msg::PoseStamped & pose,
  const TopoGraph & graph) const
{
  const double px = pose.pose.position.x;
  const double py = pose.pose.position.y;
  std::string best_id;
  double best_d = std::numeric_limits<double>::infinity();
  for (const auto & [id, n] : graph.nodes()) {
    const double d = std::hypot(n.x - px, n.y - py);
    if (d < best_d) {
      best_d = d;
      best_id = id;
    }
  }
  if (best_id.empty()) {return std::nullopt;}
  return best_id;
}

std::optional<std::string> StartGoalProjector::nearerEndpointOfEdge(
  const Result & edge_proj,
  const geometry_msgs::msg::PoseStamped & pose,
  const TopoGraph & graph)
{
  if (edge_proj.type != ResultType::ON_EDGE) {return std::nullopt;}
  auto e_opt = graph.getEdge(edge_proj.id);
  if (!e_opt) {return std::nullopt;}
  auto from_opt = graph.getNode(e_opt->from_id);
  auto to_opt = graph.getNode(e_opt->to_id);
  if (!from_opt || !to_opt) {return std::nullopt;}
  const double px = pose.pose.position.x;
  const double py = pose.pose.position.y;
  const double df = std::hypot(from_opt->x - px, from_opt->y - py);
  const double dt = std::hypot(to_opt->x - px, to_opt->y - py);
  return df <= dt ? from_opt->id : to_opt->id;
}

}  // namespace nav2_route_planner
