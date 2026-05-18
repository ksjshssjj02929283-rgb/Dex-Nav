// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#include "nav2_route_planner/start_goal_projector.hpp"

#include <cmath>
#include <limits>

namespace nav2_route_planner
{

namespace
{

struct EdgeProjection
{
  std::string id;
  double distance{std::numeric_limits<double>::infinity()};
  std::pair<double, double> xy{0.0, 0.0};
  std::size_t sample_index{0};
  std::size_t sample_total{0};
};

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

StartGoalProjector::Result StartGoalProjector::scanGraph(
  const geometry_msgs::msg::PoseStamped & pose,
  const TopoGraph & graph,
  bool apply_node_threshold,
  bool apply_edge_threshold) const
{
  Result r;
  const double px = pose.pose.position.x;
  const double py = pose.pose.position.y;

  // Bounded mode (apply_node_threshold && apply_edge_threshold) keeps the
  // legacy "NODE first" semantics for backward compatibility — both nav
  // points and junctions get a chance to win within `node_threshold`,
  // and only if NONE qualify do we fall through to the edge scan. This
  // is the behaviour the v1 GTests pin (e.g.
  // OnNodeWhenStartCloseToNavPoint) and what the v1 design doc §4.4
  // describes.
  //
  // Unlimited mode (both flags false) drops the thresholds entirely and
  // does an honest distance race between the best NODE and the best
  // EDGE — used by the 2026-05 "arbitrary start/goal" extension where
  // the caller has no upper bound on how far the pose can be from the
  // graph.

  if (apply_node_threshold && apply_edge_threshold) {
    // ── Legacy path: NODE first within node_threshold ───────────────
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
        double seg_best = std::numeric_limits<double>::infinity();
        if (e.geometry == EdgeGeometry::LINE && samples.size() == 2) {
          bool on_seg = false;
          auto proj = verticalProject(px, py,
              samples[0].first, samples[0].second,
              samples[1].first, samples[1].second,
              on_seg);
          seg_best = std::hypot(proj.first - px, proj.second - py);
        } else {
          for (const auto & s : samples) {
            const double d = std::hypot(s.first - px, s.second - py);
            if (d < seg_best) {seg_best = d;}
          }
        }
        if (seg_best < r.best_edge_distance) {
          r.best_edge_distance = seg_best;
        }
      }
      return r;
    }

    // No node within threshold — try edges within edge_threshold.
    EdgeProjection best;
    best.distance = p_.edge_threshold_m;
    for (const auto & [id, e] : graph.edges()) {
      if (!e.enabled) {continue;}
      auto samples = graph.sampleEdge(e, p_.bezier_samples_per_edge);
      if (samples.size() < 2) {continue;}
      double seg_best = std::numeric_limits<double>::infinity();
      std::size_t best_idx = 0;
      std::pair<double, double> best_xy{0, 0};
      if (e.geometry == EdgeGeometry::LINE && samples.size() == 2) {
        bool on_seg = false;
        auto proj = verticalProject(px, py,
            samples[0].first, samples[0].second,
            samples[1].first, samples[1].second,
            on_seg);
        seg_best = std::hypot(proj.first - px, proj.second - py);
        best_xy = proj;
        // DEC-011 fix: `edge_sample_index` is the floor index — the
        // last sample BEFORE projected_xy along the from→to direction.
        // For LINE with 2 samples (indices 0 and 1):
        //   • interior projection (on_seg=true): floor index = 0
        //   • snapped to from-end: 0
        //   • snapped to to-end:   1
        // The legacy `on_seg ? 1u : 0u` returned 1 for interior
        // projections, which the interpolator misread as "stand at the
        // to-end" — producing the from→to→projected_xy U-turn.
        if (on_seg) {
          best_idx = 0;
        } else {
          const double d_from = std::hypot(
            proj.first - samples[0].first,
            proj.second - samples[0].second);
          best_idx = (d_from < 1e-6) ? 0u : 1u;
        }
      } else {
        // DEC-014: per-segment verticalProject + floor index (same as
        // unlimited scanGraph BEZIER branch / DEC-013 fix).
        for (std::size_t i = 0; i + 1 < samples.size(); ++i) {
          bool on_seg = false;
          auto proj = verticalProject(px, py,
              samples[i].first, samples[i].second,
              samples[i + 1].first, samples[i + 1].second,
              on_seg);
          const double d = std::hypot(proj.first - px, proj.second - py);
          if (d < seg_best) {
            seg_best = d;
            best_xy = proj;
            if (on_seg) {
              best_idx = i;
            } else {
              const double d_to_to = std::hypot(
                proj.first - samples[i + 1].first,
                proj.second - samples[i + 1].second);
              best_idx = (d_to_to < 1e-6) ? (i + 1) : i;
            }
          }
        }
      }
      if (seg_best < r.best_edge_distance) {r.best_edge_distance = seg_best;}
      if (seg_best < best.distance) {
        best.distance = seg_best;
        best.id = id;
        best.xy = best_xy;
        best.sample_index = best_idx;
        best.sample_total = samples.size();
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
    if (samples.size() < 2) {continue;}
    double seg_best = std::numeric_limits<double>::infinity();
    std::size_t seg_idx = 0;
    std::pair<double, double> seg_xy{0, 0};
    if (e.geometry == EdgeGeometry::LINE && samples.size() == 2) {
      bool on_seg = false;
      auto proj = verticalProject(px, py,
          samples[0].first, samples[0].second,
          samples[1].first, samples[1].second,
          on_seg);
      seg_best = std::hypot(proj.first - px, proj.second - py);
      seg_xy = proj;
      // DEC-014: floor index — interior = 0, snapped-to-to = 1.
      // Mirrors the bounded scanGraph LINE branch (DEC-011 fix). Without
      // this, interior projections set idx=1, which the interpolator's
      // forward-walk loop misreads as "walk all the way to to-end then
      // back to projected_xy" → multi-meter U-turn on LINE edges.
      if (on_seg) {
        seg_idx = 0;
      } else {
        const double d_from = std::hypot(
          proj.first - samples[0].first,
          proj.second - samples[0].second);
        seg_idx = (d_from < 1e-6) ? 0u : 1u;
      }
    } else {
      // DEC-014: per-segment verticalProject (same as bounded scanGraph
      // BEZIER branch — DEC-013 fix). Returns true floor index.
      for (std::size_t i = 0; i + 1 < samples.size(); ++i) {
        bool on_seg = false;
        auto proj = verticalProject(px, py,
            samples[i].first, samples[i].second,
            samples[i + 1].first, samples[i + 1].second,
            on_seg);
        const double d = std::hypot(proj.first - px, proj.second - py);
        if (d < seg_best) {
          seg_best = d;
          seg_xy = proj;
          if (on_seg) {
            seg_idx = i;
          } else {
            const double d_to_to = std::hypot(
              proj.first - samples[i + 1].first,
              proj.second - samples[i + 1].second);
            seg_idx = (d_to_to < 1e-6) ? (i + 1) : i;
          }
        }
      }
    }
    if (seg_best < best_edge.distance) {
      best_edge.distance = seg_best;
      best_edge.id = id;
      best_edge.xy = seg_xy;
      best_edge.sample_index = seg_idx;
      best_edge.sample_total = samples.size();
    }
  }
  r.best_edge_distance = best_edge.distance;

  const bool node_avail = !best_node_id.empty();
  const bool edge_avail = !best_edge.id.empty();
  if (!node_avail && !edge_avail) {
    return r;  // empty graph
  }
  // DEC-025 fairness: smaller distance wins; on exact tie EDGE wins
  // (carries direction info, prevents start-side U-turn).
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
  return scanGraph(pose, graph,
    /*apply_node_threshold*/ true,
    /*apply_edge_threshold*/ true);
}

StartGoalProjector::Result StartGoalProjector::projectUnlimited(
  const geometry_msgs::msg::PoseStamped & pose,
  const TopoGraph & graph) const
{
  return scanGraph(pose, graph,
    /*apply_node_threshold*/ false,
    /*apply_edge_threshold*/ false);
}

StartGoalProjector::Result StartGoalProjector::projectPreferEdge(
  const geometry_msgs::msg::PoseStamped & pose,
  const TopoGraph & graph) const
{
  // Start from the unlimited scan (NODE + EDGE both populated). If the
  // graph has any enabled edge, force the result to ON_EDGE regardless
  // of which side won the fair distance race — keeps the plan glued
  // to the road geometry. Only fall back to ON_NODE when there is no
  // enabled edge at all (graph is just isolated nodes / junctions).
  Result r = scanGraph(pose, graph,
    /*apply_node_threshold*/ false,
    /*apply_edge_threshold*/ false);
  if (r.type != ResultType::ON_EDGE &&
      std::isfinite(r.best_edge_distance))
  {
    // Some enabled edge exists but the NODE happened to win — re-scan
    // for the best edge so we get its id / projected_xy / sample_index.
    const double px = pose.pose.position.x;
    const double py = pose.pose.position.y;
    EdgeProjection best_edge;
    for (const auto & [id, e] : graph.edges()) {
      if (!e.enabled) {continue;}
      auto samples = graph.sampleEdge(e, p_.bezier_samples_per_edge);
      if (samples.size() < 2) {continue;}
      double seg_best = std::numeric_limits<double>::infinity();
      std::size_t seg_idx = 0;
      std::pair<double, double> seg_xy{0, 0};
      if (e.geometry == EdgeGeometry::LINE && samples.size() == 2) {
        bool on_seg = false;
        auto proj = verticalProject(px, py,
            samples[0].first, samples[0].second,
            samples[1].first, samples[1].second,
            on_seg);
        seg_best = std::hypot(proj.first - px, proj.second - py);
        seg_xy = proj;
        if (on_seg) {
          seg_idx = 0;
        } else {
          const double d_from = std::hypot(
            proj.first - samples[0].first,
            proj.second - samples[0].second);
          seg_idx = (d_from < 1e-6) ? 0u : 1u;
        }
      } else {
        // DEC-013: per-segment vertical projection along the sampled
        // polyline. The legacy "nearest sample" gave seg_idx = closest
        // sample, which is NOT the "floor index" semantic the
        // interpolator's walking loop requires — for curves it can sit
        // one step PAST projected_xy and cause the walker to overshoot
        // then double back. The per-segment vertical scan finds the
        // exact projection and returns the floor index (segment's
        // from-end).
        for (std::size_t i = 0; i + 1 < samples.size(); ++i) {
          bool on_seg = false;
          auto proj = verticalProject(px, py,
              samples[i].first, samples[i].second,
              samples[i + 1].first, samples[i + 1].second,
              on_seg);
          const double d = std::hypot(proj.first - px, proj.second - py);
          if (d < seg_best) {
            seg_best = d;
            seg_xy = proj;
            if (on_seg) {
              seg_idx = i;
            } else {
              // Snapped to one of the segment endpoints.
              const double d_to_to = std::hypot(
                proj.first - samples[i + 1].first,
                proj.second - samples[i + 1].second);
              seg_idx = (d_to_to < 1e-6) ? (i + 1) : i;
            }
          }
        }
      }
      if (seg_best < best_edge.distance) {
        best_edge.distance = seg_best;
        best_edge.id = id;
        best_edge.xy = seg_xy;
        best_edge.sample_index = seg_idx;
        best_edge.sample_total = samples.size();
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
