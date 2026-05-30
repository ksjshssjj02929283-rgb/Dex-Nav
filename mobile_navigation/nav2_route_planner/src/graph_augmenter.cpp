// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#include "nav2_route_planner/graph_augmenter.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace nav2_route_planner
{

namespace
{

using XY = std::pair<double, double>;

/// de Casteljau split of a Bezier curve (control points `cp`) at parameter
/// t ∈ [0,1]. Returns {left, right}, each of the same degree as `cp`. The
/// split point equals left.back() == right.front() == curve(t).
std::pair<std::vector<XY>, std::vector<XY>> deCasteljauSplit(
  const std::vector<XY> & cp, double t)
{
  std::vector<XY> left, right;
  std::vector<XY> pts = cp;
  left.push_back(pts.front());
  right.push_back(pts.back());
  while (pts.size() > 1) {
    std::vector<XY> nxt(pts.size() - 1);
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
      nxt[i] = {
        pts[i].first + (pts[i + 1].first - pts[i].first) * t,
        pts[i].second + (pts[i + 1].second - pts[i].second) * t};
    }
    left.push_back(nxt.front());
    right.push_back(nxt.back());
    pts = std::move(nxt);
  }
  std::reverse(right.begin(), right.end());
  return {std::move(left), std::move(right)};
}

/// Estimate the Bezier parameter t for an ON_EDGE projection: the FLOOR
/// sample index plus the chord fraction within that sample segment.
double estimateBezierT(
  const StartGoalProjector::Result & proj,
  const std::vector<XY> & samples)
{
  if (samples.size() < 2) {return 0.0;}
  const std::size_t n = samples.size() - 1;  // number of segments
  std::size_t i = std::min(proj.edge_sample_index, n - 1);
  const double local = StartGoalProjector::fractionAlongChord(
    proj.projected_xy,
    samples[i].first, samples[i].second,
    samples[i + 1].first, samples[i + 1].second);
  return (static_cast<double>(i) + local) / static_cast<double>(n);
}

/// One projection to splice into an edge, with the virtual node id it
/// should become.
struct SplitReq
{
  const StartGoalProjector::Result * proj;
  std::string vnode_id;
};

/// Split `e` (read from `base`) at one or two projections, appending the
/// virtual node(s) and the resulting sub-edges to `out_nodes` / `out_edges`.
void splitEdge(
  const TopoEdge & e,
  const TopoGraph & base,
  int bezier_samples,
  std::vector<SplitReq> reqs,
  std::vector<TopoNode> & out_nodes,
  std::vector<TopoEdge> & out_edges)
{
  auto from = base.getNode(e.from_id);
  auto to = base.getNode(e.to_id);
  if (!from || !to) {return;}

  const bool is_bezier =
    e.geometry == EdgeGeometry::BEZIER && e.control_points.size() >= 2;

  // Compute the split parameter t for each projection.
  struct Cut
  {
    double t;
    std::string vid;
    XY xy;  // virtual node position (filled below for LINE; for BEZIER it
            // is overwritten with the exact curve point during the split)
  };
  std::vector<Cut> cuts;
  cuts.reserve(reqs.size());
  if (is_bezier) {
    const auto samples = base.sampleEdge(e, bezier_samples);
    for (const auto & r : reqs) {
      cuts.push_back({estimateBezierT(*r.proj, samples), r.vnode_id,
          r.proj->projected_xy});
    }
  } else {
    for (const auto & r : reqs) {
      const double t = StartGoalProjector::fractionAlongChord(
        r.proj->projected_xy, from->x, from->y, to->x, to->y);
      cuts.push_back({t, r.vnode_id, r.proj->projected_xy});
    }
  }
  std::sort(cuts.begin(), cuts.end(),
    [](const Cut & a, const Cut & b) {return a.t < b.t;});

  // Build the ordered node chain (from → V… → to) and, for BEZIER, the
  // control points of each piece.
  std::vector<std::string> chain_ids{e.from_id};
  std::vector<std::vector<XY>> piece_cps;  // BEZIER only

  if (is_bezier) {
    std::vector<XY> remaining = e.control_points;
    double t_prev = 0.0;
    for (auto & c : cuts) {
      const double denom = 1.0 - t_prev;
      double tt = denom > 1e-9 ? (c.t - t_prev) / denom : 0.0;
      tt = std::clamp(tt, 0.0, 1.0);
      auto [left, right] = deCasteljauSplit(remaining, tt);
      c.xy = left.back();  // exact curve point at the cut
      piece_cps.push_back(std::move(left));
      remaining = std::move(right);
      t_prev = c.t;
      chain_ids.push_back(c.vid);
    }
    piece_cps.push_back(std::move(remaining));
  } else {
    for (const auto & c : cuts) {chain_ids.push_back(c.vid);}
  }
  chain_ids.push_back(e.to_id);

  // Emit virtual nodes (chain index k+1 corresponds to cuts[k]).
  for (std::size_t k = 0; k < cuts.size(); ++k) {
    TopoNode v;
    v.id = cuts[k].vid;
    v.kind = NodeKind::NAV_POINT;
    v.x = cuts[k].xy.first;
    v.y = cuts[k].xy.second;
    v.floor = e.floor;
    out_nodes.push_back(std::move(v));
  }

  // Emit one sub-edge per chain segment. Inherit all semantic fields;
  // length=0 so rebuildFrom recomputes it from geometry.
  const std::size_t npieces = chain_ids.size() - 1;
  for (std::size_t i = 0; i < npieces; ++i) {
    TopoEdge se;
    se.id = e.id + "#" + std::to_string(i);
    se.from_id = chain_ids[i];
    se.to_id = chain_ids[i + 1];
    se.geometry = e.geometry;
    se.bidirectional = e.bidirectional;
    se.max_speed = e.max_speed;
    se.xy_tolerance = e.xy_tolerance;
    se.th_tolerance = e.th_tolerance;
    se.lane_width = e.lane_width;
    se.obs_policy = e.obs_policy;
    se.lane_subtype = e.lane_subtype;
    se.floor = e.floor;
    se.regulatory_element_ids = e.regulatory_element_ids;
    se.enabled = e.enabled;
    se.length = 0.0;
    if (is_bezier && i < piece_cps.size()) {
      se.control_points = piece_cps[i];
    }
    out_edges.push_back(std::move(se));
  }
}

}  // namespace

AugmentedGraph buildAugmentedGraph(
  const std::shared_ptr<TopoGraph> & base,
  const StartGoalProjector::Result & start_proj,
  const StartGoalProjector::Result & goal_proj,
  int bezier_samples)
{
  const bool start_edge =
    start_proj.type == StartGoalProjector::ResultType::ON_EDGE;
  const bool goal_edge =
    goal_proj.type == StartGoalProjector::ResultType::ON_EDGE;

  AugmentedGraph out;
  out.start_id = start_edge ? kVirtualStartId : start_proj.id;
  out.goal_id = goal_edge ? kVirtualGoalId : goal_proj.id;

  // No projection on an edge → nothing to split; alias the base graph.
  if (!start_edge && !goal_edge) {
    out.graph = base;
    return out;
  }

  // Copy every base node (with adjacency cleared — rebuildFrom rewires it)
  // and every base edge except the one(s) being split.
  std::unordered_set<std::string> split_ids;
  if (start_edge) {split_ids.insert(start_proj.id);}
  if (goal_edge) {split_ids.insert(goal_proj.id);}

  std::vector<TopoNode> nodes;
  std::vector<TopoEdge> edges;
  nodes.reserve(base->nodes().size() + 2);
  edges.reserve(base->edges().size() + 4);
  for (const auto & [id, n] : base->nodes()) {
    TopoNode c = n;
    c.outgoing_edge_ids.clear();
    c.incoming_edge_ids.clear();
    nodes.push_back(std::move(c));
  }
  for (const auto & [id, e] : base->edges()) {
    if (split_ids.count(id)) {continue;}
    edges.push_back(e);
  }

  const bool same_edge =
    start_edge && goal_edge && start_proj.id == goal_proj.id;

  if (same_edge) {
    auto e = base->getEdge(start_proj.id);
    if (e) {
      splitEdge(*e, *base, bezier_samples,
        {{&start_proj, kVirtualStartId}, {&goal_proj, kVirtualGoalId}},
        nodes, edges);
    }
  } else {
    if (start_edge) {
      auto e = base->getEdge(start_proj.id);
      if (e) {
        splitEdge(*e, *base, bezier_samples,
          {{&start_proj, kVirtualStartId}}, nodes, edges);
      }
    }
    if (goal_edge) {
      auto e = base->getEdge(goal_proj.id);
      if (e) {
        splitEdge(*e, *base, bezier_samples,
          {{&goal_proj, kVirtualGoalId}}, nodes, edges);
      }
    }
  }

  out.graph = std::make_shared<TopoGraph>();
  out.graph->rebuildFrom(std::move(nodes), std::move(edges));
  return out;
}

}  // namespace nav2_route_planner
