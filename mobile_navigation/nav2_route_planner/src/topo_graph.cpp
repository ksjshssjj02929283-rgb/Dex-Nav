// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#include "nav2_route_planner/topo_graph.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <utility>

namespace nav2_route_planner
{

void TopoGraph::rebuildFrom(
  std::vector<TopoNode> nodes,
  std::vector<TopoEdge> edges)
{
  std::unique_lock<std::shared_mutex> lk(mtx_);
  nodes_.clear();
  edges_.clear();
  nodes_.reserve(nodes.size());
  edges_.reserve(edges.size());

  for (auto & n : nodes) {
    nodes_.emplace(n.id, std::move(n));
  }

  // Pre-compute edge length and wire up node adjacency lists.
  for (auto & e : edges) {
    auto from_it = nodes_.find(e.from_id);
    auto to_it = nodes_.find(e.to_id);
    if (from_it == nodes_.end() || to_it == nodes_.end()) {
      // Skip dangling edges; loader is the canonical place for diagnostics.
      continue;
    }
    if (e.length <= 0.0) {
      e.length = computeEdgeLength(e);
    }
    if (e.floor.empty()) {
      e.floor = from_it->second.floor;
    }
    from_it->second.outgoing_edge_ids.push_back(e.id);
    to_it->second.incoming_edge_ids.push_back(e.id);
    if (e.bidirectional) {
      to_it->second.outgoing_edge_ids.push_back(e.id);
      from_it->second.incoming_edge_ids.push_back(e.id);
    }
    edges_.emplace(e.id, std::move(e));
  }

  ready_.store(true, std::memory_order_release);
}

std::optional<TopoNode> TopoGraph::getNode(const std::string & id) const
{
  std::shared_lock<std::shared_mutex> lk(mtx_);
  auto it = nodes_.find(id);
  if (it == nodes_.end()) {
    return std::nullopt;
  }
  return it->second;
}

std::optional<TopoEdge> TopoGraph::getEdge(const std::string & id) const
{
  std::shared_lock<std::shared_mutex> lk(mtx_);
  auto it = edges_.find(id);
  if (it == edges_.end()) {
    return std::nullopt;
  }
  return it->second;
}

std::vector<const TopoEdge *> TopoGraph::outgoing(const std::string & node_id) const
{
  std::vector<const TopoEdge *> out;
  auto it = nodes_.find(node_id);
  if (it == nodes_.end()) {
    return out;
  }
  out.reserve(it->second.outgoing_edge_ids.size());
  for (const auto & eid : it->second.outgoing_edge_ids) {
    auto eit = edges_.find(eid);
    if (eit != edges_.end() && eit->second.enabled) {
      out.push_back(&eit->second);
    }
  }
  return out;
}

std::vector<std::string> TopoGraph::findNearestNodes(
  double x, double y, double radius, std::size_t max_count) const
{
  std::vector<std::pair<double, std::string>> hits;
  const double r2 = radius * radius;
  for (const auto & [id, n] : nodes_) {
    double dx = n.x - x;
    double dy = n.y - y;
    double d2 = dx * dx + dy * dy;
    if (d2 <= r2) {
      hits.emplace_back(d2, id);
    }
  }
  std::sort(hits.begin(), hits.end(),
    [](const auto & a, const auto & b) {return a.first < b.first;});
  if (hits.size() > max_count) {
    hits.resize(max_count);
  }
  std::vector<std::string> ids;
  ids.reserve(hits.size());
  for (auto & h : hits) {
    ids.push_back(std::move(h.second));
  }
  return ids;
}

std::vector<std::string> TopoGraph::findNearestEdges(
  double x, double y, double radius, std::size_t max_count) const
{
  std::vector<std::pair<double, std::string>> hits;
  for (const auto & [id, e] : edges_) {
    if (!e.enabled) {
      continue;
    }
    auto samples = sampleEdge(e);
    double best = std::numeric_limits<double>::infinity();
    for (const auto & p : samples) {
      double dx = p.first - x;
      double dy = p.second - y;
      double d = std::hypot(dx, dy);
      if (d < best) {best = d;}
    }
    if (best <= radius) {
      hits.emplace_back(best, id);
    }
  }
  std::sort(hits.begin(), hits.end(),
    [](const auto & a, const auto & b) {return a.first < b.first;});
  if (hits.size() > max_count) {
    hits.resize(max_count);
  }
  std::vector<std::string> ids;
  ids.reserve(hits.size());
  for (auto & h : hits) {
    ids.push_back(std::move(h.second));
  }
  return ids;
}

std::vector<std::pair<double, double>> TopoGraph::sampleEdge(
  const TopoEdge & edge,
  int num_bezier_samples) const
{
  // No external lock here — caller already holds shared lock for sampleEdge
  // when invoked from findNearestEdges; outgoing/getNode below do not
  // re-lock because we use the internal map directly.
  auto from_it = nodes_.find(edge.from_id);
  auto to_it = nodes_.find(edge.to_id);
  if (from_it == nodes_.end() || to_it == nodes_.end()) {
    return {};
  }

  if (edge.geometry == EdgeGeometry::LINE || edge.control_points.size() < 2) {
    return {{from_it->second.x, from_it->second.y},
            {to_it->second.x, to_it->second.y}};
  }

  // Bernstein-Bézier sampling, num_bezier_samples + 1 points (inclusive).
  std::vector<std::pair<double, double>> out;
  out.reserve(static_cast<std::size_t>(num_bezier_samples) + 1);
  const auto & cp = edge.control_points;
  const int n = static_cast<int>(cp.size()) - 1;
  for (int s = 0; s <= num_bezier_samples; ++s) {
    double t = static_cast<double>(s) / num_bezier_samples;
    double bx = 0.0;
    double by = 0.0;
    for (int i = 0; i <= n; ++i) {
      double b = binom(n, i) * std::pow(1.0 - t, n - i) * std::pow(t, i);
      bx += b * cp[i].first;
      by += b * cp[i].second;
    }
    out.emplace_back(bx, by);
  }
  return out;
}

double TopoGraph::computeEdgeLength(const TopoEdge & edge, int num_bezier_samples) const
{
  auto pts = sampleEdge(edge, num_bezier_samples);
  if (pts.size() < 2) {
    return 0.0;
  }
  double total = 0.0;
  for (std::size_t i = 1; i < pts.size(); ++i) {
    total += std::hypot(pts[i].first - pts[i - 1].first,
                        pts[i].second - pts[i - 1].second);
  }
  return total;
}

double TopoGraph::binom(int n, int k)
{
  if (k == 0 || k == n) {return 1.0;}
  double v = 1.0;
  for (int i = 0; i < k; ++i) {
    v = v * (n - i) / (i + 1);
  }
  return v;
}

}  // namespace nav2_route_planner
