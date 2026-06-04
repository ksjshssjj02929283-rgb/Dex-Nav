// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#ifndef NAV2_ROUTE_PLANNER__TOPO_GRAPH_HPP_
#define NAV2_ROUTE_PLANNER__TOPO_GRAPH_HPP_

#include <atomic>
#include <memory>
#include <optional>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace nav2_route_planner
{

enum class NodeKind { NAV_POINT, JUNCTION };
enum class EdgeGeometry { LINE, BEZIER };

struct TopoNode
{
  std::string id;             // NavPoint.name or RouteJunction.id
  NodeKind kind{NodeKind::NAV_POINT};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  std::string floor{"1"};
  std::string semantic_label;  // human-readable name for LLM grounding
  std::vector<std::string> outgoing_edge_ids;
  std::vector<std::string> incoming_edge_ids;
};

struct TopoEdge
{
  std::string id;
  std::string from_id;
  std::string to_id;
  EdgeGeometry geometry{EdgeGeometry::LINE};
  bool bidirectional{false};
  double length{0.0};          // pre-computed geometric length (m)
  double max_speed{0.0};
  double xy_tolerance{0.3};
  double th_tolerance{0.5};
  double lane_width{0.0};
  std::string obs_policy{"STOP"};
  std::string lane_subtype;     // walkway / corridor / elevator / stair / ...
  std::string floor{"1"};
  std::vector<std::pair<double, double>> control_points;  // BEZIER
  std::vector<std::string> regulatory_element_ids;
  bool enabled{true};
};

/// Thread-safe in-memory road-graph backing the planner.
///
/// Reader-writer lock semantics:
///   - createPlan() takes a shared (read) lock for the entire plan
///   - rebuildFrom() / applyDelta() take an exclusive (write) lock
class TopoGraph
{
public:
  TopoGraph() = default;

  /// Replace entire graph atomically.
  void rebuildFrom(
    std::vector<TopoNode> nodes,
    std::vector<TopoEdge> edges);

  /// True once a rebuildFrom() has completed, regardless of whether the graph
  /// is empty. Use this to know "a load attempt finished" (e.g. to stop a
  /// background poll). For "is there a usable road network?" use isReady().
  bool isLoaded() const { return loaded_.load(std::memory_order_acquire); }

  /// True when the graph is ready to plan on: a load completed AND the graph
  /// is non-empty. An empty (0-node) graph is loaded but NOT ready — the map
  /// has no road network.
  bool isReady() const { return ready_.load(std::memory_order_acquire); }

  /// Read accessors (callers should hold lockShared() across multiple calls
  /// to keep the graph stable; single-call accessors take their own lock).
  std::shared_lock<std::shared_mutex> lockShared() const
  {
    return std::shared_lock<std::shared_mutex>(mtx_);
  }

  /// Single-call thread-safe lookups (each takes its own shared lock).
  std::optional<TopoNode> getNode(const std::string & id) const;
  std::optional<TopoEdge> getEdge(const std::string & id) const;

  /// Iteration helpers — caller MUST hold lockShared() while iterating.
  const std::unordered_map<std::string, TopoNode> & nodes() const { return nodes_; }
  const std::unordered_map<std::string, TopoEdge> & edges() const { return edges_; }

  /// Edges leaving a node (respects bidirectional). Caller must hold lockShared().
  std::vector<const TopoEdge *> outgoing(const std::string & node_id) const;

  /// Spatial queries (brute-force, O(N)). Caller must hold lockShared().
  /// Returns ids sorted by distance ascending, capped by max_count.
  std::vector<std::string> findNearestNodes(
    double x, double y, double radius,
    std::size_t max_count = 10) const;

  /// Find edges whose geometry comes within `radius` of (x,y).
  /// Caller must hold lockShared(). Returns ids sorted by distance ascending.
  std::vector<std::string> findNearestEdges(
    double x, double y, double radius,
    std::size_t max_count = 10) const;

  /// Sample points along an edge (LINE: endpoints + control_points polyline;
  /// BEZIER: Bernstein-sampled). Used by both projector and interpolator.
  /// Order is from from_id → to_id; reverse externally if needed.
  std::vector<std::pair<double, double>> sampleEdge(
    const TopoEdge & edge,
    int num_bezier_samples = 40) const;

  /// Compute planar geometric length of an edge.
  double computeEdgeLength(const TopoEdge & edge,
    int num_bezier_samples = 40) const;

  std::size_t nodeCount() const { return nodes_.size(); }
  std::size_t edgeCount() const { return edges_.size(); }

private:
  // Bernstein binomial coefficient (small N, no caching needed).
  static double binom(int n, int k);

  mutable std::shared_mutex mtx_;
  std::unordered_map<std::string, TopoNode> nodes_;
  std::unordered_map<std::string, TopoEdge> edges_;
  std::atomic<bool> loaded_{false};  // a rebuildFrom() completed (maybe empty)
  std::atomic<bool> ready_{false};   // loaded AND non-empty (usable road network)
};

}  // namespace nav2_route_planner

#endif  // NAV2_ROUTE_PLANNER__TOPO_GRAPH_HPP_
