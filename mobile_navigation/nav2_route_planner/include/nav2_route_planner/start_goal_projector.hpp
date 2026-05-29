// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#ifndef NAV2_ROUTE_PLANNER__START_GOAL_PROJECTOR_HPP_
#define NAV2_ROUTE_PLANNER__START_GOAL_PROJECTOR_HPP_

#include <limits>
#include <optional>
#include <string>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_route_planner/topo_graph.hpp"

namespace nav2_route_planner
{

/// Projects a free-space pose onto the road graph.
///
/// Two operating modes:
///   • Bounded (`project` / `projectToNode`): mirrors ROS1
///     `GetPointId`/`GetLineId`, uses `node_threshold` and `edge_threshold`
///     to reject far-away poses with INVALID/nullopt. Used by callers that
///     want a hard "must be on the graph" contract.
///   • Unlimited (`projectUnlimited` / `projectToNearestNode`): drops the
///     threshold and always returns the globally nearest NODE or EDGE
///     projection. Used by 2026-05 "arbitrary start/goal" extension —
///     callers above use Theta\* to bridge `pose ↔ projected point`.
///
/// Both modes apply the SAME fair-competition rule (DEC-025 / DEC-019):
///   • compute the nearest NODE distance (across NAV_POINT + JUNCTION)
///   • compute the nearest EDGE-projection distance (across enabled edges)
///   • smaller distance wins. On exact tie EDGE wins (it carries direction
///     info so we won't U-turn at the start).
class StartGoalProjector
{
public:
  enum class ResultType { ON_NODE, ON_EDGE, INVALID };

  struct Result
  {
    ResultType type{ResultType::INVALID};
    std::string id;                          // node_id or edge_id
    double distance{0.0};
    std::pair<double, double> projected_xy{0.0, 0.0};  // valid for ON_EDGE
    /// For ON_EDGE: index along the sampled polyline (used by interpolator
    /// to know where on the edge we joined). 0 = at from_id end.
    std::size_t edge_sample_index{0};
    std::size_t edge_total_samples{0};
    /// Diagnostic: best NODE / EDGE distances seen during the scan. Useful
    /// for the IntelligentRouter to reason about "how far is pose from the
    /// graph" without re-running the scan. Both are `+inf` if the graph
    /// had no nodes / no enabled edges respectively.
    double best_node_distance{std::numeric_limits<double>::infinity()};
    double best_edge_distance{std::numeric_limits<double>::infinity()};
  };

  struct Params
  {
    double node_threshold_m{0.5};
    double edge_threshold_m{1.0};
    int bezier_samples_per_edge{40};
  };

  explicit StartGoalProjector(Params p) : p_(p) {}

  /// Bounded project. Caller must hold graph.lockShared(). Returns
  /// type=INVALID when both thresholds miss — used by code paths that
  /// REQUIRE the pose be on (or near) the graph.
  Result project(
    const geometry_msgs::msg::PoseStamped & pose,
    const TopoGraph & graph) const;

  /// Bounded project-to-node. Returns nullopt when no node within
  /// `node_threshold`.
  std::optional<std::string> projectToNode(
    const geometry_msgs::msg::PoseStamped & pose,
    const TopoGraph & graph) const;

  /// Unlimited project — always returns the globally nearest NODE or
  /// EDGE projection, regardless of how far the pose is from the graph.
  /// Only returns INVALID when the graph is empty (no nodes AND no
  /// enabled edges). Caller must hold graph.lockShared().
  ///
  /// Tie-break: NODE/EDGE compete on plain distance; on exact tie, EDGE
  /// wins (carries direction info). This matches the bounded `project`
  /// behavior so unit tests written against bounded mode keep passing
  /// when the thresholds are relaxed.
  Result projectUnlimited(
    const geometry_msgs::msg::PoseStamped & pose,
    const TopoGraph & graph) const;

  /// Unlimited project-to-node — always returns the globally nearest
  /// node id (NAV_POINT or JUNCTION). Returns nullopt only when the
  /// graph has zero nodes.
  std::optional<std::string> projectToNearestNode(
    const geometry_msgs::msg::PoseStamped & pose,
    const TopoGraph & graph) const;

  /// EDGE-first project — always returns ON_EDGE projection if the
  /// graph has any enabled edge (the globally nearest edge projection,
  /// even if some node is geometrically closer). Falls back to ON_NODE
  /// only when the graph has zero enabled edges. INVALID only when
  /// the graph is completely empty. Caller must hold graph.lockShared().
  ///
  /// Rationale (DEC-011, 2026-05-18 user feedback): the plan must stay
  /// ON the road graph. Projecting to a NavPoint when the same pose
  /// could project onto an adjacent edge produces plans where the
  /// approach leg briefly leaves the network (a NavPoint sitting just
  /// off an edge is visible in the user's screenshots). Forcing
  /// ON_EDGE keeps the plan glued to graph geometry; the "near a
  /// NavPoint" degenerate case naturally produces an ON_EDGE result
  /// with `edge_sample_index` at the matching endpoint.
  Result projectPreferEdge(
    const geometry_msgs::msg::PoseStamped & pose,
    const TopoGraph & graph) const;

  /// Helper: given a `Result` of type ON_EDGE, return whichever endpoint
  /// of the edge is geometrically closer to `pose`. Used by callers that
  /// need an A\* goal-node id when the goal projects onto an edge.
  /// Returns nullopt if the edge or its endpoints are missing from the
  /// graph. Caller must hold graph.lockShared().
  static std::optional<std::string> nearerEndpointOfEdge(
    const Result & edge_proj,
    const geometry_msgs::msg::PoseStamped & pose,
    const TopoGraph & graph);

  /// Vertical projection of (px,py) onto segment (x1,y1)-(x2,y2).
  /// `on_segment` is true when projection lies within the segment;
  /// otherwise the nearest endpoint is returned.
  static std::pair<double, double> verticalProject(
    double px, double py,
    double x1, double y1, double x2, double y2,
    bool & on_segment);

  /// Fractional position of point `p` along the chord from (x1,y1) to
  /// (x2,y2), clamped to [0,1]: 0 at the `from` end, 1 at the `to` end.
  ///
  /// This is the position measure callers should use when weighting the
  /// partial-edge cost of a pose that projected ON_EDGE. It is exact for
  /// LINE edges (where `p` already lies on the chord) and a monotonic,
  /// bounded estimate for BEZIER edges. `edge_sample_index` must NOT be
  /// used for this: a 2-sample LINE edge reports index 0 for ANY interior
  /// projection, which collapses the cost and makes the planner treat the
  /// robot as sitting at the `from` endpoint (the root cause of the
  /// "loop back to the junction" bug).
  static double fractionAlongChord(
    const std::pair<double, double> & p,
    double x1, double y1, double x2, double y2);

private:
  /// Internal shared scan: walks every node and every enabled edge,
  /// records the best NODE distance and best EDGE projection. `apply_*`
  /// flags allow callers to skip one side. Caller must hold lockShared().
  Result scanGraph(
    const geometry_msgs::msg::PoseStamped & pose,
    const TopoGraph & graph,
    bool apply_node_threshold,
    bool apply_edge_threshold) const;

  Params p_;
};

}  // namespace nav2_route_planner

#endif  // NAV2_ROUTE_PLANNER__START_GOAL_PROJECTOR_HPP_
