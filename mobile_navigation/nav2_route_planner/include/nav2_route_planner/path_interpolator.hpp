// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#ifndef NAV2_ROUTE_PLANNER__PATH_INTERPOLATOR_HPP_
#define NAV2_ROUTE_PLANNER__PATH_INTERPOLATOR_HPP_

#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/time.hpp"

#include "nav_interfaces/msg/route_plan.hpp"

#include "nav2_route_planner/astar_searcher.hpp"
#include "nav2_route_planner/start_goal_projector.hpp"
#include "nav2_route_planner/topo_graph.hpp"

namespace nav2_route_planner
{

/// Per-edge sampled segment carrying the semantics that the controller /
/// runtime binder will consume downstream.
struct InterpolatedSegment
{
  std::string edge_id;
  std::string lane_subtype;
  double max_speed{0.0};
  std::vector<std::string> regulatory_element_ids;
  std::string floor;
  std::vector<geometry_msgs::msg::PoseStamped> samples;
};

/// Collected output of `PathInterpolator::interpolate`. Convertible to both
/// `nav_msgs/Path` (controller-friendly, no semantics) and
/// `nav_interfaces/RoutePlan` (RuntimeBinder-friendly, with semantics).
struct InterpolatedRoutePlan
{
  std::string plan_id;
  std::vector<std::string> floor_sequence;
  std::vector<InterpolatedSegment> segments;
  double total_distance{0.0};
  double estimated_time{0.0};
  int floor_changes{0};

  /// Flatten all samples into a single Path. Each sample's frame_id and
  /// stamp will be overwritten by the caller.
  nav_msgs::msg::Path toNavPath(
    const std::string & frame_id,
    const rclcpp::Time & stamp,
    bool use_final_approach_orientation) const;

  /// Build the parallel RoutePlan message for the topic publisher.
  nav_interfaces::msg::RoutePlan toMsg(
    const std::string & frame_id,
    const rclcpp::Time & stamp) const;
};

class PathInterpolator
{
public:
  struct Params
  {
    int waypoints_per_meter{100};
    int bezier_samples_per_edge{40};
    bool use_final_approach_orientation{false};
  };

  explicit PathInterpolator(Params p) : p_(p) {}

  /// Build a complete RoutePlan from the A* search result.
  ///
  /// The output is a sequence of `InterpolatedSegment`s with these
  /// reserved synthetic edge_ids:
  ///   • `__start_transition__` — free-space from `start` to the
  ///     projection point on the graph. ALWAYS emitted. Caller is
  ///     expected to Theta\*-rewrite this segment so the robot avoids
  ///     obstacles between `start` and the projected entry point.
  ///   • `__start_edge_in__`    — sampled along `start_proj.id` from
  ///     `projected_xy` to the A\*-chosen entry node. Only emitted
  ///     when `start_proj.type == ON_EDGE` (otherwise the projection
  ///     point IS the entry node).
  ///   • <per-edge segments>    — one per edge in `search_result.edge_ids`.
  ///   • `__goal_edge_out__`    — sampled along `goal_proj.id` from the
  ///     A\*-exit node to `goal_proj.projected_xy`. Only emitted when
  ///     `goal_proj.type == ON_EDGE`.
  ///   • `__goal_alignment__`   — free-space from the projection point
  ///     to `goal` pose. ALWAYS emitted. Caller is expected to Theta\*-
  ///     rewrite this segment.
  ///
  /// Caller must hold graph.lockShared().
  InterpolatedRoutePlan interpolate(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const StartGoalProjector::Result & start_proj,
    const StartGoalProjector::Result & goal_proj,
    const AStarSearcher::Result & search_result,
    const TopoGraph & graph) const;

  /// Build a plan when `start` and `goal` project onto the SAME edge.
  ///
  /// This is the dedicated path for the "both projections share an edge"
  /// case, which the graph A\* cannot serve: A\* blacklists the goal edge
  /// (DEC-009) to force an approach from a NavPoint endpoint, but here the
  /// start is ON that same edge — blacklisting it forces a detour OUT to
  /// an endpoint and BACK, i.e. a visible U-turn / loop. The fix is to walk
  /// the edge geometry DIRECTLY between the two projection points.
  ///
  /// Emits the same three reserved segments the planner's Theta\*-rewrite
  /// stage expects at the ends — `__start_transition__` (front) and
  /// `__goal_alignment__` (back) — with the real edge segment in between
  /// (kept as-is so the route stays glued to the road). The middle walk
  /// uses `edge_sample_index`'s FLOOR semantic so the first sample never
  /// overshoots the projection (the invariant behind DEC-005~014).
  ///
  /// Preconditions (caller guarantees): start_proj.type == goal_proj.type
  /// == ON_EDGE and start_proj.id == goal_proj.id. Returns an empty plan
  /// (no segments) if the edge is missing from the graph.
  /// Caller must hold graph.lockShared().
  InterpolatedRoutePlan interpolateSameEdge(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const StartGoalProjector::Result & start_proj,
    const StartGoalProjector::Result & goal_proj,
    const TopoGraph & graph) const;

  /// Sample Bernstein-Bézier curve over control points.
  /// Returns `num_samples + 1` points (inclusive of endpoints).
  static std::vector<std::pair<double, double>> sampleBezier(
    const std::vector<std::pair<double, double>> & control_points,
    int num_samples);

private:
  /// Add poses interpolating linearly between two (x,y) endpoints with
  /// `waypoints_per_meter` density. Yaw is left zero; computed at the end.
  void emitLinear(
    std::vector<geometry_msgs::msg::PoseStamped> & out,
    double x1, double y1, double x2, double y2) const;

  /// Compute yaw at each pose from forward differences (last copies prev).
  static void computeOrientations(std::vector<geometry_msgs::msg::PoseStamped> & poses);

  Params p_;
};

}  // namespace nav2_route_planner

#endif  // NAV2_ROUTE_PLANNER__PATH_INTERPOLATOR_HPP_
