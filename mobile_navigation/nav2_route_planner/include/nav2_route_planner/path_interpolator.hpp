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
    const rclcpp::Time & stamp) const;

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

  /// Build a complete RoutePlan from the A* search result over a graph
  /// already augmented with virtual nodes for the start/goal projections.
  ///
  /// Because the entry/exit nodes ARE the projection points (virtual nodes
  /// the augmenter inserted), interpolation is uniform — it just walks the
  /// A* node/edge sequence. The output segments are:
  ///   • `__start_transition__` — free-space from `start` to the A* entry
  ///     node. ALWAYS emitted; the caller Theta\*-rewrites it so the robot
  ///     reaches the road safely.
  ///   • <per-edge segments>    — one per edge in `search_result.edge_ids`,
  ///     sampled along the (sub-)edge geometry. Sub-edge ids carry the
  ///     parent road edge_id (the synthetic `#n` suffix is stripped).
  ///   • `__goal_alignment__`   — free-space from the A* exit node to the
  ///     `goal` pose. ALWAYS emitted; the caller Theta\*-rewrites it.
  ///
  /// Caller must hold graph.lockShared().
  InterpolatedRoutePlan interpolate(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const AStarSearcher::Result & search_result,
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

  /// Shared tail of every interpolation path: compute per-segment
  /// orientations, force the final sample's heading to the goal pose
  /// (unless use_final_approach_orientation keeps the path tangent), and
  /// stamp a deterministic plan_id from the start/goal coordinates.
  void finalizePlan(
    InterpolatedRoutePlan & plan,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

  Params p_;
};

}  // namespace nav2_route_planner

#endif  // NAV2_ROUTE_PLANNER__PATH_INTERPOLATOR_HPP_
