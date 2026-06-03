// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#ifndef NAV2_ROUTE_PLANNER__COSTMAP_AWARE_LAYER_HPP_
#define NAV2_ROUTE_PLANNER__COSTMAP_AWARE_LAYER_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_theta_star_planner/theta_star.hpp"

#include "nav2_route_planner/start_goal_projector.hpp"
#include "nav2_route_planner/topo_graph.hpp"

namespace nav2_route_planner
{

/// Costmap-aware safety layer on top of the road-graph planner.
///
/// Two layers, separately gated:
///   Layer 1 — start-transition lethal check (mandatory, no on/off):
///     verifies that the straight line from `start_pose` to the road-graph
///     entry point does not cross lethal cost cells, which would mean the
///     planner is silently asking the controller to walk through a wall.
///
///   Layer 2 — costmap-driven Theta* local bypass (opt-in via
///     `costmap_aware_mode` param):
///     scans the produced plan for stretches of lethal cost, then asks
///     nav2_theta_star_planner's ThetaStar to compute a free-space detour
///     between (obstacle - margin_before) and (obstacle + margin_after),
///     splicing the result back into the path.
///
/// See `7. nav2_route_planner 设计方案.md` §4.8 for the design rationale.
class CostmapAwareLayer
{
public:
  struct Params
  {
    // Layer 1 (start/goal transition): no toggle. Theta* either succeeds
    // or fails. If start or goal is in lethal cost, Theta* returns
    // unsafe and the whole plan is rejected — that is the intended
    // contract; no separate threshold needed.

    // Layer 2 (in-graph obstacle bypass, gated)
    bool   costmap_aware_mode{false};
    double obstacle_cost_threshold{220.0};
    double obstacle_margin_before_m{1.0};
    double obstacle_margin_after_m{1.0};
    double bypass_max_replan_length_m{10.0};
    int    theta_star_max_corners{8};
    bool   theta_star_allow_unknown{true};
  };

  CostmapAwareLayer(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

  void configure(const Params & p);
  void cleanup();
  void updateParams(const Params & p);  // safe to call at runtime
  bool isLayer2Enabled() const {return p_.costmap_aware_mode;}

  struct ThetaStarFailureDiagnostic
  {
    enum class Reason
    {
      NONE,
      UNAVAILABLE,
      OUT_OF_BOUNDS,
      UNSAFE_ENDPOINT,
      NO_PATH,
    };

    Reason reason{Reason::NONE};
    bool start_in_bounds{false};
    bool goal_in_bounds{false};
    unsigned int start_mx{0};
    unsigned int start_my{0};
    unsigned int goal_mx{0};
    unsigned int goal_my{0};
    unsigned char start_cost{nav2_costmap_2d::NO_INFORMATION};
    unsigned char goal_cost{nav2_costmap_2d::NO_INFORMATION};
    bool allow_unknown{false};
  };

  /// General-purpose Theta* planner on the live costmap. Used by all
  /// three call sites:
  ///   (a) start → graph entry  (Layer 1, mandatory)
  ///   (b) graph exit → goal     (Layer 1, mandatory)
  ///   (c) in-graph obstacle bypass (Layer 2, gated)
  ///
  /// Returns sampled poses (start->end order, dense at half costmap
  /// resolution) or nullopt on failure. On failure,
  /// describeLastThetaStarFailure() explains whether the cause was
  /// out of bounds, unsafe endpoint cost, missing path connectivity, or
  /// unavailable Theta*/costmap state.
  std::optional<std::vector<geometry_msgs::msg::PoseStamped>>
  planWithThetaStar(
    const geometry_msgs::msg::PoseStamped & seg_start,
    const geometry_msgs::msg::PoseStamped & seg_end);

  std::string describeLastThetaStarFailure() const;

  // ─── Layer 2 (in-graph obstacle detection) ──────────────────────────
  struct ObstacleSegment
  {
    std::size_t start_index{0};   // first pose index inside (or just before) obstacle
    std::size_t end_index{0};     // last pose index inside obstacle (inclusive)
  };

  /// Detect ALL contiguous lethal-cost runs along `plan`. Each run is
  /// expanded outward by `obstacle_margin_before_m` /
  /// `obstacle_margin_after_m` (clipped to plan ends, and clipped so
  /// adjacent runs do not overlap). Scanning stops past
  /// `bypass_max_replan_length_m` cumulative travel.
  ///
  /// This is the single obstacle-detection entry point — it works for
  /// 0/1/N obstacle runs, so callers that previously special-cased
  /// "find the first obstacle" can simply use `result.empty()` for the
  /// 0-obstacle case and `result[0]` for the first one.
  ///
  /// Returned segments are sorted by `start_index` ascending and are
  /// guaranteed non-overlapping.
  std::vector<ObstacleSegment> detectAllObstacles(
    const std::vector<geometry_msgs::msg::PoseStamped> & plan) const;

  /// Read the cost at world coordinates `(x, y)`. Returns 255 (lethal)
  /// if the point is outside the costmap bounds.
  unsigned char getCostAt(double x, double y) const;

  /// Shift an ON_EDGE projection past any lethal-cost run that contains
  /// it, sliding along the edge toward whichever endpoint is closer to
  /// `toward_pose`.
  ///
  /// Returns:
  ///   • the original projection if it isn't inside any obstacle on the edge
  ///   • a Result with updated projected_xy / edge_sample_index if it was
  ///   • nullopt if no clear point exists on this edge in the slide
  ///     direction (the edge is blocked all the way to the chosen endpoint)
  ///
  /// Reuses `detectAllObstacles` (margins already accounted for). Caller
  /// must hold graph.lockShared().
  std::optional<StartGoalProjector::Result> shiftProjectionPastObstacle(
    const StartGoalProjector::Result & in,
    const geometry_msgs::msg::PoseStamped & toward_pose,
    const TopoGraph & graph,
    int num_bezier_samples,
    int waypoints_per_meter) const;

private:

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  rclcpp::Logger logger_{rclcpp::get_logger("CostmapAwareLayer")};
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};

  // Lazily constructed in configure() so that Theta* doesn't allocate
  // unless layer 2 is at least once requested.
  std::unique_ptr<theta_star::ThetaStar> theta_star_;

  Params p_;
  std::optional<ThetaStarFailureDiagnostic> last_theta_star_failure_;
};

}  // namespace nav2_route_planner

#endif  // NAV2_ROUTE_PLANNER__COSTMAP_AWARE_LAYER_HPP_
