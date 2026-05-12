// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#ifndef NAV2_ROUTE_PLANNER__COSTMAP_AWARE_LAYER_HPP_
#define NAV2_ROUTE_PLANNER__COSTMAP_AWARE_LAYER_HPP_

#include <memory>
#include <optional>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_theta_star_planner/theta_star.hpp"

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

  /// General-purpose Theta* planner on the live costmap. Used by all
  /// three call sites:
  ///   (a) start → graph entry  (Layer 1, mandatory)
  ///   (b) graph exit → goal     (Layer 1, mandatory)
  ///   (c) in-graph obstacle bypass (Layer 2, gated)
  ///
  /// Returns sampled poses (start->end order, dense at half costmap
  /// resolution) or nullopt on failure (out of bounds / endpoint in
  /// lethal / no path). Failure is the caller's signal that the
  /// transition is unreachable.
  std::optional<std::vector<geometry_msgs::msg::PoseStamped>>
  planWithThetaStar(
    const geometry_msgs::msg::PoseStamped & seg_start,
    const geometry_msgs::msg::PoseStamped & seg_end);

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

private:
  /// Internal helper: read `(x, y)` into a cost value. Returns 255
  /// (lethal) if the point is outside the costmap.
  unsigned char getCostAt(double x, double y) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  rclcpp::Logger logger_{rclcpp::get_logger("CostmapAwareLayer")};
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};

  // Lazily constructed in configure() so that Theta* doesn't allocate
  // unless layer 2 is at least once requested.
  std::unique_ptr<theta_star::ThetaStar> theta_star_;

  Params p_;
};

}  // namespace nav2_route_planner

#endif  // NAV2_ROUTE_PLANNER__COSTMAP_AWARE_LAYER_HPP_
