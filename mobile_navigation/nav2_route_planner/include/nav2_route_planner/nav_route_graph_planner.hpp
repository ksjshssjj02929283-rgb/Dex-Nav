// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#ifndef NAV2_ROUTE_PLANNER__NAV_ROUTE_GRAPH_PLANNER_HPP_
#define NAV2_ROUTE_PLANNER__NAV_ROUTE_GRAPH_PLANNER_HPP_

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "nav2_route_planner/astar_searcher.hpp"
#include "nav2_route_planner/cost_function.hpp"
#include "nav2_route_planner/costmap_aware_layer.hpp"
#include "nav2_route_planner/graph_augmenter.hpp"
#include "nav2_route_planner/path_interpolator.hpp"
#include "nav2_route_planner/planner_fallback.hpp"
#include "nav2_route_planner/route_graph_loader.hpp"
#include "nav2_route_planner/route_plan_publisher.hpp"
#include "nav2_route_planner/start_goal_projector.hpp"
#include "nav2_route_planner/topo_graph.hpp"

namespace nav2_route_planner
{

/// Nav2 GlobalPlanner plugin that plans on the road-graph served by
/// map_asset_manager. See project doc §4.1.
class NavRouteGraphPlanner : public nav2_core::GlobalPlanner,
  public FallbackProvider
{
public:
  NavRouteGraphPlanner();
  ~NavRouteGraphPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

  /// FallbackProvider: NO_ROUTE_GRAPH iff the most recent createPlan() bailed
  /// out because the map has no route graph; NONE otherwise (including genuine
  /// on-graph routing failures, which do NOT trigger fallback).
  FallbackReason lastFallbackReason() const override {return last_fallback_reason_;}

  /// Public so the table-driven param registry in the .cpp can address
  /// fields through member-pointer lambdas. Treat as implementation
  /// detail — only `configure()` mutates it directly.
  struct Params
  {
    // Data source
    std::string asset_manager_node_name{"map_asset_manager"};
    double data_load_timeout_sec{5.0};

    // Projection — diagnostic-only (the projector always returns the
    // globally-nearest NODE/EDGE); these thresholds only drive INFO logs.
    double projection_node_threshold_m{0.5};
    double projection_edge_threshold_m{1.0};
    double node_snap_threshold_m{0.1};

    // "Same-side shortcut": when avoidance is enabled and start/goal
    // projections sit within this distance, skip graph routing and use
    // Theta\* directly. 0.0 disables independently.
    double shortcut_max_proj_distance_m{0.5};

    // Cost
    CostWeights weights;
    UserPrefs prefs;

    // Interpolation
    int waypoints_per_meter{100};
    int bezier_samples_per_edge{40};

    // Output
    bool publish_route_plan{true};
    bool use_final_approach_orientation{false};

    // RouteGridPlanner costmap avoidance (see project R43).
    CostmapAwareLayer::Params ca;
  };

private:
  Params params_;

  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

  /// createPlan helpers: the plan pipeline split into named,
  /// independently-testable stages.

  /// Project start and goal onto the graph. Emits the projection
  /// diagnostic logs. Returns nullopt when either projection is
  /// INVALID (empty graph). Caller must hold graph_->lockShared().
  std::optional<
    std::pair<StartGoalProjector::Result, StartGoalProjector::Result>>
  projectEndpoints(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);

  /// Same-side shortcut: if start_proj and goal_proj are within
  /// `shortcut_max_proj_distance_m` of each other, Theta*-direct from
  /// start to goal and return that path. Returns nullopt to fall
  /// through to graph routing (threshold disabled, distance too far,
  /// or Theta* failed).
  std::optional<nav_msgs::msg::Path> trySameSideShortcut(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const StartGoalProjector::Result & start_proj,
    const StartGoalProjector::Result & goal_proj);

  /// Theta*-rewrite a placeholder transition segment (start_transition
  /// / goal_alignment). Replaces the segment's samples in-place and
  /// retags its edge_id. Returns false if Theta* fails — caller should
  /// abort and return an empty plan.
  bool rewriteTransitionWithThetaStar(
    InterpolatedSegment & seg, const std::string & expected_id);

  /// Run obstacle detection on `nav_path` only while total route costmap
  /// avoidance is enabled. Splices Theta*-bypass segments into the path
  /// in-place (reverse-order so indices stay valid) and clears stale markers
  /// while disabled.
  void applyObstacleBypass(
    nav_msgs::msg::Path & nav_path, const rclcpp::Time & stamp);

  /// Final-orientation fixup: Theta*-rewritten segments output
  /// identity quaternions, which clobbers the goal heading the
  /// interpolator placed on the last sample. Re-apply either the
  /// goal pose orientation (default) or the path's terminal tangent
  /// when `use_final_approach_orientation` is on.
  void enforceGoalOrientation(
    nav_msgs::msg::Path & nav_path,
    const geometry_msgs::msg::PoseStamped & goal);

  // Components
  std::shared_ptr<TopoGraph> graph_;
  std::shared_ptr<RouteGraphLoader> loader_;
  std::shared_ptr<StartGoalProjector> projector_;
  std::shared_ptr<AStarSearcher> searcher_;
  std::shared_ptr<PathInterpolator> interpolator_;
  std::shared_ptr<RoutePlanPublisher> rp_publisher_;
  std::shared_ptr<CostmapAwareLayer> ca_layer_;

  // Set by createPlan; read by PlannerServer (via FallbackProvider) to decide
  // whether to dispatch the configured fallback planner.
  FallbackReason last_fallback_reason_{FallbackReason::NONE};

  // Debug marker publisher: visualizes each detected obstacle's
  // start_index / end_index on the planned path. Lifecycle-managed.
  rclcpp_lifecycle::LifecyclePublisher<
    visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_markers_pub_;

  // Nav2 standard members
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node_;
  rclcpp::Logger logger_{rclcpp::get_logger("nav2_route_planner")};
  rclcpp::Clock::SharedPtr clock_;
  std::string global_frame_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace nav2_route_planner

#endif  // NAV2_ROUTE_PLANNER__NAV_ROUTE_GRAPH_PLANNER_HPP_
