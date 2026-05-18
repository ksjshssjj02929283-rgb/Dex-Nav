// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#ifndef NAV2_ROUTE_PLANNER__NAV_ROUTE_GRAPH_PLANNER_HPP_
#define NAV2_ROUTE_PLANNER__NAV_ROUTE_GRAPH_PLANNER_HPP_

#include <memory>
#include <string>
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
#include "nav2_route_planner/path_interpolator.hpp"
#include "nav2_route_planner/route_graph_loader.hpp"
#include "nav2_route_planner/route_plan_publisher.hpp"
#include "nav2_route_planner/start_goal_projector.hpp"
#include "nav2_route_planner/topo_graph.hpp"

namespace nav2_route_planner
{

/// Nav2 GlobalPlanner plugin that plans on the road-graph served by
/// map_asset_manager. See project doc §4.1.
class NavRouteGraphPlanner : public nav2_core::GlobalPlanner
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

private:
  struct Params
  {
    // Data source
    std::string asset_manager_node_name{"map_asset_manager"};
    double data_load_timeout_sec{5.0};

    // Projection. With the project-19 simplification (DEC-004), the
    // projector ALWAYS returns the globally nearest NODE or EDGE
    // projection — the thresholds below are kept as diagnostic logging
    // breakpoints only (we INFO-log when a start/goal pose is farther
    // than these from the graph, since that's interesting to the
    // operator). The planner then Theta*-bridges (pose ↔ projected_xy)
    // free-space and traverses the graph's own geometry in between.
    double projection_node_threshold_m{0.5};
    double projection_edge_threshold_m{1.0};

    // Project 19 / DEC-005 — "same-side shortcut". When the start and
    // goal both project onto the graph within `shortcut_max_proj_distance_m`
    // of EACH OTHER (i.e. the robot and the destination sit on the
    // same side of the road network with essentially the same entry
    // point), skip the graph routing entirely and Theta\* directly
    // from start to goal. Avoids the "U-turn / drive past it" plans
    // where graph routing forces a detour through a remote NavPoint.
    // 0.0 disables the shortcut.
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

    // Costmap-aware safety (see [[7. nav2_route_planner 设计方案]] §4.8)
    CostmapAwareLayer::Params ca;
  } params_;

  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

  // Components
  std::shared_ptr<TopoGraph> graph_;
  std::shared_ptr<RouteGraphLoader> loader_;
  std::shared_ptr<StartGoalProjector> projector_;
  std::shared_ptr<AStarSearcher> searcher_;
  std::shared_ptr<PathInterpolator> interpolator_;
  std::shared_ptr<RoutePlanPublisher> rp_publisher_;
  std::shared_ptr<CostmapAwareLayer> ca_layer_;

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
