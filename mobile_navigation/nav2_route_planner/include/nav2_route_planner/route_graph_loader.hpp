// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#ifndef NAV2_ROUTE_PLANNER__ROUTE_GRAPH_LOADER_HPP_
#define NAV2_ROUTE_PLANNER__ROUTE_GRAPH_LOADER_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav_interfaces/srv/get_route_graph.hpp"
#include "nav_interfaces/srv/get_all_nav_points.hpp"

#include "nav2_route_planner/topo_graph.hpp"

namespace nav2_route_planner
{

/// Pulls RouteGraph (junctions + edges) and NavPoints from `map_asset_manager`
/// over standard ROS2 services and rebuilds the in-memory TopoGraph.
///
/// One-shot synchronous load is provided for startup; an async load is
/// provided for activate(). Refresh is currently triggered manually by the
/// owning plugin (see `loadSync`); a topic-based incremental refresh is
/// reserved for a follow-up commit (see project doc §4.3).
class RouteGraphLoader
{
public:
  RouteGraphLoader(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
    std::string asset_node_name,
    std::shared_ptr<TopoGraph> graph);

  /// Synchronously pull a fresh snapshot and call graph_->rebuildFrom().
  /// Returns false on any failure (services unavailable, RPC failure,
  /// timeout). Safe to call repeatedly.
  bool loadSync(double timeout_sec);

  /// Service names actually used (for diagnostics / logs).
  std::string getRouteGraphServiceName() const { return route_graph_srv_name_; }
  std::string getNavPointsServiceName() const { return nav_points_srv_name_; }

private:
  using GetRouteGraph = nav_interfaces::srv::GetRouteGraph;
  using GetAllNavPoints = nav_interfaces::srv::GetAllNavPoints;

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::string asset_node_name_;
  std::shared_ptr<TopoGraph> graph_;

  std::string route_graph_srv_name_;
  std::string nav_points_srv_name_;

  rclcpp::Client<GetRouteGraph>::SharedPtr route_graph_client_;
  rclcpp::Client<GetAllNavPoints>::SharedPtr nav_points_client_;
};

}  // namespace nav2_route_planner

#endif  // NAV2_ROUTE_PLANNER__ROUTE_GRAPH_LOADER_HPP_
