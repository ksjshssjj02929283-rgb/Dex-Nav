// ----------------------------------------------------------------------------
// Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
//
// All rights reserved.
// ----------------------------------------------------------------------------
#ifndef NAV2_ROUTE_PLANNER__ROUTE_GRAPH_LOADER_HPP_
#define NAV2_ROUTE_PLANNER__ROUTE_GRAPH_LOADER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav_interfaces/srv/get_route_graph.hpp"

#include "nav2_route_planner/topo_graph.hpp"

namespace nav2_route_planner
{

/// Pulls RouteGraph (junctions + edges + nav_points) from `map_asset_manager`
/// over a single ROS2 service (GetRouteGraph) and rebuilds the in-memory
/// TopoGraph.
///
/// IMPORTANT — fully asynchronous design:
///   `planner_server` uses a SingleThreadedExecutor. Any synchronous wait on
///   a service future from inside a lifecycle / action / timer callback
///   deadlocks the executor: the same thread that would dispatch the
///   response is the one blocked in wait_for(). Therefore loading runs
///   entirely in callbacks driven by a 1 Hz background poll timer:
///     1. timer ticks → if isLoaded(), cancel timer; else if service is
///        ready and no request is in flight, fire async_send_request with
///        a result callback.
///     2. response callback → rebuildFrom() the TopoGraph (or retry on
///        failure).
///   No blocking, no shared-state locking (timer + response callbacks both
///   run on the executor's main thread; the TopoGraph guards itself).
class RouteGraphLoader
{
public:
  RouteGraphLoader(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
    std::string asset_node_name,
    std::shared_ptr<TopoGraph> graph);

  /// Start the 1 Hz background load timer. Idempotent.
  void startBackgroundLoad();

  /// Cancel and release the background load timer.
  void stopBackgroundLoad();

  /// True once the graph has been populated at least once.
  bool isLoaded() const;

  /// Service name actually used (for diagnostics / logs).
  std::string getRouteGraphServiceName() const {return route_graph_srv_name_;}

private:
  using GetRouteGraph = nav_interfaces::srv::GetRouteGraph;

  void onTimer();
  void onGraphResponse(rclcpp::Client<GetRouteGraph>::SharedFuture future);
  void rebuild(const GetRouteGraph::Response & resp);

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::string asset_node_name_;
  std::shared_ptr<TopoGraph> graph_;

  std::string route_graph_srv_name_;
  rclcpp::Client<GetRouteGraph>::SharedPtr route_graph_client_;

  rclcpp::TimerBase::SharedPtr poll_timer_;
  bool request_in_flight_{false};
};

}  // namespace nav2_route_planner

#endif  // NAV2_ROUTE_PLANNER__ROUTE_GRAPH_LOADER_HPP_
