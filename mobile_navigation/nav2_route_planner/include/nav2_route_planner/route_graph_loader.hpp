// ----------------------------------------------------------------------------
// Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
//
// All rights reserved.
// ----------------------------------------------------------------------------
#ifndef NAV2_ROUTE_PLANNER__ROUTE_GRAPH_LOADER_HPP_
#define NAV2_ROUTE_PLANNER__ROUTE_GRAPH_LOADER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav_interfaces/srv/get_assets.hpp"

#include "nav2_route_planner/topo_graph.hpp"

namespace nav2_route_planner
{

/// Pulls the route graph (junctions + edges + the nav points they reference)
/// from `map_asset_manager` over a single `get_assets` call (kinds = points |
/// junctions | edges, current map) and rebuilds the in-memory TopoGraph.
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

  ~RouteGraphLoader();

  /// Start the 1 Hz background load timer. Idempotent.
  void startBackgroundLoad();

  /// Cancel and release the background load timer.
  void stopBackgroundLoad();

  /// True once the graph has been populated at least once.
  bool isLoaded() const;

  /// Service name actually used (for diagnostics / logs).
  std::string getRouteGraphServiceName() const {return route_graph_srv_name_;}

  /// Synchronous reload — fetch the RouteGraph from map_asset_manager
  /// and rebuild the in-memory TopoGraph, in one call. Safe to call
  /// from `createPlan` despite that callback running on the planner's
  /// SingleThreadedExecutor: the service client lives in its own
  /// callback group spun by a dedicated thread, so wait_for() on the
  /// future cannot deadlock the planner executor.
  ///
  /// Returns timing + count info; on failure / timeout the graph is
  /// left unchanged (previous `rebuildFrom` is still valid).
  struct LoadResult
  {
    bool success{false};
    bool service_ready{false};
    bool service_reported_failure{false};
    bool timed_out{false};
    std::chrono::milliseconds duration_total{0};   ///< full call latency
    std::chrono::milliseconds duration_rebuild{0}; ///< rebuild step only
    std::size_t node_count{0};
    std::size_t edge_count{0};
  };
  LoadResult syncLoad(std::chrono::milliseconds timeout = std::chrono::seconds(2));

private:
  using GetRouteGraph = nav_interfaces::srv::GetAssets;

  void onTimer();
  void onGraphResponse(rclcpp::Client<GetRouteGraph>::SharedFuture future);
  std::shared_ptr<GetRouteGraph::Request> makeGraphRequest();
  void rebuild(const GetRouteGraph::Response & resp);

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::string asset_node_name_;
  std::shared_ptr<TopoGraph> graph_;

  std::string route_graph_srv_name_;
  rclcpp::Client<GetRouteGraph>::SharedPtr route_graph_client_;

  /// Dedicated callback group + executor + thread so syncLoad() can
  /// wait on the service response without deadlocking the planner's
  /// own (SingleThreaded) executor.
  rclcpp::CallbackGroup::SharedPtr sync_cb_group_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> sync_executor_;
  std::thread sync_spin_thread_;

  rclcpp::TimerBase::SharedPtr poll_timer_;
  bool request_in_flight_{false};
};

}  // namespace nav2_route_planner

#endif  // NAV2_ROUTE_PLANNER__ROUTE_GRAPH_LOADER_HPP_
