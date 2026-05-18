// ----------------------------------------------------------------------------
// Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
//
// All rights reserved.
// ----------------------------------------------------------------------------
#include "nav2_route_planner/route_graph_loader.hpp"

#include <chrono>
#include <memory>
#include <utility>
#include <vector>

namespace nav2_route_planner
{

RouteGraphLoader::RouteGraphLoader(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
  std::string asset_node_name,
  std::shared_ptr<TopoGraph> graph)
: parent_(std::move(parent)),
  asset_node_name_(std::move(asset_node_name)),
  graph_(std::move(graph))
{
  auto node = parent_.lock();
  if (!node) {return;}
  // map_asset_manager exposes services under its node name, e.g.
  // /map_asset_manager/get_route_graph. The response carries junctions,
  // edges AND nav_points (see GetRouteGraph.srv) — single round-trip.
  route_graph_srv_name_ = "/" + asset_node_name_ + "/get_route_graph";

  // Dedicated callback group for syncLoad() — `automatically_add_to_
  // executor_with_node = false` keeps it OFF the planner's main
  // SingleThreadedExecutor; we spin it in our own thread instead so a
  // blocking wait_for() on the service future cannot deadlock the
  // planner.
  sync_cb_group_ = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    /*automatically_add_to_executor_with_node=*/false);
  sync_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  sync_executor_->add_callback_group(
    sync_cb_group_, node->get_node_base_interface());
  sync_spin_thread_ = std::thread([this]() {sync_executor_->spin();});

  // Both the legacy background-poll path AND the new syncLoad() use
  // the same client; binding it to `sync_cb_group_` means the timer
  // (still on the main executor) fires `async_send_request` but the
  // response future is serviced by our private thread, which is also
  // safe.
  route_graph_client_ = node->create_client<GetRouteGraph>(
    route_graph_srv_name_,
    rmw_qos_profile_services_default,
    sync_cb_group_);
}

RouteGraphLoader::~RouteGraphLoader()
{
  if (sync_executor_) {sync_executor_->cancel();}
  if (sync_spin_thread_.joinable()) {sync_spin_thread_.join();}
}

bool RouteGraphLoader::isLoaded() const
{
  return graph_ && graph_->isReady();
}

void RouteGraphLoader::startBackgroundLoad()
{
  auto node = parent_.lock();
  if (!node) {return;}
  if (isLoaded()) {return;}
  if (poll_timer_) {return;}

  RCLCPP_INFO(node->get_logger(),
    "RouteGraphLoader: background load started (service: %s)",
    route_graph_srv_name_.c_str());

  // 0.5 Hz background poll. Timer fires on the executor's main thread; the
  // service response callback fires on the same thread, so no shared-state
  // locking is needed inside the loader.
  poll_timer_ = node->create_wall_timer(
    std::chrono::seconds(2),
    [this]() {this->onTimer();});
}

void RouteGraphLoader::stopBackgroundLoad()
{
  if (poll_timer_) {
    poll_timer_->cancel();
    poll_timer_.reset();
  }
}

void RouteGraphLoader::onTimer()
{
  auto node = parent_.lock();
  if (!node) {return;}
  auto logger = node->get_logger();

  if (isLoaded()) {
    stopBackgroundLoad();
    return;
  }

  if (request_in_flight_) {return;}

  // Non-blocking readiness check — inspects the local discovery cache only.
  if (!route_graph_client_->service_is_ready()) {
    RCLCPP_INFO_THROTTLE(logger, *node->get_clock(), 5000,
      "RouteGraphLoader: waiting for %s",
      route_graph_srv_name_.c_str());
    return;
  }

  // Result-callback variant — executor delivers the response on its own
  // thread; we never block.
  request_in_flight_ = true;
  auto req = std::make_shared<GetRouteGraph::Request>();
  route_graph_client_->async_send_request(req,
    [this](rclcpp::Client<GetRouteGraph>::SharedFuture future) {
      this->onGraphResponse(future);
    });
}

void RouteGraphLoader::onGraphResponse(
  rclcpp::Client<GetRouteGraph>::SharedFuture future)
{
  request_in_flight_ = false;

  auto node = parent_.lock();
  auto logger = node ? node->get_logger()
    : rclcpp::get_logger("RouteGraphLoader");

  auto resp = future.get();
  if (!resp || !resp->success) {
    RCLCPP_WARN(logger, "GetRouteGraph reported failure — will retry");
    return;
  }
  rebuild(*resp);
}

RouteGraphLoader::LoadResult RouteGraphLoader::syncLoad(
  std::chrono::milliseconds timeout)
{
  LoadResult r;
  auto node = parent_.lock();
  if (!node) {return r;}
  auto logger = node->get_logger();

  const auto t0 = std::chrono::steady_clock::now();

  // Non-blocking readiness check on the local discovery cache.
  if (!route_graph_client_->service_is_ready()) {
    r.duration_total = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - t0);
    RCLCPP_WARN(logger,
      "syncLoad: service '%s' not ready — graph not refreshed",
      route_graph_srv_name_.c_str());
    return r;
  }
  r.service_ready = true;

  // Send request; future is serviced by `sync_executor_` on its
  // own thread, so wait_for() here cannot deadlock.
  auto req = std::make_shared<GetRouteGraph::Request>();
  auto future = route_graph_client_->async_send_request(req);
  const auto status = future.wait_for(timeout);

  if (status != std::future_status::ready) {
    r.timed_out = true;
    r.duration_total = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - t0);
    RCLCPP_WARN(logger,
      "syncLoad: GetRouteGraph timed out after %ldms — graph not refreshed",
      static_cast<long>(timeout.count()));
    return r;
  }

  auto resp = future.get();
  if (!resp || !resp->success) {
    r.service_reported_failure = true;
    r.duration_total = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - t0);
    RCLCPP_WARN(logger,
      "syncLoad: GetRouteGraph reported failure — graph not refreshed");
    return r;
  }

  const auto t_rebuild_start = std::chrono::steady_clock::now();
  rebuild(*resp);
  const auto t1 = std::chrono::steady_clock::now();

  r.success = true;
  r.duration_total = std::chrono::duration_cast<std::chrono::milliseconds>(
    t1 - t0);
  r.duration_rebuild = std::chrono::duration_cast<std::chrono::milliseconds>(
    t1 - t_rebuild_start);
  r.node_count = graph_->nodeCount();
  r.edge_count = graph_->edgeCount();
  return r;
}

void RouteGraphLoader::rebuild(const GetRouteGraph::Response & resp)
{
  std::vector<TopoNode> nodes;
  nodes.reserve(resp.nav_points.size() + resp.junctions.size());

  for (const auto & np : resp.nav_points) {
    TopoNode n;
    n.id = np.name;
    n.kind = NodeKind::NAV_POINT;
    n.x = np.x;
    n.y = np.y;
    n.yaw = np.theta;
    n.semantic_label = np.name;  // placeholder until SemanticLayer lands
    nodes.push_back(std::move(n));
  }
  for (const auto & jn : resp.junctions) {
    TopoNode n;
    n.id = jn.id;
    n.kind = NodeKind::JUNCTION;
    n.x = jn.x;
    n.y = jn.y;
    n.yaw = jn.yaw;
    nodes.push_back(std::move(n));
  }

  std::vector<TopoEdge> edges;
  edges.reserve(resp.edges.size());
  for (const auto & em : resp.edges) {
    TopoEdge e;
    e.id = em.id;
    e.from_id = em.from_ep.id;
    e.to_id = em.to_ep.id;
    e.geometry = (em.edge_type == "BEZIER") ? EdgeGeometry::BEZIER : EdgeGeometry::LINE;
    e.bidirectional = em.bidirectional;
    e.max_speed = em.max_speed;
    e.xy_tolerance = em.xy_tolerance;
    e.th_tolerance = em.th_tolerance;
    e.lane_width = em.lane_width;
    e.obs_policy = em.obs_policy;
    for (const auto & p : em.control_points) {
      e.control_points.emplace_back(p.x, p.y);
    }
    edges.push_back(std::move(e));
  }

  graph_->rebuildFrom(std::move(nodes), std::move(edges));

  if (auto node = parent_.lock()) {
    RCLCPP_INFO(node->get_logger(),
      "RouteGraph loaded: %zu nodes (%zu nav + %zu junctions), %zu edges",
      graph_->nodeCount(), resp.nav_points.size(), resp.junctions.size(),
      graph_->edgeCount());
  }
}

}  // namespace nav2_route_planner
