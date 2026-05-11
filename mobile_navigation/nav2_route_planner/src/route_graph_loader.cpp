// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#include "nav2_route_planner/route_graph_loader.hpp"

#include <chrono>
#include <future>
#include <utility>
#include <vector>

namespace nav2_route_planner
{

using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::nanoseconds;

RouteGraphLoader::RouteGraphLoader(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
  std::string asset_node_name,
  std::shared_ptr<TopoGraph> graph)
: parent_(std::move(parent)),
  asset_node_name_(std::move(asset_node_name)),
  graph_(std::move(graph))
{
  auto node = parent_.lock();
  if (!node) {
    return;
  }
  // map_asset_manager exposes services under its node name with `~/` prefix,
  // e.g. /map_asset_manager/get_route_graph.
  route_graph_srv_name_ = "/" + asset_node_name_ + "/get_route_graph";
  nav_points_srv_name_ = "/" + asset_node_name_ + "/get_all_nav_points";
  route_graph_client_ = node->create_client<GetRouteGraph>(route_graph_srv_name_);
  nav_points_client_ = node->create_client<GetAllNavPoints>(nav_points_srv_name_);
}

bool RouteGraphLoader::loadSync(double timeout_sec)
{
  auto node = parent_.lock();
  if (!node) {return false;}
  auto logger = node->get_logger();

  auto deadline = std::chrono::steady_clock::now() +
    duration_cast<nanoseconds>(duration<double>(timeout_sec));

  // 1) Wait for both services.
  auto waitFor = [&deadline, &logger](auto & client, const std::string & name) {
    while (std::chrono::steady_clock::now() < deadline) {
      if (client->wait_for_service(std::chrono::milliseconds(200))) {
        return true;
      }
      RCLCPP_INFO_THROTTLE(logger, *rclcpp::Clock::make_shared(), 1000,
        "Waiting for service %s", name.c_str());
    }
    return false;
  };
  if (!waitFor(route_graph_client_, route_graph_srv_name_)) {
    RCLCPP_ERROR(logger, "Timed out waiting for %s", route_graph_srv_name_.c_str());
    return false;
  }
  if (!waitFor(nav_points_client_, nav_points_srv_name_)) {
    RCLCPP_ERROR(logger, "Timed out waiting for %s", nav_points_srv_name_.c_str());
    return false;
  }

  // 2) Async send both, then wait for both with the remaining budget.
  auto graph_future = route_graph_client_->async_send_request(
    std::make_shared<GetRouteGraph::Request>());
  auto points_future = nav_points_client_->async_send_request(
    std::make_shared<GetAllNavPoints::Request>());

  // Note: we rely on the planner_server's executor to be spinning.
  // The future blocks until the response arrives or our deadline elapses.
  auto wait_one = [&deadline](auto & fut) -> bool {
      auto remaining = deadline - std::chrono::steady_clock::now();
      if (remaining.count() <= 0) {return false;}
      return fut.wait_for(remaining) == std::future_status::ready;
    };

  if (!wait_one(graph_future)) {
    RCLCPP_ERROR(logger, "Timed out waiting for GetRouteGraph response");
    return false;
  }
  if (!wait_one(points_future)) {
    RCLCPP_ERROR(logger, "Timed out waiting for GetAllNavPoints response");
    return false;
  }

  auto graph_resp = graph_future.get();
  auto points_resp = points_future.get();
  if (!graph_resp->success) {
    RCLCPP_ERROR(logger, "GetRouteGraph reported failure");
    return false;
  }
  if (!points_resp->success) {
    RCLCPP_ERROR(logger, "GetAllNavPoints reported failure");
    return false;
  }

  // 3) Convert ROS messages → in-memory TopoGraph.
  std::vector<TopoNode> nodes;
  nodes.reserve(points_resp->points.size() + graph_resp->junctions.size());
  for (const auto & np : points_resp->points) {
    TopoNode n;
    n.id = np.name;
    n.kind = NodeKind::NAV_POINT;
    n.x = np.x;
    n.y = np.y;
    n.yaw = np.theta;
    n.semantic_label = np.name;  // placeholder until SemanticLayer lands
    nodes.push_back(std::move(n));
  }
  for (const auto & jn : graph_resp->junctions) {
    TopoNode n;
    n.id = jn.id;
    n.kind = NodeKind::JUNCTION;
    n.x = jn.x;
    n.y = jn.y;
    n.yaw = jn.yaw;
    nodes.push_back(std::move(n));
  }

  std::vector<TopoEdge> edges;
  edges.reserve(graph_resp->edges.size());
  for (const auto & em : graph_resp->edges) {
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
  RCLCPP_INFO(logger,
    "RouteGraph loaded: %zu nodes (%zu nav + %zu junctions), %zu edges",
    graph_->nodeCount(),
    points_resp->points.size(), graph_resp->junctions.size(),
    graph_->edgeCount());
  return true;
}

}  // namespace nav2_route_planner
