// Copyright 2026 DEXE Robot Engineer
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "robot_storytelling/route_attribute_teller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>

#include "robot_storytelling/teller_registry.hpp"

namespace robot_storytelling
{

geometry_msgs::msg::Point evaluateBezier(
  const std::vector<geometry_msgs::msg::Point> & control_points, double t)
{
  std::vector<geometry_msgs::msg::Point> pts = control_points;
  while (pts.size() > 1) {
    std::vector<geometry_msgs::msg::Point> next;
    next.reserve(pts.size() - 1);
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
      geometry_msgs::msg::Point p;
      p.x = pts[i].x + (pts[i + 1].x - pts[i].x) * t;
      p.y = pts[i].y + (pts[i + 1].y - pts[i].y) * t;
      next.push_back(p);
    }
    pts = std::move(next);
  }
  return pts.front();
}

std::vector<std::pair<double, double>> sampleEdge(
  const nav_interfaces::msg::RouteEdge & edge, int num_samples)
{
  std::vector<std::pair<double, double>> polyline;
  if (edge.control_points.size() < 2U || num_samples < 1) {
    return polyline;
  }
  polyline.reserve(static_cast<std::size_t>(num_samples) + 1U);
  for (int i = 0; i <= num_samples; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(num_samples);
    const auto p = evaluateBezier(edge.control_points, t);
    polyline.emplace_back(p.x, p.y);
  }
  return polyline;
}

double pointToPolylineDistance(
  double px, double py,
  const std::vector<std::pair<double, double>> & polyline)
{
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < polyline.size(); ++i) {
    const auto & [x0, y0] = polyline[i];
    const auto & [x1, y1] = polyline[i + 1];
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double len2 = dx * dx + dy * dy;
    double t = 0.0;
    if (len2 > 1e-12) {
      t = ((px - x0) * dx + (py - y0) * dy) / len2;
      t = std::clamp(t, 0.0, 1.0);
    }
    const double cx = x0 + t * dx;
    const double cy = y0 + t * dy;
    const double dist = std::hypot(px - cx, py - cy);
    best = std::min(best, dist);
  }
  return best;
}

std::optional<std::size_t> findNearestEdge(
  double rx, double ry,
  const std::vector<nav_interfaces::msg::RouteEdge> & edges,
  double distance_threshold,
  int num_samples_per_edge)
{
  std::optional<std::size_t> best_index;
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < edges.size(); ++i) {
    const auto polyline = sampleEdge(edges[i], num_samples_per_edge);
    if (polyline.size() < 2U) {
      continue;
    }
    const double dist = pointToPolylineDistance(rx, ry, polyline);
    if (dist < best_distance) {
      best_distance = dist;
      best_index = i;
    }
  }
  if (best_index && best_distance <= distance_threshold) {
    return best_index;
  }
  return std::nullopt;
}

std::optional<nav_interfaces::msg::Story> RouteAttributeTeller::updateWithEdges(
  double rx, double ry,
  const std::vector<nav_interfaces::msg::RouteEdge> & edges)
{
  if (edges.empty()) {
    current_edge_id_.clear();
    candidate_edge_id_.clear();
    candidate_streak_ = 0;
    return std::nullopt;
  }

  const auto nearest = findNearestEdge(
    rx, ry, edges, match_distance_threshold_, num_samples_per_edge_);
  if (!nearest) {
    // 不在任何边的匹配阈值内（"离开路网"，如自由导航/避障绕行远离既有边）。
    current_edge_id_.clear();
    candidate_edge_id_.clear();
    candidate_streak_ = 0;
    return std::nullopt;
  }

  const auto & edge = edges[*nearest];

  if (current_edge_id_.empty()) {
    // 冷启动 / 图刚重建：没有需要保护的既有状态，直接采信最近边。
    current_edge_id_ = edge.id;
    candidate_edge_id_.clear();
    candidate_streak_ = 0;
  } else if (edge.id == current_edge_id_) {
    candidate_edge_id_.clear();
    candidate_streak_ = 0;
  } else if (edge.id == candidate_edge_id_) {
    ++candidate_streak_;
    if (candidate_streak_ >= switch_hysteresis_frames_) {
      current_edge_id_ = edge.id;
      candidate_edge_id_.clear();
      candidate_streak_ = 0;
    }
  } else {
    candidate_edge_id_ = edge.id;
    candidate_streak_ = 1;
  }

  if (current_edge_id_.empty()) {
    return std::nullopt;
  }

  const nav_interfaces::msg::RouteEdge * current = nullptr;
  for (const auto & e : edges) {
    if (e.id == current_edge_id_) {
      current = &e;
      break;
    }
  }
  if (!current) {
    // 已确认边在图重建后消失（如地图切换）：清空状态，下一帧重新冷启动。
    current_edge_id_.clear();
    candidate_edge_id_.clear();
    candidate_streak_ = 0;
    return std::nullopt;
  }

  nav_interfaces::msg::Story s;
  s.event_type = nav_interfaces::msg::Story::CURRENT_EDGE;
  s.source_id = current->id;
  s.confidence = 1.0f;
  s.tags = {
    "max_speed=" + std::to_string(current->max_speed),
    "obs_policy=" + current->obs_policy,
  };
  return s;
}

void RouteAttributeTeller::Init(rclcpp::Node * node)
{
  node_ = node;
  match_distance_threshold_ = node->declare_parameter("current_edge_match_distance", 0.5);
  switch_hysteresis_frames_ =
    node->declare_parameter("current_edge_switch_hysteresis_frames", 3);
  num_samples_per_edge_ = node->declare_parameter("current_edge_samples_per_edge", 20);
  const std::string asset_node_name =
    node->declare_parameter<std::string>("route_graph_asset_node", "map_asset_manager");
  const double poll_period_sec =
    node->declare_parameter("route_graph_poll_period_sec", 5.0);
  route_graph_service_name_ = "/" + asset_node_name + "/get_route_graph";

  route_graph_client_ =
    node->create_client<nav_interfaces::srv::GetRouteGraph>(route_graph_service_name_);

  poll_timer_ = node->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(poll_period_sec)),
    [this]() {onTimer();});

  RCLCPP_INFO(
    node_->get_logger(),
    "RouteAttributeTeller: polling %s every %.1fs for RouteGraph "
    "(match_distance=%.2fm, hysteresis=%d frames)",
    route_graph_service_name_.c_str(), poll_period_sec,
    match_distance_threshold_, switch_hysteresis_frames_);
}

void RouteAttributeTeller::onTimer()
{
  if (request_in_flight_ || !route_graph_client_->service_is_ready()) {
    return;
  }
  request_in_flight_ = true;
  auto req = std::make_shared<nav_interfaces::srv::GetRouteGraph::Request>();
  route_graph_client_->async_send_request(
    req,
    [this](rclcpp::Client<nav_interfaces::srv::GetRouteGraph>::SharedFuture future) {
      onGraphResponse(future);
    });
}

void RouteAttributeTeller::onGraphResponse(
  rclcpp::Client<nav_interfaces::srv::GetRouteGraph>::SharedFuture future)
{
  request_in_flight_ = false;
  const auto resp = future.get();
  if (!resp->success) {
    RCLCPP_WARN(
      node_->get_logger(), "RouteAttributeTeller: GetRouteGraph reported failure, will retry");
    return;
  }
  edges_ = resp->edges;
  if (!graph_loaded_once_) {
    graph_loaded_once_ = true;
    RCLCPP_INFO(
      node_->get_logger(), "RouteAttributeTeller: RouteGraph loaded, %zu edges", edges_.size());
  }
}

void RouteAttributeTeller::Update(
  const TellerContext & ctx,
  std::vector<nav_interfaces::msg::Story> * stories)
{
  const auto story = updateWithEdges(ctx.robot_x, ctx.robot_y, edges_);
  if (story) {
    stories->push_back(*story);
  }
}

}  // namespace robot_storytelling

ROBOT_STORYTELLER_REGISTER(RouteAttributeTeller)
