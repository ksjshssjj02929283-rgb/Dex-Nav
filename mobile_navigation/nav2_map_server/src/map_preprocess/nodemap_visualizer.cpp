// Copyright (c) DexForce
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_map_server/node_map_io.hpp"
#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{

using nav2_map_server::AdjacencyMode;
using nav2_map_server::NodeMap;

visualization_msgs::msg::Marker makeNodeSpheres(
  const NodeMap & layer,
  const std::string & frame_id,
  const std::string & ns,
  int id,
  float r, float g, float b,
  double scale)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = rclcpp::Time(0);  // RViz: use latest TF (sim / wall clock safe)
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x = scale;
  m.scale.y = scale;
  m.scale.z = scale;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = 0.95f;
  m.frame_locked = true;

  for (const auto & node : layer.nodes) {
    geometry_msgs::msg::Point p;
    p.x = node.x;
    p.y = node.y;
    p.z = 0.25;
    m.points.push_back(p);
  }
  return m;
}

void appendGrid8Edges(
  const NodeMap & layer,
  visualization_msgs::msg::Marker & edges)
{
  std::unordered_map<int, const nav2_map_server::NodeMapNode *> grid_to_node;
  for (const auto & node : layer.nodes) {
    grid_to_node[node.gy * layer.width + node.gx] = &node;
  }

  static const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  std::unordered_set<uint64_t> drawn;
  drawn.reserve(layer.nodes.size() * 4);

  for (const auto & node : layer.nodes) {
    for (int k = 0; k < 8; ++k) {
      const int nx = node.gx + dx[k];
      const int ny = node.gy + dy[k];
      if (nx < 0 || ny < 0 || nx >= layer.width || ny >= layer.height) {
        continue;
      }
      const auto it = grid_to_node.find(ny * layer.width + nx);
      if (it == grid_to_node.end()) {
        continue;
      }
      const int a = std::min(node.id, it->second->id);
      const int b = std::max(node.id, it->second->id);
      const uint64_t key = (static_cast<uint64_t>(a) << 32) | static_cast<uint64_t>(b);
      if (!drawn.insert(key).second) {
        continue;
      }
      geometry_msgs::msg::Point p0, p1;
      p0.x = node.x;
      p0.y = node.y;
      p0.z = 0.2;
      p1.x = it->second->x;
      p1.y = it->second->y;
      p1.z = 0.2;
      edges.points.push_back(p0);
      edges.points.push_back(p1);
    }
  }
}

visualization_msgs::msg::Marker makeExplicitEdges(
  const NodeMap & layer,
  const std::string & frame_id,
  const std::string & ns,
  int id,
  float r, float g, float b,
  double line_width)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = rclcpp::Time(0);
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::msg::Marker::LINE_LIST;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x = line_width;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = 0.75f;
  m.frame_locked = true;

  if (layer.adjacency_mode == AdjacencyMode::GRID8) {
    appendGrid8Edges(layer, m);
    return m;
  }

  for (size_t i = 0; i < layer.adjacency.size() && i < layer.nodes.size(); ++i) {
    const auto & a = layer.nodes[i];
    for (int nb : layer.adjacency[i]) {
      if (nb < 0 || static_cast<size_t>(nb) >= layer.nodes.size()) {
        continue;
      }
      if (nb <= static_cast<int>(i)) {
        continue;
      }
      const auto & b = layer.nodes[static_cast<size_t>(nb)];
      geometry_msgs::msg::Point p0, p1;
      p0.x = a.x;
      p0.y = a.y;
      p0.z = 0.2;
      p1.x = b.x;
      p1.y = b.y;
      p1.z = 0.2;
      m.points.push_back(p0);
      m.points.push_back(p1);
    }
  }
  return m;
}

void appendLayerMarkers(
  visualization_msgs::msg::MarkerArray & array,
  const NodeMap & layer,
  const std::string & frame_id,
  const std::string & ns,
  int & id_base,
  float r, float g, float b,
  double node_scale,
  double edge_width,
  bool draw_edges)
{
  if (layer.nodes.empty()) {
    return;
  }
  array.markers.push_back(makeNodeSpheres(
      layer, frame_id, ns, id_base++, r, g, b, node_scale));
  if (draw_edges) {
    array.markers.push_back(makeExplicitEdges(
        layer, frame_id, ns + "_edges", id_base++, r, g, b, edge_width));
  }
}

class NodeMapVisualizer : public rclcpp::Node
{
public:
  NodeMapVisualizer()
  : Node(
      "nodemap_visualizer",
      rclcpp::NodeOptions()
      .allow_undeclared_parameters(true)
      .automatically_declare_parameters_from_overrides(true))
  {
    pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/nodemap_markers", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    reloadMaps();

    const double rate = get_parameter_or("publish_rate_hz", 1.0);
    if (rate > 0.0) {
      const auto period = std::chrono::duration<double>(1.0 / rate);
      timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&NodeMapVisualizer::publishMarkers, this));
    } else {
      publishMarkers();
    }
  }

private:
  bool reloadMaps()
  {
    const std::string map_yaml = nav2_map_server::expandUserPath(
      get_parameter_or<std::string>("map_yaml", ""));
    if (map_yaml.empty()) {
      RCLCPP_ERROR(get_logger(), "Parameter map_yaml is required");
      return false;
    }

    const std::string global_yaml = nav2_map_server::defaultGlobalNodeMapPath(map_yaml);
    const std::string local_yaml = nav2_map_server::defaultLocalNodeMapPath(map_yaml);
    const std::string raw_yaml = nav2_map_server::defaultRawNodeMapPath(map_yaml);

    has_global_ = nav2_map_server::loadNodeMap(global_yaml, global_map_);
    has_local_ = nav2_map_server::loadNodeMap(local_yaml, local_map_);
    has_raw_ = nav2_map_server::loadNodeMap(raw_yaml, raw_map_);

    if (!has_global_) {
      RCLCPP_ERROR(get_logger(), "Failed to load L1 global: %s", global_yaml.c_str());
      return false;
    }
    if (!has_local_) {
      RCLCPP_WARN(get_logger(), "L2 local not found: %s", local_yaml.c_str());
    }
    if (!has_raw_) {
      RCLCPP_WARN(get_logger(), "L3 raw not found: %s", raw_yaml.c_str());
    }

    RCLCPP_INFO(
      get_logger(),
      "Loaded nodemap layers — global:%zu local:%zu raw:%zu nodes",
      global_map_.nodes.size(),
      local_map_.nodes.size(),
      raw_map_.nodes.size());
    return true;
  }

  void publishMarkers()
  {
    if (!has_global_ && !has_local_ && !has_raw_) {
      return;
    }

    const std::string frame_id = get_parameter_or<std::string>("frame_id", "map");
    const bool show_global = get_parameter_or("show_global", true);
    const bool show_local = get_parameter_or("show_local", true);
    const bool show_raw = get_parameter_or("show_raw", false);
    const double global_scale = get_parameter_or("global_node_scale", 0.25);
    const double local_scale = get_parameter_or("local_node_scale", 0.12);
    const double raw_scale = get_parameter_or("raw_node_scale", 0.04);
    const double edge_width = get_parameter_or("edge_line_width", 0.03);
    const int raw_edge_limit = get_parameter_or("raw_max_nodes_with_edges", 8000);

    visualization_msgs::msg::MarkerArray array;
    int id = 0;

    if (show_global && has_global_) {
      appendLayerMarkers(
        array, global_map_, frame_id, "L1_global", id,
        0.10f, 0.45f, 0.85f, global_scale, edge_width, true);
    }
    if (show_local && has_local_) {
      appendLayerMarkers(
        array, local_map_, frame_id, "L2_local", id,
        0.26f, 0.63f, 0.28f, local_scale, edge_width * 0.8, true);
    }
    if (show_raw && has_raw_) {
      const bool raw_edges = static_cast<int>(raw_map_.nodes.size()) <= raw_edge_limit;
      appendLayerMarkers(
        array, raw_map_, frame_id, "L3_raw", id,
        0.90f, 0.32f, 0.05f, raw_scale, edge_width * 0.4, raw_edges);
      if (!raw_edges) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 10000,
          "L3 raw has %zu nodes — showing nodes only (no edges). "
          "Increase raw_max_nodes_with_edges to draw edges.",
          raw_map_.nodes.size());
      }
    }

    for (auto & m : array.markers) {
      m.header.stamp = rclcpp::Time(0);
    }
    pub_->publish(array);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Published %zu nodemap markers (global/local/raw enabled: %d/%d/%d)",
      array.markers.size(), show_global, show_local, show_raw);
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  NodeMap global_map_;
  NodeMap local_map_;
  NodeMap raw_map_;
  bool has_global_{false};
  bool has_local_{false};
  bool has_raw_{false};
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NodeMapVisualizer>());
  rclcpp::shutdown();
  return 0;
}
