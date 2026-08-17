// Copyright (c) DexForce
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "nav2_map_server/map_preprocess.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <queue>
#include <vector>

#include "nav2_map_server/map_io.hpp"
#include "nav2_map_server/node_map_io.hpp"

namespace nav2_map_server
{
namespace
{

constexpr float kSqrt2 = 1.41421356237f;

inline int idx(int x, int y, int width)
{
  return y * width + x;
}

bool isOccupied(int8_t value, bool treat_unknown_as_obstacle)
{
  if (value < 0) {
    return treat_unknown_as_obstacle;
  }
  return value >= 50;
}

DistanceMap computeDistanceMap(
  const nav_msgs::msg::OccupancyGrid & occupancy,
  const MapPreprocessParams & params)
{
  const int width = static_cast<int>(occupancy.info.width);
  const int height = static_cast<int>(occupancy.info.height);
  const auto & data = occupancy.data;
  const float res = static_cast<float>(occupancy.info.resolution);
  const float diag = res * kSqrt2;

  DistanceMap dist;
  dist.width = width;
  dist.height = height;
  dist.resolution = occupancy.info.resolution;
  dist.origin_x = occupancy.info.origin.position.x;
  dist.origin_y = occupancy.info.origin.position.y;
  dist.max_distance_m = params.max_distance_m;
  dist.distance_m.assign(static_cast<size_t>(width * height), 0.0f);

  const float inf = std::numeric_limits<float>::infinity();
  std::vector<float> grid(static_cast<size_t>(width * height), inf);

  struct QItem
  {
    float d;
    int x;
    int y;
  };
  struct QCmp
  {
    bool operator()(const QItem & a, const QItem & b) const {return a.d > b.d;}
  };
  std::priority_queue<QItem, std::vector<QItem>, QCmp> pq;

  static const int ndx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int ndy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  static const float ncost[8] = {
    res, res, res, res, diag, diag, diag, diag};

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const int i = idx(x, y, width);
      if (isOccupied(data[static_cast<size_t>(i)], params.treat_unknown_as_obstacle)) {
        grid[static_cast<size_t>(i)] = 0.0f;
        pq.push({0.0f, x, y});
      }
    }
  }

  while (!pq.empty()) {
    const QItem cur = pq.top();
    pq.pop();
    const size_t ci = static_cast<size_t>(idx(cur.x, cur.y, width));
    if (cur.d > grid[ci]) {
      continue;
    }
    for (int k = 0; k < 8; ++k) {
      const int nx = cur.x + ndx[k];
      const int ny = cur.y + ndy[k];
      if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
        continue;
      }
      const size_t ni = static_cast<size_t>(idx(nx, ny, width));
      const float nd = cur.d + ncost[k];
      if (nd < grid[ni]) {
        grid[ni] = nd;
        pq.push({nd, nx, ny});
      }
    }
  }

  for (size_t i = 0; i < grid.size(); ++i) {
    if (std::isinf(grid[i])) {
      dist.distance_m[i] = 0.0f;
    } else {
      dist.distance_m[i] = std::min(grid[i], static_cast<float>(params.max_distance_m));
    }
  }
  return dist;
}

bool losClearanceOk(
  int x0, int y0, int x1, int y1,
  const DistanceMap & distance_map,
  double min_clearance_m)
{
  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = x0 < x1 ? 1 : -1;
  int sy = y0 < y1 ? 1 : -1;
  int err = dx - dy;
  int x = x0;
  int y = y0;

  while (true) {
    const int i = idx(x, y, distance_map.width);
    if (distance_map.distance_m[static_cast<size_t>(i)] < static_cast<float>(min_clearance_m)) {
      return false;
    }
    if (x == x1 && y == y1) {
      break;
    }
    const int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x += sx;
    }
    if (e2 < dx) {
      err += dx;
      y += sy;
    }
  }
  return true;
}

NodeMap buildNodeLayer(
  const nav_msgs::msg::OccupancyGrid & occupancy,
  const DistanceMap & distance_map,
  const std::string & layer_name,
  double min_clearance_m,
  double node_spacing_m,
  double connection_radius_m,
  const std::string & source_map_rel,
  const std::string & distance_map_rel)
{
  NodeMap layer;
  layer.layer = layer_name;
  layer.source_map = source_map_rel;
  layer.distance_map = distance_map_rel;
  layer.min_clearance_m = min_clearance_m;
  layer.node_spacing_m = node_spacing_m;
  layer.connection_radius_m = connection_radius_m;
  layer.resolution = occupancy.info.resolution;
  layer.origin_x = occupancy.info.origin.position.x;
  layer.origin_y = occupancy.info.origin.position.y;
  layer.width = static_cast<int>(occupancy.info.width);
  layer.height = static_cast<int>(occupancy.info.height);

  const int step = std::max(1, static_cast<int>(std::round(node_spacing_m / layer.resolution)));
  const int width = layer.width;
  const int height = layer.height;

  int next_id = 0;
  for (int gy = step / 2; gy < height; gy += step) {
    for (int gx = step / 2; gx < width; gx += step) {
      const size_t i = static_cast<size_t>(idx(gx, gy, width));
      if (isOccupied(occupancy.data[i], true)) {
        continue;
      }
      if (distance_map.distance_m[i] < static_cast<float>(min_clearance_m)) {
        continue;
      }
      NodeMapNode node;
      node.id = next_id++;
      node.gx = gx;
      node.gy = gy;
      node.x = layer.origin_x + (gx + 0.5) * layer.resolution;
      node.y = layer.origin_y + (gy + 0.5) * layer.resolution;
      layer.nodes.push_back(node);
    }
  }

  const double conn_radius_sq = connection_radius_m * connection_radius_m;
  layer.adjacency.resize(layer.nodes.size());

  for (size_t a = 0; a < layer.nodes.size(); ++a) {
    for (size_t b = a + 1; b < layer.nodes.size(); ++b) {
      const auto & na = layer.nodes[a];
      const auto & nb = layer.nodes[b];
      const double dx = na.x - nb.x;
      const double dy = na.y - nb.y;
      if (dx * dx + dy * dy > conn_radius_sq) {
        continue;
      }
      if (!losClearanceOk(
          na.gx, na.gy, nb.gx, nb.gy, distance_map, min_clearance_m))
      {
        continue;
      }
      layer.adjacency[a].push_back(static_cast<int>(b));
      layer.adjacency[b].push_back(static_cast<int>(a));
    }
  }

  return layer;
}

NodeMap buildRawLayer(
  const nav_msgs::msg::OccupancyGrid & occupancy,
  const MapPreprocessParams & params,
  const std::string & source_map_rel,
  const std::string & distance_map_rel)
{
  NodeMap layer;
  layer.layer = "raw";
  layer.source_map = source_map_rel;
  layer.distance_map = distance_map_rel;
  layer.adjacency_mode = AdjacencyMode::GRID8;
  layer.min_clearance_m = params.raw_min_clearance_m;
  layer.node_spacing_m = occupancy.info.resolution;
  layer.connection_radius_m = occupancy.info.resolution * std::sqrt(2.0);
  layer.resolution = occupancy.info.resolution;
  layer.origin_x = occupancy.info.origin.position.x;
  layer.origin_y = occupancy.info.origin.position.y;
  layer.width = static_cast<int>(occupancy.info.width);
  layer.height = static_cast<int>(occupancy.info.height);
  layer.node_mask_image = "nodemap_raw_mask.pgm";

  const int width = layer.width;
  const int height = layer.height;
  int next_id = 0;

  layer.grid_node_id.assign(static_cast<size_t>(width * height), -1);
  for (int gy = 0; gy < height; ++gy) {
    for (int gx = 0; gx < width; ++gx) {
      const size_t i = static_cast<size_t>(idx(gx, gy, width));
      if (isOccupied(occupancy.data[i], params.treat_unknown_as_obstacle)) {
        continue;
      }
      if (params.raw_min_clearance_m > 0.0) {
        // optional: skip if too close — raw layer uses original free cells only
      }
      NodeMapNode node;
      node.id = next_id++;
      node.gx = gx;
      node.gy = gy;
      node.x = layer.origin_x + (gx + 0.5) * layer.resolution;
      node.y = layer.origin_y + (gy + 0.5) * layer.resolution;
      layer.nodes.push_back(node);
      layer.grid_node_id[i] = node.id;
    }
  }

  return layer;
}

}  // namespace

MapPreprocessResult buildPreprocessedMaps(
  const nav_msgs::msg::OccupancyGrid & occupancy,
  const MapPreprocessParams & params)
{
  MapPreprocessResult result;
  result.distance_map = computeDistanceMap(occupancy, params);
  result.global_layer = buildNodeLayer(
    occupancy, result.distance_map, "global",
    params.global_min_clearance_m,
    params.global_node_spacing_m,
    params.global_connection_radius_m,
    "../map.yaml", "distance_map.yaml");
  result.local_layer = buildNodeLayer(
    occupancy, result.distance_map, "local",
    params.local_min_clearance_m,
    params.local_node_spacing_m,
    params.local_connection_radius_m,
    "../map.yaml", "distance_map.yaml");
  result.raw_layer = buildRawLayer(
    occupancy, params, "../map.yaml", "distance_map.yaml");
  return result;
}

bool preprocessMapYaml(
  const std::string & map_yaml,
  const MapPreprocessParams & params)
{
  nav_msgs::msg::OccupancyGrid occupancy;
  if (loadMapFromYaml(map_yaml, occupancy) != LOAD_MAP_SUCCESS) {
    return false;
  }

  const std::filesystem::path map_path(map_yaml);
  const std::filesystem::path map_dir = map_path.parent_path();
  const std::filesystem::path out_dir = map_dir / "preprocessed";
  std::filesystem::create_directories(out_dir);

  MapPreprocessResult result = buildPreprocessedMaps(occupancy, params);
  result.output_dir = out_dir.string();

  const std::string distance_yaml = (out_dir / "distance_map.yaml").string();
  const std::string global_yaml = (out_dir / "nodemap_global.yaml").string();
  const std::string local_yaml = (out_dir / "nodemap_local.yaml").string();
  const std::string raw_yaml = (out_dir / "nodemap_raw.yaml").string();

  if (!saveDistanceMap(result.distance_map, distance_yaml)) {
    return false;
  }
  if (!saveNodeMap(result.global_layer, global_yaml)) {
    return false;
  }
  if (!saveNodeMap(result.local_layer, local_yaml)) {
    return false;
  }
  if (!saveNodeMap(result.raw_layer, raw_yaml)) {
    return false;
  }
  return true;
}

}  // namespace nav2_map_server
