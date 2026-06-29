// Copyright (c) DexForce
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef NAV2_MAP_SERVER__NODE_MAP_TYPES_HPP_
#define NAV2_MAP_SERVER__NODE_MAP_TYPES_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace nav2_map_server
{

/// One node in a preprocessed planning graph (grid indices + world coords).
struct NodeMapNode
{
  int id{0};
  int gx{0};
  int gy{0};
  double x{0.0};
  double y{0.0};
};

/// Preprocessed distance field aligned with the source occupancy grid.
struct DistanceMap
{
  int width{0};
  int height{0};
  double resolution{0.05};
  double origin_x{0.0};
  double origin_y{0.0};
  double max_distance_m{2.0};
  /// distance to nearest obstacle in meters; size = width * height
  std::vector<float> distance_m;
};

enum class AdjacencyMode
{
  EXPLICIT = 0,  ///< adjacency list stored in yaml
  GRID8 = 1      ///< 8-connected grid neighbours (layer 3 raw)
};

/// Sparse graph for Theta* / local planning.
struct NodeMap
{
  static constexpr int kVersion = 2;

  int version{kVersion};
  std::string layer;  ///< "global" | "local" | "raw"
  std::string source_map;
  std::string distance_map;
  /// For raw layer: binary mask pgm (255 = node cell on original free grid)
  std::string node_mask_image;
  AdjacencyMode adjacency_mode{AdjacencyMode::EXPLICIT};
  double min_clearance_m{0.45};
  double node_spacing_m{0.80};
  double connection_radius_m{1.20};
  double resolution{0.05};
  double origin_x{0.0};
  double origin_y{0.0};
  int width{0};
  int height{0};

  std::vector<NodeMapNode> nodes;
  /// adjacency[i] = neighbor node ids (EXPLICIT mode only)
  std::vector<std::vector<int>> adjacency;
  /// grid index: size width*height, -1 or node id (filled after load for GRID8)
  std::vector<int> grid_node_id;
};

struct MapPreprocessParams
{
  bool treat_unknown_as_obstacle{true};

  /// Layer 1 — sparse global Theta*
  double global_min_clearance_m{0.40};
  double global_node_spacing_m{0.60};
  double global_connection_radius_m{0.90};

  /// Layer 2 — finer local refinement
  double local_min_clearance_m{0.30};
  double local_node_spacing_m{0.40};
  double local_connection_radius_m{0.70};

  /// Layer 3 — raw/original grid (1 cell, no extra clearance) for final goal accuracy
  double raw_min_clearance_m{0.0};

  double max_distance_m{2.0};
};

struct MapPreprocessResult
{
  DistanceMap distance_map;
  NodeMap global_layer;
  NodeMap local_layer;
  NodeMap raw_layer;
  std::string output_dir;
};

}  // namespace nav2_map_server

#endif  // NAV2_MAP_SERVER__NODE_MAP_TYPES_HPP_
