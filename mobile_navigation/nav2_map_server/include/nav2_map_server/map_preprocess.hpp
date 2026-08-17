// Copyright (c) DexForce
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef NAV2_MAP_SERVER__MAP_PREPROCESS_HPP_
#define NAV2_MAP_SERVER__MAP_PREPROCESS_HPP_

#include <string>

#include "nav2_map_server/node_map_types.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace nav2_map_server
{

/**
 * @brief Build distance map + global/local node maps from an occupancy grid.
 */
MapPreprocessResult buildPreprocessedMaps(
  const nav_msgs::msg::OccupancyGrid & occupancy,
  const MapPreprocessParams & params = MapPreprocessParams());

/**
 * @brief Load map.yaml + image, preprocess, write to <map_dir>/preprocessed/.
 * @param map_yaml Absolute or relative path to map.yaml
 * @param params Preprocess tuning
 * @return true on success
 */
bool preprocessMapYaml(
  const std::string & map_yaml,
  const MapPreprocessParams & params = MapPreprocessParams());

}  // namespace nav2_map_server

#endif  // NAV2_MAP_SERVER__MAP_PREPROCESS_HPP_
