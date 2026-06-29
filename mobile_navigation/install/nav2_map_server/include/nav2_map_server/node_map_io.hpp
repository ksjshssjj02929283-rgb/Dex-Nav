// Copyright (c) DexForce
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef NAV2_MAP_SERVER__NODE_MAP_IO_HPP_
#define NAV2_MAP_SERVER__NODE_MAP_IO_HPP_

#include <string>

#include "nav2_map_server/node_map_types.hpp"

namespace nav2_map_server
{

bool saveDistanceMap(
  const DistanceMap & distance_map,
  const std::string & yaml_path);

bool loadDistanceMap(
  const std::string & yaml_path,
  DistanceMap & distance_map);

bool saveNodeMap(
  const NodeMap & node_map,
  const std::string & yaml_path);

bool loadNodeMap(
  const std::string & yaml_path,
  NodeMap & node_map);

/// Resolve ~/.dexe/... style paths.
std::string expandUserPath(const std::string & path);

/// If map is ~/.dexe/data/maps/foo/map.yaml → .../foo/preprocessed/nodemap_global.yaml
std::string defaultGlobalNodeMapPath(const std::string & map_yaml);

std::string defaultLocalNodeMapPath(const std::string & map_yaml);

std::string defaultRawNodeMapPath(const std::string & map_yaml);

std::string defaultDistanceMapPath(const std::string & map_yaml);

}  // namespace nav2_map_server

#endif  // NAV2_MAP_SERVER__NODE_MAP_IO_HPP_
