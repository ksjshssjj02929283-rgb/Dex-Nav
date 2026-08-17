// Copyright (c) DexForce
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include <filesystem>
#include <iostream>
#include <string>

#include "nav2_map_server/map_preprocess.hpp"
#include "nav2_map_server/node_map_io.hpp"

void printUsage(const char * prog)
{
  std::cout <<
    "Usage: " << prog << " --map-yaml <path/to/map.yaml> [options]\n"
    "\n"
    "Build preprocessed maps under <map_dir>/preprocessed/:\n"
    "  distance_map.yaml + distance_map.pgm\n"
    "  nodemap_global.yaml  (layer 1, global Theta*)\n"
    "  nodemap_local.yaml   (layer 2, local refinement)\n"
    "  nodemap_raw.yaml     (layer 3, original grid for goal accuracy)\n"
    "\n"
    "Options:\n"
    "  --global-clearance <m>     default 0.45\n"
    "  --global-spacing <m>       default 0.80\n"
    "  --global-connect <m>       default 1.20\n"
    "  --local-clearance <m>      default 0.30\n"
    "  --local-spacing <m>        default 0.40\n"
    "  --local-connect <m>        default 0.70\n"
    "  --max-distance <m>         default 2.0\n"
    "  --unknown-as-obstacle      default true\n"
    "  --unknown-as-free          treat unknown as free when building maps\n"
            << std::endl;
}

int main(int argc, char ** argv)
{
  std::string map_yaml;
  nav2_map_server::MapPreprocessParams params;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      printUsage(argv[0]);
      return 0;
    }
    if (arg == "--map-yaml" && i + 1 < argc) {
      map_yaml = nav2_map_server::expandUserPath(argv[++i]);
    } else if (arg == "--global-clearance" && i + 1 < argc) {
      params.global_min_clearance_m = std::stod(argv[++i]);
    } else if (arg == "--global-spacing" && i + 1 < argc) {
      params.global_node_spacing_m = std::stod(argv[++i]);
    } else if (arg == "--global-connect" && i + 1 < argc) {
      params.global_connection_radius_m = std::stod(argv[++i]);
    } else if (arg == "--local-clearance" && i + 1 < argc) {
      params.local_min_clearance_m = std::stod(argv[++i]);
    } else if (arg == "--local-spacing" && i + 1 < argc) {
      params.local_node_spacing_m = std::stod(argv[++i]);
    } else if (arg == "--local-connect" && i + 1 < argc) {
      params.local_connection_radius_m = std::stod(argv[++i]);
    } else if (arg == "--max-distance" && i + 1 < argc) {
      params.max_distance_m = std::stod(argv[++i]);
    } else if (arg == "--unknown-as-free") {
      params.treat_unknown_as_obstacle = false;
    } else {
      std::cerr << "Unknown argument: " << arg << std::endl;
      printUsage(argv[0]);
      return 1;
    }
  }

  if (map_yaml.empty()) {
    std::cerr << "Error: --map-yaml is required\n";
    printUsage(argv[0]);
    return 1;
  }

  if (!nav2_map_server::preprocessMapYaml(map_yaml, params)) {
    std::cerr << "Failed to preprocess map: " << map_yaml << std::endl;
    return 1;
  }

  const auto out_dir = std::filesystem::path(map_yaml).parent_path() / "preprocessed";
  std::cout << "Preprocessed maps written to: " << out_dir.string() << std::endl;
  return 0;
}
