// Copyright (c) DexForce
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "nav2_map_server/node_map_io.hpp"

#include <pwd.h>
#include <unistd.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>

#include "nav2_map_server/map_mode.hpp"
#include "yaml-cpp/yaml.h"

namespace nav2_map_server
{
namespace
{

std::filesystem::path parentDir(const std::string & file_path)
{
  return std::filesystem::path(file_path).parent_path();
}

bool writeDistancePgm(const DistanceMap & distance_map, const std::string & pgm_path)
{
  FILE * fp = std::fopen(pgm_path.c_str(), "wb");
  if (!fp) {
    return false;
  }
  std::fprintf(fp, "P5\n%d %d\n255\n", distance_map.width, distance_map.height);
  for (int y = distance_map.height - 1; y >= 0; --y) {
    for (int x = 0; x < distance_map.width; ++x) {
      const size_t i = static_cast<size_t>(y * distance_map.width + x);
      const float d = distance_map.distance_m[i];
      const float norm = distance_map.max_distance_m > 1e-6 ?
        d / static_cast<float>(distance_map.max_distance_m) : 0.0f;
      const int pixel = std::clamp(static_cast<int>(std::round(norm * 254.0f)), 0, 254);
      std::fputc(pixel, fp);
    }
  }
  std::fclose(fp);
  return true;
}

bool readDistancePgm(const std::string & pgm_path, DistanceMap & distance_map)
{
  FILE * fp = std::fopen(pgm_path.c_str(), "rb");
  if (!fp) {
    return false;
  }
  int w = 0;
  int h = 0;
  int max_val = 0;
  if (std::fscanf(fp, "P5\n%d %d\n%d\n", &w, &h, &max_val) != 3) {
    std::fclose(fp);
    return false;
  }
  distance_map.width = w;
  distance_map.height = h;
  distance_map.distance_m.assign(static_cast<size_t>(w * h), 0.0f);
  std::vector<unsigned char> row(static_cast<size_t>(w));
  for (int y = h - 1; y >= 0; --y) {
    if (std::fread(row.data(), 1, static_cast<size_t>(w), fp) != static_cast<size_t>(w)) {
      std::fclose(fp);
      return false;
    }
    for (int x = 0; x < w; ++x) {
      const float norm = static_cast<float>(row[static_cast<size_t>(x)]) / 254.0f;
      distance_map.distance_m[static_cast<size_t>(y * w + x)] =
        norm * static_cast<float>(distance_map.max_distance_m);
    }
  }
  std::fclose(fp);
  return true;
}

bool writeRawMaskPgm(const NodeMap & node_map, const std::string & pgm_path)
{
  FILE * fp = std::fopen(pgm_path.c_str(), "wb");
  if (!fp) {
    return false;
  }
  std::fprintf(fp, "P5\n%d %d\n255\n", node_map.width, node_map.height);
  for (int y = node_map.height - 1; y >= 0; --y) {
    for (int x = 0; x < node_map.width; ++x) {
      const size_t i = static_cast<size_t>(y * node_map.width + x);
      const bool is_node = !node_map.grid_node_id.empty() &&
        node_map.grid_node_id[i] >= 0;
      std::fputc(is_node ? 255 : 0, fp);
    }
  }
  std::fclose(fp);
  return true;
}

bool readRawMaskPgm(const std::string & pgm_path, NodeMap & node_map)
{
  FILE * fp = std::fopen(pgm_path.c_str(), "rb");
  if (!fp) {
    return false;
  }
  int w = 0;
  int h = 0;
  int max_val = 0;
  if (std::fscanf(fp, "P5\n%d %d\n%d\n", &w, &h, &max_val) != 3) {
    std::fclose(fp);
    return false;
  }
  node_map.width = w;
  node_map.height = h;
  node_map.nodes.clear();
  node_map.grid_node_id.assign(static_cast<size_t>(w * h), -1);

  std::vector<unsigned char> row(static_cast<size_t>(w));
  int next_id = 0;
  for (int y = h - 1; y >= 0; --y) {
    if (std::fread(row.data(), 1, static_cast<size_t>(w), fp) != static_cast<size_t>(w)) {
      std::fclose(fp);
      return false;
    }
    for (int x = 0; x < w; ++x) {
      if (row[static_cast<size_t>(x)] < 128) {
        continue;
      }
      NodeMapNode node;
      node.id = next_id++;
      node.gx = x;
      node.gy = y;
      node.x = node_map.origin_x + (x + 0.5) * node_map.resolution;
      node.y = node_map.origin_y + (y + 0.5) * node_map.resolution;
      node_map.nodes.push_back(node);
      node_map.grid_node_id[static_cast<size_t>(y * w + x)] = node.id;
    }
  }
  std::fclose(fp);
  return !node_map.nodes.empty();
}

}  // namespace

std::string expandUserPath(const std::string & path)
{
  if (path.empty() || path[0] != '~') {
    return path;
  }
  std::string home;
  if (const char * environment_home = std::getenv("HOME")) {
    home = environment_home;
  } else {
    passwd user_entry{};
    passwd * result = nullptr;
    std::array<char, 16384> buffer{};
    if (getpwuid_r(
        getuid(), &user_entry, buffer.data(), buffer.size(), &result) == 0 && result)
    {
      home = user_entry.pw_dir;
    }
  }
  if (home.empty()) {
    return path;
  }
  if (path.size() == 1) {
    return home;
  }
  if (path[1] == '/') {
    return home + path.substr(1);
  }
  return path;
}

std::string defaultGlobalNodeMapPath(const std::string & map_yaml)
{
  return (parentDir(map_yaml) / "preprocessed" / "nodemap_global.yaml").string();
}

std::string defaultLocalNodeMapPath(const std::string & map_yaml)
{
  return (parentDir(map_yaml) / "preprocessed" / "nodemap_local.yaml").string();
}

std::string defaultRawNodeMapPath(const std::string & map_yaml)
{
  return (parentDir(map_yaml) / "preprocessed" / "nodemap_raw.yaml").string();
}

std::string defaultDistanceMapPath(const std::string & map_yaml)
{
  return (parentDir(map_yaml) / "preprocessed" / "distance_map.yaml").string();
}

bool saveDistanceMap(const DistanceMap & distance_map, const std::string & yaml_path)
{
  const std::filesystem::path yaml_file(yaml_path);
  const std::filesystem::path pgm_file = yaml_file.parent_path() / "distance_map.pgm";
  if (!writeDistancePgm(distance_map, pgm_file.string())) {
    return false;
  }

  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "image" << YAML::Value << pgm_file.filename().string();
  out << YAML::Key << "resolution" << YAML::Value << distance_map.resolution;
  out << YAML::Key << "origin" << YAML::Flow << YAML::BeginSeq << distance_map.origin_x <<
    distance_map.origin_y << 0.0 << YAML::EndSeq;
  out << YAML::Key << "max_distance_m" << YAML::Value << distance_map.max_distance_m;
  out << YAML::Key << "width" << YAML::Value << distance_map.width;
  out << YAML::Key << "height" << YAML::Value << distance_map.height;
  out << YAML::EndMap;

  std::ofstream ofs(yaml_path);
  if (!ofs) {
    return false;
  }
  ofs << out.c_str();
  return true;
}

bool loadDistanceMap(const std::string & yaml_path, DistanceMap & distance_map)
{
  YAML::Node yaml = YAML::LoadFile(yaml_path);
  distance_map.resolution = yaml["resolution"].as<double>();
  const auto origin = yaml["origin"].as<std::vector<double>>();
  distance_map.origin_x = origin[0];
  distance_map.origin_y = origin[1];
  distance_map.max_distance_m = yaml["max_distance_m"].as<double>(2.0);
  distance_map.width = yaml["width"].as<int>();
  distance_map.height = yaml["height"].as<int>();

  const std::filesystem::path yaml_file(yaml_path);
  const std::string image = yaml["image"].as<std::string>();
  const std::filesystem::path pgm_path = yaml_file.parent_path() / image;
  return readDistancePgm(pgm_path.string(), distance_map);
}

bool saveNodeMap(const NodeMap & node_map, const std::string & yaml_path)
{
  const std::filesystem::path yaml_file(yaml_path);
  const bool grid8 = node_map.adjacency_mode == AdjacencyMode::GRID8;

  if (grid8 && !node_map.grid_node_id.empty()) {
    const std::string mask_name = node_map.node_mask_image.empty() ?
      "nodemap_raw_mask.pgm" : node_map.node_mask_image;
    const std::filesystem::path mask_path = yaml_file.parent_path() / mask_name;
    if (!writeRawMaskPgm(node_map, mask_path.string())) {
      return false;
    }
  }

  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "version" << YAML::Value << node_map.version;
  out << YAML::Key << "layer" << YAML::Value << node_map.layer;
  out << YAML::Key << "source_map" << YAML::Value << node_map.source_map;
  out << YAML::Key << "distance_map" << YAML::Value << node_map.distance_map;
  out << YAML::Key << "adjacency_mode" << YAML::Value <<
    (grid8 ? "grid8" : "explicit");
  if (grid8) {
    out << YAML::Key << "node_mask_image" << YAML::Value <<
      (node_map.node_mask_image.empty() ? "nodemap_raw_mask.pgm" : node_map.node_mask_image);
    out << YAML::Key << "node_count" << YAML::Value << node_map.nodes.size();
  }
  out << YAML::Key << "min_clearance_m" << YAML::Value << node_map.min_clearance_m;
  out << YAML::Key << "node_spacing_m" << YAML::Value << node_map.node_spacing_m;
  out << YAML::Key << "connection_radius_m" << YAML::Value << node_map.connection_radius_m;
  out << YAML::Key << "resolution" << YAML::Value << node_map.resolution;
  out << YAML::Key << "origin" << YAML::Flow << YAML::BeginSeq << node_map.origin_x <<
    node_map.origin_y << 0.0 << YAML::EndSeq;
  out << YAML::Key << "width" << YAML::Value << node_map.width;
  out << YAML::Key << "height" << YAML::Value << node_map.height;

  if (!grid8) {
    out << YAML::Key << "nodes" << YAML::Value << YAML::BeginSeq;
    for (const auto & node : node_map.nodes) {
      out << YAML::BeginMap;
      out << YAML::Key << "id" << YAML::Value << node.id;
      out << YAML::Key << "x" << YAML::Value << node.x;
      out << YAML::Key << "y" << YAML::Value << node.y;
      out << YAML::Key << "gx" << YAML::Value << node.gx;
      out << YAML::Key << "gy" << YAML::Value << node.gy;
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;

    out << YAML::Key << "adjacency" << YAML::Value << YAML::BeginSeq;
    for (const auto & neighbors : node_map.adjacency) {
      out << YAML::Flow << neighbors;
    }
    out << YAML::EndSeq;
  }
  out << YAML::EndMap;

  std::ofstream ofs(yaml_path);
  if (!ofs) {
    return false;
  }
  ofs << out.c_str();
  return true;
}

bool loadNodeMap(const std::string & yaml_path, NodeMap & node_map)
{
  YAML::Node yaml = YAML::LoadFile(yaml_path);
  node_map.version = yaml["version"].as<int>(NodeMap::kVersion);
  node_map.layer = yaml["layer"].as<std::string>();
  node_map.source_map = yaml["source_map"].as<std::string>("");
  node_map.distance_map = yaml["distance_map"].as<std::string>("");
  const std::string adj_mode = yaml["adjacency_mode"].as<std::string>("explicit");
  node_map.adjacency_mode = (adj_mode == "grid8") ?
    AdjacencyMode::GRID8 : AdjacencyMode::EXPLICIT;
  node_map.node_mask_image = yaml["node_mask_image"].as<std::string>("");
  node_map.min_clearance_m = yaml["min_clearance_m"].as<double>();
  node_map.node_spacing_m = yaml["node_spacing_m"].as<double>();
  node_map.connection_radius_m = yaml["connection_radius_m"].as<double>();
  node_map.resolution = yaml["resolution"].as<double>();
  const auto origin = yaml["origin"].as<std::vector<double>>();
  node_map.origin_x = origin[0];
  node_map.origin_y = origin[1];
  node_map.width = yaml["width"].as<int>();
  node_map.height = yaml["height"].as<int>();

  if (node_map.adjacency_mode == AdjacencyMode::GRID8) {
    const std::filesystem::path yaml_file(yaml_path);
    const std::string mask_name = node_map.node_mask_image.empty() ?
      "nodemap_raw_mask.pgm" : node_map.node_mask_image;
    return readRawMaskPgm((yaml_file.parent_path() / mask_name).string(), node_map);
  }

  node_map.nodes.clear();
  for (const auto & n : yaml["nodes"]) {
    NodeMapNode node;
    node.id = n["id"].as<int>();
    node.x = n["x"].as<double>();
    node.y = n["y"].as<double>();
    node.gx = n["gx"].as<int>();
    node.gy = n["gy"].as<int>();
    node_map.nodes.push_back(node);
  }

  node_map.adjacency.clear();
  node_map.adjacency.resize(node_map.nodes.size());
  const auto adj = yaml["adjacency"];
  for (size_t i = 0; i < adj.size() && i < node_map.adjacency.size(); ++i) {
    for (const auto & nb : adj[static_cast<int>(i)]) {
      node_map.adjacency[i].push_back(nb.as<int>());
    }
  }
  return !node_map.nodes.empty();
}

}  // namespace nav2_map_server
