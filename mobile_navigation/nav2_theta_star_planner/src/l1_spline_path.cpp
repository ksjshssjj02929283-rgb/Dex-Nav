// Copyright (c) DexForce
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "nav2_theta_star_planner/l1_spline_path.hpp"

namespace nav2_theta_star_planner
{

bool L1SplinePathBuilder::build(
  const std::vector<coordsW> & graph_waypoints,
  const nav2_map_server::DistanceMap * distance_map,
  const L1SplinePathConfig & config,
  std::vector<coordsW> & dense_path,
  L1SplinePathStats * stats)
{
  dense_path.clear();
  if (graph_waypoints.size() < 2) {
    if (!graph_waypoints.empty()) {
      dense_path = graph_waypoints;
    }
    return true;
  }

  L1SplinePathStats local_stats;
  local_stats.graph_waypoints = graph_waypoints.size();

  std::vector<coordsW> working = graph_waypoints;
  if (config.simplify_angle_rad > 0.0 && working.size() >= 3) {
    CurvePathSmoother::simplifyCollinear(working, config.simplify_angle_rad);
  }
  local_stats.simplified_waypoints = working.size();

  CurvePathSmootherConfig curve_cfg;
  curve_cfg.corner_angle_rad = config.corner_angle_rad;
  curve_cfg.sample_step_m = config.sample_step_m;
  curve_cfg.fillet_max_leg_fraction = config.fillet_max_leg_fraction;

  CurveFitStats fillet_stats;
  CurvePathSmoother::fitCurvesGeometric(
    working, curve_cfg, distance_map, config.clearance_radius_factor, &fillet_stats);

  local_stats.fillet_corners_seen = fillet_stats.fillet_corners_seen;
  local_stats.fillet_corners_applied = fillet_stats.fillet_corners_applied;
  local_stats.fillet_corners_rejected = fillet_stats.fillet_corners_rejected;

  if (distance_map && !distance_map->distance_m.empty()) {
    local_stats.min_corner_clearance_m =
      CurvePathSmoother::minWaypointClearanceM(graph_waypoints, distance_map);
  }

  dense_path.swap(working);
  local_stats.dense_poses = dense_path.size();

  if (stats) {
    *stats = local_stats;
  }
  return dense_path.size() >= 2;
}

}  // namespace nav2_theta_star_planner
