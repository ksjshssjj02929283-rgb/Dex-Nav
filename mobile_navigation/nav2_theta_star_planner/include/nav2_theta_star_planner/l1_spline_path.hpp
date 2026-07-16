// Copyright (c) DexForce
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef NAV2_THETA_STAR_PLANNER__L1_SPLINE_PATH_HPP_
#define NAV2_THETA_STAR_PLANNER__L1_SPLINE_PATH_HPP_

#include <vector>

#include "nav2_map_server/node_map_types.hpp"
#include "nav2_theta_star_planner/curve_path_smoother.hpp"
#include "nav2_theta_star_planner/theta_star.hpp"

namespace nav2_theta_star_planner
{

/// Plan E: sample L1 graph waypoints as straight edge segments + geometric corner fillets.
/// Uses distance_map (same as L1 planning), not costmap inflation.
struct L1SplinePathConfig
{
  double sample_step_m{0.05};
  /// Min turn angle to attempt a fillet (rad).
  double corner_angle_rad{0.25};
  /// Remove interior graph vertices below this turn angle before filleting (rad).
  double simplify_angle_rad{0.10};
  double fillet_max_leg_fraction{0.45};
  /// Max fillet radius from corner clearance: R <= clear(B) * tan(θ/2) * factor
  double clearance_radius_factor{0.85};
};

struct L1SplinePathStats
{
  size_t graph_waypoints{0};
  size_t simplified_waypoints{0};
  size_t sparse_poses{0};
  size_t dense_poses{0};
  size_t fillet_corners_seen{0};
  size_t fillet_corners_applied{0};
  size_t fillet_corners_rejected{0};
  double min_corner_clearance_m{0.0};
};

class L1SplinePathBuilder
{
public:
  /// Dense path from sparse L1 graph vertices (no string-pull / no costmap gradient).
  static bool build(
    const std::vector<coordsW> & graph_waypoints,
    const nav2_map_server::DistanceMap * distance_map,
    const L1SplinePathConfig & config,
    std::vector<coordsW> & dense_path,
    L1SplinePathStats * stats = nullptr);
};

}  // namespace nav2_theta_star_planner

#endif  // NAV2_THETA_STAR_PLANNER__L1_SPLINE_PATH_HPP_
