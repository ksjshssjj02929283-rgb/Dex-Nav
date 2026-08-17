// Copyright (c) DexForce
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef NAV2_THETA_STAR_PLANNER__CURVE_PATH_SMOOTHER_HPP_
#define NAV2_THETA_STAR_PLANNER__CURVE_PATH_SMOOTHER_HPP_

#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_map_server/node_map_types.hpp"
#include "nav2_theta_star_planner/theta_star.hpp"

namespace nav2_theta_star_planner
{

struct CurvePathSmootherConfig
{
  /// Corner angle above which a fillet is attempted (rad).
  double corner_angle_rad{0.17};
  /// Sample step along lines / arcs (m).
  double sample_step_m{0.05};
  /// Max fraction of each leg trimmed for fillet tangent points (0–0.65).
  double fillet_max_leg_fraction{0.35};
};

struct CurveFitStats
{
  size_t line_segments{0};
  size_t arc_segments{0};
  size_t spline_segments{0};
  size_t fallback_segments{0};
  size_t fillet_corners_seen{0};
  size_t fillet_corners_applied{0};
  size_t fillet_corners_rejected{0};
  size_t fillet_reject_geometry{0};
  size_t fillet_reject_arc_costmap{0};
  size_t fillet_reject_ingress{0};
  size_t fillet_reject_radius_exhausted{0};
};

/// Corner fillets on L1 waypoints: straight legs stay straight; curves honor L1 clearance.
class CurvePathSmoother
{
public:
  /// Drop nearly-collinear interior vertices (reduces L1 graph micro-zigzag before filleting).
  static void simplifyCollinear(
    std::vector<coordsW> & path, double min_turn_angle_rad);

  /// Geometry-only corner fillets (no costmap check). L1 path is already validated.
  /// Two-pass: replace corners on sparse polyline, then uniform densification.
  static bool fitCurvesGeometric(
    std::vector<coordsW> & path,
    const CurvePathSmootherConfig & config,
    const nav2_map_server::DistanceMap * distance_map = nullptr,
    double clearance_radius_factor = 0.85,
    CurveFitStats * stats = nullptr);

  static bool fitCurves(
    std::vector<coordsW> & path,
    const nav2_costmap_2d::Costmap2D * costmap,
    const nav2_map_server::DistanceMap * distance_map,
    bool allow_unknown,
    const CurvePathSmootherConfig & config,
    CurveFitStats * stats = nullptr);

  static double minWaypointClearanceM(
    const std::vector<coordsW> & waypoints,
    const nav2_map_server::DistanceMap * distance_map);

private:
  static double lookupClearanceM(
    double x, double y,
    const nav2_map_server::DistanceMap * distance_map);

  static bool isCornerAt(
    const std::vector<coordsW> & path, size_t i, double corner_angle_rad);

  static bool isPointFreeOnCostmap(
    double x, double y,
    const nav2_costmap_2d::Costmap2D * costmap,
    bool allow_unknown);

  static bool validateSample(
    double x, double y,
    const nav2_costmap_2d::Costmap2D * costmap,
    const nav2_map_server::DistanceMap * distance_map,
    bool allow_unknown,
    double l1_clearance_floor_m);

  static bool validateArcPolyline(
    const std::vector<coordsW> & points,
    const nav2_costmap_2d::Costmap2D * costmap,
    bool allow_unknown);

  static bool validateArcDistanceClearance(
    const std::vector<coordsW> & points,
    const nav2_map_server::DistanceMap * distance_map,
    double corner_clearance_m,
    double radius);

  static bool validatePolylineWithFloor(
    const std::vector<coordsW> & points,
    const nav2_costmap_2d::Costmap2D * costmap,
    const nav2_map_server::DistanceMap * distance_map,
    bool allow_unknown,
    double l1_clearance_floor_m);

  static bool sampleLine(
    const coordsW & a, const coordsW & b, double step, std::vector<coordsW> & out);

  static bool appendLineChecked(
    const coordsW & a, const coordsW & b, double step,
    const nav2_costmap_2d::Costmap2D * costmap,
    const nav2_map_server::DistanceMap * distance_map,
    bool allow_unknown,
    double l1_clearance_floor_m,
    std::vector<coordsW> & out);

  static bool appendLineCostmapOnly(
    const coordsW & a, const coordsW & b, double step,
    const nav2_costmap_2d::Costmap2D * costmap,
    bool allow_unknown,
    std::vector<coordsW> & out);

  static bool tryCornerFillet(
    const coordsW & from,
    const coordsW & a, const coordsW & b, const coordsW & c,
    double step, double max_leg_fraction,
    const nav2_costmap_2d::Costmap2D * costmap,
    const nav2_map_server::DistanceMap * distance_map,
    bool allow_unknown,
    double corner_clearance_m,
    std::vector<coordsW> & out,
    coordsW & exit_point,
    CurveFitStats * reject_stats = nullptr);

  static bool computeCornerFilletGeometric(
    const coordsW & a, const coordsW & b, const coordsW & c,
    double step, double max_leg_fraction,
    const nav2_map_server::DistanceMap * distance_map,
    double clearance_radius_factor,
    coordsW & t1, coordsW & t2,
    std::vector<coordsW> & arc_pts);
};

}  // namespace nav2_theta_star_planner

#endif  // NAV2_THETA_STAR_PLANNER__CURVE_PATH_SMOOTHER_HPP_
