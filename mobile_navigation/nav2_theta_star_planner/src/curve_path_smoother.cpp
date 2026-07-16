// Copyright (c) DexForce
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "nav2_theta_star_planner/curve_path_smoother.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_theta_star_planner/omni_path_smoother.hpp"

namespace nav2_theta_star_planner
{

namespace
{
constexpr double kEps = 1e-6;

double dist(const coordsW & a, const coordsW & b)
{
  return std::hypot(b.x - a.x, b.y - a.y);
}

void appendUnique(std::vector<coordsW> & out, const coordsW & p)
{
  if (out.empty() || dist(out.back(), p) > kEps) {
    out.push_back(p);
  }
}

coordsW lerp(const coordsW & a, const coordsW & b, double t)
{
  return {a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};
}

bool normalize2(double & x, double & y)
{
  const double len = std::hypot(x, y);
  if (len < kEps) {
    return false;
  }
  x /= len;
  y /= len;
  return true;
}

double signedTurnRad(double v1x, double v1y, double v2x, double v2y)
{
  return std::atan2(v1x * v2y - v1y * v2x, v1x * v2x + v1y * v2y);
}

/// Tangent-continuous circular fillet: arc tangents match incoming/outgoing straight legs.
bool buildTangentFilletArc(
  const coordsW & b,
  double v_in_x, double v_in_y,
  double v_out_x, double v_out_y,
  double turn_rad, double radius, double step,
  std::vector<coordsW> & arc_pts,
  coordsW & t1, coordsW & t2)
{
  const double half_turn = std::abs(turn_rad) * 0.5;
  if (half_turn < 1e-4) {
    return false;
  }

  const double trim = radius / std::tan(half_turn);
  t1 = {b.x - v_in_x * trim, b.y - v_in_y * trim};
  t2 = {b.x + v_out_x * trim, b.y + v_out_y * trim};

  // Inward normal from incoming leg toward arc center.
  double nx = -v_in_y;
  double ny = v_in_x;
  if (turn_rad < 0.0) {
    nx = -nx;
    ny = -ny;
  }

  const coordsW center = {t1.x + nx * radius, t1.y + ny * radius};
  (void)center;
  // Skip redundant center-radius sanity check (float + small-R false rejects).

  const double a0 = std::atan2(t1.y - center.y, t1.x - center.x);
  const double a1 = std::atan2(t2.y - center.y, t2.x - center.x);
  double sweep = a1 - a0;
  if (turn_rad > 0.0 && sweep < 0.0) {
    sweep += 2.0 * M_PI;
  } else if (turn_rad < 0.0 && sweep > 0.0) {
    sweep -= 2.0 * M_PI;
  }
  if (std::abs(sweep) > M_PI) {
    sweep -= (turn_rad > 0.0 ? 2.0 * M_PI : -2.0 * M_PI);
  }

  const double arc_len = std::abs(sweep) * radius;
  const int samples = std::max(2, static_cast<int>(arc_len / step));
  arc_pts.clear();
  arc_pts.reserve(static_cast<size_t>(samples + 1));
  for (int i = 0; i <= samples; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(samples);
    const double ang = a0 + sweep * t;
    arc_pts.push_back(
      {center.x + radius * std::cos(ang), center.y + radius * std::sin(ang)});
  }
  return true;
}
}  // namespace

void CurvePathSmoother::simplifyCollinear(
  std::vector<coordsW> & path, double min_turn_angle_rad)
{
  if (path.size() < 3) {
    return;
  }

  std::vector<coordsW> out;
  out.reserve(path.size());
  out.push_back(path.front());
  for (size_t i = 1; i + 1 < path.size(); ++i) {
    if (isCornerAt(path, i, min_turn_angle_rad)) {
      if (out.empty() ||
        std::hypot(path[i].x - out.back().x, path[i].y - out.back().y) > kEps)
      {
        out.push_back(path[i]);
      }
    }
  }
  if (out.empty() ||
    std::hypot(path.back().x - out.back().x, path.back().y - out.back().y) > kEps)
  {
    out.push_back(path.back());
  }
  path.swap(out);
}

double CurvePathSmoother::lookupClearanceM(
  double x, double y,
  const nav2_map_server::DistanceMap * distance_map)
{
  if (!distance_map || distance_map->distance_m.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  const int gx = static_cast<int>(std::floor((x - distance_map->origin_x) /
    distance_map->resolution));
  const int gy = static_cast<int>(std::floor((y - distance_map->origin_y) /
    distance_map->resolution));
  if (gx < 0 || gy < 0 || gx >= distance_map->width || gy >= distance_map->height) {
    return 0.0;
  }
  return static_cast<double>(
    distance_map->distance_m[static_cast<size_t>(gy * distance_map->width + gx)]);
}

double CurvePathSmoother::minWaypointClearanceM(
  const std::vector<coordsW> & waypoints,
  const nav2_map_server::DistanceMap * distance_map)
{
  double min_c = std::numeric_limits<double>::infinity();
  for (const auto & p : waypoints) {
    min_c = std::min(min_c, lookupClearanceM(p.x, p.y, distance_map));
  }
  return min_c;
}

bool CurvePathSmoother::isCornerAt(
  const std::vector<coordsW> & path, size_t i, double corner_angle_rad)
{
  if (i == 0 || i + 1 >= path.size()) {
    return false;
  }
  const double v1x = path[i].x - path[i - 1].x;
  const double v1y = path[i].y - path[i - 1].y;
  const double v2x = path[i + 1].x - path[i].x;
  const double v2y = path[i + 1].y - path[i].y;
  const double len1 = std::hypot(v1x, v1y);
  const double len2 = std::hypot(v2x, v2y);
  if (len1 < kEps || len2 < kEps) {
    return false;
  }
  const double turn = signedTurnRad(v1x / len1, v1y / len1, v2x / len2, v2y / len2);
  return std::abs(turn) > corner_angle_rad;
}

bool CurvePathSmoother::isPointFreeOnCostmap(
  double x, double y,
  const nav2_costmap_2d::Costmap2D * costmap,
  bool allow_unknown)
{
  if (!costmap) {
    return false;
  }
  unsigned int mx, my;
  if (!costmap->worldToMap(x, y, mx, my)) {
    return false;
  }
  const unsigned char cost = costmap->getCost(mx, my);
  if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    return false;
  }
  if (cost == nav2_costmap_2d::NO_INFORMATION && !allow_unknown) {
    return false;
  }
  return true;
}

bool CurvePathSmoother::validateSample(
  double x, double y,
  const nav2_costmap_2d::Costmap2D * costmap,
  const nav2_map_server::DistanceMap * distance_map,
  bool allow_unknown,
  double l1_clearance_floor_m)
{
  if (!isPointFreeOnCostmap(x, y, costmap, allow_unknown)) {
    return false;
  }
  if (distance_map && !distance_map->distance_m.empty()) {
    const double clearance = lookupClearanceM(x, y, distance_map);
    if (clearance + 1e-4 < l1_clearance_floor_m) {
      return false;
    }
  }
  return true;
}

bool CurvePathSmoother::validateArcPolyline(
  const std::vector<coordsW> & points,
  const nav2_costmap_2d::Costmap2D * costmap,
  bool allow_unknown)
{
  if (points.size() < 2) {
    return points.empty() ||
           isPointFreeOnCostmap(points.front().x, points.front().y, costmap, allow_unknown);
  }
  for (size_t i = 1; i < points.size(); ++i) {
    if (!OmniPathSmoother::isLineFree(
        costmap, points[i - 1].x, points[i - 1].y, points[i].x, points[i].y, allow_unknown))
    {
      return false;
    }
  }
  return true;
}

bool CurvePathSmoother::validateArcDistanceClearance(
  const std::vector<coordsW> & points,
  const nav2_map_server::DistanceMap * distance_map,
  double corner_clearance_m,
  double radius)
{
  if (!distance_map || distance_map->distance_m.empty()) {
    return true;
  }
  const double floor = std::max(
    0.08,
    corner_clearance_m - radius * 1.25);
  for (const auto & p : points) {
    if (lookupClearanceM(p.x, p.y, distance_map) + 1e-4 < floor) {
      return false;
    }
  }
  return true;
}

bool CurvePathSmoother::validatePolylineWithFloor(
  const std::vector<coordsW> & points,
  const nav2_costmap_2d::Costmap2D * costmap,
  const nav2_map_server::DistanceMap * distance_map,
  bool allow_unknown,
  double l1_clearance_floor_m)
{
  for (const auto & p : points) {
    if (!validateSample(
        p.x, p.y, costmap, distance_map, allow_unknown, l1_clearance_floor_m))
    {
      return false;
    }
  }
  return true;
}

bool CurvePathSmoother::sampleLine(
  const coordsW & a, const coordsW & b, double step, std::vector<coordsW> & out)
{
  const double length = dist(a, b);
  appendUnique(out, a);
  if (length < kEps) {
    return true;
  }
  const int loops = std::max(1, static_cast<int>(length / step));
  for (int i = 1; i <= loops; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(loops);
    appendUnique(out, lerp(a, b, t));
  }
  return true;
}

bool CurvePathSmoother::appendLineChecked(
  const coordsW & a, const coordsW & b, double step,
  const nav2_costmap_2d::Costmap2D * costmap,
  const nav2_map_server::DistanceMap * distance_map,
  bool allow_unknown,
  double l1_clearance_floor_m,
  std::vector<coordsW> & out)
{
  const double length = dist(a, b);
  if (length < kEps) {
    appendUnique(out, b);
    return true;
  }

  const int loops = std::max(1, static_cast<int>(length / step));
  for (int i = 0; i <= loops; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(loops);
    const coordsW p = (i == 0) ? a : lerp(a, b, t);
    if (i > 0 && !validateSample(
        p.x, p.y, costmap, distance_map, allow_unknown, l1_clearance_floor_m))
    {
      return false;
    }
    appendUnique(out, p);
  }
  return true;
}

bool CurvePathSmoother::appendLineCostmapOnly(
  const coordsW & a, const coordsW & b, double step,
  const nav2_costmap_2d::Costmap2D * costmap,
  bool allow_unknown,
  std::vector<coordsW> & out)
{
  if (!OmniPathSmoother::isLineFree(costmap, a.x, a.y, b.x, b.y, allow_unknown)) {
    return false;
  }
  return sampleLine(a, b, step, out);
}

bool CurvePathSmoother::tryCornerFillet(
  const coordsW & from,
  const coordsW & a, const coordsW & b, const coordsW & c,
  double step, double max_leg_fraction,
  const nav2_costmap_2d::Costmap2D * costmap,
  const nav2_map_server::DistanceMap * distance_map,
  bool allow_unknown,
  double corner_clearance_m,
  std::vector<coordsW> & out,
  coordsW & exit_point,
  CurveFitStats * reject_stats)
{
  const double leg_ab = dist(a, b);
  const double leg_bc = dist(b, c);
  if (leg_ab < kEps || leg_bc < kEps) {
    return false;
  }

  double v_in_x = b.x - a.x;
  double v_in_y = b.y - a.y;
  double v_out_x = c.x - b.x;
  double v_out_y = c.y - b.y;
  if (!normalize2(v_in_x, v_in_y) || !normalize2(v_out_x, v_out_y)) {
    return false;
  }

  const double turn = signedTurnRad(v_in_x, v_in_y, v_out_x, v_out_y);
  if (std::abs(turn) < 1e-3) {
    return false;
  }

  const double max_trim = std::min(leg_ab, leg_bc) *
    std::clamp(max_leg_fraction, 0.05, 0.65);
  const double half_turn = std::abs(turn) * 0.5;
  const double tan_half = std::tan(half_turn);
  const double max_radius = max_trim * tan_half;
  const double min_radius = std::max(step * 0.5, 0.015);
  const bool use_distance_arc = distance_map && !distance_map->distance_m.empty();

  enum class FailReason
  {
    None,
    Geometry,
    ArcCostmap,
    IngressLine,
    RadiusExhausted
  };
  FailReason last_fail = FailReason::RadiusExhausted;

  if (max_radius < min_radius) {
    if (reject_stats) {
      ++reject_stats->fillet_reject_radius_exhausted;
    }
    return false;
  }

  for (double radius = max_radius; radius >= min_radius; radius *= 0.75) {
    const double trim = radius / tan_half;
    if (trim > max_trim + kEps) {
      continue;
    }

    coordsW t1{};
    coordsW t2{};
    std::vector<coordsW> arc_pts;
    if (!buildTangentFilletArc(
        b, v_in_x, v_in_y, v_out_x, v_out_y, turn, radius, step, arc_pts, t1, t2))
    {
      last_fail = FailReason::Geometry;
      continue;
    }

    const bool arc_ok = use_distance_arc ?
      validateArcDistanceClearance(arc_pts, distance_map, corner_clearance_m, radius) :
      validateArcPolyline(arc_pts, costmap, allow_unknown);
    if (!arc_ok) {
      last_fail = FailReason::ArcCostmap;
      continue;
    }

    const size_t out_mark = out.size();
    if (!OmniPathSmoother::isLineFree(
        costmap, from.x, from.y, t1.x, t1.y, allow_unknown))
    {
      last_fail = FailReason::IngressLine;
      continue;
    }
    if (!appendLineCostmapOnly(from, t1, step, costmap, allow_unknown, out)) {
      out.resize(out_mark);
      last_fail = FailReason::IngressLine;
      continue;
    }

    for (size_t k = 1; k + 1 < arc_pts.size(); ++k) {
      appendUnique(out, arc_pts[k]);
    }
    appendUnique(out, t2);
    exit_point = t2;
    return true;
  }

  if (reject_stats) {
    switch (last_fail) {
      case FailReason::Geometry:
        ++reject_stats->fillet_reject_geometry;
        break;
      case FailReason::ArcCostmap:
        ++reject_stats->fillet_reject_arc_costmap;
        break;
      case FailReason::IngressLine:
        ++reject_stats->fillet_reject_ingress;
        break;
      case FailReason::RadiusExhausted:
      default:
        ++reject_stats->fillet_reject_radius_exhausted;
        break;
    }
  }
  return false;
}

bool CurvePathSmoother::computeCornerFilletGeometric(
  const coordsW & a, const coordsW & b, const coordsW & c,
  double step, double max_leg_fraction,
  const nav2_map_server::DistanceMap * distance_map,
  double clearance_radius_factor,
  coordsW & t1, coordsW & t2,
  std::vector<coordsW> & arc_pts)
{
  const double leg_ab = dist(a, b);
  const double leg_bc = dist(b, c);
  if (leg_ab < kEps || leg_bc < kEps) {
    return false;
  }

  double v_in_x = b.x - a.x;
  double v_in_y = b.y - a.y;
  double v_out_x = c.x - b.x;
  double v_out_y = c.y - b.y;
  if (!normalize2(v_in_x, v_in_y) || !normalize2(v_out_x, v_out_y)) {
    return false;
  }

  const double turn = signedTurnRad(v_in_x, v_in_y, v_out_x, v_out_y);
  if (std::abs(turn) < 1e-3) {
    return false;
  }

  const double max_trim = std::min(leg_ab, leg_bc) *
    std::clamp(max_leg_fraction, 0.05, 0.65);
  const double half_turn = std::abs(turn) * 0.5;
  const double tan_half = std::tan(half_turn);
  const double max_radius = max_trim * tan_half;
  const double min_radius = std::max(step * 0.5, 0.01);

  double clear_cap = std::numeric_limits<double>::infinity();
  if (distance_map && !distance_map->distance_m.empty()) {
    const double clear_b = lookupClearanceM(b.x, b.y, distance_map);
    if (std::isfinite(clear_b) && clear_b > kEps) {
      clear_cap = clear_b * tan_half * std::clamp(clearance_radius_factor, 0.3, 1.0);
    }
  }

  for (double radius = std::min(max_radius, clear_cap);
    radius >= min_radius;
    radius *= 0.75)
  {
    const double trim = radius / tan_half;
    if (trim > max_trim + kEps) {
      continue;
    }

    if (!buildTangentFilletArc(
        b, v_in_x, v_in_y, v_out_x, v_out_y, turn, radius, step, arc_pts, t1, t2))
    {
      continue;
    }
    return true;
  }

  return false;
}

bool CurvePathSmoother::fitCurvesGeometric(
  std::vector<coordsW> & path,
  const CurvePathSmootherConfig & config,
  const nav2_map_server::DistanceMap * distance_map,
  double clearance_radius_factor,
  CurveFitStats * stats)
{
  if (path.size() < 2) {
    return true;
  }

  CurveFitStats local_stats;
  const size_t n = path.size();
  std::vector<coordsW> sparse;
  sparse.reserve(path.size() * 2);
  sparse.push_back(path.front());

  size_t i = 0;
  while (i < n - 1) {
    if (i + 2 < n && isCornerAt(path, i + 1, config.corner_angle_rad)) {
      ++local_stats.fillet_corners_seen;
      coordsW t1{};
      coordsW t2{};
      std::vector<coordsW> arc_pts;
      if (computeCornerFilletGeometric(
          path[i], path[i + 1], path[i + 2],
          config.sample_step_m, config.fillet_max_leg_fraction,
          distance_map, clearance_radius_factor, t1, t2, arc_pts))
      {
        ++local_stats.arc_segments;
        ++local_stats.fillet_corners_applied;
        appendUnique(sparse, t1);
        for (size_t k = 1; k + 1 < arc_pts.size(); ++k) {
          appendUnique(sparse, arc_pts[k]);
        }
        appendUnique(sparse, t2);
        ++i;
        continue;
      }
      ++local_stats.fillet_corners_rejected;
    }

    appendUnique(sparse, path[i + 1]);
    ++local_stats.line_segments;
    ++i;
  }

  std::vector<coordsW> dense;
  dense.reserve(sparse.size() * 4);
  dense.push_back(sparse.front());
  for (size_t j = 0; j + 1 < sparse.size(); ++j) {
    sampleLine(dense.back(), sparse[j + 1], config.sample_step_m, dense);
  }

  if (dense.size() >= 2) {
    path.swap(dense);
  }

  if (stats) {
    *stats = local_stats;
  }
  return true;
}

bool CurvePathSmoother::fitCurves(
  std::vector<coordsW> & path,
  const nav2_costmap_2d::Costmap2D * costmap,
  const nav2_map_server::DistanceMap * distance_map,
  bool allow_unknown,
  const CurvePathSmootherConfig & config,
  CurveFitStats * stats)
{
  if (path.size() < 2 || !costmap) {
    return true;
  }

  CurveFitStats local_stats;
  const bool has_distance = distance_map && !distance_map->distance_m.empty();
  const double global_floor = has_distance ?
    minWaypointClearanceM(path, distance_map) :
    std::numeric_limits<double>::infinity();

  std::vector<coordsW> fitted;
  fitted.reserve(path.size() * 4);
  appendUnique(fitted, path.front());

  size_t i = 0;
  while (i < path.size() - 1) {
    if (i + 2 < path.size() &&
      isCornerAt(path, i + 1, config.corner_angle_rad))
    {
      ++local_stats.fillet_corners_seen;
      const coordsW & a = path[i];
      const coordsW & b = path[i + 1];
      const coordsW & c = path[i + 2];
      const double clear_b = has_distance ?
        lookupClearanceM(b.x, b.y, distance_map) :
        std::numeric_limits<double>::infinity();

      coordsW exit_pt{};
      const size_t before = fitted.size();
      if (tryCornerFillet(
          fitted.back(), a, b, c, config.sample_step_m, config.fillet_max_leg_fraction,
          costmap, distance_map, allow_unknown, clear_b, fitted, exit_pt, &local_stats))
      {
        ++local_stats.arc_segments;
        ++local_stats.fillet_corners_applied;
        if (!appendLineCostmapOnly(
            exit_pt, c, config.sample_step_m, costmap, allow_unknown, fitted))
        {
          fitted.resize(before);
          appendLineCostmapOnly(
            fitted.back(), b, config.sample_step_m, costmap, allow_unknown, fitted);
          appendLineCostmapOnly(
            b, c, config.sample_step_m, costmap, allow_unknown, fitted);
          ++local_stats.fallback_segments;
        } else {
          ++local_stats.line_segments;
        }
        i += 2;
        continue;
      }

      ++local_stats.fillet_corners_rejected;
    }

    const coordsW & p1 = path[i + 1];
    const double floor = has_distance ?
      std::min(global_floor, minWaypointClearanceM({path[i], p1}, distance_map)) :
      global_floor;
    if (!appendLineChecked(
        fitted.back(), p1, config.sample_step_m, costmap, distance_map, allow_unknown,
        floor, fitted))
    {
      appendUnique(fitted, p1);
      ++local_stats.fallback_segments;
    } else {
      ++local_stats.line_segments;
    }
    ++i;
  }

  if (fitted.size() >= 2) {
    path.swap(fitted);
  }

  if (stats) {
    *stats = local_stats;
  }
  return true;
}

}  // namespace nav2_theta_star_planner
