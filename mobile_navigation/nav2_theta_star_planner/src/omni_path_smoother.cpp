// Copyright (c) DexForce
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "nav2_theta_star_planner/omni_path_smoother.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "nav2_costmap_2d/cost_values.hpp"

namespace nav2_theta_star_planner
{

namespace
{
}  // namespace

bool OmniPathSmoother::isCellFree(
  const nav2_costmap_2d::Costmap2D * costmap,
  unsigned int mx, unsigned int my,
  bool allow_unknown)
{
  if (!costmap || mx >= costmap->getSizeInCellsX() || my >= costmap->getSizeInCellsY()) {
    return false;
  }
  const unsigned char cost = costmap->getCost(mx, my);
  if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    return false;
  }
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return allow_unknown;
  }
  return true;
}

bool OmniPathSmoother::isLineFree(
  const nav2_costmap_2d::Costmap2D * costmap,
  double x0, double y0, double x1, double y1,
  bool allow_unknown)
{
  if (!costmap) {
    return false;
  }

  unsigned int mx0, my0, mx1, my1;
  if (!costmap->worldToMap(x0, y0, mx0, my0) || !costmap->worldToMap(x1, y1, mx1, my1)) {
    return false;
  }

  int cx = static_cast<int>(mx0);
  int cy = static_cast<int>(my0);
  const int x1i = static_cast<int>(mx1);
  const int y1i = static_cast<int>(my1);
  const int dx = std::abs(x1i - cx);
  const int dy = std::abs(y1i - cy);
  const int sx = x1i > cx ? 1 : -1;
  const int sy = y1i > cy ? 1 : -1;
  const int u_x = (sx - 1) / 2;
  const int u_y = (sy - 1) / 2;
  int f = 0;

  if (!isCellFree(costmap, static_cast<unsigned int>(cx), static_cast<unsigned int>(cy), allow_unknown)) {
    return false;
  }

  if (dx >= dy) {
    while (cx != x1i) {
      f += dy;
      if (f >= dx) {
        if (!isCellFree(
            costmap,
            static_cast<unsigned int>(cx + u_x),
            static_cast<unsigned int>(cy + u_y),
            allow_unknown))
        {
          return false;
        }
        cy += sy;
        f -= dx;
      }
      if (f != 0 && !isCellFree(
          costmap,
          static_cast<unsigned int>(cx + u_x),
          static_cast<unsigned int>(cy + u_y),
          allow_unknown))
      {
        return false;
      }
      if (dy == 0 &&
        !isCellFree(costmap, static_cast<unsigned int>(cx + u_x), static_cast<unsigned int>(cy), allow_unknown) &&
        !isCellFree(
          costmap,
          static_cast<unsigned int>(cx + u_x),
          static_cast<unsigned int>(cy - 1),
          allow_unknown))
      {
        return false;
      }
      cx += sx;
    }
  } else {
    while (cy != y1i) {
      f += dx;
      if (f >= dy) {
        if (!isCellFree(
            costmap,
            static_cast<unsigned int>(cx + u_x),
            static_cast<unsigned int>(cy + u_y),
            allow_unknown))
        {
          return false;
        }
        cx += sx;
        f -= dy;
      }
      if (f != 0 && !isCellFree(
          costmap,
          static_cast<unsigned int>(cx + u_x),
          static_cast<unsigned int>(cy + u_y),
          allow_unknown))
      {
        return false;
      }
      if (dx == 0 &&
        !isCellFree(costmap, static_cast<unsigned int>(cx), static_cast<unsigned int>(cy + u_y), allow_unknown) &&
        !isCellFree(
          costmap,
          static_cast<unsigned int>(cx - 1),
          static_cast<unsigned int>(cy + u_y),
          allow_unknown))
      {
        return false;
      }
      cy += sy;
    }
  }

  return isCellFree(costmap, static_cast<unsigned int>(x1i), static_cast<unsigned int>(y1i), allow_unknown);
}

std::vector<bool> OmniPathSmoother::markMustKeepVertices(
  const std::vector<coordsW> & path,
  double corner_angle_rad)
{
  const size_t n = path.size();
  std::vector<bool> must_keep(n, false);
  if (n == 0) {
    return must_keep;
  }

  must_keep.front() = true;
  must_keep.back() = true;
  if (n < 3) {
    return must_keep;
  }

  std::vector<bool> is_corner(n, false);
  for (size_t i = 1; i + 1 < n; ++i) {
    const double v1x = path[i].x - path[i - 1].x;
    const double v1y = path[i].y - path[i - 1].y;
    const double v2x = path[i + 1].x - path[i].x;
    const double v2y = path[i + 1].y - path[i].y;
    const double len1 = std::hypot(v1x, v1y);
    const double len2 = std::hypot(v2x, v2y);
    if (len1 < 1e-6 || len2 < 1e-6) {
      is_corner[i] = true;
      continue;
    }
    const double dot = (v1x * v2x + v1y * v2y) / (len1 * len2);
    const double turn_angle = std::acos(std::clamp(dot, -1.0, 1.0));
    is_corner[i] = turn_angle > corner_angle_rad;
  }

  for (size_t i = 0; i < n; ++i) {
    if (is_corner[i]) {
      must_keep[i] = true;
    }
    // Keep the vertex before a corner so shortcuts do not cut into the turn.
    if (i + 1 < n && is_corner[i + 1]) {
      must_keep[i] = true;
    }
  }

  return must_keep;
}

bool OmniPathSmoother::stringPull(
  std::vector<coordsW> & path,
  const nav2_costmap_2d::Costmap2D * costmap,
  bool allow_unknown,
  const OmniPathSmootherConfig & config)
{
  if (path.size() < 3 || !costmap) {
    return true;
  }

  const std::vector<bool> must_keep = markMustKeepVertices(
    path, config.string_pull_corner_angle_rad);

  std::vector<coordsW> shortened;
  shortened.reserve(path.size());
  shortened.push_back(path.front());

  size_t anchor = 0;
  while (anchor < path.size() - 1) {
    size_t furthest = anchor + 1;
    for (size_t candidate = path.size() - 1; candidate > anchor + 1; --candidate) {
      if (!isLineFree(
          costmap,
          path[anchor].x, path[anchor].y,
          path[candidate].x, path[candidate].y,
          allow_unknown))
      {
        continue;
      }

      bool skips_required = false;
      for (size_t k = anchor + 1; k < candidate; ++k) {
        if (must_keep[k]) {
          skips_required = true;
          break;
        }
      }
      if (skips_required) {
        continue;
      }

      furthest = candidate;
      break;
    }

    anchor = furthest;
    if (anchor < path.size() - 1) {
      shortened.push_back(path[anchor]);
    }
  }

  if (shortened.back().x != path.back().x || shortened.back().y != path.back().y) {
    shortened.push_back(path.back());
  }

  path.swap(shortened);
  return true;
}

bool OmniPathSmoother::gradientSmoothImpl(
  nav_msgs::msg::Path & path,
  const nav2_costmap_2d::Costmap2D * costmap,
  const OmniPathSmootherConfig & config,
  bool allow_unknown,
  int & refinement_counter)
{
  const unsigned int path_size = path.poses.size();
  if (path_size < 3) {
    return true;
  }

  nav_msgs::msg::Path working = path;
  int iterations = 0;
  double change = config.tolerance;

  while (change >= config.tolerance) {
    ++iterations;
    change = 0.0;

    if (iterations >= config.max_iterations) {
      path = working;
      return false;
    }

    for (unsigned int i = 1; i + 1 < path_size; ++i) {
      const double orig_x = path.poses[i].pose.position.x;
      const double orig_y = path.poses[i].pose.position.y;
      const double prev_x = working.poses[i - 1].pose.position.x;
      const double prev_y = working.poses[i - 1].pose.position.y;
      const double next_x = working.poses[i + 1].pose.position.x;
      const double next_y = working.poses[i + 1].pose.position.y;
      const double before_x = working.poses[i].pose.position.x;
      const double before_y = working.poses[i].pose.position.y;

      double sx = before_x + config.w_data * (orig_x - before_x) +
        config.w_smooth * (prev_x + next_x - (2.0 * before_x));
      double sy = before_y + config.w_data * (orig_y - before_y) +
        config.w_smooth * (prev_y + next_y - (2.0 * before_y));

      unsigned int mx, my;
      if (!costmap ||
        !costmap->worldToMap(sx, sy, mx, my) ||
        isCellFree(costmap, mx, my, allow_unknown))
      {
        working.poses[i].pose.position.x = sx;
        working.poses[i].pose.position.y = sy;
        change += std::hypot(sx - before_x, sy - before_y);
      }
    }
  }

  if (config.do_refinement && refinement_counter < config.refinement_num) {
    ++refinement_counter;
    gradientSmoothImpl(working, costmap, config, allow_unknown, refinement_counter);
  }

  path = working;
  return true;
}

bool OmniPathSmoother::gradientSmooth(
  nav_msgs::msg::Path & path,
  const nav2_costmap_2d::Costmap2D * costmap,
  const OmniPathSmootherConfig & config,
  bool allow_unknown)
{
  if (path.poses.size() < 3) {
    return true;
  }

  int refinement_counter = 0;
  return gradientSmoothImpl(path, costmap, config, allow_unknown, refinement_counter);
}

}  // namespace nav2_theta_star_planner
