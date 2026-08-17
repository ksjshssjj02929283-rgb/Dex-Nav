// Copyright (c) DexForce
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef NAV2_THETA_STAR_PLANNER__OMNI_PATH_SMOOTHER_HPP_
#define NAV2_THETA_STAR_PLANNER__OMNI_PATH_SMOOTHER_HPP_

#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_theta_star_planner/theta_star.hpp"

namespace nav2_theta_star_planner
{

struct OmniPathSmootherConfig
{
  int max_iterations{1000};
  double w_data{0.2};
  double w_smooth{0.3};
  double tolerance{1e-10};
  bool do_refinement{true};
  int refinement_num{2};
  /// Vertices with turn angle above this (rad) are kept; the point before a corner is also kept.
  double string_pull_corner_angle_rad{0.17};
};

/// Geometry-only path smoother for omnidirectional bases (no curvature / turning-radius limits).
class OmniPathSmoother
{
public:
  /// Greedy line-of-sight shortcutting; never skips corner vertices or their predecessors.
  static bool stringPull(
    std::vector<coordsW> & path,
    const nav2_costmap_2d::Costmap2D * costmap,
    bool allow_unknown,
    const OmniPathSmootherConfig & config);

  /// Bresenham line collision check (same threshold as string pull).
  static bool isLineFree(
    const nav2_costmap_2d::Costmap2D * costmap,
    double x0, double y0, double x1, double y1,
    bool allow_unknown);

  /// Laplacian-style smoothing on dense paths; keeps endpoints fixed and checks costmap collisions.
  static bool gradientSmooth(
    nav_msgs::msg::Path & path,
    const nav2_costmap_2d::Costmap2D * costmap,
    const OmniPathSmootherConfig & config,
    bool allow_unknown);

private:
  static std::vector<bool> markMustKeepVertices(
    const std::vector<coordsW> & path,
    double corner_angle_rad);

  static bool isCellFree(
    const nav2_costmap_2d::Costmap2D * costmap,
    unsigned int mx, unsigned int my,
    bool allow_unknown);

  static bool gradientSmoothImpl(
    nav_msgs::msg::Path & path,
    const nav2_costmap_2d::Costmap2D * costmap,
    const OmniPathSmootherConfig & config,
    bool allow_unknown,
    int & refinement_counter);
};

}  // namespace nav2_theta_star_planner

#endif  // NAV2_THETA_STAR_PLANNER__OMNI_PATH_SMOOTHER_HPP_
