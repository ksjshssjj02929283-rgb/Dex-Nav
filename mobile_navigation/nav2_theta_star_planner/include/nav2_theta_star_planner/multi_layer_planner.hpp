// Copyright (c) DexForce
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef NAV2_THETA_STAR_PLANNER__MULTI_LAYER_PLANNER_HPP_
#define NAV2_THETA_STAR_PLANNER__MULTI_LAYER_PLANNER_HPP_

#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_map_server/node_map_types.hpp"
#include "nav2_theta_star_planner/node_theta_star.hpp"

namespace theta_star
{

/// Three-layer sequential planner: global → local → raw (original grid).
class MultiLayerNodePlanner
{
public:
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  nav2_map_server::DistanceMap distance_map_;
  nav2_map_server::NodeMap global_map_;
  nav2_map_server::NodeMap local_map_;
  nav2_map_server::NodeMap raw_map_;

  double w_traversal_cost_{2.0};
  double w_euc_cost_{1.0};
  double w_heuristic_cost_{1.0};
  bool allow_unknown_{false};

  bool hasMaps() const
  {
    return !global_map_.nodes.empty();
  }

  /// @param goal_unreachable_out true when true goal lies in obstacle → path ends at L1 node
  bool generatePath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::vector<coordsW> & raw_path,
    bool & goal_unreachable_out);

private:
  bool isTrueGoalReachable(unsigned int gx, unsigned int gy) const;
  bool planOnLayer(
    const nav2_map_server::NodeMap & layer_map,
    double min_clearance,
    double from_x, double from_y,
    double to_x, double to_y,
    std::vector<coordsW> & segment) const;
  void appendSegment(
    std::vector<coordsW> & combined,
    const std::vector<coordsW> & segment) const;
};

}  // namespace theta_star

#endif  // NAV2_THETA_STAR_PLANNER__MULTI_LAYER_PLANNER_HPP_
