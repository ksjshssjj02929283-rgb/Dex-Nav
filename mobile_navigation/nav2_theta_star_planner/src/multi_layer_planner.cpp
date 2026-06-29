// Copyright (c) DexForce
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "nav2_theta_star_planner/multi_layer_planner.hpp"

#include <cmath>

namespace theta_star
{

bool MultiLayerNodePlanner::isTrueGoalReachable(unsigned int gx, unsigned int gy) const
{
  if (!raw_map_.grid_node_id.empty()) {
    const int idx = static_cast<int>(gy) * raw_map_.width + static_cast<int>(gx);
    if (idx >= 0 && idx < static_cast<int>(raw_map_.grid_node_id.size())) {
      return raw_map_.grid_node_id[static_cast<size_t>(idx)] >= 0;
    }
    return false;
  }
  if (!costmap_) {
    return false;
  }
  const int cost = costmap_->getCost(gx, gy);
  return (cost == UNKNOWN_COST && allow_unknown_) || cost == FREE_COST;
}

bool MultiLayerNodePlanner::planOnLayer(
  const nav2_map_server::NodeMap & layer_map,
  double min_clearance,
  double from_x, double from_y,
  double to_x, double to_y,
  std::vector<coordsW> & segment) const
{
  segment.clear();
  if (layer_map.nodes.empty()) {
    return false;
  }

  NodeThetaStar planner;
  planner.costmap_ = costmap_;
  planner.distance_map_ = distance_map_;
  planner.w_euc_cost_ = w_euc_cost_;
  planner.w_traversal_cost_ = w_traversal_cost_;
  planner.w_heuristic_cost_ = w_heuristic_cost_;
  planner.allow_unknown_ = allow_unknown_;
  planner.setNodeMap(layer_map, min_clearance);
  planner.setStartAndGoalWorld(from_x, from_y, to_x, to_y);

  if (planner.isUnsafeToPlan()) {
    return false;
  }
  return planner.generatePath(segment);
}

void MultiLayerNodePlanner::appendSegment(
  std::vector<coordsW> & combined,
  const std::vector<coordsW> & segment) const
{
  if (segment.empty()) {
    return;
  }
  for (size_t i = 0; i < segment.size(); ++i) {
    if (!combined.empty()) {
      const auto & last = combined.back();
      const auto & next = segment[i];
      if (std::hypot(last.x - next.x, last.y - next.y) < 1e-3) {
        continue;
      }
    }
    combined.push_back(segment[i]);
  }
}

bool MultiLayerNodePlanner::generatePath(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::vector<coordsW> & raw_path,
  bool & goal_unreachable_out)
{
  raw_path.clear();
  goal_unreachable_out = false;

  if (!hasMaps() || !costmap_) {
    return false;
  }

  const double true_gx = goal.pose.position.x;
  const double true_gy = goal.pose.position.y;
  unsigned int mgx, mgy;
  costmap_->worldToMap(true_gx, true_gy, mgx, mgy);

  const bool goal_reachable = isTrueGoalReachable(mgx, mgy);
  goal_unreachable_out = !goal_reachable;

  // Layer 1: start → sparse global node nearest true goal
  NodeThetaStar finder;
  finder.setNodeMap(global_map_, global_map_.min_clearance_m);
  const int global_goal_id = finder.findNearestNode(static_cast<int>(mgx), static_cast<int>(mgy));
  if (global_goal_id < 0) {
    return false;
  }
  const auto & global_goal_node = global_map_.nodes[static_cast<size_t>(global_goal_id)];

  std::vector<coordsW> seg_global;
  if (!planOnLayer(
      global_map_, global_map_.min_clearance_m,
      start.pose.position.x, start.pose.position.y,
      global_goal_node.x, global_goal_node.y,
      seg_global))
  {
    return false;
  }
  appendSegment(raw_path, seg_global);

  if (!goal_reachable) {
    // Goal in obstacle: stop at L1 sparse node (already path end)
    return !raw_path.empty();
  }

  // Layer 2: global endpoint → local node nearest true goal
  if (!local_map_.nodes.empty()) {
    finder.setNodeMap(local_map_, local_map_.min_clearance_m);
    const int local_goal_id = finder.findNearestNode(static_cast<int>(mgx), static_cast<int>(mgy));

    if (local_goal_id >= 0) {
      const auto & local_goal = local_map_.nodes[static_cast<size_t>(local_goal_id)];
      std::vector<coordsW> seg_local;
      if (planOnLayer(
          local_map_, local_map_.min_clearance_m,
          global_goal_node.x, global_goal_node.y,
          local_goal.x, local_goal.y,
          seg_local))
      {
        appendSegment(raw_path, seg_local);

        // Layer 3: local endpoint → exact true goal on raw/original grid
        if (!raw_map_.nodes.empty()) {
          std::vector<coordsW> seg_raw;
          const auto & local_end = seg_local.empty() ? global_goal_node : local_map_.nodes[
            static_cast<size_t>(local_goal_id)];
          if (planOnLayer(
              raw_map_, raw_map_.min_clearance_m,
              local_end.x, local_end.y,
              true_gx, true_gy,
              seg_raw))
          {
            appendSegment(raw_path, seg_raw);
          } else {
            // Raw segment failed: append true goal if LOS from local end is clear
            coordsW g{true_gx, true_gy};
            raw_path.push_back(g);
          }
        } else {
          coordsW g{true_gx, true_gy};
          raw_path.push_back(g);
        }
      } else {
        // L2 failed: try L3 directly from global node
        std::vector<coordsW> seg_raw;
        if (!raw_map_.nodes.empty() &&
          planOnLayer(raw_map_, raw_map_.min_clearance_m,
          global_goal_node.x, global_goal_node.y, true_gx, true_gy, seg_raw))
        {
          appendSegment(raw_path, seg_raw);
        } else {
          coordsW g{true_gx, true_gy};
          raw_path.push_back(g);
        }
      }
    }
  } else if (!raw_map_.nodes.empty()) {
    std::vector<coordsW> seg_raw;
    if (planOnLayer(
        raw_map_, raw_map_.min_clearance_m,
        global_goal_node.x, global_goal_node.y,
        true_gx, true_gy, seg_raw))
    {
      appendSegment(raw_path, seg_raw);
    } else {
      coordsW g{true_gx, true_gy};
      raw_path.push_back(g);
    }
  } else {
    coordsW g{true_gx, true_gy};
    raw_path.push_back(g);
  }

  return !raw_path.empty();
}

}  // namespace theta_star
