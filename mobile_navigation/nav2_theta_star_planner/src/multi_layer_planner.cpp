// Copyright (c) DexForce
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "nav2_theta_star_planner/multi_layer_planner.hpp"

#include <cmath>

#include "nav2_costmap_2d/cost_values.hpp"

namespace theta_star
{

bool MultiLayerNodePlanner::isTrueGoalReachable(unsigned int gx, unsigned int gy) const
{
  if (!costmap_) {
    return false;
  }
  if (gx >= costmap_->getSizeInCellsX() || gy >= costmap_->getSizeInCellsY()) {
    return false;
  }
  const unsigned char cost = costmap_->getCost(gx, gy);
  if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    return false;
  }
  return cost == nav2_costmap_2d::FREE_SPACE ||
         (cost == nav2_costmap_2d::NO_INFORMATION && allow_unknown_);
}

bool MultiLayerNodePlanner::planOnLayer(
  const nav2_map_server::NodeMap & layer_map,
  double /*min_clearance*/,
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
  // Preprocessed graph edges already encode clearance; skip distance-map LOS in search.
  planner.w_euc_cost_ = w_euc_cost_;
  planner.w_traversal_cost_ = w_traversal_cost_;
  planner.w_heuristic_cost_ = w_heuristic_cost_;
  planner.allow_unknown_ = allow_unknown_;
  planner.max_expansions_ = static_cast<int>(layer_map.nodes.size()) + 64;
  planner.setNodeMap(layer_map, 0.0);
  planner.setStartAndGoalWorld(from_x, from_y, to_x, to_y);

  if (planner.isUnsafeToPlan()) {
    return false;
  }
  return planner.generatePath(segment);
}

bool MultiLayerNodePlanner::planFinalLegOnCostmap(
  double from_x, double from_y,
  double to_x, double to_y,
  std::vector<coordsW> & segment) const
{
  segment.clear();
  const double dist = std::hypot(to_x - from_x, to_y - from_y);
  if (dist < 1e-3) {
    segment.push_back({from_x, from_y});
    return true;
  }

  // Short final approach: straight segment (L1 already routed through sparse nodes).
  if (dist <= kDirectFinalLegMaxDistM) {
    segment.push_back({from_x, from_y});
    segment.push_back({to_x, to_y});
    return true;
  }

  if (!costmap_) {
    return false;
  }

  ThetaStar planner;
  planner.costmap_ = costmap_;
  planner.w_euc_cost_ = w_euc_cost_;
  planner.w_traversal_cost_ = w_traversal_cost_;
  planner.w_heuristic_cost_ = w_heuristic_cost_;
  planner.allow_unknown_ = allow_unknown_;

  geometry_msgs::msg::PoseStamped start_pose;
  geometry_msgs::msg::PoseStamped goal_pose;
  start_pose.pose.position.x = from_x;
  start_pose.pose.position.y = from_y;
  goal_pose.pose.position.x = to_x;
  goal_pose.pose.position.y = to_y;
  planner.setStartAndGoal(start_pose, goal_pose);

  if (planner.isUnsafeToPlan()) {
    return false;
  }
  return planner.generatePath(segment);
}

void MultiLayerNodePlanner::appendFinalApproach(
  std::vector<coordsW> & raw_path,
  double from_x, double from_y,
  double goal_x, double goal_y) const
{
  std::vector<coordsW> seg_final;
  if (planFinalLegOnCostmap(from_x, from_y, goal_x, goal_y, seg_final)) {
    appendSegment(raw_path, seg_final);
    return;
  }
  coordsW g{goal_x, goal_y};
  raw_path.push_back(g);
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

  // L1: sparse global graph (fast graph A*, no per-edge LOS on costmap).
  NodeThetaStar finder;
  finder.setNodeMap(global_map_, 0.0);
  const int global_goal_id = finder.findNearestNodeWorld(true_gx, true_gy);
  if (global_goal_id < 0) {
    return false;
  }
  const auto & global_goal_node = global_map_.nodes[static_cast<size_t>(global_goal_id)];

  std::vector<coordsW> seg_global;
  if (!planOnLayer(
      global_map_, 0.0,
      start.pose.position.x, start.pose.position.y,
      global_goal_node.x, global_goal_node.y,
      seg_global))
  {
    return false;
  }
  appendSegment(raw_path, seg_global);

  if (!goal_reachable) {
    return !raw_path.empty();
  }

  const double leg_from_x = seg_global.empty() ? start.pose.position.x : seg_global.back().x;
  const double leg_from_y = seg_global.empty() ? start.pose.position.y : seg_global.back().y;

  if (kUseLocalLayer && !local_map_.nodes.empty()) {
    finder.setNodeMap(local_map_, 0.0);
    const int local_goal_id = finder.findNearestNodeWorld(true_gx, true_gy);
    if (local_goal_id >= 0) {
      const auto & local_goal = local_map_.nodes[static_cast<size_t>(local_goal_id)];
      std::vector<coordsW> seg_local;
      if (planOnLayer(
          local_map_, 0.0,
          global_goal_node.x, global_goal_node.y,
          local_goal.x, local_goal.y,
          seg_local))
      {
        appendSegment(raw_path, seg_local);
        const double lx = seg_local.empty() ? leg_from_x : seg_local.back().x;
        const double ly = seg_local.empty() ? leg_from_y : seg_local.back().y;
        appendFinalApproach(raw_path, lx, ly, true_gx, true_gy);
        return !raw_path.empty();
      }
    }
  }

  // Default: L1 global waypoints + short final leg (line or costmap Theta*).
  appendFinalApproach(raw_path, leg_from_x, leg_from_y, true_gx, true_gy);
  return !raw_path.empty();
}

}  // namespace theta_star
