// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// Lexicographic 多代价 DP 规划核心。从 dcm lex_local_planner/lex_planner.h 移植：
//   - costmap_2d::Costmap2D* -> nav2_costmap_2d::Costmap2D*
//   - 删除 ros::Publisher / visualization() rviz 调试方法
//   - DP 算法（buildAdjacencyMatrix / connectAdjacencyMatrix / searchAdjacencyMatrix /
//     edgePropagation / extractPath ...）原样保留
#ifndef NAV2_LEX_LOCAL_PLANNER__LEX_PLANNER_HPP_
#define NAV2_LEX_LOCAL_PLANNER__LEX_PLANNER_HPP_

#include <iostream>
#include <vector>
#include <mutex>
#include <memory>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

#include "nav2_lex_local_planner/data_type.hpp"
#include "nav2_lex_local_planner/rollout_generator.hpp"
#include "nav2_lex_local_planner/planning_params.hpp"

namespace nav2_lex_local_planner
{

class LexPlanner
{
public:
  LexPlanner(nav2_costmap_2d::Costmap2D * costmap,
             std::shared_ptr<nav2_lex_local_planner::PlanningParams> planning_params);
  ~LexPlanner();

  void findPath(
          const std::vector<std::vector<nav2_lex_local_planner::WayPoint>> & alternative_paths,
          std::vector<nav2_lex_local_planner::WayPoint> & final_path);

private:
  void buildAdjacencyMatrix(const std::vector<std::vector<nav2_lex_local_planner::WayPoint>> & alternative_paths);
  void connectAdjacencyMatrix();
  bool searchAdjacencyMatrix();

  bool edgePropagation(state_t * state_from, state_t * state_to, float edgeCosts[NUM_COSTS]);
  bool extractPath(state_t * stateCur);

  std::vector<nav2_lex_local_planner::WayPoint>
  processPath(std::vector<nav2_lex_local_planner::WayPoint> pathIn);

  bool isInCollision(float x, float y);
  float getCloseCollisionCost(float x, float y);
  bool isCloseCollision(float x, float y);

  void updateCosts(state_t * fromState, state_t * toState, int neighborInd);
  state_t * minCostStateInQueue(std::vector<state_t *> Queue);
  float distance(state_t * state_from, state_t * state_to);

private:
  nav2_costmap_2d::Costmap2D * local_costmap_;

  std::vector<nav2_lex_local_planner::WayPoint> final_path_;

  std::vector<state_t *> node_list_;
  std::vector<std::vector<state_t *>> adjacency_matrix_;
  int adjacency_width_grid_;
  int adjacency_length_grid_;

  std::shared_ptr<nav2_lex_local_planner::PlanningParams> planning_params_;

  std::mutex mtx_;
};

}  // namespace nav2_lex_local_planner

#endif  // NAV2_LEX_LOCAL_PLANNER__LEX_PLANNER_HPP_
