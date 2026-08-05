// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// Rollout 生成器（横向发散候选轨迹）。核心算法：Run / extractPartFromTrajectory /
// generateRunoffTrajectory / calculateRollInTrajectories。
#ifndef NAV2_LEX_LOCAL_PLANNER__ROLLOUT_GENERATOR_HPP_
#define NAV2_LEX_LOCAL_PLANNER__ROLLOUT_GENERATOR_HPP_

#include <iostream>
#include <vector>
#include <memory>
#include <string>

#include "nav2_lex_local_planner/data_type.hpp"
#include "nav2_lex_local_planner/utility.hpp"
#include "nav2_lex_local_planner/planning_params.hpp"

namespace nav2_lex_local_planner
{

#define LANE_CHANGE_SPEED_FACTOR 0.5

class RolloutGenerator
{
public:
  explicit RolloutGenerator(std::shared_ptr<nav2_lex_local_planner::PlanningParams> planning_params);

  ~RolloutGenerator();

  void Run(const nav2_lex_local_planner::WayPoint curr_pose,
           const double curr_speed,
           std::vector<nav2_lex_local_planner::WayPoint> & path_in,
           std::vector<std::vector<nav2_lex_local_planner::WayPoint>> & alternative_paths);

private:
  void extractPartFromTrajectory(
            const std::vector<nav2_lex_local_planner::WayPoint> & originalPath,
            const nav2_lex_local_planner::WayPoint & currnetPos,
            const double & minDistance,
            const double & waypointDensity,
            std::vector<nav2_lex_local_planner::WayPoint> & extractedPath);

  void generateRunoffTrajectory(const std::vector<std::vector<nav2_lex_local_planner::WayPoint>> & referencePaths,
            const nav2_lex_local_planner::WayPoint & carPos, const double & speed, const double & microPlanDistance,
            const double & carTipMargin, const double & rollInMargin, const double & rollInSpeedFactor,
            const double & pathDensity, const double & rollOutDensity, const int & rollOutNumber,
            const double & SmoothDataWeight, const double & SmoothWeight, const double & SmoothTolerance,
            std::vector<std::vector<std::vector<nav2_lex_local_planner::WayPoint>>> & rollOutsPaths,
            std::vector<nav2_lex_local_planner::WayPoint> & sampledPoints_debug);

  void calculateRollInTrajectories(
            const nav2_lex_local_planner::WayPoint & carPos, const double & speed,
            const std::vector<nav2_lex_local_planner::WayPoint> & originalCenter,
            int & start_index, int & end_index, std::vector<double> & end_laterals,
            std::vector<std::vector<nav2_lex_local_planner::WayPoint>> & rollInPaths, const double & max_roll_distance,
            const double & carTipMargin, const double & rollInMargin, const double & rollInSpeedFactor,
            const double & pathDensity, const double & rollOutDensity, const int & rollOutNumber,
            const double & SmoothDataWeight, const double & SmoothWeight, const double & SmoothTolerance,
            std::vector<nav2_lex_local_planner::WayPoint> & sampledPoints);

private:
  nav2_lex_local_planner::WayPoint current_pose_;
  nav2_lex_local_planner::WayPoint init_pose_;

  std::vector<std::vector<nav2_lex_local_planner::WayPoint>> global_paths_;
  std::vector<nav2_lex_local_planner::WayPoint> central_trajectory_smoothed_;
  std::vector<std::vector<nav2_lex_local_planner::WayPoint>> global_path_sections_;
  std::vector<std::vector<std::vector<nav2_lex_local_planner::WayPoint>>> rollouts_;

  bool currentPose_flag;

  std::shared_ptr<nav2_lex_local_planner::PlanningParams> plannig_params_;
};

}  // namespace nav2_lex_local_planner

#endif  // NAV2_LEX_LOCAL_PLANNER__ROLLOUT_GENERATOR_HPP_
