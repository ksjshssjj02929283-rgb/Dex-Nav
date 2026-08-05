// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// 几何/路径工具函数：geometry_msgs::msg::PoseStamped，tf2 等价实现。
#ifndef NAV2_LEX_LOCAL_PLANNER__UTILITY_HPP_
#define NAV2_LEX_LOCAL_PLANNER__UTILITY_HPP_

#include <iostream>
#include <vector>
#include <cmath>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_lex_local_planner/data_type.hpp"

namespace nav2_lex_local_planner
{

#define distance2points(from, to) sqrt(pow(to.x - from.x, 2) + pow(to.y - from.y, 2))
#define distance2points_pow(from, to) pow(to.x - from.x, 2) + pow(to.y - from.y, 2)
#define calLength(v) sqrt(v.x * v.x + v.y * v.y)
#define RAD2DEG_ME 180. / M_PI

int getNextClosePointIndex(const std::vector<WayPoint> & trajectory,
                           const WayPoint & curr_pos,
                           const int & prevIndex = 0);

double calDiffBetweenTwoAngle(const double & a1, const double & a2);

double cast_from_0_to_2PI_Angle(const double & ang);

double cast_from_PI_to_PI_Angle(const double & ang);

double diffBetweenTwoAngle(const double & a1, const double & a2);

bool getRelativeInfo(const std::vector<WayPoint> & trajectory,
                     const WayPoint & p,
                     RelativeInfo & info);

void poseStampedMsgToWayPoint(const geometry_msgs::msg::PoseStamped & pose,
                              nav2_lex_local_planner::WayPoint & wp);

void poseVec2LocalLane(
          const std::vector<geometry_msgs::msg::PoseStamped> & pos_vec,
          std::vector<nav2_lex_local_planner::WayPoint> & path);

void localLane2PoseVec(
          const std::vector<nav2_lex_local_planner::WayPoint> & path,
          std::vector<geometry_msgs::msg::PoseStamped> & pos_vec,
          const std::string & frame_id = "map");

void calculatePathYaw(std::vector<nav2_lex_local_planner::WayPoint> & pathIn);

void fixPathDensity(std::vector<nav2_lex_local_planner::WayPoint> & path,
                    const double & pathDensity);

void smoothPath(std::vector<nav2_lex_local_planner::WayPoint> & path,
                double weight_data, double weight_smooth, double tolerance);

double calcAngleAndCost(std::vector<nav2_lex_local_planner::WayPoint> & path);

}  // namespace nav2_lex_local_planner

#endif  // NAV2_LEX_LOCAL_PLANNER__UTILITY_HPP_
