// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// 从 dcm lex_local_planner/rollout_generator.cpp 移植。rollout 生成算法
// （Run / extractPartFromTrajectory / generateRunoffTrajectory /
//  calculateRollInTrajectories）原样保留；
// 删除 ROS1 publisher/initROS 及仅 rviz 调试的 nav_msgs::Path 辅助函数与
// visualLaneInRviz 调用。
#include <algorithm>
#include <vector>

#include "nav2_lex_local_planner/rollout_generator.hpp"

namespace nav2_lex_local_planner
{

RolloutGenerator::RolloutGenerator(std::shared_ptr<nav2_lex_local_planner::PlanningParams> planning_params)
: plannig_params_(planning_params)
{
  currentPose_flag = false;
}


RolloutGenerator::~RolloutGenerator()
{
}

/**
 * @description: 主循环函数
 */
void RolloutGenerator::Run(
        const nav2_lex_local_planner::WayPoint curr_pose,
        const double curr_speed,
        std::vector<nav2_lex_local_planner::WayPoint> & path_in,
        std::vector<std::vector<nav2_lex_local_planner::WayPoint>> & alternative_paths)
{
  if (path_in.size() == 0) {
    return;
  }

  current_pose_ = curr_pose;
  calcAngleAndCost(path_in);

  global_paths_.clear();
  global_paths_.push_back(path_in);

  if (global_paths_.size() == 0)
    return;

  // 对路径进行截取和重新采样
  global_path_sections_.clear();
  for (size_t i = 0; i < global_paths_.size(); i++) {
    central_trajectory_smoothed_.clear();
    extractPartFromTrajectory(global_paths_[i], current_pose_, plannig_params_->microPlanDistance,
      plannig_params_->pathDensity, central_trajectory_smoothed_);
    global_path_sections_.push_back(central_trajectory_smoothed_);
  }

  // 获取采样后的路径点（rolloutNumber + 1 条路径的点）
  std::vector<nav2_lex_local_planner::WayPoint> sampled_points;
  generateRunoffTrajectory(global_path_sections_,
    current_pose_,
    curr_speed,
    plannig_params_->microPlanDistance,
    plannig_params_->carTipMargin,
    plannig_params_->rollInMargin,
    plannig_params_->speedProfileFactor,
    plannig_params_->pathDensity,
    plannig_params_->rollOutDensity,
    plannig_params_->rollOutNumber,
    plannig_params_->smoothingDataWeight,
    plannig_params_->smoothingSmoothWeight,
    plannig_params_->smoothingToleranceError,
    rollouts_,
    sampled_points);

  alternative_paths = rollouts_[0];
}


/**
 * @brief 生成候选 rollouts
 */
void RolloutGenerator::generateRunoffTrajectory(const std::vector<std::vector<nav2_lex_local_planner::WayPoint>> & referencePaths,
  const nav2_lex_local_planner::WayPoint & carPos,
  const double & speed,
  const double & microPlanDistance,
  const double & carTipMargin,
  const double & rollInMargin,
  const double & rollInSpeedFactor,
  const double & pathDensity,
  const double & rollOutDensity,
  const int & rollOutNumber,
  const double & SmoothDataWeight,
  const double & SmoothWeight,
  const double & SmoothTolerance,
  std::vector<std::vector<std::vector<nav2_lex_local_planner::WayPoint>>> & rollOutsPaths,
  std::vector<nav2_lex_local_planner::WayPoint> & sampledPoints_debug)
{
  if (referencePaths.size() == 0)
    return;
  if (microPlanDistance <= 0)
    return;
  rollOutsPaths.clear();
  sampledPoints_debug.clear();  // for visualization only

  // 对于每条参考路径，都生成一组 rolloutsPath
  for (unsigned int i = 0; i < referencePaths.size(); i++) {
    std::vector<std::vector<nav2_lex_local_planner::WayPoint>> local_rollOutPaths;
    int s_index = 0, e_index = 0;
    std::vector<double> e_distances;

    if (referencePaths.at(i).size() > 0) {
      calculateRollInTrajectories(carPos, speed, referencePaths.at(i), s_index, e_index, e_distances,
        local_rollOutPaths, microPlanDistance, carTipMargin, rollInMargin,
        rollInSpeedFactor, pathDensity, rollOutDensity, rollOutNumber,
        SmoothDataWeight, SmoothWeight, SmoothTolerance, sampledPoints_debug);
    }
    rollOutsPaths.push_back(local_rollOutPaths);
  }
}


/**
 * @brief 由中心轨迹的采样点计算候选 rollouts 的采样点并平滑
 */
void RolloutGenerator::calculateRollInTrajectories(const nav2_lex_local_planner::WayPoint & carPos,
  const double & speed,
  const std::vector<nav2_lex_local_planner::WayPoint> & originalCenter,
  int & start_index,
  int & end_index,
  std::vector<double> & end_laterals,
  std::vector<std::vector<nav2_lex_local_planner::WayPoint>> & rollInPaths,
  const double & max_roll_distance,
  const double & carTipMargin,
  const double & rollInMargin,
  const double & rollInSpeedFactor,
  const double & pathDensity,
  const double & rollOutDensity,
  const int & rollOutNumber,
  const double & SmoothDataWeight,
  const double & SmoothWeight,
  const double & SmoothTolerance,
  std::vector<nav2_lex_local_planner::WayPoint> & sampledPoints)
{
  nav2_lex_local_planner::WayPoint p;

  // Step: 1. 数据准备
  int iLimitIndex = (carTipMargin / 0.3) / pathDensity;
  if (iLimitIndex >= static_cast<int>(originalCenter.size()))
    iLimitIndex = originalCenter.size() - 1;

  // Get Closest Index
  nav2_lex_local_planner::RelativeInfo info;
  nav2_lex_local_planner::getRelativeInfo(originalCenter, carPos, info);

  // 剩余路径的长度
  double remaining_distance = 0;
  int close_index = info.iBack;
  for (unsigned int i = close_index; i < originalCenter.size() - 1; i++) {
    if (i > 0)
      remaining_distance += distance2points(originalCenter[i].pos, originalCenter[i + 1].pos);
  }

  double initial_roll_in_distance = info.perp_distance;

  std::vector<nav2_lex_local_planner::WayPoint> RollOutStratPath;

  // calculate the starting index
  double d_limit = 0;
  unsigned int far_index = close_index;

  // calculate end index
  // 发散的距离
  double start_distance = rollInSpeedFactor * speed + rollInMargin;
  if (start_distance > remaining_distance)
    start_distance = remaining_distance;

  // 得到发散的终点 far_index
  d_limit = 0;
  for (unsigned int i = close_index; i < originalCenter.size(); i++) {
    if (i > 0)
      d_limit += distance2points(originalCenter[i].pos, originalCenter[i - 1].pos);

    if (d_limit >= start_distance) {
      far_index = i;
      break;
    }
  }

  // 横向扩展的点的列表
  // 横向扩展距离 = rollOutNumber / 2 * rollOutDensity
  int centralTrajectoryIndex = rollOutNumber / 2;
  std::vector<double> end_distance_list;
  for (int i = 0; i < rollOutNumber + 1; i++) {
    double end_roll_in_distance = rollOutDensity * (i - centralTrajectoryIndex);
    end_distance_list.push_back(end_roll_in_distance);
  }

  start_index = close_index;
  end_index = far_index;  // end_index 是第二个阶段结尾的点坐标
  end_laterals = end_distance_list;

  // calculate the actual calculation starting index
  d_limit = 0;
  unsigned int smoothing_start_index = start_index;
  unsigned int smoothing_end_index = end_index;

  // 计算 smoothing_start_index
  for (unsigned int i = smoothing_start_index; i < originalCenter.size(); i++) {
    if (i > 0)
      d_limit += distance2points(originalCenter[i].pos, originalCenter[i - 1].pos);
    if (d_limit > carTipMargin)
      break;

    smoothing_start_index++;  // 这个是一个阶段结尾的点下标
  }

  // 计算 smoothing_end_index
  d_limit = 0;
  for (unsigned int i = end_index; i < originalCenter.size(); i++) {
    if (i > 0)
      d_limit += distance2points(originalCenter[i].pos, originalCenter[i - 1].pos);
    if (d_limit > carTipMargin)
      break;

    smoothing_end_index++;
  }

  // steps 是第二阶段的步数
  int nSteps = end_index - smoothing_start_index;

  rollInPaths.clear();
  std::vector<double> inc_list;  // 每个横向点的增量
  std::vector<double> inc_list_inc;
  for (int i = 0; i < rollOutNumber + 1; i++) {
    rollInPaths.push_back(std::vector<nav2_lex_local_planner::WayPoint>());  // 生成 rollOutNumber + 1 个空的路径
    double diff = end_laterals.at(i) - initial_roll_in_distance;
    inc_list.push_back(diff / static_cast<double>(nSteps));
    inc_list_inc.push_back(0);
  }

  // 生成 rollOutNumber + 1 个不需要平滑的路径
  std::vector<std::vector<nav2_lex_local_planner::WayPoint>> execluded_from_smoothing;
  for (int i = 0; i < rollOutNumber + 1; i++)
    execluded_from_smoothing.push_back(std::vector<nav2_lex_local_planner::WayPoint>());

  // Step: 2. 生成 rollInPaths
  // Step: 2.1 每个路径都生成 close ~ smoothing_start_index 的路径点
  // 开始阶段，close index 到 smoothing_start_index
  for (unsigned int j = start_index; j < smoothing_start_index; j++) {
    p = originalCenter.at(j);
    double original_speed = p.v;
    for (int i = 0; i < rollOutNumber + 1; i++) {
      p.pos.x = originalCenter.at(j).pos.x - initial_roll_in_distance * cos(p.pos.yaw + M_PI_2);
      p.pos.y = originalCenter.at(j).pos.y - initial_roll_in_distance * sin(p.pos.yaw + M_PI_2);

      if (i != centralTrajectoryIndex)
        p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
      else
        p.v = original_speed;

      if (static_cast<int>(j) < iLimitIndex)
        execluded_from_smoothing.at(i).push_back(p);
      else
        rollInPaths.at(i).push_back(p);
    }
  }

  // Step: 2.2 每个路径都生成 smoothing_start_index ~ end_index 的路径点
  // 长度为 start_distance = rollInSpeedFactor * speed + rollInMargin
  for (unsigned int j = smoothing_start_index; j < static_cast<unsigned int>(end_index); j++) {
    p = originalCenter.at(j);
    double original_speed = p.v;
    for (int i = 0; i < rollOutNumber + 1; i++) {
      inc_list_inc[i] += inc_list[i];
      double d = inc_list_inc[i];
      p.pos.x = originalCenter.at(j).pos.x - initial_roll_in_distance * cos(p.pos.yaw + M_PI_2) - d * cos(p.pos.yaw + M_PI_2);
      p.pos.y = originalCenter.at(j).pos.y - initial_roll_in_distance * sin(p.pos.yaw + M_PI_2) - d * sin(p.pos.yaw + M_PI_2);

      if (i != centralTrajectoryIndex)
        p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
      else
        p.v = original_speed;

      rollInPaths.at(i).push_back(p);
    }
  }

  // Step: 2.3 每个路径都生成 end_index ~ smoothing_end_index 的路径点
  for (unsigned int j = end_index; j < smoothing_end_index; j++) {
    p = originalCenter.at(j);
    double original_speed = p.v;
    for (int i = 0; i < rollOutNumber + 1; i++) {
      double d = end_laterals.at(i);
      p.pos.x = originalCenter.at(j).pos.x - d * cos(p.pos.yaw + M_PI_2);
      p.pos.y = originalCenter.at(j).pos.y - d * sin(p.pos.yaw + M_PI_2);

      if (i != centralTrajectoryIndex)
        p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
      else
        p.v = original_speed;
      rollInPaths.at(i).push_back(p);
    }
  }

  // Step: 2.4 将 execluded path 插入到 rollInPaths 的开头
  for (int i = 0; i < rollOutNumber + 1; i++) {
    rollInPaths.at(i).insert(rollInPaths.at(i).begin(),
                 execluded_from_smoothing.at(i).begin(),
                 execluded_from_smoothing.at(i).end());
  }


  // Step: 2.5 每个路径生成 smoothing_end_index ~ originalCenter.size() 的路径点
  d_limit = 0;
  for (unsigned int j = smoothing_end_index; j < originalCenter.size(); j++) {
    if (j > 0)
      d_limit += distance2points(originalCenter.at(j).pos, originalCenter.at(j - 1).pos);

    if (d_limit > max_roll_distance)
      break;
    p = originalCenter.at(j);
    double original_speed = p.v;
    for (unsigned int i = 0; i < rollInPaths.size(); i++) {
      double d = end_laterals.at(i);
      p.pos.x = originalCenter.at(j).pos.x - d * cos(p.pos.yaw + M_PI_2);
      p.pos.y = originalCenter.at(j).pos.y - d * sin(p.pos.yaw + M_PI_2);

      if (static_cast<int>(i) != centralTrajectoryIndex)
        p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
      else
        p.v = original_speed;

      rollInPaths.at(i).push_back(p);
    }
  }

  // Step: 3. 平滑路径
  // Step: 3.1 将路径最后的 seg_num_2 (5) 个点设置为原始路径的点
  int path_length = rollInPaths[0].size();

  // the last section
  int seg_num_2 = smoothing_start_index - start_index;
  for (int j = path_length - seg_num_2; j < path_length; j++) {
    int origin_path_index = std::min(static_cast<int>(j + start_index), static_cast<int>(originalCenter.size()) - 1);
    p = originalCenter.at(origin_path_index);
    double original_speed = p.v;
    for (int i = 0; i < rollOutNumber + 1; i++) {
      p.pos.x = originalCenter.at(origin_path_index).pos.x;
      p.pos.y = originalCenter.at(origin_path_index).pos.y;

      if (i != centralTrajectoryIndex)
        p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
      else
        p.v = original_speed;

      rollInPaths[i][j] = p;
    }
  }

  // the second to last section
  // reset offset std::vector
  inc_list.clear();
  inc_list_inc.clear();
  for (int i = 0; i < rollOutNumber + 1; i++) {
    double diff = end_laterals.at(i);
    inc_list.push_back(diff / static_cast<double>(nSteps));
    inc_list_inc.push_back(0);
  }
  for (int j = smoothing_start_index; j < end_index; j++) {
    for (int i = 0; i < rollOutNumber + 1; i++) {
      inc_list_inc[i] += inc_list[i];
    }
  }

  int seg_num_1 = end_index - smoothing_start_index;
  if (seg_num_1 > 0) {
    int j_start = std::max(path_length - seg_num_2 - seg_num_1, static_cast<int>(smoothing_start_index - start_index));
    for (int j = j_start; j < path_length - seg_num_2; j++) {
      int origin_path_index = std::min(static_cast<int>(j + start_index), static_cast<int>(originalCenter.size()) - 1);
      p = originalCenter.at(origin_path_index);
      double original_speed = p.v;
      for (int i = 0; i < rollOutNumber + 1; i++) {
        inc_list_inc[i] -= inc_list[i];
        double d = inc_list_inc[i];
        p.pos.x = originalCenter.at(origin_path_index).pos.x - d * cos(p.pos.yaw + M_PI_2);
        p.pos.y = originalCenter.at(origin_path_index).pos.y - d * sin(p.pos.yaw + M_PI_2);

        if (i != centralTrajectoryIndex)
          p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
        else
          p.v = original_speed;

        rollInPaths[i][j] = p;
      }
    }
  }

  // smooth path
  for (int i = 0; i < rollOutNumber + 1; i++) {
    nav2_lex_local_planner::smoothPath(rollInPaths.at(i), SmoothDataWeight, SmoothWeight, SmoothTolerance);
  }
}

/**
 * @brief 对路径进行截取和重新采样
 */
void RolloutGenerator::extractPartFromTrajectory(const std::vector<nav2_lex_local_planner::WayPoint> & originalPath,
  const nav2_lex_local_planner::WayPoint & currentPos,
  const double & minDistance,
  const double & waypointDensity,
  std::vector<nav2_lex_local_planner::WayPoint> & extractedPath)
{
  if (originalPath.size() < 2)
    return;
  extractedPath.clear();
  // 找到距离当前位置最近的点
  int close_index = nav2_lex_local_planner::getNextClosePointIndex(originalPath, currentPos);

  double dis = 0;
  if (close_index >= static_cast<int>(originalPath.size()) - 1)
    close_index = originalPath.size() - 2;

  // 将 close_index 之前 2m 内的路径点插入到 extractedPath 中
  // close_index -> 0
  for (int i = close_index; i >= 0; i--) {
    extractedPath.insert(extractedPath.begin(), originalPath[i]);
    if (i < static_cast<int>(originalPath.size()))
      dis += hypot(originalPath[i].pos.y - originalPath[i + 1].pos.y, originalPath[i].pos.x - originalPath[i + 1].pos.x);
    if (dis > 2)
      break;
  }
  // 将 close_index+1 之后 minDistance 内的路径点插入到 extractedPath 中
  // close_index -> end
  dis = 0;
  for (int i = close_index + 1; i < static_cast<int>(originalPath.size()); i++) {
    extractedPath.push_back(originalPath[i]);
    if (i > 0)
      dis += hypot(originalPath[i].pos.y - originalPath[i + 1].pos.y, originalPath[i].pos.x - originalPath[i + 1].pos.x);
    if (dis > minDistance)
      break;
  }
  if (extractedPath.size() < 2) {
    std::cout << std::endl
          << "[lex_local_planner] Extracted Rollout Path is too Small, Size = " << extractedPath.size() << std::endl;
    return;
  }
  // 路径进行重新采样
  nav2_lex_local_planner::fixPathDensity(extractedPath, waypointDensity);
  // 计算 extractedPath 的角度和 cost
  calcAngleAndCost(extractedPath);
}

}  // namespace nav2_lex_local_planner
