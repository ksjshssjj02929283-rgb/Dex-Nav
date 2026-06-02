// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// 从 dcm lex_local_planner/lex_planner.cpp 移植。Lexicographic 多代价 DP
// （邻接矩阵 / Dijkstra 分层 cost 比较 / 提取路径）原样保留；
// ROS 边界替换：costmap_2d::Costmap2D -> nav2_costmap_2d::Costmap2D；
// costmap_2d::{LETHAL_OBSTACLE,...} -> nav2_costmap_2d::；ros::ok() -> rclcpp::ok()；
// 删除 ros::Publisher 与 visualization() rviz 调试方法。
#include <algorithm>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "nav2_lex_local_planner/utility.hpp"
#include "nav2_lex_local_planner/lex_planner.hpp"

namespace nav2_lex_local_planner
{

LexPlanner::LexPlanner(
        nav2_costmap_2d::Costmap2D * costmap,
        std::shared_ptr<nav2_lex_local_planner::PlanningParams> planning_params)
: local_costmap_(costmap),
  adjacency_width_grid_(0),
  adjacency_length_grid_(0),
  planning_params_(planning_params)
{
}

LexPlanner::~LexPlanner()
{
}

void LexPlanner::findPath(
        const std::vector<std::vector<nav2_lex_local_planner::WayPoint>> & alternative_paths,
        std::vector<nav2_lex_local_planner::WayPoint> & final_path)
{
  std::lock_guard<std::mutex> lock(mtx_);
  final_path_.clear();

  if (alternative_paths.size() == 0 || alternative_paths[0].size() == 0) {
    return;
  }

  // 1. 构建邻接矩阵
  buildAdjacencyMatrix(alternative_paths);
  // 2. 建立邻接矩阵链接关系
  connectAdjacencyMatrix();
  // 3. 在邻接矩阵中搜索最短路径
  searchAdjacencyMatrix();

  final_path = final_path_;
}

/**
 * @brief 构建邻接矩阵
 */
void LexPlanner::buildAdjacencyMatrix(
          const std::vector<std::vector<nav2_lex_local_planner::WayPoint>> & alternative_paths)
{
  // Step: 1. 清除旧的 node_list_
  for (size_t i = 0; i < node_list_.size(); ++i) {
    state_t * stateCur = node_list_[i];
    delete stateCur;
  }
  node_list_.clear();
  adjacency_matrix_.clear();

  // Step: 2. 创建新的 node_list_，并与 adjacency_matrix_ 关联
  adjacency_width_grid_ = alternative_paths.size();        // 行数是轨迹的数量
  adjacency_length_grid_ = alternative_paths[0].size();    // 列数是每条轨迹的长度

  adjacency_matrix_.resize(adjacency_width_grid_);
  for (int i = 0; i < adjacency_width_grid_; ++i) {
    adjacency_matrix_[i].resize(adjacency_length_grid_);
  }

  // 对于每个节点都创建一个 state_t 结构体，然后将其放入 node_list_ 中
  for (size_t i = 0; i < alternative_paths.size(); ++i) {
    for (size_t j = 0; j < alternative_paths[i].size(); ++j) {
      state_t * newState = new state_t;
      newState->x = alternative_paths[i][j].pos.x;
      newState->y = alternative_paths[i][j].pos.y;
      newState->z = 0;
      newState->theta = alternative_paths[i][j].pos.yaw;

      newState->idx = i;
      newState->idy = j;
      newState->validFlag = true;
      newState->stateId = node_list_.size();

      // node_list_ 中的元素是指针，都与 adjacency_matrix_ 中的元素关联
      node_list_.push_back(newState);
      adjacency_matrix_[i][j] = newState;
    }
  }
}

/**
 * @brief 建立邻接矩阵链接关系
 */
void LexPlanner::connectAdjacencyMatrix()
{
  // Step: 1. 检查某个节点是否在障碍物中，如果在，则将其 validFlag 置为 false
  for (size_t i = 0; i < node_list_.size(); ++i) {
    if (isInCollision(node_list_[i]->x, node_list_[i]->y)) {
      node_list_[i]->validFlag = false;
      continue;
    }
  }

  int connection_length = 1;
  // connect adjacency matrix
  for (int i = 0; i < adjacency_width_grid_; ++i) {
    for (int j = 0; j < adjacency_length_grid_; ++j) {
      if (adjacency_matrix_[i][j]->validFlag == false) {
        continue;
      }

      state_t * state_from = adjacency_matrix_[i][j];  // 第 i 行第 j 列的点

      // 建立当前点与八邻域内的点的关系
      for (int m = -connection_length; m <= connection_length; ++m) {  // (-1,0,1)
        for (int n = -connection_length; n <= connection_length; ++n) {  // (-1,0,1)
          if (m == 0 && n == 0) {  // do not add itself
            continue;
          }

          int id_x = i + m;
          int id_y = j + n;
          // 矩阵外的点跳过
          if (id_x < 0 || id_x >= adjacency_width_grid_ || id_y < 0 || id_y >= adjacency_length_grid_) {
            continue;
          }
          // 邻域点不可用跳过
          if (adjacency_matrix_[id_x][id_y]->validFlag == false) {
            continue;
          }

          state_t * state_to = adjacency_matrix_[id_x][id_y];  // 第 i+m 行第 j+n 列的点

          float edgeCosts[NUM_COSTS];
          // 如果 from 和 to 是联通的，则计算其 cost, 并将 to 加入 from 的 neighborList 中
          if (edgePropagation(state_from, state_to, edgeCosts) == true) {
            neighbor_t thisNeighbor;
            for (int q = 0; q < NUM_COSTS; ++q) {
              thisNeighbor.edgeCosts[q] = edgeCosts[q];
            }
            thisNeighbor.neighbor = state_to;
            state_from->neighborList.push_back(thisNeighbor);
          }
        }
      }
    }
  }
}


/**
 * @brief 计算 from 节点到 to 节点的多代价
 * @param edgeCosts 需要计算的 cost 值，因为是一个数组，作为指针传入
 */
bool LexPlanner::edgePropagation(state_t * state_from, state_t * state_to, float edgeCosts[NUM_COSTS])
{
  // 0. initialize edgeCosts
  for (int i = 0; i < NUM_COSTS; ++i) {
    edgeCosts[i] = 0;
  }
  // 1. segment the edge for collision checking
  // state_to 到 state_from 的栅格数
  int steps = round(distance(state_from, state_to) / local_costmap_->getResolution());
  // 每一步的 x,y,z 的增量（m）
  float stepX = (state_to->x - state_from->x) / static_cast<float>(steps);
  float stepY = (state_to->y - state_from->y) / static_cast<float>(steps);
  float stepZ = (state_to->z - state_from->z) / static_cast<float>(steps);
  // 2. allocate memory for a state, this state must be deleted after collision checking
  state_t * stateCur = new state_t;
  stateCur->x = state_from->x;
  stateCur->y = state_from->y;
  stateCur->z = state_from->z;

  // 3. collision checking loop
  for (int stepCount = 0; stepCount < steps; ++stepCount) {
    stateCur->x += stepX;
    stateCur->y += stepY;
    stateCur->z += stepZ;

    if (isInCollision(stateCur->x, stateCur->y)) {
      delete stateCur;
      return false;
    }

    // close to obstacle cost （0 是该节点到下一个节点之间所有的栅格的 cost 之和）
    edgeCosts[0] += local_costmap_->getResolution() * getCloseCollisionCost(stateCur->x, stateCur->y);
  }

  // lane change cost (1 是当前节点到下一个节点是否有换道，当前道路到其它道路换道的个数)
  int centralTrajectoryIndex = planning_params_->rollOutNumber / 2;
  if (planning_params_->enableAlignCenterline) {
    edgeCosts[1] = static_cast<float>(abs(state_from->idx - centralTrajectoryIndex)) * planning_params_->rollOutDensity;
  }

  // distance cost （2 是当前节点到下一个节点的距离）
  edgeCosts[2] = distance(state_from, state_to);  // distance cost

  // 第一列和最后一列的节点的 cost 都是 0
  if (state_from->idy == 0 && state_to->idy == 0) {
    for (int i = 0; i < NUM_COSTS; ++i) {
      edgeCosts[i] = 0;
    }
  }

  // treat last column nodes all the same
  if (state_from->idy == adjacency_length_grid_ - 1 &&
      state_to->idy == adjacency_length_grid_ - 1)
  {
    for (int i = 0; i < NUM_COSTS; ++i) {
      edgeCosts[i] = 0;
    }
  }

  delete stateCur;
  return true;
}


/**
 * @brief 使用 Dijkstra 算法搜索最短路径
 */
bool LexPlanner::searchAdjacencyMatrix()
{
  // Step: 1. reset costs，重置所有节点到根节点的 cost 为无穷大
  for (size_t i = 0; i < node_list_.size(); ++i) {
    for (int j = 0; j < NUM_COSTS; ++j) {
      node_list_[i]->costsToRoot[j] = FLT_MAX;
    }
  }

  // Step: 2. define start state （从中间轨迹的节点开始搜索）
  state_t * startState = adjacency_matrix_[static_cast<int>(adjacency_width_grid_ / 2)][0];
  for (int i = 0; i < NUM_COSTS; ++i) {
    startState->costsToRoot[i] = 0;
  }

  // Step: 3. search graph （Dijkstra 搜索算法搜索最短路径）
  std::vector<state_t *> Queue;
  Queue.push_back(startState);

  while (Queue.size() > 0 && rclcpp::ok()) {
    // find the state that can offer lowest cost in this depth and remove it from Queue
    state_t * fromState = minCostStateInQueue(Queue);
    Queue.erase(remove(Queue.begin(), Queue.end(), fromState), Queue.end());
    // loop through all neighbors of this state
    for (size_t i = 0; i < fromState->neighborList.size(); ++i) {
      state_t * toState = fromState->neighborList[i].neighbor;

      if (toState->validFlag == false) {
        continue;
      }
      // Loop through cost hierarchy
      for (int costIndex = 0; costIndex < NUM_COSTS; ++costIndex) {
        float thisCost = fromState->costsToRoot[costIndex] + fromState->neighborList[i].edgeCosts[costIndex];
        // If cost can be improved, update this node with new costs
        if (thisCost < toState->costsToRoot[costIndex]) {
          updateCosts(fromState, toState, i);  // update toState's costToRoot
          toState->parentState = fromState;    // change parent for toState
          Queue.push_back(toState);
        } else if (thisCost == toState->costsToRoot[costIndex]) {
          // If cost is same, go to compare secondary cost
          continue;
        } else {
          // If cost becomes higher, abort this propagation
          break;
        }
      }
    }
  }

  // 4. find goal state
  Queue.clear();
  for (int i = 0; i < adjacency_width_grid_; ++i) {
    Queue.push_back(adjacency_matrix_[i][adjacency_length_grid_ - 1]);  // 压入每个路径上最后一个点
  }

  state_t * goalState = minCostStateInQueue(Queue);

  // 5. extract path
  if (goalState->parentState == NULL) {
    return false;
  } else {
    extractPath(goalState);
  }

  return true;
}


/**
 * @brief 更新 costsToRoot
 */
void LexPlanner::updateCosts(state_t * fromState, state_t * toState, int neighborInd)
{
  for (int i = 0; i < NUM_COSTS; ++i) {
    toState->costsToRoot[i] = fromState->costsToRoot[i] + fromState->neighborList[neighborInd].edgeCosts[i];
  }
}


/**
 * @brief 从当前 Queue 中找到能提供 cost 最小的节点（按 cost 分层比较）
 */
state_t * LexPlanner::minCostStateInQueue(std::vector<state_t *> Queue)
{
  // Loop through cost hierarchy
  for (int costIndex = 0; costIndex < NUM_COSTS; ++costIndex) {
    std::vector<state_t *> tempQueue;
    float minCost = FLT_MAX;
    // loop through nodes saved in Queue
    for (std::vector<state_t *>::const_iterator iter2 = Queue.begin(); iter2 != Queue.end(); ++iter2) {
      state_t * thisState = *iter2;
      if (thisState->costsToRoot[costIndex] < minCost) {
        minCost = thisState->costsToRoot[costIndex];
        tempQueue.clear();
        tempQueue.push_back(thisState);
      } else if (thisState->costsToRoot[costIndex] == minCost) {
        tempQueue.push_back(thisState);
      }
    }
    // Queue is used again for next loop
    Queue.clear();
    Queue = tempQueue;
  }
  return Queue[0];
}


/**
 * @brief 从 goalState 开始，回溯找到搜索的路径
 */
bool LexPlanner::extractPath(state_t * stateCur)
{
  std::vector<nav2_lex_local_planner::WayPoint> pathOut;

  final_path_.clear();

  while (1) {
    nav2_lex_local_planner::WayPoint poseCur;
    poseCur.pos.x = stateCur->x;
    poseCur.pos.y = stateCur->y;
    poseCur.pos.z = stateCur->z;
    poseCur.pos.yaw = stateCur->theta;

    pathOut.insert(pathOut.begin(), poseCur);

    if (stateCur->parentState == NULL) {
      break;
    } else {
      stateCur = stateCur->parentState;
    }
  }
  if (pathOut.size() == 0) {
    return false;
  } else {
    final_path_ = processPath(pathOut);
    return true;
  }
}


std::vector<nav2_lex_local_planner::WayPoint> LexPlanner::processPath(
    std::vector<nav2_lex_local_planner::WayPoint> pathIn)
{
  calculatePathYaw(pathIn);
  fixPathDensity(pathIn, planning_params_->pathDensity);
  smoothPath(pathIn,
             planning_params_->smoothingDataWeight,
             planning_params_->smoothingSmoothWeight,
             planning_params_->smoothingToleranceError);
  return pathIn;
}

float LexPlanner::distance(state_t * state_from, state_t * state_to)
{
  return sqrt((state_to->x - state_from->x) * (state_to->x - state_from->x) +
              (state_to->y - state_from->y) * (state_to->y - state_from->y) +
              (state_to->z - state_from->z) * (state_to->z - state_from->z));
}

bool LexPlanner::isInCollision(float x, float y)
{
  unsigned int mx, my;
  local_costmap_->worldToMap(x, y, mx, my);
  if (mx >= local_costmap_->getSizeInCellsX() ||
      my >= local_costmap_->getSizeInCellsY())
  {
    return true;
  }

  unsigned char cost = local_costmap_->getCost(mx, my);
  if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
      cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
      cost == nav2_costmap_2d::NO_INFORMATION)
  {
    return true;
  } else {
    return false;
  }
}


float LexPlanner::getCloseCollisionCost(float x, float y)
{
  unsigned int mx, my;
  local_costmap_->worldToMap(x, y, mx, my);

  if (mx >= local_costmap_->getSizeInCellsX() ||
      my >= local_costmap_->getSizeInCellsY())
  {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }

  unsigned char cost = local_costmap_->getCost(mx, my);

  if (cost > 20)
    return static_cast<float>(cost);
  else
    return 0.0;
}


bool LexPlanner::isCloseCollision(float x, float y)
{
  unsigned int mx, my;
  local_costmap_->worldToMap(x, y, mx, my);

  if (mx >= local_costmap_->getSizeInCellsX() ||
      my >= local_costmap_->getSizeInCellsY())
  {
    return true;
  }

  unsigned char cost = local_costmap_->getCost(mx, my);

  if (cost > 50)
    return true;
  else
    return false;
}

}  // namespace nav2_lex_local_planner
