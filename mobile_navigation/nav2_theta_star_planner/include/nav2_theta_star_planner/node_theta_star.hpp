// Copyright (c) DexForce
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef NAV2_THETA_STAR_PLANNER__NODE_THETA_STAR_HPP_
#define NAV2_THETA_STAR_PLANNER__NODE_THETA_STAR_HPP_

#include <queue>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_map_server/node_map_types.hpp"
#include "nav2_theta_star_planner/theta_star.hpp"

namespace theta_star
{

/// A* on a preprocessed node graph (edges validated at map build time).
class NodeThetaStar
{
public:
  static constexpr int kDefaultMaxExpansions = 10000;

  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  nav2_map_server::DistanceMap distance_map_;
  nav2_map_server::NodeMap node_map_;

  double w_traversal_cost_{2.0};
  double w_euc_cost_{1.0};
  double w_heuristic_cost_{1.0};
  bool allow_unknown_{false};
  double min_clearance_m_{0.45};

  int nodes_opened{0};
  int max_expansions_{kDefaultMaxExpansions};

  bool hasNodeMap() const {return !node_map_.nodes.empty();}

  void setNodeMap(const nav2_map_server::NodeMap & map, double min_clearance_m);

  void setStartAndGoal(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);

  void setStartAndGoalWorld(double start_x, double start_y, double goal_x, double goal_y);

  bool generatePath(std::vector<coordsW> & raw_path);

  bool isUnsafeToPlan() const;

  int findNearestNode(int gx, int gy) const;

  int findNearestNodeWorld(double wx, double wy) const;

private:
  struct GraphNode
  {
    int id{-1};
    int gx{0};
    int gy{0};
    double g{INF_COST};
    double h{INF_COST};
    double f{INF_COST};
    const GraphNode * parent{nullptr};
    bool in_queue{false};
  };

  struct GraphComp
  {
    bool operator()(const GraphNode * a, const GraphNode * b) const
    {
      return a->f > b->f;
    }
  };

  bool generatePathGraphAStar(std::vector<coordsW> & raw_path);
  void collectNeighborIds(int node_id, std::vector<int> & out) const;
  void backtraceWorld(std::vector<coordsW> & raw_points, const GraphNode * curr) const;
  void backtrace(std::vector<coordsW> & raw_points, const GraphNode * curr) const;

  int start_id_{-1};
  int goal_id_{-1};
  std::vector<GraphNode> graph_nodes_;
  std::priority_queue<GraphNode *, std::vector<GraphNode *>, GraphComp> queue_;

  bool losCheck(int x0, int y0, int x1, int y1, double & sl_cost) const;
  bool isGridFree(int gx, int gy) const;
  double traversalCost(int gx, int gy) const;
  void resetParent(GraphNode * curr);
  void expandNeighbors(GraphNode * curr);
  void clearSearch();
  void buildGridIndex();
};

}  // namespace theta_star

#endif  // NAV2_THETA_STAR_PLANNER__NODE_THETA_STAR_HPP_
