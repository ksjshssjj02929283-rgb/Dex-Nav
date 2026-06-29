// Copyright (c) DexForce
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "nav2_theta_star_planner/node_theta_star.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace theta_star
{
namespace
{
inline int gridIdx(int gx, int gy, int width)
{
  return gy * width + gx;
}
}  // namespace

void NodeThetaStar::setNodeMap(const nav2_map_server::NodeMap & map, double min_clearance_m)
{
  node_map_ = map;
  min_clearance_m_ = min_clearance_m;
  buildGridIndex();
}

void NodeThetaStar::buildGridIndex()
{
  const size_t ncells = static_cast<size_t>(node_map_.width * node_map_.height);
  node_map_.grid_node_id.assign(ncells, -1);
  for (const auto & node : node_map_.nodes) {
    if (node.gx >= 0 && node.gx < node_map_.width &&
      node.gy >= 0 && node.gy < node_map_.height)
    {
      node_map_.grid_node_id[static_cast<size_t>(gridIdx(node.gx, node.gy, node_map_.width))] =
        node.id;
    }
  }
}

bool NodeThetaStar::isGridFree(int gx, int gy) const
{
  if (node_map_.layer == "raw" && !node_map_.grid_node_id.empty()) {
    if (gx < 0 || gy < 0 || gx >= node_map_.width || gy >= node_map_.height) {
      return false;
    }
    return node_map_.grid_node_id[static_cast<size_t>(gridIdx(gx, gy, node_map_.width))] >= 0;
  }

  if (!costmap_) {
    return false;
  }
  if (gx < 0 || gy < 0 ||
    gx >= static_cast<int>(costmap_->getSizeInCellsX()) ||
    gy >= static_cast<int>(costmap_->getSizeInCellsY()))
  {
    return false;
  }
  const int cost = costmap_->getCost(static_cast<unsigned int>(gx), static_cast<unsigned int>(gy));
  return (cost == UNKNOWN_COST && allow_unknown_) || cost == FREE_COST;
}

double NodeThetaStar::traversalCost(int gx, int gy) const
{
  if (node_map_.layer == "raw") {
    return 0.0;
  }
  const double curr_cost = 26.0 + 0.9 * costmap_->getCost(
    static_cast<unsigned int>(gx), static_cast<unsigned int>(gy));
  return w_traversal_cost_ * curr_cost * curr_cost / LETHAL_COST / LETHAL_COST;
}

bool NodeThetaStar::losCheck(int x0, int y0, int x1, int y1, double & sl_cost) const
{
  sl_cost = 0.0;
  const bool check_distance = min_clearance_m_ > 1e-6 && !distance_map_.distance_m.empty();

  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = x0 < x1 ? 1 : -1;
  int sy = y0 < y1 ? 1 : -1;
  int err = dx - dy;
  int x = x0;
  int y = y0;

  while (true) {
    if (!isGridFree(x, y)) {
      return false;
    }
    if (check_distance) {
      const int i = y * distance_map_.width + x;
      if (i < 0 || i >= static_cast<int>(distance_map_.distance_m.size())) {
        return false;
      }
      if (distance_map_.distance_m[static_cast<size_t>(i)] <
        static_cast<float>(min_clearance_m_))
      {
        return false;
      }
    }

    sl_cost += traversalCost(x, y);

    if (x == x1 && y == y1) {
      break;
    }
    const int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x += sx;
    }
    if (e2 < dx) {
      err += dx;
      y += sy;
    }
  }
  return true;
}

int NodeThetaStar::findNearestNode(int gx, int gy) const
{
  int best = -1;
  double best_dist = std::numeric_limits<double>::infinity();
  for (const auto & node : node_map_.nodes) {
    const double dx = static_cast<double>(node.gx - gx);
    const double dy = static_cast<double>(node.gy - gy);
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_dist) {
      best_dist = d2;
      best = node.id;
    }
  }
  return best;
}

void NodeThetaStar::clearSearch()
{
  graph_nodes_.clear();
  queue_ = std::priority_queue<GraphNode *, std::vector<GraphNode *>, GraphComp>();
  start_id_ = -1;
  goal_id_ = -1;
}

void NodeThetaStar::setStartAndGoal(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  setStartAndGoalWorld(
    start.pose.position.x, start.pose.position.y,
    goal.pose.position.x, goal.pose.position.y);
}

void NodeThetaStar::setStartAndGoalWorld(
  double start_x, double start_y, double goal_x, double goal_y)
{
  clearSearch();

  unsigned int sgx, sgy, dgx, dgy;
  costmap_->worldToMap(start_x, start_y, sgx, sgy);
  costmap_->worldToMap(goal_x, goal_y, dgx, dgy);

  graph_nodes_.resize(node_map_.nodes.size());
  for (size_t i = 0; i < node_map_.nodes.size(); ++i) {
    graph_nodes_[i].id = node_map_.nodes[i].id;
    graph_nodes_[i].gx = node_map_.nodes[i].gx;
    graph_nodes_[i].gy = node_map_.nodes[i].gy;
  }

  start_id_ = findNearestNode(static_cast<int>(sgx), static_cast<int>(sgy));
  goal_id_ = findNearestNode(static_cast<int>(dgx), static_cast<int>(dgy));
}

bool NodeThetaStar::isUnsafeToPlan() const
{
  return start_id_ < 0 || goal_id_ < 0;
}

void NodeThetaStar::resetParent(GraphNode * curr)
{
  if (!curr || !curr->parent || !curr->parent->parent) {
    return;
  }
  const GraphNode * maybe_par = curr->parent->parent;
  double los_cost = 0.0;
  if (losCheck(curr->gx, curr->gy, maybe_par->gx, maybe_par->gy, los_cost)) {
    const double g_cost = maybe_par->g +
      w_euc_cost_ * std::hypot(
      curr->gx - maybe_par->gx,
      curr->gy - maybe_par->gy) + los_cost;
    if (g_cost < curr->g) {
      curr->parent = maybe_par;
      curr->g = g_cost;
      curr->f = g_cost + curr->h;
    }
  }
}

void NodeThetaStar::expandNeighbors(GraphNode * curr)
{
  curr->in_queue = false;

  auto relaxNeighbor = [&](GraphNode * neighbor) {
      if (!neighbor || neighbor->id == curr->id) {
        return;
      }
      const double edge = w_euc_cost_ * std::hypot(
        neighbor->gx - curr->gx, neighbor->gy - curr->gy) +
        traversalCost(neighbor->gx, neighbor->gy);
      const double g_cost = curr->g + edge;
      const GraphNode & goal_node = graph_nodes_[static_cast<size_t>(goal_id_)];
      const double h_cost = w_heuristic_cost_ * std::hypot(
        neighbor->gx - goal_node.gx, neighbor->gy - goal_node.gy);
      const double f_cost = g_cost + h_cost;
      if (neighbor->f > f_cost || !neighbor->in_queue) {
        neighbor->g = g_cost;
        neighbor->h = h_cost;
        neighbor->f = f_cost;
        neighbor->parent = curr;
        if (!neighbor->in_queue) {
          neighbor->in_queue = true;
          queue_.push(neighbor);
        }
      }
    };

  if (node_map_.adjacency_mode == nav2_map_server::AdjacencyMode::GRID8) {
    static const int moves[8][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1},
      {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
    for (const auto & mv : moves) {
      const int nx = curr->gx + mv[0];
      const int ny = curr->gy + mv[1];
      if (nx < 0 || ny < 0 || nx >= node_map_.width || ny >= node_map_.height) {
        continue;
      }
      const int nid = node_map_.grid_node_id[static_cast<size_t>(gridIdx(nx, ny, node_map_.width))];
      if (nid < 0 || nid >= static_cast<int>(graph_nodes_.size())) {
        continue;
      }
      relaxNeighbor(&graph_nodes_[static_cast<size_t>(nid)]);
    }
    return;
  }

  if (static_cast<size_t>(curr->id) >= node_map_.adjacency.size()) {
    return;
  }

  for (int nb_id : node_map_.adjacency[static_cast<size_t>(curr->id)]) {
    if (nb_id < 0 || nb_id >= static_cast<int>(graph_nodes_.size())) {
      continue;
    }
    relaxNeighbor(&graph_nodes_[static_cast<size_t>(nb_id)]);
  }
}

bool NodeThetaStar::generatePath(std::vector<coordsW> & raw_path)
{
  raw_path.clear();
  if (start_id_ < 0 || goal_id_ < 0 ||
    start_id_ >= static_cast<int>(graph_nodes_.size()) ||
    goal_id_ >= static_cast<int>(graph_nodes_.size()))
  {
    return false;
  }

  for (auto & node : graph_nodes_) {
    node.g = INF_COST;
    node.h = INF_COST;
    node.f = INF_COST;
    node.parent = nullptr;
    node.in_queue = false;
  }

  GraphNode * start = &graph_nodes_[static_cast<size_t>(start_id_)];
  GraphNode * goal = &graph_nodes_[static_cast<size_t>(goal_id_)];

  if (start_id_ == goal_id_) {
    coordsW w{};
    costmap_->mapToWorld(
      static_cast<unsigned int>(start->gx), static_cast<unsigned int>(start->gy), w.x, w.y);
    raw_path.push_back(w);
    return true;
  }

  start->g = traversalCost(start->gx, start->gy);
  start->h = w_heuristic_cost_ * std::hypot(start->gx - goal->gx, start->gy - goal->gy);
  start->f = start->g + start->h;
  start->parent = start;
  start->in_queue = true;
  queue_.push(start);
  nodes_opened = 0;

  GraphNode * curr = start;
  while (!queue_.empty()) {
    ++nodes_opened;

    if (curr->id == goal_id_) {
      break;
    }

    resetParent(curr);
    expandNeighbors(curr);

    curr = queue_.top();
    queue_.pop();
  }

  if (curr->id != goal_id_) {
    return false;
  }

  backtrace(raw_path, curr);
  return !raw_path.empty();
}

void NodeThetaStar::backtrace(std::vector<coordsW> & raw_points, const GraphNode * curr) const
{
  std::vector<coordsW> rev;
  const GraphNode * n = curr;
  while (n && n->parent) {
    coordsW w{};
    costmap_->mapToWorld(
      static_cast<unsigned int>(n->gx), static_cast<unsigned int>(n->gy), w.x, w.y);
    rev.push_back(w);
    if (n->parent == n) {
      break;
    }
    n = n->parent;
  }

  raw_points.reserve(rev.size());
  for (int i = static_cast<int>(rev.size()) - 1; i >= 0; --i) {
    raw_points.push_back(rev[static_cast<size_t>(i)]);
  }
}

}  // namespace theta_star
