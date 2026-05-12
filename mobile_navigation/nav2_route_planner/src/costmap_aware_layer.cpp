// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#include "nav2_route_planner/costmap_aware_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

namespace nav2_route_planner
{

CostmapAwareLayer::CostmapAwareLayer(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
: parent_(std::move(parent)),
  costmap_ros_(std::move(costmap_ros))
{
  auto node = parent_.lock();
  if (node) {logger_ = node->get_logger();}
  if (costmap_ros_) {
    costmap_ = costmap_ros_->getCostmap();
  }
}

void CostmapAwareLayer::configure(const Params & p)
{
  p_ = p;
  // Pre-allocate ThetaStar so the first bypass call is cheap.
  theta_star_ = std::make_unique<theta_star::ThetaStar>();
  theta_star_->how_many_corners_ = p_.theta_star_max_corners;
  theta_star_->allow_unknown_ = p_.theta_star_allow_unknown;
  if (costmap_) {
    theta_star_->costmap_ = costmap_;
  }
  RCLCPP_INFO(
    logger_,
    "CostmapAwareLayer configured: layer1=Theta* (start/goal transitions) "
    "layer2=%s obstacle_thr=%.1f margin_before=%.2fm margin_after=%.2fm "
    "bypass_max=%.2fm theta_corners=%d unknown=%s",
    p_.costmap_aware_mode ? "ON" : "OFF",
    p_.obstacle_cost_threshold,
    p_.obstacle_margin_before_m, p_.obstacle_margin_after_m,
    p_.bypass_max_replan_length_m,
    p_.theta_star_max_corners,
    p_.theta_star_allow_unknown ? "true" : "false");
}

void CostmapAwareLayer::cleanup()
{
  theta_star_.reset();
  costmap_ = nullptr;
  costmap_ros_.reset();
}

void CostmapAwareLayer::updateParams(const Params & p)
{
  p_ = p;
  if (theta_star_) {
    theta_star_->how_many_corners_ = p_.theta_star_max_corners;
    theta_star_->allow_unknown_ = p_.theta_star_allow_unknown;
  }
}

std::optional<std::vector<geometry_msgs::msg::PoseStamped>>
CostmapAwareLayer::planWithThetaStar(
  const geometry_msgs::msg::PoseStamped & seg_start,
  const geometry_msgs::msg::PoseStamped & seg_end)
{
  if (!theta_star_ || !costmap_) {return std::nullopt;}

  // Re-attach costmap pointer in case the wrapper was just reactivated.
  theta_star_->costmap_ = costmap_;

  // Both endpoints must be in-bounds.
  unsigned int mxs, mys, mxg, myg;
  {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
    if (!costmap_->worldToMap(seg_start.pose.position.x, seg_start.pose.position.y, mxs, mys) ||
      !costmap_->worldToMap(seg_end.pose.position.x, seg_end.pose.position.y, mxg, myg))
    {
      RCLCPP_WARN(logger_, "planWithThetaStar: endpoints out of costmap bounds");
      return std::nullopt;
    }
  }

  theta_star_->setStartAndGoal(seg_start, seg_end);
  if (theta_star_->isUnsafeToPlan()) {
    RCLCPP_WARN(logger_,
      "planWithThetaStar: unsafe to plan (start or goal in lethal cost)");
    return std::nullopt;
  }

  std::vector<coordsW> raw;
  if (!theta_star_->generatePath(raw) || raw.empty()) {
    RCLCPP_WARN(logger_, "planWithThetaStar: ThetaStar failed to generate path");
    return std::nullopt;
  }

  // ⚠️ DO NOT REVERSE `raw`.
  //
  // ThetaStar::backtrace (nav2_theta_star_planner/src/theta_star.cpp:153)
  // already emits waypoints in src→goal order: it builds `path_rev` as
  // [goal, ..., src] by following parent_id, then does a final reverse
  // loop into `raw_points`. So `raw[0] == src == seg_start` and
  // `raw.back() == goal == seg_end` — exactly what we need.
  //
  // A previous version of this file reversed `raw` again here, citing
  // "upstream returns goal->start". That was wrong and caused the dense
  // bypass to walk goal→src, which after splicing into the surrounding
  // path produced a visible self-loop / fold-back (the user's reported
  // "A→B→theta_star" symptom). Keep `raw` in src→goal order.

  // Sample at half costmap resolution between consecutive waypoints so the
  // controller has a dense path to track. Output remains in src→goal
  // order, terminating exactly at seg_end.
  const double res = std::max(0.5 * costmap_->getResolution(), 0.01);
  std::vector<geometry_msgs::msg::PoseStamped> out;
  out.reserve(raw.size() * 4);

  geometry_msgs::msg::PoseStamped tmpl = seg_start;
  tmpl.pose.orientation.w = 1.0;

  for (std::size_t i = 0; i + 1 < raw.size(); ++i) {
    const auto & a = raw[i];
    const auto & b = raw[i + 1];
    const double dist = std::hypot(b.x - a.x, b.y - a.y);
    const int steps = std::max(1, static_cast<int>(std::ceil(dist / res)));
    for (int k = 0; k < steps; ++k) {
      const double t = static_cast<double>(k) / steps;
      geometry_msgs::msg::PoseStamped p = tmpl;
      p.pose.position.x = a.x + t * (b.x - a.x);
      p.pose.position.y = a.y + t * (b.y - a.y);
      out.push_back(p);
    }
  }
  // Append the final waypoint (seg_end / goal side) so the path
  // terminates exactly at the requested endpoint.
  geometry_msgs::msg::PoseStamped last = tmpl;
  last.pose.position = seg_end.pose.position;
  out.push_back(last);

  return out;
}

unsigned char CostmapAwareLayer::getCostAt(double x, double y) const
{
  if (!costmap_) {return nav2_costmap_2d::FREE_SPACE;}
  unsigned int mx, my;
  if (!costmap_->worldToMap(x, y, mx, my)) {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }
  return costmap_->getCost(mx, my);
}

std::vector<CostmapAwareLayer::ObstacleSegment>
CostmapAwareLayer::detectAllObstacles(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan) const
{
  std::vector<ObstacleSegment> out;
  if (!costmap_ || plan.size() < 2) {return out;}

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());

  // Pre-compute cumulative arc-length so margin walks are O(1) lookups.
  std::vector<double> cum_len(plan.size(), 0.0);
  for (std::size_t i = 1; i < plan.size(); ++i) {
    const auto & a = plan[i - 1].pose.position;
    const auto & b = plan[i].pose.position;
    cum_len[i] = cum_len[i - 1] + std::hypot(b.x - a.x, b.y - a.y);
  }
  const double scan_budget = p_.bypass_max_replan_length_m;

  // Forward scan for ALL contiguous lethal runs.
  std::vector<std::pair<std::size_t, std::size_t>> raw_runs;
  bool in_obstacle = false;
  std::size_t first_in = 0;
  for (std::size_t i = 1; i < plan.size(); ++i) {
    if (cum_len[i] > scan_budget) {break;}
    const auto & cur = plan[i].pose.position;
    const auto cost = getCostAt(cur.x, cur.y);
    const bool lethal = cost >= p_.obstacle_cost_threshold;
    if (lethal && !in_obstacle) {
      first_in = i;
      in_obstacle = true;
    } else if (!lethal && in_obstacle) {
      raw_runs.emplace_back(first_in, i - 1);
      in_obstacle = false;
    }
  }
  if (in_obstacle) {
    raw_runs.emplace_back(first_in, plan.size() - 1);
  }
  if (raw_runs.empty()) {return out;}

  // Margin expansion using cum_len. walkBack stops when distance from
  // first_in is ≥ margin OR we hit index 0; walkFwd similarly.
  auto walkBack = [&cum_len](std::size_t i, double margin) -> std::size_t {
      if (i == 0) {return 0;}
      const double target = cum_len[i] - margin;
      while (i > 0 && cum_len[i - 1] >= target) {--i;}
      return i;
    };
  auto walkFwd = [&cum_len, n = plan.size()](std::size_t i, double margin)
    -> std::size_t {
      const double target = cum_len[i] + margin;
      while (i + 1 < n && cum_len[i + 1] <= target) {++i;}
      return i;
    };

  // Build expanded segments, then enforce non-overlap by clipping
  // adjacent ranges to (mid-point) split.
  out.reserve(raw_runs.size());
  for (const auto & r : raw_runs) {
    ObstacleSegment s;
    s.start_index = walkBack(r.first, p_.obstacle_margin_before_m);
    s.end_index = walkFwd(r.second, p_.obstacle_margin_after_m);
    if (!out.empty() && s.start_index <= out.back().end_index) {
      // Overlap with previous run — merge into one larger run.
      out.back().end_index = std::max(out.back().end_index, s.end_index);
      continue;
    }
    out.push_back(s);
  }

  for (std::size_t k = 0; k < out.size(); ++k) {
    RCLCPP_DEBUG(
      logger_,
      "Layer 2: obstacle run %zu raw=[%zu..%zu] expanded=[%zu..%zu]",
      k, raw_runs[k].first, raw_runs[k].second,
      out[k].start_index, out[k].end_index);
  }
  return out;
}


}  // namespace nav2_route_planner
