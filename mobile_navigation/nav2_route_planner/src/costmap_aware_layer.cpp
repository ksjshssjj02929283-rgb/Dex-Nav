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

  // ThetaStar returns waypoints in goal->start order (per upstream
  // convention). We expose them start->goal here.
  std::reverse(raw.begin(), raw.end());

  // Sample at half costmap resolution between consecutive waypoints so the
  // controller has a dense path to track.
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
  // Append the final waypoint (goal-side) so the path actually reaches seg_end
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

CostmapAwareLayer::ObstacleSegment
CostmapAwareLayer::detectObstacle(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan) const
{
  ObstacleSegment seg;
  if (!costmap_ || plan.size() < 2) {return seg;}

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());

  // Forward scan for the first lethal run, with hard stop at
  // `bypass_max_replan_length_m` of accumulated travel distance.
  std::size_t first_in = 0;
  std::size_t first_out = 0;
  bool in_obstacle = false;
  bool found = false;
  double traversed = 0.0;

  for (std::size_t i = 1; i < plan.size(); ++i) {
    const auto & prev = plan[i - 1].pose.position;
    const auto & cur = plan[i].pose.position;
    traversed += std::hypot(cur.x - prev.x, cur.y - prev.y);
    if (traversed > p_.bypass_max_replan_length_m) {break;}

    const auto cost = getCostAt(cur.x, cur.y);
    const bool lethal = cost >= p_.obstacle_cost_threshold;
    if (lethal && !in_obstacle) {
      first_in = i;
      in_obstacle = true;
      found = true;
    } else if (!lethal && in_obstacle) {
      first_out = i;
      in_obstacle = false;
      break;  // first complete run handled
    }
  }

  if (!found) {return seg;}
  if (in_obstacle) {
    // Plan ends inside the obstacle — clip to last index
    first_out = plan.size() - 1;
  }

  // Expand outward by margins. Walk backwards from first_in until
  // cumulative distance ≥ margin_before, similarly forward from first_out.
  auto walkBack = [&plan](std::size_t i, double margin) -> std::size_t {
      if (i == 0) {return 0;}
      double acc = 0.0;
      while (i > 0) {
        const auto & a = plan[i - 1].pose.position;
        const auto & b = plan[i].pose.position;
        acc += std::hypot(b.x - a.x, b.y - a.y);
        if (acc >= margin) {return i - 1;}
        --i;
      }
      return 0;
    };
  auto walkFwd = [&plan](std::size_t i, double margin) -> std::size_t {
      if (i + 1 >= plan.size()) {return plan.size() - 1;}
      double acc = 0.0;
      while (i + 1 < plan.size()) {
        const auto & a = plan[i].pose.position;
        const auto & b = plan[i + 1].pose.position;
        acc += std::hypot(b.x - a.x, b.y - a.y);
        if (acc >= margin) {return i + 1;}
        ++i;
      }
      return plan.size() - 1;
    };

  seg.start_index = walkBack(first_in, p_.obstacle_margin_before_m);
  seg.end_index = walkFwd(first_out, p_.obstacle_margin_after_m);
  seg.has_obstacle = true;

  RCLCPP_DEBUG(
    logger_,
    "Layer 2: obstacle run [%zu..%zu] expanded to [%zu..%zu]",
    first_in, first_out, seg.start_index, seg.end_index);
  return seg;
}


}  // namespace nav2_route_planner
