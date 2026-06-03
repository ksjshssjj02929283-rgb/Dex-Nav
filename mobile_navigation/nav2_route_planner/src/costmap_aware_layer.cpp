// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#include "nav2_route_planner/costmap_aware_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <utility>
#include <vector>

namespace nav2_route_planner
{

namespace
{

std::string costName(unsigned char cost)
{
  if (cost == nav2_costmap_2d::FREE_SPACE) {
    return "FREE_SPACE/free";
  }
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return "NO_INFORMATION/unknown";
  }
  if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
    return "LETHAL_OBSTACLE/lethal";
  }
  if (cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    return "INSCRIBED_INFLATED_OBSTACLE/inscribed";
  }
  return "INFLATED_OR_WEIGHTED_COST/inflated";
}

}  // namespace

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

std::string CostmapAwareLayer::describeLastThetaStarFailure() const
{
  if (!last_theta_star_failure_) {
    return "没有最近一次 Theta* 失败诊断(no recent Theta* failure diagnostic)";
  }

  const auto & d = *last_theta_star_failure_;
  std::ostringstream oss;
  switch (d.reason) {
    case ThetaStarFailureDiagnostic::Reason::UNAVAILABLE:
      oss << "Theta* 或 costmap 不可用(theta_star/costmap unavailable)";
      break;
    case ThetaStarFailureDiagnostic::Reason::OUT_OF_BOUNDS:
      oss << "端点超出代价地图范围(out of costmap bounds): "
          << "start=" << (d.start_in_bounds ? "IN" : "OUT")
          << ", goal=" << (d.goal_in_bounds ? "IN" : "OUT");
      if (d.start_in_bounds) {
        oss << ", start_cell=(" << d.start_mx << "," << d.start_my << ")";
      }
      if (d.goal_in_bounds) {
        oss << ", goal_cell=(" << d.goal_mx << "," << d.goal_my << ")";
      }
      break;
    case ThetaStarFailureDiagnostic::Reason::UNSAFE_ENDPOINT:
      oss << "端点代价不安全(endpoint unsafe): "
          << "start_cost=" << static_cast<int>(d.start_cost)
          << "(" << costName(d.start_cost) << "), "
          << "goal_cost=" << static_cast<int>(d.goal_cost)
          << "(" << costName(d.goal_cost) << "), "
          << "allow_unknown=" << (d.allow_unknown ? "true" : "false")
          << "; Theta* 只允许 FREE_SPACE(0)"
          << (d.allow_unknown ? " 或 NO_INFORMATION(255)" : "")
          << " 作为安全端点";
      break;
    case ThetaStarFailureDiagnostic::Reason::NO_PATH:
      oss << "端点在图内且安全，但 Theta* 搜索不到连通路径(no path found): "
          << "start_cell=(" << d.start_mx << "," << d.start_my << ")"
          << ", goal_cell=(" << d.goal_mx << "," << d.goal_my << ")"
          << ", start_cost=" << static_cast<int>(d.start_cost)
          << "(" << costName(d.start_cost) << ")"
          << ", goal_cost=" << static_cast<int>(d.goal_cost)
          << "(" << costName(d.goal_cost) << ")";
      break;
    case ThetaStarFailureDiagnostic::Reason::NONE:
    default:
      oss << "没有失败原因(no failure reason)";
      break;
  }
  return oss.str();
}

std::optional<std::vector<geometry_msgs::msg::PoseStamped>>
CostmapAwareLayer::planWithThetaStar(
  const geometry_msgs::msg::PoseStamped & seg_start,
  const geometry_msgs::msg::PoseStamped & seg_end)
{
  last_theta_star_failure_.reset();
  if (!theta_star_ || !costmap_) {
    ThetaStarFailureDiagnostic d;
    d.reason = ThetaStarFailureDiagnostic::Reason::UNAVAILABLE;
    d.allow_unknown = p_.theta_star_allow_unknown;
    last_theta_star_failure_ = d;
    RCLCPP_WARN(logger_, "planWithThetaStar: %s", describeLastThetaStarFailure().c_str());
    return std::nullopt;
  }

  // Re-attach costmap pointer in case the wrapper was just reactivated.
  theta_star_->costmap_ = costmap_;

  // Both endpoints must be in-bounds.
  unsigned int mxs{0}, mys{0}, mxg{0}, myg{0};
  unsigned char start_cost = nav2_costmap_2d::NO_INFORMATION;
  unsigned char goal_cost = nav2_costmap_2d::NO_INFORMATION;
  {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
    const bool start_in_bounds =
      costmap_->worldToMap(seg_start.pose.position.x, seg_start.pose.position.y, mxs, mys);
    const bool goal_in_bounds =
      costmap_->worldToMap(seg_end.pose.position.x, seg_end.pose.position.y, mxg, myg);
    if (!start_in_bounds || !goal_in_bounds) {
      ThetaStarFailureDiagnostic d;
      d.reason = ThetaStarFailureDiagnostic::Reason::OUT_OF_BOUNDS;
      d.start_in_bounds = start_in_bounds;
      d.goal_in_bounds = goal_in_bounds;
      d.start_mx = mxs;
      d.start_my = mys;
      d.goal_mx = mxg;
      d.goal_my = myg;
      d.allow_unknown = p_.theta_star_allow_unknown;
      last_theta_star_failure_ = d;
      RCLCPP_WARN(logger_, "planWithThetaStar: %s", describeLastThetaStarFailure().c_str());
      return std::nullopt;
    }

    start_cost = costmap_->getCost(mxs, mys);
    goal_cost = costmap_->getCost(mxg, myg);
  }

  theta_star_->setStartAndGoal(seg_start, seg_end);
  if (theta_star_->isUnsafeToPlan()) {
    ThetaStarFailureDiagnostic d;
    d.reason = ThetaStarFailureDiagnostic::Reason::UNSAFE_ENDPOINT;
    d.start_in_bounds = true;
    d.goal_in_bounds = true;
    d.start_mx = mxs;
    d.start_my = mys;
    d.goal_mx = mxg;
    d.goal_my = myg;
    d.start_cost = start_cost;
    d.goal_cost = goal_cost;
    d.allow_unknown = p_.theta_star_allow_unknown;
    last_theta_star_failure_ = d;
    RCLCPP_WARN(logger_, "planWithThetaStar: %s", describeLastThetaStarFailure().c_str());
    return std::nullopt;
  }

  std::vector<coordsW> raw;
  if (!theta_star_->generatePath(raw) || raw.empty()) {
    ThetaStarFailureDiagnostic d;
    d.reason = ThetaStarFailureDiagnostic::Reason::NO_PATH;
    d.start_in_bounds = true;
    d.goal_in_bounds = true;
    d.start_mx = mxs;
    d.start_my = mys;
    d.goal_mx = mxg;
    d.goal_my = myg;
    d.start_cost = start_cost;
    d.goal_cost = goal_cost;
    d.allow_unknown = p_.theta_star_allow_unknown;
    last_theta_star_failure_ = d;
    RCLCPP_WARN(logger_, "planWithThetaStar: %s", describeLastThetaStarFailure().c_str());
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

std::optional<StartGoalProjector::Result>
CostmapAwareLayer::shiftProjectionPastObstacle(
  const StartGoalProjector::Result & in,
  const geometry_msgs::msg::PoseStamped & toward_pose,
  const TopoGraph & graph,
  int num_bezier_samples,
  int waypoints_per_meter) const
{
  if (in.type != StartGoalProjector::ResultType::ON_EDGE) {return in;}
  if (!costmap_) {return in;}
  auto e = graph.getEdge(in.id);
  if (!e) {return in;}
  auto from_n = graph.getNode(e->from_id);
  auto to_n = graph.getNode(e->to_id);
  if (!from_n || !to_n) {return in;}

  // Slide direction: whichever endpoint is closer to `toward_pose`.
  //   start_proj: toward_pose=goal → slides toward the goal side
  //   goal_proj : toward_pose=start → slides toward the start side
  // i.e. both projections shift INTO the route's middle, on the side from
  // which the other endpoint approaches, so the post-shift point is on the
  // reachable half of the edge.
  const double d_from = std::hypot(
    from_n->x - toward_pose.pose.position.x,
    from_n->y - toward_pose.pose.position.y);
  const double d_to = std::hypot(
    to_n->x - toward_pose.pose.position.x,
    to_n->y - toward_pose.pose.position.y);
  const bool slide_toward_to = (d_to < d_from);

  // Build a dense pose path along the edge (from→to order).
  const auto samples = graph.sampleEdge(*e, num_bezier_samples);
  if (samples.size() < 2) {return in;}
  std::vector<geometry_msgs::msg::PoseStamped> path;
  for (std::size_t k = 0; k + 1 < samples.size(); ++k) {
    const double sx = samples[k].first;
    const double sy = samples[k].second;
    const double ex = samples[k + 1].first;
    const double ey = samples[k + 1].second;
    const double dist = std::hypot(ex - sx, ey - sy);
    const int n_steps =
      std::max(1, static_cast<int>(dist * waypoints_per_meter));
    for (int i = 0; i < n_steps; ++i) {
      const double t = static_cast<double>(i) / n_steps;
      geometry_msgs::msg::PoseStamped p;
      p.pose.position.x = sx + t * (ex - sx);
      p.pose.position.y = sy + t * (ey - sy);
      p.pose.orientation.w = 1.0;
      path.push_back(p);
    }
  }
  geometry_msgs::msg::PoseStamped last;
  last.pose.position.x = samples.back().first;
  last.pose.position.y = samples.back().second;
  last.pose.orientation.w = 1.0;
  path.push_back(last);
  if (path.size() < 2) {return in;}

  // Find pose closest to the original projected_xy.
  std::size_t proj_idx = 0;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path.size(); ++i) {
    const double dx = path[i].pose.position.x - in.projected_xy.first;
    const double dy = path[i].pose.position.y - in.projected_xy.second;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {best_d2 = d2; proj_idx = i;}
  }

  const auto obstacles = detectAllObstacles(path);
  if (obstacles.empty()) {return in;}

  auto containingObstacle =
    [&](std::size_t idx) -> const ObstacleSegment * {
      for (const auto & obs : obstacles) {
        if (idx >= obs.start_index && idx <= obs.end_index) {return &obs;}
      }
      return nullptr;
    };

  // Iterate so back-to-back obstacles (touching margins) are all skipped.
  std::size_t new_idx = proj_idx;
  while (auto * obs = containingObstacle(new_idx)) {
    if (slide_toward_to) {
      if (obs->end_index + 1 >= path.size()) {return std::nullopt;}
      new_idx = obs->end_index + 1;
    } else {
      if (obs->start_index == 0) {return std::nullopt;}
      new_idx = obs->start_index - 1;
    }
  }
  if (new_idx == proj_idx) {return in;}

  // Update projected_xy and edge_sample_index. The latter uses chord
  // fraction (FLOOR) for consistency with the planner's other partial-edge
  // computations; exact for LINE, monotonic for BEZIER.
  StartGoalProjector::Result out = in;
  out.projected_xy.first = path[new_idx].pose.position.x;
  out.projected_xy.second = path[new_idx].pose.position.y;
  const double new_t = StartGoalProjector::fractionAlongChord(
    out.projected_xy, from_n->x, from_n->y, to_n->x, to_n->y);
  const double idx_f = new_t * static_cast<double>(samples.size() - 1);
  out.edge_sample_index = static_cast<std::size_t>(std::floor(idx_f));
  if (out.edge_sample_index + 1 >= samples.size()) {
    out.edge_sample_index =
      samples.size() >= 2 ? samples.size() - 2 : 0;
  }
  return out;
}


}  // namespace nav2_route_planner
