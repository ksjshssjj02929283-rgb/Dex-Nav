// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#include "nav2_route_planner/path_interpolator.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_route_planner
{

namespace
{

geometry_msgs::msg::Quaternion yawToQuat(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

geometry_msgs::msg::PoseStamped makePose(double x, double y)
{
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.w = 1.0;
  return p;
}

}  // namespace

std::vector<std::pair<double, double>> PathInterpolator::sampleBezier(
  const std::vector<std::pair<double, double>> & cp,
  int num_samples)
{
  // Bernstein binomial — small N, recompute is fine.
  auto binom = [](int n, int k) {
      if (k == 0 || k == n) {return 1.0;}
      double v = 1.0;
      for (int i = 0; i < k; ++i) {v = v * (n - i) / (i + 1);}
      return v;
    };
  std::vector<std::pair<double, double>> out;
  out.reserve(static_cast<std::size_t>(num_samples) + 1);
  const int n = static_cast<int>(cp.size()) - 1;
  for (int s = 0; s <= num_samples; ++s) {
    double t = static_cast<double>(s) / num_samples;
    double bx = 0.0;
    double by = 0.0;
    for (int i = 0; i <= n; ++i) {
      double b = binom(n, i) * std::pow(1.0 - t, n - i) * std::pow(t, i);
      bx += b * cp[i].first;
      by += b * cp[i].second;
    }
    out.emplace_back(bx, by);
  }
  return out;
}

void PathInterpolator::emitLinear(
  std::vector<geometry_msgs::msg::PoseStamped> & out,
  double x1, double y1, double x2, double y2) const
{
  const double dist = std::hypot(x2 - x1, y2 - y1);
  if (dist < 1e-6) {return;}
  const int loops = std::max(1, static_cast<int>(dist * p_.waypoints_per_meter));
  for (int k = 1; k <= loops; ++k) {
    double t = static_cast<double>(k) / loops;
    geometry_msgs::msg::PoseStamped p;
    p.pose.position.x = x1 + t * (x2 - x1);
    p.pose.position.y = y1 + t * (y2 - y1);
    p.pose.orientation.w = 1.0;
    out.push_back(std::move(p));
  }
}

void PathInterpolator::computeOrientations(std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  if (poses.size() < 2) {return;}
  for (std::size_t i = 0; i + 1 < poses.size(); ++i) {
    double dx = poses[i + 1].pose.position.x - poses[i].pose.position.x;
    double dy = poses[i + 1].pose.position.y - poses[i].pose.position.y;
    if (std::hypot(dx, dy) < 1e-6) {
      // Carry forward previous orientation if any
      if (i > 0) {
        poses[i].pose.orientation = poses[i - 1].pose.orientation;
      } else {
        poses[i].pose.orientation.w = 1.0;
      }
      continue;
    }
    poses[i].pose.orientation = yawToQuat(std::atan2(dy, dx));
  }
  // Last pose copies its predecessor's heading.
  poses.back().pose.orientation = poses[poses.size() - 2].pose.orientation;
}

void PathInterpolator::finalizePlan(
  InterpolatedRoutePlan & plan,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  for (auto & seg : plan.segments) {
    computeOrientations(seg.samples);
  }
  // Force the very last sample's orientation to the goal's heading
  // (matches nav2_theta_star_planner's end-of-plan behavior).
  if (!plan.segments.empty() && !plan.segments.back().samples.empty()) {
    if (!p_.use_final_approach_orientation) {
      plan.segments.back().samples.back().pose.orientation = goal.pose.orientation;
    }
  }
  std::ostringstream id;
  id << "plan_" << static_cast<int64_t>(start.pose.position.x * 1000) << "_"
     << static_cast<int64_t>(start.pose.position.y * 1000) << "_to_"
     << static_cast<int64_t>(goal.pose.position.x * 1000) << "_"
     << static_cast<int64_t>(goal.pose.position.y * 1000);
  plan.plan_id = id.str();
}

InterpolatedRoutePlan PathInterpolator::interpolate(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const AStarSearcher::Result & search_result,
  const TopoGraph & graph) const
{
  InterpolatedRoutePlan plan;
  if (!search_result.success || search_result.node_ids.empty()) {
    return plan;
  }

  // Entry / exit nodes ARE the start/goal projection points — the augmenter
  // inserted them as virtual nodes (or they were already real nodes).
  auto entry = graph.getNode(search_result.node_ids.front());
  auto exit = graph.getNode(search_result.node_ids.back());
  if (!entry || !exit) {return plan;}

  // Synthetic sub-edge ids look like "<edge>#0"; report the parent road id.
  auto parentId = [](const std::string & id) {
      const auto h = id.find('#');
      return h == std::string::npos ? id : id.substr(0, h);
    };

  // ── Segment 1: __start_transition__ — free-space start → entry node.
  // The caller Theta*-rewrites this so the robot reaches the road safely.
  {
    InterpolatedSegment seg;
    seg.edge_id = "__start_transition__";
    seg.lane_subtype = "transition";
    seg.floor = entry->floor;
    seg.samples.push_back(start);
    emitLinear(seg.samples,
      start.pose.position.x, start.pose.position.y, entry->x, entry->y);
    plan.segments.push_back(std::move(seg));
    plan.floor_sequence.push_back(entry->floor);
  }

  // ── Middle: walk each edge of the A* result, sampled along its geometry.
  for (std::size_t i = 0; i < search_result.edge_ids.size(); ++i) {
    auto e_opt = graph.getEdge(search_result.edge_ids[i]);
    if (!e_opt) {continue;}
    const TopoEdge & e = *e_opt;
    auto edge_pts = graph.sampleEdge(e, p_.bezier_samples_per_edge);
    if (edge_pts.size() < 2) {continue;}
    // Traverse in the direction A* used (node_ids[i] → node_ids[i+1]).
    if (e.from_id != search_result.node_ids[i]) {
      std::reverse(edge_pts.begin(), edge_pts.end());
    }

    InterpolatedSegment seg;
    seg.edge_id = parentId(e.id);
    seg.lane_subtype = e.lane_subtype;
    seg.max_speed = e.max_speed;
    seg.regulatory_element_ids = e.regulatory_element_ids;
    seg.floor = e.floor;
    seg.samples.push_back(makePose(edge_pts.front().first, edge_pts.front().second));
    for (std::size_t k = 1; k < edge_pts.size(); ++k) {
      emitLinear(seg.samples,
        edge_pts[k - 1].first, edge_pts[k - 1].second,
        edge_pts[k].first, edge_pts[k].second);
    }
    plan.segments.push_back(std::move(seg));

    if (plan.floor_sequence.empty() || plan.floor_sequence.back() != e.floor) {
      plan.floor_sequence.push_back(e.floor);
      plan.floor_changes++;
    }
    plan.total_distance += e.length;
  }

  // ── Segment N: __goal_alignment__ — free-space exit node → goal.
  // The caller Theta*-rewrites this. The exact goal pose terminates it.
  {
    InterpolatedSegment seg;
    seg.edge_id = "__goal_alignment__";
    seg.lane_subtype = "transition";
    seg.floor = exit->floor;
    seg.samples.push_back(makePose(exit->x, exit->y));
    emitLinear(seg.samples, exit->x, exit->y,
      goal.pose.position.x, goal.pose.position.y);
    seg.samples.push_back(goal);
    plan.segments.push_back(std::move(seg));
  }

  finalizePlan(plan, start, goal);
  return plan;
}

// ── Conversions ─────────────────────────────────────────────────────────

nav_msgs::msg::Path InterpolatedRoutePlan::toNavPath(
  const std::string & frame_id,
  const rclcpp::Time & stamp) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = stamp;
  for (const auto & seg : segments) {
    for (const auto & p : seg.samples) {
      geometry_msgs::msg::PoseStamped ps = p;
      ps.header.frame_id = frame_id;
      ps.header.stamp = stamp;
      path.poses.push_back(std::move(ps));
    }
  }
  return path;
}

nav_interfaces::msg::RoutePlan InterpolatedRoutePlan::toMsg(
  const std::string & frame_id,
  const rclcpp::Time & stamp) const
{
  nav_interfaces::msg::RoutePlan msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.plan_id = plan_id;
  msg.floor_sequence = floor_sequence;
  msg.total_distance = static_cast<float>(total_distance);
  msg.estimated_time = static_cast<float>(estimated_time);
  msg.floor_changes = floor_changes;
  msg.segments.reserve(segments.size());
  for (const auto & seg : segments) {
    nav_interfaces::msg::RoutePlanSegment sm;
    sm.edge_id = seg.edge_id;
    sm.lane_subtype = seg.lane_subtype;
    sm.max_speed = static_cast<float>(seg.max_speed);
    sm.regulatory_element_ids = seg.regulatory_element_ids;
    sm.floor = seg.floor;
    sm.samples.reserve(seg.samples.size());
    for (const auto & ps : seg.samples) {
      geometry_msgs::msg::PoseStamped p = ps;
      p.header.frame_id = frame_id;
      p.header.stamp = stamp;
      sm.samples.push_back(std::move(p));
    }
    msg.segments.push_back(std::move(sm));
  }
  return msg;
}

}  // namespace nav2_route_planner
