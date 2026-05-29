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
  const StartGoalProjector::Result & start_proj,
  const StartGoalProjector::Result & goal_proj,
  const AStarSearcher::Result & search_result,
  const TopoGraph & graph) const
{
  InterpolatedRoutePlan plan;
  if (!search_result.success || search_result.node_ids.empty()) {
    return plan;
  }

  // The A*-chosen entry / exit nodes for the graph leg.
  const std::string entry_node_id = search_result.node_ids.front();
  const std::string exit_node_id = search_result.node_ids.back();
  auto entry_opt = graph.getNode(entry_node_id);
  auto exit_opt = graph.getNode(exit_node_id);
  if (!entry_opt || !exit_opt) {return plan;}

  // ── Segment 1: __start_transition__ ─ free-space, start → projected_xy ─
  //
  // This segment is ALWAYS just `start → projection point`. The caller is
  // expected to Theta\*-rewrite it. We never put the entry node here —
  // that's handled by the __start_edge_in__ segment below for ON_EDGE
  // starts.
  {
    const double proj_x = (start_proj.type == StartGoalProjector::ResultType::ON_EDGE)
      ? start_proj.projected_xy.first  : entry_opt->x;
    const double proj_y = (start_proj.type == StartGoalProjector::ResultType::ON_EDGE)
      ? start_proj.projected_xy.second : entry_opt->y;

    InterpolatedSegment seg;
    seg.edge_id = "__start_transition__";
    seg.lane_subtype = "transition";
    seg.floor = entry_opt->floor;
    seg.samples.push_back(start);
    emitLinear(seg.samples,
      start.pose.position.x, start.pose.position.y,
      proj_x, proj_y);
    plan.segments.push_back(std::move(seg));
    plan.floor_sequence.push_back(entry_opt->floor);
  }

  // ── Segment 1.5: __start_edge_in__ ─ walk along edge geometry ──
  //
  // Walk the sampled polyline from projected_xy back to the A*-chosen
  // entry node, using the FLOOR semantic of `edge_sample_index` (the
  // projector returns the from-end of the segment containing
  // projected_xy). The walk goes:
  //   • entry == from_id: reverse — projected_xy → edge_pts[idx] → ... → edge_pts[0]
  //   • entry == to_id:   forward — projected_xy → edge_pts[idx+1] → ... → edge_pts[N-1]
  // Now safe for BEZIER curves: `idx` is the last sample BEFORE the
  // projection, so walking through it can never overshoot.
  if (start_proj.type == StartGoalProjector::ResultType::ON_EDGE) {
    auto e_opt = graph.getEdge(start_proj.id);
    if (e_opt) {
      auto edge_pts = graph.sampleEdge(*e_opt, p_.bezier_samples_per_edge);
      if (edge_pts.size() >= 2) {
        const bool entry_is_from = (e_opt->from_id == entry_node_id);
        InterpolatedSegment seg;
        seg.edge_id = "__start_edge_in__";
        seg.lane_subtype = e_opt->lane_subtype;
        seg.max_speed = e_opt->max_speed;
        seg.floor = e_opt->floor;
        seg.samples.push_back(
          makePose(start_proj.projected_xy.first,
              start_proj.projected_xy.second));
        if (entry_is_from) {
          // Reverse-walk: projected_xy → edge_pts[idx] → edge_pts[idx-1] → ... → edge_pts[0]
          for (std::size_t i = start_proj.edge_sample_index + 1; i > 0; --i) {
            const std::size_t j = i - 1;
            emitLinear(seg.samples,
              seg.samples.back().pose.position.x,
              seg.samples.back().pose.position.y,
              edge_pts[j].first, edge_pts[j].second);
          }
        } else {
          // Forward-walk: projected_xy → edge_pts[idx+1] → ... → edge_pts[N-1]
          for (std::size_t i = start_proj.edge_sample_index + 1;
            i < edge_pts.size(); ++i)
          {
            emitLinear(seg.samples,
              seg.samples.back().pose.position.x,
              seg.samples.back().pose.position.y,
              edge_pts[i].first, edge_pts[i].second);
          }
        }
        plan.segments.push_back(std::move(seg));
      }
    }
  }

  // ── Segment 2: per-edge sampling along the A* result ────────────────
  //
  // Goal-edge fallback safety net: when A* could not find an alternative
  // path around goal_proj.id and ended up walking that edge as its LAST
  // edge, truncate it at projected_xy (don't walk past the projection
  // into the unwalked half of the edge, which is the half most likely
  // to contain obstacles). Skips __goal_edge_out__ below in that case.
  const bool last_edge_is_goal_edge =
    !search_result.edge_ids.empty() &&
    goal_proj.type == StartGoalProjector::ResultType::ON_EDGE &&
    search_result.edge_ids.back() == goal_proj.id;

  for (std::size_t i = 0; i < search_result.edge_ids.size(); ++i) {
    const std::string & eid = search_result.edge_ids[i];
    const std::string & from_id = search_result.node_ids[i];
    auto e_opt = graph.getEdge(eid);
    if (!e_opt) {continue;}
    const TopoEdge & e = *e_opt;
    const bool is_last = (i + 1 == search_result.edge_ids.size());
    const bool truncate_at_proj = is_last && last_edge_is_goal_edge;

    // Determine traversal direction.
    const bool reverse = (e.from_id != from_id);

    auto edge_pts = graph.sampleEdge(e, p_.bezier_samples_per_edge);
    if (edge_pts.size() < 2) {continue;}
    if (reverse) {
      std::reverse(edge_pts.begin(), edge_pts.end());
    }

    // Compute truncation index for the goal-edge fallback. The
    // `edge_sample_index` is in the ORIGINAL from→to indexing; if we
    // reversed it for traversal, flip the index too.
    std::size_t truncate_idx = edge_pts.size() - 1;
    if (truncate_at_proj) {
      const std::size_t orig_max = edge_pts.size() - 1;
      const std::size_t in_dir_idx = reverse
        ? (orig_max - std::min(goal_proj.edge_sample_index, orig_max))
        : std::min(goal_proj.edge_sample_index, orig_max);
      truncate_idx = in_dir_idx;
    }

    std::vector<geometry_msgs::msg::PoseStamped> seg_samples;
    seg_samples.push_back(
      makePose(edge_pts.front().first, edge_pts.front().second));
    for (std::size_t k = 1; k <= truncate_idx; ++k) {
      emitLinear(seg_samples,
        edge_pts[k - 1].first, edge_pts[k - 1].second,
        edge_pts[k].first, edge_pts[k].second);
    }
    if (truncate_at_proj) {
      // Snap exactly to projected_xy (truncate_idx is the rounded sample
      // neighbouring the projection).
      emitLinear(seg_samples,
        seg_samples.back().pose.position.x,
        seg_samples.back().pose.position.y,
        goal_proj.projected_xy.first, goal_proj.projected_xy.second);
    }

    InterpolatedSegment seg;
    seg.edge_id = e.id;
    seg.lane_subtype = e.lane_subtype;
    seg.max_speed = e.max_speed;
    seg.regulatory_element_ids = e.regulatory_element_ids;
    seg.floor = e.floor;
    seg.samples = std::move(seg_samples);
    plan.segments.push_back(std::move(seg));

    if (plan.floor_sequence.empty() || plan.floor_sequence.back() != e.floor) {
      plan.floor_sequence.push_back(e.floor);
      plan.floor_changes++;
    }
    if (truncate_at_proj && edge_pts.size() > 1) {
      const double frac = static_cast<double>(truncate_idx) /
        static_cast<double>(edge_pts.size() - 1);
      plan.total_distance += e.length * frac;
    } else {
      plan.total_distance += e.length;
    }
  }

  // ── Segment N-1: __goal_edge_out__ ─ straight from entry to projected_xy ─
  //
  // Emit ONE direct linear segment from the entry endpoint to
  // projected_xy. Walking through intermediate edge samples up to
  // `goal_proj.edge_sample_index` is unsafe: verticalProject reports
  // edge_sample_index = 1 for ANY interior projection on a 2-sample LINE
  // edge, which a walking loop misreads as "to-end" and produces
  // `from-end → to-end → projected_xy` (a U-turn that walks the FULL edge,
  // overshoots, and swings back). The direct emitLinear is bug-free for
  // LINE edges and an acceptable chord approximation for BEZIER edges
  // (this segment is Theta*-rewritable anyway, so topological correctness
  // matters more than geometric fidelity).
  // SKIPPED when A* already walked goal_proj.id (truncated above).
  if (goal_proj.type == StartGoalProjector::ResultType::ON_EDGE &&
      !last_edge_is_goal_edge)
  {
    // Walk along edge geometry, mirror of __start_edge_in__.
    // exit_node end → walk to projected_xy.
    //   • exit == from_id: forward — from-end → edge_pts[1..idx] → projected_xy
    //   • exit == to_id:   reverse — to-end   → edge_pts[N-2..idx+1] → projected_xy
    auto e_opt = graph.getEdge(goal_proj.id);
    if (e_opt) {
      auto edge_pts = graph.sampleEdge(*e_opt, p_.bezier_samples_per_edge);
      if (edge_pts.size() >= 2) {
        const bool exit_is_from = (e_opt->from_id == exit_node_id);
        InterpolatedSegment seg;
        seg.edge_id = "__goal_edge_out__";
        seg.lane_subtype = e_opt->lane_subtype;
        seg.max_speed = e_opt->max_speed;
        seg.floor = e_opt->floor;
        seg.samples.push_back(makePose(exit_opt->x, exit_opt->y));
        if (exit_is_from) {
          for (std::size_t i = 1;
            i <= goal_proj.edge_sample_index && i < edge_pts.size(); ++i)
          {
            emitLinear(seg.samples,
              seg.samples.back().pose.position.x,
              seg.samples.back().pose.position.y,
              edge_pts[i].first, edge_pts[i].second);
          }
        } else {
          const std::size_t N = edge_pts.size();
          for (std::size_t i = N - 1; i > goal_proj.edge_sample_index + 1; ) {
            --i;
            emitLinear(seg.samples,
              seg.samples.back().pose.position.x,
              seg.samples.back().pose.position.y,
              edge_pts[i].first, edge_pts[i].second);
          }
        }
        // Snap to projected_xy exactly.
        emitLinear(seg.samples,
          seg.samples.back().pose.position.x,
          seg.samples.back().pose.position.y,
          goal_proj.projected_xy.first, goal_proj.projected_xy.second);
        plan.segments.push_back(std::move(seg));
      }
    }
  }

  // ── Segment N: __goal_alignment__ ─ free-space, projected_xy → goal ─
  //
  // ALWAYS emitted. Caller is expected to Theta\*-rewrite this segment.
  {
    const double proj_x = (goal_proj.type == StartGoalProjector::ResultType::ON_EDGE)
      ? goal_proj.projected_xy.first  : exit_opt->x;
    const double proj_y = (goal_proj.type == StartGoalProjector::ResultType::ON_EDGE)
      ? goal_proj.projected_xy.second : exit_opt->y;

    InterpolatedSegment seg;
    seg.edge_id = "__goal_alignment__";
    seg.lane_subtype = "transition";
    seg.floor = exit_opt->floor;
    seg.samples.push_back(makePose(proj_x, proj_y));
    emitLinear(seg.samples, proj_x, proj_y,
      goal.pose.position.x, goal.pose.position.y);
    // Append the goal pose itself to terminate the path exactly.
    seg.samples.push_back(goal);
    plan.segments.push_back(std::move(seg));
  }

  finalizePlan(plan, start, goal);
  return plan;
}

InterpolatedRoutePlan PathInterpolator::interpolateSameEdge(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const StartGoalProjector::Result & start_proj,
  const StartGoalProjector::Result & goal_proj,
  const TopoGraph & graph) const
{
  InterpolatedRoutePlan plan;

  auto e_opt = graph.getEdge(start_proj.id);  // == goal_proj.id (precondition)
  if (!e_opt) {return plan;}
  auto edge_pts = graph.sampleEdge(*e_opt, p_.bezier_samples_per_edge);
  if (edge_pts.size() < 2) {return plan;}

  const std::size_t n_max = edge_pts.size() - 1;
  const std::size_t si = std::min(start_proj.edge_sample_index, n_max);
  const std::size_t gi = std::min(goal_proj.edge_sample_index, n_max);

  // ── Segment 1: __start_transition__ — free-space start → proj(start).
  // Straight chord here; the planner Theta\*-rewrites this front segment.
  {
    InterpolatedSegment seg;
    seg.edge_id = "__start_transition__";
    seg.lane_subtype = "transition";
    seg.floor = e_opt->floor;
    seg.samples.push_back(start);
    emitLinear(seg.samples,
      start.pose.position.x, start.pose.position.y,
      start_proj.projected_xy.first, start_proj.projected_xy.second);
    plan.segments.push_back(std::move(seg));
  }

  // ── Segment 2: walk the SAME edge from proj(start) → proj(goal). The
  // FLOOR semantic of edge_sample_index (last sample BEFORE the
  // projection along from→to) guarantees the first intermediate sample
  // sits strictly between the two projections, so the walk never
  // overshoots into the unwanted half of the edge.
  //   • forward  (si <= gi): proj(start) → edge_pts[si+1 .. gi] → proj(goal)
  //   • backward (si >  gi): proj(start) → edge_pts[si .. gi+1] → proj(goal)
  {
    InterpolatedSegment seg;
    seg.edge_id = e_opt->id;
    seg.lane_subtype = e_opt->lane_subtype;
    seg.max_speed = e_opt->max_speed;
    seg.regulatory_element_ids = e_opt->regulatory_element_ids;
    seg.floor = e_opt->floor;
    seg.samples.push_back(
      makePose(start_proj.projected_xy.first, start_proj.projected_xy.second));
    if (si <= gi) {
      for (std::size_t k = si + 1; k <= gi; ++k) {
        emitLinear(seg.samples,
          seg.samples.back().pose.position.x,
          seg.samples.back().pose.position.y,
          edge_pts[k].first, edge_pts[k].second);
      }
    } else {
      for (std::size_t k = si; k > gi; --k) {
        emitLinear(seg.samples,
          seg.samples.back().pose.position.x,
          seg.samples.back().pose.position.y,
          edge_pts[k].first, edge_pts[k].second);
      }
    }
    emitLinear(seg.samples,
      seg.samples.back().pose.position.x,
      seg.samples.back().pose.position.y,
      goal_proj.projected_xy.first, goal_proj.projected_xy.second);

    // Actual walked length along the road portion (diagnostic).
    for (std::size_t k = 1; k < seg.samples.size(); ++k) {
      plan.total_distance += std::hypot(
        seg.samples[k].pose.position.x - seg.samples[k - 1].pose.position.x,
        seg.samples[k].pose.position.y - seg.samples[k - 1].pose.position.y);
    }
    plan.segments.push_back(std::move(seg));
  }

  // ── Segment 3: __goal_alignment__ — free-space proj(goal) → goal.
  {
    InterpolatedSegment seg;
    seg.edge_id = "__goal_alignment__";
    seg.lane_subtype = "transition";
    seg.floor = e_opt->floor;
    seg.samples.push_back(
      makePose(goal_proj.projected_xy.first, goal_proj.projected_xy.second));
    emitLinear(seg.samples,
      goal_proj.projected_xy.first, goal_proj.projected_xy.second,
      goal.pose.position.x, goal.pose.position.y);
    seg.samples.push_back(goal);
    plan.segments.push_back(std::move(seg));
  }

  plan.floor_sequence.push_back(e_opt->floor);

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
