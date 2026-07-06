// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// T 型（梯形/恒加速度）速度规划器实现：可达性回退 + 加速段 + 减速段。
#include "nav2_speed_planner/trapezoidal_speed_planner.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace nav2_speed_planner
{

void TrapezoidalSpeedPlanner::assignVelocityProfile(
  std::vector<TrajPointWork> & work, double curr_vel, double cruise_limit)
{
  if (work.size() < 2) {
    return;
  }
  if (cruise_limit < 1e-3) {
    // 全部点被安全置零（起点即阻塞）：保持零速剖面。
    return;
  }

  bool has_zero = false;
  for (const auto & p : work) {
    if (p.speed_limit < 1e-3) {
      has_zero = true;
      break;
    }
  }
  const double curr_min_speed = has_zero ? 0.0 : min_speed_;

  // 可达性回退（receding-horizon：基于当前速度）：加减速所需距离超过有效长度时，
  // 解三角形剖面峰值速度，并把剖面上限收紧到峰值（min，保留曲率/蠕行等更严约束）。
  double speed_limit = cruise_limit;
  const double accel_path_length = computeAccelLength(curr_vel, speed_limit);
  const double decel_path_length = computeDecelLength(curr_vel, curr_min_speed);
  const double path_length = computeValidSpeedPathLength(work);
  const double no_stop_length = std::max(path_length - stop_distance_, 0.0);
  if (accel_path_length + decel_path_length > no_stop_length) {
    speed_limit = std::sqrt(
      (2 * accel_ * decel_ * no_stop_length + decel_ * curr_vel * curr_vel -
      accel_ * curr_min_speed * curr_min_speed) / (decel_ - accel_));
    for (auto & p : work) {
      if (p.speed_limit >= 1e-3) {
        p.speed_limit = std::min(p.speed_limit, speed_limit);
      }
    }
  }

  computeAccelPlan(work, curr_vel, speed_limit);   // B1 加速段
  computeDecelPlan(work, curr_vel, speed_limit);   // B2 减速段
}

// ============================ B1 加速段 ============================

void TrapezoidalSpeedPlanner::computeAccelPlan(
  std::vector<TrajPointWork> & path, const double curr_vel, const double speed_limit)
{
  const double accel_path_length = computeAccelLength(curr_vel, speed_limit);
  // 安全置零绝对优先：起点被置零（阻塞）时不得抬升。
  if (path[0].speed_limit >= 1e-3) {
    path[0].speed_limit = curr_vel;
  }
  const std::vector<double> arc_s = computeCumulativeArcLengths(path);
  for (size_t i = 1; i < path.size(); i++) {
    // 当前路径点距离起点的沿路弧长。
    const double s_t = arc_s[i];
    if (s_t > accel_path_length) {
      break;
    }
    // 按当前速度 + 加速度，到达该点所需时间，再求速度。
    double t = (-curr_vel + std::sqrt(curr_vel * curr_vel + 2 * accel_ * s_t)) / accel_;
    double v = curr_vel + accel_ * t;
    if (v > speed_limit) {
      v = speed_limit;
    }
    // 只降不升：保留曲率/障碍/蠕行等已有更严约束。
    path[i].speed_limit = std::min(path[i].speed_limit, v);
  }
}

// ============================ B2 减速段 ============================

void TrapezoidalSpeedPlanner::computeDecelPlan(
  std::vector<TrajPointWork> & path, const double curr_vel, const double speed_limit)
{
  // 减速路径计算从后往前。
  const std::vector<double> arc_s = computeCumulativeArcLengths(path);

  // 末端减速：锚定距终点 stop_distance_ 处降到 end_speed_（蠕行区包络由 A4 保证）；
  // end_speed_ < 1e-3 时无蠕行区，减速曲线在终点收敛到 0。写入只降不升。
  const double v_end = std::max(end_speed_, 0.0);
  if (stop_distance_ > 0 || v_end < 1e-3) {
    size_t first_no_stop_index = path.size() - 1;
    if (v_end >= 1e-3 && stop_distance_ > 0) {
      for (size_t i = path.size() - 1; i > 0; i--) {
        if (arc_s.back() - arc_s[i] > stop_distance_) {
          first_no_stop_index = std::min(i + 1, path.size() - 1);
          break;
        }
      }
    } else {
      path[first_no_stop_index].speed_limit =
        std::min(path[first_no_stop_index].speed_limit, v_end);
    }
    double stop_decel_path_length = computeDecelLength(curr_vel, v_end);
    // index 0 是机器人当前速度，不参与设置。
    for (size_t i = first_no_stop_index; i > 0; i--) {
      const double s_t = arc_s[first_no_stop_index] - arc_s[i];
      double v = std::sqrt(v_end * v_end - 2 * decel_ * s_t);
      if (v > speed_limit) {
        v = speed_limit;
      }
      path[i].speed_limit = std::min(path[i].speed_limit, v);
      if (s_t > stop_decel_path_length) {
        break;
      }
    }
  }

  // 曲线前减速：以第一个小半径点作为目标速度点，向前减速（曲率已由 Phase A3 写入）。
  if ((curr_path_length_ - stop_distance_) > 0) {
    size_t first_curve_decel_index = path.size() - 1;
    bool has_curve_decel = false;
    for (size_t i = 0; i < path.size(); i++) {
      const double radius = std::fabs(1.0 / path[i].curvature);
      const double & min_rad = regulated_linear_scaling_min_radius_;
      if (radius > min_rad) {
        continue;
      }
      has_curve_decel = true;
      first_curve_decel_index = i;
      break;
    }
    if (has_curve_decel) {
      double curr_min_speed = min_curvature_speed_;
      double curve_decel_path_length = computeDecelLength(curr_vel, curr_min_speed);
      for (int i = static_cast<int>(first_curve_decel_index); i > 0; i--) {
        const double s_t = arc_s[first_curve_decel_index] - arc_s[i];
        double v = std::sqrt(curr_min_speed * curr_min_speed - 2 * decel_ * s_t);
        if (v > speed_limit) {
          v = speed_limit;
        }
        path[i].speed_limit = std::min(path[i].speed_limit, v);  // 只降不升
        if (s_t > curve_decel_path_length) {
          break;
        }
      }
    }
  }

  // 零速点前减速过渡：障碍与缓冲带已由 A1 置零，此处只做从首个零速点向前的减速曲线；
  // 末端硬停的零点复用同一原语（min 语义下与末端减速结果一致）。
  size_t first_zero_index = path.size();
  for (size_t i = 1; i < path.size(); ++i) {
    if (path[i].speed_limit < 1e-3) {
      first_zero_index = i;
      break;
    }
  }
  if (first_zero_index < path.size()) {
    double curr_min_speed = 0.0;
    double collision_decel_path_length = computeDecelLength(curr_vel, curr_min_speed);

    for (size_t i = first_zero_index; i > 0; i--) {
      const double s_t = arc_s[first_zero_index] - arc_s[i];
      double v = std::sqrt(curr_min_speed * curr_min_speed - 2 * decel_ * s_t);
      if (v > speed_limit) {
        v = speed_limit;
      }
      path[i].speed_limit = std::min(path[i].speed_limit, v);  // 只降不升
      if (s_t > collision_decel_path_length) {
        break;
      }
    }
  }
}

// ============================ 加减速距离 ============================

double TrapezoidalSpeedPlanner::computeAccelLength(
  const double curr_vel, const double speed_limit) const
{
  if (curr_vel >= speed_limit) {  // 不需要加速
    return 0.0;
  }
  double t = (speed_limit - curr_vel) / accel_;
  double s = curr_vel * t + 0.5 * accel_ * t * t;
  return s;
}

double TrapezoidalSpeedPlanner::computeDecelLength(
  const double curr_vel, const double min_speed) const
{
  if (curr_vel <= min_speed) {  // 不需要减速
    return 0.0;
  }
  double t = (min_speed - curr_vel) / decel_;
  double s = curr_vel * t + 0.5 * decel_ * t * t;
  return s;
}

double TrapezoidalSpeedPlanner::computeValidSpeedPathLength(
  const std::vector<TrajPointWork> & path)
{
  double length = 0.0;
  for (int i = 1; i < static_cast<int>(path.size()); ++i) {
    double dx = path.at(i).pose.pose.position.x - path.at(i - 1).pose.pose.position.x;
    double dy = path.at(i).pose.pose.position.y - path.at(i - 1).pose.pose.position.y;
    length += std::sqrt(dx * dx + dy * dy);

    // 第一个零速点也包含在路径长度中。
    if (path.at(i).speed_limit == 0.0) {
      break;
    }
  }
  return length;
}

}  // namespace nav2_speed_planner

PLUGINLIB_EXPORT_CLASS(
  nav2_speed_planner::TrapezoidalSpeedPlanner, nav2_core::SpeedPlanner)
