// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// A1 障碍约束：逐点查询 costmap、置零障碍点与其前方缓冲带，并显式区分
// 「无障碍」与「无法评估」（TF/costmap 不可用）两种结果。
//
// 本文件是 SpeedPlannerBase 的一部分（同类跨编译单元），签名与成员访问
// 与拆分前完全一致。
#include "nav2_speed_planner/speed_planner_base.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"

#include "constraints/planner_internal.hpp"

using namespace nav2_speed_planner::internal;  // NOLINT(build/namespaces)

namespace nav2_speed_planner
{

ObstacleConstraintOutcome SpeedPlannerBase::produceObstacleConstraint(
  const std::vector<TrajPointWork> & path)
{
  ObstacleConstraintOutcome outcome;
  outcome.constraint = VelocityConstraint(path.size());
  using TimingClock = std::chrono::steady_clock;
  const auto total_start = TimingClock::now();
  last_obstacle_timing_ = ObstacleConstraintTiming();
  const auto finish_timing = [&]() {
      last_obstacle_timing_.total_ms = std::chrono::duration<double, std::milli>(
        TimingClock::now() - total_start).count();
    };

  // A1 障碍约束（安全置零绝对优先）：首个障碍点及其后全部置零，
  // 障碍前 obstacle_zero_buffer_ 缓冲带置零（index 0 为机器人当前位置，不动）。
  if (path.empty()) {
    finish_timing();
    return outcome;  // 空路径：可评估，无障碍
  }

  // 路径与 costmap 坐标系对齐：FollowPath 路径通常在 map 系，而宿主的 local
  // costmap 在 odom 系——直接用路径坐标查询会相差 map→odom 修正量，把自由区
  // 路径按偏移量"平移"进墙带造成误停障。查询前统一变换到 costmap 系
  // （2D 刚体变换，每周期取一次 TF，逐点仅乘加）。
  double tx = 0.0, ty = 0.0, cos_t = 1.0, sin_t = 0.0, yaw_offset = 0.0;
  bool need_transform = false;
  std::string costmap_frame;
  if (costmap_ros_) {
    costmap_frame = costmap_ros_->getGlobalFrameID();
  }
  if (costmap_ros_ && tf_) {
    if (!path_frame_.empty() && !costmap_frame.empty() && path_frame_ != costmap_frame) {
      last_obstacle_timing_.transform_required = true;
      geometry_msgs::msg::TransformStamped to_costmap;
      const auto tf_start = TimingClock::now();
      try {
        to_costmap = tf_->lookupTransform(costmap_frame, path_frame_, tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        last_obstacle_timing_.tf_ms = std::chrono::duration<double, std::milli>(
          TimingClock::now() - tf_start).count();
        // 变换不可用（如启动期 map 尚未发布）：本周期跳过障碍约束，
        // 好过用错误坐标误停障；下一周期重试（receding-horizon）。
        if (clock_) {
          RCLCPP_WARN_THROTTLE(
            logger_, *clock_, 1000,
            "[%s] cannot transform path (%s) into costmap frame (%s) after %.3f ms: %s",
            plugin_name_.c_str(), path_frame_.c_str(), costmap_frame.c_str(),
            last_obstacle_timing_.tf_ms, ex.what());
        }
        finish_timing();
        // ⚠️ 关键改动：TF 失败是"没法查"，不是"没障碍"。此前两者同为 return false，
        // 调用方无法区分 → 隐式 FAIL_OPEN（该周期无任何障碍约束仍按巡航速度行驶）。
        outcome.evaluable = false;
        outcome.unavailable_reason = "tf_transform_failed";
        return outcome;
      }
      last_obstacle_timing_.tf_ms = std::chrono::duration<double, std::milli>(
        TimingClock::now() - tf_start).count();
      const double yaw = tf2::getYaw(to_costmap.transform.rotation);
      cos_t = std::cos(yaw);
      sin_t = std::sin(yaw);
      tx = to_costmap.transform.translation.x;
      ty = to_costmap.transform.translation.y;
      yaw_offset = yaw;
      need_transform = true;
    }
  }

  // costmap 缺失同样是"没法查"。整轮判一次，不逐点判——逐点判会把它和
  // OUT_OF_RANGE（正常的覆盖范围外）混淆。
  if (!isObstacleQueryAvailable()) {
    finish_timing();
    outcome.evaluable = false;
    outcome.unavailable_reason = "costmap_unavailable";
    return outcome;
  }

  const auto scan_start = TimingClock::now();
  const std::vector<double> arc_s = computeCumulativeArcLengths(path);
  // 全路径扫描并用 region_type 标记障碍语义（透传到 RobotTrajectory，供消费方与
  // 剖面日志评判使用；M11 真语义接入后此处改为叠加标记而非独占）：
  //   obstacle        — path 点真正落在障碍格内（触发停障）
  //   obstacle_exempt — 起点豁免区内的障碍命中（仅标记不置零：贴墙停放防起步死锁）
  //   obstacle_buffer — 首个障碍前的缓冲带（置零）
  size_t first_obstacle_index = path.size();
  for (size_t i = 0; i < path.size(); ++i) {
    double wx = path[i].pose.pose.position.x;
    double wy = path[i].pose.pose.position.y;
    // R47：footprint 查询需要姿态；路径点朝向随路径坐标系一起变换到 costmap 系。
    double wyaw = tf2::getYaw(path[i].pose.pose.orientation);
    if (need_transform) {
      const double px = wx;
      const double py = wy;
      wx = cos_t * px - sin_t * py + tx;
      wy = sin_t * px + cos_t * py + ty;
      wyaw += yaw_offset;
    }
    const PointObstacleStatus point_status = checkObstacleAt(wx, wy, wyaw);
    if (point_status == PointObstacleStatus::OUT_OF_RANGE) {
      // 超出 costmap 覆盖：无信息，不判障、不触发策略，只计数供观测。
      ++outcome.out_of_range_points;
      continue;
    }
    ++outcome.scanned_points;
    if (point_status == PointObstacleStatus::FREE) {
      continue;
    }
    if (arc_s[i] <= obstacle_start_exempt_distance_) {
      outcome.constraint.region_type[i] = "obstacle_exempt";
      continue;
    }
    outcome.constraint.region_type[i] = "obstacle";
    last_planning_diagnostics_.first_obstacle_index =
    (first_obstacle_index == path.size()) ? -1 : static_cast<int>(first_obstacle_index);
  last_planning_diagnostics_.first_obstacle_arc =
    (first_obstacle_index == path.size() || first_obstacle_index >= arc_s.size()) ?
    -1.0 : arc_s[first_obstacle_index];
  if (first_obstacle_index == path.size()) {
      first_obstacle_index = i;
    }
  }
  last_obstacle_timing_.scan_ms = std::chrono::duration<double, std::milli>(
    TimingClock::now() - scan_start).count();
  if (first_obstacle_index == path.size()) {
    finish_timing();
    return outcome;  // 查过了，确实没有障碍
  }
  const auto apply_start = TimingClock::now();
  for (size_t i = first_obstacle_index; i < path.size(); ++i) {
    outcome.constraint.set(i, 0.0, ConstraintSource::kObstacle);
  }
  const double buffer_start = arc_s[first_obstacle_index] - obstacle_zero_buffer_;
  for (size_t j = first_obstacle_index; j-- > 1; ) {
    if (arc_s[j] <= buffer_start) {
      break;
    }
    outcome.constraint.set(j, 0.0, ConstraintSource::kObstacleBuffer);
    if (outcome.constraint.region_type[j].empty()) {
      outcome.constraint.region_type[j] = "obstacle_buffer";   // 障碍点本身的标注优先
    }
  }
  last_obstacle_timing_.apply_ms = std::chrono::duration<double, std::milli>(
    TimingClock::now() - apply_start).count();
  finish_timing();
  outcome.has_obstacle = true;
  return outcome;
}


double SpeedPlannerBase::footprintCostAt(double x, double y, double theta) const
{
  // R47：原实现是路径中心线单点 costmap 格值查询，和 RPP 的 footprint 查询口径不一致——
  // 障碍贴中心线两侧、不压中心线本身时（拐弯贴墙、货架棱角）会被漏检，只能靠 RPP
  // 的 isCollisionImminent 在很近的距离二值急停。现与 RPP 同源做 footprint 查询。
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_ ? costmap_ros_->getCostmap() : nullptr;
  if (costmap == nullptr || !collision_checker_) {
    return kFootprintCostUnavailable;
  }
  unsigned int mx, my;
  if (!costmap->worldToMap(x, y, mx, my)) {
    return kFootprintCostUnavailable;
  }

  std::vector<geometry_msgs::msg::Point> footprint = costmap_ros_->getRobotFootprint();
  if (footprint.size() < 3) {
    return kFootprintCostUnavailable;
  }
  std::vector<geometry_msgs::msg::Point> oriented;
  nav2_costmap_2d::transformFootprint(x, y, theta, footprint, oriented);

  // ⚠️ 幻影障碍防护（R47 回归修复）：nav2 的 FootprintCollisionChecker::footprintCost
  // 在任一顶点 worldToMap 失败时**直接返回 LETHAL_OBSTACLE**。local costmap 为 8×8
  // rolling（±4m），footprint 半对角约 0.495m，于是路径上超过约 3.5m 的点每周期都会
  // 因顶点出图被判成障碍。出图属于"无信息"而非"有障碍"，这里显式判定并按不可用返回
  // （与 R47 之前单点查询 worldToMap 失败即 false 的语义一致）。
  for (const auto & point : oriented) {
    unsigned int px, py;
    if (!costmap->worldToMap(point.x, point.y, px, py)) {
      return kFootprintCostUnavailable;
    }
  }

  collision_checker_->setCostmap(costmap);
  return collision_checker_->footprintCost(oriented);
}


bool SpeedPlannerBase::isObstacleQueryAvailable() const
{
  return costmap_ros_ != nullptr && costmap_ros_->getCostmap() != nullptr;
}


PointObstacleStatus SpeedPlannerBase::checkObstacleAt(
  double x, double y, double theta) const
{
  const double footprint_cost = footprintCostAt(x, y, theta);
  if (footprint_cost == kFootprintCostUnavailable) {
    // 位姿或 footprint 顶点越出 costmap → 该点无信息。
    // 这不是"约束不可评估"，只是这一点在感知范围外（rolling window 只有 ±4m），
    // 属正常情况；若按不可评估处理并 FAIL_CLOSED，长于约 3.5m 的路径会全程零速。
    return PointObstacleStatus::OUT_OF_RANGE;
  }
  if (footprint_cost == static_cast<double>(nav2_costmap_2d::NO_INFORMATION)) {
    return PointObstacleStatus::OUT_OF_RANGE;  // 未知区域同样无信息，不判障
  }
  return footprint_cost >= static_cast<double>(obstacle_cost_threshold_) ?
         PointObstacleStatus::OBSTACLE : PointObstacleStatus::FREE;
}



}  // namespace nav2_speed_planner
