// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// A3 曲率 / 横向加速度约束：`v ≤ √(a_lat_max / |κ|)`。
//
// 旧式为 `v = cruise × radius / min_radius`——同一半径的弯道在不同巡航速度下
// 被限到不同速度，实测弯道最低速随外部限速在 [0.125, 0.417] 漂移 3.3 倍
//（横向加速度 0.031 → 0.347 m/s²）。它还是"A3 必须排在 A2 巡航基准之后"这条
// 顺序依赖的唯一来源，故入参 `cruise_reference` 随公式一并移除。
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

VelocityConstraint SpeedPlannerBase::produceLateralAccelConstraint(
  const std::vector<TrajPointWork> & path,
  const std::vector<double> & curvature_vec) const
{
  VelocityConstraint c(path.size());
  // 全 kUnconstrained 的产出 = 本约束不参与合并，`applyConstraint` 逐点跳过。
  // 这是显式关闭，区别于把 `min_radius` 设成极小值让判据恒不触发。
  if (!enable_curvature_constraint_ || path.size() < 3) {
    return c;
  }

  const std::vector<TrajPointWork> & output = path;

  // 对路径上每个点：写入曲率，并按小半径处限速。
  std::vector<int> filtered_points;
  for (size_t i = 0; i < output.size(); ++i) {
    const double radius = std::fabs(1.0 / curvature_vec.at(i));
    // `min_radius` 只作**激活门**（多小才算弯道），不再参与限速值的计算。
    if (radius > regulated_linear_scaling_min_radius_) {
      continue;
    }
    // 物理式：向心加速度 v²/r ≤ a_lat_max。与巡航速度、与外部限速均无关。
    double v_curvature_max = std::sqrt(max_lateral_accel_ * radius);

    // 如果该点速度已经小于阈值，则不再要求其大于阈值。
    v_curvature_max = std::max(v_curvature_max, min_curvature_speed_);
    if (output.at(i).velocity_upper_bound < v_curvature_max) {
      continue;
    }

    // 将速度限幅在 v_curvature_max 之下。
    if (output.at(i).velocity_upper_bound > v_curvature_max) {
      c.set(i, v_curvature_max, ConstraintSource::kLateralAccel);
      c.region_type[i] = "curve";   // 语义标注：曲率限速段（锚点可视化/日志）
      filtered_points.push_back(static_cast<int>(i));
    }
  }

  // 转弯保持匀速：把相近的限速点聚成区间，区间内统一取最小速度。
  const double dist_threshold = constant_velocity_dist_threshold_;
  std::vector<std::tuple<size_t, size_t, double>> latacc_filtered_ranges;
  size_t start_index = 0;
  size_t end_index = 0;
  bool is_updated = false;
  double min_latacc_velocity = 0.0;
  for (size_t i = 0; i < filtered_points.size(); ++i) {
    const size_t index = static_cast<size_t>(filtered_points.at(i));

    if (is_updated == false) {
      start_index = index;
      end_index = index;
      min_latacc_velocity = c.upper_bound[index];
      is_updated = true;
      continue;
    }
    // 将 start_index 后 dist_threshold 内的点归入同一区间。
    double seg_dist = std::sqrt(
      std::pow(
        output.at(end_index).pose.pose.position.x - output.at(index).pose.pose.position.x, 2) +
      std::pow(
        output.at(end_index).pose.pose.position.y - output.at(index).pose.pose.position.y, 2));
    if (seg_dist < dist_threshold) {
      end_index = index;
      min_latacc_velocity = std::min(c.upper_bound[index], min_latacc_velocity);
    } else {
      latacc_filtered_ranges.emplace_back(start_index, end_index, min_latacc_velocity);
      start_index = index;
      end_index = index;
      min_latacc_velocity = c.upper_bound[index];
    }
  }
  if (is_updated) {
    latacc_filtered_ranges.emplace_back(start_index, end_index, min_latacc_velocity);
  }

  for (size_t i = 0; i < output.size(); ++i) {
    for (const auto & lat_acc_filtered_range : latacc_filtered_ranges) {
      const size_t s_idx = std::get<0>(lat_acc_filtered_range);
      const size_t e_idx = std::get<1>(lat_acc_filtered_range);
      const double min_v = std::get<2>(lat_acc_filtered_range);

      if (s_idx <= i && i <= e_idx && enable_constant_velocity_while_turning_) {
        // 只降不升：区间内安全置零点/更严约束不得被抬升到 min_v。
        c.set(i, std::min(c.upper_bound[i], min_v), ConstraintSource::kLateralAccel);
        break;
      }
    }
  }

  // O-6 曲率约束前后延伸（独立于 enable_constant_velocity_while_turning_）：
  // 弯道本身的进入坡道已由 Phase B 的 backward pass 生成，但那是"物理上刚好
  // 够减速"的最晚时刻——零安全边际。这里让弯前 curve_lead_in_distance_、
  // 弯后 curve_lead_out_distance_ 弧长内也直接钳到该弯的限速，提前量成为
  // Phase B 减速坡道的显式起点，给跟踪滞后（R22）留边际，也让转弯前的
  // 减速更从容。沿用与主循环相同的 region_type="curve" 标注，RViz 锚点
  // （curve_start/curve_end）与日志自动反映延伸后的真实范围，无需改动。
  if ((curve_lead_in_distance_ > 1e-6 || curve_lead_out_distance_ > 1e-6) &&
    !latacc_filtered_ranges.empty())
  {
    const std::vector<double> arc_s = computeCumulativeArcLengths(path);
    for (const auto & range : latacc_filtered_ranges) {
      const size_t s_idx = std::get<0>(range);
      const size_t e_idx = std::get<1>(range);
      const double min_v = std::get<2>(range);

      if (curve_lead_in_distance_ > 1e-6) {
        const double lead_in_start_s = arc_s[s_idx] - curve_lead_in_distance_;
        size_t i = s_idx;
        while (i-- > 1) {  // index 0 为状态戳，不参与（同 O-2 backward pass 约定）
          if (arc_s[i] < lead_in_start_s) {
            break;
          }
          c.set(i, std::min(c.upper_bound[i], min_v), ConstraintSource::kLateralAccel);
          c.region_type[i] = "curve";
        }
      }
      if (curve_lead_out_distance_ > 1e-6) {
        const double lead_out_end_s = arc_s[e_idx] + curve_lead_out_distance_;
        for (size_t i = e_idx + 1; i < output.size(); ++i) {
          if (arc_s[i] > lead_out_end_s) {
            break;
          }
          c.set(i, std::min(c.upper_bound[i], min_v), ConstraintSource::kLateralAccel);
          c.region_type[i] = "curve";
        }
      }
    }
  }
  return c;
}


std::vector<double> SpeedPlannerBase::calcTrajectoryCurvatureFrom3Points(
  const std::vector<TrajPointWork> & path, double compute_dist) const
{
  // ⚠ **按弧长取三点的尝试已回退（2026-08-07）。**
  //
  // 动机看似严密：实机全局路径点距 min 2.56 / p50 41.9 / max 70.5 mm
  //（27 倍跨度，177 个样本），而本函数原用 points_interval_（**平均**点距）
  // 换算 idx_dist，隐含等距假设 → 推断密集处三点跨度过小会数值退化。
  //
  // 但离线对拍**否定了这个推断**：新增「弯道中段插入密集点簇」场景后，
  //   密集点距 3 mm ：弯道段最低速 0.10734 → **0.05000**（= end_speed，更差）
  //   密集点距 10 mm：0.05000 → 0.05000（无改善）
  //   密集点距 20 mm：0.25002 → 0.25002（无变化）
  //   求解器失败：11 → **12**
  // 按弧长回溯 0.1 m 时会跨越「密集簇 ↔ 正常段」边界，拟合几何反而更不一致
  //（**此解释未经验证**，与本轮其它未验证推断同等不可信）。
  //
  // 教训：从代码读出机制 → 推理它导致故障 → **不验证就实现**，本轮连续三次
  // 得到无效或更差的结果（包络可达化 / 曲率排除状态戳 / 本次）。真正有效的两次
  //（C1 逐点限速、目标双向 clamp）都是**先有离线可复现场景，再改代码**。
  //
  // 恢复按索引实现。不等距是否真的致病，待 ③ 侧验证器建成后重新判定。
  const size_t n = path.size();
  std::vector<double> k_arr(n, 0.0);
  if (n < 3) {
    return k_arr;
  }
  compute_dist = std::max(compute_dist, 1e-3);
  const size_t max_idx_dist = static_cast<size_t>(std::floor((n - 1) / 2.0));
  const size_t idx_dist = std::max<size_t>(
    1, std::min(
      max_idx_dist,
      static_cast<size_t>(std::round(compute_dist / std::max(points_interval_, 1e-6)))));

  for (size_t i = 1; i + 1 < n; i++) {
    double curvature = 0.0;
    try {
      curvature = calcCurvature(
        path.at(i - std::min(idx_dist, i)).pose.pose, path.at(i).pose.pose,
        path.at(i + std::min(idx_dist, n - 1 - i)).pose.pose);
    } catch (std::exception const & e) {
      if (clock_) {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "%s", e.what());
      }
      curvature = (i > 1) ? k_arr.at(i - 1) : 0.0;
    }
    k_arr.at(i) = curvature;
  }
  return k_arr;
}


double SpeedPlannerBase::calcCurvature(
  const geometry_msgs::msg::Pose & p1,
  const geometry_msgs::msg::Pose & p2,
  const geometry_msgs::msg::Pose & p3)
{
  // Menger 曲率：https://en.wikipedia.org/wiki/Menger_curvature
  auto dist = [](const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b) {
      double dx = a.position.x - b.position.x;
      double dy = a.position.y - b.position.y;
      return std::sqrt(dx * dx + dy * dy);
    };
  const double denominator = dist(p1, p2) * dist(p2, p3) * dist(p3, p1);
  if (std::fabs(denominator) < 1e-10) {
    throw std::runtime_error("points are too close for curvature calculation.");
  }
  return 2.0 *
         ((p2.position.x - p1.position.x) * (p3.position.y - p1.position.y) -
         (p2.position.y - p1.position.y) * (p3.position.x - p1.position.x)) / denominator;
}


}  // namespace nav2_speed_planner
