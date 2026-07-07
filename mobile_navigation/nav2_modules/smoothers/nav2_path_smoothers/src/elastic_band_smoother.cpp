// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#include "nav2_path_smoothers/elastic_band_smoother.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_path_smoothers/smoother_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_path_smoothers
{

void ElasticBandSmoother::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer>/*tf*/,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>/*footprint_sub*/)
{
  auto node = parent.lock();
  logger_ = node->get_logger();
  costmap_sub_ = costmap_sub;

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_iterations", rclcpp::ParameterValue(100));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".w_smooth", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".w_obstacle", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".step_size", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".obstacle_cost_threshold", rclcpp::ParameterValue(128));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".convergence_tol", rclcpp::ParameterValue(1e-4));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".reject_on_collision", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".max_iterations", max_iterations_);
  node->get_parameter(name + ".w_smooth", w_smooth_);
  node->get_parameter(name + ".w_obstacle", w_obstacle_);
  node->get_parameter(name + ".step_size", step_size_);
  node->get_parameter(name + ".obstacle_cost_threshold", obstacle_cost_threshold_);
  node->get_parameter(name + ".convergence_tol", convergence_tol_);
  node->get_parameter(name + ".reject_on_collision", reject_on_collision_);

  RCLCPP_INFO(
    logger_,
    "ElasticBandSmoother configured: iters=%d w_smooth=%.2f w_obstacle=%.2f step=%.2f "
    "obstacle_threshold=%d", max_iterations_, w_smooth_, w_obstacle_, step_size_,
    obstacle_cost_threshold_);
}

bool ElasticBandSmoother::smooth(nav_msgs::msg::Path & path, const rclcpp::Duration & max_time)
{
  const auto t0 = std::chrono::steady_clock::now();
  const auto deadline = std::chrono::duration<double>(max_time.seconds());
  const size_t n = path.poses.size();
  if (n < 3) {
    return true;
  }

  // costmap 不可用时退化为纯弹性平滑（无障碍斥力、无碰撞校验）。
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
  if (costmap_sub_) {
    try {
      costmap = costmap_sub_->getCostmap();
    } catch (const std::exception &) {
      RCLCPP_WARN_ONCE(logger_, "ElasticBandSmoother: costmap unavailable, elastic-only mode");
    }
  }
  const double res = costmap ? costmap->getResolution() : 0.05;

  const auto cost_at = [&](double x, double y) -> double {
      unsigned int mx, my;
      if (!costmap->worldToMap(x, y, mx, my)) {
        return 0.0;
      }
      const unsigned char c = costmap->getCost(mx, my);
      return c == nav2_costmap_2d::NO_INFORMATION ? 0.0 : static_cast<double>(c);
    };

  const nav_msgs::msg::Path original = path;
  std::vector<double> xs(n), ys(n);
  for (size_t i = 0; i < n; ++i) {
    xs[i] = path.poses[i].pose.position.x;
    ys[i] = path.poses[i].pose.position.y;
  }

  bool time_ok = true;
  for (int it = 0; it < max_iterations_; ++it) {
    double max_disp = 0.0;
    for (size_t i = 1; i + 1 < n; ++i) {
      // 内张力：拉向前后邻点中点。
      double fx = w_smooth_ * (0.5 * (xs[i - 1] + xs[i + 1]) - xs[i]);
      double fy = w_smooth_ * (0.5 * (ys[i - 1] + ys[i + 1]) - ys[i]);
      // 障碍斥力：代价超阈值时沿数值梯度反方向推离，幅值随代价占比缩放。
      if (costmap) {
        const double c0 = cost_at(xs[i], ys[i]);
        if (c0 > obstacle_cost_threshold_) {
          const double gx = (cost_at(xs[i] + res, ys[i]) - cost_at(xs[i] - res, ys[i])) / 2.0;
          const double gy = (cost_at(xs[i], ys[i] + res) - cost_at(xs[i], ys[i] - res)) / 2.0;
          const double gn = std::hypot(gx, gy);
          if (gn > 1e-9) {
            const double mag = (c0 - obstacle_cost_threshold_) /
              (nav2_costmap_2d::LETHAL_OBSTACLE - obstacle_cost_threshold_);
            fx += w_obstacle_ * mag * res * (-gx / gn);
            fy += w_obstacle_ * mag * res * (-gy / gn);
          }
        }
      }
      xs[i] += step_size_ * fx;
      ys[i] += step_size_ * fy;
      max_disp = std::max(max_disp, std::hypot(step_size_ * fx, step_size_ * fy));
    }
    if (max_disp < convergence_tol_) {
      break;
    }
    if (std::chrono::steady_clock::now() - t0 > deadline) {
      time_ok = false;
      break;
    }
  }

  // 碰撞防护：平滑把点挪进障碍则整条弃用。
  if (reject_on_collision_ && costmap) {
    for (size_t i = 0; i < n; ++i) {
      if (cost_at(xs[i], ys[i]) >= nav2_costmap_2d::LETHAL_OBSTACLE) {
        RCLCPP_WARN(
          logger_, "ElasticBandSmoother: smoothed path hits lethal cost, keeping original");
        path = original;
        return true;
      }
    }
  }

  for (size_t i = 1; i + 1 < n; ++i) {
    path.poses[i].pose.position.x = xs[i];
    path.poses[i].pose.position.y = ys[i];
  }
  recomputeOrientations(path);
  return time_ok;
}

}  // namespace nav2_path_smoothers

PLUGINLIB_EXPORT_CLASS(nav2_path_smoothers::ElasticBandSmoother, nav2_core::Smoother)
