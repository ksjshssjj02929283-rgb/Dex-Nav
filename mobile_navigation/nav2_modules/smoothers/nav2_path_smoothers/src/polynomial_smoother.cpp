// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#include "nav2_path_smoothers/polynomial_smoother.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_path_smoothers/smoother_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_path_smoothers
{

void PolynomialSmoother::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer>/*tf*/,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>/*footprint_sub*/)
{
  auto node = parent.lock();
  logger_ = node->get_logger();
  costmap_sub_ = costmap_sub;

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".poly_order", rclcpp::ParameterValue(5));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".window_size", rclcpp::ParameterValue(15));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".reject_on_collision", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".poly_order", poly_order_);
  node->get_parameter(name + ".window_size", window_size_);
  node->get_parameter(name + ".reject_on_collision", reject_on_collision_);

  poly_order_ = std::max(2, std::min(poly_order_, 5));
  // 窗口须为奇数且至少 order+2（最小二乘超定）。
  window_size_ = std::max(window_size_, poly_order_ + 2);
  if (window_size_ % 2 == 0) {
    ++window_size_;
  }
  coeff_cache_.clear();
  RCLCPP_INFO(
    logger_, "PolynomialSmoother configured: order=%d window=%d reject_on_collision=%s",
    poly_order_, window_size_, reject_on_collision_ ? "true" : "false");
}

const std::vector<double> & PolynomialSmoother::sgCoefficients(int half_window)
{
  auto it = coeff_cache_.find(half_window);
  if (it != coeff_cache_.end()) {
    return it->second;
  }
  // 求中心点 SG 系数：A_{jk} = j^k（j∈[-h,h]，k∈[0,order]），
  // c = A (AᵀA)⁻¹ e₀ —— 对 M z = e₀ 做带主元高斯消元（M ≤ 6×6）。
  const int h = half_window;
  const int m = 2 * h + 1;
  const int k = std::min(poly_order_, m - 2);
  const int dim = k + 1;

  std::vector<std::vector<double>> A(m, std::vector<double>(dim, 1.0));
  for (int j = 0; j < m; ++j) {
    const double x = static_cast<double>(j - h);
    for (int p = 1; p < dim; ++p) {
      A[j][p] = A[j][p - 1] * x;
    }
  }
  std::vector<std::vector<double>> M(dim, std::vector<double>(dim + 1, 0.0));
  for (int r = 0; r < dim; ++r) {
    for (int c = 0; c < dim; ++c) {
      double s = 0.0;
      for (int j = 0; j < m; ++j) {
        s += A[j][r] * A[j][c];
      }
      M[r][c] = s;
    }
  }
  M[0][dim] = 1.0;  // e₀
  for (int col = 0; col < dim; ++col) {
    int piv = col;
    for (int r = col + 1; r < dim; ++r) {
      if (std::fabs(M[r][col]) > std::fabs(M[piv][col])) {
        piv = r;
      }
    }
    std::swap(M[col], M[piv]);
    const double d = M[col][col];
    for (int c = col; c <= dim; ++c) {
      M[col][c] /= d;
    }
    for (int r = 0; r < dim; ++r) {
      if (r == col) {
        continue;
      }
      const double f = M[r][col];
      for (int c = col; c <= dim; ++c) {
        M[r][c] -= f * M[col][c];
      }
    }
  }
  std::vector<double> coeff(m, 0.0);
  for (int j = 0; j < m; ++j) {
    double s = 0.0;
    for (int p = 0; p < dim; ++p) {
      s += A[j][p] * M[p][dim];
    }
    coeff[j] = s;
  }
  return coeff_cache_.emplace(half_window, std::move(coeff)).first->second;
}

bool PolynomialSmoother::smooth(nav_msgs::msg::Path & path, const rclcpp::Duration & max_time)
{
  const auto t0 = std::chrono::steady_clock::now();
  const size_t n = path.poses.size();
  if (static_cast<int>(n) < poly_order_ + 3) {
    return true;  // 过短路径无平滑意义
  }

  const nav_msgs::msg::Path original = path;
  const int max_half = (window_size_ - 1) / 2;

  std::vector<double> xs(n), ys(n);
  for (size_t i = 0; i < n; ++i) {
    xs[i] = path.poses[i].pose.position.x;
    ys[i] = path.poses[i].pose.position.y;
  }
  // 端点固定；边界内点用收缩对称窗（窗口不足以支撑拟合阶数时保持原值）。
  for (size_t i = 1; i + 1 < n; ++i) {
    const int h = std::min<int>({max_half, static_cast<int>(i), static_cast<int>(n - 1 - i)});
    if (2 * h + 1 < poly_order_ + 2) {
      continue;
    }
    const auto & c = sgCoefficients(h);
    double sx = 0.0, sy = 0.0;
    for (int j = -h; j <= h; ++j) {
      sx += c[j + h] * xs[i + j];
      sy += c[j + h] * ys[i + j];
    }
    path.poses[i].pose.position.x = sx;
    path.poses[i].pose.position.y = sy;
  }

  // 碰撞防护：平滑把点挪进障碍则整条弃用（保守——平滑是锦上添花，不做冒险）。
  if (reject_on_collision_ && costmap_sub_) {
    try {
      auto costmap = costmap_sub_->getCostmap();
      for (const auto & ps : path.poses) {
        unsigned int mx, my;
        if (costmap->worldToMap(ps.pose.position.x, ps.pose.position.y, mx, my) &&
          costmap->getCost(mx, my) >= nav2_costmap_2d::LETHAL_OBSTACLE)
        {
          RCLCPP_WARN(
            logger_, "PolynomialSmoother: smoothed path hits lethal cost, keeping original");
          path = original;
          return true;
        }
      }
    } catch (const std::exception &) {
      RCLCPP_WARN_ONCE(logger_, "PolynomialSmoother: costmap unavailable, skip collision check");
    }
  }

  recomputeOrientations(path);
  return std::chrono::steady_clock::now() - t0 <=
         std::chrono::duration<double>(max_time.seconds());
}

}  // namespace nav2_path_smoothers

PLUGINLIB_EXPORT_CLASS(nav2_path_smoothers::PolynomialSmoother, nav2_core::Smoother)
