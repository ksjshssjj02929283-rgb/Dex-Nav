// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// 多项式平滑器（Savitzky-Golay 滑窗最小二乘，默认 5 次）：
// 对每个内部点，在其邻域窗口内做 k 次多项式最小二乘拟合并取拟合值——
// 等价于一次系数可预计算的卷积（等间距假设），O(n) 单遍完成。
// 特性：保长趋势/保峰（多项式阶数高于均值类滤波）、无迭代、耗时确定；
// 端点固定不动（起点=机器人当前位、终点=goal 语义不可动）。
#ifndef NAV2_PATH_SMOOTHERS__POLYNOMIAL_SMOOTHER_HPP_
#define NAV2_PATH_SMOOTHERS__POLYNOMIAL_SMOOTHER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "nav2_core/smoother.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_path_smoothers
{

class PolynomialSmoother : public nav2_core::Smoother
{
public:
  PolynomialSmoother() = default;
  ~PolynomialSmoother() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub) override;

  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}

  bool smooth(nav_msgs::msg::Path & path, const rclcpp::Duration & max_time) override;

protected:
  /// 半窗 h、阶数 k 的 SG 中心点卷积系数（按 h 缓存；等间距假设）。
  const std::vector<double> & sgCoefficients(int half_window);

  rclcpp::Logger logger_{rclcpp::get_logger("PolynomialSmoother")};
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;

  int poly_order_{5};
  int window_size_{15};
  bool reject_on_collision_{true};
  std::unordered_map<int, std::vector<double>> coeff_cache_;
};

}  // namespace nav2_path_smoothers

#endif  // NAV2_PATH_SMOOTHERS__POLYNOMIAL_SMOOTHER_HPP_
