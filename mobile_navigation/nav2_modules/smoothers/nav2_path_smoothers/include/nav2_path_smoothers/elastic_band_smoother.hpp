// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// 弹性带平滑器（Elastic Band）：迭代式"内张力 + 障碍斥力"平衡——
//   内张力把每个点拉向前后邻点中点（拉直/平滑），
//   障碍斥力沿 costmap 代价梯度反方向把点推离高代价区。
// 特性：代价感知（平滑同时主动避开障碍侧），迭代收敛；耗时随迭代数上限有界。
// 端点固定不动（起点=机器人当前位、终点=goal 语义不可动）。
#ifndef NAV2_PATH_SMOOTHERS__ELASTIC_BAND_SMOOTHER_HPP_
#define NAV2_PATH_SMOOTHERS__ELASTIC_BAND_SMOOTHER_HPP_

#include <memory>
#include <string>

#include "nav2_core/smoother.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_path_smoothers
{

class ElasticBandSmoother : public nav2_core::Smoother
{
public:
  ElasticBandSmoother() = default;
  ~ElasticBandSmoother() override = default;

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
  rclcpp::Logger logger_{rclcpp::get_logger("ElasticBandSmoother")};
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;

  int max_iterations_{100};
  double w_smooth_{0.5};
  double w_obstacle_{0.3};
  double step_size_{0.3};
  int obstacle_cost_threshold_{128};
  double convergence_tol_{1e-4};
  bool reject_on_collision_{true};
};

}  // namespace nav2_path_smoothers

#endif  // NAV2_PATH_SMOOTHERS__ELASTIC_BAND_SMOOTHER_HPP_
