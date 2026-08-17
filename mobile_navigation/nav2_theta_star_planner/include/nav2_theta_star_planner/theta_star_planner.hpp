// Copyright 2020 Anshumaan Singh
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_THETA_STAR_PLANNER__THETA_STAR_PLANNER_HPP_
#define NAV2_THETA_STAR_PLANNER__THETA_STAR_PLANNER_HPP_

#include <iostream>
#include <cmath>
#include <string>
#include <chrono>
#include <queue>
#include <algorithm>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_theta_star_planner/theta_star.hpp"
#include "nav2_theta_star_planner/multi_layer_planner.hpp"
#include "nav2_theta_star_planner/omni_path_smoother.hpp"
#include "nav2_theta_star_planner/curve_path_smoother.hpp"
#include "nav2_theta_star_planner/l1_spline_path.hpp"
#include "nav2_util/geometry_utils.hpp"

using rcl_interfaces::msg::ParameterType;

namespace nav2_theta_star_planner
{

/// Path post-process mode (rollback: set smooth_path_mode:=legacy in yaml).
enum class SmoothPathMode
{
  LEGACY = 0,
  L1_SPLINE = 1
};

class ThetaStarPlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;

  void activate() override;

  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

protected:
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("ThetaStarPlanner")};
  std::string global_frame_, name_;
  bool use_final_approach_orientation_;
  /// Geometry-only post-smoothing with no minimum-turning-radius constraint.
  bool smooth_path_{false};
  bool smooth_do_string_pull_{true};
  bool smooth_do_gradient_{true};
  bool smooth_do_curve_fit_{false};
  bool smooth_log_timing_{false};
  SmoothPathMode smooth_path_mode_{SmoothPathMode::LEGACY};
  double smooth_l1_clearance_radius_factor_{0.85};
  double smooth_l1_corner_angle_rad_{0.25};
  double smooth_l1_simplify_angle_rad_{0.10};
  bool smooth_l1_do_gradient_{true};
  OmniPathSmootherConfig path_smooth_config_;
  CurvePathSmootherConfig curve_path_config_;

  // parent node weak ptr
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node_;

  std::unique_ptr<theta_star::ThetaStar> planner_;
  std::unique_ptr<theta_star::MultiLayerNodePlanner> multi_layer_planner_;

  bool use_node_map_{false};
  std::string node_map_yaml_;
  std::string local_map_yaml_;
  std::string raw_map_yaml_;
  std::string distance_map_yaml_;
  bool node_map_loaded_{false};

  bool loadNodeMapResources();

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;


  /**
   * @brief the function responsible for calling the algorithm and retrieving a path from it
   * @return global_path is the planned path to be taken
   */
  void getPlan(
    nav_msgs::msg::Path & global_path,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);

  /**
   * @brief interpolates points between the consecutive waypoints of the path
   * @param raw_path is used to send in the path received from the planner
   * @param dist_bw_points is used to send in the interpolation_resolution (which has been set as the costmap resolution)
   * @return the final path with waypoints at a distance of the value of interpolation_resolution of each other
   */
  static nav_msgs::msg::Path linearInterpolation(
    const std::vector<coordsW> & raw_path,
    const double & dist_bw_points);

  static nav_msgs::msg::Path denseInterpolation(
    const std::vector<coordsW> & raw_path,
    const double & dist_bw_points);

  /**
   * @brief Interpolate sparse waypoints and optionally apply geometry-only smoothing.
   */
  void buildGeometricPath(
    nav_msgs::msg::Path & global_path,
    const std::vector<coordsW> & raw_path);

  void buildGeometricPathLegacy(
    nav_msgs::msg::Path & global_path,
    const std::vector<coordsW> & raw_path);

  void buildGeometricPathL1Spline(
    nav_msgs::msg::Path & global_path,
    const std::vector<coordsW> & raw_path);

  /**
   * @brief 将栅格中心路径安全连接到用户给出的连续坐标起点/终点
   *
   * Theta* 搜索节点仍是栅格中心；本函数在平滑结束后补首尾短连接，使控制器跟踪的
   * 几何终点与用户目标完全一致。连接沿线逐点检查膨胀后的 costmap，失败时不发布
   * 可能穿障碍物的路径。
   *
   * @param path 已生成的栅格中心几何路径
   * @param start 用户给出的精确连续坐标起点
   * @param goal 用户给出的精确连续坐标终点
   * @param connect_goal false 表示规划器有意返回替代终点，此时不强接原始 goal
   * @return 首尾连接均安全时为 true
   */
  bool connectPathToExactEndpoints(
    nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    bool connect_goal = true) const;

  /** @brief 检查一段连续世界坐标直线是否全部落在 Theta* 可通行栅格中 */
  bool isWorldSegmentSafe(
    const geometry_msgs::msg::Point & from,
    const geometry_msgs::msg::Point & to) const;

  static SmoothPathMode parseSmoothPathMode(const std::string & mode);

  /**
   * @brief Callback executed when a paramter change is detected
   * @param parameters list of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};
}   //  namespace nav2_theta_star_planner

#endif  //  NAV2_THETA_STAR_PLANNER__THETA_STAR_PLANNER_HPP_
