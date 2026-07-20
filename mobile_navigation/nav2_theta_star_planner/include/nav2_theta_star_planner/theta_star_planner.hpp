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
#include "nav2_theta_star_planner/swerve_steering_model.hpp"
#include "nav2_util/geometry_utils.hpp"

using rcl_interfaces::msg::ParameterType;

namespace nav2_theta_star_planner
{

/// 路径点 yaw 赋值策略（assign_path_orientations=true 时生效）
enum class PathOrientationMode
{
  NONE = 0,     ///< 不赋中间点朝向（与上游 Theta* 一致，仅终点由 createPlan 处理）
  AUTO,         ///< 推荐：起终点同 yaw→全程平移；不同→沿路径弧长从 start 插值到 goal
  START,        ///< 强制全程用起点 yaw（忽略起终点是否相同）
  GOAL,         ///< 强制全程用 goal yaw
  TANGENT       ///< 每点用指向下一段的路径切线（差速/阿克曼风格）
};

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
  /// 是否为路径各点写入 orientation（默认 true，解决中间点 yaw=0 导致 MPPI 乱转）
  bool assign_path_orientations_{true};
  PathOrientationMode path_orientation_mode_{PathOrientationMode::AUTO};
  /// |start_yaw - goal_yaw| 小于该值(rad) 时视为“同朝向”，全程不做 yaw 插值
  double path_orientation_same_threshold_rad_{0.02};
  /// 是否插入 orientation mode 零位移标记。参数名保留 tangent_ 前缀兼容旧配置：
  /// TANGENT=首尾各一个、GOAL=起点一个、START=起点两个、NONE=终点一个；AUTO 无 marker。
  bool tangent_in_place_rotation_markers_{false};

  /**
   * @brief B 层：沿路径仿真舵轮 IK，限制舵角跳变（对齐 mobile_cmd_omni_region.cpp）
   * @see enforceSteeringContinuityOnPath()
   */
  bool enforce_steering_continuity_{true};
  SwerveSteeringConfig swerve_steering_config_;

  /// Geometry-only post-smoothing for omnidirectional bases (no turning-radius limits).
  bool smooth_path_{true};
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

  /**
   * @brief Interpolate sparse waypoints and optionally apply omni path smoothing.
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

  static SmoothPathMode parseSmoothPathMode(const std::string & mode);

  /**
   * @brief 解析 path_orientation_mode 字符串参数
   * @param mode "auto" | "none" | "start" | "goal" | "tangent"（大小写不敏感）
   */
  static PathOrientationMode parsePathOrientationMode(const std::string & mode);

  /**
   * @brief 为 /plan 各点写入 orientation，供 MPPI PathAlign(use_path_orientations) 使用
   * @param path 已插值的几何路径
   * @param start 规划起点（mode=start 时取其 yaw）
   * @param goal 规划终点（mode=goal 或末点 override 时取其 yaw）
   */
  void assignPathOrientations(
    nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

  /**
   * @brief 在 assignPathOrientations 之后调用：调整路径 yaw 使舵轮目标角连续
   * @param path 几何路径（仅修改 orientation）
   * @param goal_yaw 终点目标 yaw（rad）
   */
  void applySteeringContinuityConstraint(
    nav_msgs::msg::Path & path,
    double goal_yaw) const;

  /**
   * @brief Callback executed when a paramter change is detected
   * @param parameters list of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};
}   //  namespace nav2_theta_star_planner

#endif  //  NAV2_THETA_STAR_PLANNER__THETA_STAR_PLANNER_HPP_
