// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// T 型曲线速度规划器（nav2_core::SpeedPlanner 实现）。
// 由 dcm `tcurve_speed_planner::TCurveSpeedPlanner`（ROS1）移植而来：
// 完整保留三段式逻辑（加速段 / 匀速段 / 减速段 + 曲率/障碍约束）。
#ifndef NAV2_TCURVE_SPEED_PLANNER__TCURVE_SPEED_PLANNER_HPP_
#define NAV2_TCURVE_SPEED_PLANNER__TCURVE_SPEED_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_interfaces/msg/robot_trajectory.hpp"

#include "nav2_core/speed_planner.hpp"

namespace nav2_tcurve_speed_planner
{

/**
 * @struct TrajPointWork
 * @brief 内部工作结构，对齐 dcm `dcm_msgs::PathPose` 的 {pose, speed_limit, curvature}。
 *        透传字段（speed_hint/region_type/segment_id）来自输入 path（若有语义）。
 */
struct TrajPointWork
{
  geometry_msgs::msg::PoseStamped pose;
  double speed_limit{0.0};
  double curvature{0.0};

  // —— 透传字段（来自带语义的 path；nav_msgs/Path 无此字段时置默认）——
  float speed_hint{0.0f};
  std::string region_type;
  std::string segment_id;
};

/**
 * @class TCurveSpeedPlanner
 * @brief T 型曲线速度规划器，实现 nav2_core::SpeedPlanner 冻结接口。
 */
class TCurveSpeedPlanner : public nav2_core::SpeedPlanner
{
public:
  TCurveSpeedPlanner() = default;
  ~TCurveSpeedPlanner() override = default;

  // —— nav2_core::SpeedPlanner 接口实现（签名不得修改）——
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  nav_interfaces::msg::RobotTrajectory computeSpeedPlan(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::Twist & current_velocity) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  // —— 三段式核心算法（移植自 dcm，对工作结构 std::vector<TrajPointWork> 操作）——

  /// 计算全局限速（含障碍检测置零 + 无法达到最大速度时回退求解）。
  double getSpeedLimit(std::vector<TrajPointWork> & path, const double curr_vel);

  /// 将限速 clamp 到 [min_speed_, max_speed_] 后回填到所有非零速点。
  void setPlanSpeedLimit(std::vector<TrajPointWork> & path, const double speed_limit);

  /// 加速段：按当前速度 + 加速度推进速度直至达到 speed_limit。
  void computeAccelPlan(
    std::vector<TrajPointWork> & path, const double curr_vel, const double speed_limit);

  /// 减速段：停止距离减速 + 曲率前减速 + 障碍前减速。
  void computeDecelPlan(
    std::vector<TrajPointWork> & path, const double curr_vel, const double speed_limit);

  /// 曲率约束：小半径处限速 + 转弯保持匀速（可选）。
  void applyCurvatureConstraint(std::vector<TrajPointWork> & path, const double ex_speed_limit);

  /// 计算"有速度"的有效路径长度（遇第一个零速点截断）。
  double computeValidSpeedPathLength(const std::vector<TrajPointWork> & path);

  /// 加速所需距离（v0→speed_limit，固定加速度）。
  double computeAccelLength(const double curr_vel, const double speed_limit) const;

  /// 减速所需距离（v0→min_speed，固定减速度，decel_ < 0）。
  double computeDecelLength(const double curr_vel, const double min_speed) const;

  // —— 路径工具纯函数（nav2_util 无等价，移植自 dcm nav_util::path_utils）——

  /// 路径几何总长。
  static double computePathLength(const std::vector<TrajPointWork> & path);

  /// 三点圆拟合曲率序列（Menger 曲率）。
  std::vector<double> calcTrajectoryCurvatureFrom3Points(
    const std::vector<TrajPointWork> & path, size_t idx_dist);

  /// 三点 Menger 曲率（denominator 过小抛 std::runtime_error）。
  static double calcCurvature(
    const geometry_msgs::msg::Pose & p1,
    const geometry_msgs::msg::Pose & p2,
    const geometry_msgs::msg::Pose & p3);

  // —— 成员 ——
  std::string plugin_name_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("TCurveSpeedPlanner")};

  // 缓存路径（来自 setPlan）。
  nav_msgs::msg::Path current_plan_;

  // 参数（默认值同 dcm initialize）。
  double max_speed_{0.6};
  double min_speed_{0.03};
  double accel_{0.35};
  double decel_{-0.15};
  double curve_compute_dist_{0.1};
  double decel_distance_before_curve_{0.5};
  double decel_distance_after_curve_{0.5};
  double regulated_linear_scaling_min_radius_{1.2};
  double min_curvature_speed_{0.1};
  double constant_velocity_dist_threshold_{0.1};
  bool enable_constant_velocity_while_turning_{false};
  double stop_speed_{0.03};
  double stop_distance_{0.2};
  double collision_stop_distance_{0.2};

  // 外部限速（setSpeedLimit 保存；computeSpeedPlan 与 path speed_hint 取严）。
  // NO_SPEED_LIMIT 含义同 nav2：未设外部限速。
  double speed_limit_{0.0};
  bool speed_limit_is_percentage_{false};

  // 运行时缓存。
  double curr_path_length_{0.0};
  double points_interval_{0.1};
};

}  // namespace nav2_tcurve_speed_planner

#endif  // NAV2_TCURVE_SPEED_PLANNER__TCURVE_SPEED_PLANNER_HPP_
