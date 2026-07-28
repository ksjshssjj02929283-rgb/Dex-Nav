// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#ifndef NAV2_CORE__SPEED_PLANNER_HPP_
#define NAV2_CORE__SPEED_PLANNER_HPP_

#include <memory>
#include <string>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/transform_listener.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_interfaces/msg/robot_trajectory.hpp"

namespace nav2_core
{

struct SpeedPlanningBoundary
{
  geometry_msgs::msg::Twist velocity;
  double acceleration{0.0};
  bool acceleration_valid{false};
  builtin_interfaces::msg::Time source_stamp;
  double source_age{0.0};
};

struct SpeedPlanningDiagnostics
{
  bool terminal_infeasible{false};
};

/**
 * @class SpeedPlanner
 * @brief 五层架构 ④速度规划层接口（项目 32 / DEC-007，Nav2 原生缺此层）。
 *
 * 职责：把"只有几何"的局部路径升级为**带速度/加速度/时间的时空轨迹**
 *       （RobotTrajectory），按 path 内 speed_hint/region_type 段语义做限速与平滑。
 * 边界：不改路径几何（③Local 定），不做闭环跟踪（⑤Controller），不决定走哪（①②）。
 * 由 ControllerServer 作为三段 pipeline 的第 2 段持有（pluginlib 加载）。
 */
class SpeedPlanner
{
public:
  using Ptr = std::shared_ptr<nav2_core::SpeedPlanner>;

  virtual ~SpeedPlanner() {}

  /**
   * @brief 生命周期 configure（对齐 nav2_core::Controller）。
   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) = 0;

  virtual void cleanup() = 0;
  virtual void activate() = 0;
  virtual void deactivate() = 0;

  /**
   * @brief 设置待规划速度的局部几何路径（来自 ③Local）。
   */
  virtual void setPlan(const nav_msgs::msg::Path & path) = 0;

  /**
   * @brief 核心：几何 path + 当前运动边界 → 带速度的时空轨迹。
   *
   * 实现需填充 RobotTrajectory.points[] 的运动学/时间字段：
   *   longitudinal_velocity / acceleration / relative_time / path_kappa，
   * 并将几何点的 segment_id / region_type / speed_hint **透传**到对应轨迹点。
   *
   * @param path     局部几何路径（来自 setPlan 或本次传入）
   * @param boundary 当前实测速度与可选的连续加速度边界
   * @return 带速度的时空轨迹
   */
  virtual nav_interfaces::msg::RobotTrajectory computeSpeedPlan(
    const nav_msgs::msg::Path & path,
    const SpeedPlanningBoundary & boundary) = 0;

  virtual SpeedPlanningDiagnostics getLastPlanningDiagnostics() const
  {
    return SpeedPlanningDiagnostics();
  }

  /**
   * @brief 外部限速（区域/事件下发；与 path 内 speed_hint 取严）。
   * @param speed_limit 绝对值(m/s) 或占最大速度百分比
   * @param percentage  true=百分比，false=绝对值
   */
  virtual void setSpeedLimit(const double & speed_limit, const bool & percentage) = 0;
};

}  // namespace nav2_core

#endif  // NAV2_CORE__SPEED_PLANNER_HPP_
