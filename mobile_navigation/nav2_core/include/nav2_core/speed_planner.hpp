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

struct InitialMotionState
{
  geometry_msgs::msg::Twist velocity;
  double acceleration{0.0};
  bool acceleration_valid{false};
  // 默认要求速度规划停障。只有实际运行的控制器明确声明具备局部绕障能力时，
  // ControllerServer 才会关闭路径障碍零速约束。
  bool should_stop_for_path_obstacle{true};
  // ControllerServer owns the goal lifecycle. Once terminal braking starts,
  // same-goal geometry updates must not let a receding-horizon planner coast
  // or accelerate again because the refreshed path happened to become longer.
  bool terminal_brake_latched{false};
  builtin_interfaces::msg::Time source_stamp;
  double source_age{0.0};
};

enum class SpeedPlanningTerminalBranch
{
  NONE,
  PATH_OBSTACLE_STOP,
  GOAL_TERMINAL
};

struct SpeedPlanningDiagnostics
{
  bool terminal_infeasible{false};
  bool terminal_brake_required{false};
  /// latch 时冻结的执行裕度 [m]；-1 表示未锁存。锁存后不再随实测速度衰减，
  /// 否则制动目标会随速度下降而滑向终点，裕度被逐周期蚕食。
  /// 末端不可行的**原因码**（SCurveSpeedPlanner::TerminalFeasibility::Reason）。
  /// 归一前 7 处赋值只留下一个 bool，无法区分"稳定末段不可达"与"求解器失败"。
  int terminal_infeasible_reason{0};
  /// 不可行的**类别**（FeasibilityResult::Kind）：1=物理不可行（消费侧须响应）、
  /// 2=求解失败（开发者定位）。此前只有一个 bool，两者分不开。
  int terminal_infeasible_kind{0};
  /// 结果来源（FeasibilityResult::Provenance）：0=正常求解、1=走了制动兜底。
  /// 对应 Apollo「避免把 fallback 轨迹当正常优化结果分析」。
  int terminal_result_provenance{0};
  /// 进入兜底路径时的**首因**（FeasibilityResult::Reason）。最终 reason 会被
  /// 兜底后的判定覆写，只有它能回答"当初为什么进兜底"。
  int terminal_entry_reason{0};
  /// 当前停车约束来源，用于区分路径障碍停车与目标末端制动。
  SpeedPlanningTerminalBranch terminal_branch{SpeedPlanningTerminalBranch::NONE};
  /// A1 检测到的首个障碍点索引与其弧长（-1 = 无障碍）。
  int first_obstacle_index{-1};
  double first_obstacle_arc{-1.0};
  int solver_code{0};
  double solver_v0{0.0}, solver_a0{0.0}, solver_vmax{0.0};
  double solver_target_v{0.0}, solver_distance{0.0};
  double terminal_latched_reserve{-1.0};
  /// 本周期实际下发给 applyTerminalSpeedEnvelope 的制动锚点 [m]（距当前位置）。
  /// 这是"裕度到底有没有作用到剖面上"的唯一直接证据，此前不可观测。
  double terminal_anchor_s{-1.0};
  double terminal_execution_reserve{0.0};
  double terminal_braking_distance{0.0};
  double terminal_distance_available{0.0};
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
    const InitialMotionState & boundary) = 0;

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
