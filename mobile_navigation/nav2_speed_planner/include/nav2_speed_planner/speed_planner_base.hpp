// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// 速度规划器抽象基类（nav2_core::SpeedPlanner 实现）。
//
// 架构：base 实现全部 7 个冻结接口（configure/cleanup/activate/deactivate/
//       setPlan/computeSpeedPlan/setSpeedLimit），其中 computeSpeedPlan 为
//       **模板方法**（Template Method 模式）：负责 path→TrajPointWork 构建、
//       初始限速取严、组装 RobotTrajectory；中间留 protected 纯虚 hook
//       assignVelocityProfile() 让派生类填速度剖面（使 base 仍为抽象类）。
//
// 复用：assignConstantAccelProfile() 是从 dcm tcurve 移植的「恒加速度参考内核」
//       （getSpeedLimit/setPlanSpeedLimit/computeAccelPlan/computeDecelPlan/
//       applyCurvatureConstraint 全套，逻辑一行不改，含已修复的 std::clamp），
//       供 T 型直接调用、S 型 scaffold 安全回退。
#ifndef NAV2_SPEED_PLANNER__SPEED_PLANNER_BASE_HPP_
#define NAV2_SPEED_PLANNER__SPEED_PLANNER_BASE_HPP_

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

namespace nav2_speed_planner
{

/**
 * @struct TrajPointWork
 * @brief 内部工作结构，对齐 dcm `dcm_msgs::PathPose` 的 {pose, speed_limit, curvature}。
 *        透传字段（speed_hint/region_type/segment_id）来自输入 path（若有语义）。
 *        accel / rel_time 默认 0，供 S 型曲线（jerk 限制）未来填充使用。
 */
struct TrajPointWork
{
  geometry_msgs::msg::PoseStamped pose;
  double speed_limit{0.0};
  double curvature{0.0};

  // —— S 型曲线未来字段（恒加速度内核默认 0，组装时按需回填）——
  double accel{0.0};
  double rel_time{0.0};

  // —— 透传字段（来自带语义的 path；nav_msgs/Path 无此字段时置默认）——
  float speed_hint{0.0f};
  std::string region_type;
  std::string segment_id;
};

/**
 * @class SpeedPlannerBase
 * @brief 速度规划器抽象基类：concrete 实现 nav2_core::SpeedPlanner 全部 7 个纯虚，
 *        通过 protected 纯虚 assignVelocityProfile() 保持抽象，留给派生类填速度剖面。
 */
class SpeedPlannerBase : public nav2_core::SpeedPlanner
{
public:
  SpeedPlannerBase() = default;
  ~SpeedPlannerBase() override = default;

  // —— nav2_core::SpeedPlanner 接口实现（签名不得修改）——
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief 模板方法（Template Method）：path + 当前速度 → 带速度的时空轨迹。
   *        固定流程：构建 work（pose / 初始 speed_limit=max_speed / 曲率预留）→
   *        取严初始限速 → 调 protected 纯虚 assignVelocityProfile(work, curr_vel) →
   *        组装 RobotTrajectory（longitudinal_velocity / path_kappa / accel / rel_time / 透传）。
   */
  nav_interfaces::msg::RobotTrajectory computeSpeedPlan(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::Twist & current_velocity) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  // ============================================================
  // 派生类必须实现的速度剖面填充 hook（使 base 保持抽象）。
  // 入参 work：已填 pose / 初始 speed_limit / 段语义；curr_vel：当前速度。
  // 派生类负责把每点的 work.speed_limit（及可选 accel/rel_time）写成最终速度剖面。
  // ============================================================
  virtual void assignVelocityProfile(
    std::vector<TrajPointWork> & work, double curr_vel) = 0;

  // ============================================================
  // 可复用「恒加速度参考内核」（移植自 dcm tcurve，逻辑完整保留）。
  // T 型派生类直接调用；S 型 scaffold 作安全回退。
  // ============================================================
  void assignConstantAccelProfile(std::vector<TrajPointWork> & work, double curr_vel);

  // —— 恒加速度内核子步骤（移植自 dcm，对工作结构 std::vector<TrajPointWork> 操作）——

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

  // —— 共享工具纯函数（nav2_util 无等价，移植自 dcm nav_util::path_utils）——

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

  /// 障碍检测：costmap 上 (x,y) 处是否非 FREE_SPACE（共享工具，供派生/内核复用）。
  bool isObstacleAt(double x, double y) const;

  // —— 成员（protected：派生类可访问）——
  std::string plugin_name_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("SpeedPlannerBase")};

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

}  // namespace nav2_speed_planner

#endif  // NAV2_SPEED_PLANNER__SPEED_PLANNER_BASE_HPP_
