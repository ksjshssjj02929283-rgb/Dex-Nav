#ifndef NAV2_SPEED_PLANNER__SPEED_PLAN_STATE_HPP_
#define NAV2_SPEED_PLANNER__SPEED_PLAN_STATE_HPP_

#include <limits>
#include <mutex>
#include <optional>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/speed_planner.hpp"
#include "nav_interfaces/msg/robot_trajectory.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/time.hpp"

namespace nav2_speed_planner
{

enum class InitialMotionSource
{
  FIRST_CYCLE,
  EXPIRED,
  FRAME_MISMATCH,
  GOAL_MISMATCH,
  BRANCH_MISMATCH,
  SPATIAL_MISMATCH,
  // 低速起步时，旧轨迹的空间采样仍停在零速首点，但实测速度已经脱离零速。
  // 该状态只用于非末端起步恢复，不代表一般速度偏差。
  LOW_SPEED_STARTUP_RESEED,
  VELOCITY_MISMATCH,
  // P1-2（重构方案 §2.3）：连续 failsafe_deviation_cycles 周期都命中 VELOCITY_MISMATCH，
  // 说明计划已经系统性偏离现实（打滑/卡阻等），此时连加速度也不再继承，完整 reseed。
  VELOCITY_MISMATCH_FAILSAFE,
  INVALID_NUMERIC,
  EXPLICIT_INVALIDATION
};

const char * initialMotionSourceName(InitialMotionSource reason);

struct InitialMotionDecision
{
  nav2_core::InitialMotionState state;
  // Apollo-style trajectory stitching prefix. Unlike state inheritance, this
  // keeps the already accepted future pose/v/a/t samples themselves. The
  // ControllerServer trims the current path at this prefix tail, plans only
  // the suffix, then concatenates both trajectories.
  nav_interfaces::msg::RobotTrajectory stitching_prefix;
  nav2_core::InitialMotionState stitching_boundary;
  std::optional<InitialMotionSource> reason;
  // P1-2：速度是否取自继承的计划值（true）而不是本周期实测值（false）。
  // 与 inherited()（=加速度是否继承）分开表达——VELOCITY_MISMATCH 时加速度仍继承，
  // 但速度已经重锚定到实测，两者不再总是同步。
  bool velocity_inherited{false};

  // 裁决的两个**输入**及其偏差。只上报裁决结果（state.velocity）时，看不出
  // v0 跳变到底是"实测真的高"还是"计划采样点跳了"——两者的根治方向完全不同
  // （前者收紧 replan_dev，后者查局部路径裁剪），缺了这三个量只能靠猜。
  // 结构性不连续（frame/goal/branch/spatial 等 reseed 路径）下无计划采样值，
  // 保持 NaN。
  double measured_velocity{std::numeric_limits<double>::quiet_NaN()};
  double sampled_velocity{std::numeric_limits<double>::quiet_NaN()};
  double velocity_deviation{std::numeric_limits<double>::quiet_NaN()};

  bool inherited() const
  {
    return state.acceleration_valid;
  }

  bool hasStitchingPrefix() const
  {
    return stitching_prefix.points.size() >= 2;
  }
};

struct TrajectoryStitchingPlan
{
  nav_interfaces::msg::RobotTrajectory prefix;
  nav_msgs::msg::Path suffix_path;
  nav2_core::InitialMotionState suffix_boundary;
  double prefix_duration{0.0};
};

/// Validate the accepted future prefix against the freshly pruned path, snap
/// its endpoints to that path, and build a suffix path beginning at the prefix
/// tail. Returns false when geometry changed or too little suffix remains.
bool prepareTrajectoryStitching(
  const nav_msgs::msg::Path & path,
  const InitialMotionDecision & decision,
  double max_path_error,
  TrajectoryStitchingPlan & plan,
  std::string * rejection_reason = nullptr);

/// Concatenate a preserved prefix and a newly planned suffix. The suffix time
/// origin is shifted to the prefix tail and its duplicate first point removed.
nav_interfaces::msg::RobotTrajectory concatenateStitchedTrajectory(
  const nav_interfaces::msg::RobotTrajectory & prefix,
  const nav_interfaces::msg::RobotTrajectory & suffix);

class InitialMotionArbiter
{
public:
  struct Config
  {
    double max_age{0.5};
    double max_spatial_error{0.15};
    // P1-2（重构方案 §2.3，Autoware 初值状态机对齐）：|实测 - 计划采样速度| 在此内，
    // 正常态才继承计划速度（不再无条件重锚定到实测）；超出但结构性连续性仍成立时，
    // 仍继承加速度、只重锚定速度（VELOCITY_MISMATCH）。旧字段名 max_velocity_error
    // 语义是"超出即完整拒绝"，与新语义不同，改名避免和旧行为混淆；默认值从 0.25
    // 收紧到方案建议的 0.10（约巡航速度 0.6 的 17%）。
    double replan_dev{0.10};
    // 终端制动时只针对“实测比计划更快”收紧门限。低速下0.05m/s绝对偏差
    // 已足以让制动距离估小一倍，不能继续沿用巡航0.10m/s门限。
    double terminal_overspeed_replan_dev{0.02};
    // 连续命中 VELOCITY_MISMATCH 达到此周期数，升级为 VELOCITY_MISMATCH_FAILSAFE
    // （加速度也不再继承）——防止计划持续脱离现实时被无限期"原谅"。
    int failsafe_deviation_cycles{5};
    double goal_position_tolerance{0.05};
    double goal_yaw_tolerance{0.2};
    double branch_heading_tolerance{0.7853981634};
    double stitching_horizon{0.20};
    double stitching_min_duration{0.10};
    double stitching_max_time_error{0.15};
  };

  InitialMotionArbiter();
  explicit InitialMotionArbiter(const Config & config);

  void configure(const Config & config);

  InitialMotionDecision resolve(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::Twist & measured_velocity,
    const rclcpp::Time & now,
    bool terminal_brake_latched = false) const;

  void accept(
    const nav_interfaces::msg::RobotTrajectory & trajectory,
    double consumed_target_velocity,
    double consumed_target_acceleration,
    double consumed_target_relative_time,
    const rclcpp::Time & now);

  void reset(InitialMotionSource reason = InitialMotionSource::EXPLICIT_INVALIDATION);

  bool valid() const;

private:
  void resetUnlocked(InitialMotionSource reason);

  mutable std::mutex mutex_;
  Config config_;
  nav_interfaces::msg::RobotTrajectory accepted_trajectory_;
  double accepted_target_velocity_{0.0};
  double accepted_target_acceleration_{0.0};
  double accepted_target_relative_time_{0.0};
  rclcpp::Time accepted_trajectory_time_{0, 0, RCL_ROS_TIME};
  bool valid_{false};
  InitialMotionSource invalidation_reason_{InitialMotionSource::FIRST_CYCLE};
  // P1-2：连续命中 VELOCITY_MISMATCH 的周期计数（resolve() 是 const 查询接口，
  // 计数属于跨周期观测状态而非输出，因此 mutable，和 mutex_ 同一惯例）。
  mutable int consecutive_velocity_deviation_{0};
};

}  // namespace nav2_speed_planner

#endif  // NAV2_SPEED_PLANNER__SPEED_PLAN_STATE_HPP_
