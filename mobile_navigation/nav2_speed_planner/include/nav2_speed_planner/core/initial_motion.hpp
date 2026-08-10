#ifndef NAV2_SPEED_PLANNER__SPEED_PLAN_STATE_HPP_
#define NAV2_SPEED_PLANNER__SPEED_PLAN_STATE_HPP_

#include <mutex>
#include <optional>

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
  std::optional<InitialMotionSource> reason;
  // P1-2：速度是否取自继承的计划值（true）而不是本周期实测值（false）。
  // 与 inherited()（=加速度是否继承）分开表达——VELOCITY_MISMATCH 时加速度仍继承，
  // 但速度已经重锚定到实测，两者不再总是同步。
  bool velocity_inherited{false};

  bool inherited() const
  {
    return state.acceleration_valid;
  }
};

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
    // 连续命中 VELOCITY_MISMATCH 达到此周期数，升级为 VELOCITY_MISMATCH_FAILSAFE
    // （加速度也不再继承）——防止计划持续脱离现实时被无限期"原谅"。
    int failsafe_deviation_cycles{5};
    double goal_position_tolerance{0.05};
    double goal_yaw_tolerance{0.2};
    double branch_heading_tolerance{0.7853981634};
  };

  InitialMotionArbiter();
  explicit InitialMotionArbiter(const Config & config);

  void configure(const Config & config);

  InitialMotionDecision resolve(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::Twist & measured_velocity,
    const rclcpp::Time & now) const;

  void accept(
    const nav_interfaces::msg::RobotTrajectory & trajectory,
    double consumed_target_velocity,
    double consumed_target_acceleration,
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
  rclcpp::Time accepted_trajectory_time_{0, 0, RCL_ROS_TIME};
  bool valid_{false};
  InitialMotionSource invalidation_reason_{InitialMotionSource::FIRST_CYCLE};
  // P1-2：连续命中 VELOCITY_MISMATCH 的周期计数（resolve() 是 const 查询接口，
  // 计数属于跨周期观测状态而非输出，因此 mutable，和 mutex_ 同一惯例）。
  mutable int consecutive_velocity_deviation_{0};
};

}  // namespace nav2_speed_planner

#endif  // NAV2_SPEED_PLANNER__SPEED_PLAN_STATE_HPP_
