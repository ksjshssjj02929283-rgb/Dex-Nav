// Copyright (c) 2026 Dexory
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef NAV2_CONTROLLER__NEAR_GOAL_ODOM_CONTEXT_IMPL_HPP_
#define NAV2_CONTROLLER__NEAR_GOAL_ODOM_CONTEXT_IMPL_HPP_

#include <mutex>

#include "nav2_controller/near_goal_odom_context.hpp"

namespace nav2_controller
{

struct NearGoalOdomParameters
{
  NearGoalOdomMode mode{NearGoalOdomMode::DISABLED};
  double entry_distance{0.10};
  int entry_confirm_cycles{3};
  double position_lon_tolerance{0.025};
  double position_lat_tolerance{0.025};
  int position_confirm_cycles{2};
  double fine_yaw_entry_error{0.10};
  int fine_yaw_entry_confirm_cycles{3};
  double yaw_latch_max_error_jump{0.05};
  double final_yaw_tolerance{0.02};
  int stop_confirm_cycles{3};
  double odom_position_drift_tolerance{0.025};
  double position_hold_lon_tolerance{0.04};
  double position_hold_lat_tolerance{0.04};
  double global_position_abort_tolerance{0.15};
  double global_yaw_correction_abort_tolerance{0.35};
  double odom_jump_position_tolerance{0.10};
  double odom_jump_yaw_tolerance{0.30};
  double tf_failure_timeout{0.30};
  double trans_stopped_velocity{0.01};
  double rot_stopped_velocity{0.02};
};

struct NearGoalOdomInput
{
  geometry_msgs::msg::Pose robot_in_odom;
  geometry_msgs::msg::Pose live_goal_in_odom;
  double live_map_position_error{0.0};
  double live_map_lon_error{0.0};
  double live_map_lat_error{0.0};
  double live_map_yaw_error{0.0};
  double map_to_odom_x{0.0};
  double map_to_odom_y{0.0};
  double map_to_odom_yaw{0.0};
  double actual_vx{0.0};
  double actual_wz{0.0};
  double tf_age{0.0};
  bool tf_valid{false};
  // 控制周期时间只用于故障超时，避免位姿时间戳停滞时超时失效。
  rclcpp::Time evaluation_stamp;
  rclcpp::Time stamp;
};

class NearGoalOdomContext : public NearGoalOdomSnapshotProvider
{
public:
  explicit NearGoalOdomContext(const NearGoalOdomParameters & parameters);

  // 在短临界区内更新参数；模式变化时清空旧锁存状态。
  void setParameters(const NearGoalOdomParameters & parameters);
  // 开始新的目标代次，并幂等清空全部末端状态。
  void reset(uint64_t goal_generation, NearGoalTransitionReason reason);
  // 使用同一控制周期输入推进状态机并返回线程安全副本。
  NearGoalOdomSnapshot update(const NearGoalOdomInput & input);
  NearGoalOdomSnapshot snapshot() const override;

private:
  // 以下函数只允许在持有mutex_时调用。
  void resetStateLocked(uint64_t goal_generation, NearGoalTransitionReason reason);
  void transitionToLocked(
    NearGoalOdomStage stage, NearGoalTransitionReason reason);
  bool transitionAllowedLocked(NearGoalOdomStage stage) const;
  void setControlDirectiveLocked(NearGoalControlDirective directive);
  void updateErrorsLocked(const NearGoalOdomInput & input);
  // 使用已锁存的Odom目标立即刷新位置误差，避免锁存首周期保留默认零值。
  void updateOdomPositionErrorsLocked(
    const geometry_msgs::msg::Pose & robot_in_odom);
  bool robotStoppedLocked() const;
  // 线速度停稳后允许同向角速度直接交给最终转向，反向旋转仍需先完全停稳。
  bool readyForYawHandoffLocked(double live_map_yaw_error) const;
  bool positionHoldValidLocked() const;
  bool odomContinuousLocked(const NearGoalOdomInput & input);

  mutable std::mutex mutex_;
  NearGoalOdomParameters parameters_;
  NearGoalOdomSnapshot snapshot_;
  int entry_confirm_count_{0};
  int position_confirm_count_{0};
  int fine_yaw_confirm_count_{0};
  int stop_confirm_count_{0};
  int yaw_latch_failure_count_{0};
  bool braking_to_position_{false};
  bool previous_odom_valid_{false};
  bool previous_map_to_odom_valid_{false};
  geometry_msgs::msg::Pose previous_robot_in_odom_;
  rclcpp::Time previous_stamp_;
  double previous_map_to_odom_x_{0.0};
  double previous_map_to_odom_y_{0.0};
  double previous_map_to_odom_yaw_{0.0};
  double yaw_consistency_at_latch_{0.0};
  bool tf_failure_active_{false};
  rclcpp::Time tf_failure_start_;
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__NEAR_GOAL_ODOM_CONTEXT_IMPL_HPP_
