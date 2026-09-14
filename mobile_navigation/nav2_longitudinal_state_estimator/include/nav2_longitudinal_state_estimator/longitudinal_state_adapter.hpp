// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0

#ifndef NAV2_LONGITUDINAL_STATE_ESTIMATOR__LONGITUDINAL_STATE_ADAPTER_HPP_
#define NAV2_LONGITUDINAL_STATE_ESTIMATOR__LONGITUDINAL_STATE_ADAPTER_HPP_

#include <cstddef>
#include <limits>
#include <mutex>

#include "rclcpp/time.hpp"

namespace nav2_longitudinal_state_estimator
{

/// Validates and adapts the already-fused robot_localization velocity and
/// acceleration. This class intentionally contains no second Kalman filter:
/// /robot_localization outputs already include wheel odometry, lidar pose and
/// IMU information, so fusing the same sensors again would double-count them.
class LongitudinalStateAdapter
{
public:
  struct Config
  {
    bool enabled{true};
    double max_age{0.06};
    double max_stamp_skew{0.01};
    double velocity_std_floor{0.02};
    double acceleration_std_floor{0.15};
    double max_velocity_std{0.10};
    double max_acceleration_std{0.60};
    double min_acceleration{-0.60};
    double max_acceleration{0.70};
    double stationary_max_abs_velocity{0.01};
    double stationary_max_abs_acceleration{0.08};
    size_t min_velocity_samples{5};
    size_t min_acceleration_samples{5};
  };

  struct Estimate
  {
    bool valid{false};
    bool velocity_valid{false};
    bool acceleration_valid{false};
    bool stationary{false};
    double velocity{std::numeric_limits<double>::quiet_NaN()};
    double acceleration_raw{std::numeric_limits<double>::quiet_NaN()};
    double acceleration{std::numeric_limits<double>::quiet_NaN()};
    double velocity_std{std::numeric_limits<double>::quiet_NaN()};
    double acceleration_std{std::numeric_limits<double>::quiet_NaN()};
    double velocity_age{std::numeric_limits<double>::quiet_NaN()};
    double acceleration_age{std::numeric_limits<double>::quiet_NaN()};
    double stamp_skew{std::numeric_limits<double>::quiet_NaN()};
    size_t velocity_samples{0};
    size_t acceleration_samples{0};
    size_t rejected_samples{0};
    size_t acceleration_clamps{0};
    rclcpp::Time source_stamp{0, 0, RCL_ROS_TIME};
  };

  LongitudinalStateAdapter();
  explicit LongitudinalStateAdapter(const Config & config);

  void configure(const Config & config);
  void addVelocity(double velocity, double variance, const rclcpp::Time & stamp);
  void addAcceleration(double acceleration, double variance, const rclcpp::Time & stamp);
  Estimate estimate(const rclcpp::Time & now) const;
  void reset();

private:
  void resetUnlocked();
  bool acceptClockUnlocked(const rclcpp::Time & stamp);

  mutable std::mutex mutex_;
  Config config_;
  bool clock_initialized_{false};
  rcl_clock_type_t clock_type_{RCL_ROS_TIME};
  bool has_velocity_{false};
  bool has_acceleration_{false};
  double velocity_{0.0};
  double velocity_variance_{0.0};
  double acceleration_raw_{0.0};
  double acceleration_{0.0};
  double acceleration_variance_{0.0};
  rclcpp::Time velocity_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time acceleration_stamp_{0, 0, RCL_ROS_TIME};
  size_t velocity_samples_{0};
  size_t acceleration_samples_{0};
  size_t rejected_samples_{0};
  size_t acceleration_clamps_{0};
};

}  // namespace nav2_longitudinal_state_estimator

#endif  // NAV2_LONGITUDINAL_STATE_ESTIMATOR__LONGITUDINAL_STATE_ADAPTER_HPP_
