// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0

#include "nav2_longitudinal_state_estimator/longitudinal_state_adapter.hpp"

#include <algorithm>
#include <cmath>

namespace nav2_longitudinal_state_estimator
{

LongitudinalStateAdapter::LongitudinalStateAdapter()
: LongitudinalStateAdapter(Config())
{
}

LongitudinalStateAdapter::LongitudinalStateAdapter(const Config & config)
{
  configure(config);
}

void LongitudinalStateAdapter::configure(const Config & config)
{
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;
  config_.max_age = std::max(config_.max_age, 0.0);
  config_.max_stamp_skew = std::max(config_.max_stamp_skew, 0.0);
  config_.velocity_std_floor = std::max(config_.velocity_std_floor, 1e-6);
  config_.acceleration_std_floor = std::max(config_.acceleration_std_floor, 1e-6);
  config_.max_velocity_std = std::max(
    config_.max_velocity_std, config_.velocity_std_floor);
  config_.max_acceleration_std = std::max(
    config_.max_acceleration_std, config_.acceleration_std_floor);
  if (config_.min_acceleration > config_.max_acceleration) {
    std::swap(config_.min_acceleration, config_.max_acceleration);
  }
  config_.stationary_max_abs_velocity = std::max(
    config_.stationary_max_abs_velocity, 0.0);
  config_.stationary_max_abs_acceleration = std::max(
    config_.stationary_max_abs_acceleration, 0.0);
  config_.min_velocity_samples = std::max<size_t>(config_.min_velocity_samples, 1);
  config_.min_acceleration_samples = std::max<size_t>(config_.min_acceleration_samples, 1);
  resetUnlocked();
}

bool LongitudinalStateAdapter::acceptClockUnlocked(const rclcpp::Time & stamp)
{
  if (!clock_initialized_) {
    clock_type_ = stamp.get_clock_type();
    clock_initialized_ = true;
    return true;
  }
  if (clock_type_ != stamp.get_clock_type()) {
    resetUnlocked();
    clock_type_ = stamp.get_clock_type();
    clock_initialized_ = true;
  }
  return true;
}

void LongitudinalStateAdapter::addVelocity(
  double velocity, double variance, const rclcpp::Time & stamp)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!std::isfinite(velocity) || !std::isfinite(variance) || variance < 0.0 ||
    stamp.nanoseconds() <= 0)
  {
    ++rejected_samples_;
    return;
  }
  acceptClockUnlocked(stamp);
  if (has_velocity_ && stamp <= velocity_stamp_) {
    ++rejected_samples_;
    return;
  }
  velocity_ = velocity;
  velocity_variance_ = variance;
  velocity_stamp_ = stamp;
  has_velocity_ = true;
  ++velocity_samples_;
}

void LongitudinalStateAdapter::addAcceleration(
  double acceleration, double variance, const rclcpp::Time & stamp)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!std::isfinite(acceleration) || !std::isfinite(variance) || variance < 0.0 ||
    stamp.nanoseconds() <= 0)
  {
    ++rejected_samples_;
    return;
  }
  acceptClockUnlocked(stamp);
  if (has_acceleration_ && stamp <= acceleration_stamp_) {
    ++rejected_samples_;
    return;
  }
  acceleration_raw_ = acceleration;
  acceleration_ = std::clamp(
    acceleration, config_.min_acceleration, config_.max_acceleration);
  if (std::fabs(acceleration_ - acceleration) > 1e-12) {
    ++acceleration_clamps_;
  }
  acceleration_variance_ = variance;
  acceleration_stamp_ = stamp;
  has_acceleration_ = true;
  ++acceleration_samples_;
}

LongitudinalStateAdapter::Estimate LongitudinalStateAdapter::estimate(
  const rclcpp::Time & now) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  Estimate result;
  result.velocity_samples = velocity_samples_;
  result.acceleration_samples = acceleration_samples_;
  result.rejected_samples = rejected_samples_;
  result.acceleration_clamps = acceleration_clamps_;
  if (!config_.enabled || !clock_initialized_ || !has_velocity_ || !has_acceleration_ ||
    now.get_clock_type() != clock_type_)
  {
    return result;
  }

  result.velocity = velocity_;
  result.acceleration_raw = acceleration_raw_;
  result.acceleration = acceleration_;
  result.velocity_std = std::sqrt(
    std::max(
      velocity_variance_, config_.velocity_std_floor * config_.velocity_std_floor));
  result.acceleration_std = std::sqrt(
    std::max(
      acceleration_variance_,
      config_.acceleration_std_floor * config_.acceleration_std_floor));
  result.velocity_age = (now - velocity_stamp_).seconds();
  result.acceleration_age = (now - acceleration_stamp_).seconds();
  result.stamp_skew = std::fabs((velocity_stamp_ - acceleration_stamp_).seconds());
  result.source_stamp = velocity_stamp_ < acceleration_stamp_ ?
    velocity_stamp_ : acceleration_stamp_;

  result.velocity_valid = std::isfinite(result.velocity_age) &&
    result.velocity_age >= 0.0 && result.velocity_age <= config_.max_age &&
    result.velocity_std <= config_.max_velocity_std &&
    velocity_samples_ >= config_.min_velocity_samples;
  result.acceleration_valid = std::isfinite(result.acceleration_age) &&
    result.acceleration_age >= 0.0 && result.acceleration_age <= config_.max_age &&
    result.acceleration_std <= config_.max_acceleration_std &&
    acceleration_samples_ >= config_.min_acceleration_samples;
  result.valid = result.velocity_valid && result.acceleration_valid &&
    result.stamp_skew <= config_.max_stamp_skew;
  result.stationary = result.valid &&
    std::fabs(result.velocity) <= config_.stationary_max_abs_velocity &&
    std::fabs(result.acceleration) <= config_.stationary_max_abs_acceleration;
  return result;
}

void LongitudinalStateAdapter::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  resetUnlocked();
}

void LongitudinalStateAdapter::resetUnlocked()
{
  clock_initialized_ = false;
  clock_type_ = RCL_ROS_TIME;
  has_velocity_ = false;
  has_acceleration_ = false;
  velocity_ = 0.0;
  velocity_variance_ = 0.0;
  acceleration_raw_ = 0.0;
  acceleration_ = 0.0;
  acceleration_variance_ = 0.0;
  velocity_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  acceleration_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  velocity_samples_ = 0;
  acceleration_samples_ = 0;
  rejected_samples_ = 0;
  acceleration_clamps_ = 0;
}

}  // namespace nav2_longitudinal_state_estimator
