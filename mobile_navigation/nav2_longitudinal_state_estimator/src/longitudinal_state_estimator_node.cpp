// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0

#include "nav2_longitudinal_state_estimator/longitudinal_state_estimator_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace nav2_longitudinal_state_estimator
{

LongitudinalStateEstimatorNode::LongitudinalStateEstimatorNode(
  const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("longitudinal_state_estimator", "", options)
{
  declare_parameter(
    "velocity_topic", std::string("/localization/odometry/filtered"));
  declare_parameter(
    "acceleration_topic", std::string("/localization/accel/filtered"));
  declare_parameter("state_topic", state_topic_);
  declare_parameter("diagnostics_topic", diagnostics_topic_);
  declare_parameter("publish_frequency", publish_frequency_);

  declare_parameter("adapter.enabled", true);
  declare_parameter("adapter.max_age", 0.06);
  declare_parameter("adapter.max_stamp_skew", 0.01);
  declare_parameter("adapter.velocity_std_floor", 0.02);
  declare_parameter("adapter.acceleration_std_floor", 0.15);
  declare_parameter("adapter.max_velocity_std", 0.10);
  declare_parameter("adapter.max_acceleration_std", 0.60);
  declare_parameter("adapter.min_acceleration", -0.60);
  declare_parameter("adapter.max_acceleration", 0.70);
  declare_parameter("adapter.stationary_max_abs_velocity", 0.01);
  declare_parameter("adapter.stationary_max_abs_acceleration", 0.08);
  declare_parameter("adapter.min_velocity_samples", 5);
  declare_parameter("adapter.min_acceleration_samples", 5);
}

LongitudinalStateEstimatorNode::CallbackReturn
LongitudinalStateEstimatorNode::on_configure(const rclcpp_lifecycle::State &)
{
  LongitudinalStateAdapter::Config config;
  get_parameter("adapter.enabled", config.enabled);
  get_parameter("adapter.max_age", config.max_age);
  get_parameter("adapter.max_stamp_skew", config.max_stamp_skew);
  get_parameter("adapter.velocity_std_floor", config.velocity_std_floor);
  get_parameter("adapter.acceleration_std_floor", config.acceleration_std_floor);
  get_parameter("adapter.max_velocity_std", config.max_velocity_std);
  get_parameter("adapter.max_acceleration_std", config.max_acceleration_std);
  get_parameter("adapter.min_acceleration", config.min_acceleration);
  get_parameter("adapter.max_acceleration", config.max_acceleration);
  get_parameter(
    "adapter.stationary_max_abs_velocity", config.stationary_max_abs_velocity);
  get_parameter(
    "adapter.stationary_max_abs_acceleration", config.stationary_max_abs_acceleration);
  int min_velocity_samples = 5;
  int min_acceleration_samples = 5;
  get_parameter("adapter.min_velocity_samples", min_velocity_samples);
  get_parameter("adapter.min_acceleration_samples", min_acceleration_samples);
  config.min_velocity_samples = static_cast<size_t>(std::max(min_velocity_samples, 1));
  config.min_acceleration_samples = static_cast<size_t>(
    std::max(min_acceleration_samples, 1));
  adapter_.configure(config);

  std::string velocity_topic;
  std::string acceleration_topic;
  get_parameter("velocity_topic", velocity_topic);
  get_parameter("acceleration_topic", acceleration_topic);
  get_parameter("state_topic", state_topic_);
  get_parameter("diagnostics_topic", diagnostics_topic_);
  get_parameter("publish_frequency", publish_frequency_);
  publish_frequency_ = std::max(publish_frequency_, 1.0);

  velocity_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    velocity_topic, rclcpp::SensorDataQoS(),
    std::bind(
      &LongitudinalStateEstimatorNode::velocityCallback, this,
      std::placeholders::_1));
  acceleration_sub_ = create_subscription<
    geometry_msgs::msg::AccelWithCovarianceStamped>(
    acceleration_topic, rclcpp::SensorDataQoS(),
    std::bind(
      &LongitudinalStateEstimatorNode::accelerationCallback, this,
      std::placeholders::_1));
  state_pub_ = create_publisher<msg::LongitudinalState>(
    state_topic_, rclcpp::SensorDataQoS());
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    diagnostics_topic_, rclcpp::QoS(10).reliable());
  publish_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_frequency_),
    std::bind(&LongitudinalStateEstimatorNode::publishEstimate, this));

  RCLCPP_INFO(
    get_logger(),
    "Configured robot_localization longitudinal adapter: velocity='%s' "
    "acceleration='%s' state='%s' %.1fHz.",
    velocity_topic.c_str(), acceleration_topic.c_str(), state_topic_.c_str(),
    publish_frequency_);
  return CallbackReturn::SUCCESS;
}

LongitudinalStateEstimatorNode::CallbackReturn
LongitudinalStateEstimatorNode::on_activate(const rclcpp_lifecycle::State &)
{
  state_pub_->on_activate();
  diagnostics_pub_->on_activate();
  createBond();
  return CallbackReturn::SUCCESS;
}

LongitudinalStateEstimatorNode::CallbackReturn
LongitudinalStateEstimatorNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  state_pub_->on_deactivate();
  diagnostics_pub_->on_deactivate();
  destroyBond();
  return CallbackReturn::SUCCESS;
}

LongitudinalStateEstimatorNode::CallbackReturn
LongitudinalStateEstimatorNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  publish_timer_.reset();
  velocity_sub_.reset();
  acceleration_sub_.reset();
  state_pub_.reset();
  diagnostics_pub_.reset();
  adapter_.reset();
  return CallbackReturn::SUCCESS;
}

LongitudinalStateEstimatorNode::CallbackReturn
LongitudinalStateEstimatorNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

void LongitudinalStateEstimatorNode::velocityCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  adapter_.addVelocity(
    msg->twist.twist.linear.x, msg->twist.covariance[0], stamp);
}

void LongitudinalStateEstimatorNode::accelerationCallback(
  const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg)
{
  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  adapter_.addAcceleration(
    msg->accel.accel.linear.x, msg->accel.covariance[0], stamp);
}

void LongitudinalStateEstimatorNode::publishEstimate()
{
  if (!state_pub_ || !state_pub_->is_activated()) {
    return;
  }
  const auto estimate = adapter_.estimate(now());
  msg::LongitudinalState message;
  message.header.stamp = estimate.source_stamp;
  message.header.frame_id = "base_footprint";
  message.valid = estimate.valid;
  message.velocity_valid = estimate.velocity_valid;
  message.acceleration_valid = estimate.acceleration_valid;
  message.stationary = estimate.stationary;
  message.velocity = estimate.velocity;
  message.acceleration_raw = estimate.acceleration_raw;
  message.acceleration = estimate.acceleration;
  message.velocity_std = estimate.velocity_std;
  message.acceleration_std = estimate.acceleration_std;
  message.velocity_age = estimate.velocity_age;
  message.acceleration_age = estimate.acceleration_age;
  message.stamp_skew = estimate.stamp_skew;
  message.velocity_samples = estimate.velocity_samples;
  message.acceleration_samples = estimate.acceleration_samples;
  message.rejected_samples = estimate.rejected_samples;
  message.acceleration_clamps = estimate.acceleration_clamps;
  state_pub_->publish(message);
  publishDiagnostics(estimate);
}

void LongitudinalStateEstimatorNode::publishDiagnostics(
  const LongitudinalStateAdapter::Estimate & estimate)
{
  if (!diagnostics_pub_ || !diagnostics_pub_->is_activated()) {
    return;
  }
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "longitudinal_state_adapter";
  status.hardware_id = "robot_localization";
  status.level = estimate.valid ? diagnostic_msgs::msg::DiagnosticStatus::OK :
    diagnostic_msgs::msg::DiagnosticStatus::WARN;
  status.message = estimate.valid ? "valid" : "not_ready_stale_or_unsynchronized";
  const auto add = [&status](const std::string & key, const auto & value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      item.value = std::to_string(value);
      status.values.push_back(std::move(item));
    };
  add("velocity", estimate.velocity);
  add("acceleration_raw", estimate.acceleration_raw);
  add("acceleration", estimate.acceleration);
  add("velocity_std", estimate.velocity_std);
  add("acceleration_std", estimate.acceleration_std);
  add("velocity_age", estimate.velocity_age);
  add("acceleration_age", estimate.acceleration_age);
  add("stamp_skew", estimate.stamp_skew);
  add("rejected_samples", estimate.rejected_samples);
  add("acceleration_clamps", estimate.acceleration_clamps);
  array.status.push_back(std::move(status));
  diagnostics_pub_->publish(array);
}

}  // namespace nav2_longitudinal_state_estimator

RCLCPP_COMPONENTS_REGISTER_NODE(
  nav2_longitudinal_state_estimator::LongitudinalStateEstimatorNode)
