// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0

#ifndef NAV2_LONGITUDINAL_STATE_ESTIMATOR__LONGITUDINAL_STATE_ESTIMATOR_NODE_HPP_
#define NAV2_LONGITUDINAL_STATE_ESTIMATOR__LONGITUDINAL_STATE_ESTIMATOR_NODE_HPP_

#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "nav2_longitudinal_state_estimator/longitudinal_state_adapter.hpp"
#include "nav2_longitudinal_state_estimator/msg/longitudinal_state.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace nav2_longitudinal_state_estimator
{

class LongitudinalStateEstimatorNode : public nav2_util::LifecycleNode
{
public:
  using CallbackReturn = nav2_util::CallbackReturn;
  explicit LongitudinalStateEstimatorNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void velocityCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void accelerationCallback(
    const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg);
  void publishEstimate();
  void publishDiagnostics(const LongitudinalStateAdapter::Estimate & estimate);

  LongitudinalStateAdapter adapter_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr velocity_sub_;
  rclcpp::Subscription<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr
    acceleration_sub_;
  rclcpp_lifecycle::LifecyclePublisher<msg::LongitudinalState>::SharedPtr state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  double publish_frequency_{25.0};
  std::string state_topic_{"/navigation/longitudinal_state"};
  std::string diagnostics_topic_{"/navigation/longitudinal_state_diagnostics"};
};

}  // namespace nav2_longitudinal_state_estimator

#endif  // NAV2_LONGITUDINAL_STATE_ESTIMATOR__LONGITUDINAL_STATE_ESTIMATOR_NODE_HPP_
