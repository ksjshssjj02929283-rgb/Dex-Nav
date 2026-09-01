// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0

#include <memory>

#include "nav2_longitudinal_state_estimator/longitudinal_state_estimator_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<
    nav2_longitudinal_state_estimator::LongitudinalStateEstimatorNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
