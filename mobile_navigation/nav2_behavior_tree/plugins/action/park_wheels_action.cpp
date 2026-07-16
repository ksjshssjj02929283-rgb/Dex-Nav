// Copyright (c) 2026
//
// [ParkWheels] BT Action 客户端实现。

#include <memory>
#include <string>

#include "nav2_behavior_tree/plugins/action/park_wheels_action.hpp"

namespace nav2_behavior_tree
{

ParkWheelsAction::ParkWheelsAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<mobile_joy_control::action::ParkWheels>(xml_tag_name, action_name, conf)
{
}

void ParkWheelsAction::on_tick()
{
  double steer_angle_rad = 0.785;
  double timeout_sec = 3.0;
  double settle_tolerance_rad = 0.08;
  getInput("steer_angle_rad", steer_angle_rad);
  getInput("timeout_sec", timeout_sec);
  getInput("settle_tolerance_rad", settle_tolerance_rad);

  if (steer_angle_rad <= 0.0) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[ParkWheels] steer_angle_rad=%.3f invalid, using 0.785", steer_angle_rad);
    steer_angle_rad = 0.785;
  }
  if (timeout_sec <= 0.0) {
    timeout_sec = 3.0;
  }
  if (settle_tolerance_rad <= 0.0) {
    settle_tolerance_rad = 0.08;
  }

  goal_.steer_angle_rad = static_cast<float>(steer_angle_rad);
  goal_.timeout_sec = static_cast<float>(timeout_sec);
  goal_.settle_tolerance_rad = static_cast<float>(settle_tolerance_rad);

  RCLCPP_INFO(
    node_->get_logger(),
    "[BT][Action:ParkWheels] request steer=%.3f rad timeout=%.2f s tol=%.3f rad",
    goal_.steer_angle_rad, goal_.timeout_sec, goal_.settle_tolerance_rad);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      // 默认 Action Server 名：/park_wheels（由 mobile_cmd_omni 提供）
      return std::make_unique<nav2_behavior_tree::ParkWheelsAction>(
        name, "park_wheels", config);
    };

  factory.registerBuilder<nav2_behavior_tree::ParkWheelsAction>("ParkWheels", builder);
}
