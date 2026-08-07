// Copyright (c) 2026
//
// [SteeringAngleControl] BT 一次性静止对舵客户端实现，不携带驻车锁存策略。

#include <memory>
#include <string>

#include "nav2_behavior_tree/plugins/action/steering_angle_control_action.hpp"

namespace nav2_behavior_tree
{

SteeringAngleControlAction::SteeringAngleControlAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav_interfaces::action::SteeringAngleControl>(
    xml_tag_name, action_name, conf)
{
}

void SteeringAngleControlAction::on_tick()
{
  double front_left_angle_rad = 0.785;
  double front_right_angle_rad = -0.785;
  double rear_left_angle_rad = -0.785;
  double rear_right_angle_rad = 0.785;
  double timeout_sec = 3.0;
  double settle_tolerance_rad = 0.08;
  getInput("front_left_angle_rad", front_left_angle_rad);
  getInput("front_right_angle_rad", front_right_angle_rad);
  getInput("rear_left_angle_rad", rear_left_angle_rad);
  getInput("rear_right_angle_rad", rear_right_angle_rad);
  getInput("timeout_sec", timeout_sec);
  getInput("settle_tolerance_rad", settle_tolerance_rad);

  if (timeout_sec <= 0.0) {
    timeout_sec = 3.0;
  }
  if (settle_tolerance_rad <= 0.0) {
    settle_tolerance_rad = 0.08;
  }

  goal_.target_steer_angles_rad = {
    static_cast<float>(front_left_angle_rad),
    static_cast<float>(front_right_angle_rad),
    static_cast<float>(rear_left_angle_rad),
    static_cast<float>(rear_right_angle_rad)};
  goal_.timeout_sec = static_cast<float>(timeout_sec);
  goal_.settle_tolerance_rad = static_cast<float>(settle_tolerance_rad);

  RCLCPP_INFO(
    node_->get_logger(),
    "[BT][Action:SteeringAngleControl] request targets=[%.3f,%.3f,%.3f,%.3f] rad "
    "timeout=%.2f s tol=%.3f rad",
    goal_.target_steer_angles_rad[0], goal_.target_steer_angles_rad[1],
    goal_.target_steer_angles_rad[2], goal_.target_steer_angles_rad[3],
    goal_.timeout_sec, goal_.settle_tolerance_rad);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::SteeringAngleControlAction>(
        name, "steering_angle_control", config);
    };

  factory.registerBuilder<nav2_behavior_tree::SteeringAngleControlAction>(
    "SteeringAngleControl", builder);
}
