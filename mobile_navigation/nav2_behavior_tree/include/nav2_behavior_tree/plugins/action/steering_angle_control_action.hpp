// Copyright (c) 2026
//
// [SteeringAngleControl] BT Action 客户端：调用 /steering_angle_control，
// 在四轮零速条件下完成一次目标舵角对齐；是否用于驻车由行为树上层决定。
// 对应 Server 在 mobile_joy_control/mobile_cmd_omni 中实现。

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__STEERING_ANGLE_CONTROL_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__STEERING_ANGLE_CONTROL_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav_interfaces/action/steering_angle_control.hpp"

namespace nav2_behavior_tree
{

class SteeringAngleControlAction
  : public BtActionNode<nav_interfaces::action::SteeringAngleControl>
{
public:
  SteeringAngleControlAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<double>("front_left_angle_rad", 0.785, "Front-left steer angle (rad)"),
        BT::InputPort<double>("front_right_angle_rad", -0.785, "Front-right steer angle (rad)"),
        BT::InputPort<double>("rear_left_angle_rad", -0.785, "Rear-left steer angle (rad)"),
        BT::InputPort<double>("rear_right_angle_rad", 0.785, "Rear-right steer angle (rad)"),
        BT::InputPort<double>("timeout_sec", 3.0, "Max time to settle (s)"),
        BT::InputPort<double>(
          "settle_tolerance_rad", 0.08, "Max |error| per wheel to succeed (rad)"),
      });
  }
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__STEERING_ANGLE_CONTROL_ACTION_HPP_
