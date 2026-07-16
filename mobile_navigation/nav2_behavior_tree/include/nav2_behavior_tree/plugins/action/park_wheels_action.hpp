// Copyright (c) 2026
//
// [ParkWheels] BT Action 客户端：调用 /park_wheels，使四轮朝里驻车。
// 对应 Server 在 mobile_joy_control/mobile_cmd_omni 中实现。

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PARK_WHEELS_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PARK_WHEELS_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
// [ParkWheels] 接口在 mobile_joy_control 包内，不依赖定制 nav2_msgs
#include "mobile_joy_control/action/park_wheels.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief BT 节点 <ParkWheels/>：向 /park_wheels 发送驻车请求
 */
class ParkWheelsAction : public BtActionNode<mobile_joy_control::action::ParkWheels>
{
public:
  ParkWheelsAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<double>("steer_angle_rad", 0.785, "Inward steer magnitude (rad)"),
        BT::InputPort<double>("timeout_sec", 3.0, "Max time to settle (s)"),
        BT::InputPort<double>(
          "settle_tolerance_rad", 0.08, "Max |error| per wheel to succeed (rad)"),
      });
  }
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PARK_WHEELS_ACTION_HPP_
