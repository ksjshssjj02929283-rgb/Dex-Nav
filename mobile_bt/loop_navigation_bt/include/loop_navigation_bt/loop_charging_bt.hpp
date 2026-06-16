#ifndef LOOP_CHARGING_BT_HPP_
#define LOOP_CHARGING_BT_HPP_

#include "behaviortree_cpp_v3/action_node.h"
#include "chassis_interfaces/action/auto_charging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace loop_charging_bt {

class CallAutoChargingAction : public BT::StatefulActionNode {
public:
    using AutoCharging = chassis_interfaces::action::AutoCharging;
    using GoalHandleAutoCharging = rclcpp_action::ClientGoalHandle<AutoCharging>;

    CallAutoChargingAction(const std::string& name,
                           const BT::NodeConfiguration& config,
                           rclcpp::Node::SharedPtr node_ptr);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<AutoCharging>::SharedPtr action_client_;
    GoalHandleAutoCharging::SharedPtr goal_handle_;
    bool goal_response_received_{false};
    bool result_received_{false};
    bool action_succeeded_{false};
    std::string result_message_;
};

}  // namespace loop_charging_bt

#endif  // LOOP_CHARGING_BT_HPP_
