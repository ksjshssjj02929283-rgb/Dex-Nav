#ifndef LOOP_CHARGING_BT_HPP_
#define LOOP_CHARGING_BT_HPP_

#include <atomic>
#include <cstdint>

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
    // 用代次编号过滤上一次回充请求延迟到达的异步回调。
    std::atomic<std::uint64_t> goal_generation_{0};
    bool goal_response_received_{false};
    bool result_received_{false};
    bool action_succeeded_{false};
    bool force_charging_{false};                  // 是否强制执行回充，由 YAML 参数控制。
    double action_server_wait_timeout_sec_{1.0};  // 等待 /auto_charging Action server 的时间。
    std::string result_message_;
};

}  // namespace loop_charging_bt

#endif  // LOOP_CHARGING_BT_HPP_
