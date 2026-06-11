#include "loop_navigation_bt/loop_charging_bt.hpp"

namespace loop_charging_bt {

CallAutoChargingAction::CallAutoChargingAction(const std::string& name,
                                               const BT::NodeConfiguration& config,
                                               rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_(node_ptr) {
    action_client_ = rclcpp_action::create_client<AutoCharging>(node_, "/auto_charging");
}

BT::PortsList CallAutoChargingAction::providedPorts() { return {}; }

BT::NodeStatus CallAutoChargingAction::onStart() {
    RCLCPP_INFO(node_->get_logger(), "'CallAutoChargingAction' node started, sending /auto_charging action goal.");

    // Mark charging active and pause navigation while charging/undocking sequence runs
    try {
        config().blackboard->set("charging_active", true);
        config().blackboard->set("nav_paused", true);
    } catch (...) {
        RCLCPP_WARN(node_->get_logger(), "Failed to set flags on blackboard");
    }

    if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node_->get_logger(), "Action server /auto_charging not available.");
        return BT::NodeStatus::FAILURE;
    }

    goal_handle_.reset();
    goal_response_received_ = false;
    result_received_ = false;
    action_succeeded_ = false;
    result_message_.clear();

    AutoCharging::Goal goal;
    goal.force = false;

    auto send_goal_options = rclcpp_action::Client<AutoCharging>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](const GoalHandleAutoCharging::SharedPtr& goal_handle) {
        goal_response_received_ = true;
        goal_handle_ = goal_handle;
        if (!goal_handle_) {
            result_received_ = true;
            action_succeeded_ = false;
            result_message_ = "auto charging goal was rejected";
            RCLCPP_ERROR(node_->get_logger(), "Auto charging goal was rejected by action server.");
        }
    };
    send_goal_options.result_callback = [this](const GoalHandleAutoCharging::WrappedResult& result) {
        result_received_ = true;
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                action_succeeded_ = result.result ? result.result->success : false;
                result_message_ =
                    result.result ? result.result->message : "auto charging action succeeded with empty result";
                break;
            case rclcpp_action::ResultCode::ABORTED:
                action_succeeded_ = false;
                result_message_ = result.result ? result.result->message : "auto charging action aborted";
                break;
            case rclcpp_action::ResultCode::CANCELED:
                action_succeeded_ = false;
                result_message_ = result.result ? result.result->message : "auto charging action canceled";
                break;
            default:
                action_succeeded_ = false;
                result_message_ = "auto charging action returned unknown result";
                break;
        }
    };

    action_client_->async_send_goal(goal, send_goal_options);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CallAutoChargingAction::onRunning() {
    if (!goal_response_received_) {
        return BT::NodeStatus::RUNNING;
    }

    if (!goal_handle_ && result_received_) {
        RCLCPP_ERROR(node_->get_logger(), "/auto_charging action goal rejected: %s", result_message_.c_str());
        return BT::NodeStatus::FAILURE;
    }

    if (!result_received_) {
        return BT::NodeStatus::RUNNING;
    }

    if (action_succeeded_) {
        RCLCPP_INFO(node_->get_logger(), "/auto_charging action succeeded: %s", result_message_.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_ERROR(node_->get_logger(), "/auto_charging action failed: %s", result_message_.c_str());
    return BT::NodeStatus::FAILURE;
}

void CallAutoChargingAction::onHalted() {
    RCLCPP_WARN(node_->get_logger(), "'CallAutoChargingAction' node halted.");

    if (goal_handle_) {
        action_client_->async_cancel_goal(goal_handle_);
    }
}

}  // namespace loop_charging_bt
