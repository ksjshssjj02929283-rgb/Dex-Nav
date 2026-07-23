#include "loop_navigation_bt/loop_charging_bt.hpp"
#include "loop_navigation_bt/logging.hpp"

namespace loop_charging_bt {

CallAutoChargingAction::CallAutoChargingAction(const std::string& name,
                                               const BT::NodeConfiguration& config,
                                               rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_(node_ptr) {
    action_client_ = rclcpp_action::create_client<AutoCharging>(node_, "/auto_charging");
    if (!node_->has_parameter("charging.force")) {
        node_->declare_parameter("charging.force", force_charging_);
    }
    if (!node_->has_parameter("charging.action_server_wait_timeout_sec")) {
        node_->declare_parameter("charging.action_server_wait_timeout_sec", action_server_wait_timeout_sec_);
    }
    node_->get_parameter("charging.force", force_charging_);
    node_->get_parameter("charging.action_server_wait_timeout_sec", action_server_wait_timeout_sec_);
    if (action_server_wait_timeout_sec_ <= 0.0) {
        LOOP_RCLCPP_WARN(node_->get_logger(),
                    "charging.action_server_wait_timeout_sec 必须大于 0，重置为 1.0 秒。");
        action_server_wait_timeout_sec_ = 1.0;
    }
}

BT::PortsList CallAutoChargingAction::providedPorts() { return {}; }

BT::NodeStatus CallAutoChargingAction::onStart() {
    LOOP_RCLCPP_INFO(node_->get_logger(),
                     "回充动作节点 CallAutoChargingAction 已启动，正在向 /auto_charging 发送回充目标。");

    // 标记回充流程已接管机器人，直到退桩完成才允许恢复循环导航。
    try {
        config().blackboard->set("charging_active", true);
        config().blackboard->set("nav_paused", true);
    } catch (...) {
        LOOP_RCLCPP_WARN(node_->get_logger(), "设置行为树黑板中的回充状态标志失败。");
    }

    if (!action_client_->wait_for_action_server(std::chrono::duration<double>(action_server_wait_timeout_sec_))) {
        LOOP_RCLCPP_ERROR(node_->get_logger(), "回充动作服务 /auto_charging 当前不可用。");
        return BT::NodeStatus::FAILURE;
    }

    goal_handle_.reset();
    goal_response_received_ = false;
    result_received_ = false;
    action_succeeded_ = false;
    result_message_.clear();

    AutoCharging::Goal goal;
    goal.force = force_charging_;

    const std::uint64_t goal_generation = ++goal_generation_;
    auto send_goal_options = rclcpp_action::Client<AutoCharging>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        [this, goal_generation](const GoalHandleAutoCharging::SharedPtr& goal_handle) {
        if (goal_generation != goal_generation_.load()) {
            return;
        }
        goal_response_received_ = true;
        goal_handle_ = goal_handle;
        if (!goal_handle_) {
            result_received_ = true;
            action_succeeded_ = false;
            result_message_ = "回充目标被动作服务拒绝";
            LOOP_RCLCPP_ERROR(node_->get_logger(), "回充目标被动作服务 /auto_charging 拒绝。");
        }
    };
    send_goal_options.result_callback =
        [this, goal_generation](const GoalHandleAutoCharging::WrappedResult& result) {
        if (goal_generation != goal_generation_.load()) {
            return;
        }
        result_received_ = true;
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                action_succeeded_ = result.result ? result.result->success : false;
                result_message_ = result.result ? result.result->message : "回充动作成功，但返回结果为空";
                break;
            case rclcpp_action::ResultCode::ABORTED:
                action_succeeded_ = false;
                result_message_ = result.result ? result.result->message : "回充动作已中止";
                break;
            case rclcpp_action::ResultCode::CANCELED:
                action_succeeded_ = false;
                result_message_ = result.result ? result.result->message : "回充动作已取消";
                break;
            default:
                action_succeeded_ = false;
                result_message_ = "回充动作返回了未知结果";
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
        LOOP_RCLCPP_ERROR(node_->get_logger(), "/auto_charging 回充目标被拒绝：%s", result_message_.c_str());
        return BT::NodeStatus::FAILURE;
    }

    if (!result_received_) {
        return BT::NodeStatus::RUNNING;
    }

    if (action_succeeded_) {
        LOOP_RCLCPP_INFO(node_->get_logger(), "/auto_charging 回充动作执行成功：%s", result_message_.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    LOOP_RCLCPP_ERROR(node_->get_logger(), "/auto_charging 回充动作执行失败：%s", result_message_.c_str());
    return BT::NodeStatus::FAILURE;
}

void CallAutoChargingAction::onHalted() {
    LOOP_RCLCPP_WARN(node_->get_logger(), "回充动作节点 CallAutoChargingAction 已停止。");

    ++goal_generation_;
    if (goal_handle_) {
        action_client_->async_cancel_goal(goal_handle_);
        goal_handle_.reset();
    }
}

}  // namespace loop_charging_bt
