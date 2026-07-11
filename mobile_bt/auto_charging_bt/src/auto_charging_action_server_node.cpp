#include <behaviortree_cpp_v3/bt_factory.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <thread>

#include "auto_charging_bt/auto_charging_bt.hpp"
#include "chassis_interfaces/action/auto_charging.hpp"

class AutoChargingActionServer : public rclcpp::Node {
public:
    using AutoCharging = chassis_interfaces::action::AutoCharging;
    using GoalHandleAutoCharging = rclcpp_action::ServerGoalHandle<AutoCharging>;

    AutoChargingActionServer() : Node("auto_charging_action_server") {
        charging_node_ = std::make_shared<AutoChargingNode>();

        action_server_ = rclcpp_action::create_server<AutoCharging>(
            this, "/auto_charging",
            std::bind(&AutoChargingActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&AutoChargingActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&AutoChargingActionServer::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "AutoCharging Action Server started");
    }

private:
    rclcpp_action::Server<AutoCharging>::SharedPtr action_server_;
    std::shared_ptr<AutoChargingNode> charging_node_;
    std::mutex execution_mutex_;

    // ============================
    // Goal
    // ============================
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&,
                                            std::shared_ptr<const AutoCharging::Goal> goal) {
        RCLCPP_INFO(get_logger(), "Receive auto charging goal, force = %d", goal->force);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // ============================
    // Cancel
    // ============================
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleAutoCharging>) {
        RCLCPP_WARN(get_logger(), "Auto charging canceled by client");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // ============================
    // Execute
    // ============================
    void handle_accepted(const std::shared_ptr<GoalHandleAutoCharging> goal_handle) {
        std::thread{std::bind(&AutoChargingActionServer::execute, this, goal_handle)}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleAutoCharging> goal_handle) {
        // 尝试获取互斥锁，如果已有任务在执行则立即返回失败
        std::unique_lock<std::mutex> lock(execution_mutex_, std::try_to_lock);
        if (!lock.owns_lock()) {
            RCLCPP_WARN(get_logger(), "❌ 回充任务已在执行中，拒绝新的请求");
            auto result = std::make_shared<AutoCharging::Result>();
            result->success = false;
            result->message = "Another auto charging task is already running";
            goal_handle->abort(result);
            return;
        }

        RCLCPP_INFO(get_logger(), "🔒 获取执行锁，开始执行回充任务");

        auto feedback = std::make_shared<AutoCharging::Feedback>();
        auto result = std::make_shared<AutoCharging::Result>();

        // ========= BT init =========
        BT::BehaviorTreeFactory factory;
        factory.registerNodeType<NavigateToCharging>("NavigateToCharging");
        factory.registerNodeType<Dock>("Dock");
        factory.registerNodeType<CheckCharging>("CheckCharging");
        factory.registerNodeType<CheckBatteryAboveThreshold>("CheckBatteryAboveThreshold");
        factory.registerNodeType<IsCharging>("IsCharging");
        factory.registerNodeType<IsConnectedToCharger>("IsConnectedToCharger");
        factory.registerNodeType<Undocking>("Undocking");
        factory.registerNodeType<CounterCheck>("CounterCheck");
        factory.registerNodeType<IncrementCounter>("IncrementCounter");
        factory.registerNodeType<AttemptLogger>("AttemptLogger");
        factory.registerNodeType<StartDetection>("StartDetection");
        factory.registerNodeType<StopDetection>("StopDetection");

        geometry_msgs::msg::PoseStamped goal_pose;
        try {
            goal_pose = readChargingPose(
                "/home/dexforce/w1/install/mobile_perception/lib/mobile_perception/charging_pose.json");
        } catch (const std::exception& e) {
            result->success = false;
            result->message = e.what();
            goal_handle->abort(result);
            return;
        }

        auto blackboard = BT::Blackboard::create();
        blackboard->set("charging_node", charging_node_);
        blackboard->set("retry_count", 0);
        blackboard->set("charging_pose", goal_pose);

        const std::string xml_path =
            ament_index_cpp::get_package_share_directory("auto_charging_bt") + "/config/charging_bt.xml";
        RCLCPP_INFO(charging_node_->get_logger(), "加载行为树 XML: %s", xml_path.c_str());
        BT::Tree tree;
        try {
            tree = factory.createTreeFromFile(xml_path, blackboard);
        } catch (const std::exception& e) {
            std::string message;
            message = std::string("加载行为树失败: ") + e.what();
            RCLCPP_ERROR(charging_node_->get_logger(), "%s", message.c_str());
            charging_node_->stopDetection();  // 确保停止检测服务
            result->success = false;
            result->message = message;
            goal_handle->abort(result);
            return;
        }

        rclcpp::Rate rate(10);
        RCLCPP_INFO(charging_node_->get_logger(), "开始执行回充行为树（action 调用）...");
        std::string last_status_str;
        auto last_status_log_time = get_clock()->now();
        bool status_log_initialized = false;

        // ========= Main loop =========
        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                charging_node_->stopDetection();
                for (int i = 0; i < 5; i++) charging_node_->publishTwist(0.0, 0.0);

                result->success = false;
                result->message = "Auto charging canceled";
                goal_handle->canceled(result);
                RCLCPP_INFO(get_logger(), "🔓 释放执行锁，回充任务被取消");
                return;
            }

            BT::NodeStatus status = tree.tickRoot();
            const std::string status_str = std::string(BT::toStr(status));
            const bool terminal_status =
                status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE;
            const double status_log_elapsed = (get_clock()->now() - last_status_log_time).seconds();
            if (!status_log_initialized || status_str != last_status_str || terminal_status ||
                status_log_elapsed >= 2.0) {
                RCLCPP_INFO(charging_node_->get_logger(), "行为树运行状态: %s", status_str.c_str());
                last_status_str = status_str;
                last_status_log_time = get_clock()->now();
                status_log_initialized = true;
            }

            feedback->retry_count = blackboard->get<int>("retry_count");
            feedback->current_state = status_str;

            goal_handle->publish_feedback(feedback);

            if (status == BT::NodeStatus::SUCCESS) {
                charging_node_->stopDetection();
                result->success = true;
                result->message = "auto charging success";
                goal_handle->succeed(result);
                RCLCPP_INFO(get_logger(), "🔓 释放执行锁，回充任务成功完成");
                return;
            }

            if (status == BT::NodeStatus::FAILURE) {
                charging_node_->stopDetection();
                result->success = false;
                result->message = "auto charging failed";
                goal_handle->abort(result);
                RCLCPP_INFO(get_logger(), "🔓 释放执行锁，回充任务失败");
                return;
            }

            rclcpp::spin_some(charging_node_);
            rate.sleep();
        }

        RCLCPP_INFO(get_logger(), "🔓 释放执行锁，回充任务异常退出");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutoChargingActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
