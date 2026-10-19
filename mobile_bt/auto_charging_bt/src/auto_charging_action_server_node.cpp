#include <behaviortree_cpp_v3/bt_factory.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

#include "auto_charging_bt/auto_charging_bt.hpp"
#include "chassis_interfaces/action/auto_charging.hpp"

namespace {

class TaskInterruptionReset {
public:
    explicit TaskInterruptionReset(std::shared_ptr<AutoChargingNode> node)
        : node_(std::move(node)) {}

    ~TaskInterruptionReset() {
        if (node_) node_->clearTaskInterruptionCheck();
    }

private:
    std::shared_ptr<AutoChargingNode> node_;
};

}  // namespace

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

        RCLCPP_INFO(get_logger(), "Auto-charging action server ready; action=/auto_charging.");
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
        RCLCPP_INFO(get_logger(), "Auto-charging goal received; force=%s.",
                    goal->force ? "true" : "false");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // ============================
    // Cancel
    // ============================
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleAutoCharging>) {
        RCLCPP_INFO(get_logger(), "Auto-charging cancel requested.");
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
            RCLCPP_WARN(get_logger(), "Auto-charging goal rejected; another task is running.");
            auto result = std::make_shared<AutoCharging::Result>();
            result->success = false;
            result->message = "Another auto-charging task is running.";
            goal_handle->abort(result);
            return;
        }

        RCLCPP_INFO(get_logger(), "Auto-charging task started.");

        auto feedback = std::make_shared<AutoCharging::Feedback>();
        auto result = std::make_shared<AutoCharging::Result>();

        // ========= BT init =========
        BT::BehaviorTreeFactory factory;
        factory.registerNodeType<NavigateToCharging>("NavigateToCharging");
        factory.registerNodeType<Dock>("Dock");
        factory.registerNodeType<CheckCharging>("CheckCharging");
        factory.registerNodeType<CheckBatteryAboveThreshold>("CheckBatteryAboveThreshold");
        factory.registerNodeType<IsCharging>("IsCharging");
        factory.registerNodeType<IsChargingNavigationEnabled>("IsChargingNavigationEnabled");
        factory.registerNodeType<Undocking>("Undocking");
        factory.registerNodeType<CounterCheck>("CounterCheck");
        factory.registerNodeType<IncrementCounter>("IncrementCounter");
        factory.registerNodeType<AttemptLogger>("AttemptLogger");
        factory.registerNodeType<StartDetection>("StartDetection");
        factory.registerNodeType<StopDetection>("StopDetection");

        // 已经稳定处于充电状态时直接成功，不再依赖导航后端或地图资产。
        const auto entry_state = charging_node_->confirmEntryChargerState();
        if (goal_handle->is_canceling()) {
            result->success = false;
            result->message = "Auto-charging task canceled.";
            goal_handle->canceled(result);
            RCLCPP_INFO(get_logger(), "Auto-charging task canceled.");
            return;
        }
        if (entry_state == AutoChargingNode::ChargerState::CHARGING) {
            result->success = true;
            result->message = "Robot is already charging.";
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(), "Auto-charging skipped; robot is already charging.");
            return;
        }

        AutoChargingNode::NavigationBackendSnapshot navigation_backend;
        try {
            navigation_backend = charging_node_->waitForNavigationBackend(
                charging_node_->get_parameter(
                    "charging.navigation_backend_wait_timeout_sec").as_double());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(charging_node_->get_logger(), "%s", e.what());
            result->success = false;
            result->message = e.what();
            goal_handle->abort(result);
            return;
        }

        const bool navigate_to_charging =
            navigation_backend.name == AutoChargingNode::kNavigationBackendNav2;
        geometry_msgs::msg::PoseStamped goal_pose;
        ChargingPointAsset charging_point;
        rclcpp::Subscription<nav_interfaces::msg::MapState>::SharedPtr map_state_subscription;
        bool current_map_changed = false;
        std::string map_change_message;
        if (navigate_to_charging) {
            RCLCPP_INFO(charging_node_->get_logger(),
                        "Auto-charging mode selected; backend=nav2, navigate before docking.");
            try {
                charging_point = readChargingPointFromCurrentMap(
                    charging_node_,
                    charging_node_->get_parameter("charging.map_state_wait_timeout_sec").as_double());
                goal_pose = charging_point.pose;
                RCLCPP_INFO(charging_node_->get_logger(),
                            "Charging point loaded; name=%s, file=%s.",
                            charging_point.name.c_str(), charging_point.assets_file.c_str());
            } catch (const std::exception& e) {
                RCLCPP_ERROR(charging_node_->get_logger(), "%s", e.what());
                result->success = false;
                result->message = e.what();
                goal_handle->abort(result);
                return;
            }

            const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
            map_state_subscription = charging_node_->create_subscription<nav_interfaces::msg::MapState>(
                "/map_manager/map_state", qos,
                [&current_map_changed, &map_change_message, charging_point](
                    const nav_interfaces::msg::MapState::SharedPtr msg) {
                    if (current_map_changed ||
                        (msg->epoch == charging_point.map_epoch &&
                         msg->map_folder == charging_point.map_folder &&
                         msg->map_name == charging_point.map_name)) {
                        return;
                    }

                    std::ostringstream message;
                    message << "Auto-charging stopped; map changed, expected="
                            << charging_point.map_name << "@" << charging_point.map_epoch
                            << ", actual=" << msg->map_name << "@" << msg->epoch << ".";
                    map_change_message = message.str();
                    current_map_changed = true;
                });
        } else {
            RCLCPP_INFO(charging_node_->get_logger(),
                        "Auto-charging mode selected; backend=path_navigation, dock directly.");
        }

        charging_node_->setTaskInterruptionCheck(
            [node = charging_node_, goal_handle, navigation_backend, &current_map_changed]() {
                return goal_handle->is_canceling() || current_map_changed ||
                       node->navigationBackendChanged(navigation_backend);
            });
        TaskInterruptionReset interruption_reset(charging_node_);

        auto blackboard = BT::Blackboard::create();
        blackboard->set("charging_node", charging_node_);
        blackboard->set("retry_count", 0);
        blackboard->set("nav_attempt_count", 0);
        blackboard->set("dock_attempt_count", 0);
        blackboard->set("charging_pose", goal_pose);
        blackboard->set("navigate_to_charging", navigate_to_charging);
        blackboard->set("dock_timeout",
                        charging_node_->get_parameter("charging.dock_timeout_sec").as_double());
        blackboard->set("qr_detection_timeout",
                        charging_node_->get_parameter("charging.qr_detection_timeout_sec").as_double());
        blackboard->set("undocking_duration",
                        charging_node_->get_parameter("undocking.duration").as_double());

        const std::string xml_path =
            ament_index_cpp::get_package_share_directory("auto_charging_bt") + "/config/charging_bt.xml";
        BT::Tree tree;
        try {
            tree = factory.createTreeFromFile(xml_path, blackboard);
        } catch (const std::exception& e) {
            std::string message;
            message = std::string("Behavior tree loading failed; file=") + xml_path +
                      ", error=" + e.what() + ".";
            RCLCPP_ERROR(charging_node_->get_logger(), "%s", message.c_str());
            charging_node_->stopDetection();  // 确保停止检测服务
            result->success = false;
            result->message = message;
            goal_handle->abort(result);
            return;
        }

        rclcpp::Rate rate(10);

        // ========= Main loop =========
        while (rclcpp::ok()) {
            // 先派发 Nav2、检测目标和诊断回调，再评估行为树状态，避免状态判断落后一个 tick。
            rclcpp::spin_some(charging_node_);
            if (goal_handle->is_canceling()) {
                // halt 行为树，让 StatefulActionNode（如 NavigateToCharging、Dock）
                // 的 onHalted 被调用，从而取消 Nav2 goal / 停止对接控制器。
                // 否则 Nav2 会继续执行旧 goal，导致机器人仍向回充点移动。
                tree.haltTree();
                // 任务退出前持续处理 Nav2 回调，确保定向取消已经得到响应。
                const bool nav_cancel_confirmed = charging_node_->waitForNavCancellation(5.0);
                charging_node_->stopDetection();
                for (int i = 0; i < 5; i++) charging_node_->publishTwist(0.0, 0.0);

                result->success = false;
                result->message = nav_cancel_confirmed
                    ? "Auto-charging task canceled."
                    : "Auto-charging task canceled; Nav2 cancel was not confirmed.";
                goal_handle->canceled(result);
                RCLCPP_INFO(get_logger(), "Auto-charging task canceled.");
                return;
            }
            if (charging_node_->navigationBackendChanged(navigation_backend)) {
                std::ostringstream message;
                message << "Auto-charging stopped; navigation backend changed, expected="
                        << navigation_backend.name << ", actual="
                        << charging_node_->currentNavigationBackendName() << ".";
                const std::string backend_change_message = message.str();
                RCLCPP_ERROR(charging_node_->get_logger(), "%s", backend_change_message.c_str());
                tree.haltTree();
                const bool nav_cancel_confirmed = charging_node_->waitForNavCancellation(5.0);
                charging_node_->stopDetection();
                for (int i = 0; i < 5; i++) charging_node_->publishTwist(0.0, 0.0);

                result->success = false;
                result->message = backend_change_message;
                if (!nav_cancel_confirmed) {
                    result->message += " Nav2 goal cancel was not confirmed.";
                }
                goal_handle->abort(result);
                return;
            }
            if (current_map_changed) {
                RCLCPP_ERROR(charging_node_->get_logger(), "%s", map_change_message.c_str());
                tree.haltTree();
                const bool nav_cancel_confirmed = charging_node_->waitForNavCancellation(5.0);
                charging_node_->stopDetection();
                for (int i = 0; i < 5; i++) charging_node_->publishTwist(0.0, 0.0);

                result->success = false;
                result->message = map_change_message;
                if (!nav_cancel_confirmed) {
                    result->message += " Nav2 goal cancel was not confirmed.";
                }
                goal_handle->abort(result);
                return;
            }

            BT::NodeStatus status = tree.tickRoot();
            const std::string status_str = std::string(BT::toStr(status));

            // 行为树分别记录导航和对接尝试次数；第一次属于初始尝试，不计入重试次数。
            const int nav_attempt_count = blackboard->get<int>("nav_attempt_count");
            const int dock_attempt_count = blackboard->get<int>("dock_attempt_count");
            feedback->retry_count = std::max(0, nav_attempt_count - 1) +
                                    std::max(0, dock_attempt_count - 1);
            // 保持行为树根状态，供客户端按 RUNNING / SUCCESS / FAILURE 判断任务进度。
            feedback->current_state = status_str;

            goal_handle->publish_feedback(feedback);

            if (status == BT::NodeStatus::SUCCESS) {
                charging_node_->stopDetection();
                result->success = true;
                result->message = "Auto-charging task succeeded.";
                goal_handle->succeed(result);
                RCLCPP_INFO(get_logger(), "Auto-charging task succeeded.");
                return;
            }

            if (status == BT::NodeStatus::FAILURE) {
                charging_node_->stopDetection();
                result->success = false;
                result->message = "Auto-charging task failed.";
                goal_handle->abort(result);
                RCLCPP_ERROR(get_logger(), "Auto-charging task failed.");
                return;
            }

            rate.sleep();
        }

        RCLCPP_WARN(get_logger(), "Auto-charging task stopped; ROS is shutting down.");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutoChargingActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
