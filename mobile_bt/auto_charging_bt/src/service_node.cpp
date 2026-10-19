#include <behaviortree_cpp_v3/bt_factory.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "auto_charging_bt/auto_charging_bt.hpp"

#include <memory>
#include <sstream>
#include <utility>

// 全局节点指针定义（与库中的声明匹配）
std::shared_ptr<AutoChargingNode> charging_node;

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

// 执行一次完整的回充流程，返回 true 表示成功，false 表示失败
bool run_charging_flow(std::string& message) {
    RCLCPP_INFO(charging_node->get_logger(), "Auto-charging task started.");

    // 注册行为树节点工厂
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
    if (charging_node->confirmEntryChargerState() == AutoChargingNode::ChargerState::CHARGING) {
        message = "robot is already charging";
        RCLCPP_INFO(charging_node->get_logger(),
                    "Auto-charging skipped; robot is already charging.");
        return true;
    }

    // 读取充电点配置
    geometry_msgs::msg::PoseStamped goal_pose;
    ChargingPointAsset charging_point;
    rclcpp::Subscription<nav_interfaces::msg::MapState>::SharedPtr map_state_subscription;
    bool current_map_changed = false;
    std::string map_change_message;
    AutoChargingNode::NavigationBackendSnapshot navigation_backend;
    try {
        navigation_backend = charging_node->waitForNavigationBackend(
            charging_node->get_parameter(
                "charging.navigation_backend_wait_timeout_sec").as_double());
    } catch (const std::exception& e) {
        message = e.what();
        RCLCPP_ERROR(charging_node->get_logger(), "%s", message.c_str());
        charging_node->stopDetection();
        return false;
    }

    const bool navigate_to_charging =
        navigation_backend.name == AutoChargingNode::kNavigationBackendNav2;
    if (navigate_to_charging) {
        RCLCPP_INFO(charging_node->get_logger(),
                    "Auto-charging mode selected; backend=nav2, navigate before docking.");
        try {
            charging_point = readChargingPointFromCurrentMap(
                charging_node,
                charging_node->get_parameter("charging.map_state_wait_timeout_sec").as_double());
            goal_pose = charging_point.pose;
            RCLCPP_INFO(charging_node->get_logger(),
                        "Charging point loaded; name=%s, file=%s.",
                        charging_point.name.c_str(), charging_point.assets_file.c_str());
        } catch (const std::exception& e) {
            message = e.what();
            RCLCPP_ERROR(charging_node->get_logger(), "%s", message.c_str());
            charging_node->stopDetection();
            return false;
        }

        const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        map_state_subscription = charging_node->create_subscription<nav_interfaces::msg::MapState>(
            "/map_manager/map_state", qos,
            [&current_map_changed, &map_change_message, charging_point](
                const nav_interfaces::msg::MapState::SharedPtr msg) {
                if (current_map_changed ||
                    (msg->epoch == charging_point.map_epoch &&
                     msg->map_folder == charging_point.map_folder &&
                     msg->map_name == charging_point.map_name)) {
                    return;
                }

                std::ostringstream output;
                output << "Auto-charging stopped; map changed, expected="
                       << charging_point.map_name << "@" << charging_point.map_epoch
                       << ", actual=" << msg->map_name << "@" << msg->epoch << ".";
                map_change_message = output.str();
                current_map_changed = true;
            });
    } else {
        RCLCPP_INFO(charging_node->get_logger(),
                    "Auto-charging mode selected; backend=path_navigation, dock directly.");
    }

    charging_node->setTaskInterruptionCheck(
        [node = charging_node, navigation_backend, &current_map_changed]() {
            return current_map_changed || node->navigationBackendChanged(navigation_backend);
        });
    TaskInterruptionReset interruption_reset(charging_node);

    // 初始化Blackboard
    auto blackboard = BT::Blackboard::create();
    blackboard->set("charging_node", charging_node);
    blackboard->set("retry_count", 0);
    blackboard->set("nav_attempt_count", 0);
    blackboard->set("dock_attempt_count", 0);
    blackboard->set("charging_pose", goal_pose);
    blackboard->set("navigate_to_charging", navigate_to_charging);
    blackboard->set("dock_timeout",
                    charging_node->get_parameter("charging.dock_timeout_sec").as_double());
    blackboard->set("qr_detection_timeout",
                    charging_node->get_parameter("charging.qr_detection_timeout_sec").as_double());
    blackboard->set("undocking_duration",
                    charging_node->get_parameter("undocking.duration").as_double());

    // 从XML文件加载行为树
    std::string xml_path = ament_index_cpp::get_package_share_directory("auto_charging_bt") + "/config/charging_bt.xml";
    BT::Tree tree;
    try {
        tree = factory.createTreeFromFile(xml_path, blackboard);
    } catch (const std::exception& e) {
        message = std::string("Behavior tree loading failed; file=") + xml_path +
                  ", error=" + e.what() + ".";
        RCLCPP_ERROR(charging_node->get_logger(), "%s", message.c_str());
        charging_node->stopDetection();  // 确保停止检测服务
        return false;
    }

    // 主执行循环
    rclcpp::Rate rate(10);
    bool success = false;
    while (rclcpp::ok()) {
        // 先派发 Nav2、检测目标和诊断回调，再评估行为树状态，避免状态判断落后一个 tick。
        rclcpp::spin_some(charging_node);
        if (charging_node->navigationBackendChanged(navigation_backend)) {
            std::ostringstream output;
            output << "Auto-charging stopped; navigation backend changed, expected="
                   << navigation_backend.name << ", actual="
                   << charging_node->currentNavigationBackendName() << ".";
            message = output.str();
            RCLCPP_ERROR(charging_node->get_logger(), "%s", message.c_str());
            tree.haltTree();
            const bool nav_cancel_confirmed = charging_node->waitForNavCancellation(5.0);
            if (!nav_cancel_confirmed) {
                message += " Nav2 goal cancel was not confirmed.";
            }
            success = false;
            break;
        }
        if (current_map_changed) {
            RCLCPP_ERROR(charging_node->get_logger(), "%s", map_change_message.c_str());
            tree.haltTree();
            const bool nav_cancel_confirmed = charging_node->waitForNavCancellation(5.0);
            message = map_change_message;
            if (!nav_cancel_confirmed) {
                message += " Nav2 goal cancel was not confirmed.";
            }
            success = false;
            break;
        }
        BT::NodeStatus status = tree.tickRoot();
        if (status == BT::NodeStatus::SUCCESS) {
            RCLCPP_INFO(charging_node->get_logger(), "Auto-charging task succeeded.");
            success = true;
            message = "auto_charging success";
            break;
        } else if (status == BT::NodeStatus::FAILURE) {
            RCLCPP_ERROR(charging_node->get_logger(), "Auto-charging task failed.");
            success = false;
            message = "auto_charging failed";
            break;
        }
        rate.sleep();
    }

    // 确保停止
    // charging_node->backward(0.0, 0.01);
    for (int i = 0; i < 5; i++) charging_node->publishTwist(0.0, 0.0);

    // 无论成功或失败，都停止检测服务
    charging_node->stopDetection();

    return success;
}

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // 创建全局功能节点
    charging_node = std::make_shared<AutoChargingNode>();

    // 创建 service：/auto_charging 使用 Trigger
    auto node = std::make_shared<rclcpp::Node>("auto_charging_service_node");
    auto service = node->create_service<std_srvs::srv::Trigger>(
        "/auto_charging", [](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            std::string message;
            bool ok = run_charging_flow(message);
            response->success = ok;
            response->message = message;
        });

    RCLCPP_INFO(node->get_logger(), "Auto-charging service ready; service=/auto_charging.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
