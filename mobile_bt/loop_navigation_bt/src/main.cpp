#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>

#include <rclcpp/rclcpp.hpp>
// #include "ament_index_cpp/get_package_share_directory.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "loop_navigation_bt/loop_charging_bt.hpp"
#include "loop_navigation_bt/logging.hpp"
#include "loop_navigation_bt/loop_navigation_bt.hpp"

// 将同一个 ROS 2 节点注入每个行为树自定义节点。
template <typename T>
auto create_builder(rclcpp::Node::SharedPtr node) {
    return [node](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<T>(name, config, node);
    };
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("loop_navigation_bt_node");

    LOOP_RCLCPP_INFO(node->get_logger(), "Starting loop navigation behavior tree node.");

    // 这些参数由 config/loop_navigation_bt_params.yaml 统一加载。
    const double tick_rate_hz = node->declare_parameter<double>("loop.tick_rate_hz", 10.0);
    const double low_battery_threshold = node->declare_parameter<double>("battery.low_threshold", 30.0);
    const double resume_battery_threshold = node->declare_parameter<double>("battery.resume_threshold", 80.0);
    const double undocking_duration = node->declare_parameter<double>("undocking.duration_sec", 4.0);
    const double undocking_linear_speed = node->declare_parameter<double>("undocking.linear_speed", 0.15);

    if (tick_rate_hz <= 0.0 ||
        low_battery_threshold < 0.0 || low_battery_threshold > 100.0 ||
        resume_battery_threshold <= low_battery_threshold || resume_battery_threshold > 100.0 ||
        undocking_duration < 0.0 || undocking_linear_speed < 0.0) {
        LOOP_RCLCPP_ERROR(node->get_logger(),
                     "循环回充参数非法：请检查 tick 频率、电量阈值和退桩参数。");
        rclcpp::shutdown();
        return 1;
    }

    LOOP_RCLCPP_INFO(node->get_logger(), "回充电量参数：当前电量 < %.2f%% 时触发回充，达到 %.2f%% 后允许退桩。",
                     low_battery_threshold, resume_battery_threshold);

    BT::BehaviorTreeFactory factory;

    // 注册行为树自定义节点，并把同一个 ROS 2 节点传给它们。
    factory.registerBuilder<loop_navigation_bt::LoopNavigateFourPoints>(
        "LoopNavigateFourPoints", create_builder<loop_navigation_bt::LoopNavigateFourPoints>(node));
    factory.registerBuilder<loop_navigation_bt::CheckBatteryAboveThreshold>(
        "CheckBatteryAboveThreshold", create_builder<loop_navigation_bt::CheckBatteryAboveThreshold>(node));
    factory.registerBuilder<loop_navigation_bt::WaitForBatteryAboveThreshold>(
        "WaitForBatteryAboveThreshold", create_builder<loop_navigation_bt::WaitForBatteryAboveThreshold>(node));
    factory.registerBuilder<loop_navigation_bt::CancelNavigation>(
        "CancelNavigation", create_builder<loop_navigation_bt::CancelNavigation>(node));
    factory.registerBuilder<loop_charging_bt::CallAutoChargingAction>(
        "CallCharging", create_builder<loop_charging_bt::CallAutoChargingAction>(node));
    factory.registerBuilder<loop_navigation_bt::Undocking>("Undocking",
                                                           create_builder<loop_navigation_bt::Undocking>(node));

    // 从安装目录加载行为树 XML 和导航点 JSON。
    std::string pkg_share_dir;
    try {
        pkg_share_dir = ament_index_cpp::get_package_share_directory("loop_navigation_bt");
    } catch (const std::exception& e) {
        LOOP_RCLCPP_ERROR(node->get_logger(), "Package 'loop_navigation_bt' not found. Cannot locate config files.");
        return 1;
    }

    std::string xml_file = pkg_share_dir + "/config/loop_navigate_bt.xml";
    std::string points_file = pkg_share_dir + "/config/navigation_points.json";

    // std::string xml_file =
    //     "/home/dexforce/w1/dexe_mobile_application/mobile_bt/loop_navigation_bt/config/loop_navigate_bt.xml";
    // std::string points_file =
    //     "/home/dexforce/w1/dexe_mobile_application/mobile_bt/loop_navigation_bt/config/navigation_points.json";

    // blackboard 保存跨节点共享的配置和流程状态。
    auto blackboard = BT::Blackboard::create();
    blackboard->set("navigation_points", points_file);
    blackboard->set("low_battery_threshold", low_battery_threshold);
    blackboard->set("resume_battery_threshold", resume_battery_threshold);
    blackboard->set("undocking_duration", undocking_duration);
    blackboard->set("undocking_linear_speed", undocking_linear_speed);
    blackboard->set("charging_active", false);
    blackboard->set("nav_paused", false);
    blackboard->set("should_stop_navigation", false);
    blackboard->set("nav_loop_complete", false);

    LOOP_RCLCPP_INFO(node->get_logger(), "Loading behavior tree from: %s", xml_file.c_str());

    BT::Tree tree;
    try {
        tree = factory.createTreeFromFile(xml_file, blackboard);
    } catch (const std::exception& e) {
        LOOP_RCLCPP_ERROR(node->get_logger(), "Failed to load behavior tree: %s", e.what());
        return 1;
    }

    // Add loggers for debugging
    // BT::StdCoutLogger logger_cout(tree);
    // BT::FileLogger logger_file(tree, "bt_trace.fbl");

    LOOP_RCLCPP_INFO(node->get_logger(), "Behavior tree started.");

    // 主循环只负责 tick 行为树和处理 ROS 回调，具体状态由行为树节点维护。
    rclcpp::Rate rate(tick_rate_hz);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        try {
            if (blackboard->get<bool>("nav_loop_complete")) {
                LOOP_RCLCPP_INFO(node->get_logger(), "Navigation loop completed. Shutting down program.");
                break;
            }
        } catch (...) {
        }

        BT::NodeStatus status = tree.tickRoot();

        // 行为树返回 FAILURE 表示导航、回充或参数链路出现不可恢复错误。
        if (status == BT::NodeStatus::FAILURE) {
            LOOP_RCLCPP_ERROR(node->get_logger(), "Behavior tree returned FAILURE. Shutting down.");
            break;
        }

        rate.sleep();
    }

    LOOP_RCLCPP_INFO(node->get_logger(), "Shutting down node.");
    rclcpp::shutdown();
    return 0;
}
