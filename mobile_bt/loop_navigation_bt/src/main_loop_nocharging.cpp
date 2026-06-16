/**
 * main_loop_nocharging.cpp
 *
 * 无充电循环导航入口：
 *   - 持续循环四个点位导航
 *   - 检测到电量低于阈值后，导航到第一个点位并停止程序
 *   - 不触发充电流程
 */

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "loop_navigation_bt/loop_navigation_bt.hpp"

template <typename T>
auto create_builder(rclcpp::Node::SharedPtr node) {
    return [node](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<T>(name, config, node);
    };
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("loop_nocharging_bt_node");

    RCLCPP_INFO(node->get_logger(), "Starting loop navigation (no-charging) behavior tree node.");

    BT::BehaviorTreeFactory factory;

    factory.registerBuilder<loop_navigation_bt::LoopNavigateFourPoints>(
        "LoopNavigateFourPoints", create_builder<loop_navigation_bt::LoopNavigateFourPoints>(node));
    factory.registerBuilder<loop_navigation_bt::CheckBatteryAboveThreshold>(
        "CheckBatteryAboveThreshold", create_builder<loop_navigation_bt::CheckBatteryAboveThreshold>(node));
    factory.registerBuilder<loop_navigation_bt::CancelNavigation>(
        "CancelNavigation", create_builder<loop_navigation_bt::CancelNavigation>(node));
    factory.registerBuilder<loop_navigation_bt::NavigateToFirstPointAndStop>(
        "NavigateToFirstPointAndStop", create_builder<loop_navigation_bt::NavigateToFirstPointAndStop>(node));

    std::string pkg_share_dir;
    try {
        pkg_share_dir = ament_index_cpp::get_package_share_directory("loop_navigation_bt");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Package 'loop_navigation_bt' not found: %s", e.what());
        return 1;
    }

    std::string xml_file = pkg_share_dir + "/config/loop_nocharging_bt.xml";
    std::string points_file = pkg_share_dir + "/config/navigation_points.json";

    auto blackboard = BT::Blackboard::create();
    blackboard->set("navigation_points", points_file);
    blackboard->set("low_battery_shutdown", false);

    RCLCPP_INFO(node->get_logger(), "Loading behavior tree from: %s", xml_file.c_str());

    BT::Tree tree;
    try {
        tree = factory.createTreeFromFile(xml_file, blackboard);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Failed to load behavior tree: %s", e.what());
        return 1;
    }

    BT::StdCoutLogger logger_cout(tree);

    RCLCPP_INFO(node->get_logger(), "Behavior tree started. Looping through navigation points...");

    rclcpp::Rate rate(10);  // 10 Hz
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        // Check if low-battery recovery has completed before next tick
        try {
            bool stop = blackboard->get<bool>("low_battery_shutdown");
            if (stop) {
                RCLCPP_INFO(node->get_logger(), "Low battery: robot returned to first point. Shutting down program.");
                break;
            }
        } catch (...) {
        }

        BT::NodeStatus status = tree.tickRoot();

        if (status == BT::NodeStatus::FAILURE) {
            RCLCPP_ERROR(node->get_logger(), "Behavior tree returned FAILURE. Shutting down.");
            break;
        }

        rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(), "Shutting down node.");
    rclcpp::shutdown();
    return 0;
}
