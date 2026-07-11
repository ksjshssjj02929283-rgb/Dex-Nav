#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>

#include <rclcpp/rclcpp.hpp>
// #include "ament_index_cpp/get_package_share_directory.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "loop_navigation_bt/loop_charging_bt.hpp"
#include "loop_navigation_bt/loop_navigation_bt.hpp"

// Helper function to create a node builder
template <typename T>
auto create_builder(rclcpp::Node::SharedPtr node) {
    return [node](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<T>(name, config, node);
    };
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("loop_navigation_bt_node");

    RCLCPP_INFO(node->get_logger(), "Starting loop navigation behavior tree node.");

    BT::BehaviorTreeFactory factory;

    // Register all the custom nodes, passing the ROS2 node to them
    factory.registerBuilder<loop_navigation_bt::LoopNavigateFourPoints>(
        "LoopNavigateFourPoints", create_builder<loop_navigation_bt::LoopNavigateFourPoints>(node));
    factory.registerBuilder<loop_navigation_bt::CheckBatteryAboveThreshold>(
        "CheckBatteryAboveThreshold", create_builder<loop_navigation_bt::CheckBatteryAboveThreshold>(node));
    factory.registerBuilder<loop_navigation_bt::CancelNavigation>(
        "CancelNavigation", create_builder<loop_navigation_bt::CancelNavigation>(node));
    factory.registerBuilder<loop_charging_bt::CallAutoChargingAction>(
        "CallCharging",
        create_builder<loop_charging_bt::CallAutoChargingAction>(node));  // Registering as "CallCharging"
    factory.registerBuilder<loop_navigation_bt::Undocking>("Undocking",
                                                           create_builder<loop_navigation_bt::Undocking>(node));

    // Find the package share directory to locate config files
    std::string pkg_share_dir;
    try {
        pkg_share_dir = ament_index_cpp::get_package_share_directory("loop_navigation_bt");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Package 'loop_navigation_bt' not found. Cannot locate config files.");
        return 1;
    }

    std::string xml_file = pkg_share_dir + "/config/loop_navigate_bt.xml";
    std::string points_file = pkg_share_dir + "/config/navigation_points.json";

    // std::string xml_file =
    //     "/home/dexforce/w1/dexe_mobile_application/mobile_bt/loop_navigation_bt/config/loop_navigate_bt.xml";
    // std::string points_file =
    //     "/home/dexforce/w1/dexe_mobile_application/mobile_bt/loop_navigation_bt/config/navigation_points.json";

    // Set the path to the navigation points on the blackboard
    auto blackboard = BT::Blackboard::create();
    blackboard->set("navigation_points", points_file);

    RCLCPP_INFO(node->get_logger(), "Loading behavior tree from: %s", xml_file.c_str());

    BT::Tree tree;
    try {
        tree = factory.createTreeFromFile(xml_file, blackboard);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Failed to load behavior tree: %s", e.what());
        return 1;
    }

    // Add loggers for debugging
    // BT::StdCoutLogger logger_cout(tree);
    // BT::FileLogger logger_file(tree, "bt_trace.fbl");

    RCLCPP_INFO(node->get_logger(), "Behavior tree started.");

    // Main execution loop
    rclcpp::Rate rate(10);  // 10Hz
    while (rclcpp::ok()) {
        BT::NodeStatus status = tree.tickRoot();

        // The main loop is a ReactiveSequence, so it will run indefinitely unless
        // a major failure occurs in the tree structure itself.
        if (status == BT::NodeStatus::FAILURE) {
            RCLCPP_ERROR(node->get_logger(), "Behavior tree returned FAILURE. Shutting down.");
            break;
        }

        // Spin to process ROS2 callbacks (e.g., for battery state)
        rclcpp::spin_some(node);
        rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(), "Shutting down node.");
    rclcpp::shutdown();
    return 0;
}
