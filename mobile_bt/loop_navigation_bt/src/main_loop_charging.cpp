#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>

#include <rclcpp/rclcpp.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "loop_navigation_bt/loop_charging_bt.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("loop_charging_bt_node");

    BT::BehaviorTreeFactory factory;

    // Pass the ROS2 node to the custom node constructor
    auto call_charging_builder = [node](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<loop_charging_bt::CallAutoChargingAction>(name, config, node);
    };

    factory.registerBuilder<loop_charging_bt::CallAutoChargingAction>("CallAutoChargingService", call_charging_builder);

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("loop_navigation_bt");
    std::string xml_file = package_share_directory + "/config/loop_charging_bt.xml";

    RCLCPP_INFO(node->get_logger(), "Loading behavior tree from: %s", xml_file.c_str());

    BT::Tree tree = factory.createTreeFromFile(xml_file);

    // Add loggers
    BT::StdCoutLogger logger_cout(tree);
    BT::FileLogger logger_file(tree, "bt_trace.fbl");

    rclcpp::Rate rate(10);  // Tick the tree at 10Hz

    while (rclcpp::ok()) {
        BT::NodeStatus status = tree.tickRoot();
        if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) {
            RCLCPP_INFO(node->get_logger(), "Behavior tree finished with status: %s", BT::toStr(status).c_str());
            // In a looping tree, this might indicate a problem or an intended stop.
            // For this example, we'll just stop.
            break;
        }
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
