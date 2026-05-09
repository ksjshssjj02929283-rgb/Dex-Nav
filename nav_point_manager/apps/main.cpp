#include <rclcpp/rclcpp.hpp>

#include "nav_point_manager/nav_point_manager_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<nav_point_manager::NavPointManagerNode>();

    // 在构造完成后初始化 action 服务器
    node->initializeActionServers();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    RCLCPP_INFO(node->get_logger(), "Nav Point Manager Node started");

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
