#include "navigation_adapter/navigation_adapter_node.hpp"

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<navigation_adapter::NavigationAdapterNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
    node.reset();

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return 0;
}
