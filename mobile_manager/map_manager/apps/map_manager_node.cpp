#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

#include "map_manager_ros.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<map_manager::MapManagerROS>();

    // 使用 MultiThreadedExecutor 以支持 async service call 的响应处理
    // rclcpp::spin(node);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
