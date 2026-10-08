#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "path_follower/path_follower.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<path_follower::PathFollowerNode>(rclcpp::NodeOptions());
    // LifecycleNode 不是 rclcpp::Node，spin 要拿 base interface。
    // 节点起来后停在 unconfigured，由 nav2_lifecycle_manager（或手动 ros2 lifecycle set）驱动。
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
