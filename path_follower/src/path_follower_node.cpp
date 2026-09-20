#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "path_follower/path_follower.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<path_follower::PathFollowerNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
