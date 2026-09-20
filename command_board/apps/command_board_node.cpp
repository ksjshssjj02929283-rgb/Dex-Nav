
// ----------------------------------------------------------------------------
//
// Copyright (c) 2021--2026 DexForce Technology Co., Ltd.
//
// All Rights Reserved.
//
//    Authors: Li Yao <liyao2@dexforce.top>
// ----------------------------------------------------------------------------

#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

#include "command_board_ros.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Command Board ROS Node Started");

    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<command_board::CommandBoardManagerROS>();
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
