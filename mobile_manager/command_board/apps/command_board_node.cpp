
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
#include "navigation_facade.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Command Board ROS Node Started");

    // 显式指定 2 个工作线程（匹配 auto_mode.sh 的 taskset -c 0-1，避免默认 8 线程在高频订阅下产生惊群锁争抢）
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    auto node = std::make_shared<command_board::CommandBoardManagerROS>();
    executor.add_node(node);
    // 导航门面（节点名 navigation）与指令板同进程、独立节点（DEC-035：宿主 command_board）
    auto navigation = std::make_shared<command_board::NavigationFacade>();
    executor.add_node(navigation);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
