#include <csignal>
#include <iostream>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mission_manager_ros.hpp"

// 全局执行器指针，用于信号处理
static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> g_executor = nullptr;

void signal_handler(int signal) {
    if (g_executor) {
        std::cout << "\nShutting down executor..." << std::endl;
        g_executor->cancel();
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto mission_manager_ros = std::make_shared<pnc::MissionManagerROS>();

    // 创建 MultiThreadedExecutor 支持 Reentrant 回调组
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 4);
    executor->add_node(mission_manager_ros);

    // 保存全局引用用于信号处理
    g_executor = executor;

    RCLCPP_INFO(mission_manager_ros->get_logger(), "Mission Manager started with MultiThreadedExecutor (4 threads)");

    // 设置信号处理
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // 运行执行器（在调用线程中）
    try {
        executor->spin();
    } catch (...) {
        RCLCPP_ERROR(mission_manager_ros->get_logger(), "Executor exception");
    }

    rclcpp::shutdown();

    return 0;
}
