/**
 * main_loop_nocharging.cpp
 *
 * 无充电循环导航入口：
 *   - 按导航计划持续循环多个点位，坐标从当前地图资产读取
 *   - 检测到电量低于阈值后，导航到第一个点位并停止程序
 *   - 不触发充电流程
 */

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>

#include "loop_navigation_bt/logging.hpp"
#include "loop_navigation_bt/loop_navigation_bt.hpp"

template <typename T>
auto create_builder(rclcpp::Node::SharedPtr node) {
    return [node](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<T>(name, config, node);
    };
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("loop_nocharging_bt_node");

    LOOP_RCLCPP_INFO(node->get_logger(), "Starting loop navigation (no-charging) behavior tree node.");

    BT::BehaviorTreeFactory factory;

    factory.registerBuilder<loop_navigation_bt::LoopNavigateFourPoints>(
        "LoopNavigateFourPoints", create_builder<loop_navigation_bt::LoopNavigateFourPoints>(node));
    factory.registerBuilder<loop_navigation_bt::CheckBatteryAboveThreshold>(
        "CheckBatteryAboveThreshold", create_builder<loop_navigation_bt::CheckBatteryAboveThreshold>(node));
    factory.registerBuilder<loop_navigation_bt::CancelNavigation>(
        "CancelNavigation", create_builder<loop_navigation_bt::CancelNavigation>(node));
    factory.registerBuilder<loop_navigation_bt::NavigateToFirstPointAndStop>(
        "NavigateToFirstPointAndStop", create_builder<loop_navigation_bt::NavigateToFirstPointAndStop>(node));

    std::string pkg_share_dir;
    try {
        pkg_share_dir = ament_index_cpp::get_package_share_directory("loop_navigation_bt");
    } catch (const std::exception& e) {
        LOOP_RCLCPP_ERROR(node->get_logger(), "Package 'loop_navigation_bt' not found: %s", e.what());
        loop_navigation_bt::writeStartupFailureLog("程序安装不完整，未找到循环导航配置");
        rclcpp::shutdown();
        return 1;
    }

    std::string xml_file = pkg_share_dir + "/config/loop_nocharging_bt.xml";
    std::string points_file = pkg_share_dir + "/config/navigation_points.json";
    // 无回充模式也读取同一份 YAML；launch 参数可临时覆盖电量检查开关。
    const double tick_rate_hz = node->declare_parameter<double>("loop.tick_rate_hz", 10.0);
    const double low_battery_threshold = node->declare_parameter<double>("battery.low_threshold", 30.0);
    const bool check_battery =
        node->declare_parameter<bool>("battery.check_enabled_in_nocharging_mode", false);
    const double map_state_wait_timeout_sec =
        node->declare_parameter<double>("navigation.map_state_wait_timeout_sec", 5.0);

    if (tick_rate_hz <= 0.0 || low_battery_threshold < 0.0 || low_battery_threshold > 100.0 ||
        !std::isfinite(map_state_wait_timeout_sec) || map_state_wait_timeout_sec <= 0.0) {
        LOOP_RCLCPP_ERROR(node->get_logger(), "无回充循环参数非法：请检查 tick 频率和低电量阈值。");
        loop_navigation_bt::writeStartupFailureLog("启动参数错误，请联系技术人员");
        rclcpp::shutdown();
        return 1;
    }

    const auto map_context =
        loop_navigation_bt::waitForCurrentMapAssets(node, map_state_wait_timeout_sec);
    if (!map_context.valid()) {
        loop_navigation_bt::writeStartupFailureLog("没有取得有效的当前地图或地图资产文件");
        rclcpp::shutdown();
        return 1;
    }

    // blackboard 中既保存配置，也保存行为树通知主循环退出的状态标志。
    auto blackboard = BT::Blackboard::create();
    blackboard->set("navigation_points", points_file);
    blackboard->set("map_assets_file", map_context.assets_file);
    blackboard->set("map_epoch", map_context.epoch);
    blackboard->set("map_changed", false);
    blackboard->set("test_stop_reason", std::string());
    blackboard->set("test_failure_reason", std::string());
    blackboard->set("low_battery_shutdown", false);
    blackboard->set("nav_loop_complete", false);
    blackboard->set("check_battery", check_battery);
    blackboard->set("low_battery_threshold", low_battery_threshold);

    LOOP_RCLCPP_INFO(node->get_logger(), "Loading behavior tree from: %s", xml_file.c_str());
    LOOP_RCLCPP_INFO(node->get_logger(), "电量检查已%s。", check_battery ? "启用" : "禁用");

    BT::Tree tree;
    try {
        tree = factory.createTreeFromFile(xml_file, blackboard);
    } catch (const std::exception& e) {
        LOOP_RCLCPP_ERROR(node->get_logger(), "Failed to load behavior tree: %s", e.what());
        loop_navigation_bt::writeStartupFailureLog("行为树加载失败，请联系技术人员");
        rclcpp::shutdown();
        return 1;
    }

    // 不启用 StdCoutLogger，避免输出 Unix 秒格式的行为树状态变化日志。

    LOOP_RCLCPP_INFO(node->get_logger(), "Behavior tree started. Looping through navigation points...");

    rclcpp::Rate rate(tick_rate_hz);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        try {
            if (blackboard->get<bool>("map_changed")) {
                LOOP_RCLCPP_ERROR(node->get_logger(), "检测到地图切换，立即停止当前行为树。");
                tree.haltTree();
                rclcpp::spin_some(node);
                break;
            }
        } catch (...) {
        }

        // 低电量返回首点完成后，由行为树通过 blackboard 通知主循环退出。
        try {
            bool stop = blackboard->get<bool>("low_battery_shutdown");
            if (stop) {
                LOOP_RCLCPP_INFO(node->get_logger(), "低电量处理完成：机器人已返回第一个点，程序即将退出。");
                blackboard->set("test_stop_reason", std::string("低电量返回首点后停止"));
                break;
            }
        } catch (...) {
        }

        try {
            bool complete = blackboard->get<bool>("nav_loop_complete");
            if (complete) {
                LOOP_RCLCPP_INFO(node->get_logger(), "Navigation loop completed. Shutting down program.");
                break;
            }
        } catch (...) {
        }

        BT::NodeStatus status = tree.tickRoot();

        if (status == BT::NodeStatus::FAILURE) {
            LOOP_RCLCPP_ERROR(node->get_logger(), "Behavior tree returned FAILURE. Shutting down.");
            try {
                if (blackboard->get<std::string>("test_failure_reason").empty() &&
                    blackboard->get<std::string>("test_stop_reason").empty()) {
                    blackboard->set("test_failure_reason", std::string("行为树执行失败"));
                }
            } catch (...) {
            }
            break;
        }

        rate.sleep();
    }

    LOOP_RCLCPP_INFO(node->get_logger(), "Shutting down node.");
    rclcpp::shutdown();
    return 0;
}
