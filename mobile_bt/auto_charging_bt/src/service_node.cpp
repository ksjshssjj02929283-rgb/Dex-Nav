#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "auto_charging_bt/auto_charging_bt.hpp"

#include <memory>

// 全局节点指针定义（与库中的声明匹配）
std::shared_ptr<AutoChargingNode> charging_node;

namespace {

// 执行一次完整的回充流程，返回 true 表示成功，false 表示失败
bool run_charging_flow(std::string& message) {
    // 注册行为树节点工厂
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<NavigateToCharging>("NavigateToCharging");
    factory.registerNodeType<Dock>("Dock");
    factory.registerNodeType<CheckCharging>("CheckCharging");
    factory.registerNodeType<CheckBatteryAboveThreshold>("CheckBatteryAboveThreshold");
    factory.registerNodeType<IsCharging>("IsCharging");
    factory.registerNodeType<IsConnectedToCharger>("IsConnectedToCharger");
    factory.registerNodeType<Undocking>("Undocking");
    factory.registerNodeType<CounterCheck>("CounterCheck");
    factory.registerNodeType<IncrementCounter>("IncrementCounter");
    factory.registerNodeType<AttemptLogger>("AttemptLogger");
    factory.registerNodeType<StartDetection>("StartDetection");
    factory.registerNodeType<StopDetection>("StopDetection");

    // 启动 AprilTag 检测服务
    // if (!charging_node->startDetection())
    // {
    //     message = "启动 AprilTag 检测服务失败";
    //     RCLCPP_ERROR(charging_node->get_logger(), "%s", message.c_str());
    //     return false;
    // }

    // 读取充电点配置
    std::string json_path = "/home/dexforce/w1/install/mobile_perception/lib/mobile_perception/charging_pose.json";
    geometry_msgs::msg::PoseStamped goal_pose;
    try {
        goal_pose = readChargingPose(json_path);
        RCLCPP_INFO(charging_node->get_logger(), "成功读取充电点配置");
    } catch (const std::exception& e) {
        message = std::string("读取充电点配置失败: ") + e.what();
        RCLCPP_ERROR(charging_node->get_logger(), "%s", message.c_str());
        charging_node->stopDetection();  // 确保停止检测服务
        return false;
    }

    // 初始化Blackboard
    auto blackboard = BT::Blackboard::create();
    blackboard->set("charging_node", charging_node);
    blackboard->set("retry_count", 0);
    blackboard->set("charging_pose", goal_pose);

    // 从XML文件加载行为树
    std::string xml_path = ament_index_cpp::get_package_share_directory("auto_charging_bt") + "/config/charging_bt.xml";
    BT::Tree tree;
    try {
        tree = factory.createTreeFromFile(xml_path, blackboard);
    } catch (const std::exception& e) {
        message = std::string("加载行为树失败: ") + e.what();
        RCLCPP_ERROR(charging_node->get_logger(), "%s", message.c_str());
        charging_node->stopDetection();  // 确保停止检测服务
        return false;
    }

    // 控制台日志记录器（可选）
    // BT::StdCoutLogger logger_cout(tree);
    RCLCPP_INFO(charging_node->get_logger(), "开始执行一键回充行为树（service 调用）...");

    // 主执行循环
    rclcpp::Rate rate(10);
    bool success = false;
    while (rclcpp::ok()) {
        BT::NodeStatus status = tree.tickRoot();
        if (status == BT::NodeStatus::SUCCESS) {
            RCLCPP_INFO(charging_node->get_logger(), "✅ 回充流程完成！");
            success = true;
            message = "auto_charging success";
            break;
        } else if (status == BT::NodeStatus::FAILURE) {
            RCLCPP_ERROR(charging_node->get_logger(), "❌ 回充流程失败！");
            success = false;
            message = "auto_charging failed";
            break;
        }
        rclcpp::spin_some(charging_node);
        rate.sleep();
    }

    // 确保停止
    // charging_node->backward(0.0, 0.01);
    for (int i = 0; i < 5; i++) charging_node->publishTwist(0.0, 0.0);

    // 无论成功或失败，都停止检测服务
    charging_node->stopDetection();

    return success;
}

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // 创建全局功能节点
    charging_node = std::make_shared<AutoChargingNode>();

    // 创建 service：/auto_charging 使用 Trigger
    auto node = std::make_shared<rclcpp::Node>("auto_charging_service_node");
    auto service = node->create_service<std_srvs::srv::Trigger>(
        "/auto_charging", [](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            std::string message;
            bool ok = run_charging_flow(message);
            response->success = ok;
            response->message = message;
        });

    RCLCPP_INFO(node->get_logger(), "auto_charging service ready on /auto_charging");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
