#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nlohmann/json.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>

#include "docking_controller.h"

class DockingControllerTester : public rclcpp::Node {
public:
    DockingControllerTester() : Node("docking_controller_tester") {
        // 参数：目标名称
        this->declare_parameter<std::string>("goal_name", "charging_pose");
        std::string goal_name = this->get_parameter("goal_name").as_string();

        // 读取配置文件
        std::string pkg_dir = ament_index_cpp::get_package_share_directory("auto_charging_bt");
        std::string cfg_path = pkg_dir + "/config/charging_pose.json";
        nlohmann::json cfg;
        try {
            std::ifstream f(cfg_path);
            if (!f.is_open()) throw std::runtime_error("无法打开配置文件: " + cfg_path);
            cfg = nlohmann::json::parse(f);
        } catch (const std::exception &e) {
            RCLCPP_FATAL(this->get_logger(), "读取配置失败: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        if (!cfg.contains(goal_name)) {
            RCLCPP_FATAL(this->get_logger(), "配置中未找到目标 %s", goal_name.c_str());
            rclcpp::shutdown();
            return;
        }

        auto &jp = cfg[goal_name];
        target_pose_.position.x = jp["position"]["x"].get<double>();
        target_pose_.position.y = jp["position"]["y"].get<double>();
        target_pose_.position.z = jp["position"]["z"].get<double>();
        target_pose_.orientation.x = jp["quaternion"]["x"].get<double>();
        target_pose_.orientation.y = jp["quaternion"]["y"].get<double>();
        target_pose_.orientation.z = jp["quaternion"]["z"].get<double>();
        target_pose_.orientation.w = jp["quaternion"]["w"].get<double>();

        // 设置控制器参数
        controller_.setLinearTolerance(0.02);
        controller_.setMinAngularSpeed(0.00);

        // 发布器
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("control/cmd_vel_nav", 10);

        // 订阅器
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "apriltag_poses", 30, std::bind(&DockingControllerTester::poseCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "DockingController 测试节点启动，目标: %s", goal_name.c_str());
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        if (msg->poses.empty()) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "未检测到标签");
            publishZero();
            return;
        }

        const geometry_msgs::msg::Pose &cur = msg->poses[0];
        auto [v, w] = controller_.getVelocity(cur, target_pose_, -1);

        if (controller_.atGoal(cur, target_pose_)) {
            publishZero();
            RCLCPP_INFO(this->get_logger(), "到达目标，停止");
            return;
        }

        geometry_msgs::msg::Twist tw;
        tw.linear.x = v;
        tw.angular.z = w;
        cmd_pub_->publish(tw);

        RCLCPP_INFO(this->get_logger(), "vel: %.3f  omega: %.3f", v, w);
    }

    void publishZero() {
        geometry_msgs::msg::Twist zero;
        cmd_pub_->publish(zero);
    }

    DockingController controller_;
    geometry_msgs::msg::Pose target_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DockingControllerTester>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
