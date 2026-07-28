// ----------------------------------------------------------------------------
// Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
//
// All rights reserved.
// ----------------------------------------------------------------------------

#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace w1_diff_gazebo
{

class SimCmdVelBridge : public rclcpp::Node
{
public:
  SimCmdVelBridge()
  : Node("sim_cmd_vel_bridge")
  {
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/control/cmd_vel_nav");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/control/cmd_vel");

    publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      output_topic, rclcpp::QoS(1));
    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      input_topic, rclcpp::QoS(1),
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {
        publisher_->publish(*message);
      });

    RCLCPP_INFO(
      get_logger(), "Bridging simulation velocity commands: %s -> %s",
      input_topic.c_str(), output_topic.c_str());
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

}  // namespace w1_diff_gazebo

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<w1_diff_gazebo::SimCmdVelBridge>());
  rclcpp::shutdown();
  return 0;
}
