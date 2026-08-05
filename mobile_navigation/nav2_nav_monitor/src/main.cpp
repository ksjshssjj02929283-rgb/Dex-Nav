// Copyright 2026 DEXE Robot Engineer

#include "rclcpp/rclcpp.hpp"
#include "nav2_nav_monitor/nav_monitor_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_nav_monitor::NavMonitorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
