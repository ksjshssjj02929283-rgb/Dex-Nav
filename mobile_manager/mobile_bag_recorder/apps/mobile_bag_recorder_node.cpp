// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "mobile_bag_recorder/mobile_bag_recorder_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<mobile_bag_recorder::MobileBagRecorderNode>();
  node->init();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
