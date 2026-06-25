// P2 功能测试：驱动 DockingCharging 7 阶段状态机走到 SUCCESS（DEC-011 Part2）。
// 模拟：每周期喂 robot/target；当有 curr_goal 则瞬移机器人到该 goal + SetCurrGoalReached；
// 进入倒车段(controller=straight)后发"充电中"电池 → SUCCESS。
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_interfaces/msg/battery_info.hpp"

#include "docking_base_plugins/docking/docking_charging.hpp"

using docking_base_core::DockingStatus;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("p2_docking_test");
  node->declare_parameter("task.wait_perception_time", 0.5);  // 短感知等待：留时间算检查点（生产为 3s）
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  auto task = std::make_shared<docking_base_plugins::DockingCharging>();
  task->configure(node, "task", tf_buffer);
  task->Init();

  auto pub_node = std::make_shared<rclcpp::Node>("p2_fake_battery");
  auto batt_pub = pub_node->create_publisher<nav_interfaces::msg::BatteryInfo>("/battery_info", 1);

  // dock 目标在 odom (2,0,0)
  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id = "odom";
  target.pose.position.x = 2.0; target.pose.orientation.w = 1.0;

  // 机器人起点 (0,0)
  geometry_msgs::msg::PoseStamped robot;
  robot.header.frame_id = "odom";
  robot.pose.orientation.w = 1.0;

  std::vector<std::string> seen;
  DockingStatus st = DockingStatus::UNKNOWN;
  rclcpp::Rate rate(20.0);

  for (int i = 0; i < 200 && rclcpp::ok(); ++i) {
    target.header.stamp = node->now();
    robot.header.stamp = node->now();
    task->UpdateRobotPose(robot);
    task->UpdateTargetPose(target);

    // 倒车段开始发充电
    nav_interfaces::msg::BatteryInfo b;
    b.charge_state = (task->GetCurrControllerName() == "straight");
    batt_pub->publish(b);
    rclcpp::spin_some(pub_node);
    rclcpp::spin_some(node->get_node_base_interface());

    st = task->UpdateTaskStage();

    // 记录阶段指令包（controller+checker）
    const std::string tag = task->GetCurrControllerName() + "/" + task->GetCurrGoalCheckerName();
    if (seen.empty() || seen.back() != tag) {seen.push_back(tag);}

    // 模拟到达：瞬移机器人到当前目标 + 置到位
    auto goal = task->GetCurrGoal();
    if (goal) {
      robot = *goal;
      robot.header.frame_id = "odom";
      task->SetCurrGoalReached();
    }

    if (st == DockingStatus::SUCCESS || st == DockingStatus::FAILED) {break;}
    rate.sleep();
  }

  std::string chain;
  for (auto & s : seen) {chain += s + " -> ";}
  RCLCPP_INFO(node->get_logger(), "阶段指令包链: %s", chain.c_str());
  if (st == DockingStatus::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "P2 PASS: DockingCharging 状态机走到 SUCCESS");
  } else {
    RCLCPP_ERROR(node->get_logger(), "P2 FAIL: 终态=%d", static_cast<int>(st));
  }
  rclcpp::shutdown();
  return st == DockingStatus::SUCCESS ? 0 : 1;
}
