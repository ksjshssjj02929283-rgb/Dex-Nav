// P1 功能测试：直接实例化 AprilTagPerception，configure→start→发假 apriltag_target_pose→getDockPose。
// 验证感知组件 启停/取结果 链路（DEC-011 Part1）。
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "docking_base_plugins/perception/apriltag_perception.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("p1_perception_test");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  auto perc = std::make_shared<docking_base_plugins::AprilTagPerception>();
  perc->configure(node, "perc", tf_buffer);
  perc->start();

  // 假检测发布用独立普通 Node（LifecycleNode 的 publisher 未 activate 不发消息）
  auto pub_node = std::make_shared<rclcpp::Node>("p1_fake_detector");
  auto pub = pub_node->create_publisher<geometry_msgs::msg::PoseStamped>("apriltag_target_pose", 1);

  bool ok = false;
  geometry_msgs::msg::PoseStamped out;
  rclcpp::Rate rate(20.0);
  for (int i = 0; i < 60 && rclcpp::ok(); ++i) {
    geometry_msgs::msg::PoseStamped det;
    det.header.frame_id = "odom";
    det.header.stamp = node->now();
    det.pose.position.x = 2.0;
    det.pose.position.y = 0.5;
    det.pose.orientation.w = 1.0;
    pub->publish(det);
    rclcpp::spin_some(pub_node);
    rclcpp::spin_some(node->get_node_base_interface());
    rate.sleep();
    if (perc->getDockPose(out, "odom")) {ok = true; break;}
  }

  if (ok) {
    RCLCPP_INFO(node->get_logger(),
      "P1 PASS: getDockPose -> (%.2f, %.2f) frame=%s",
      out.pose.position.x, out.pose.position.y, out.header.frame_id.c_str());
  } else {
    RCLCPP_ERROR(node->get_logger(), "P1 FAIL: getDockPose 未取到位姿");
  }
  perc->stop();
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
