#include <memory>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_graceful_controller/smooth_control_law.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);   // 仅用于初始化静态类型支持，不需要 spin

  /* 1. 构造控制律对象 —— 参数按你实际调参填写 */
  const double k_phi        = 3.0;
  const double k_delta      = 2.0;
  const double beta         = 0.4;
  const double lambda       = 2.0;
  const double slowdown_r   = 0.5;
  const double v_lin_min    = 0.02;
  const double v_lin_max    = 0.5;
  const double v_ang_max    = 1.0;

  auto control_law =
    std::make_unique<nav2_graceful_controller::SmoothControlLaw>(
      k_phi, k_delta, beta, lambda, slowdown_r,
      v_lin_min, v_lin_max, v_ang_max);

  /* 2. 人为给定一个目标位姿 */
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = 1.0;
  target_pose.position.y = 0.5;
  target_pose.position.z = 0.0;
  // 朝向 30°
  double yaw = 30.0 * M_PI / 180.0;
  target_pose.orientation.w = std::cos(yaw / 2);
  target_pose.orientation.x = 0;
  target_pose.orientation.y = 0;
  target_pose.orientation.z = std::sin(yaw / 2);

  /* 3. 计算速度 */
  geometry_msgs::msg::Twist cmd;
  bool backward = false;  // 想测倒车就改成 true
  cmd = control_law->calculateRegularVelocity(target_pose, backward);

  /* 4. 打印结果 */
  std::cout << "-------- SmoothControlLaw 手动测试 --------" << std::endl;
  std::cout << "给定 pose:  x=" << target_pose.position.x
            << "  y=" << target_pose.position.y
            << "  yaw=" << yaw << " rad" << std::endl;
  std::cout << "计算 twist: linear.x=" << cmd.linear.x
            << "  angular.z=" << cmd.angular.z << std::endl;

  rclcpp::shutdown();
  return 0;
}