// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// 严格对齐 urdf/w1_diff_sim.urdf 物理与运动学参数:
//   - 驱动轮关节: WHEEL_LEFT (xyz="0 0.2539 0.053"), WHEEL_RIGHT (xyz="0 -0.2539 0.053")
//   - 轮距 wheel_separation: 0.5078 m
//   - 轮径 wheel_diameter:   0.126 m (半径 r = 0.063 m, 轮宽 0.045 m)
//   - 最大车轮扭矩 max_wheel_torque: 30.0 N·m
//   - 最大车轮加速度 max_wheel_acceleration: 1.0 m/s^2
//   - 底盘主包络尺寸: 0.70 m (长) x 0.50 m (宽) x 0.14 m (高), 质量 60.0 kg, Izz = 3.70 kg·m^2
//   - 里程计更新率: 100 Hz (/feedback/odom, odom -> base_footprint)
//   - 关节状态更新率: 30 Hz (/joint_states: WHEEL_LEFT, WHEEL_RIGHT)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

namespace w1_diff_gazebo
{

class W1DiffKinematicEmulator : public rclcpp::Node
{
public:
  W1DiffKinematicEmulator()
  : Node("w1_diff_kinematic_emulator")
  {
    // 1. 严格按照 w1_diff_sim.urdf 声明默认物理与运动学参数
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    publish_map_odom_tf_ = declare_parameter<bool>("publish_map_odom_tf", true);

    wheel_separation_ = declare_parameter<double>("wheel_separation", 0.5078);
    wheel_diameter_ = declare_parameter<double>("wheel_diameter", 0.126);
    wheel_radius_ = 0.5 * wheel_diameter_;
    max_wheel_torque_ = declare_parameter<double>("max_wheel_torque", 30.0);
    max_wheel_acceleration_ = declare_parameter<double>("max_wheel_acceleration", 1.0);
    chassis_length_ = declare_parameter<double>("chassis_length", 0.70);
    chassis_width_ = declare_parameter<double>("chassis_width", 0.50);
    chassis_mass_ = declare_parameter<double>("chassis_mass", 60.0);

    update_rate_hz_ = declare_parameter<double>("update_rate_hz", 100.0);
    joint_state_rate_hz_ = declare_parameter<double>("joint_state_rate_hz", 30.0);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.5);

    x_ = declare_parameter<double>("initial_x", 0.0);
    y_ = declare_parameter<double>("initial_y", 0.0);
    theta_ = declare_parameter<double>("initial_theta", 0.0);

    // 2. 发布器与 TF 广播器
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/feedback/odom", rclcpp::QoS(10));
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(10));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    // 3. 订阅底盘速度指令 (对齐 URDF remapping: /control/cmd_vel 以及导航输出 /control/cmd_vel_nav)
    auto cmd_cb = [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      target_v_ = msg->linear.x;
      target_omega_ = msg->angular.z;
      last_cmd_time_ = now();
    };

    cmd_vel_sub_nav_ = create_subscription<geometry_msgs::msg::Twist>(
      "/control/cmd_vel_nav", rclcpp::QoS(10), cmd_cb);
    cmd_vel_sub_ctrl_ = create_subscription<geometry_msgs::msg::Twist>(
      "/control/cmd_vel", rclcpp::QoS(10), cmd_cb);
    cmd_vel_sub_std_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(10), cmd_cb);

    // 4. 订阅 /initialpose 重置底盘位姿与轮速状态
    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", rclcpp::QoS(10),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        tf2::Quaternion q(
          msg->pose.pose.orientation.x,
          msg->pose.pose.orientation.y,
          msg->pose.pose.orientation.z,
          msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        theta_ = yaw;

        current_v_ = 0.0;
        current_omega_ = 0.0;
        target_v_ = 0.0;
        target_omega_ = 0.0;
        v_left_ = 0.0;
        v_right_ = 0.0;
        wheel_left_pos_ = 0.0;
        wheel_right_pos_ = 0.0;

        RCLCPP_INFO(
          get_logger(),
          "🔄 [w1_diff_sim] 重置底盘位姿: x=%.3f, y=%.3f, theta=%.3f rad (%.1f deg)",
          x_, y_, theta_, theta_ * 180.0 / M_PI);
      });

    // 5. 广播 URDF 静态 TF 链 (map->odom, base_footprint->base_link->sensors)
    publishUrdfStaticTransforms();

    last_update_time_ = now();
    last_cmd_time_ = now();

    // 100Hz 差速轮系运动学递推定时器
    auto odom_period = std::chrono::duration<double>(1.0 / update_rate_hz_);
    odom_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(odom_period),
      std::bind(&W1DiffKinematicEmulator::stepKinematics, this));

    // 30Hz 车轮关节状态发布定时器 (WHEEL_LEFT, WHEEL_RIGHT)
    auto js_period = std::chrono::duration<double>(1.0 / joint_state_rate_hz_);
    joint_state_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(js_period),
      std::bind(&W1DiffKinematicEmulator::publishWheelJointStates, this));

    RCLCPP_INFO(
      get_logger(),
      "\n====================================================================\n"
      "  W1 双轮差速底盘 URDF 级运动学仿真节点已启动 (w1_diff_sim.urdf)\n"
      "  - 轮距 wheel_separation     : %.4f m (WHEEL_LEFT/RIGHT y = ±%.4f m)\n"
      "  - 轮径 wheel_diameter       : %.3f m (半径 r = %.3f m)\n"
      "  - 车轮最大加速度            : %.2f m/s^2 | 最大扭矩: %.1f N·m\n"
      "  - 底盘包络尺寸              : %.2f m x %.2f m (质量 %.1f kg)\n"
      "  - 里程计反馈                : /feedback/odom (%.0f Hz)\n"
      "  - 车轮关节反馈              : /joint_states  (%.0f Hz, WHEEL_LEFT/RIGHT)\n"
      "====================================================================",
      wheel_separation_, 0.5 * wheel_separation_,
      wheel_diameter_, wheel_radius_,
      max_wheel_acceleration_, max_wheel_torque_,
      chassis_length_, chassis_width_, chassis_mass_,
      update_rate_hz_, joint_state_rate_hz_);
  }

private:
  geometry_msgs::msg::TransformStamped makeStaticTf(
    const rclcpp::Time & stamp,
    const std::string & parent,
    const std::string & child,
    double x, double y, double z,
    double roll, double pitch, double yaw)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = parent;
    t.child_frame_id = child;
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = z;
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    return t;
  }

  void publishUrdfStaticTransforms()
  {
    auto stamp = now();
    std::vector<geometry_msgs::msg::TransformStamped> static_tfs;

    if (publish_map_odom_tf_) {
      static_tfs.push_back(makeStaticTf(stamp, map_frame_, odom_frame_, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    }
    // 严格对应 w1_diff_sim.urdf 中的固定关节
    // base_joint: base_footprint -> base_link (xyz="0 0 0.01")
    static_tfs.push_back(makeStaticTf(stamp, base_frame_, "base_link", 0.0, 0.0, 0.01, 0.0, 0.0, 0.0));
    // BASE_LINK_TO_CHASSIS_CONNECTOR: base_link -> chassis_base_link
    static_tfs.push_back(makeStaticTf(stamp, "base_link", "chassis_base_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    // laser_front_joint: base_link -> laser_front_link (xyz="0.174973 0.145982 0.233649", rpy="0 0 0.7854")
    static_tfs.push_back(makeStaticTf(stamp, "base_link", "laser_front_link", 0.174973, 0.145982, 0.233649, 0.0, 0.0, 0.7854));
    // laser_rear_joint: base_link -> laser_rear_link (xyz="-0.174973 -0.145982 0.233649", rpy="0 0 -2.3562")
    static_tfs.push_back(makeStaticTf(stamp, "base_link", "laser_rear_link", -0.174973, -0.145982, 0.233649, 0.0, 0.0, -2.3562));
    // imu_chassis_joint: base_link -> imu_chassis_link (xyz="0.273 0 0")
    static_tfs.push_back(makeStaticTf(stamp, "base_link", "imu_chassis_link", 0.273, 0.0, 0.0, 0.0, 0.0, 0.0));

    static_tf_broadcaster_->sendTransform(static_tfs);
  }

  void stepKinematics()
  {
    auto current_time = now();
    double dt = (current_time - last_update_time_).seconds();
    if (dt <= 0.0 || dt > 0.2) {
      dt = 1.0 / update_rate_hz_;
    }
    last_update_time_ = current_time;

    // 1. 指令看门狗超时检测
    if ((current_time - last_cmd_time_).seconds() > command_timeout_sec_) {
      target_v_ = 0.0;
      target_omega_ = 0.0;
    }

    // 2. 逆运动学：将车体目标指令 (target_v_, target_omega_) 分解为左右驱动轮目标线速度
    //    v_L = v - (omega * wheel_separation) / 2
    //    v_R = v + (omega * wheel_separation) / 2
    double v_left_target = target_v_ - 0.5 * target_omega_ * wheel_separation_;
    double v_right_target = target_v_ + 0.5 * target_omega_ * wheel_separation_;

    // 3. 施加 URDF <max_wheel_acceleration>1.0</max_wheel_acceleration> 车轮加速度限制
    //    (在转向和加减速时保持左右轮协调缩放，防止单轮饱和导致轨迹畸变)
    double dv_left = v_left_target - v_left_;
    double dv_right = v_right_target - v_right_;
    double max_dv = max_wheel_acceleration_ * dt;
    double max_req_dv = std::max(std::abs(dv_left), std::abs(dv_right));

    if (max_req_dv > max_dv && max_req_dv > 1e-6) {
      double scale = max_dv / max_req_dv;
      v_left_ += dv_left * scale;
      v_right_ += dv_right * scale;
    } else {
      v_left_ = v_left_target;
      v_right_ = v_right_target;
    }

    // 4. 积分左右车轮旋转角度 (rad)
    double omega_left = v_left_ / wheel_radius_;
    double omega_right = v_right_ / wheel_radius_;
    wheel_left_pos_ += omega_left * dt;
    wheel_right_pos_ += omega_right * dt;

    // 5. 正运动学：由左右车轮真实线速度恢复车体真实速度 (current_v_, current_omega_)
    current_v_ = 0.5 * (v_right_ + v_left_);
    current_omega_ = (v_right_ - v_left_) / wheel_separation_;

    // 6. SE(2) 二阶 Runge-Kutta (中点航向) 位姿递推
    double delta_theta = current_omega_ * dt;
    double mid_theta = theta_ + 0.5 * delta_theta;
    x_ += current_v_ * std::cos(mid_theta) * dt;
    y_ += current_v_ * std::sin(mid_theta) * dt;
    theta_ += delta_theta;
    while (theta_ > M_PI) theta_ -= 2.0 * M_PI;
    while (theta_ < -M_PI) theta_ += 2.0 * M_PI;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);

    // 7. 广播动态 TF: odom -> base_footprint
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf_msg);

    // 8. 发布里程计 /feedback/odom
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.pose.covariance[0] = 1e-3;
    odom.pose.covariance[7] = 1e-3;
    odom.pose.covariance[35] = 1e-3;

    odom.twist.twist.linear.x = current_v_;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = current_omega_;
    odom.twist.covariance[0] = 1e-3;
    odom.twist.covariance[35] = 1e-3;

    odom_pub_->publish(odom);
  }

  void publishWheelJointStates()
  {
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.name = {"WHEEL_LEFT", "WHEEL_RIGHT"};
    js.position = {wheel_left_pos_, wheel_right_pos_};
    js.velocity = {v_left_ / wheel_radius_, v_right_ / wheel_radius_};
    js.effort = {0.0, 0.0};
    joint_state_pub_->publish(js);
  }

  // 位姿与轮系状态变量
  double x_{0.0};
  double y_{0.0};
  double theta_{0.0};

  double current_v_{0.0};
  double current_omega_{0.0};
  double target_v_{0.0};
  double target_omega_{0.0};

  double v_left_{0.0};
  double v_right_{0.0};
  double wheel_left_pos_{0.0};
  double wheel_right_pos_{0.0};

  // w1_diff_sim.urdf 物理参数
  double wheel_separation_{0.5078};
  double wheel_diameter_{0.126};
  double wheel_radius_{0.063};
  double max_wheel_torque_{30.0};
  double max_wheel_acceleration_{1.0};
  double chassis_length_{0.70};
  double chassis_width_{0.50};
  double chassis_mass_{60.0};

  double update_rate_hz_{100.0};
  double joint_state_rate_hz_{30.0};
  double command_timeout_sec_{0.5};

  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_footprint"};
  std::string map_frame_{"map"};
  bool publish_map_odom_tf_{true};

  rclcpp::Time last_update_time_;
  rclcpp::Time last_cmd_time_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_nav_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_ctrl_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_std_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
  rclcpp::TimerBase::SharedPtr joint_state_timer_;
};

} // namespace w1_diff_gazebo

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<w1_diff_gazebo::W1DiffKinematicEmulator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
