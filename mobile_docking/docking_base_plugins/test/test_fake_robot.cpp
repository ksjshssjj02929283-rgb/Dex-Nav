// P3a 闭环验证用的 fake_robot：订阅 /cmd_vel 做运动学积分，广播 fixed→base tf；
// 启动数秒后发 dock_robot action goal，收到 result 后打印并 shutdown。
// 机器人模拟器：订 cmd_vel 积分→广播 odom/base_link tf；与 docking_server 一起 launch 跑端到端。
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "docking_base_msgs/action/dock_robot.hpp"

using namespace std::chrono_literals;
using DockRobot = docking_base_msgs::action::DockRobot;

class FakeRobot : public rclcpp::Node
{
public:
  FakeRobot()
  : rclcpp::Node("fake_robot")
  {
    fixed_frame_ = declare_parameter("fixed_frame", std::string("odom"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));
    x_ = declare_parameter("init_x", 0.0);
    y_ = declare_parameter("init_y", 0.0);
    yaw_ = declare_parameter("init_yaw", 0.0);
    double goal_delay = declare_parameter("goal_delay", 3.0);

    tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    // map→odom 恒等静态变换（贴近真机：定位提供 map；部分 controller 硬编码 map 系）
    static_br_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    geometry_msgs::msg::TransformStamped m2o;
    m2o.header.stamp = now();
    m2o.header.frame_id = "map";
    m2o.child_frame_id = fixed_frame_;
    m2o.transform.rotation.w = 1.0;
    static_br_->sendTransform(m2o);
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr m) {last_cmd_ = *m; last_cmd_time_ = now();});
    action_client_ = rclcpp_action::create_client<DockRobot>(this, "dock_robot");

    last_cmd_time_ = now();
    last_step_ = now();
    timer_ = create_wall_timer(20ms, [this] {step();});
    goal_timer_ = create_wall_timer(
      std::chrono::duration<double>(goal_delay), [this] {sendGoal();});
  }

private:
  void step()
  {
    auto t = now();
    double dt = (t - last_step_).seconds();
    last_step_ = t;
    if (dt <= 0.0 || dt > 0.5) {dt = 0.0;}

    double vx = last_cmd_.linear.x, wz = last_cmd_.angular.z;
    if ((t - last_cmd_time_).seconds() > 0.5) {vx = 0.0; wz = 0.0;}  // cmd 过期视为停
    x_ += vx * std::cos(yaw_) * dt;
    y_ += vx * std::sin(yaw_) * dt;
    yaw_ += wz * dt;

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = t;
    tf.header.frame_id = fixed_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf2::Quaternion q; q.setRPY(0, 0, yaw_);
    tf.transform.rotation = tf2::toMsg(q);
    tf_br_->sendTransform(tf);
  }

  void sendGoal()
  {
    goal_timer_->cancel();
    if (!action_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(), "❌ dock_robot action server 不可用");
      rclcpp::shutdown();
      return;
    }
    DockRobot::Goal goal;
    auto opts = rclcpp_action::Client<DockRobot>::SendGoalOptions();
    opts.result_callback = [this](const rclcpp_action::ClientGoalHandle<DockRobot>::WrappedResult & r) {
        const bool ok = (r.code == rclcpp_action::ResultCode::SUCCEEDED) && r.result->success;
        RCLCPP_INFO(get_logger(), "%s 对接结果 success=%d (机器人最终 x=%.3f y=%.3f yaw=%.3f)",
          ok ? "✅" : "❌", static_cast<int>(r.result->success), x_, y_, yaw_);
        rclcpp::shutdown();
      };
    RCLCPP_INFO(get_logger(), "▶ 发送 dock_robot goal（起点 x=%.3f y=%.3f）", x_, y_);
    action_client_->async_send_goal(goal, opts);
  }

  std::string fixed_frame_, base_frame_;
  double x_{0}, y_{0}, yaw_{0};
  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_cmd_time_, last_step_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_br_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp_action::Client<DockRobot>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_, goal_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeRobot>());
  rclcpp::shutdown();
  return 0;
}
