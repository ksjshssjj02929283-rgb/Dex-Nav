#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>

namespace
{
constexpr char kFilteredOdomTopic[] = "/localization/odometry/filtered";
constexpr char kPoseTopic[] = "/feedback/chassis_pose";
constexpr char kTargetFrame[] = "map";
constexpr auto kLogThrottle = std::chrono::seconds(5);
constexpr auto kOdomTimeout = std::chrono::seconds(2);
constexpr auto kMonitorPeriod = std::chrono::seconds(1);

std::string remove_leading_slashes(const std::string &frame_id)
{
  const auto first_non_slash = frame_id.find_first_not_of('/');
  if (first_non_slash == std::string::npos) {
    return {};
  }
  return frame_id.substr(first_non_slash);
}
}  // namespace

class BaseLinkPosition : public rclcpp::Node
{
public:
  BaseLinkPosition()
: Node("baselink_position")
  {
    rclcpp::QoS output_qos(rclcpp::KeepLast(10));
    output_qos.reliable().durability_volatile();

    rclcpp::QoS input_qos(rclcpp::KeepLast(1));
    input_qos.reliable().durability_volatile();

    publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(kPoseTopic, output_qos);
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      kFilteredOdomTopic,
      input_qos,
      std::bind(&BaseLinkPosition::odom_callback, this, std::placeholders::_1));
    monitor_timer_ = create_wall_timer(
      kMonitorPeriod,
      std::bind(&BaseLinkPosition::monitor_odom, this));
    last_valid_odom_time_ = SteadyClock::now();

    RCLCPP_INFO(
      get_logger(),
      "Chassis pose publisher started; %s -> %s.",
      kFilteredOdomTopic,
      kPoseTopic);
  }

private:
  using SteadyClock = std::chrono::steady_clock;

  void log_problem(const std::string & message)
  {
    const auto now = SteadyClock::now();
    if (has_problem_log_time_ && now - last_problem_log_time_ < kLogThrottle) {
      return;
    }
    RCLCPP_WARN(get_logger(), "%s", message.c_str());
    last_problem_log_time_ = now;
    has_problem_log_time_ = true;
  }

  void monitor_odom()
  {
    const auto elapsed = SteadyClock::now() - last_valid_odom_time_;
    if (elapsed > kOdomTimeout) {
      const auto elapsed_seconds =
        std::chrono::duration<double>(elapsed).count();
      log_problem(
        "No valid odometry received for " + std::to_string(elapsed_seconds) + " seconds.");
      odom_timeout_active_ = true;
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const std::string frame_id = remove_leading_slashes(msg->header.frame_id);
    const std::string child_frame_id = remove_leading_slashes(msg->child_frame_id);
    if (frame_id != kTargetFrame ||
      (child_frame_id != "base_link" && child_frame_id != "base_footprint"))
    {
      ++rejected_odom_count_;
      log_problem(
        "Rejected odometry; expected map frame with base_link or base_footprint, got " +
        frame_id + "/" + child_frame_id + ". Rejected messages: " +
        std::to_string(rejected_odom_count_) + ".");
      return;
    }

    last_valid_odom_time_ = SteadyClock::now();
    if (!has_valid_odom_) {
      RCLCPP_INFO(get_logger(), "Received valid odometry.");
    } else if (odom_timeout_active_) {
      RCLCPP_INFO(get_logger(), "Odometry recovered.");
    }
    has_valid_odom_ = true;
    odom_timeout_active_ = false;

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = kTargetFrame;
    pose_msg.header.stamp = now();
    pose_msg.pose.position.x = msg->pose.pose.position.x;
    pose_msg.pose.position.y = msg->pose.pose.position.y;
    pose_msg.pose.position.z = 0.0;
    pose_msg.pose.orientation = msg->pose.pose.orientation;
    publisher_->publish(pose_msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;
  SteadyClock::time_point last_valid_odom_time_;
  SteadyClock::time_point last_problem_log_time_;
  bool has_problem_log_time_{false};
  bool has_valid_odom_{false};
  bool odom_timeout_active_{false};
  std::size_t rejected_odom_count_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<BaseLinkPosition>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
