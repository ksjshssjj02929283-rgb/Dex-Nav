// AprilTagPerception 实现。见 [[9. docking_base 内部架构重构设计]] Part1。
#include "docking_base_plugins/perception/apriltag_perception.hpp"

#include <memory>
#include <string>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace docking_base_plugins
{

void AprilTagPerception::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf)
{
  name_ = name;
  tf2_buffer_ = tf;
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error("AprilTagPerception: 无法 lock parent lifecycle node");
  }

  using nav2_util::declare_parameter_if_not_declared;
  declare_parameter_if_not_declared(node_, name + ".detected_topic", rclcpp::ParameterValue("apriltag_target_pose"));
  declare_parameter_if_not_declared(node_, name + ".start_service", rclcpp::ParameterValue("start_apriltag_detection"));
  declare_parameter_if_not_declared(node_, name + ".stop_service", rclcpp::ParameterValue("stop_apriltag_detection"));
  declare_parameter_if_not_declared(node_, name + ".detection_timeout", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name + ".detected_topic", detected_topic_);
  node_->get_parameter(name + ".start_service", start_service_);
  node_->get_parameter(name + ".stop_service", stop_service_);
  node_->get_parameter(name + ".detection_timeout", detection_timeout_);

  start_client_ = node_->create_client<std_srvs::srv::Trigger>(start_service_);
  stop_client_ = node_->create_client<std_srvs::srv::Trigger>(stop_service_);
  detection_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    detected_topic_, 1,
    std::bind(&AprilTagPerception::detectionCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "AprilTagPerception[%s] configured (topic=%s)",
    name.c_str(), detected_topic_.c_str());
}

void AprilTagPerception::cleanup()
{
  start_client_.reset();
  stop_client_.reset();
  detection_sub_.reset();
}

void AprilTagPerception::start()
{
  if (!start_client_->wait_for_service(std::chrono::milliseconds(300))) {
    RCLCPP_WARN(node_->get_logger(), "[%s] %s 服务不可用，跳过（仿真/无感知时正常）",
      name_.c_str(), start_service_.c_str());
    return;
  }
  start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  RCLCPP_INFO(node_->get_logger(), "[%s] 已请求启动 AprilTag 检测", name_.c_str());
}

void AprilTagPerception::stop()
{
  if (!stop_client_ || !stop_client_->wait_for_service(std::chrono::milliseconds(300))) {return;}
  stop_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  RCLCPP_INFO(node_->get_logger(), "[%s] 已请求停止 AprilTag 检测", name_.c_str());
}

void AprilTagPerception::detectionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_ = *msg;
  got_detection_ = true;
}

bool AprilTagPerception::getDockPose(
  geometry_msgs::msg::PoseStamped & dock_pose, const std::string & target_frame)
{
  geometry_msgs::msg::PoseStamped latest;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!got_detection_) {return false;}
    latest = latest_;
  }
  // 时效校验
  const rclcpp::Time stamp(latest.header.stamp);
  if ((node_->now() - stamp).seconds() > detection_timeout_) {
    return false;
  }
  // 变换到 target_frame（同 frame 直接返回）
  if (latest.header.frame_id == target_frame || target_frame.empty()) {
    dock_pose = latest;
    return true;
  }
  try {
    tf2_buffer_->transform(latest, dock_pose, target_frame, tf2::durationFromSec(0.2));
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "[%s] dock 位姿变换到 %s 失败: %s", name_.c_str(), target_frame.c_str(), ex.what());
    return false;
  }
}

}  // namespace docking_base_plugins

PLUGINLIB_EXPORT_CLASS(docking_base_plugins::AprilTagPerception, docking_base_core::PerceptionInterface)
