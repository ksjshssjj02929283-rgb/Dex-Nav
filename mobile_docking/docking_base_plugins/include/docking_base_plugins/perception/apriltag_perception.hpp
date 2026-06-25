// AprilTag 对接感知实现（DEC-011 Part1）。封装 mobile_perception 的
// start/stop_apriltag_detection 服务 + apriltag_target_pose 订阅，零改感知侧。
#ifndef DOCKING_BASE_PLUGINS__APRILTAG_PERCEPTION_HPP_
#define DOCKING_BASE_PLUGINS__APRILTAG_PERCEPTION_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "docking_base_core/perception_interface.hpp"

namespace docking_base_plugins
{

class AprilTagPerception : public docking_base_core::PerceptionInterface
{
public:
  AprilTagPerception() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf) override;
  void cleanup() override;
  void start() override;
  void stop() override;
  bool getDockPose(
    geometry_msgs::msg::PoseStamped & dock_pose, const std::string & target_frame) override;

protected:
  void detectionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr detection_sub_;

  std::mutex mutex_;
  geometry_msgs::msg::PoseStamped latest_;
  bool got_detection_{false};

  // 参数
  std::string detected_topic_;
  std::string start_service_;
  std::string stop_service_;
  double detection_timeout_{1.0};
};

}  // namespace docking_base_plugins

#endif  // DOCKING_BASE_PLUGINS__APRILTAG_PERCEPTION_HPP_
