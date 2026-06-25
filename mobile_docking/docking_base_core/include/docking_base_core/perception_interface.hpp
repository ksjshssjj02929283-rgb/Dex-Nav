// 对接感知抽象接口（与对接逻辑解耦，DEC-011 Part1）
// 对接逻辑（DockingInterface/Server）只调 start/stop/getDockPose，不关心感知实现（AprilTag/其它）。
#ifndef DOCKING_BASE_CORE__PERCEPTION_INTERFACE_HPP_
#define DOCKING_BASE_CORE__PERCEPTION_INTERFACE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"

namespace docking_base_core
{

/**
 * @class PerceptionInterface
 * @brief 对接感知抽象：启停感知 + 取最新 dock 位姿。实现（如 AprilTag）可 pluginlib 加载，
 *        按 dock_type 选不同感知源。与对接任务逻辑解耦。
 */
class PerceptionInterface
{
public:
  using Ptr = std::shared_ptr<PerceptionInterface>;

  virtual ~PerceptionInterface() = default;

  /**
   * @param parent 宿主 lifecycle node
   * @param name   该感知实例名（参数命名空间）
   * @param tf     TF buffer（位姿变换用）
   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf) = 0;

  /** @brief 释放资源 */
  virtual void cleanup() = 0;

  /** @brief 启动感知（如调用检测节点的 start 服务）。对接开始时调。 */
  virtual void start() = 0;

  /** @brief 停止感知。对接结束时调。 */
  virtual void stop() = 0;

  /**
   * @brief 取最新 dock 位姿，变换到 target_frame。
   * @param dock_pose [out] 结果位姿（target_frame 下）
   * @param target_frame 期望输出坐标系（如 odom/map）
   * @return 有新鲜检测且变换成功为 true；无检测/超时/变换失败为 false
   */
  virtual bool getDockPose(
    geometry_msgs::msg::PoseStamped & dock_pose, const std::string & target_frame) = 0;

  std::string getName() {return name_;}

protected:
  std::string name_;
};

}  // namespace docking_base_core

#endif  // DOCKING_BASE_CORE__PERCEPTION_INTERFACE_HPP_
