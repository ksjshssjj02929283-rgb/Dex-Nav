// 对接局部规划抽象（ROS1 BaseLocalPlanner+BaseSpeedPlanner 合一）：
// 取 DockingInterface 的「段路径」（检查点间的粗路径，通常两点）→ 输出带逐点速度的
// nav_interfaces::RobotTrajectory（几何重采样 + 纵向速度剖面），交给 nav2_core::Controller
// 经 setTrajectory 消费（规划/控制分离）。
// docking 不做障碍避让（遇障由上层安全监测停车），故规划层只做几何成形 + 运动学速度剖面，
// 不读 costmap 绕障。
#ifndef DOCKING_BASE_CORE__DOCKING_PLANNER_HPP_
#define DOCKING_BASE_CORE__DOCKING_PLANNER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_interfaces/msg/robot_trajectory.hpp"
#include "tf2_ros/buffer.h"

namespace docking_base_core
{

/**
 * @class DockingPlanner
 * @brief 对接局部规划抽象基类。pluginlib 加载，DockingServer 按阶段指令包选 planner_id。
 *        输入段路径（coarse）→ 输出带逐点纵向速度的 RobotTrajectory（不避障）。
 */
class DockingPlanner
{
public:
  using Ptr = std::shared_ptr<DockingPlanner>;

  virtual ~DockingPlanner() = default;

  /**
   * @param parent 宿主 lifecycle node
   * @param name   该 planner 实例名（参数命名空间）
   * @param tf     TF buffer
   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf) = 0;

  virtual void activate() {}
  virtual void deactivate() {}
  virtual void cleanup() {}

  /**
   * @brief 段路径 → 带逐点纵向速度的轨迹（不避障）。
   * @param coarse_path 段路径（含起点检查点 → 当前目标，通常两点）
   * @param robot_pose  机器人当前位姿（与 coarse_path 同坐标系）
   * @param vel_limit   本阶段纵向速度上限（m/s，>0）
   * @param reverse     是否倒车（true → longitudinal_velocity 取负 + decision_label=REVERSE）
   * @return RobotTrajectory（points 逐点 pose + longitudinal_velocity + path_kappa + decision_label）；
   *         规划失败返回空 points 的轨迹
   */
  virtual nav_interfaces::msg::RobotTrajectory createTrajectory(
    const nav_msgs::msg::Path & coarse_path,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    double vel_limit, bool reverse) = 0;

  std::string getName() {return name_;}

protected:
  std::string name_;
};

}  // namespace docking_base_core

#endif  // DOCKING_BASE_CORE__DOCKING_PLANNER_HPP_
