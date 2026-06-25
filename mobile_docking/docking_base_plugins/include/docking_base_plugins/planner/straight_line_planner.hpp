// 直线对接局部规划：段路径（粗，通常两点）→ 等距重采样（位置线性插值 + 朝向 slerp）
// + 梯形纵向速度剖面（0→巡航→0，末端停在目标；倒车取负 + decision_label=REVERSE）。
// 速度剖面已用 scripts/validate_velocity_profile.py 数值验证（accel≤a_max、末端速度=0）。
// 不读 costmap、不避障。
#ifndef DOCKING_BASE_PLUGINS__STRAIGHT_LINE_PLANNER_HPP_
#define DOCKING_BASE_PLUGINS__STRAIGHT_LINE_PLANNER_HPP_

#include <memory>
#include <string>

#include "docking_base_core/docking_planner.hpp"

namespace docking_base_plugins
{

class StraightLinePlanner : public docking_base_core::DockingPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf) override;

  nav_interfaces::msg::RobotTrajectory createTrajectory(
    const nav_msgs::msg::Path & coarse_path,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    double vel_limit, bool reverse) override;

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  double path_resolution_{0.05};  // 重采样步长（m）
  double max_accel_{0.2};         // 纵向加速度上限（m/s^2），梯形剖面用
};

}  // namespace docking_base_plugins

#endif  // DOCKING_BASE_PLUGINS__STRAIGHT_LINE_PLANNER_HPP_
