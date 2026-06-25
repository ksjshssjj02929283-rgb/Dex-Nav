// 充电对接任务（ROS1 DockingCharging 7 阶段移植，DEC-011 Part2）
#ifndef DOCKING_BASE_PLUGINS__DOCKING_CHARGING_HPP_
#define DOCKING_BASE_PLUGINS__DOCKING_CHARGING_HPP_

#include <memory>
#include <string>

#include "nav_interfaces/msg/battery_info.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "docking_base_core/docking_interface.hpp"

namespace docking_base_plugins
{

enum class ChargingStage
{
  FAILED = -1, INIT = 0, PERCEPTION = 1, TO_ROTATION = 2,
  ROTATING = 3, DRAWBACK = 4, MORE_DRAWBACK = 5, SUCCESS = 6
};

class DockingCharging : public docking_base_core::DockingInterface
{
public:
  DockingCharging() = default;
  bool Init() override;
  docking_base_core::DockingStatus UpdateTaskStage() override;

protected:
  void GenerateCheckPoint() override;
  void batteryCallback(const nav_interfaces::msg::BatteryInfo::SharedPtr msg);

  ChargingStage curr_stage_{ChargingStage::INIT};

  // 检查点（dock 坐标系下定义，再变换到 fixed_frame）
  geometry_msgs::msg::PoseStamped rotation_start_point_;   // 桩前 0.55m 待旋转点 (yaw=0)
  geometry_msgs::msg::PoseStamped drawback_point_;         // 同点旋转 180° (yaw=π)
  geometry_msgs::msg::PoseStamped docking_stop_point_;     // 倒车贴桩停止点 (-0.32, yaw=π)
  geometry_msgs::msg::PoseStamped more_drawback_stop_point_;  // 更深 (-0.26, yaw=π)

  rclcpp::Subscription<nav_interfaces::msg::BatteryInfo>::SharedPtr battery_sub_;
  bool is_charging_{false};

  rclcpp::Time perception_start_;
  rclcpp::Time drawback_start_;
  bool drawback_timing_{false};
  double wait_perception_time_{3.0};
  double drawback_timeout_{10.0};
  double charging_threshold_{0.5};
};

}  // namespace docking_base_plugins

#endif  // DOCKING_BASE_PLUGINS__DOCKING_CHARGING_HPP_
