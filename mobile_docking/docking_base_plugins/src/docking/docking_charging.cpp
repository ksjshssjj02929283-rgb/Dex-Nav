// DockingCharging 实现（移植 ROS1 docking_charging.cpp，DEC-011 Part2）
#include "docking_base_plugins/docking/docking_charging.hpp"

#include <cmath>
#include <memory>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace docking_base_plugins
{

using docking_base_core::DockingStatus;

bool DockingCharging::Init()
{
  using nav2_util::declare_parameter_if_not_declared;
  declare_parameter_if_not_declared(node_, name_ + ".charging_threshold", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node_, name_ + ".wait_perception_time", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(node_, name_ + ".drawback_timeout", rclcpp::ParameterValue(10.0));
  node_->get_parameter(name_ + ".charging_threshold", charging_threshold_);
  node_->get_parameter(name_ + ".wait_perception_time", wait_perception_time_);
  node_->get_parameter(name_ + ".drawback_timeout", drawback_timeout_);

  // 充电状态来自 command_board 的 /battery_info（BatteryInfo，源自 /diagnostics_agg PowerBoard）
  battery_sub_ = node_->create_subscription<nav_interfaces::msg::BatteryInfo>(
    "/battery_info", 1,
    std::bind(&DockingCharging::batteryCallback, this, std::placeholders::_1));

  curr_stage_ = ChargingStage::INIT;
  curr_controller_ = "pid";
  curr_goal_checker_ = "stopped";
  use_local_planner_ = true;
  linear_vel_limit_ = 0.1;
  curr_goal_.reset();
  drawback_timing_ = false;
  return true;
}

void DockingCharging::batteryCallback(const nav_interfaces::msg::BatteryInfo::SharedPtr msg)
{
  std::lock_guard<std::recursive_mutex> lock(data_mutex_);
  // 优先用 PowerBoard 充电标志；charging_threshold>0 时辅以电流阈值兜底（正电流=充电）
  is_charging_ = msg->charge_state || (charging_threshold_ > 0.0 && msg->current > charging_threshold_);
}

void DockingCharging::GenerateCheckPoint()
{
  // 仅 PERCEPTION/TO_ROTATION 阶段、且距桩 >=0.75m 时更新（近距感知不可靠，冻结防抖）
  if (curr_stage_ != ChargingStage::PERCEPTION && curr_stage_ != ChargingStage::TO_ROTATION) {
    return;
  }
  if (!robot_pose_) {return;}
  const double dist = std::hypot(
    robot_pose_->pose.position.x - target_pose_.pose.position.x,
    robot_pose_->pose.position.y - target_pose_.pose.position.y);
  if (dist < 0.75) {return;}

  auto make = [](double x, double yaw) {
      geometry_msgs::msg::PoseStamped p;
      p.pose.position.x = x;
      tf2::Quaternion q; q.setRPY(0, 0, yaw);
      p.pose.orientation = tf2::toMsg(q);
      return p;
    };
  rotation_start_point_ = make(-0.55, 0.0);
  drawback_point_ = make(-0.55, M_PI);
  docking_stop_point_ = make(-0.32, M_PI);
  more_drawback_stop_point_ = make(-0.26, M_PI);

  // dock 坐标系 → fixed_frame：T_fixed_dock 由 target_pose_ 构造，doTransform 各检查点
  geometry_msgs::msg::TransformStamped T;
  T.header = target_pose_.header;
  T.transform.translation.x = target_pose_.pose.position.x;
  T.transform.translation.y = target_pose_.pose.position.y;
  T.transform.translation.z = target_pose_.pose.position.z;
  T.transform.rotation = target_pose_.pose.orientation;
  tf2::doTransform(rotation_start_point_, rotation_start_point_, T);
  tf2::doTransform(drawback_point_, drawback_point_, T);
  tf2::doTransform(docking_stop_point_, docking_stop_point_, T);
  tf2::doTransform(more_drawback_stop_point_, more_drawback_stop_point_, T);
}

DockingStatus DockingCharging::UpdateTaskStage()
{
  std::lock_guard<std::recursive_mutex> lock(data_mutex_);
  curr_status_ = DockingStatus::EXECUTING;
  if (!target_info_updated_ || !robot_pose_) {return curr_status_;}

  if (is_charging_) {curr_stage_ = ChargingStage::SUCCESS;}

  if (curr_stage_ == ChargingStage::INIT) {
    prev_check_point_ = robot_pose_;
    perception_start_ = node_->now();
    curr_stage_ = ChargingStage::PERCEPTION;
  }

  if (curr_stage_ == ChargingStage::PERCEPTION) {
    prev_check_point_ = robot_pose_;
    curr_goal_.reset();
    if ((node_->now() - perception_start_).seconds() < wait_perception_time_) {
      return curr_status_;
    }
    curr_stage_ = ChargingStage::TO_ROTATION;
  }

  if (curr_stage_ == ChargingStage::TO_ROTATION) {
    curr_controller_ = "pid"; curr_goal_checker_ = "stopped";
    curr_goal_ = std::make_shared<geometry_msgs::msg::PoseStamped>(rotation_start_point_);
    need_enable_reverse_ = false; linear_vel_limit_ = 0.15; use_local_planner_ = true;
    if (is_current_goal_reached_) {
      is_current_goal_reached_ = false;
      if (isSamePose(*robot_pose_, rotation_start_point_, 0.1, 0.03)) {
        curr_stage_ = ChargingStage::ROTATING;
        prev_check_point_ = std::make_shared<geometry_msgs::msg::PoseStamped>(rotation_start_point_);
      } else {curr_stage_ = ChargingStage::FAILED;}
    }
  }

  if (curr_stage_ == ChargingStage::ROTATING) {
    curr_controller_ = "rotation"; curr_goal_checker_ = "rotation";
    curr_goal_ = std::make_shared<geometry_msgs::msg::PoseStamped>(drawback_point_);
    need_enable_reverse_ = false; linear_vel_limit_ = 0.1; use_local_planner_ = true;
    if (is_current_goal_reached_) {
      is_current_goal_reached_ = false;
      if (isSamePose(*robot_pose_, drawback_point_, 0.1, 0.03)) {
        curr_stage_ = ChargingStage::DRAWBACK;
        prev_check_point_ = std::make_shared<geometry_msgs::msg::PoseStamped>(drawback_point_);
      } else {curr_stage_ = ChargingStage::FAILED;}
    }
  }

  if (curr_stage_ == ChargingStage::DRAWBACK) {
    curr_controller_ = "straight"; curr_goal_checker_ = "longitudinal";
    curr_goal_ = std::make_shared<geometry_msgs::msg::PoseStamped>(docking_stop_point_);
    need_enable_reverse_ = true; linear_vel_limit_ = 0.03; use_local_planner_ = false;
    if (is_current_goal_reached_) {
      is_current_goal_reached_ = false;
      if (is_charging_) {
        curr_stage_ = ChargingStage::SUCCESS;
      } else {
        prev_check_point_ = std::make_shared<geometry_msgs::msg::PoseStamped>(drawback_point_);
        curr_stage_ = ChargingStage::MORE_DRAWBACK;
      }
    }
  }

  if (curr_stage_ == ChargingStage::MORE_DRAWBACK) {
    curr_controller_ = "straight"; curr_goal_checker_ = "longitudinal";
    curr_goal_ = std::make_shared<geometry_msgs::msg::PoseStamped>(more_drawback_stop_point_);
    need_enable_reverse_ = true; linear_vel_limit_ = 0.03; use_local_planner_ = false;
    if (is_current_goal_reached_ || is_charging_) {
      is_current_goal_reached_ = false;
      curr_stage_ = ChargingStage::SUCCESS;
    }
  }

  // DRAWBACK/MORE_DRAWBACK 阶段超时保护
  if (curr_stage_ == ChargingStage::DRAWBACK || curr_stage_ == ChargingStage::MORE_DRAWBACK) {
    if (!drawback_timing_) {drawback_start_ = node_->now(); drawback_timing_ = true;}
    if ((node_->now() - drawback_start_).seconds() > drawback_timeout_) {
      curr_stage_ = ChargingStage::FAILED;
    }
  } else {
    drawback_timing_ = false;
  }

  if (curr_stage_ == ChargingStage::SUCCESS) {curr_status_ = DockingStatus::SUCCESS;}
  else if (curr_stage_ == ChargingStage::FAILED) {curr_status_ = DockingStatus::FAILED;}
  return curr_status_;
}

}  // namespace docking_base_plugins

PLUGINLIB_EXPORT_CLASS(docking_base_plugins::DockingCharging, docking_base_core::DockingInterface)
