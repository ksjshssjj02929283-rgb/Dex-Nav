// DockingExit 实现（移植 ROS1 docking_exit.cpp，DEC-011 Part2）
#include "docking_base_plugins/docking/docking_exit.hpp"

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace docking_base_plugins
{

using docking_base_core::DockingStatus;

bool DockingExit::Init()
{
  using nav2_util::declare_parameter_if_not_declared;
  declare_parameter_if_not_declared(node_, name_ + ".exit_distance", rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(node_, name_ + ".exit_velocity", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".exit_distance", exit_distance_);
  node_->get_parameter(name_ + ".exit_velocity", exit_velocity_);
  curr_stage_ = ExitStage::INIT;
  curr_controller_ = "straight";
  curr_goal_checker_ = "longitudinal";
  use_local_planner_ = false;
  linear_vel_limit_ = exit_velocity_;
  return true;
}

void DockingExit::GenerateCheckPoint()
{
  if (!robot_pose_) {return;}
  // 出桩点 = 机器人 base 前方 exit_distance；用 robot_pose_ 作 base→fixed 变换
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = exit_distance_;
  p.pose.orientation.w = 1.0;
  geometry_msgs::msg::TransformStamped T;
  T.header = robot_pose_->header;
  T.transform.translation.x = robot_pose_->pose.position.x;
  T.transform.translation.y = robot_pose_->pose.position.y;
  T.transform.translation.z = robot_pose_->pose.position.z;
  T.transform.rotation = robot_pose_->pose.orientation;
  tf2::doTransform(p, exit_point_, T);
}

DockingStatus DockingExit::UpdateTaskStage()
{
  std::lock_guard<std::recursive_mutex> lock(data_mutex_);
  curr_status_ = DockingStatus::EXECUTING;
  if (!robot_pose_) {return curr_status_;}

  if (curr_stage_ == ExitStage::INIT) {
    prev_check_point_ = robot_pose_;
    GenerateCheckPoint();
    target_info_updated_ = true;
    curr_stage_ = ExitStage::TO_EXIT_POINT;
  }

  if (curr_stage_ == ExitStage::TO_EXIT_POINT) {
    curr_controller_ = "straight"; curr_goal_checker_ = "longitudinal";
    curr_goal_ = std::make_shared<geometry_msgs::msg::PoseStamped>(exit_point_);
    need_enable_reverse_ = false; linear_vel_limit_ = exit_velocity_; use_local_planner_ = false;
    if (is_current_goal_reached_) {
      is_current_goal_reached_ = false;
      curr_stage_ = isSamePose(*robot_pose_, exit_point_, 0.1, 0.03) ?
        ExitStage::SUCCESS : ExitStage::FAILED;
    }
  }

  if (curr_stage_ == ExitStage::SUCCESS) {curr_status_ = DockingStatus::SUCCESS;}
  else if (curr_stage_ == ExitStage::FAILED) {curr_status_ = DockingStatus::FAILED;}
  return curr_status_;
}

}  // namespace docking_base_plugins

PLUGINLIB_EXPORT_CLASS(docking_base_plugins::DockingExit, docking_base_core::DockingInterface)
