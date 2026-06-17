/***********************************************************
 *
 * @file: straight_controller.cpp
 * @brief: 直线跟踪 PID local planner（ROS2 nav2_core::Controller 插件）
 * @author: Yang Haodong, Guo Zhanyu, Wu Maojia, Guo Linfeng
 * @update: 2026-06
 * @version: 2.0
 *
 * Copyright (c) 2023, Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 * 移植自 dcm_navigation（ROS1 nav_core::BaseController）。状态机 / PID / 参数逻辑完整保留，
 * 仅替换 ROS1→ROS2 边界：getRobotPose→入参 pose、odom helper→入参 velocity、
 * dcm_msgs::Path→内部 StraightPathPoint 缓存（speed_limit / curvature / pose）。
 *
 **********************************************************/

#include "straight_controller/straight_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_util/node_utils.hpp"

namespace straight_controller
{

void StraightController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();

  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // ---- goal reached tolerance ----
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".long_precision", rclcpp::ParameterValue(0.02));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".lat_precision", rclcpp::ParameterValue(0.04));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".o_precision", rclcpp::ParameterValue(0.5));

  // ---- linear velocity ----
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_v", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_v", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_v_inc", rclcpp::ParameterValue(0.5));

  // ---- angular velocity ----
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_w", rclcpp::ParameterValue(1.57));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_w", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_w_inc", rclcpp::ParameterValue(1.57));

  // ---- 前进后退路径的最长距离 ----
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_path_length", rclcpp::ParameterValue(0.5));

  // ---- 嵌套 PID 配置（逐项 declare 为 <name>.<pid>.<field>）----
  auto declare_pid = [&](const std::string & pid, controller_common::PidConf & conf,
      bool integ_enable, double integ_sat, double kp, double ki, double kd) {
      nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + "." + pid + ".integrator_enable",
        rclcpp::ParameterValue(integ_enable));
      nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + "." + pid + ".integrator_saturation_level",
        rclcpp::ParameterValue(integ_sat));
      nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + "." + pid + ".kp", rclcpp::ParameterValue(kp));
      nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + "." + pid + ".ki", rclcpp::ParameterValue(ki));
      nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + "." + pid + ".kd", rclcpp::ParameterValue(kd));

      node->get_parameter(plugin_name_ + "." + pid + ".integrator_enable", conf.integrator_enable);
      node->get_parameter(
        plugin_name_ + "." + pid + ".integrator_saturation_level", conf.integrator_saturation_level);
      node->get_parameter(plugin_name_ + "." + pid + ".kp", conf.kp);
      node->get_parameter(plugin_name_ + "." + pid + ".ki", conf.ki);
      node->get_parameter(plugin_name_ + "." + pid + ".kd", conf.kd);
    };

  declare_pid("station_pid_conf", station_pid_conf_, true, 0.3, 1.00, 0.01, 0.10);
  declare_pid("high_speed_pid_conf", high_speed_pid_conf_, true, 0.3, 1.00, 0.01, 0.10);
  declare_pid("low_speed_pid_conf", low_speed_pid_conf_, true, 0.3, 1.00, 0.01, 0.10);
  declare_pid("reverse_station_pid_conf", reverse_station_pid_conf_, true, 0.5, 1.00, 0.01, 0.10);
  declare_pid("reverse_speed_pid_conf", reverse_speed_pid_conf_, true, 0.5, 1.00, 0.01, 0.10);

  // ---- 航向手工 PID 参数 ----
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".k_w_p", rclcpp::ParameterValue(1.00));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".k_w_i", rclcpp::ParameterValue(0.01));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".k_w_d", rclcpp::ParameterValue(0.10));

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".switch_speed", rclcpp::ParameterValue(0.1));

  // ---- 读取标量参数 ----
  node->get_parameter(plugin_name_ + ".long_precision", long_precision_);
  node->get_parameter(plugin_name_ + ".lat_precision", lat_precision_);
  node->get_parameter(plugin_name_ + ".o_precision", o_precision_);
  node->get_parameter(plugin_name_ + ".max_v", max_v_);
  node->get_parameter(plugin_name_ + ".min_v", min_v_);
  node->get_parameter(plugin_name_ + ".max_v_inc", max_v_inc_);
  node->get_parameter(plugin_name_ + ".max_w", max_w_);
  node->get_parameter(plugin_name_ + ".min_w", min_w_);
  node->get_parameter(plugin_name_ + ".max_w_inc", max_w_inc_);
  node->get_parameter(plugin_name_ + ".max_path_length", max_path_length_);
  node->get_parameter(plugin_name_ + ".k_w_p", k_w_p_);
  node->get_parameter(plugin_name_ + ".k_w_i", k_w_i_);
  node->get_parameter(plugin_name_ + ".k_w_d", k_w_d_);
  node->get_parameter(plugin_name_ + ".switch_speed", switch_speed_);

  // ---- 控制周期：取 controller_frequency（与 nav2 controller_server 对齐）----
  double controller_frequency = 20.0;
  if (!node->get_parameter("controller_frequency", controller_frequency)) {
    nav2_util::declare_parameter_if_not_declared(
      node, "controller_frequency", rclcpp::ParameterValue(20.0));
    node->get_parameter("controller_frequency", controller_frequency);
  }
  if (controller_frequency <= 0.0) {
    controller_frequency = 20.0;
  }
  d_t_ = 1.0 / controller_frequency;

  e_w_ = i_w_ = 0.0;

  target_pose_pub_ =
    node->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 10);
  desired_vel_pub_ =
    node->create_publisher<std_msgs::msg::Float64>("desired_linear_vel", 10);

  station_pid_controller_.Init(station_pid_conf_);
  speed_pid_controller_.Init(low_speed_pid_conf_);

  initialized_ = true;
  RCLCPP_INFO(logger_, "StraightController (PID) configured: %s", plugin_name_.c_str());
}

void StraightController::activate()
{
  RCLCPP_INFO(
    logger_, "Activating controller: %s of type straight_controller::StraightController",
    plugin_name_.c_str());
  target_pose_pub_->on_activate();
  desired_vel_pub_->on_activate();
}

void StraightController::deactivate()
{
  RCLCPP_INFO(
    logger_, "Deactivating controller: %s of type straight_controller::StraightController",
    plugin_name_.c_str());
  target_pose_pub_->on_deactivate();
  desired_vel_pub_->on_deactivate();
}

void StraightController::cleanup()
{
  RCLCPP_INFO(
    logger_, "Cleaning up controller: %s of type straight_controller::StraightController",
    plugin_name_.c_str());
  target_pose_pub_.reset();
  desired_vel_pub_.reset();
}

bool StraightController::cachePath(
  const std::vector<StraightPathPoint> & raw_points,
  double max_total_length, bool length_check)
{
  if (raw_points.empty()) {
    return false;
  }

  // 直线控制的路径不能过长，否则无法保证控制精度（保留 ROS1 约束）
  if (length_check && computePathLength(raw_points) > max_total_length) {
    RCLCPP_ERROR(
      logger_,
      "[StraightController] : Path length is too long, straight planner will not control well, "
      "please modify your logic");
    return false;
  }

  std::unique_lock<std::mutex> lock(plan_mutex_);
  controller_path_ = raw_points;

  // transform path into map frame（保留 ROS1：逐点转 map）
  for (auto & p : controller_path_) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = p.header;
    ps.pose = p.pose;
    try {
      tf_->transform(ps, ps, "map", tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(logger_, "[StraightController] : transform path to map failed: %s", ex.what());
      return false;
    }
    p.header = ps.header;
    p.pose = ps.pose;
  }

  // reset plan parameters（保留 ROS1：从第二个点起）
  closest_plan_index_ = std::min(1, static_cast<int>(controller_path_.size()) - 1);

  goal_x_ = controller_path_.back().pose.position.x;
  goal_y_ = controller_path_.back().pose.position.y;
  goal_yaw_ = tf2::getYaw(controller_path_.back().pose.orientation);
  goal_ps_.header = controller_path_.back().header;
  goal_ps_.pose = controller_path_.back().pose;

  e_w_ = i_w_ = 0.0;
  return true;
}

void StraightController::setPlan(const nav_msgs::msg::Path & path)
{
  if (!initialized_) {
    RCLCPP_ERROR(
      logger_, "This planner has not been configured, please call configure() before using it");
    return;
  }
  if (path.poses.empty()) {
    return;
  }

  // 几何路径：每点期望速度退化为 setSpeedLimit 上限（speed_limit_，0=不限速）
  std::vector<StraightPathPoint> raw;
  raw.reserve(path.poses.size());
  for (const auto & ps : path.poses) {
    StraightPathPoint p;
    p.header = ps.header;
    if (p.header.frame_id.empty()) {
      p.header.frame_id = path.header.frame_id;
    }
    p.pose = ps.pose;
    p.speed_limit = speed_limit_;
    p.curvature = 0.0;
    raw.push_back(p);
  }

  cachePath(raw, max_path_length_, true);
}

void StraightController::setTrajectory(const nav_interfaces::msg::RobotTrajectory & trajectory)
{
  if (!initialized_) {
    RCLCPP_ERROR(
      logger_, "This planner has not been configured, please call configure() before using it");
    return;
  }
  if (trajectory.points.empty()) {
    return;
  }

  // RobotTrajectory → 内部缓存：
  //   longitudinal_velocity → speed_limit（每点期望速度）
  //   path_kappa            → curvature
  //   pose                  → 几何
  std::vector<StraightPathPoint> raw;
  raw.reserve(trajectory.points.size());
  for (const auto & tp : trajectory.points) {
    StraightPathPoint p;
    p.header = tp.pose.header;
    if (p.header.frame_id.empty()) {
      p.header.frame_id = trajectory.header.frame_id;
    }
    p.pose = tp.pose.pose;

    double v = tp.longitudinal_velocity;
    // 与 setSpeedLimit 保存的上限取严（speed_limit_ > 0 时生效）
    if (speed_limit_ > 0.0) {
      double mag = std::min(std::fabs(v), speed_limit_);
      v = std::copysign(mag, v == 0.0 ? 1.0 : v);
      if (tp.longitudinal_velocity == 0.0) {
        v = 0.0;
      }
    }
    p.speed_limit = v;
    p.curvature = tp.path_kappa;
    raw.push_back(p);
  }

  cachePath(raw, max_path_length_, true);
}

bool StraightController::isGoalReached()
{
  // 与 ROS1 行为一致：始终返回 false（到达判定交由上层 goal_checker）
  return false;
}

geometry_msgs::msg::TwistStamped StraightController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.angular.z = 0.0;

  if (!initialized_) {
    RCLCPP_ERROR(logger_, "Straight controller has not been configured");
    return cmd_vel;
  }

  std::unique_lock<std::mutex> lock(plan_mutex_);

  if (controller_path_.empty()) {
    RCLCPP_WARN(logger_, "StraightController : no path set");
    return cmd_vel;
  }

  // current pose（替代 ROS1 getRobotPose；入参 pose 即 odom 系当前位姿）
  geometry_msgs::msg::PoseStamped current_ps_odom = pose;

  // 需要两个 pose：一个用于正常计算，一个用于倒车（将朝向旋转 180°）
  if (reverse_enabled_) {
    double reverse_yaw = tf2::getYaw(current_ps_odom.pose.orientation) + M_PI;
    regularizeAngle(reverse_yaw);
    tf2::Quaternion q;
    q.setRPY(0, 0, reverse_yaw);
    current_ps_odom.pose.orientation = tf2::toMsg(q);
  }

  pruneGlobalPlan(current_ps_odom, controller_path_, 0.5);

  if (controller_path_.empty()) {
    RCLCPP_WARN(logger_, "StraightController : path empty after pruning");
    return cmd_vel;
  }

  // PID 的轨迹在 map 坐标系下，把当前位姿转到 map
  geometry_msgs::msg::PoseStamped cur_for_tf = current_ps_odom;
  cur_for_tf.header.stamp = rclcpp::Time(0);
  try {
    tf_->transform(cur_for_tf, current_ps_, "map", tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(logger_, "StraightController : transform current pose to map failed: %s", ex.what());
    return cmd_vel;
  }

  // 当前速度（替代 ROS1 odom helper）
  double curr_long_vel = velocity.linear.x;
  double curr_ang_vel = velocity.angular.z;

  // 最近的点用来计算速度的 PID，目标点用来控制角度
  closest_plan_index_ = getClosestWaypointID(controller_path_, current_ps_, 0.1, true);
  if (closest_plan_index_ < 0) {
    RCLCPP_WARN(logger_, "StraightController : Failed to get closest waypoint");
    return cmd_vel;
  }

  desired_linear_vel_ = controller_path_[closest_plan_index_].speed_limit;
  if (reverse_enabled_) {
    desired_linear_vel_ = -desired_linear_vel_;
  }

  // 最近点为目标点（in map frame）
  closest_ps_.header = controller_path_.at(closest_plan_index_).header;
  closest_ps_.pose = controller_path_.at(closest_plan_index_).pose;

  double err_x, err_y;  // desired x, y in base frame
  double err_theta;

  // transform from map into base_frame
  geometry_msgs::msg::PoseStamped closest_in_base;
  closest_ps_.header.stamp = rclcpp::Time(0);
  try {
    tf_->transform(
      closest_ps_, closest_in_base, costmap_ros_->getBaseFrameID(), tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      logger_, "StraightController : transform closest pose to base failed: %s", ex.what());
    return cmd_vel;
  }
  // 如果是倒车，target_in_base.x 是负的，所以计算的速度也是负的
  err_x = closest_in_base.pose.position.x;
  err_y = closest_in_base.pose.position.y;
  (void)err_y;

  // current angle
  double theta_rob = tf2::getYaw(current_ps_.pose.orientation);  // [-pi, pi]

  if (reverse_enabled_) {
    theta_rob -= M_PI;
    regularizeAngle(theta_rob);
  }

  err_theta = std::fmod(goal_yaw_ - theta_rob + M_PI, 2 * M_PI) - M_PI;
  regularizeAngle(err_theta);

  cmd_vel.twist.linear.x = LinearSpeedControl(curr_long_vel, err_x);
  cmd_vel.twist.angular.z = AngularSpeedControl(curr_ang_vel, err_theta);

  // publish goal pose + desired velocity
  if (target_pose_pub_->is_activated()) {
    target_pose_pub_->publish(goal_ps_);
  }
  publishDesiredVelocity();
  return cmd_vel;
}

void StraightController::publishDesiredVelocity()
{
  if (desired_vel_pub_ && desired_vel_pub_->is_activated()) {
    std_msgs::msg::Float64 desired_vel;
    desired_vel.data = desired_linear_vel_;
    desired_vel_pub_->publish(desired_vel);
  }
}

double StraightController::LinearSpeedControl(double curr_long_vel, double e_dist)
{
  double curr_vel = curr_long_vel;

  if (reverse_enabled_) {
    station_pid_controller_.SetPID(reverse_station_pid_conf_);
    speed_pid_controller_.SetPID(reverse_speed_pid_conf_);
  } else {
    station_pid_controller_.SetPID(station_pid_conf_);
    if (curr_vel < switch_speed_) {
      speed_pid_controller_.SetPID(low_speed_pid_conf_);
    } else {
      speed_pid_controller_.SetPID(high_speed_pid_conf_);
    }
  }

  double speed_error = desired_linear_vel_ - curr_vel;

  double speed_offset = station_pid_controller_.Control(e_dist, d_t_);

  double speed_controller_input = speed_offset + speed_error;

  // Step: 计算加速度（速度是加速度的积分，所以得到的是加速度误差）
  double acceleration_cmd_closeloop = speed_pid_controller_.Control(speed_controller_input, d_t_);
  double acceleration_cmd_closeloop_limited =
    std::clamp(acceleration_cmd_closeloop, -max_v_inc_, max_v_inc_);

  double v_output = curr_vel + acceleration_cmd_closeloop_limited * d_t_;

  if (!std::isfinite(v_output)) {
    v_output = 0.0;
  }
  if (!reverse_enabled_) {
    v_output = std::clamp(v_output, min_v_, desired_linear_vel_ * 1.5);
  } else {
    v_output = std::clamp(v_output, desired_linear_vel_ * 1.5, -min_v_);
  }

  return v_output;
}

double StraightController::AngularSpeedControl(double curr_ang_vel, double err_theta)
{
  regularizeAngle(err_theta);

  double w_d = err_theta / d_t_;
  if (std::fabs(w_d) > max_w_) {
    w_d = std::copysign(max_w_, w_d);
  }

  double w = curr_ang_vel;
  double e_w = w_d - w;
  i_w_ += e_w * d_t_;
  double d_w = (e_w - e_w_) / d_t_;
  e_w_ = e_w;

  double w_inc = k_w_p_ * e_w + k_w_i_ * i_w_ + k_w_d_ * d_w;

  if (std::fabs(w_inc) > max_w_inc_) {
    w_inc = std::copysign(max_w_inc_, w_inc);
  }

  double w_cmd = w + w_inc;
  if (std::fabs(w_cmd) > max_w_) {
    w_cmd = std::copysign(max_w_, w_cmd);
  } else if (std::fabs(w_cmd) < min_w_) {
    w_cmd = std::copysign(min_w_, w_cmd);
  }

  double err_theta_abs = std::fabs(err_theta);
  if (err_theta_abs < M_PI / 8.0) {
    w_cmd *= 0.25;
  } else if (err_theta_abs < M_PI_4) {
    double factor = 1 - (M_PI_4 - err_theta_abs) / (M_PI_4 - M_PI / 8.0) * 0.25;
    w_cmd *= factor;
  }

  return w_cmd;
}

double StraightController::computePathLength(const std::vector<StraightPathPoint> & path) const
{
  double length = 0.0;
  for (int i = 1; i < static_cast<int>(path.size()); ++i) {
    double dx = path.at(i).pose.position.x - path.at(i - 1).pose.position.x;
    double dy = path.at(i).pose.position.y - path.at(i - 1).pose.position.y;
    length += std::sqrt(dx * dx + dy * dy);
  }
  return length;
}

int StraightController::getClosestWaypointID(
  const std::vector<StraightPathPoint> & path,
  const geometry_msgs::msg::PoseStamped & pose,
  double ignore_dist,
  bool ignore_first) const
{
  if (path.size() < 2) {  // 至少两个点，start 和 goal
    return -1;
  }

  int closest_id = -1;

  if (ignore_dist < 0.0) {
    ignore_dist = 0.0;
  }

  // 如果距离小于 ignore_dist，直接返回最后一个点的 id
  double dgx = path.back().pose.position.x - pose.pose.position.x;
  double dgy = path.back().pose.position.y - pose.pose.position.y;
  double dist_to_goal = std::hypot(dgx, dgy);
  if (dist_to_goal < ignore_dist) {
    return static_cast<int>(path.size()) - 1;
  }

  double min_dist = std::numeric_limits<double>::max();
  for (int i = 0; i < static_cast<int>(path.size()); ++i) {
    double dx = path.at(i).pose.position.x - pose.pose.position.x;
    double dy = path.at(i).pose.position.y - pose.pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < min_dist) {
      min_dist = dist;
      closest_id = i;
    }
  }

  if (ignore_first) {
    closest_id += 1;
    if (closest_id >= static_cast<int>(path.size())) {
      closest_id = static_cast<int>(path.size()) - 1;
    }
  }

  return closest_id;
}

bool StraightController::pruneGlobalPlan(
  const geometry_msgs::msg::PoseStamped & global_pose,
  std::vector<StraightPathPoint> & global_path,
  double dist_behind_robot)
{
  if (global_path.empty()) {
    return true;
  }

  try {
    // transform robot pose into the plan frame
    geometry_msgs::msg::TransformStamped global_to_plan_transform = tf_->lookupTransform(
      global_path.front().header.frame_id,
      global_pose.header.frame_id,
      tf2::TimePointZero);

    geometry_msgs::msg::PoseStamped robot;
    tf2::doTransform(global_pose, robot, global_to_plan_transform);

    double dist_thresh_sq = dist_behind_robot * dist_behind_robot;

    // iterate plan until a pose close to the robot is found
    auto it = global_path.begin();
    auto erase_end = it;
    bool found = false;
    while (it != global_path.end()) {
      double dx = robot.pose.position.x - it->pose.position.x;
      double dy = robot.pose.position.y - it->pose.position.y;
      double dist_sq = dx * dx + dy * dy;
      if (dist_sq < dist_thresh_sq) {
        erase_end = it;
        found = true;
        break;
      }
      ++it;
    }
    if (!found) {
      return false;
    }

    if (erase_end != global_path.begin()) {
      global_path.erase(global_path.begin(), erase_end);
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG(logger_, "Cannot prune path since no transform is available: %s", ex.what());
    return false;
  }
  return true;
}

void StraightController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  // 保存上限（绝对值 m/s 或相对 max_v_ 的百分比）；与每点轨迹速度取严。
  if (speed_limit == 0.0) {
    speed_limit_ = 0.0;  // 0 = 不限速（系统默认）
    return;
  }
  if (percentage) {
    speed_limit_ = std::fabs(max_v_ * speed_limit / 100.0);
  } else {
    speed_limit_ = std::fabs(speed_limit);
  }
}

}  // namespace straight_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(straight_controller::StraightController, nav2_core::Controller)
