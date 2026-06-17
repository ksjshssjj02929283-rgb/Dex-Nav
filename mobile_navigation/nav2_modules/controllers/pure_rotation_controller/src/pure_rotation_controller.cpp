/***********************************************************
 *
 * @file: pure_rotation_controller.cpp
 * @brief: 原地旋转控制器（纵向 station/speed PID + 独立角速度 PID 并行）。
 *         ROS1 nav_core::BaseController → ROS2 nav2_core::Controller 移植。
 * @author: Yang Haodong, Guo Zhanyu, Wu Maojia （原作者）
 *
 **********************************************************/

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"

#include "pure_rotation_controller/pure_rotation_controller.hpp"

namespace pure_rotation_controller
{

void PureRotationController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();

  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // ---- 标量参数 ----
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".long_precision", rclcpp::ParameterValue(0.02));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".lat_precision", rclcpp::ParameterValue(0.04));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".o_precision", rclcpp::ParameterValue(0.02));

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_v", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_v", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_v_inc", rclcpp::ParameterValue(0.5));

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_w", rclcpp::ParameterValue(1.57));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_w", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_w_inc", rclcpp::ParameterValue(1.57));

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".enable_lon_adjust", rclcpp::ParameterValue(false));

  node->get_parameter(plugin_name_ + ".long_precision", long_precision_);
  node->get_parameter(plugin_name_ + ".lat_precision", lat_precision_);
  node->get_parameter(plugin_name_ + ".o_precision", o_precision_);
  node->get_parameter(plugin_name_ + ".max_v", max_v_);
  node->get_parameter(plugin_name_ + ".min_v", min_v_);
  node->get_parameter(plugin_name_ + ".max_v_inc", max_v_inc_);
  node->get_parameter(plugin_name_ + ".max_w", max_w_);
  node->get_parameter(plugin_name_ + ".min_w", min_w_);
  node->get_parameter(plugin_name_ + ".max_w_inc", max_w_inc_);
  node->get_parameter(plugin_name_ + ".enable_lon_adjust", enable_lon_adjust_);

  // ---- 嵌套 PID 配置（station / speed / angular），逐项 declare 为 name.<pid>.<field> ----
  auto declare_pid = [&](const std::string & ns, controller_common::PidConf & conf,
      bool integrator_enable_def, double saturation_def) {
      nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + "." + ns + ".integrator_enable",
        rclcpp::ParameterValue(integrator_enable_def));
      nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + "." + ns + ".integrator_saturation_level",
        rclcpp::ParameterValue(saturation_def));
      nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + "." + ns + ".kp", rclcpp::ParameterValue(1.00));
      nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + "." + ns + ".ki", rclcpp::ParameterValue(0.01));
      nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + "." + ns + ".kd", rclcpp::ParameterValue(0.10));
      nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + "." + ns + ".kaw", rclcpp::ParameterValue(0.0));
      nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + "." + ns + ".saturation", rclcpp::ParameterValue(0.0));

      node->get_parameter(plugin_name_ + "." + ns + ".integrator_enable", conf.integrator_enable);
      node->get_parameter(
        plugin_name_ + "." + ns + ".integrator_saturation_level",
        conf.integrator_saturation_level);
      node->get_parameter(plugin_name_ + "." + ns + ".kp", conf.kp);
      node->get_parameter(plugin_name_ + "." + ns + ".ki", conf.ki);
      node->get_parameter(plugin_name_ + "." + ns + ".kd", conf.kd);
      node->get_parameter(plugin_name_ + "." + ns + ".kaw", conf.kaw);
      node->get_parameter(
        plugin_name_ + "." + ns + ".saturation", conf.output_saturation_level);
    };

  // station_pid_conf：默认 integrator_enable=true, saturation_level=0.3
  declare_pid("station_pid_conf", station_pid_conf_, true, 0.3);
  // speed_pid_conf：默认 integrator_enable=true, saturation_level=0.3
  declare_pid("speed_pid_conf", speed_pid_conf_, true, 0.3);
  // angular_pid_conf：默认 integrator_enable=true, saturation_level=0.5
  declare_pid("angular_pid_conf", angular_pid_conf_, true, 0.5);

  // 控制周期：从全局 controller_frequency 读取（对应 dcm 的 /move_base/controller_frequency）
  double controller_frequency = 20.0;
  if (!node->get_parameter("controller_frequency", controller_frequency) ||
    controller_frequency <= 0.0)
  {
    controller_frequency = 20.0;
  }
  d_t_ = 1.0 / controller_frequency;

  station_pid_controller_.Init(station_pid_conf_);
  speed_pid_controller_.Init(speed_pid_conf_);
  angular_pid_controller_.Init(angular_pid_conf_);

  initialized_ = true;
  RCLCPP_INFO(logger_, "PureRotationController configured (enable_lon_adjust=%d).",
    static_cast<int>(enable_lon_adjust_));
}

void PureRotationController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up PureRotationController: %s", plugin_name_.c_str());
  controller_path_ = nav_msgs::msg::Path();
  point_speed_limits_.clear();
  point_kappas_.clear();
  initialized_ = false;
}

void PureRotationController::activate()
{
  RCLCPP_INFO(logger_, "Activating PureRotationController: %s", plugin_name_.c_str());
}

void PureRotationController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating PureRotationController: %s", plugin_name_.c_str());
}

void PureRotationController::setPlan(const nav_msgs::msg::Path & path)
{
  if (!initialized_) {
    RCLCPP_ERROR(
      logger_, "This planner has not been configured, please call configure() before using it");
    return;
  }
  if (path.poses.empty()) {
    return;
  }

  std::unique_lock<std::mutex> lock(plan_mutex_);

  controller_path_ = path;
  // 几何路径无速度信息：每点期望速度默认 0，曲率默认 0
  point_speed_limits_.assign(path.poses.size(), 0.0);
  point_kappas_.assign(path.poses.size(), 0.0);

  plan_index_ = std::min(1, static_cast<int>(controller_path_.poses.size()) - 1);

  const double new_goal_x = controller_path_.poses.back().pose.position.x;
  const double new_goal_y = controller_path_.poses.back().pose.position.y;
  const double new_goal_yaw = tf2::getYaw(controller_path_.poses.back().pose.orientation);

  if (goal_x_ != new_goal_x || goal_y_ != new_goal_y || goal_yaw_ != new_goal_yaw) {
    goal_x_ = new_goal_x;
    goal_y_ = new_goal_y;
    goal_yaw_ = new_goal_yaw;

    theta_errors_.clear();
    longitude_errors_.clear();
  }
}

void PureRotationController::setTrajectory(const nav_interfaces::msg::RobotTrajectory & trajectory)
{
  if (!initialized_) {
    RCLCPP_ERROR(
      logger_, "This planner has not been configured, please call configure() before using it");
    return;
  }
  if (trajectory.points.empty()) {
    return;
  }

  std::unique_lock<std::mutex> lock(plan_mutex_);

  // 把带速度时空轨迹拆为：几何 nav_msgs::msg::Path + 并行的 per-point 速度 / 曲率向量。
  // 字段映射：TrajectoryPoint.longitudinal_velocity → 每点期望速度（dcm PathPose.speed_limit）；
  //          TrajectoryPoint.path_kappa             → 每点曲率（dcm curvature）；
  //          TrajectoryPoint.pose                   → 几何（dcm pose）。
  controller_path_ = nav_msgs::msg::Path();
  controller_path_.header = trajectory.header;
  controller_path_.poses.reserve(trajectory.points.size());
  point_speed_limits_.clear();
  point_speed_limits_.reserve(trajectory.points.size());
  point_kappas_.clear();
  point_kappas_.reserve(trajectory.points.size());

  for (const auto & pt : trajectory.points) {
    geometry_msgs::msg::PoseStamped ps = pt.pose;
    // 轨迹在 map 系下消费：若点位姿不在 map 系，转换到 map。
    if (!ps.header.frame_id.empty() && ps.header.frame_id != "map") {
      geometry_msgs::msg::PoseStamped ps_map;
      if (nav2_util::transformPoseInTargetFrame(ps, ps_map, *tf_, "map")) {
        ps = ps_map;
      }
    }
    controller_path_.poses.push_back(ps);

    // longitudinal_velocity 为该点沿切向期望速度（可为负=倒车）；0 时退回 speed_hint。
    double v = pt.longitudinal_velocity;
    if (v == 0.0 && pt.speed_hint != 0.0f) {
      v = static_cast<double>(pt.speed_hint);
    }
    point_speed_limits_.push_back(v);
    point_kappas_.push_back(pt.path_kappa);
  }

  plan_index_ = std::min(1, static_cast<int>(controller_path_.poses.size()) - 1);

  const double new_goal_x = controller_path_.poses.back().pose.position.x;
  const double new_goal_y = controller_path_.poses.back().pose.position.y;
  const double new_goal_yaw = tf2::getYaw(controller_path_.poses.back().pose.orientation);

  if (goal_x_ != new_goal_x || goal_y_ != new_goal_y || goal_yaw_ != new_goal_yaw) {
    goal_x_ = new_goal_x;
    goal_y_ = new_goal_y;
    goal_yaw_ = new_goal_yaw;

    theta_errors_.clear();
    longitude_errors_.clear();
  }
}

bool PureRotationController::isGoalReached()
{
  // 保留 dcm 行为：恒返回 false，到达由上层状态机判定。
  return false;
}

geometry_msgs::msg::TwistStamped PureRotationController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = clock_ ? clock_->now() : rclcpp::Clock().now();
  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.angular.z = 0.0;

  if (!initialized_) {
    RCLCPP_ERROR(logger_, "PureRotationController has not been configured");
    return cmd_vel;
  }

  std::unique_lock<std::mutex> lock(plan_mutex_);

  if (controller_path_.poses.empty()) {
    RCLCPP_WARN(logger_, "PureRotationController : empty plan");
    return cmd_vel;
  }

  // 当前位姿：用入参 pose 替代 dcm 内部 getRobotPose。约定 pose 已在全局（map）系下。
  current_ps_ = pose;

  // 当前速度：用入参 velocity 替代 dcm 的 odom helper。
  const double curr_lin_vel = velocity.linear.x;

  plan_index_ = getClosestWaypointID(controller_path_, current_ps_, 0.1, true);
  if (plan_index_ < 0) {
    RCLCPP_WARN(logger_, "PureRotationController : Failed to get closest waypoint");
    return cmd_vel;
  }

  desired_linear_vel_ = point_speed_limits_[plan_index_];
  // setSpeedLimit 保存的上限与轨迹速度取严（保符号）。
  if (std::isfinite(speed_limit_)) {
    const double sign = (desired_linear_vel_ < 0.0) ? -1.0 : 1.0;
    desired_linear_vel_ = sign * std::min(std::abs(desired_linear_vel_), speed_limit_);
  }

  if (reverse_enabled_) {
    desired_linear_vel_ = -desired_linear_vel_;
  }

  // 最近点为目标点（map 系）
  target_ps_.header = controller_path_.poses.at(plan_index_).header;
  target_ps_.pose = controller_path_.poses.at(plan_index_).pose;

  // 目标点从 map 转到 base 系，取纵向误差 err_x
  double err_x = 0.0;
  geometry_msgs::msg::PoseStamped target_in_base;
  geometry_msgs::msg::PoseStamped target_for_tf = target_ps_;
  target_for_tf.header.stamp = rclcpp::Time(0);
  if (nav2_util::transformPoseInTargetFrame(
      target_for_tf, target_in_base, *tf_, costmap_ros_->getBaseFrameID()))
  {
    // 倒车时 target_in_base.x 为负，因此计算的速度也为负。
    err_x = target_in_base.pose.position.x;
  } else {
    RCLCPP_WARN(logger_, "PureRotationController : Failed to transform target into base frame");
  }

  // 当前航向角
  const double theta_rob = tf2::getYaw(current_ps_.pose.orientation);  // [-pi, pi]
  double err_theta = std::fmod(goal_yaw_ - theta_rob + M_PI, 2 * M_PI) - M_PI;
  regularizeAngle(err_theta);

  cmd_vel.twist.angular.z = AngularSpeedControl(err_theta);
  if (enable_lon_adjust_) {
    cmd_vel.twist.linear.x = LinearSpeedControl(curr_lin_vel, err_x) * 0.25;
  } else {
    cmd_vel.twist.linear.x = 0.0;
  }

  return cmd_vel;
}

void PureRotationController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (speed_limit <= 0.0) {
    // 0 或负值视为取消限速。
    speed_limit_ = std::numeric_limits<double>::max();
    return;
  }
  if (percentage) {
    speed_limit_ = max_v_ * speed_limit / 100.0;
  } else {
    speed_limit_ = speed_limit;
  }
}

double PureRotationController::LinearSpeedControl(double curr_lin_vel, double e_dist)
{
  const double curr_vel = curr_lin_vel;

  const double speed_error = desired_linear_vel_ - curr_vel;

  const double speed_offset = station_pid_controller_.Control(e_dist, d_t_);
  const double speed_controller_input = speed_offset + speed_error;

  // Step: 计算加速度（速度是加速度的积分，所以得到的是加速度误差）
  double acceleration_cmd_closeloop =
    speed_pid_controller_.Control(speed_controller_input, d_t_);
  const double acceleration_cmd_closeloop_limited =
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

double PureRotationController::AngularSpeedControl(double err_theta)
{
  regularizeAngle(err_theta);

  double vel_angular_output = angular_pid_controller_.Control(err_theta, d_t_);

  if (!std::isfinite(vel_angular_output)) {
    vel_angular_output = 0.0;
  }

  vel_angular_output = std::clamp(vel_angular_output, min_w_, max_w_);

  return vel_angular_output;
}

int PureRotationController::getClosestWaypointID(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & pose,
  double ignore_dist,
  bool ignore_first) const
{
  if (path.poses.size() < 2) {  // 至少两个点，start 和 goal
    return -1;
  }

  int closest_id = -1;

  if (ignore_dist < 0.0) {
    ignore_dist = 0.0;
  }

  const double gdx = path.poses.back().pose.position.x - pose.pose.position.x;
  const double gdy = path.poses.back().pose.position.y - pose.pose.position.y;
  const double dist_to_goal = std::sqrt(gdx * gdx + gdy * gdy);
  if (dist_to_goal < ignore_dist) {
    return static_cast<int>(path.poses.size()) - 1;
  }

  double min_dist = std::numeric_limits<double>::max();
  for (int i = 0; i < static_cast<int>(path.poses.size()); ++i) {
    const double dx = path.poses.at(i).pose.position.x - pose.pose.position.x;
    const double dy = path.poses.at(i).pose.position.y - pose.pose.position.y;
    const double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < min_dist) {
      min_dist = dist;
      closest_id = i;
    }
  }

  if (ignore_first) {
    closest_id += 1;
    if (closest_id >= static_cast<int>(path.poses.size())) {
      closest_id = static_cast<int>(path.poses.size()) - 1;
    }
  }

  return closest_id;
}

double PureRotationController::CalculateMean(const std::deque<double> & data)
{
  double sum = 0.0;
  for (const auto & value : data) {
    sum += value;
  }
  return sum / static_cast<double>(data.size());
}

double PureRotationController::CalculateVariance(const std::deque<double> & data, double mean)
{
  double sum_sq = 0.0;
  for (const auto & value : data) {
    const double d = value - mean;
    sum_sq += d * d;
  }
  return sum_sq / static_cast<double>(data.size());
}

}  // namespace pure_rotation_controller

PLUGINLIB_EXPORT_CLASS(
  pure_rotation_controller::PureRotationController, nav2_core::Controller)
