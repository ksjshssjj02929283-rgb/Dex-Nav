// Copyright (c) 2023 Guo Linfeng (original ROS1 dcm_navigation)
// Copyright (c) 2026 dexforce (ROS2 / nav2_core port)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_pid_controller/pid_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_pid_controller
{

void PIDController::configure(
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

  global_frame_ = costmap_ros_->getGlobalFrameID();
  base_frame_ = costmap_ros_->getBaseFrameID();

  using nav2_util::declare_parameter_if_not_declared;
  const std::string & p = plugin_name_;

  declare_parameter_if_not_declared(node, p + ".long_precision", rclcpp::ParameterValue(0.02));
  declare_parameter_if_not_declared(node, p + ".lat_precision", rclcpp::ParameterValue(0.04));
  declare_parameter_if_not_declared(node, p + ".o_precision", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, p + ".angular_error_deadband", rclcpp::ParameterValue(0.008));

  declare_parameter_if_not_declared(node, p + ".max_v", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, p + ".min_v", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(node, p + ".max_v_inc", rclcpp::ParameterValue(0.5));

  declare_parameter_if_not_declared(node, p + ".max_w", rclcpp::ParameterValue(1.57));
  declare_parameter_if_not_declared(node, p + ".min_w", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(node, p + ".max_w_inc", rclcpp::ParameterValue(1.57));

  declare_parameter_if_not_declared(node, p + ".k_theta", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, p + ".switch_speed", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, p + ".default_desired_linear_vel", rclcpp::ParameterValue(0.15));
  declare_parameter_if_not_declared(
    node, p + ".final_rotation_max_v", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, p + ".final_rotation_linear_deadband", rclcpp::ParameterValue(0.005));
  declare_parameter_if_not_declared(
    node, p + ".speed_loop_cmd_reseed_error", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, p + ".angular_cmd_reseed_error", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, p + ".final_target_lock_distance", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, p + ".traj_speed_lookahead", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, p + ".traj_time_lookahead", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(node, p + ".verbose", rclcpp::ParameterValue(1));

  int verbose = 1;
  node->get_parameter(p + ".verbose", verbose);
  verbose_ = std::max(0, verbose);

  node->get_parameter(p + ".long_precision", long_precision_);
  node->get_parameter(p + ".lat_precision", lat_precision_);
  node->get_parameter(p + ".o_precision", o_precision_);
  node->get_parameter(p + ".angular_error_deadband", angular_error_deadband_);
  angular_error_deadband_ = std::fabs(angular_error_deadband_);

  node->get_parameter(p + ".max_v", max_v_);
  node->get_parameter(p + ".min_v", min_v_);
  node->get_parameter(p + ".max_v_inc", max_v_inc_);

  node->get_parameter(p + ".max_w", max_w_);
  node->get_parameter(p + ".min_w", min_w_);
  node->get_parameter(p + ".max_w_inc", max_w_inc_);

  node->get_parameter(p + ".k_theta", k_theta_);
  node->get_parameter(p + ".switch_speed", switch_speed_);
  node->get_parameter(p + ".default_desired_linear_vel", default_desired_linear_vel_);
  node->get_parameter(p + ".final_rotation_max_v", final_rotation_max_v_);
  node->get_parameter(p + ".final_rotation_linear_deadband", final_rotation_linear_deadband_);
  final_rotation_max_v_ = std::fabs(final_rotation_max_v_);
  final_rotation_linear_deadband_ = std::fabs(final_rotation_linear_deadband_);
  node->get_parameter(p + ".speed_loop_cmd_reseed_error", speed_cmd_reseed_error_);
  speed_cmd_reseed_error_ = std::fabs(speed_cmd_reseed_error_);
  node->get_parameter(p + ".angular_cmd_reseed_error", angular_cmd_reseed_error_);
  angular_cmd_reseed_error_ = std::fabs(angular_cmd_reseed_error_);
  node->get_parameter(p + ".final_target_lock_distance", final_target_lock_distance_);
  final_target_lock_distance_ = std::fabs(final_target_lock_distance_);
  node->get_parameter(p + ".traj_speed_lookahead", traj_speed_lookahead_);
  traj_speed_lookahead_ = std::fabs(traj_speed_lookahead_);
  node->get_parameter(p + ".traj_time_lookahead", traj_time_lookahead_);

  // ---- nested PID configs: name + ".<pid>.<field>" ----
  auto declare_pid = [&](const std::string & pid, nav2_controller_common::PidConf & conf,
      bool integrator_enable_def, double sat_def)
    {
      const std::string base = p + "." + pid;
      declare_parameter_if_not_declared(
        node, base + ".integrator_enable", rclcpp::ParameterValue(integrator_enable_def));
      declare_parameter_if_not_declared(
        node, base + ".integrator_saturation_level", rclcpp::ParameterValue(sat_def));
      declare_parameter_if_not_declared(node, base + ".kp", rclcpp::ParameterValue(1.00));
      declare_parameter_if_not_declared(node, base + ".ki", rclcpp::ParameterValue(0.01));
      declare_parameter_if_not_declared(node, base + ".kd", rclcpp::ParameterValue(0.10));
      declare_parameter_if_not_declared(node, base + ".kaw", rclcpp::ParameterValue(0.0));

      node->get_parameter(base + ".integrator_enable", conf.integrator_enable);
      node->get_parameter(base + ".integrator_saturation_level", conf.integrator_saturation_level);
      node->get_parameter(base + ".kp", conf.kp);
      node->get_parameter(base + ".ki", conf.ki);
      node->get_parameter(base + ".kd", conf.kd);
      node->get_parameter(base + ".kaw", conf.kaw);
    };

  declare_pid("high_speed_station_pid_conf", high_speed_station_pid_conf_, true, 0.3);
  declare_pid("low_speed_station_pid_conf", low_speed_station_pid_conf_, true, 0.3);
  declare_pid("high_speed_pid_conf", high_speed_pid_conf_, true, 0.3);
  declare_pid("low_speed_pid_conf", low_speed_pid_conf_, true, 0.3);
  declare_pid("reverse_station_pid_conf", reverse_station_pid_conf_, true, 0.5);
  declare_pid("reverse_speed_pid_conf", reverse_speed_pid_conf_, true, 0.5);
  declare_pid("angular_station_pid_conf", angular_station_pid_conf_, true, 0.5);

  // control time step from the controller server frequency
  double controller_frequency = 20.0;
  node->get_parameter_or("controller_frequency", controller_frequency, 20.0);
  if (controller_frequency > 0.0) {
    d_t_ = 1.0 / controller_frequency;
  }
  traj_time_lookahead_ = nav2_core::computeEffectiveTimeLookahead(traj_time_lookahead_, d_t_);

  station_nav2_pid_controller_.Init(low_speed_station_pid_conf_);
  speed_nav2_pid_controller_.Init(low_speed_pid_conf_);
  angular_station_nav2_pid_controller_.Init(angular_station_pid_conf_);

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&PIDController::dynamicParametersCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "PID controller \"%s\" configured.", plugin_name_.c_str());
}

rcl_interfaces::msg::SetParametersResult PIDController::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    if (parameter.get_name() != plugin_name_ + ".verbose") {
      continue;
    }
    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
      result.successful = false;
      result.reason = "verbose must be an integer (0 = quiet, >=1 = per-cycle diagnostics)";
      continue;
    }
    verbose_ = static_cast<int>(std::max<int64_t>(0, parameter.as_int()));
    RCLCPP_INFO(logger_, "PIDController verbose -> %d", verbose_.load());
  }
  return result;
}


void PIDController::activate()
{
  RCLCPP_INFO(logger_, "Activating PID controller \"%s\".", plugin_name_.c_str());
  last_speed_cmd_valid_ = false;
  last_angular_cmd_valid_ = false;
}

void PIDController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating PID controller \"%s\".", plugin_name_.c_str());
}

void PIDController::cleanup()
{
  dyn_params_handler_.reset();
  RCLCPP_INFO(logger_, "Cleaning up PID controller \"%s\".", plugin_name_.c_str());
  std::unique_lock<std::mutex> lock(plan_mutex_);
  last_trajectory_tracking_target_valid_ = false;
  controller_traj_.clear();
  trajectory_time_valid_ = false;
}

bool PIDController::cachePathAsTrajectory(const nav_msgs::msg::Path & path)
{
  // 单次 lookup + 逐点 doTransform：逐点 tf_->transform 在 TF 瞬时滞后时每点可阻塞到
  // 超时上限，规划级每控制周期分发的情况下会拖垮控制循环节拍。
  std::string source_frame = path.header.frame_id;
  if (!path.poses.empty() && !path.poses.front().header.frame_id.empty()) {
    source_frame = path.poses.front().header.frame_id;
  }
  geometry_msgs::msg::TransformStamped tf_to_global;
  bool need_transform = false;
  if (!source_frame.empty() && source_frame != global_frame_) {
    try {
      tf_to_global = tf_->lookupTransform(
        global_frame_, source_frame, rclcpp::Time(0), tf2::durationFromSec(0.05));
      need_transform = true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        logger_,
        "setPlan: TF transform from frame '%s' to '%s' failed: %s. "
        "Rejecting this path update and keeping the previous valid trajectory.",
        source_frame.c_str(), global_frame_.c_str(), ex.what());
      return false;
    }
  }

  std::vector<TrajPoint> transformed_traj;
  transformed_traj.reserve(path.poses.size());
  for (size_t i = 0; i < path.poses.size(); ++i) {
    TrajPoint tp;
    tp.pose = path.poses[i];
    if (tp.pose.header.frame_id.empty()) {
      tp.pose.header = path.header;
    }
    if (need_transform) {
      tf2::doTransform(tp.pose, tp.pose, tf_to_global);
    } else {
      tp.pose.header.frame_id = global_frame_;
    }
    tp.speed_limit = default_desired_linear_vel_;  // geometric path: fallback speed
    tp.x = tp.pose.pose.position.x;
    tp.y = tp.pose.pose.position.y;
    tp.curvature = 0.0;
    transformed_traj.push_back(std::move(tp));
  }

  if (transformed_traj.empty()) {
    RCLCPP_WARN(logger_, "setPlan: transformed trajectory is empty");
    return false;
  }

  const auto & input_goal = path.poses.back();
  const auto & cached_start = transformed_traj.front().pose.pose;
  const auto & cached_goal = transformed_traj.back().pose.pose;
  RCLCPP_DEBUG(
    logger_,
    "PIDController setPlan cached trajectory: plugin=%s poses=%zu source_frame=%s "
    "global_frame=%s input_goal=(x=%.3f y=%.3f yaw=%.3f) "
    "cached_start=(x=%.3f y=%.3f yaw=%.3f) cached_goal=(x=%.3f y=%.3f yaw=%.3f)",
    plugin_name_.c_str(), transformed_traj.size(),
    input_goal.header.frame_id.empty() ? path.header.frame_id.c_str() : input_goal.header.frame_id.c_str(),
    global_frame_.c_str(),
    input_goal.pose.position.x, input_goal.pose.position.y,
    tf2::getYaw(input_goal.pose.orientation),
    cached_start.position.x, cached_start.position.y, tf2::getYaw(cached_start.orientation),
    cached_goal.position.x, cached_goal.position.y, tf2::getYaw(cached_goal.orientation));

  controller_traj_ = std::move(transformed_traj);
  trajectory_time_valid_ = false;
  return true;
}

void PIDController::setPlan(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    RCLCPP_WARN(logger_, "setPlan: received empty path");
    return;
  }

  std::unique_lock<std::mutex> lock(plan_mutex_);
  if (!cachePathAsTrajectory(path)) {
    return;
  }

  plan_index_ = std::min(1, static_cast<int>(controller_traj_.size()) - 1);

  const auto & back = controller_traj_.back().pose.pose;
  goal_x_ = back.position.x;
  goal_y_ = back.position.y;
  goal_yaw_ = tf2::getYaw(back.orientation);
}

void PIDController::setTrajectory(const nav_interfaces::msg::RobotTrajectory & trajectory)
{
  if (trajectory.points.empty()) {
    RCLCPP_WARN(logger_, "setTrajectory: received empty trajectory");
    return;
  }

  std::unique_lock<std::mutex> lock(plan_mutex_);

  // 单次 lookup + 逐点 doTransform（同 cachePathAsTrajectory，避免逐点 TF 阻塞）。
  std::string source_frame = trajectory.header.frame_id;
  if (!trajectory.points.empty() && !trajectory.points.front().pose.header.frame_id.empty()) {
    source_frame = trajectory.points.front().pose.header.frame_id;
  }
  geometry_msgs::msg::TransformStamped tf_to_global;
  bool need_transform = false;
  if (!source_frame.empty() && source_frame != global_frame_) {
    try {
      tf_to_global = tf_->lookupTransform(
        global_frame_, source_frame, rclcpp::Time(0), tf2::durationFromSec(0.05));
      need_transform = true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        logger_,
        "setTrajectory: TF transform from frame '%s' to '%s' failed: %s. "
        "Rejecting this trajectory update and keeping the previous valid trajectory.",
        source_frame.c_str(), global_frame_.c_str(), ex.what());
      return;
    }
  }

  std::vector<TrajPoint> transformed_traj;
  transformed_traj.reserve(trajectory.points.size());
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    const auto & pt = trajectory.points[i];
    TrajPoint tp;
    tp.pose = pt.pose;
    if (tp.pose.header.frame_id.empty()) {
      tp.pose.header = trajectory.header;
    }
    if (need_transform) {
      tf2::doTransform(tp.pose, tp.pose, tf_to_global);
    } else {
      tp.pose.header.frame_id = global_frame_;
    }
    // per-point desired velocity and curvature from SpeedPlanner
    tp.speed_limit = pt.longitudinal_velocity;
    tp.curvature = pt.path_kappa;
    tp.accel = pt.acceleration;
    tp.x = tp.pose.pose.position.x;
    tp.y = tp.pose.pose.position.y;
    tp.relative_time = nav2_core::durationToSeconds(pt.relative_time);
    transformed_traj.push_back(std::move(tp));
  }

  if (transformed_traj.empty()) {
    RCLCPP_WARN(logger_, "setTrajectory: transformed trajectory is empty");
    return;
  }

  const auto & input_goal = trajectory.points.back().pose;
  const auto & cached_start = transformed_traj.front().pose.pose;
  const auto & cached_goal = transformed_traj.back().pose.pose;
  RCLCPP_DEBUG(
    logger_,
    "PIDController setTrajectory cached trajectory: plugin=%s poses=%zu source_frame=%s "
    "global_frame=%s input_goal=(x=%.3f y=%.3f yaw=%.3f) "
    "cached_start=(x=%.3f y=%.3f yaw=%.3f) cached_goal=(x=%.3f y=%.3f yaw=%.3f)",
    plugin_name_.c_str(), transformed_traj.size(),
    input_goal.header.frame_id.empty() ? trajectory.header.frame_id.c_str() : input_goal.header.frame_id.c_str(),
    global_frame_.c_str(),
    input_goal.pose.position.x, input_goal.pose.position.y,
    tf2::getYaw(input_goal.pose.orientation),
    cached_start.position.x, cached_start.position.y, tf2::getYaw(cached_start.orientation),
    cached_goal.position.x, cached_goal.position.y, tf2::getYaw(cached_goal.orientation));

  controller_traj_ = std::move(transformed_traj);
  trajectory_time_valid_ = nav2_core::hasValidRelativeTime(controller_traj_);

  plan_index_ = std::min(1, static_cast<int>(controller_traj_.size()) - 1);

  const auto & back = controller_traj_.back().pose.pose;
  goal_x_ = back.position.x;
  goal_y_ = back.position.y;
  goal_yaw_ = tf2::getYaw(back.orientation);
}

void PIDController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (speed_limit == 0.0) {
    // 0 means "no limit"
    speed_limit_ = std::numeric_limits<double>::infinity();
    return;
  }
  if (percentage) {
    speed_limit_ = std::fabs(max_v_ * speed_limit / 100.0);
  } else {
    speed_limit_ = std::fabs(speed_limit);
  }
}

bool PIDController::isGoalReached()
{
  return false;
}

const char * PIDController::controllerStageName(ControllerStage stage) const
{
  switch (stage) {
    case FINISH:
      return "FINISH";
    case APPROACHING:
      return "APPROACHING";
    case ROTATING_TO_GOAL:
      return "ROTATING_TO_GOAL";
    default:
      return "UNKNOWN";
  }
}

geometry_msgs::msg::TwistStamped PIDController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = base_frame_;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.angular.z = 0.0;

  std::unique_lock<std::mutex> lock(plan_mutex_);
  last_trajectory_tracking_target_valid_ = false;

  if (controller_traj_.empty()) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "computeVelocityCommands: no plan set");
    return cmd_vel;
  }

  const double input_pose_yaw = tf2::getYaw(pose.pose.orientation);

  // current pose is provided by nav2 in the global frame
  current_ps_ = pose;
  if (current_ps_.header.frame_id.empty()) {
    current_ps_.header.frame_id = global_frame_;
  }

  // when reversing, treat the robot as if it faced the opposite direction
  if (reverse_enabled_) {
    double reverse_yaw = tf2::getYaw(current_ps_.pose.orientation) + M_PI;
    regularizeAngle(reverse_yaw);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, reverse_yaw);
    current_ps_.pose.orientation = tf2::toMsg(q);
  }

  pruneGlobalPlan(current_ps_, controller_traj_, 2.0);

  // 距终点小于 final_target_lock_distance_ 时直接锁定终点索引（不再最近点搜索）：
  // 机器人微小位移会让"最近点"在相邻但方位角相差很大的候选点间跳变，
  // 锁定终点后瞄准基准不再随机器人位置漂移。
  plan_index_ = getClosestWaypointID(
    controller_traj_, current_ps_, final_target_lock_distance_, true);
  if (plan_index_ < 0) {
    RCLCPP_WARN(logger_, "computeVelocityCommands: failed to get closest waypoint");
    return cmd_vel;
  }

  // 速度参考与瞄准目标解耦：plan_index_ 距终点内会锁定到终点索引（瞄准需要），
  // 但终点的 speed_limit 是收尾蠕行值——直接拿它当 desired_v 会在锁定的瞬间
  // 跳变到蠕行速度，而不是沿轨迹连续下降。这里单独做最近点搜索取速度。
  double traj_v = 0.0;
  double traj_ff_accel = 0.0;
  const bool have_traj_speed = nearestTrajectorySpeed(current_ps_, traj_v, traj_ff_accel);
  desired_linear_vel_ = have_traj_speed ? traj_v : controller_traj_[plan_index_].speed_limit;
  // clamp the per-point desired speed by the externally-imposed speed limit
  if (std::isfinite(speed_limit_)) {
    desired_linear_vel_ = std::min(std::fabs(desired_linear_vel_), speed_limit_) *
      (desired_linear_vel_ < 0.0 ? -1.0 : 1.0);
  }
  if (reverse_enabled_) {
    desired_linear_vel_ = -std::fabs(desired_linear_vel_);
  }

  // closest point is the target point (in global frame)
  target_ps_ = controller_traj_[plan_index_].pose;

  double theta_to_goal;  // angle from robot to target
  double theta_traj;     // angle along the trajectory
  double theta_dir;      // desired direction

  theta_to_goal = std::atan2(
    (target_ps_.pose.position.y - current_ps_.pose.position.y),
    (target_ps_.pose.position.x - current_ps_.pose.position.x));

  int next_plan_index = plan_index_ + 1;
  if (next_plan_index < static_cast<int>(controller_traj_.size())) {
    theta_traj = std::atan2(
      (controller_traj_[next_plan_index].pose.pose.position.y - target_ps_.pose.position.y),
      (controller_traj_[next_plan_index].pose.pose.position.x - target_ps_.pose.position.x));

    if (std::fabs(theta_traj - theta_to_goal) > M_PI) {
      if (theta_traj > theta_to_goal) {
        theta_to_goal += 2 * M_PI;
      } else {
        theta_traj += 2 * M_PI;
      }
    }
    theta_dir = k_theta_ * theta_to_goal + (1 - k_theta_) * theta_traj;
  } else {
    theta_dir = theta_to_goal;
  }
  regularizeAngle(theta_dir);

  double err_x, err_y;
  double err_theta;

  // transform target from global into base frame
  geometry_msgs::msg::PoseStamped target_in_base;
  geometry_msgs::msg::PoseStamped target_query = target_ps_;
  target_query.header.stamp = rclcpp::Time(0);
  try {
    tf_->transform(target_query, target_in_base, base_frame_, tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(logger_, "computeVelocityCommands: TF transform to %s failed: %s",
      base_frame_.c_str(), ex.what());
    return cmd_vel;
  }
  // for reverse, target_in_base.x is negative, so the computed speed is negative too
  err_x = target_in_base.pose.position.x;
  err_y = target_in_base.pose.position.y;

  const auto & cached_goal_ps = controller_traj_.back().pose;
  const double goal_dx_global = cached_goal_ps.pose.position.x - pose.pose.position.x;
  const double goal_dy_global = cached_goal_ps.pose.position.y - pose.pose.position.y;
  const double goal_dist_global = std::hypot(goal_dx_global, goal_dy_global);
  double goal_yaw_error_global = goal_yaw_ - input_pose_yaw;
  regularizeAngle(goal_yaw_error_global);

  double goal_x_base = std::numeric_limits<double>::quiet_NaN();
  double goal_y_base = std::numeric_limits<double>::quiet_NaN();
  try {
    geometry_msgs::msg::PoseStamped goal_in_base;
    geometry_msgs::msg::PoseStamped goal_query = cached_goal_ps;
    goal_query.header.stamp = rclcpp::Time(0);
    tf_->transform(goal_query, goal_in_base, base_frame_, tf2::durationFromSec(0.1));
    goal_x_base = goal_in_base.pose.position.x;
    goal_y_base = goal_in_base.pose.position.y;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "computeVelocityCommands: cached goal TF transform from %s to %s failed: %s",
      cached_goal_ps.header.frame_id.c_str(), base_frame_.c_str(), ex.what());
  }

  double goal_checker_long_precision = std::numeric_limits<double>::quiet_NaN();
  double goal_checker_lat_precision = std::numeric_limits<double>::quiet_NaN();
  bool goal_checker_tolerance_valid = false;
  if (goal_checker != nullptr) {
    geometry_msgs::msg::Pose pose_tolerance;
    geometry_msgs::msg::Twist velocity_tolerance;
    goal_checker_tolerance_valid = goal_checker->getTolerances(pose_tolerance, velocity_tolerance);
    if (goal_checker_tolerance_valid) {
      goal_checker_long_precision = pose_tolerance.position.x;
      goal_checker_lat_precision = pose_tolerance.position.y;
    }
  }

  auto strict_positive_min = [](double configured, double checker) {
      if (std::isfinite(checker) && checker > 0.0) {
        return std::min(std::fabs(configured), checker);
      }
      return std::fabs(configured);
    };

  const double final_goal_long_precision =
    strict_positive_min(long_precision_, goal_checker_long_precision);
  const double final_goal_lat_precision =
    strict_positive_min(lat_precision_, goal_checker_lat_precision);
  const double final_goal_dist_precision =
    std::max(final_goal_long_precision, final_goal_lat_precision);
  constexpr double kFinalRotationEntryMinDistance = 0.03;
  const double final_rotation_entry_long_precision =
    std::max(final_goal_long_precision, kFinalRotationEntryMinDistance);
  const double final_rotation_entry_lat_precision =
    std::max(final_goal_lat_precision, kFinalRotationEntryMinDistance);
  const double final_rotation_entry_dist_precision =
    std::max(final_goal_dist_precision, kFinalRotationEntryMinDistance);

  const bool target_position_tolerance =
    std::fabs(err_x) < final_goal_long_precision &&
    std::fabs(err_y) < final_goal_lat_precision;
  const bool final_goal_error_valid =
    std::isfinite(goal_x_base) && std::isfinite(goal_y_base);
  const bool final_goal_position_tolerance =
    final_goal_error_valid &&
    std::fabs(goal_x_base) < final_goal_long_precision &&
    std::fabs(goal_y_base) < final_goal_lat_precision &&
    goal_dist_global < final_goal_dist_precision;
  const bool final_rotation_entry_position_tolerance =
    final_goal_error_valid &&
    std::fabs(goal_x_base) < final_rotation_entry_long_precision &&
    std::fabs(goal_y_base) < final_rotation_entry_lat_precision &&
    goal_dist_global < final_rotation_entry_dist_precision;
  const double final_rotation_linear_error =
    final_goal_error_valid ? goal_x_base : err_x;
  const char * final_rotation_linear_source =
    final_goal_error_valid ? "goal_base_x" : "target_wp_base_x";

  double dist_to_target = std::hypot(
    target_ps_.pose.position.x - current_ps_.pose.position.x,
    target_ps_.pose.position.y - current_ps_.pose.position.y);

  // Step: switch mode
  if (goal_dist_global > 0.2 || curr_controller_stage_ == FINISH ||
    (curr_controller_stage_ == ROTATING_TO_GOAL && !final_rotation_entry_position_tolerance))
  {
    curr_controller_stage_ = APPROACHING;
    theta_errors_.clear();
    longitude_errors_.clear();
  }

  if (final_rotation_entry_position_tolerance) {
    curr_controller_stage_ = ROTATING_TO_GOAL;
  }

  // current heading
  double theta_rob = tf2::getYaw(current_ps_.pose.orientation);  // [-pi, pi]
  double target_yaw_for_log = theta_dir;

  if (curr_controller_stage_ == ROTATING_TO_GOAL) {
    if (reverse_enabled_) {
      theta_rob -= M_PI;
      regularizeAngle(theta_rob);
    }
    target_yaw_for_log = goal_yaw_;
    err_theta = std::fmod(goal_yaw_ - theta_rob + M_PI, 2 * M_PI) - M_PI;
  } else {
    target_yaw_for_log = theta_dir;
    err_theta = std::fmod(theta_dir - theta_rob + M_PI, 2 * M_PI) - M_PI;
  }
  regularizeAngle(target_yaw_for_log);
  regularizeAngle(err_theta);

  // longitudinal error statistics (sliding window of 20)
  const double longitude_error_for_stats =
    final_goal_error_valid ? std::fabs(goal_x_base) : std::fabs(err_x);
  longitude_errors_.push_back(longitude_error_for_stats);
  while (longitude_errors_.size() > 20) {
    longitude_errors_.pop_front();
  }
  double longitude_mean = std::numeric_limits<double>::max();
  if (longitude_errors_.size() >= 20) {
    longitude_mean = CalculateMean(longitude_errors_);
    (void)CalculateVariance(longitude_errors_, longitude_mean);
  }

  // heading error statistics (sliding window of 50)
  theta_errors_.push_back(std::fabs(err_theta));
  while (theta_errors_.size() > 50) {
    theta_errors_.pop_front();
  }
  double theta_mean = std::numeric_limits<double>::infinity();
  if (theta_errors_.size() >= 50) {
    theta_mean = CalculateMean(theta_errors_);
    (void)CalculateVariance(theta_errors_, theta_mean);
  }

  if (curr_controller_stage_ != prev_controller_stage_) {
    if (curr_controller_stage_ == ROTATING_TO_GOAL) {
      angular_station_nav2_pid_controller_.Reset();
    }
    if (curr_controller_stage_ == APPROACHING ||
      curr_controller_stage_ == ROTATING_TO_GOAL)
    {
      // 阶段切换是控制权边界：上一段累积的积分与指令基准对新阶段是陈旧状态。
      // approach 分支 position_deadband=0，不能依赖 deadband 路径里的 Reset。
      station_nav2_pid_controller_.Reset();
      speed_nav2_pid_controller_.Reset();
      last_speed_cmd_valid_ = false;
      last_angular_cmd_valid_ = false;
    }
    switch (curr_controller_stage_) {
      case FINISH:
        RCLCPP_INFO(logger_, "controller stage: FINISH");
        break;
      case APPROACHING:
        RCLCPP_INFO(logger_, "controller stage: APPROACHING");
        break;
      case ROTATING_TO_GOAL:
        RCLCPP_INFO(logger_, "controller stage: ROTATING_TO_GOAL");
        break;
      default:
        break;
    }
  }
  prev_controller_stage_ = curr_controller_stage_;

  double angle_to_heading = 0.0;
  const char * active_branch = "unset";
  const bool final_rotation_control = curr_controller_stage_ == ControllerStage::ROTATING_TO_GOAL;
  last_linear_position_cmd_ = 0.0;
  last_linear_target_v_ = 0.0;
  last_linear_speed_error_ = 0.0;
  last_linear_accel_cmd_ = 0.0;
  last_linear_accel_limited_ = 0.0;
  last_linear_limit_ = 0.0;
  last_linear_min_target_v_ = 0.0;
  last_linear_deadband_ = 0.0;
  if (!reverse_enabled_ &&
    !final_rotation_control &&
    false)
  {
    active_branch = "rotate_to_path";
    rotateToHeading(cmd_vel.twist.linear.x, cmd_vel.twist.angular.z, angle_to_heading, velocity);
  } else {
    switch (curr_controller_stage_) {
      case FINISH: {
          active_branch = "finish_zero";
          cmd_vel.twist.linear.x = 0.0;
          cmd_vel.twist.angular.z = 0.0;
          break;
        }
      case APPROACHING: {
          active_branch = "approaching_traj_speed";
          // 期望速度 = 最近轨迹点的规划速度（速度环跟踪目标，方向由纵向误差决定）：
          // 减速尾段/曲率限速/障碍零速已由速度规划层逐点编码，跟踪即可；
          // desired_v=0（障碍置零点）时 LinearSpeedControl 直接输出 0。
          // 末端停车精度由 ROTATING_TO_GOAL 入口的 final goal 容差门控接管。
          // err_x < 0（参考点在机器人身后 = 越过目标）允许负速度倒回——
          // 距离收敛优先于"只前进"；倒回上限不被末端蠕行速度钳死，
          // 放宽到 final_rotation_max_v（与最终旋转段线性纠偏权限一致）。
          const double desired_v = std::fabs(desired_linear_vel_);
          const double raw_max_cmd_v = (err_x < 0.0) ?
            std::max(desired_v, final_rotation_max_v_) : desired_v;
          // 朝向-速度平滑耦合：速度上限连续按 cos(err_theta) 衰减（非离散置零，
          // 无速度跳变）。err_theta=0 时不缩放；|err_theta|>=90° 时缩为 0。
          // 机器人平移时瞄准点方位角会以 v·sin(Δθ)/d 的速率扫动（Δθ=朝向误差、
          // d=到瞄准点距离），朝向误差越大越接近该点扫动越剧烈，形成"越转越追不上"
          // 的正反馈——按 cos(Δθ) 缩放 v 后峰值扫动速率减半且从 90° 挪到 45°，
          // 配合上方瞄准点已锁定终点（不再随机器人平移漂移）共同压住这个反馈。
          const double heading_scale = headingSpeedScale(err_theta);
          const double max_cmd_v = raw_max_cmd_v * heading_scale;
          cmd_vel.twist.linear.x = LinearSpeedControl(
            velocity, err_x, max_cmd_v, desired_v, 0.0, traj_ff_accel);
          if (have_traj_speed) {
            last_trajectory_tracking_target_valid_ = true;
            last_trajectory_tracking_target_v_ = desired_linear_vel_;
            last_trajectory_tracking_target_a_ = traj_ff_accel;
          }
          cmd_vel.twist.angular.z = AngularSpeedControl(velocity, err_theta);
          break;
        }
      case ROTATING_TO_GOAL: {
          active_branch = "rotating_to_goal_signed_speed_pid";
          cmd_vel.twist.linear.x = LinearSpeedControl(
            velocity, final_rotation_linear_error, final_rotation_max_v_,
            final_rotation_max_v_, final_rotation_linear_deadband_);
          cmd_vel.twist.angular.z = AngularSpeedControl(velocity, err_theta, true);
          break;
        }
      default:
        active_branch = "unknown_zero";
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        break;
    }
  }

  if (verbose_) {
    constexpr double kRad2Deg = 180.0 / M_PI;
    // 角度一律用度（写作 deg 而非 °，避免多字节符号在部分终端/日志管道乱码）；
    // 误差标 lon/lat 而非 x/y——base_x/base_y 读起来像全局坐标，实为机体系投影。
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 100,
      "PIDController %s/%s | pose(%.3f, %.3f, %+.1fdeg) -> goal(%.3f, %.3f, %+.1fdeg) "
      "straight %.4f | goal_err lon %+.4f lat %+.4f yaw %+.1fdeg | "
      "aim[%d/%zu] lon %+.4f lat %+.4f heading_err %+.1fdeg aim_yaw %+.1fdeg | "
      "speed desired %.3f measured %+.3f err %+.3f limit %.3f pos_loop %+.3f "
      "accel %+.3f(lim %+.3f) | cmd vx %+.3f wz %+.3f",
      controllerStageName(curr_controller_stage_), active_branch,
      pose.pose.position.x, pose.pose.position.y, input_pose_yaw * kRad2Deg,
      cached_goal_ps.pose.position.x, cached_goal_ps.pose.position.y,
      goal_yaw_ * kRad2Deg, goal_dist_global,
      goal_x_base, goal_y_base, goal_yaw_error_global * kRad2Deg,
      plan_index_, controller_traj_.size(),
      err_x, err_y, err_theta * kRad2Deg, target_yaw_for_log * kRad2Deg,
      desired_linear_vel_, velocity.linear.x, last_linear_speed_error_,
      last_linear_limit_, last_linear_position_cmd_,
      last_linear_accel_cmd_, last_linear_accel_limited_,
      cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);
  }

  RCLCPP_DEBUG_THROTTLE(
    logger_, *clock_, 100,
    "PIDController command: plugin=%s stage=%s branch=%s plan_index=%d/%zu "
    "desired_v=%.3f "
    "robot_pose=(frame=%s x=%.3f y=%.3f yaw=%.3f control_yaw=%.3f reverse=%s) "
    "cached_goal=(frame=%s x=%.3f y=%.3f yaw=%.3f) "
    "goal_error=(global_dx=%.3f global_dy=%.3f dist=%.3f yaw=%.3f "
    "base_x=%.3f base_y=%.3f) "
    "target_wp=(frame=%s index=%d x=%.3f y=%.3f yaw=%.3f base_x=%.3f base_y=%.3f) "
    "stage_gate=(target_in_tol=%s final_in_tol=%s final_valid=%s "
    "pid_tol=(long=%.3f lat=%.3f) goal_checker_tol=(valid=%s long=%.3f lat=%.3f) "
    "effective_tol=(long=%.3f lat=%.3f dist=%.3f) "
    "rotation_entry=%s rotation_entry_tol=(long=%.3f lat=%.3f dist=%.3f) "
    "final_linear_source=%s final_linear_error=%.3f) "
    "errors=(x=%.3f y=%.3f theta=%.3f dist_target=%.3f "
    "long_mean=%.3f theta_mean=%.3f) "
    "angle=(current_yaw=%.3f target_yaw=%.3f error=%.3f cmd_wz=%.3f) "
    "linear_ctrl=(pos_cmd=%.3f target_v=%.3f speed_error=%.3f accel=%.3f accel_limited=%.3f "
    "limit=%.3f min_target=%.3f deadband=%.3f max_v_inc=%.3f final_rotation=%s) "
    "angular_cfg=(deadband=%.3f min_w=%.3f max_w=%.3f kp=%.3f final_rotation=%s) "
    "heading=(theta_dir=%.3f angle_to_path=%.3f) odom=(vx=%.3f wz=%.3f) "
    "cmd_vel=(vx=%.3f wz=%.3f)",
    plugin_name_.c_str(), controllerStageName(curr_controller_stage_), active_branch,
    plan_index_, controller_traj_.size(), desired_linear_vel_,
    pose.header.frame_id.c_str(), pose.pose.position.x, pose.pose.position.y,
    input_pose_yaw, theta_rob, reverse_enabled_ ? "true" : "false",
    cached_goal_ps.header.frame_id.c_str(), cached_goal_ps.pose.position.x,
    cached_goal_ps.pose.position.y, goal_yaw_,
    goal_dx_global, goal_dy_global, goal_dist_global, goal_yaw_error_global,
    goal_x_base, goal_y_base,
    target_ps_.header.frame_id.c_str(), plan_index_, target_ps_.pose.position.x,
    target_ps_.pose.position.y, tf2::getYaw(target_ps_.pose.orientation), err_x, err_y,
    target_position_tolerance ? "true" : "false",
    final_goal_position_tolerance ? "true" : "false",
    final_goal_error_valid ? "true" : "false",
    long_precision_, lat_precision_,
    goal_checker_tolerance_valid ? "true" : "false",
    goal_checker_long_precision, goal_checker_lat_precision,
    final_goal_long_precision, final_goal_lat_precision, final_goal_dist_precision,
    final_rotation_entry_position_tolerance ? "true" : "false",
    final_rotation_entry_long_precision, final_rotation_entry_lat_precision,
    final_rotation_entry_dist_precision,
    final_rotation_linear_source, final_rotation_linear_error,
    err_x, err_y, err_theta, dist_to_target, longitude_mean, theta_mean,
    theta_rob, target_yaw_for_log, err_theta, cmd_vel.twist.angular.z,
    last_linear_position_cmd_, last_linear_target_v_, last_linear_speed_error_,
    last_linear_accel_cmd_, last_linear_accel_limited_, last_linear_limit_,
    last_linear_min_target_v_, last_linear_deadband_, max_v_inc_,
    final_rotation_control ? "true" : "false",
    angular_error_deadband_, min_w_, max_w_, angular_station_pid_conf_.kp,
    final_rotation_control ? "true" : "false",
    theta_dir, angle_to_heading,
    velocity.linear.x, velocity.angular.z,
    cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);

  return cmd_vel;
}

double PIDController::LinearSpeedControl(
  const geometry_msgs::msg::Twist & velocity,
  double e_dist,
  double max_linear_speed,
  double min_target_speed,
  double position_deadband,
  double feedforward_accel)
{
  double curr_vel = velocity.linear.x;
  if (reverse_enabled_) {
    station_nav2_pid_controller_.SetPID(reverse_station_pid_conf_);
    speed_nav2_pid_controller_.SetPID(reverse_speed_pid_conf_);
  } else {
    if (std::fabs(curr_vel) < switch_speed_) {
      station_nav2_pid_controller_.SetPID(low_speed_station_pid_conf_);
      speed_nav2_pid_controller_.SetPID(low_speed_pid_conf_);
    } else {
      station_nav2_pid_controller_.SetPID(high_speed_station_pid_conf_);
      speed_nav2_pid_controller_.SetPID(high_speed_pid_conf_);
    }
  }

  last_linear_position_cmd_ = 0.0;
  last_linear_target_v_ = 0.0;
  last_linear_speed_error_ = 0.0;
  last_linear_accel_cmd_ = 0.0;
  last_linear_accel_limited_ = 0.0;
  last_linear_limit_ = 0.0;
  last_linear_min_target_v_ = std::fabs(min_target_speed);
  last_linear_deadband_ = std::fabs(position_deadband);

  if (std::fabs(e_dist) <= std::fabs(position_deadband)) {
    station_nav2_pid_controller_.Reset();
    speed_nav2_pid_controller_.Reset();
    last_speed_cmd_ = 0.0;
    last_speed_cmd_valid_ = true;
    return 0.0;
  }

  double linear_limit = std::fabs(max_linear_speed);
  if (!std::isfinite(linear_limit) || linear_limit <= 0.0) {
    linear_limit = std::fabs(desired_linear_vel_) * 1.5;
  }
  if (!std::isfinite(linear_limit) || linear_limit <= 0.0) {
    linear_limit = std::fabs(default_desired_linear_vel_);
  }
  if (std::isfinite(speed_limit_)) {
    linear_limit = std::min(linear_limit, std::fabs(speed_limit_));
  }
  linear_limit = std::min(linear_limit, std::fabs(max_v_));
  last_linear_limit_ = linear_limit;
  if (!std::isfinite(linear_limit) || linear_limit <= 0.0) {
    // 满停（障碍零速/无有效限幅）：清两级积分并归零指令基准，
    // 避免停车期间的陈旧积分在恢复行驶时注入瞬态。
    station_nav2_pid_controller_.Reset();
    speed_nav2_pid_controller_.Reset();
    last_speed_cmd_ = 0.0;
    last_speed_cmd_valid_ = true;
    return 0.0;
  }

  double position_cmd = station_nav2_pid_controller_.Control(e_dist, d_t_);
  if (!std::isfinite(position_cmd)) {
    position_cmd = 0.0;
  }
  position_cmd = std::clamp(position_cmd, -linear_limit, linear_limit);
  if (position_cmd * e_dist < 0.0) {
    position_cmd = 0.0;
  }

  const double min_target_abs = std::min(std::fabs(min_target_speed), linear_limit);
  double target_vel = position_cmd;
  if (min_target_abs > 0.0 && std::fabs(target_vel) < min_target_abs) {
    target_vel = std::copysign(min_target_abs, e_dist);
  }
  target_vel = std::clamp(target_vel, -linear_limit, linear_limit);

  const double speed_error = target_vel - curr_vel;
  // 前馈 + 反馈：纯反馈跟踪持续下降的斜坡目标存在稳态滞后（≈斜率/kp），
  // 前馈把规划层已知的"此刻期望加速度"直接喂给控制器，反馈只需修正残差。
  // 减速坡道起点等场景下前馈会合法地与瞬时速度误差反号（目标尚高于实测、
  // 但坡道已要求减速），因此不做"加速度必须与误差同向"的方向守卫，
  // 方向与幅值安全由下方符号感知输出界承担。
  const double feedback_accel = speed_nav2_pid_controller_.Control(speed_error, d_t_);
  double acceleration_cmd = feedforward_accel + feedback_accel;
  if (!std::isfinite(acceleration_cmd)) {
    acceleration_cmd = 0.0;
  }
  const double acceleration_cmd_limited =
    std::clamp(acceleration_cmd, -std::fabs(max_v_inc_), std::fabs(max_v_inc_));

  // 指令积分器：输出锚定上一周期指令，实测只进误差项。
  if (!last_speed_cmd_valid_ ||
    std::fabs(last_speed_cmd_ - curr_vel) > speed_cmd_reseed_error_)
  {
    last_speed_cmd_ = curr_vel;
    last_speed_cmd_valid_ = true;
  }
  double v_output = last_speed_cmd_ + acceleration_cmd_limited * d_t_;

  // 符号感知输出界：目标速度既是跟踪参考也是上限（前向不越目标、不下穿零倒转，
  // 倒退目标对称）；制动权保留（可低于目标直至 0）。
  const double bound_hi = std::min(std::max(0.0, target_vel), linear_limit);
  const double bound_lo = std::max(std::min(0.0, target_vel), -linear_limit);
  v_output = std::clamp(v_output, bound_lo, bound_hi);

  // 反死锁兜底：目标要求脱离零速，但指令被符号界钳在零上。此时前馈是负的
  // （方向与目标相反），而 0 又会成为下一周期的指令基准 —— 逐周期自复现，
  // 机器人永远起不来（R49：末端 6cm 静止 7s，指令恒 0）。这种前馈描述的是
  // 与被跟踪目标不同的工况（采样点配错段），本周期退化为纯反馈。
  // 只在"输出贴零 + 目标非零"这一个几何下触发，不影响正常制动权限：
  // 车还在跑时 v_output = curr_vel + a·dt > 0，不会贴到零界。
  double applied_accel_cmd = acceleration_cmd;
  double applied_accel_limited = acceleration_cmd_limited;
  if (std::fabs(v_output) <= 1e-9 && std::fabs(target_vel) > 1e-6 &&
    feedforward_accel * target_vel < 0.0)
  {
    applied_accel_cmd = std::isfinite(feedback_accel) ? feedback_accel : 0.0;
    applied_accel_limited =
      std::clamp(applied_accel_cmd, -std::fabs(max_v_inc_), std::fabs(max_v_inc_));
    v_output = std::clamp(
      last_speed_cmd_ + applied_accel_limited * d_t_, bound_lo, bound_hi);
  }

  if (min_v_ > 0.0 && std::fabs(v_output) > 0.0 && std::fabs(v_output) < min_v_) {
    v_output = std::copysign(min_v_, v_output);
  }
  v_output = std::clamp(v_output, -linear_limit, linear_limit);
  last_speed_cmd_ = v_output;

  last_linear_position_cmd_ = position_cmd;
  last_linear_target_v_ = target_vel;
  last_linear_speed_error_ = speed_error;
  last_linear_accel_cmd_ = applied_accel_cmd;
  last_linear_accel_limited_ = applied_accel_limited;
  last_linear_limit_ = linear_limit;

  return v_output;
}

double PIDController::AngularSpeedControl(
  const geometry_msgs::msg::Twist & velocity, double err_theta, bool final_rotation)
{
  regularizeAngle(err_theta);
  if (std::fabs(err_theta) <= angular_error_deadband_) {
    angular_station_nav2_pid_controller_.Reset();
    last_angular_cmd_ = 0.0;
    last_angular_cmd_valid_ = true;
    return 0.0;
  }

  double target_w = 0.0;
  if (final_rotation) {
    target_w = angular_station_pid_conf_.kp * err_theta;
  } else {
    target_w = angular_station_nav2_pid_controller_.Control(err_theta, d_t_);
  }

  if (!std::isfinite(target_w)) {
    target_w = 0.0;
  }

  if (target_w * err_theta < 0.0) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "Angular PID output reversed: error=%.4f raw_wz=%.4f, forcing error direction",
      err_theta, target_w);
    target_w = std::copysign(min_w_, err_theta);
  }

  const double angular_sign = std::copysign(1.0, err_theta);
  double target_abs = std::fabs(target_w);
  if (min_w_ > 0.0 && target_abs < min_w_) {
    target_abs = min_w_;
  }
  if (target_abs > 0.0) {
    target_abs = std::clamp(target_abs, min_w_, max_w_);
  }
  target_w = angular_sign * target_abs;

  // 变化率限幅：镜像线速度通道的加速度积分器（指令锚定上一周期、实测只在
  // 重新播种时参与）——朝向误差突变（如切入 ROTATING_TO_GOAL）时角速度
  // 不再单周期直接砸到 target_w，而是按 max_w_inc_ 逐周期爬升/制动。
  double curr_w = velocity.angular.z;
  if (!last_angular_cmd_valid_ ||
    std::fabs(last_angular_cmd_ - curr_w) > angular_cmd_reseed_error_)
  {
    last_angular_cmd_ = curr_w;
    last_angular_cmd_valid_ = true;
  }
  const double angular_accel_cmd = std::clamp(
    (target_w - last_angular_cmd_) / d_t_, -std::fabs(max_w_inc_), std::fabs(max_w_inc_));
  double w_output = last_angular_cmd_ + angular_accel_cmd * d_t_;

  // 符号感知输出界：target_w 既是跟踪参考也是上限，不越过目标；制动权保留
  // （可低于目标直至 0，同线速度通道 bound_hi/bound_lo 模式）。
  const double bound_hi = std::max(0.0, target_w);
  const double bound_lo = std::min(0.0, target_w);
  w_output = std::clamp(w_output, bound_lo, bound_hi);

  last_angular_cmd_ = w_output;
  return w_output;
}

double PIDController::CalculateMean(const std::deque<double> & data)
{
  double sum = 0.0;
  for (const auto & value : data) {
    sum += value;
  }
  return sum / static_cast<double>(data.size());
}

double PIDController::CalculateVariance(const std::deque<double> & data, double mean)
{
  double sum_sq = 0.0;
  for (const auto & value : data) {
    double diff = value - mean;
    sum_sq += diff * diff;
  }
  return sum_sq / static_cast<double>(data.size());
}

int PIDController::getClosestWaypointID(
  const std::vector<TrajPoint> & traj,
  const geometry_msgs::msg::PoseStamped & pose,
  double ignore_dist,
  bool ignore_first)
{
  if (traj.empty()) {
    return -1;
  }
  if (traj.size() == 1) {
    return 0;
  }

  int closest_id = -1;

  if (ignore_dist < 0.0) {
    ignore_dist = 0.0;
  }

  double dx_goal = traj.back().pose.pose.position.x - pose.pose.position.x;
  double dy_goal = traj.back().pose.pose.position.y - pose.pose.position.y;
  double dist_to_goal = std::hypot(dx_goal, dy_goal);
  if (dist_to_goal < ignore_dist) {
    closest_id = static_cast<int>(traj.size()) - 1;
    return closest_id;
  }

  double min_dist = std::numeric_limits<double>::max();
  for (int i = 0; i < static_cast<int>(traj.size()); ++i) {
    double dx = traj.at(i).pose.pose.position.x - pose.pose.position.x;
    double dy = traj.at(i).pose.pose.position.y - pose.pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < min_dist) {
      min_dist = dist;
      closest_id = i;
    }
  }

  if (ignore_first) {
    closest_id += 1;
    if (closest_id >= static_cast<int>(traj.size())) {
      closest_id = static_cast<int>(traj.size()) - 1;
    }
  }

  return closest_id;
}

bool PIDController::pruneGlobalPlan(
  const geometry_msgs::msg::PoseStamped & global_pose,
  std::vector<TrajPoint> & traj,
  double dist_behind_robot)
{
  if (traj.empty()) {
    return true;
  }

  // traj is already in the global frame; global_pose is too, so no TF needed.
  const double dist_thresh_sq = dist_behind_robot * dist_behind_robot;

  auto it = traj.begin();
  auto erase_end = it;
  bool found = false;
  while (it != traj.end()) {
    double dx = global_pose.pose.position.x - it->pose.pose.position.x;
    double dy = global_pose.pose.position.y - it->pose.pose.position.y;
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
  if (erase_end != traj.begin()) {
    traj.erase(traj.begin(), erase_end);
  }
  return true;
}

bool PIDController::nearestTrajectorySpeed(
  const geometry_msgs::msg::PoseStamped & pose, double & traj_speed, double & traj_accel)
{
  if (controller_traj_.empty()) {
    return false;
  }
  nav2_core::TrajectoryPreview preview;
  if (!nav2_core::sampleTrajectoryPreview(
      controller_traj_, pose.pose.position.x, pose.pose.position.y,
      trajectory_time_valid_, traj_time_lookahead_, traj_speed_lookahead_, preview))
  {
    return false;
  }
  traj_speed = preview.velocity;
  traj_accel = preview.acceleration;
  return true;
}

bool PIDController::getLastTrajectoryTrackingTarget(
  double & velocity, double & acceleration)
{
  std::unique_lock<std::mutex> lock(plan_mutex_);
  if (!last_trajectory_tracking_target_valid_) {
    return false;
  }
  velocity = last_trajectory_tracking_target_v_;
  acceleration = last_trajectory_tracking_target_a_;
  return true;
}

bool PIDController::shouldRotateToPath(
  const geometry_msgs::msg::PoseStamped & carrot_pose, double & angle_to_path)
{
  angle_to_path = std::atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
  return std::fabs(angle_to_path) > M_PI / 6.0;  // 30 deg
}

void PIDController::rotateToHeading(
  double & linear_vel, double & angular_vel,
  const double & angle_to_path, const geometry_msgs::msg::Twist & curr_vel)
{
  linear_vel = 0.0;

  (void)curr_vel;
  angular_vel = AngularSpeedControl(curr_vel, angle_to_path);
}

}  // namespace nav2_pid_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_pid_controller::PIDController, nav2_core::Controller)
