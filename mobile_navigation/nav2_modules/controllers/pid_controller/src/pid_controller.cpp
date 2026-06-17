// Copyright (c) 2023 Yang Haodong, Guo Zhanyu, Wu Maojia (original ROS1 dcm_navigation)
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

#include "pid_controller/pid_controller.hpp"

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

namespace pid_controller
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

  declare_parameter_if_not_declared(node, p + ".max_v", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, p + ".min_v", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(node, p + ".max_v_inc", rclcpp::ParameterValue(0.5));

  declare_parameter_if_not_declared(node, p + ".max_w", rclcpp::ParameterValue(1.57));
  declare_parameter_if_not_declared(node, p + ".min_w", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(node, p + ".max_w_inc", rclcpp::ParameterValue(1.57));

  declare_parameter_if_not_declared(node, p + ".k_theta", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, p + ".switch_speed", rclcpp::ParameterValue(0.1));

  node->get_parameter(p + ".long_precision", long_precision_);
  node->get_parameter(p + ".lat_precision", lat_precision_);
  node->get_parameter(p + ".o_precision", o_precision_);

  node->get_parameter(p + ".max_v", max_v_);
  node->get_parameter(p + ".min_v", min_v_);
  node->get_parameter(p + ".max_v_inc", max_v_inc_);

  node->get_parameter(p + ".max_w", max_w_);
  node->get_parameter(p + ".min_w", min_w_);
  node->get_parameter(p + ".max_w_inc", max_w_inc_);

  node->get_parameter(p + ".k_theta", k_theta_);
  node->get_parameter(p + ".switch_speed", switch_speed_);

  // ---- nested PID configs: name + ".<pid>.<field>" ----
  auto declare_pid = [&](const std::string & pid, controller_common::PidConf & conf,
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

  station_pid_controller_.Init(low_speed_station_pid_conf_);
  speed_pid_controller_.Init(low_speed_pid_conf_);
  angular_station_pid_controller_.Init(angular_station_pid_conf_);

  RCLCPP_INFO(logger_, "PID controller \"%s\" configured.", plugin_name_.c_str());
}

void PIDController::activate()
{
  RCLCPP_INFO(logger_, "Activating PID controller \"%s\".", plugin_name_.c_str());
}

void PIDController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating PID controller \"%s\".", plugin_name_.c_str());
}

void PIDController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up PID controller \"%s\".", plugin_name_.c_str());
  std::unique_lock<std::mutex> lock(plan_mutex_);
  controller_traj_.clear();
}

void PIDController::cachePathAsTrajectory(const nav_msgs::msg::Path & path)
{
  controller_traj_.clear();
  controller_traj_.reserve(path.poses.size());

  for (const auto & ps_in : path.poses) {
    TrajPoint tp;
    tp.pose = ps_in;
    // transform geometry into the costmap global frame
    if (!tp.pose.header.frame_id.empty() && tp.pose.header.frame_id != global_frame_) {
      try {
        geometry_msgs::msg::PoseStamped ps_out;
        tf_->transform(tp.pose, ps_out, global_frame_, tf2::durationFromSec(0.1));
        tp.pose = ps_out;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(logger_, "setPlan: TF transform to %s failed: %s",
          global_frame_.c_str(), ex.what());
        tp.pose.header.frame_id = global_frame_;
      }
    } else {
      tp.pose.header.frame_id = global_frame_;
    }
    tp.speed_limit = 0.0;  // geometric path: no speed info
    tp.curvature = 0.0;
    controller_traj_.push_back(std::move(tp));
  }
}

void PIDController::setPlan(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    RCLCPP_WARN(logger_, "setPlan: received empty path");
    return;
  }

  std::unique_lock<std::mutex> lock(plan_mutex_);
  cachePathAsTrajectory(path);

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
  controller_traj_.clear();
  controller_traj_.reserve(trajectory.points.size());

  for (const auto & pt : trajectory.points) {
    TrajPoint tp;
    tp.pose = pt.pose;
    if (tp.pose.header.frame_id.empty()) {
      tp.pose.header = trajectory.header;
    }
    // transform geometry into the costmap global frame
    if (!tp.pose.header.frame_id.empty() && tp.pose.header.frame_id != global_frame_) {
      try {
        geometry_msgs::msg::PoseStamped ps_out;
        tf_->transform(tp.pose, ps_out, global_frame_, tf2::durationFromSec(0.1));
        tp.pose = ps_out;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(logger_, "setTrajectory: TF transform to %s failed: %s",
          global_frame_.c_str(), ex.what());
        tp.pose.header.frame_id = global_frame_;
      }
    } else {
      tp.pose.header.frame_id = global_frame_;
    }
    // per-point desired velocity and curvature from SpeedPlanner
    tp.speed_limit = pt.longitudinal_velocity;
    tp.curvature = pt.path_kappa;
    controller_traj_.push_back(std::move(tp));
  }

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

geometry_msgs::msg::TwistStamped PIDController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = base_frame_;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.angular.z = 0.0;

  std::unique_lock<std::mutex> lock(plan_mutex_);

  if (controller_traj_.empty()) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "computeVelocityCommands: no plan set");
    return cmd_vel;
  }

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

  plan_index_ = getClosestWaypointID(controller_traj_, current_ps_, 0.1, true);
  if (plan_index_ < 0) {
    RCLCPP_WARN(logger_, "computeVelocityCommands: failed to get closest waypoint");
    return cmd_vel;
  }

  desired_linear_vel_ = controller_traj_[plan_index_].speed_limit;
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

  // current heading
  double theta_rob = tf2::getYaw(current_ps_.pose.orientation);  // [-pi, pi]

  if (curr_controller_stage_ == ROTATING_TO_GOAL ||
    (std::fabs(err_x) < long_precision_ && std::fabs(err_y) < lat_precision_))
  {
    if (reverse_enabled_) {
      theta_rob -= M_PI;
      regularizeAngle(theta_rob);
    }
    err_theta = std::fmod(goal_yaw_ - theta_rob + M_PI, 2 * M_PI) - M_PI;
  } else {
    err_theta = std::fmod(theta_dir - theta_rob + M_PI, 2 * M_PI) - M_PI;
  }
  regularizeAngle(err_theta);

  // longitudinal error statistics (sliding window of 20)
  longitude_errors_.push_back(std::fabs(err_x));
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
  double theta_mean = std::numeric_limits<double>::max();
  if (theta_errors_.size() >= 50) {
    theta_mean = CalculateMean(theta_errors_);
    (void)CalculateVariance(theta_errors_, theta_mean);
  }

  double dist_to_goal = std::hypot(
    target_ps_.pose.position.x - current_ps_.pose.position.x,
    target_ps_.pose.position.y - current_ps_.pose.position.y);

  // Step: switch mode
  if (dist_to_goal > 0.2 || curr_controller_stage_ == FINISH ||
    (dist_to_goal > 0.05 && curr_controller_stage_ == ROTATING_TO_GOAL))
  {
    curr_controller_stage_ = APPROACHING;
    theta_errors_.clear();
    longitude_errors_.clear();
  }

  if (longitude_mean < long_precision_ &&
    std::fabs(err_x) < long_precision_ && std::fabs(err_y) < lat_precision_)
  {
    curr_controller_stage_ = ROTATING_TO_GOAL;
  }

  if (curr_controller_stage_ != prev_controller_stage_) {
    switch (curr_controller_stage_) {
      case FINISH:
        RCLCPP_WARN(logger_, "controller stage: FINISH");
        break;
      case APPROACHING:
        RCLCPP_WARN(logger_, "controller stage: APPROACHING");
        break;
      case ROTATING_TO_GOAL:
        RCLCPP_WARN(logger_, "controller stage: ROTATING_TO_GOAL");
        break;
      default:
        break;
    }
  }
  prev_controller_stage_ = curr_controller_stage_;

  double angle_to_heading;
  if (!reverse_enabled_ &&
    curr_controller_stage_ != ControllerStage::ROTATING_TO_GOAL &&
    shouldRotateToPath(target_in_base, angle_to_heading))
  {
    rotateToHeading(cmd_vel.twist.linear.x, cmd_vel.twist.angular.z, angle_to_heading, velocity);
  } else {
    switch (curr_controller_stage_) {
      case FINISH: {
          cmd_vel.twist.linear.x = 0.0;
          cmd_vel.twist.angular.z = 0.0;
          break;
        }
      case APPROACHING: {
          cmd_vel.twist.linear.x = LinearSpeedControl(velocity, err_x);
          cmd_vel.twist.angular.z = AngularSpeedControl(velocity, err_theta);
          break;
        }
      case ROTATING_TO_GOAL: {
          cmd_vel.twist.linear.x = LinearSpeedControl(velocity, err_x) * 0.25;
          cmd_vel.twist.angular.z = AngularSpeedControl(velocity, err_theta);
          break;
        }
      default:
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        break;
    }
  }

  return cmd_vel;
}

double PIDController::LinearSpeedControl(
  const geometry_msgs::msg::Twist & velocity, double e_dist)
{
  double curr_vel = velocity.linear.x;
  if (reverse_enabled_) {
    station_pid_controller_.SetPID(reverse_station_pid_conf_);
    speed_pid_controller_.SetPID(reverse_speed_pid_conf_);
  } else {
    if (curr_vel < switch_speed_) {
      station_pid_controller_.SetPID(low_speed_station_pid_conf_);
      speed_pid_controller_.SetPID(low_speed_pid_conf_);
    } else {
      station_pid_controller_.SetPID(high_speed_station_pid_conf_);
      speed_pid_controller_.SetPID(high_speed_pid_conf_);
    }
  }

  double speed_error = desired_linear_vel_ - curr_vel;

  double speed_offset = station_pid_controller_.Control(e_dist, d_t_);
  double speed_controller_input = speed_offset + speed_error;

  double acceleration_cmd_closeloop =
    speed_pid_controller_.Control(speed_controller_input, d_t_);
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

double PIDController::AngularSpeedControl(
  const geometry_msgs::msg::Twist & /*velocity*/, double err_theta)
{
  regularizeAngle(err_theta);

  double vel_angular_output = angular_station_pid_controller_.Control(err_theta, d_t_);

  if (!std::isfinite(vel_angular_output)) {
    vel_angular_output = 0.0;
  }

  vel_angular_output = std::clamp(vel_angular_output, min_w_, max_w_);

  return vel_angular_output;
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
  if (traj.size() < 2) {  // need at least start and goal
    return -1;
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

  const double & dt = d_t_;
  const double min_feasible_angular_speed = curr_vel.angular.z - 2.0 * dt;
  const double max_feasible_angular_speed = curr_vel.angular.z + 2.0 * dt;
  angular_vel = AngularSpeedControl(curr_vel, angle_to_path);
  angular_vel = std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);
}

}  // namespace pid_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(pid_controller::PIDController, nav2_core::Controller)
