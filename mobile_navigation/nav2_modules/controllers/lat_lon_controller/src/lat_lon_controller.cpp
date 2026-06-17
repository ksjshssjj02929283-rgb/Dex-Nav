/***********************************************************
 *
 * @file: lat_lon_controller.cpp
 * @brief: 纵横解耦 PID 跟踪 controller（nav2_core::Controller 插件实现）。
 *         算法逻辑（五阶段状态机 / 纵横 PID / 参数）完整移植自 dcm 蓝本，
 *         仅替换 ROS1 → ROS2 边界（接口 / 参数 / 日志 / 消息 / costmap / 轨迹消费）。
 * @author: Yang Haodong, Guo Zhanyu, Wu Maojia（原 dcm 版本）
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
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_util/node_utils.hpp"

#include "lat_lon_controller/lat_lon_controller.hpp"

namespace lat_lon_controller
{

void LatLonController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  auto node = parent.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  base_frame_ = costmap_ros_->getBaseFrameID();
  global_frame_ = costmap_ros_->getGlobalFrameID();

  using nav2_util::declare_parameter_if_not_declared;
  const std::string & p = plugin_name_;

  declare_parameter_if_not_declared(node, p + ".p_window", rclcpp::ParameterValue(0.5));
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
  declare_parameter_if_not_declared(node, p + ".controller_frequency", rclcpp::ParameterValue(20.0));

  // 嵌套 PID 配置（逐项 declare 为 name+".<pid>.<field>"）
  auto declare_pid = [&](const std::string & pid, const controller_common::PidConf & def) {
      declare_parameter_if_not_declared(
        node, p + "." + pid + ".integrator_enable",
        rclcpp::ParameterValue(def.integrator_enable));
      declare_parameter_if_not_declared(
        node, p + "." + pid + ".integrator_saturation_level",
        rclcpp::ParameterValue(def.integrator_saturation_level));
      declare_parameter_if_not_declared(
        node, p + "." + pid + ".kp", rclcpp::ParameterValue(def.kp));
      declare_parameter_if_not_declared(
        node, p + "." + pid + ".ki", rclcpp::ParameterValue(def.ki));
      declare_parameter_if_not_declared(
        node, p + "." + pid + ".kd", rclcpp::ParameterValue(def.kd));
      declare_parameter_if_not_declared(
        node, p + "." + pid + ".kaw", rclcpp::ParameterValue(def.kaw));
      declare_parameter_if_not_declared(
        node, p + "." + pid + ".saturation", rclcpp::ParameterValue(def.output_saturation_level));
    };

  auto get_pid = [&](const std::string & pid, controller_common::PidConf & conf) {
      node->get_parameter(p + "." + pid + ".integrator_enable", conf.integrator_enable);
      node->get_parameter(
        p + "." + pid + ".integrator_saturation_level", conf.integrator_saturation_level);
      node->get_parameter(p + "." + pid + ".kp", conf.kp);
      node->get_parameter(p + "." + pid + ".ki", conf.ki);
      node->get_parameter(p + "." + pid + ".kd", conf.kd);
      node->get_parameter(p + "." + pid + ".kaw", conf.kaw);
      node->get_parameter(p + "." + pid + ".saturation", conf.output_saturation_level);
    };

  // 默认值与 dcm 蓝本一致
  controller_common::PidConf station_def;
  station_def.integrator_enable = true;
  station_def.integrator_saturation_level = 0.3;
  station_def.kp = 1.0; station_def.ki = 0.01; station_def.kd = 0.10;

  controller_common::PidConf reverse_def = station_def;
  reverse_def.integrator_saturation_level = 0.5;

  controller_common::PidConf angular_def = station_def;
  angular_def.integrator_saturation_level = 0.5;

  declare_pid("high_speed_station_pid_conf", station_def);
  declare_pid("low_speed_station_pid_conf", station_def);
  declare_pid("high_speed_pid_conf", station_def);
  declare_pid("low_speed_pid_conf", station_def);
  declare_pid("reverse_station_pid_conf", reverse_def);
  declare_pid("reverse_speed_pid_conf", reverse_def);
  declare_pid("angular_station_pid_conf", angular_def);

  // 读取标量参数
  node->get_parameter(p + ".p_window", p_window_);
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

  // 读取嵌套 PID
  get_pid("high_speed_station_pid_conf", high_speed_station_pid_conf_);
  get_pid("low_speed_station_pid_conf", low_speed_station_pid_conf_);
  get_pid("high_speed_pid_conf", high_speed_pid_conf_);
  get_pid("low_speed_pid_conf", low_speed_pid_conf_);
  get_pid("reverse_station_pid_conf", reverse_station_pid_conf_);
  get_pid("reverse_speed_pid_conf", reverse_speed_pid_conf_);
  get_pid("angular_station_pid_conf", angular_station_pid_conf_);

  // 控制周期：优先用全局 controller_frequency（move_base 等价），否则用插件参数默认
  double controller_frequency = 20.0;
  if (!node->get_parameter("controller_frequency", controller_frequency)) {
    node->get_parameter(p + ".controller_frequency", controller_frequency);
  }
  if (controller_frequency <= 0.0) {
    controller_frequency = 20.0;
  }
  d_t_ = 1.0 / controller_frequency;

  station_pid_controller_.Init(low_speed_station_pid_conf_);
  speed_pid_controller_.Init(low_speed_pid_conf_);
  angular_station_pid_controller_.Init(angular_station_pid_conf_);

  initialized_ = true;
  RCLCPP_INFO(logger_, "LatLonController (%s) configured.", plugin_name_.c_str());
}

void LatLonController::activate()
{
  RCLCPP_INFO(logger_, "Activating LatLonController: %s", plugin_name_.c_str());
}

void LatLonController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating LatLonController: %s", plugin_name_.c_str());
}

void LatLonController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up LatLonController: %s", plugin_name_.c_str());
  controller_traj_.clear();
}

void LatLonController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (speed_limit <= 0.0) {
    // 0 或负值视为取消限制
    speed_limit_ = std::numeric_limits<double>::infinity();
    return;
  }
  if (percentage) {
    speed_limit_ = max_v_ * speed_limit / 100.0;
  } else {
    speed_limit_ = speed_limit;
  }
}

void LatLonController::cacheTrajectory(const std::vector<TrajPoint> & input)
{
  std::unique_lock<std::mutex> lock(plan_mutex_);

  controller_traj_.clear();
  controller_traj_.reserve(input.size());

  // 将轨迹变换到 map（global）frame，与 dcm 蓝本一致
  for (const auto & tp : input) {
    TrajPoint out = tp;
    try {
      geometry_msgs::msg::PoseStamped ps = tp.pose;
      if (!ps.header.frame_id.empty() && ps.header.frame_id != global_frame_) {
        tf_->transform(ps, ps, global_frame_, tf2::durationFromSec(0.1));
      }
      out.pose = ps;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(logger_, "LatLonController: failed to transform plan pose: %s", ex.what());
      out.pose = tp.pose;
    }
    controller_traj_.push_back(out);
  }

  if (controller_traj_.empty()) {
    return;
  }

  plan_index_ = std::min(1, static_cast<int>(controller_traj_.size()) - 1);

  const auto & back = controller_traj_.back().pose.pose;
  const double back_yaw = tf2::getYaw(back.orientation);
  if (goal_x_ != back.position.x || goal_y_ != back.position.y || goal_yaw_ != back_yaw) {
    goal_x_ = back.position.x;
    goal_y_ = back.position.y;
    goal_yaw_ = back_yaw;
  }
}

void LatLonController::setPlan(const nav_msgs::msg::Path & path)
{
  if (!initialized_) {
    RCLCPP_ERROR(logger_, "LatLonController not configured, call configure() first.");
    return;
  }
  if (path.poses.empty()) {
    return;
  }

  // 几何路径无速度信息：速度回退到 max_v_
  std::vector<TrajPoint> input;
  input.reserve(path.poses.size());
  for (const auto & ps : path.poses) {
    TrajPoint tp;
    tp.pose = ps;
    if (tp.pose.header.frame_id.empty()) {
      tp.pose.header = path.header;
    }
    tp.speed_limit = max_v_;
    input.push_back(tp);
  }
  cacheTrajectory(input);
}

void LatLonController::setTrajectory(const nav_interfaces::msg::RobotTrajectory & trajectory)
{
  if (!initialized_) {
    RCLCPP_ERROR(logger_, "LatLonController not configured, call configure() first.");
    return;
  }
  if (trajectory.points.empty()) {
    return;
  }

  // 带速度时空轨迹：longitudinal_velocity 作为每点期望速度（dcm PathPose.speed_limit）。
  // path_kappa（曲率）随点缓存，pose 作为几何输入。
  std::vector<TrajPoint> input;
  input.reserve(trajectory.points.size());
  for (const auto & pt : trajectory.points) {
    TrajPoint tp;
    tp.pose = pt.pose;
    if (tp.pose.header.frame_id.empty()) {
      tp.pose.header = trajectory.header;
    }
    // 期望速度优先取 longitudinal_velocity；为 0 时回退 speed_hint，再回退 max_v_
    double v = std::fabs(pt.longitudinal_velocity);
    if (v <= 0.0) {
      v = (pt.speed_hint > 0.0f) ? static_cast<double>(pt.speed_hint) : max_v_;
    }
    tp.speed_limit = v;
    input.push_back(tp);
  }
  cacheTrajectory(input);
}

bool LatLonController::isGoalReached()
{
  return goal_reached_;
}

geometry_msgs::msg::TwistStamped LatLonController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = base_frame_;
  cmd_vel.header.stamp = clock_ ? clock_->now() : rclcpp::Time();
  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.angular.z = 0.0;

  if (!initialized_) {
    RCLCPP_ERROR(logger_, "LatLonController has not been configured");
    return cmd_vel;
  }

  std::unique_lock<std::mutex> lock(plan_mutex_);

  if (controller_traj_.empty()) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "LatLonController: empty trajectory");
    return cmd_vel;
  }

  // 当前位姿由入参 pose 提供（global/map frame），替代 dcm 内部 getRobotPose + tf 到 map。
  current_ps_ = pose;

  // 倒车：将朝向旋转 180 度（保留 dcm 逻辑）
  if (reverse_enabled_) {
    double reverse_yaw = tf2::getYaw(current_ps_.pose.orientation) + M_PI;
    regularizeAngle(reverse_yaw);
    tf2::Quaternion q;
    q.setRPY(0, 0, reverse_yaw);
    current_ps_.pose.orientation = tf2::toMsg(q);
  }

  // 裁剪机器人后方的轨迹
  pruneGlobalPlan(current_ps_, controller_traj_, 2.0);

  // 用入参 velocity 构造合成 odom（替代 base_local_planner::OdometryHelperRos）
  nav_msgs::msg::Odometry base_odom;
  base_odom.twist.twist = velocity;

  plan_index_ = getClosestWaypointID(controller_traj_, current_ps_, 0.1, true);
  if (plan_index_ < 0) {
    RCLCPP_WARN(logger_, "LatLonController: Failed to get closest waypoint");
    return cmd_vel;
  }

  desired_linear_vel_ = controller_traj_[plan_index_].speed_limit;
  // 与 setSpeedLimit 上限取严
  desired_linear_vel_ = std::min(desired_linear_vel_, speed_limit_);

  if (reverse_enabled_) {
    desired_linear_vel_ = -desired_linear_vel_;
  }

  // 最近点为目标点（map frame）
  target_ps_ = controller_traj_.at(plan_index_).pose;

  double theta_to_goal;  // 机器人与目标点的角度
  double theta_traj;     // 轨迹的角度
  double theta_dir;      // 期望角度

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

  double err_x, err_y;  // desired x, y in base frame
  double err_theta;

  // transform target from map into base_frame
  geometry_msgs::msg::PoseStamped target_in_base;
  geometry_msgs::msg::PoseStamped target_query = target_ps_;
  target_query.header.stamp = rclcpp::Time(0);
  try {
    tf_->transform(target_query, target_in_base, base_frame_, tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(logger_, "LatLonController: target transform to base failed: %s", ex.what());
    return cmd_vel;
  }
  err_x = target_in_base.pose.position.x;
  err_y = target_in_base.pose.position.y;

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

  // 计算距离误差均值 / 方差
  longitude_errors_.push_back(std::fabs(err_x));
  while (longitude_errors_.size() > 20) {
    longitude_errors_.pop_front();
  }
  double longitude_mean = std::numeric_limits<double>::max();
  double longitude_var = std::numeric_limits<double>::max();
  if (longitude_errors_.size() >= 20) {
    longitude_mean = calculateMean(longitude_errors_);
    longitude_var = calculateVariance(longitude_errors_, longitude_mean);
  }
  (void)longitude_var;

  theta_errors_.push_back(std::fabs(err_theta));
  while (theta_errors_.size() > 50) {
    theta_errors_.pop_front();
  }
  double theta_mean = std::numeric_limits<double>::max();
  double theta_var = std::numeric_limits<double>::max();
  if (theta_errors_.size() >= 50) {
    theta_mean = calculateMean(theta_errors_);
    theta_var = calculateVariance(theta_errors_, theta_mean);
  }
  (void)theta_var;

  // 状态维护
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

  // 距离目标点 0.25m 时，开始调整角度
  if (curr_controller_stage_ == APPROACHING && dist_to_goal < 0.25) {
    curr_controller_stage_ = ADJUST_HEADING;
  }
  // 调整角度后，开始调整距离
  if (curr_controller_stage_ == ADJUST_HEADING) {
    if (theta_mean < o_precision_) {
      curr_controller_stage_ = ADJUST_DISTANCE;
    }
  }
  // 调整距离后，若横向误差仍大，回到调整角度
  if (curr_controller_stage_ == ADJUST_DISTANCE) {
    if (longitude_mean < long_precision_ && std::fabs(err_y) > lat_precision_) {
      curr_controller_stage_ = ADJUST_HEADING;
    }
  }

  if (longitude_mean < long_precision_ * 0.5 &&
    std::fabs(err_x) < long_precision_ && std::fabs(err_y) < lat_precision_ * 0.75)
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
      case ADJUST_HEADING:
        RCLCPP_WARN(logger_, "controller stage: ADJUST_HEADING");
        break;
      case ADJUST_DISTANCE:
        RCLCPP_WARN(logger_, "controller stage: ADJUST_DISTANCE");
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
    rotateToHeading(cmd_vel.twist.linear.x, cmd_vel.twist.angular.z, angle_to_heading, base_odom);
  } else {
    switch (curr_controller_stage_) {
      case FINISH: {
          cmd_vel.twist.linear.x = 0.0;
          cmd_vel.twist.angular.z = 0.0;
          break;
        }
      case APPROACHING: {
          cmd_vel.twist.linear.x = linearSpeedControl(base_odom, err_x);
          cmd_vel.twist.angular.z = angularSpeedControl(base_odom, err_theta);
          break;
        }
      case ADJUST_HEADING: {
          cmd_vel.twist.linear.x = linearSpeedControl(base_odom, err_x) * 0.75;
          cmd_vel.twist.angular.z = angularSpeedControl(base_odom, err_theta);
          break;
        }
      case ADJUST_DISTANCE: {
          cmd_vel.twist.linear.x = linearSpeedControl(base_odom, err_x) * 0.75;
          cmd_vel.twist.angular.z = angularSpeedControl(base_odom, err_theta);
          break;
        }
      case ROTATING_TO_GOAL: {
          cmd_vel.twist.linear.x = linearSpeedControl(base_odom, err_x) * 0.25;
          cmd_vel.twist.angular.z = angularSpeedControl(base_odom, err_theta);
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

double LatLonController::linearSpeedControl(
  const nav_msgs::msg::Odometry & base_odometry, double e_dist)
{
  double curr_vel = base_odometry.twist.twist.linear.x;
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

double LatLonController::angularSpeedControl(
  const nav_msgs::msg::Odometry & /*base_odometry*/, double err_theta)
{
  regularizeAngle(err_theta);

  double vel_angular_output = angular_station_pid_controller_.Control(err_theta, d_t_);

  if (!std::isfinite(vel_angular_output)) {
    vel_angular_output = 0.0;
  }

  vel_angular_output = std::clamp(vel_angular_output, min_w_, max_w_);

  return vel_angular_output;
}

double LatLonController::calculateMean(const std::deque<double> & data)
{
  double sum = 0.0;
  for (const auto & value : data) {
    sum += value;
  }
  return sum / static_cast<double>(data.size());
}

double LatLonController::calculateVariance(const std::deque<double> & data, double mean)
{
  double sum_sq = 0.0;
  for (const auto & value : data) {
    double diff = value - mean;
    sum_sq += diff * diff;
  }
  return sum_sq / static_cast<double>(data.size());
}

int LatLonController::getClosestWaypointID(
  const std::vector<TrajPoint> & traj,
  const geometry_msgs::msg::PoseStamped & pose,
  double ignore_dist,
  bool ignore_first)
{
  if (traj.size() < 2) {  // 至少两个点（start 和 goal）
    return -1;
  }

  int closest_id = -1;

  if (ignore_dist < 0.0) {
    ignore_dist = 0.0;
  }

  // 若距终点足够近，直接返回最后一个点
  double dx_goal = traj.back().pose.pose.position.x - pose.pose.position.x;
  double dy_goal = traj.back().pose.pose.position.y - pose.pose.position.y;
  double dist_to_goal = std::sqrt(dx_goal * dx_goal + dy_goal * dy_goal);
  if (dist_to_goal < ignore_dist) {
    return static_cast<int>(traj.size()) - 1;
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

bool LatLonController::pruneGlobalPlan(
  const geometry_msgs::msg::PoseStamped & global_pose,
  std::vector<TrajPoint> & traj,
  double dist_behind_robot)
{
  if (traj.empty()) {
    return true;
  }

  // 轨迹已缓存于 global_frame_，global_pose 也在 global_frame_，无需再做 tf 变换
  double dist_thresh_sq = dist_behind_robot * dist_behind_robot;

  std::size_t erase_end = 0;
  bool found = false;
  for (std::size_t i = 0; i < traj.size(); ++i) {
    double dx = global_pose.pose.position.x - traj[i].pose.pose.position.x;
    double dy = global_pose.pose.position.y - traj[i].pose.pose.position.y;
    double dist_sq = dx * dx + dy * dy;
    if (dist_sq < dist_thresh_sq) {
      erase_end = i;
      found = true;
      break;
    }
  }

  if (!found) {
    return false;
  }

  if (erase_end > 0) {
    traj.erase(traj.begin(), traj.begin() + erase_end);
  }
  return true;
}

bool LatLonController::shouldRotateToPath(
  const geometry_msgs::msg::PoseStamped & carrot_pose, double & angle_to_path)
{
  angle_to_path = std::atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
  return std::fabs(angle_to_path) > M_PI / 6.0;  // 30°
}

void LatLonController::rotateToHeading(
  double & linear_vel, double & angular_vel,
  const double & angle_to_path, const nav_msgs::msg::Odometry & curr_odom)
{
  linear_vel = 0.0;
  const double & dt = d_t_;
  const double min_feasible_angular_speed = curr_odom.twist.twist.angular.z - 2.0 * dt;
  const double max_feasible_angular_speed = curr_odom.twist.twist.angular.z + 2.0 * dt;
  angular_vel = angularSpeedControl(curr_odom, angle_to_path);
  angular_vel = std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);
}

}  // namespace lat_lon_controller

PLUGINLIB_EXPORT_CLASS(lat_lon_controller::LatLonController, nav2_core::Controller)
