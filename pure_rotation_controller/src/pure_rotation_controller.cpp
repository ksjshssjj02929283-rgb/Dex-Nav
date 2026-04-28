/***********************************************************
 *
 * @file: pure_rotation_controller.cpp
 * @breif: Contains the Proportional–Integral–Derivative (PID) controller local planner class
 * @author: Yang Haodong, Guo Zhanyu, Wu Maojia
 * @update: 2023-10-1
 * @version: 1.1
 *
 * Copyright (c) 2023，Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#include <cmath>

#include <pluginlib/class_list_macros.h>

#include "pure_rotation_controller/pure_rotation_controller.h"
#include "nav_util/path_utils.hpp"

PLUGINLIB_EXPORT_CLASS(pure_rotation_controller::PureRotationController, nav_core::BaseController)

namespace pure_rotation_controller
{
/**
 * @brief Construct a new PureRotationController object
 */
PureRotationController::PureRotationController()
  : initialized_(false), tf_(nullptr), costmap_ros_(nullptr), goal_reached_(false), plan_index_(0)
{
}

/**
 * @brief Construct a new PureRotationController object
 */
PureRotationController::PureRotationController(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  initialize(name, tf, costmap_ros);
}

/**
 * @brief Destroy the PureRotationController object
 */
PureRotationController::~PureRotationController()
{
}

/**
 * @brief Initialization of the local planner
 * @param name        the name to give this instance of the trajectory planner
 * @param tf          a pointer to a transform listener
 * @param costmap_ros the cost map to use for assigning costs to trajectories
 */
void PureRotationController::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!initialized_)
  {
    initialized_ = true;
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    ros::NodeHandle nh = ros::NodeHandle("~/" + name);

    nh.param("long_precision", long_precision_, 0.02);
    nh.param("lat_precision", lat_precision_, 0.04);
    nh.param("o_precision", o_precision_, 0.02);

    nh.param("max_v", max_v_, 0.5);
    nh.param("min_v", min_v_, 0.0);
    nh.param("max_v_inc", max_v_inc_, 0.5);

    nh.param("max_w", max_w_, 1.57);
    nh.param("min_w", min_w_, 0.0);
    nh.param("max_w_inc", max_w_inc_, 1.57);

    nh.param("station_pid_conf/integrator_enable", station_pid_conf_.integrator_enable, true);
    nh.param("station_pid_conf/integrator_saturation_level", station_pid_conf_.integrator_saturation_level, 0.3);
    nh.param("station_pid_conf/kp", station_pid_conf_.kp, 1.00);
    nh.param("station_pid_conf/ki", station_pid_conf_.ki, 0.01);
    nh.param("station_pid_conf/kd", station_pid_conf_.kd, 0.10);

    nh.param("speed_pid_conf/integrator_enable", speed_pid_conf_.integrator_enable, true);
    nh.param("speed_pid_conf/integrator_saturation_level", speed_pid_conf_.integrator_saturation_level, 0.3);
    nh.param("speed_pid_conf/kp", speed_pid_conf_.kp, 1.00);
    nh.param("speed_pid_conf/ki", speed_pid_conf_.ki, 0.01);
    nh.param("speed_pid_conf/kd", speed_pid_conf_.kd, 0.10);

    nh.param("angular_pid_conf/integrator_enable", angular_pid_conf_.integrator_enable, true);
    nh.param("angular_pid_conf/integrator_saturation_level", angular_pid_conf_.integrator_saturation_level, 0.5);
    nh.param("angular_pid_conf/kp", angular_pid_conf_.kp, 1.00);
    nh.param("angular_pid_conf/ki", angular_pid_conf_.ki, 0.01);
    nh.param("angular_pid_conf/kd", angular_pid_conf_.kd, 0.10);

    nh.param("enable_lon_adjust_", enable_lon_adjust_, false);

    double controller_freqency;
    nh.getParam("/move_base/controller_frequency", controller_freqency);
    d_t_ = 1 / controller_freqency;

    odom_helper_ = new base_local_planner::OdometryHelperRos("/odom");
    target_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/target_pose", 10);
    desired_vel_pub_  = nh.advertise<std_msgs_stamped::Float64Stamped>("/move_base/desired_linear_vel", 10);

    station_pid_controller_.Init(station_pid_conf_);
    speed_pid_controller_.Init(speed_pid_conf_);

    angular_pid_controller_.Init(angular_pid_conf_);
    // angular_speed_pid_controller_.Init(angular_speed_pid_conf_);

    ROS_INFO("PID planner initialized!");
  }
  else
    ROS_WARN("PID planner has already been initialized.");
}

/**
 * @brief  Set the plan that the controller is following
 * @param orig_global_plan the plan to pass to the controller
 * @return  true if the plan was updated successfully, else false
 */
bool PureRotationController::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }
  if (orig_global_plan.empty()) {
    return false;
  }

  // ROS_INFO("PureRotationController : Got new plan");
  std::unique_lock<std::mutex> lock(plan_mutex_, std::adopt_lock);
  // set new plan
  global_plan_.clear();
  global_plan_ = orig_global_plan;

  // reset plan parameters
  plan_index_ = std::min(1, (int)global_plan_.size() - 1);

  if (goal_x_ != global_plan_.back().pose.position.x ||
      goal_y_ != global_plan_.back().pose.position.y ||
      goal_yaw_ != tf2::getYaw(global_plan_.back().pose.orientation))
  {
    goal_x_ = global_plan_.back().pose.position.x;
    goal_y_ = global_plan_.back().pose.position.y;
    goal_yaw_ = tf2::getYaw(global_plan_.back().pose.orientation);;
    // goal_reached_ = false;

    theta_errors_.clear();
    longitude_errors_.clear();
  }

  return true;
}

bool PureRotationController::setPlan(const dcm_msgs::Path& dcm_path)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  if (dcm_path.path_poses.empty()) {
    return false;
  }

  // ROS_INFO("PureRotationController : Got new plan");
  std::unique_lock<std::mutex> lock(plan_mutex_, std::adopt_lock);
  // set new plan
  controller_path_ = dcm_path;

  // transform path into map frame
  for (auto& path_pose : controller_path_.path_poses) {
    geometry_msgs::PoseStamped ps;
    ps.header = path_pose.header;
    ps.pose = path_pose.pose;
    tf_->transform(ps, ps, "map");
    path_pose.header = ps.header;
    path_pose.pose = ps.pose;
  }

  // reset plan parameters
  plan_index_ = std::min(1, (int)controller_path_.path_poses.size() - 1);


  {
    goal_x_ = controller_path_.path_poses.back().pose.position.x;
    goal_y_ = controller_path_.path_poses.back().pose.position.y;
    goal_yaw_ = tf2::getYaw(controller_path_.path_poses.back().pose.orientation);;
    // goal_reached_ = false;

    theta_errors_.clear();
    longitude_errors_.clear();
  }

  return true;
}

/**
 * @brief  Check if the goal pose has been achieved
 * @return True if achieved, false otherwise
 */
bool PureRotationController::isGoalReached()
{
  // if (!initialized_)
  // {
  //   ROS_ERROR("PID planner has not been initialized");
  //   return false;
  // }

  // if (goal_reached_)
  // {
  //   ROS_INFO("GOAL Reached!");
  //   return true;
  // }
  return false;
}

/**
 * @brief Given the current position, orientation, and velocity of the robot, compute the velocity commands
 * @param cmd_vel will be filled with the velocity command to be passed to the robot base
 * @return  true if a valid trajectory was found, else false
 */
bool PureRotationController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (!initialized_)
  {
    ROS_ERROR("PID planner has not been initialized");
    return false;
  }

  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;

  // if (goal_reached_) {
  //   return true;
  // }

  assert(!controller_path_.path_poses.empty());
  // current pose
  geometry_msgs::PoseStamped current_ps_odom;
  if (!costmap_ros_->getRobotPose(current_ps_odom)) {
    ROS_WARN("Rotation Controller : Failed to get robot pose");
    return false;
  }

  // PID的轨迹是在map坐标系下
  // transform into map
  tf_->transform(current_ps_odom, current_ps_, "map");

  std::unique_lock<std::mutex> lock(plan_mutex_, std::adopt_lock);
  // 从轨迹中找到距离机器人最近的点

  // odometry observation - getting robot velocities in robot frame
  nav_msgs::Odometry base_odom;
  odom_helper_->getOdom(base_odom);

  double vel = std::hypot(base_odom.twist.twist.linear.x, base_odom.twist.twist.linear.y);

  plan_index_ = nav_util::getClosestWaypointID(controller_path_, current_ps_, 0.1, true);
  if (plan_index_ < 0) {
    ROS_WARN("PureRotationController : Failed to get closest waypoint");
    return false;
  }
  // std::cout << "plan_index_: " << plan_index_ << std::endl;

  desired_linear_vel_ = controller_path_.path_poses[plan_index_].speed_limit;
  // std::cout << "pid: desired_linear_vel_: " << desired_linear_vel_ << std::endl;

  if (reverse_enabled_) {
    desired_linear_vel_ = -desired_linear_vel_;
  }
  // 最近点为目标点（in map frame）
  target_ps_.header = controller_path_.path_poses.at(plan_index_).header;
  target_ps_.pose = controller_path_.path_poses.at(plan_index_).pose;

  double err_x, err_y;  // desired x, y in base frame
  double err_theta;

  // transform from map into base_frame
  geometry_msgs::PoseStamped target_in_base;
  target_ps_.header.stamp = ros::Time(0);
  tf_->transform(target_ps_, target_in_base, costmap_ros_->getBaseFrameID());
  // 如果是倒车的情况，target_in_base.x是负的，所以计算的速度也是负的
  err_x = target_in_base.pose.position.x;
  err_y = target_in_base.pose.position.y;
  
  // current angle
  double theta_rob = tf2::getYaw(current_ps_.pose.orientation);  // [-pi, pi]

  err_theta = std::fmod(goal_yaw_ - theta_rob + M_PI, 2 * M_PI) - M_PI;

  // std::cout << "goal_yaw_: " << goal_yaw_ << " theta_rob: " << theta_rob << " err_theta： " << err_theta << std::endl;
  regularizeAngle(err_theta);
  // std::cout << "err_x: " << err_x << ", err_y: " << err_y << ", err_theta: " << err_theta << std::endl;

  cmd_vel.angular.z = AngularSpeedControl(base_odom, err_theta);
  if (enable_lon_adjust_) {
    cmd_vel.linear.x = LinearSpeedControl(base_odom, err_x) * 0.25;
  } else {
    cmd_vel.linear.x = 0.0;
  }

  // publish next target_ps_ pose
  target_pose_pub_.publish(target_ps_);
  publishDesiredVelocity();
  return true;
}

void PureRotationController::publishDesiredVelocity()
{
  std_msgs_stamped::Float64Stamped desired_vel_;
  desired_vel_.header.stamp = ros::Time::now();
  desired_vel_.data = desired_linear_vel_;
  desired_vel_pub_.publish(desired_vel_);
}

/**
 * @brief PID controller in linear
 * @param base_odometry odometry of the robot, to get velocity
 * @param err_x         desired x in body frame
 * @param err_y         desired y in body frame
 * @return  linear velocity
 */
double PureRotationController::LinearSpeedControl(const nav_msgs::Odometry& base_odometry, double e_dist)
{
  // std::cout << "desired_linear_vel_ : " << desired_linear_vel_ << std::endl;
  double curr_vel = base_odometry.twist.twist.linear.x;

  // std::cout << "curr_vel: " << curr_vel << std::endl;
  double speed_error = desired_linear_vel_ - curr_vel;
  // std::cout << "speed_error: " << speed_error << std::endl;

  double speed_offset =
    station_pid_controller_.Control(e_dist, d_t_);
  // std::cout << "speed_offset: " << speed_offset << std::endl;
  // if (enable_leadlag) {
  //   speed_offset = station_leadlag_controller_.Control(speed_offset, ts);
  // }
  double speed_controller_input = speed_offset + speed_error;
  // std::cout << "speed_controller_input: " << speed_controller_input << std::endl;
  // double speed_controller_input_limited = 
  //                       std::clamp(speed_controller_input, -desired_linear_vel_,
  //                                  desired_linear_vel_);
  // std::cout << "speed_controller_input_limited: " << speed_controller_input_limited << std::endl;
  double acceleration_cmd_closeloop = 0.0;

  // Step: 4. 计算加速度 (速度是加速度的积分，所以得到的是加速度误差)
  // std::cout << "d_t_ : " << d_t_ << std::endl;
  acceleration_cmd_closeloop =
      speed_pid_controller_.Control(speed_controller_input, d_t_);
  // std::cout << "acceleration_cmd_closeloop: " << acceleration_cmd_closeloop << std::endl;
  double acceleration_cmd_closeloop_limited = 
                        std::clamp(acceleration_cmd_closeloop, -max_v_inc_,
                                   max_v_inc_);
  // std::cout << "acceleration_cmd_closeloop_limited: " << acceleration_cmd_closeloop_limited << std::endl;
  double v_output = curr_vel + acceleration_cmd_closeloop_limited * d_t_;

  if (!std::isfinite(v_output)) {
    v_output = 0.0;
  }
  if (!reverse_enabled_) {
    v_output = std::clamp(v_output, min_v_, desired_linear_vel_ * 1.5);
  } else {
    v_output = std::clamp(v_output, desired_linear_vel_ * 1.5, -min_v_);
  }
  // std::cout << "v_output: " << v_output << std::endl << std::endl;
  return v_output;
}


double PureRotationController::AngularSpeedControl(const nav_msgs::Odometry& base_odometry, double err_theta)
{
  regularizeAngle(err_theta);

  double curr_vel_angular = base_odometry.twist.twist.angular.z;

  double vel_angular_output =
    angular_pid_controller_.Control(err_theta, d_t_);

  if (!std::isfinite(vel_angular_output)) {
    vel_angular_output = 0.0;
  }

  vel_angular_output = std::clamp(vel_angular_output, min_w_, max_w_);

  return vel_angular_output;
}

double PureRotationController::CalculateMean(const std::deque<double>& data)
{
  double sum = 0.0;
  for (const auto& value : data) {
    sum += value;
  }
  double avg = sum / static_cast<double>(data.size());
  return avg;
}

double PureRotationController::CalculateVariance(const std::deque<double>& data, double mean)
{
  double sumSquaredDifferences = 0.0;
  for (const auto& value : data) {
    double difference = value - mean;
    sumSquaredDifferences += difference * difference;
  }
  double variance = sumSquaredDifferences / static_cast<double>(data.size());
  return variance;
}

}  // namespace pure_rotation_controller
