/***********************************************************
 *
 * @file: straight_controller.cpp
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

#include "straight_controller/straight_controller.h"
#include "nav_util/geometry_utils.hpp"
#include "nav_util/path_utils.hpp"

PLUGINLIB_EXPORT_CLASS(straight_controller::StraightController, nav_core::BaseController)

namespace straight_controller
{
/**
 * @brief Construct a new StraightController object
 */
StraightController::StraightController()
  : initialized_(false), tf_(nullptr), costmap_ros_(nullptr), goal_reached_(false), closest_plan_index_(0)
{
}

/**
 * @brief Construct a new StraightController object
 */
StraightController::StraightController(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  initialize(name, tf, costmap_ros);
}

/**
 * @brief Destroy the StraightController object
 */
StraightController::~StraightController()
{
}

/**
 * @brief Initialization of the local planner
 * @param name        the name to give this instance of the trajectory planner
 * @param tf          a pointer to a transform listener
 * @param costmap_ros the cost map to use for assigning costs to trajectories
 */
void StraightController::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!initialized_)
  {
    initialized_ = true;
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    ros::NodeHandle nh = ros::NodeHandle("~/" + name);

    nh.param("long_precision", long_precision_, 0.02);
    nh.param("lat_precision", lat_precision_, 0.04);
    nh.param("o_precision", o_precision_, 0.5);

    nh.param("max_v", max_v_, 0.5);
    nh.param("min_v", min_v_, 0.0);
    nh.param("max_v_inc", max_v_inc_, 0.5);

    nh.param("max_w", max_w_, 1.57);
    nh.param("min_w", min_w_, 0.0);
    nh.param("max_w_inc", max_w_inc_, 1.57);

    // 前进后退路径的最长距离
    nh.param("max_path_length", max_path_length_, 0.5);

    nh.param("station_pid_conf/integrator_enable", station_pid_conf_.integrator_enable, true);
    nh.param("station_pid_conf/integrator_saturation_level", station_pid_conf_.integrator_saturation_level, 0.3);
    nh.param("station_pid_conf/kp", station_pid_conf_.kp, 1.00);
    nh.param("station_pid_conf/ki", station_pid_conf_.ki, 0.01);
    nh.param("station_pid_conf/kd", station_pid_conf_.kd, 0.10);

    nh.param("high_speed_pid_conf/integrator_enable", high_speed_pid_conf_.integrator_enable, true);
    nh.param("high_speed_pid_conf/integrator_saturation_level", high_speed_pid_conf_.integrator_saturation_level, 0.3);
    nh.param("high_speed_pid_conf/kp", high_speed_pid_conf_.kp, 1.00);
    nh.param("high_speed_pid_conf/ki", high_speed_pid_conf_.ki, 0.01);
    nh.param("high_speed_pid_conf/kd", high_speed_pid_conf_.kd, 0.10);

    nh.param("low_speed_pid_conf/integrator_enable", low_speed_pid_conf_.integrator_enable, true);
    nh.param("low_speed_pid_conf/integrator_saturation_level", low_speed_pid_conf_.integrator_saturation_level, 0.3);
    nh.param("low_speed_pid_conf/kp", low_speed_pid_conf_.kp, 1.00);
    nh.param("low_speed_pid_conf/ki", low_speed_pid_conf_.ki, 0.01);
    nh.param("low_speed_pid_conf/kd", low_speed_pid_conf_.kd, 0.10);

    nh.param("reverse_station_pid_conf/integrator_enable", reverse_station_pid_conf_.integrator_enable, true);
    nh.param("reverse_station_pid_conf/integrator_saturation_level", reverse_station_pid_conf_.integrator_saturation_level, 0.5);
    nh.param("reverse_station_pid_conf/kp", reverse_station_pid_conf_.kp, 1.00);
    nh.param("reverse_station_pid_conf/ki", reverse_station_pid_conf_.ki, 0.01);
    nh.param("reverse_station_pid_conf/kd", reverse_station_pid_conf_.kd, 0.10);

    nh.param("reverse_speed_pid_conf/integrator_enable", reverse_speed_pid_conf_.integrator_enable, true);
    nh.param("reverse_speed_pid_conf/integrator_saturation_level", reverse_speed_pid_conf_.integrator_saturation_level, 0.5);
    nh.param("reverse_speed_pid_conf/kp", reverse_speed_pid_conf_.kp, 1.00);
    nh.param("reverse_speed_pid_conf/ki", reverse_speed_pid_conf_.ki, 0.01);
    nh.param("reverse_speed_pid_conf/kd", reverse_speed_pid_conf_.kd, 0.10);

    nh.param("k_w_p", k_w_p_, 1.00);
    nh.param("k_w_i", k_w_i_, 0.01);
    nh.param("k_w_d", k_w_d_, 0.10);

    nh.param("switch_speed", switch_speed_, 0.1);

    double controller_freqency;
    nh.getParam("/move_base/controller_frequency", controller_freqency);
    d_t_ = 1 / controller_freqency;

    e_w_ = i_w_ = 0.0;

    odom_helper_ = new base_local_planner::OdometryHelperRos("/odom");
    target_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/target_pose", 10);
    desired_vel_pub_  = nh.advertise<std_msgs_stamped::Float64Stamped>("/move_base/desired_linear_vel", 10);

    station_pid_controller_.Init(station_pid_conf_);
    speed_pid_controller_.Init(low_speed_pid_conf_);

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
bool StraightController::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }
  if (orig_global_plan.empty()) {
    return false;
  }

  // ROS_INFO("StraightController : Got new plan");
  std::unique_lock<std::mutex> lock(plan_mutex_, std::adopt_lock);
  // set new plan
  global_plan_.clear();
  global_plan_ = orig_global_plan;

  // reset plan parameters
  closest_plan_index_ = std::min(1, (int)global_plan_.size() - 1);

  if (goal_x_ != global_plan_.back().pose.position.x ||
      goal_y_ != global_plan_.back().pose.position.y ||
      goal_yaw_ != tf2::getYaw(global_plan_.back().pose.orientation))
  {
    goal_x_ = global_plan_.back().pose.position.x;
    goal_y_ = global_plan_.back().pose.position.y;
    goal_yaw_ = tf2::getYaw(global_plan_.back().pose.orientation);
    goal_ps_ = global_plan_.back();
    // goal_reached_ = false;

    e_w_ = i_w_ = 0.0;
  }

  return true;
}

bool StraightController::setPlan(const dcm_msgs::Path& dcm_path)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  if (dcm_path.path_poses.empty()) {
    return false;
  }

  // 直线控制的路径不能过长，否则无法保证控制精度
  if (nav_util::computePathLength(dcm_path) > max_path_length_) {
    ROS_ERROR("[StraightController] : Path length is too long, straight planner will not control well, please modify your logic");
    return false;
  }

  // ROS_INFO("StraightController : Got new plan");
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
  closest_plan_index_ = std::min(1, (int)controller_path_.path_poses.size() - 1);


  {
    goal_x_ = controller_path_.path_poses.back().pose.position.x;
    goal_y_ = controller_path_.path_poses.back().pose.position.y;
    goal_yaw_ = tf2::getYaw(controller_path_.path_poses.back().pose.orientation);
    goal_ps_.header = controller_path_.path_poses.back().header;
    goal_ps_.pose = controller_path_.path_poses.back().pose;
    // goal_reached_ = false;

    e_w_ = i_w_ = 0.0;
  }

  return true;
}


/**
 * @brief  Check if the goal pose has been achieved
 * @return True if achieved, false otherwise
 */
bool StraightController::isGoalReached()
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
bool StraightController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (!initialized_)
  {
    ROS_ERROR("Straight controller has not been initialized");
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
    return false;
  }

  // 需要搞两个pose ，一个pose用于正常计算，一个pose用于计算倒车
  if (reverse_enabled_) {
    // 将current_ps_odom旋转180度
    double reverse_yaw = tf::getYaw(current_ps_odom.pose.orientation) + M_PI;
    regularizeAngle(reverse_yaw);
    current_ps_odom.pose.orientation = tf::createQuaternionMsgFromYaw(reverse_yaw);
  }

  pruneGlobalPlan(*tf_, current_ps_odom, controller_path_, 0.5);

  // PID的轨迹是在map坐标系下
  // transform into map
  tf_->transform(current_ps_odom, current_ps_, "map");

  std::unique_lock<std::mutex> lock(plan_mutex_, std::adopt_lock);
  // 从轨迹中找到距离机器人最近的点

  // odometry observation - getting robot velocities in robot frame
  nav_msgs::Odometry base_odom;
  odom_helper_->getOdom(base_odom);

  double vel = std::hypot(base_odom.twist.twist.linear.x, base_odom.twist.twist.linear.y);

  // 最近的点，用来计算速度的PID
  // 目标点，用来控制角度

  closest_plan_index_ = nav_util::getClosestWaypointID(controller_path_, current_ps_, 0.1, true);
  if (closest_plan_index_ < 0) {
    ROS_WARN("StraightController : Failed to get closest waypoint");
    return false;
  }
  // std::cout << "closest_plan_index_: " << closest_plan_index_ << std::endl;

  desired_linear_vel_ = controller_path_.path_poses[closest_plan_index_].speed_limit;
  if (reverse_enabled_) {
    desired_linear_vel_ = -desired_linear_vel_;
  }

  // 最近点为目标点（in map frame）
  closest_ps_.header = controller_path_.path_poses.at(closest_plan_index_).header;
  closest_ps_.pose = controller_path_.path_poses.at(closest_plan_index_).pose;

  double err_x, err_y;  // desired x, y in base frame
  double err_theta;

  // transform from map into base_frame
  geometry_msgs::PoseStamped closest_in_base;
  closest_ps_.header.stamp = ros::Time(0);
  tf_->transform(closest_ps_, closest_in_base, costmap_ros_->getBaseFrameID());
  // 如果是倒车的情况，target_in_base.x是负的，所以计算的速度也是负的
  err_x = closest_in_base.pose.position.x;
  err_y = closest_in_base.pose.position.y;
  
  // current angle
  double theta_rob = tf2::getYaw(current_ps_.pose.orientation);  // [-pi, pi]

  if (reverse_enabled_) {
    theta_rob -= M_PI;
    regularizeAngle(theta_rob);
  }
  err_theta = std::fmod(goal_yaw_ - theta_rob + M_PI, 2 * M_PI) - M_PI;

  // std::cout << "goal_yaw_: " << goal_yaw_ << " theta_rob: " << theta_rob << " err_theta： " << err_theta << std::endl;
  regularizeAngle(err_theta);
  // std::cout << "err_x: " << err_x << ", err_y: " << err_y << ", err_theta: " << err_theta << std::endl;

  err_theta = std::fmod(goal_yaw_ - theta_rob + M_PI, 2 * M_PI) - M_PI;
  regularizeAngle(err_theta);

  cmd_vel.linear.x = LinearSpeedControl(base_odom, err_x);
  cmd_vel.angular.z = AngularSpeedControl(base_odom, err_theta);

  // publish goal pose
  target_pose_pub_.publish(goal_ps_);
  publishDesiredVelocity();
  return true;
}

void StraightController::publishDesiredVelocity()
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
double StraightController::LinearSpeedControl(const nav_msgs::Odometry& base_odometry, double e_dist)
{
  // std::cout << "desired_linear_vel_ : " << desired_linear_vel_ << std::endl;
  double curr_vel = base_odometry.twist.twist.linear.x;

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


/**
 * @brief PID controller in angular
 * @param base_odometry odometry of the robot, to get velocity
 * @param err_theta       the error between the current and desired theta
 * @return  angular velocity
 */
double StraightController::AngularSpeedControl(const nav_msgs::Odometry& base_odometry, double err_theta)
{
  regularizeAngle(err_theta);

  double w_d = err_theta / d_t_;
  if (std::fabs(w_d) > max_w_)
    w_d = std::copysign(max_w_, w_d);

  double w = base_odometry.twist.twist.angular.z;
  double e_w = w_d - w;
  i_w_ += e_w * d_t_;
  double d_w = (e_w - e_w_) / d_t_;
  e_w_ = e_w;

  double w_inc = k_w_p_ * e_w + k_w_i_ * i_w_ + k_w_d_ * d_w;

  if (std::fabs(w_inc) > max_w_inc_)
    w_inc = std::copysign(max_w_inc_, w_inc);

  double w_cmd = w + w_inc;
  if (std::fabs(w_cmd) > max_w_)
    w_cmd = std::copysign(max_w_, w_cmd);
  else if (std::fabs(w_cmd) < min_w_)
    w_cmd = std::copysign(min_w_, w_cmd);

  double err_theta_abs = std::fabs(err_theta);
  if (err_theta_abs < M_PI / 8.0) {
    w_cmd *= 0.25;
  } else if (err_theta_abs < M_PI_4) {
    double factor = 1 - (M_PI_4 - err_theta_abs) / (M_PI_4 - M_PI / 8.0) * 0.25;
    w_cmd *= factor;
  }

  return w_cmd;
}


double StraightController::CalculateMean(const std::deque<double>& data)
{
  double sum = 0.0;
  for (const auto& value : data) {
    sum += value;
  }
  double avg = sum / static_cast<double>(data.size());
  return avg;
}

double StraightController::CalculateVariance(const std::deque<double>& data, double mean)
{
  double sumSquaredDifferences = 0.0;
  for (const auto& value : data) {
    double difference = value - mean;
    sumSquaredDifferences += difference * difference;
  }
  double variance = sumSquaredDifferences / static_cast<double>(data.size());
  return variance;
}


bool StraightController::pruneGlobalPlan(
        const tf2_ros::Buffer& tf,
        const geometry_msgs::PoseStamped& global_pose,
        dcm_msgs::Path& global_path,
        double dist_behind_robot)
{
  if (global_path.path_poses.empty())
    return true;

  try
  {
    // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
    geometry_msgs::TransformStamped global_to_plan_transform = tf.lookupTransform(global_path.path_poses.front().header.frame_id,
                                                                                  global_pose.header.frame_id,
                                                                                  ros::Time(0));
    geometry_msgs::PoseStamped robot;
    tf2::doTransform(global_pose, robot, global_to_plan_transform);
    
    double dist_thresh_sq = dist_behind_robot*dist_behind_robot;
    
    // iterate plan until a pose close the robot is found
    std::vector<dcm_msgs::PathPose>::iterator it = global_path.path_poses.begin();
    std::vector<dcm_msgs::PathPose>::iterator erase_end = it;
    while (it != global_path.path_poses.end())
    {
      double dx = robot.pose.position.x - it->pose.position.x;
      double dy = robot.pose.position.y - it->pose.position.y;
      double dist_sq = dx * dx + dy * dy;
      if (dist_sq < dist_thresh_sq)
      {
        erase_end = it;
        break;
      }
      ++it;
    }
    if (erase_end == global_path.path_poses.end())
      return false;
    
    if (erase_end != global_path.path_poses.begin())
      global_path.path_poses.erase(global_path.path_poses.begin(), erase_end);
  }
  catch (const tf::TransformException& ex)
  {
    ROS_DEBUG("Cannot prune path since no transform is available: %s\n", ex.what());
    return false;
  }
  return true;
}

}  // namespace straight_controller
