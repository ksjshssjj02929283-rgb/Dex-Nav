/***********************************************************
 *
 * @file: pure_rotation_controller.h
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

#ifndef pure_rotation_controller_H_
#define pure_rotation_controller_H_

#include <deque>

#include <ros/ros.h>
#include <nav_core/base_controller.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs_stamped/Float64Stamped.h>

#include <base_local_planner/odometry_helper_ros.h>
#include <tf2/utils.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>

#include <mutex>

#include "dcm_msgs/Path.h"
#include "controller_common/pid_controller.h"

namespace pure_rotation_controller
{
/**
 * @brief A class implementing a local planner using the PID
 */
class PureRotationController : public nav_core::BaseController
{
public:
  /**
   * @brief Construct a new PureRotationController object
   */
  PureRotationController();

  /**
   * @brief Construct a new PureRotationController object
   */
  PureRotationController(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Destroy the PureRotationController object
   */
  ~PureRotationController();

  /**
   * @brief Initialization of the local planner
   * @param name        the name to give this instance of the trajectory planner
   * @param tf          a pointer to a transform listener
   * @param costmap_ros the cost map to use for assigning costs to trajectories
   */
  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Set the plan that the controller is following
   * @param orig_global_plan the plan to pass to the controller
   * @return  true if the plan was updated successfully, else false
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) override;

  bool setPlan(const dcm_msgs::Path& dcm_path) override;

  /**
   * @brief  Check if the goal pose has been achieved
   * @return True if achieved, false otherwise
   */
  bool isGoalReached();

  /**
   * @brief Given the current position, orientation, and velocity of the robot, compute the velocity commands
   * @param cmd_vel will be filled with the velocity command to be passed to the robot base
   * @return  true if a valid trajectory was found, else false
   */
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

private:
  double CalculateMean(const std::deque<double>& data);
  double CalculateVariance(const std::deque<double>& data, double mean);

  inline void regularizeAngle(double& angle) {
    angle = angle - 2.0 * M_PI * std::floor((angle + M_PI) / (2.0 * M_PI));
  }

  void publishDesiredVelocity();

  /**
   * @brief PID controller in linear
   * @param base_odometry odometry of the robot, to get velocity
   * @param b_x_d         desired x in body frame
   * @param b_y_d         desired y in body frame
   * @return  linear velocity
   */
  double LinearSpeedControl(const nav_msgs::Odometry& base_odometry, double e_dist);

  /**
   * @brief PID controller in angular
   * @param base_odometry odometry of the robot, to get velocity
   * @param e_theta       the error between the current and desired theta
   * @return  angular velocity
   */
  double AngularSpeedControl(const nav_msgs::Odometry& base_odometry, double e_theta);

private:
  bool initialized_, goal_reached_;
  tf2_ros::Buffer* tf_;
  costmap_2d::Costmap2DROS* costmap_ros_;

  int plan_index_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  geometry_msgs::PoseStamped target_ps_;
  geometry_msgs::PoseStamped current_ps_; // current pose

  double long_precision_, lat_precision_, o_precision_;  // goal reached tolerance
  double d_t_;                        // control time step

  double max_v_, min_v_, max_v_inc_;  // linear velocity
  double max_w_, min_w_, max_w_inc_;  // angular velocity

  base_local_planner::OdometryHelperRos* odom_helper_;
  ros::Publisher target_pose_pub_, current_pose_pub_;
  ros::Publisher desired_vel_pub_;

  double goal_x_, goal_y_;
  double goal_yaw_;

  std::deque<double> theta_errors_;
  std::deque<double> longitude_errors_;

  std::mutex plan_mutex_;

  dcm_msgs::Path controller_path_;

  // 线速度PID控制器
  controller_common::PIDController station_pid_controller_;
  controller_common::PIDController speed_pid_controller_;

  controller_common::PidConf station_pid_conf_;
  controller_common::PidConf speed_pid_conf_;

  // 角速度PID控制器
  controller_common::PIDController angular_pid_controller_;
  // controller_common::PureRotationController angular_speed_pure_rotation_controller_;

  controller_common::PidConf angular_pid_conf_;
  // controller_common::PidConf angular_speed_pid_conf_;

  double desired_linear_vel_ = 0.0;
  double acc_max_ = 0.5;

  bool enable_lon_adjust_;  // 纵向距离调整使能
};

}  // namespace pure_rotation_controller

#endif // pure_rotation_controller_H_
