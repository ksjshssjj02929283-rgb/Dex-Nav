/***********************************************************
 *
 * @file: straight_controller.h
 * @breif: 控制机器人走直线的controller
 * @author: guo linfeng
 * @update: 2024.06.20
 * @version: 1.1
 *
 * Copyright (c) 2024, Guo Linfeng
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/

#ifndef STRAIGHT_CONTROLLER_H_
#define STRAIGHT_CONTROLLER_H_

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

namespace straight_controller
{
/**
 * @brief A class implementing a local planner using the PID
 */
class StraightController : public nav_core::BaseController
{
public:
  /**
   * @brief Construct a new StraightController object
   */
  StraightController();

  /**
   * @brief Construct a new StraightController object
   */
  StraightController(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Destroy the StraightController object
   */
  ~StraightController();

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

  bool pruneGlobalPlan(const tf2_ros::Buffer& tf,
                       const geometry_msgs::PoseStamped& global_pose,
                       dcm_msgs::Path& global_path,
                       double dist_behind_robot);

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
  double max_path_length_ = 0.5;

  int closest_plan_index_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  geometry_msgs::PoseStamped closest_ps_;
  geometry_msgs::PoseStamped current_ps_; // current pose
  geometry_msgs::PoseStamped goal_ps_;
  geometry_msgs::PoseStamped target_ps_in_odom_fixed_; // current pose
  double long_precision_, lat_precision_, o_precision_;  // goal reached tolerance
  double d_t_;                        // control time step

  double max_v_, min_v_, max_v_inc_;  // linear velocity
  double max_w_, min_w_, max_w_inc_;  // angular velocity

  double k_w_p_, k_w_i_, k_w_d_;  // pid controller params

  double e_w_;
  double i_w_;

  base_local_planner::OdometryHelperRos* odom_helper_;
  ros::Publisher target_pose_pub_, current_pose_pub_;
  ros::Publisher desired_vel_pub_;

  double goal_x_, goal_y_;
  double goal_yaw_;

  std::deque<double> theta_errors_;
  std::deque<double> longitude_errors_;

  std::mutex plan_mutex_;

  dcm_msgs::Path controller_path_;

  controller_common::PIDController station_pid_controller_;
  controller_common::PIDController speed_pid_controller_;

  controller_common::PidConf station_pid_conf_;
  controller_common::PidConf high_speed_pid_conf_;
  controller_common::PidConf low_speed_pid_conf_;

  controller_common::PidConf reverse_station_pid_conf_;
  controller_common::PidConf reverse_speed_pid_conf_;
  double switch_speed_ = 0.1;

  double desired_linear_vel_ = 0.0;
  double acc_max_ = 0.5;
};

}  // namespace straight_controller

#endif // STRAIGHT_CONTROLLER_H_
