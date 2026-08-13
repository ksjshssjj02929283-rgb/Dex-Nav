/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  Copyright (c) 2019, Intel Corporation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef NAV2_CORE__CONTROLLER_HPP_
#define NAV2_CORE__CONTROLLER_HPP_

#include <memory>
#include <string>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/transform_listener.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_interfaces/msg/robot_trajectory.hpp"
#include "nav2_core/goal_checker.hpp"


namespace nav2_core
{

/**
 * @class PathObstacleAvoidanceCapability
 * @brief 控制器可选能力：能够绕开参考路径上的局部障碍，而不是要求速度规划停车。
 *
 * ControllerServer 通过 dynamic_cast 探测该能力。不实现本接口的控制器按安全默认值
 * 处理：速度规划必须在路径障碍前停车，避免依赖具体插件名称判断控制器行为。
 */
class PathObstacleAvoidanceCapability
{
public:
  virtual ~PathObstacleAvoidanceCapability() = default;
  virtual bool canAvoidPathObstacles() const = 0;
};

/**
 * @class Controller
 * @brief controller interface that acts as a virtual base class for all controller plugins
 */
class Controller
{
public:
  using Ptr = std::shared_ptr<nav2_core::Controller>;


  /**
   * @brief Virtual destructor
   */
  virtual ~Controller() {}

  /**
   * @param  parent pointer to user's node
   * @param  costmap_ros A pointer to the costmap
   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    std::string name, std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) = 0;

  /**
   * @brief Method to cleanup resources.
   */
  virtual void cleanup() = 0;

  /**
   * @brief Method to active planner and any threads involved in execution.
   */
  virtual void activate() = 0;

  /**
   * @brief Method to deactive planner and any threads involved in execution.
   */
  virtual void deactivate() = 0;

  /**
   * @brief local setPlan - Sets the global plan
   * @param path The global plan
   */
  virtual void setPlan(const nav_msgs::msg::Path & path) = 0;

  /**
   * @brief 设置带速度的时空轨迹（五层架构 ④SpeedPlanner 输出，项目 32 / DEC-011）。
   *
   * 轨迹跟踪型 controller override 此方法，消费 per-point longitudinal_velocity / path_kappa
   * 等做纯跟踪（不再自行决速）。默认空实现，保持对现有几何路径型 controller 向后兼容。
   * @param trajectory 带速度 / 加速度 / 时间 / 段语义的轨迹（来自 SpeedPlanner）
   */
  virtual void setTrajectory(const nav_interfaces::msg::RobotTrajectory & /*trajectory*/) {}

  /**
   * @brief Return the trajectory state used by the most recent longitudinal
   *        tracking command. Direct-control branches return false.
   */
  virtual bool getLastTrajectoryTrackingTarget(
    double & /*velocity*/, double & /*acceleration*/)
  {
    return false;
  }

  /**
   * @brief Controller computeVelocityCommands - calculates the best command given the current pose and velocity
   *
   * It is presumed that the global plan is already set.
   *
   * This is mostly a wrapper for the protected computeVelocityCommands
   * function which has additional debugging info.
   *
   * @param pose Current robot pose
   * @param velocity Current robot velocity
   * @param goal_checker Pointer to the current goal checker the task is utilizing
   * @return The best command for the robot to drive
   */
  virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) = 0;

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   */
  virtual void setSpeedLimit(const double & speed_limit, const bool & percentage) = 0;
};

}  // namespace nav2_core

#endif  // NAV2_CORE__CONTROLLER_HPP_
