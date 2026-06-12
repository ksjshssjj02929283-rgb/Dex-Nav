/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
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

#include <memory>
#include <string>
#include <limits>
#include <vector>
#include <cmath>
#include "nav2_controller/plugins/longitudinal_goal_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "angles/angles.h"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_controller
{

LongitudinalGoalChecker::LongitudinalGoalChecker()
: lon_goal_tolerance_(0.05),
  lat_goal_tolerance_(0.03),
  yaw_goal_tolerance_(0.25),
  stateful_(true),
  check_xy_(true),
  xy_goal_tolerance_sq_(0.0625)
{
}

void LongitudinalGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>/*costmap_ros*/)
{
  plugin_name_ = plugin_name;
  auto node = parent.lock();

  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".lon_goal_tolerance", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".lat_goal_tolerance", rclcpp::ParameterValue(0.03));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".stateful", rclcpp::ParameterValue(true));

  node->get_parameter(plugin_name + ".lon_goal_tolerance", lon_goal_tolerance_);
  node->get_parameter(plugin_name + ".lat_goal_tolerance", lat_goal_tolerance_);
  node->get_parameter(plugin_name + ".yaw_goal_tolerance", yaw_goal_tolerance_);
  node->get_parameter(plugin_name + ".stateful", stateful_);

  xy_goal_tolerance_sq_ = lon_goal_tolerance_ * lat_goal_tolerance_;

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&LongitudinalGoalChecker::dynamicParametersCallback, this, _1));
}

void LongitudinalGoalChecker::reset()
{
  check_xy_ = true;
}

bool LongitudinalGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist &)
{
  const double goal_yaw = tf2::getYaw(goal_pose.orientation);
  const double dx = query_pose.position.x - goal_pose.position.x;
  const double dy = query_pose.position.y - goal_pose.position.y;
  // 在当前公共坐标系内，按目标朝向投影为真正纵向/横向误差
  const double lon_err = std::cos(goal_yaw) * dx + std::sin(goal_yaw) * dy;
  const double lat_err = -std::sin(goal_yaw) * dx + std::cos(goal_yaw) * dy;
  double dyaw = angles::shortest_angular_distance(
    tf2::getYaw(query_pose.orientation),
    goal_yaw);
  if(hypot(lon_err, lat_err) < 0.25)
  {
    RCLCPP_INFO(rclcpp::get_logger("LongitudinalGoalChecker"),"Near goal: lon_err: %.4f, lat_err: %.4f", lon_err, lat_err);
  }
  if (check_xy_) {
    if(fabs(lon_err) > lon_goal_tolerance_ || 
       fabs(lat_err) > lat_goal_tolerance_) 
    {
      return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("LongitudinalGoalChecker"),"#####  goal reached error: lon_err: %.4f, lat_err: %.4f, yaw_err: %.4f", lon_err, lat_err, dyaw);
    // We are within the window
    // If we are stateful, change the state.
    if (stateful_) {
      check_xy_ = false;
    }
  }
  if(!(fabs(dyaw) < yaw_goal_tolerance_))
  {
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("LongitudinalGoalChecker"),"lon_goal_tolerance: %.3f, lat_goal_tolerance: %.3f, yaw_goal_tolerance: %.3f", lon_goal_tolerance_, lat_goal_tolerance_, yaw_goal_tolerance_);
  RCLCPP_INFO(rclcpp::get_logger("LongitudinalGoalChecker"),"LongitudinalGoalChecker: Goal reached!");
  RCLCPP_INFO(rclcpp::get_logger("LongitudinalGoalChecker"),"goal reached error: lon_err: %.4f, lat_err: %.4f, yaw_err: %.4f", lon_err, lat_err, dyaw);
  

  const double query_yaw = tf2::getYaw(query_pose.orientation);
  RCLCPP_INFO(
    rclcpp::get_logger("LongitudinalGoalChecker"),
    "query_pose: (x=%.4f, y=%.4f, yaw=%.4f)  goal_pose: (x=%.4f, y=%.4f, yaw=%.4f)",
    query_pose.position.x, query_pose.position.y, query_yaw,
    goal_pose.position.x, goal_pose.position.y, goal_yaw);

  return true;
}

bool LongitudinalGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  double invalid_field = std::numeric_limits<double>::lowest();

  pose_tolerance.position.x = lon_goal_tolerance_;
  pose_tolerance.position.y = lat_goal_tolerance_;
  pose_tolerance.position.z = invalid_field;
  pose_tolerance.orientation =
    nav2_util::geometry_utils::orientationAroundZAxis(yaw_goal_tolerance_);

  vel_tolerance.linear.x = invalid_field;
  vel_tolerance.linear.y = invalid_field;
  vel_tolerance.linear.z = invalid_field;

  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = invalid_field;

  return true;
}

rcl_interfaces::msg::SetParametersResult
LongitudinalGoalChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".lon_goal_tolerance") {
        lon_goal_tolerance_ = parameter.as_double();
        // xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;
      } else if (name == plugin_name_ + ".yaw_goal_tolerance") {
        yaw_goal_tolerance_ = parameter.as_double();
      } else if (name == plugin_name_ + ".lat_goal_tolerance") {
        lat_goal_tolerance_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".stateful") {
        stateful_ = parameter.as_bool();
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace nav2_controller

PLUGINLIB_EXPORT_CLASS(nav2_controller::LongitudinalGoalChecker, nav2_core::GoalChecker)
