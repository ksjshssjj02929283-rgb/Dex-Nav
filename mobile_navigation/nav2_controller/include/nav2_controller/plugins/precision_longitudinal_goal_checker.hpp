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

#ifndef NAV2_CONTROLLER__PLUGINS__PRECISION_LONGITUDINAL_GOAL_CHECKER_HPP_
#define NAV2_CONTROLLER__PLUGINS__PRECISION_LONGITUDINAL_GOAL_CHECKER_HPP_

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/goal_checker.hpp"

namespace nav2_controller
{

/**
 * @class PrecisionLongitudinalGoalChecker
 * @brief Goal checker with desired 1 cm precision and 2 cm fallback acceptance.
 */
class PrecisionLongitudinalGoalChecker : public nav2_core::GoalChecker
{
public:
  PrecisionLongitudinalGoalChecker();

  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void reset() override;
  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;
  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override;

protected:
  enum class PrecisionStatus
  {
    OutsideAccept,
    AcceptOnly,
    DesiredUnstable,
    DesiredStable,
    YawUnstable,
    StoppedUnstable
  };

  void resetRefineState();
  void recordAbsErrors(double abs_lon_err, double abs_lat_err, double abs_yaw_err);
  double meanAbs(const std::deque<double> & data) const;
  void publishGoalError(
    const geometry_msgs::msg::Pose & query_pose,
    const geometry_msgs::msg::Pose & goal_pose,
    double goal_yaw,
    double current_yaw,
    double lon_err,
    double lat_err,
    double yaw_err,
    double distance_err,
    bool yaw_within_tolerance,
    bool in_desired_window,
    bool in_accept_window,
    bool stable_desired,
    const std::string & result_type);
  void logStatusTransition(
    PrecisionStatus status,
    double lon_err,
    double lat_err,
    double yaw_err,
    bool stable_desired,
    int stable_window_count,
    double avg_lon_err,
    double avg_lat_err,
    double avg_yaw_err);

  double desired_lon_goal_tolerance_, desired_lat_goal_tolerance_;
  double accept_lon_goal_tolerance_, accept_lat_goal_tolerance_;
  double yaw_goal_tolerance_;
  int refine_cycles_, stable_window_size_;
  int refine_count_;
  bool accept_refine_restart_used_, desired_refine_restart_used_;
  bool check_stopped_;
  PrecisionStatus last_status_{PrecisionStatus::OutsideAccept};
  double trans_stopped_velocity_, rot_stopped_velocity_;

  std::deque<double> lon_errors_;
  std::deque<double> lat_errors_;
  std::deque<double> yaw_errors_;

  rclcpp::Clock::SharedPtr clock_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    goal_error_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::string plugin_name_;

  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PLUGINS__PRECISION_LONGITUDINAL_GOAL_CHECKER_HPP_
