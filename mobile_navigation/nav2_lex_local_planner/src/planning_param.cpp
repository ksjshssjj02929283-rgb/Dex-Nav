// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// 从 dcm lex_local_planner/planning_param.cpp 移植。默认值原样保留；
// ROS 边界替换：loadRosParamFromNodeHandle(ros::NodeHandle) ->
//   declareAndLoad(node, name)，用 declare_parameter + get_parameter；
// 删除 reconfigure(LEXLocalPlannerConfig)（dynamic_reconfigure）；
// print() std::cout -> RCLCPP_INFO(logger, ...)。
#include <iostream>
#include <string>
#include <mutex>

#include "nav2_lex_local_planner/planning_params.hpp"

namespace nav2_lex_local_planner
{

PlanningParams::PlanningParams()
{
  maxSpeed = 3;
  lateralSkipDistance = 6.0;
  minSpeed = 0;
  planningDistance = 10000;
  microPlanDistance = 30;
  carTipMargin = 4.0;
  rollInMargin = 12.0;
  rollInSpeedFactor = 0.25;
  pathDensity = 0.25;
  rollOutDensity = 0.5;
  rollOutNumber = 4;
  horizonDistance = 120;
  minFollowingDistance = 35;
  minDistanceToAvoid = 15;
  maxDistanceToAvoid = 5;
  speedProfileFactor = 1.0;
  smoothingDataWeight = 0.47;
  smoothingSmoothWeight = 0.2;
  smoothingToleranceError = 0.1;
  stopSignStopTime = 2.0;

  additionalBrakingDistance = 10.0;
  verticalSafetyDistance = 0.0;
  horizontalSafetyDistancel = 0.0;

  giveUpDistance = -4;
  nReliableCount = 2;

  enableHeadingSmoothing = false;
  enableSwerving = false;
  enableFollowing = false;
  enableTrafficLightBehavior = false;
  enableLaneChange = false;
  enableStopSignBehavior = false;
  enabTrajectoryVelocities = false;
  minIndicationDistance = 15;

  enableAlignCenterline = false;
}

void PlanningParams::declareAndLoad(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name)
{
  auto declare_get_double = [&](const std::string & key, double & value) {
      std::string full = name + "." + key;
      if (!node->has_parameter(full)) {
        node->declare_parameter(full, rclcpp::ParameterValue(value));
      }
      node->get_parameter(full, value);
    };
  auto declare_get_int = [&](const std::string & key, int & value) {
      std::string full = name + "." + key;
      if (!node->has_parameter(full)) {
        node->declare_parameter(full, rclcpp::ParameterValue(value));
      }
      node->get_parameter(full, value);
    };
  auto declare_get_bool = [&](const std::string & key, bool & value) {
      std::string full = name + "." + key;
      if (!node->has_parameter(full)) {
        node->declare_parameter(full, rclcpp::ParameterValue(value));
      }
      node->get_parameter(full, value);
    };

  declare_get_double("max_speed", maxSpeed);
  declare_get_double("min_speed", minSpeed);
  declare_get_double("planning_distance", planningDistance);
  declare_get_double("micro_plan_distance", microPlanDistance);
  declare_get_double("car_tip_margin", carTipMargin);
  declare_get_double("roll_in_margin", rollInMargin);
  declare_get_double("roll_in_speed_factor", rollInSpeedFactor);
  declare_get_double("path_density", pathDensity);
  declare_get_double("roll_out_density", rollOutDensity);
  declare_get_int("roll_out_number", rollOutNumber);
  declare_get_double("horizon_distance", horizonDistance);
  declare_get_double("min_following_distance", minFollowingDistance);
  declare_get_double("min_distance_to_avoid", minDistanceToAvoid);
  declare_get_double("max_distance_to_avoid", maxDistanceToAvoid);
  declare_get_double("lateral_skip_distance", lateralSkipDistance);
  declare_get_double("speed_profile_factor", speedProfileFactor);
  declare_get_double("smoothing_data_weight", smoothingDataWeight);
  declare_get_double("smoothing_smooth_weight", smoothingSmoothWeight);
  declare_get_double("smoothing_tolerance_error", smoothingToleranceError);
  declare_get_double("stop_sign_stop_time", stopSignStopTime);
  declare_get_double("additional_braking_distance", additionalBrakingDistance);
  declare_get_double("vertical_safety_distance", verticalSafetyDistance);
  declare_get_double("horizontal_safety_distance", horizontalSafetyDistancel);
  declare_get_double("give_up_distance", giveUpDistance);
  declare_get_int("n_reliable_count", nReliableCount);
  declare_get_bool("enable_lane_change", enableLaneChange);
  declare_get_bool("enable_swerving", enableSwerving);
  declare_get_bool("enable_following", enableFollowing);
  declare_get_bool("enable_heading_smoothing", enableHeadingSmoothing);
  declare_get_bool("enable_traffic_light_behavior", enableTrafficLightBehavior);
  declare_get_bool("enable_stop_sign_behavior", enableStopSignBehavior);
  declare_get_bool("enable_trajectory_velocities", enabTrajectoryVelocities);
  declare_get_double("min_indication_distance", minIndicationDistance);
  declare_get_bool("enable_align_centerline", enableAlignCenterline);
}

void PlanningParams::print(const rclcpp::Logger & logger)
{
  RCLCPP_INFO(logger, "PlanningParams:");
  RCLCPP_INFO(logger, " -- max_speed: %f", maxSpeed);
  RCLCPP_INFO(logger, " -- min_speed: %f", minSpeed);
  RCLCPP_INFO(logger, " -- planning_distance: %f", planningDistance);
  RCLCPP_INFO(logger, " -- micro_plan_distance: %f", microPlanDistance);
  RCLCPP_INFO(logger, " -- car_tip_margin: %f", carTipMargin);
  RCLCPP_INFO(logger, " -- roll_in_margin: %f", rollInMargin);
  RCLCPP_INFO(logger, " -- roll_in_speed_factor: %f", rollInSpeedFactor);
  RCLCPP_INFO(logger, " -- path_density: %f", pathDensity);
  RCLCPP_INFO(logger, " -- roll_out_density: %f", rollOutDensity);
  RCLCPP_INFO(logger, " -- roll_out_number: %d", rollOutNumber);
  RCLCPP_INFO(logger, " -- horizon_distance: %f", horizonDistance);
  RCLCPP_INFO(logger, " -- min_following_distance: %f", minFollowingDistance);
  RCLCPP_INFO(logger, " -- min_distance_to_avoid: %f", minDistanceToAvoid);
  RCLCPP_INFO(logger, " -- max_distance_to_avoid: %f", maxDistanceToAvoid);
  RCLCPP_INFO(logger, " -- lateral_skip_distance: %f", lateralSkipDistance);
  RCLCPP_INFO(logger, " -- speed_profile_factor: %f", speedProfileFactor);
  RCLCPP_INFO(logger, " -- smoothing_data_weight: %f", smoothingDataWeight);
  RCLCPP_INFO(logger, " -- smoothing_smooth_weight: %f", smoothingSmoothWeight);
  RCLCPP_INFO(logger, " -- stop_sign_stop_time: %f", stopSignStopTime);
  RCLCPP_INFO(logger, " -- additional_braking_distance: %f", additionalBrakingDistance);
  RCLCPP_INFO(logger, " -- vertical_safety_distance: %f", verticalSafetyDistance);
  RCLCPP_INFO(logger, " -- horizontal_safety_distance: %f", horizontalSafetyDistancel);
  RCLCPP_INFO(logger, " -- give_up_distance: %f", giveUpDistance);
  RCLCPP_INFO(logger, " -- n_reliable_count: %d", nReliableCount);
  RCLCPP_INFO(logger, " -- enable_lane_change: %d", enableLaneChange);
  RCLCPP_INFO(logger, " -- enable_swerving: %d", enableSwerving);
  RCLCPP_INFO(logger, " -- enable_following: %d", enableFollowing);
  RCLCPP_INFO(logger, " -- enable_heading_smoothing: %d", enableHeadingSmoothing);
  RCLCPP_INFO(logger, " -- enable_traffic_light_behavior: %d", enableTrafficLightBehavior);
  RCLCPP_INFO(logger, " -- enable_stop_sign_behavior: %d", enableStopSignBehavior);
  RCLCPP_INFO(logger, " -- enable_trajectory_velocities: %d", enabTrajectoryVelocities);
  RCLCPP_INFO(logger, " -- min_indication_distance: %f", minIndicationDistance);
  RCLCPP_INFO(logger, " -- enable_align_centerline: %d", enableAlignCenterline);
}

}  // namespace nav2_lex_local_planner
