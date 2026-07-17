// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// 规划参数容器：使用 ROS2 declare_parameter + get_parameter（参照 nav2 约定）
#ifndef NAV2_LEX_LOCAL_PLANNER__PLANNING_PARAMS_HPP_
#define NAV2_LEX_LOCAL_PLANNER__PLANNING_PARAMS_HPP_

#include <iostream>
#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_lex_local_planner
{

class PlanningParams
{
public:
  double maxSpeed;
  double minSpeed;
  double planningDistance;
  double microPlanDistance;
  double carTipMargin;
  double rollInMargin;
  double rollInSpeedFactor;
  double pathDensity;
  double rollOutDensity;
  int rollOutNumber;
  double horizonDistance;
  double minFollowingDistance;  // should be bigger than Distance to follow
  double minDistanceToAvoid;    // should be smaller than minFollowingDistance and larger than maxDistanceToAvoid
  double maxDistanceToAvoid;    // should be smaller than minDistanceToAvoid
  double lateralSkipDistance;
  double speedProfileFactor;
  double smoothingDataWeight;
  double smoothingSmoothWeight;
  double smoothingToleranceError;

  double stopSignStopTime;

  double additionalBrakingDistance;
  double verticalSafetyDistance;
  double horizontalSafetyDistancel;

  double giveUpDistance;

  int nReliableCount;

  bool enableLaneChange;
  bool enableSwerving;
  bool enableFollowing;
  bool enableHeadingSmoothing;
  bool enableTrafficLightBehavior;
  bool enableStopSignBehavior;

  bool enabTrajectoryVelocities;
  double minIndicationDistance;

  bool enableAlignCenterline;

  PlanningParams();

  /**
   * @brief 用 ROS2 参数机制声明并加载参数（替代 ROS1 loadRosParamFromNodeHandle）。
   * @param node 宿主 lifecycle 节点（已 lock）
   * @param name 插件实例名（参数前缀，如 "<name>.max_speed"）
   */
  void declareAndLoad(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & name);

  void print(const rclcpp::Logger & logger);

  std::mutex config_mutex_;  //!< Mutex for config accesses and changes
};

}  // namespace nav2_lex_local_planner

#endif  // NAV2_LEX_LOCAL_PLANNER__PLANNING_PARAMS_HPP_
