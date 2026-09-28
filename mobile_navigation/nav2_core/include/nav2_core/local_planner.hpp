// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
#ifndef NAV2_CORE__LOCAL_PLANNER_HPP_
#define NAV2_CORE__LOCAL_PLANNER_HPP_

#include <memory>
#include <string>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/transform_listener.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_core
{

/**
 * @class LocalPlanner
 * @brief 五层架构 ③局部规划层接口（项目 32 / DEC-007）。
 *
 * 职责：输入上游全局/路网几何路径 + 实时 costmap，输出避障后的**局部几何路径**。
 * 边界：不规划速度（交 ④SpeedPlanner），不出 cmd_vel（交 ⑤Controller），
 *       不做拓扑选路（交 ①Routing）。
 * 由 ControllerServer 作为三段 pipeline 的第 1 段持有（pluginlib 加载）。
 */
class LocalPlanner
{
public:
  using Ptr = std::shared_ptr<nav2_core::LocalPlanner>;

  virtual ~LocalPlanner() {}

  /**
   * @brief 生命周期 configure（对齐 nav2_core::Controller）。
   * @param parent      宿主 lifecycle 节点
   * @param name        插件实例名
   * @param tf          tf buffer
   * @param costmap_ros 局部 costmap（实时障碍）
   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) = 0;

  virtual void cleanup() = 0;
  virtual void activate() = 0;
  virtual void deactivate() = 0;

  /**
   * @brief 设置上游全局/路网几何路径（来自 ②Global）。
   */
  virtual void setPlan(const nav_msgs::msg::Path & path) = 0;

  /**
   * @brief 基于当前 pose/速度 + 实时 costmap，计算避障后的局部几何路径。
   * @param pose       当前机器人位姿（map/odom 系）
   * @param velocity   当前速度
   * @param local_path 输出：避障后的局部几何路径（PoseStamped[]，无速度）
   * @return 成功返回 true；无可行局部路径返回 false（交上层 recovery）
   */
  virtual bool computeLocalPlan(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav_msgs::msg::Path & local_path) = 0;

  /** Request cooperative cancellation of an in-flight planning call. */
  virtual void requestCancel() {}

  /**
   * @brief 限速透传（与 goal_checker 协同；默认无操作）。
   */
  virtual void setSpeedLimit(const double & /*speed_limit*/, const bool & /*percentage*/) {}
};

}  // namespace nav2_core

#endif  // NAV2_CORE__LOCAL_PLANNER_HPP_
