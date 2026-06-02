// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0
//
// ROS2 Nav2 局部规划插件入口。实现冻结接口 nav2_core::LocalPlanner。
// 从 dcm lex_local_planner/lex_local_planner_ros.h（nav_core::BaseLocalPlanner）移植：
//   - initialize(name, tf2_ros::Buffer*, costmap_2d::Costmap2DROS*)
//       -> configure(parent, name, shared_ptr<tf2_ros::Buffer>, shared_ptr<Costmap2DROS>)
//          + activate()/deactivate()/cleanup()
//   - 删除 dynamic_reconfigure / odom_helper（速度改由 computeLocalPlan 入参 velocity 提供）
//   - 删除 ROS1 g_plan_pub_/l_plan_pub_ rviz 发布器
//   - setPlan(vector<PoseStamped>) -> setPlan(const nav_msgs::msg::Path &)
//   - computeLocalPlan(vector<PoseStamped>&) -> computeLocalPlan(pose, velocity, nav_msgs::msg::Path&)
#ifndef NAV2_LEX_LOCAL_PLANNER__LEX_LOCAL_PLANNER_HPP_
#define NAV2_LEX_LOCAL_PLANNER__LEX_LOCAL_PLANNER_HPP_

#include <memory>
#include <string>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"

#include "nav2_core/local_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

#include "nav2_lex_local_planner/data_type.hpp"
#include "nav2_lex_local_planner/planning_params.hpp"
#include "nav2_lex_local_planner/rollout_generator.hpp"
#include "nav2_lex_local_planner/lex_planner.hpp"

namespace nav2_lex_local_planner
{

class LexLocalPlanner : public nav2_core::LocalPlanner
{
public:
  LexLocalPlanner();
  ~LexLocalPlanner() override;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  bool computeLocalPlan(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav_msgs::msg::Path & local_path) override;

protected:
  /**
   * @brief 将全局路径裁剪/变换到 global_frame 的窗口内（max_plan_length）。
   *        从 dcm transformGlobalPlan 移植，tf 用 shared_ptr<tf2_ros::Buffer>，
   *        ROS1 tf 异常类型改为 tf2::TransformException。
   */
  bool transformGlobalPlan(
    const std::vector<geometry_msgs::msg::PoseStamped> & global_plan,
    const geometry_msgs::msg::PoseStamped & global_pose,
    const std::string & global_frame, double max_plan_length,
    std::vector<geometry_msgs::msg::PoseStamped> & transformed_plan,
    int * current_goal_idx, geometry_msgs::msg::TransformStamped * tf_plan_to_global) const;

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Logger logger_{rclcpp::get_logger("LexLocalPlanner")};
  rclcpp::Clock::SharedPtr clock_;

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;

  std::shared_ptr<nav2_lex_local_planner::PlanningParams> planning_params_;

  std::string global_frame_;     //!< Global frame for the costmap
  std::string robot_base_frame_;  //!< Used as the base frame for the robot
  std::string name_;

  std::vector<geometry_msgs::msg::PoseStamped> global_plan_;  //!< Store the current global plan
  std::vector<nav2_lex_local_planner::WayPoint> global_waypoints_;
  std::unique_ptr<nav2_lex_local_planner::RolloutGenerator> rollout_generator_;
  std::unique_ptr<nav2_lex_local_planner::LexPlanner> lex_planner_;

  std::mutex mtx_;
};

}  // namespace nav2_lex_local_planner

#endif  // NAV2_LEX_LOCAL_PLANNER__LEX_LOCAL_PLANNER_HPP_
