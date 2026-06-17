/***********************************************************
 *
 * @file: straight_controller.hpp
 * @brief: 控制机器人走直线的 controller（ROS2 nav2_core::Controller 插件）
 * @author: guo linfeng
 * @update: 2026.06
 * @version: 2.0
 *
 * Copyright (c) 2024, Guo Linfeng
 * All rights reserved.
 * --------------------------------------------------------
 *
 * 由 dcm_navigation/nav_modules/controller/straight_controller（ROS1 nav_core::BaseController）
 * 移植而来。算法（纵向 PID 保速度 + 手工 PID 控航向 + 状态机）完整保留，仅替换 ROS1→ROS2 边界。
 *
 **********************************************************/

#ifndef STRAIGHT_CONTROLLER__STRAIGHT_CONTROLLER_HPP_
#define STRAIGHT_CONTROLLER__STRAIGHT_CONTROLLER_HPP_

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "tf2_ros/buffer.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float64.hpp"

#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

#include "nav_interfaces/msg/robot_trajectory.hpp"

#include "controller_common/pid_controller.h"

namespace straight_controller
{

/**
 * @brief 内部轨迹点缓存：几何 pose（map 系）+ 期望速度 + 曲率。
 *
 * 替代 ROS1 中的 dcm_msgs::PathPose：speed_limit ← TrajectoryPoint.longitudinal_velocity，
 * curvature ← TrajectoryPoint.path_kappa，pose ← TrajectoryPoint.pose。
 */
struct StraightPathPoint
{
  std_msgs::msg::Header header;
  geometry_msgs::msg::Pose pose;
  double speed_limit{0.0};
  double curvature{0.0};
};

/**
 * @class StraightController
 * @brief 基于 PID 的直线跟踪 local planner，实现 nav2_core::Controller 接口。
 */
class StraightController : public nav2_core::Controller
{
public:
  StraightController() = default;
  ~StraightController() override = default;

  /**
   * @brief 配置控制器（对应 ROS1 initialize）
   * @param parent      生命周期节点 WeakPtr
   * @param name        插件实例名
   * @param tf          TF buffer
   * @param costmap_ros 局部代价地图
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief 清理资源
   */
  void cleanup() override;

  /**
   * @brief 激活控制器（激活 publisher）
   */
  void activate() override;

  /**
   * @brief 反激活控制器
   */
  void deactivate() override;

  /**
   * @brief 设置几何全局路径（无速度，speed_limit 退化为 setSpeedLimit 上限）
   * @param path 全局路径（map 或可转 map 系）
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief 设置带速度/曲率的时空轨迹（五层架构 ④SpeedPlanner 输出）
   * @param trajectory 带 longitudinal_velocity / path_kappa 的轨迹
   */
  void setTrajectory(const nav_interfaces::msg::RobotTrajectory & trajectory) override;

  /**
   * @brief 计算速度指令（对应 ROS1 computeVelocityCommands）
   * @param pose         当前机器人位姿（替代原 getRobotPose）
   * @param velocity     当前机器人速度（替代原 odom helper）
   * @param goal_checker 目标检查器（未使用）
   * @return 速度指令（TwistStamped）
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  /**
   * @brief 限制最大线速度。本控制器保存上限，与每点轨迹速度取严。
   * @param speed_limit 绝对值 (m/s) 或百分比
   * @param percentage  true=百分比，false=绝对值
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  /**
   * @brief 检查是否到达目标（保留接口，逻辑同 ROS1：恒返回 false）
   */
  bool isGoalReached();

  /**
   * @brief 设置倒车使能（保留 ROS1 BaseController 的 public 方法）
   */
  void setReverseEnabled(const bool enabled_reverse) {reverse_enabled_ = enabled_reverse;}

private:
  // ---- 角度归一化到 [-pi, pi] ----
  inline void regularizeAngle(double & angle)
  {
    angle = angle - 2.0 * M_PI * std::floor((angle + M_PI) / (2.0 * M_PI));
  }

  // ---- 路径长度（移植自 nav_util::computePathLength(dcm_msgs::Path)）----
  double computePathLength(const std::vector<StraightPathPoint> & path) const;

  // ---- 最近点索引（移植自 nav_util::getClosestWaypointID(dcm_msgs::Path)）----
  int getClosestWaypointID(
    const std::vector<StraightPathPoint> & path,
    const geometry_msgs::msg::PoseStamped & pose,
    double ignore_dist = 0.0,
    bool ignore_first = false) const;

  // ---- 裁剪机器人身后路径（移植自 ROS1 pruneGlobalPlan，按距离）----
  bool pruneGlobalPlan(
    const geometry_msgs::msg::PoseStamped & global_pose,
    std::vector<StraightPathPoint> & global_path,
    double dist_behind_robot);

  // ---- 把 nav_msgs::Path / RobotTrajectory 缓存为内部轨迹（转 map 系）----
  bool cachePath(
    const std::vector<StraightPathPoint> & raw_points,
    double max_total_length, bool length_check);

  void publishDesiredVelocity();

  /**
   * @brief 纵向 PID（保速度 + 站点偏差），返回线速度。
   * @param curr_long_vel 当前纵向线速度（来自入参 velocity）
   * @param e_dist        body 系下到目标点的纵向偏差
   */
  double LinearSpeedControl(double curr_long_vel, double e_dist);

  /**
   * @brief 航向手工 PID（k_w_p/i/d），返回角速度。
   * @param curr_ang_vel 当前角速度（来自入参 velocity）
   * @param e_theta      航向偏差
   */
  double AngularSpeedControl(double curr_ang_vel, double e_theta);

private:
  // ---- ROS2 句柄 ----
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("StraightController")};
  rclcpp::Clock::SharedPtr clock_;
  std::string plugin_name_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>>
    target_pose_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>>
    desired_vel_pub_;

  // ---- 状态 ----
  bool initialized_{false};
  bool goal_reached_{false};
  bool reverse_enabled_{false};
  double max_path_length_{0.5};

  int closest_plan_index_{0};
  geometry_msgs::msg::PoseStamped closest_ps_;
  geometry_msgs::msg::PoseStamped current_ps_;   // current pose in map
  geometry_msgs::msg::PoseStamped goal_ps_;

  double long_precision_{0.02};
  double lat_precision_{0.04};
  double o_precision_{0.5};       // goal reached tolerance
  double d_t_{0.1};               // control time step

  double max_v_{0.5}, min_v_{0.0}, max_v_inc_{0.5};      // linear velocity
  double max_w_{1.57}, min_w_{0.0}, max_w_inc_{1.57};    // angular velocity

  double k_w_p_{1.0}, k_w_i_{0.01}, k_w_d_{0.10};        // 航向手工 PID 参数

  double e_w_{0.0};
  double i_w_{0.0};

  double goal_x_{0.0}, goal_y_{0.0};
  double goal_yaw_{0.0};

  std::mutex plan_mutex_;

  std::vector<StraightPathPoint> controller_path_;   // 内部轨迹（map 系）

  controller_common::PIDController station_pid_controller_;
  controller_common::PIDController speed_pid_controller_;

  controller_common::PidConf station_pid_conf_;
  controller_common::PidConf high_speed_pid_conf_;
  controller_common::PidConf low_speed_pid_conf_;
  controller_common::PidConf reverse_station_pid_conf_;
  controller_common::PidConf reverse_speed_pid_conf_;
  double switch_speed_{0.1};

  double desired_linear_vel_{0.0};
  double acc_max_{0.5};

  // setSpeedLimit 保存的全局速度上限（与每点轨迹速度取严）；0 = 不限速
  double speed_limit_{0.0};
};

}  // namespace straight_controller

#endif  // STRAIGHT_CONTROLLER__STRAIGHT_CONTROLLER_HPP_
