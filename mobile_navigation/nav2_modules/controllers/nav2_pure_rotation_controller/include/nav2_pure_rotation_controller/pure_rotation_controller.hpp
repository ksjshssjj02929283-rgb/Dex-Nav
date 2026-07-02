/***********************************************************
 *
 * @file: nav2_pure_rotation_controller.hpp
 * @brief: 原地旋转控制器（纵向 station/speed PID + 独立角速度 PID 并行）。
 *         由 dcm nav2_pure_rotation_controller（nav_core::BaseController）移植为
 *         ROS2 nav2_core::Controller pluginlib 插件。状态机 / PID / 参数逻辑保持不变，
 *         仅替换 ROS1→ROS2 边界。
 * @author: Yang Haodong, Guo Zhanyu, Wu Maojia （原作者）
 * @port:   ROS1 → ROS2 nav2_core::Controller
 *
 **********************************************************/

#ifndef PURE_ROTATION_CONTROLLER__PURE_ROTATION_CONTROLLER_HPP_
#define PURE_ROTATION_CONTROLLER__PURE_ROTATION_CONTROLLER_HPP_

#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_interfaces/msg/robot_trajectory.hpp"

#include "tf2_ros/buffer.h"

#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "nav2_controller_common/pid_controller.h"

namespace nav2_pure_rotation_controller
{
/**
 * @brief 原地旋转 controller（纵向 station/speed PID + 独立角速度 PID 并行）
 *
 * enable_lon_adjust 控制是否纯旋转：false = 只转（纵向输出 0），true = 同时做纵向距离微调。
 */
class PureRotationController : public nav2_core::Controller
{
public:
  PureRotationController() = default;
  ~PureRotationController() override = default;

  /**
   * @brief 配置 controller（ROS2 生命周期）。对应 dcm initialize()。
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  /**
   * @brief 几何路径入口（无速度信息）。对应 dcm setPlan(std::vector<PoseStamped>)。
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief 带速度时空轨迹入口。对应 dcm setPlan(const dcm_msgs::Path&)。
   *        消费 per-point longitudinal_velocity（期望速度）/ path_kappa（曲率）/ pose（几何）。
   */
  void setTrajectory(const nav_interfaces::msg::RobotTrajectory & trajectory) override;

  /**
   * @brief 计算速度指令。对应 dcm computeVelocityCommands(Twist&)。
   *        用入参 pose 替代内部 getRobotPose、velocity 替代 odom helper。
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  /**
   * @brief 限制最大线速度（与轨迹速度取严）。dcm 无对应实现。
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  /**
   * @brief 目标是否到达。保留 dcm 行为（恒返回 false，由上层状态机判定）。
   */
  bool isGoalReached();

  /**
   * @brief 设置倒车使能（保留 dcm 倒车逻辑）。
   */
  void setReverseEnabled(const bool enabled_reverse) {reverse_enabled_ = enabled_reverse;}

private:
  double CalculateMean(const std::deque<double> & data);
  double CalculateVariance(const std::deque<double> & data, double mean);

  inline void regularizeAngle(double & angle)
  {
    angle = angle - 2.0 * M_PI * std::floor((angle + M_PI) / (2.0 * M_PI));
  }

  /**
   * @brief 纵向 PID（station + speed 串级）。对应 dcm LinearSpeedControl。
   * @param curr_lin_vel 当前线速度（来自 computeVelocityCommands 的 velocity 入参）
   * @param e_dist       base 系下的纵向误差
   */
  double LinearSpeedControl(double curr_lin_vel, double e_dist);

  /**
   * @brief 角速度 PID。对应 dcm AngularSpeedControl。
   * @param e_theta 当前与目标航向角误差
   */
  double AngularSpeedControl(double e_theta);

  /**
   * @brief 找到轨迹上距机器人最近的点 ID（移植自 nav_util::getClosestWaypointID 的 dcm_msgs::Path 重载，
   *        改为操作缓存的 nav_msgs::msg::Path 几何）。
   */
  int getClosestWaypointID(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & pose,
    double ignore_dist = 0.0,
    bool ignore_first = false) const;

  // ---- ROS2 上下文 ----
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("PureRotationController")};
  rclcpp::Clock::SharedPtr clock_;
  std::string plugin_name_;

  // ---- 状态 ----
  bool initialized_ = false;
  bool goal_reached_ = false;
  bool reverse_enabled_ = false;

  int plan_index_ = 0;

  // 缓存的轨迹：几何 + 每点期望速度（longitudinal_velocity）/ 曲率（path_kappa）
  nav_msgs::msg::Path controller_path_;          // map 系下几何
  std::vector<double> point_speed_limits_;       // 每点期望速度（与 controller_path_.poses 对齐）
  std::vector<double> point_kappas_;             // 每点曲率（与 controller_path_.poses 对齐）

  geometry_msgs::msg::PoseStamped target_ps_;    // 当前目标点（map 系）
  geometry_msgs::msg::PoseStamped current_ps_;   // 当前位姿（map 系）

  // ---- 参数 ----
  double long_precision_ = 0.02;
  double lat_precision_ = 0.04;
  double o_precision_ = 0.02;   // 更严（0.02）
  double d_t_ = 0.05;           // 控制周期 = 1 / controller_frequency

  double max_v_ = 0.5, min_v_ = 0.0, max_v_inc_ = 0.5;       // 线速度
  double max_w_ = 1.57, min_w_ = 0.0, max_w_inc_ = 1.57;     // 角速度

  double goal_x_ = 0.0, goal_y_ = 0.0, goal_yaw_ = 0.0;

  std::deque<double> theta_errors_;
  std::deque<double> longitude_errors_;

  std::mutex plan_mutex_;

  // 纵向 PID（station + speed 串级）
  nav2_controller_common::PIDController station_nav2_pid_controller_;
  nav2_controller_common::PIDController speed_nav2_pid_controller_;
  nav2_controller_common::PidConf station_pid_conf_;
  nav2_controller_common::PidConf speed_pid_conf_;

  // 角速度 PID
  nav2_controller_common::PIDController angular_nav2_pid_controller_;
  nav2_controller_common::PidConf angular_pid_conf_;

  double desired_linear_vel_ = 0.0;
  double acc_max_ = 0.5;

  double speed_limit_ = std::numeric_limits<double>::max();   // setSpeedLimit 保存的上限（绝对值 m/s）

  bool enable_lon_adjust_ = false;   // 纵向距离调整使能；false = 纯旋转（只转）
};

}  // namespace nav2_pure_rotation_controller

#endif  // PURE_ROTATION_CONTROLLER__PURE_ROTATION_CONTROLLER_HPP_
