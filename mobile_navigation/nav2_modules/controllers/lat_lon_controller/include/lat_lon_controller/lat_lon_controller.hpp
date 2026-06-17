/***********************************************************
 *
 * @file: lat_lon_controller.hpp
 * @brief: 纵横解耦 PID 跟踪 controller，移植为 nav2_core::Controller 插件。
 *         横向 PID 纠偏 + 纵向 PID 前进；五阶段状态机
 *         APPROACHING → ADJUST_HEADING → ADJUST_DISTANCE → ROTATING_TO_GOAL；
 *         p_window 前视。算法（状态机 / PID / 参数）完整保留自 dcm 蓝本，
 *         仅替换 ROS1 → ROS2 边界。
 * @author: Yang Haodong, Guo Zhanyu, Wu Maojia（原 dcm 版本）
 *
 **********************************************************/

#ifndef LAT_LON_CONTROLLER__LAT_LON_CONTROLLER_HPP_
#define LAT_LON_CONTROLLER__LAT_LON_CONTROLLER_HPP_

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_interfaces/msg/robot_trajectory.hpp"

#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "controller_common/pid_controller.h"

namespace lat_lon_controller
{

/**
 * @brief 纵横解耦 PID 局部跟踪 controller（nav2_core::Controller 插件）。
 */
class LatLonController : public nav2_core::Controller
{
public:
  LatLonController() = default;
  ~LatLonController() override = default;

  /**
   * @brief 配置插件（替代 ROS1 initialize）。声明参数 + 初始化 PID。
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  /**
   * @brief 几何路径输入（无速度信息）。将 Path 缓存为内部轨迹，速度回退到 max_v_。
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief 带速度时空轨迹输入（五层架构 ④SpeedPlanner 输出）。
   *        消费 per-point longitudinal_velocity 作为期望速度、path_kappa 作为曲率。
   */
  void setTrajectory(const nav_interfaces::msg::RobotTrajectory & trajectory) override;

  /**
   * @brief 计算速度指令。pose = 当前位姿（global/map frame），
   *        velocity = 当前机体速度（替代 odom helper）。状态机 / PID 逻辑保持。
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  /**
   * @brief 限制最大线速度（与轨迹速度取严）。
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  /**
   * @brief 目标是否到达（保留为类方法）。
   */
  bool isGoalReached();

  /**
   * @brief 倒车使能（保留为 public 方法，倒车逻辑保留）。
   */
  void setReverseEnabled(const bool enabled_reverse) {reverse_enabled_ = enabled_reverse;}

private:
  // ── 单条缓存轨迹点（几何 + 期望速度），统一几何路径与时空轨迹两种输入 ──
  struct TrajPoint
  {
    geometry_msgs::msg::PoseStamped pose;
    double speed_limit{0.0};  // 期望速度 (m/s)
  };

  double calculateMean(const std::deque<double> & data);
  double calculateVariance(const std::deque<double> & data, double mean);

  inline void regularizeAngle(double & angle)
  {
    angle = angle - 2.0 * M_PI * std::floor((angle + M_PI) / (2.0 * M_PI));
  }

  // 在缓存轨迹（map frame）中找到距当前位姿最近点；移植自 nav_util::getClosestWaypointID
  int getClosestWaypointID(
    const std::vector<TrajPoint> & traj,
    const geometry_msgs::msg::PoseStamped & pose,
    double ignore_dist = 0.0,
    bool ignore_first = false);

  // 裁剪机器人后方的轨迹点；移植自 dcm pruneGlobalPlan
  bool pruneGlobalPlan(
    const geometry_msgs::msg::PoseStamped & global_pose,
    std::vector<TrajPoint> & traj,
    double dist_behind_robot);

  bool shouldRotateToPath(
    const geometry_msgs::msg::PoseStamped & carrot_pose, double & angle_to_path);

  void rotateToHeading(
    double & linear_vel, double & angular_vel,
    const double & angle_to_path, const nav_msgs::msg::Odometry & curr_odom);

  /**
   * @brief 纵向 PID（线速度）。
   */
  double linearSpeedControl(const nav_msgs::msg::Odometry & base_odometry, double e_dist);

  /**
   * @brief 横向 / 朝向 PID（角速度）。
   */
  double angularSpeedControl(const nav_msgs::msg::Odometry & base_odometry, double e_theta);

  // 把 nav_msgs::msg::Path / RobotTrajectory 缓存为统一 TrajPoint 序列（变换到 map frame）
  void cacheTrajectory(const std::vector<TrajPoint> & input);

private:
  enum ControllerStage
  {
    FINISH,
    APPROACHING,
    ADJUST_HEADING,
    ADJUST_DISTANCE,
    ROTATING_TO_GOAL
  };

  // ── ROS2 句柄 ──
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("LatLonController")};
  rclcpp::Clock::SharedPtr clock_;
  std::string plugin_name_;
  std::string base_frame_;
  std::string global_frame_{"map"};

  bool initialized_{false};
  bool goal_reached_{false};
  bool reverse_enabled_{false};

  // ── 轨迹缓存（map frame）──
  int plan_index_{0};
  std::vector<TrajPoint> controller_traj_;
  geometry_msgs::msg::PoseStamped target_ps_;
  geometry_msgs::msg::PoseStamped current_ps_;  // current pose (map frame)

  double p_window_{0.5};                 // next point distance (前视)
  double long_precision_{0.02};
  double lat_precision_{0.04};
  double o_precision_{0.5};              // goal reached tolerance
  double d_t_{0.1};                       // control time step

  double max_v_{0.5}, min_v_{0.0}, max_v_inc_{0.5};   // linear velocity
  double max_w_{1.57}, min_w_{0.0}, max_w_inc_{1.57}; // angular velocity

  double k_theta_{0.5};                   // pid controller params

  double goal_x_{0.0}, goal_y_{0.0};
  double goal_yaw_{0.0};

  double speed_limit_{std::numeric_limits<double>::infinity()};  // setSpeedLimit 上限

  ControllerStage curr_controller_stage_{FINISH};
  ControllerStage prev_controller_stage_{FINISH};

  std::deque<double> theta_errors_;
  std::deque<double> longitude_errors_;

  std::mutex plan_mutex_;

  // ── PID（来自 controller_common，已移植）──
  controller_common::PIDController station_pid_controller_;
  controller_common::PIDController speed_pid_controller_;

  controller_common::PidConf high_speed_station_pid_conf_;
  controller_common::PidConf low_speed_station_pid_conf_;
  controller_common::PidConf high_speed_pid_conf_;
  controller_common::PidConf low_speed_pid_conf_;
  controller_common::PidConf reverse_station_pid_conf_;
  controller_common::PidConf reverse_speed_pid_conf_;

  controller_common::PIDController angular_station_pid_controller_;
  controller_common::PidConf angular_station_pid_conf_;

  double switch_speed_{0.1};
  double desired_linear_vel_{0.0};
  double acc_max_{0.5};
};

}  // namespace lat_lon_controller

#endif  // LAT_LON_CONTROLLER__LAT_LON_CONTROLLER_HPP_
