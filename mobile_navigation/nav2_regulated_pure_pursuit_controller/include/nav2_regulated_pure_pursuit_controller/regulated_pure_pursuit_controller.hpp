// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__REGULATED_PURE_PURSUIT_CONTROLLER_HPP_
#define NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__REGULATED_PURE_PURSUIT_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_controller_common/pid_controller.h"
#include "nav_interfaces/msg/robot_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

namespace nav2_regulated_pure_pursuit_controller
{

/**
 * @class nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
 * @brief Regulated pure pursuit controller plugin
 */
class RegulatedPurePursuitController : public nav2_core::Controller
{
public:
  /**
   * @brief Constructor for nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
   */
  RegulatedPurePursuitController() = default;

  /**
   * @brief Destrructor for nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
   */
  ~RegulatedPurePursuitController() override = default;

  /**
   * @brief Configure controller state machine
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup controller state machine
   */
  void cleanup() override;

  /**
   * @brief Activate controller state machine
   */
  void activate() override;

  /**
   * @brief Deactivate controller state machine
   */
  void deactivate() override;

  /**
   * @brief Compute the best command given the current pose and velocity, with possible debug information
   *
   * Same as above computeVelocityCommands, but with debug results.
   * If the results pointer is not null, additional information about the twists
   * evaluated will be in results after the call.
   *
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param goal_checker   Ptr to the goal checker for this task in case useful in computing commands
   * @return          Best command
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * /*goal_checker*/) override;

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /**
   * @brief Transforms global plan into same frame as pose and clips poses ineligible for lookaheadPoint
   * Points ineligible to be selected as a lookahead point if they are any of the following:
   * - Outside the local_costmap (collision avoidance cannot be assured)
   * @param pose pose to transform
   * @return Path in new frame
   */
  nav_msgs::msg::Path transformGlobalPlan(
    const geometry_msgs::msg::PoseStamped & pose,
    geometry_msgs::msg::PoseStamped * robot_pose_in_plan = nullptr);

  /**
   * @brief Get lookahead distance
   * @param cmd the current speed to use to compute lookahead point
   * @return lookahead distance
   */
  double getLookAheadDistance(const geometry_msgs::msg::Twist &);

  /**
   * @brief Creates a PointStamped message for visualization
   * @param carrot_pose Input carrot point as a PoseStamped
   * @return CarrotMsg a carrot point marker, PointStamped
   */
  std::unique_ptr<geometry_msgs::msg::PointStamped> createCarrotMsg(
    const geometry_msgs::msg::PoseStamped & carrot_pose);

  /**
   * @brief Whether robot should rotate to rough path heading. Hysteresis: enters
   *        rotation at rotate_to_path_entry_angle_, only exits once the angle
   *        drops below the smaller rotate_to_path_exit_angle_ -- prevents
   *        mode-flapping when the angle hovers near the entry threshold.
   * @param carrot_pose current lookahead point
   * @param angle_to_path Angle of robot output relatie to carrot marker
   * @return Whether should rotate to path heading
   */
  bool shouldRotateToPath(
    const geometry_msgs::msg::PoseStamped & carrot_pose, double & angle_to_path);

  /**
   * @brief Whether robot should rotate to final goal orientation
   * @param carrot_pose current lookahead point
   * @return Whether should rotate to goal heading
   */
  bool shouldRotateToGoalHeading(const geometry_msgs::msg::PoseStamped & carrot_pose);

  /**
   * @brief Create a smooth and kinematically smoothed rotation command. Shared by
   *        both rotate_to_path and rotate_to_goal call sites -- each passes its own
   *        angular_vel_max/angular_kp so the two modes stay independently tunable.
   *        Angular command is a plain P controller on the angle error, clamped to
   *        angular_vel_max, then further clamped by kinematic feasibility.
   * @param linear_vel linear velocity
   * @param angular_vel angular velocity
   * @param angle_to_path Angle of robot output relatie to carrot marker
   * @param curr_speed the current robot speed
   * @param angular_vel_max saturation ceiling for this call site
   * @param angular_kp proportional gain on angle_to_path for this call site
   */
  void rotateToHeading(
    double & linear_vel, double & angular_vel,
    const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed,
    double angular_vel_max, double angular_kp);

  /**
   * @brief Whether collision is imminent
   * @param robot_pose Pose of robot
   * @param carrot_pose Pose of carrot
   * @param linear_vel linear velocity to forward project
   * @param angular_vel angular velocity to forward project
   * @param carrot_dist Distance to the carrot for PP
   * @return Whether collision is imminent
   */
  bool isCollisionImminent(
    const geometry_msgs::msg::PoseStamped &,
    const double &, const double &,
    const double &);

  /**
   * @brief checks for collision at projected pose
   * @param x Pose of pose x
   * @param y Pose of pose y
   * @param theta orientation of Yaw
   * @return Whether in collision
   */
  bool inCollision(
    const double & x,
    const double & y,
    const double & theta);
  /**
   * @brief Cost at a point
   * @param x Pose of pose x
   * @param y Pose of pose y
   * @return Cost of pose in costmap
   */
  double costAtPose(const double & x, const double & y);

  double approachVelocityScalingFactor(
    const nav_msgs::msg::Path & path
  ) const;

  void applyApproachVelocityScaling(
    const nav_msgs::msg::Path & path,
    double & linear_vel
  ) const;

  /**
   * @brief apply regulation constraints to the system
   * @param linear_vel robot command linear velocity input
   * @param lookahead_dist optimal lookahead distance
   * @param curvature curvature of path
   * @param speed Speed of robot
   * @param pose_cost cost at this pose
   * @param have_traj 是否有轨迹规划速度可用——为真时终点收尾减速已由轨迹逐点
   *   编码，跳过 applyApproachVelocityScaling 避免与轨迹自带的收尾曲线复利叠加
   */
  void applyConstraints(
    const double & curvature, const geometry_msgs::msg::Twist & speed,
    const double & pose_cost, const nav_msgs::msg::Path & path, bool have_traj,
    double & linear_vel, double & sign);

  /**
   * @brief Find the intersection a circle and a line segment.
   * This assumes the circle is centered at the origin.
   * If no intersection is found, a floating point error will occur.
   * @param p1 first endpoint of line segment
   * @param p2 second endpoint of line segment
   * @param r radius of circle
   * @return point of intersection
   */
  static geometry_msgs::msg::Point circleSegmentIntersection(
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2,
    double r);

  /**
   * @brief Get lookahead point
   * @param lookahead_dist Optimal lookahead distance
   * @param path Current global path
   * @return Lookahead point
   */
  geometry_msgs::msg::PoseStamped getLookAheadPoint(const double &, const nav_msgs::msg::Path &);

  /**
   * @brief checks for the cusp position
   * @param pose Pose input to determine the cusp position
   * @return robot distance from the cusp
   */
  double findVelocitySignChange(const nav_msgs::msg::Path & transformed_plan);

  /**
   * Get the greatest extent of the costmap in meters from the center.
   * @return max of distance from center in meters to edge of costmap
   */
  double getCostmapMaxExtent() const;

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  /// 缓存速度规划轨迹（保留原始 plan frame；每控制周期重新下发，陈旧窗 ≤1 周期）。
  void setTrajectory(const nav_interfaces::msg::RobotTrajectory & trajectory) override;

  /// 最近点起前瞻窗内的最大规划速度（起点状态戳为当前速度，静止时
  /// 精确最近点恒 0 会自引用死锁——前瞻破环）；traj_accel 输出与返回速度
  /// 同一轨迹点的规划加速度，供速度环前馈使用。
  bool nearestTrajectorySpeed(
    const geometry_msgs::msg::PoseStamped & pose, double & traj_speed, double & traj_accel);

  /// 速度环：反馈（PID）+ 前馈（规划加速度）→ 限幅 → 积分 → 不越过目标。
  /// 纯反馈跟踪一个持续下降的斜坡目标时存在稳态滞后（≈斜率/kp）；前馈项
  /// 直接告诉控制器"此刻规划期望的加速度"，把滞后压到反馈只需修正的残差。
  double trackLinearVelocity(double target_vel, double measured_vel, double feedforward_accel);

  /// 机器人到最近路径段的垂距（cross-track error，机器人系）。
  double computeCrossTrackError(const nav_msgs::msg::Path & transformed_plan) const;

  /// ⑤ 反馈约束（只降不升）：横向误差减速 + 朝向误差减速 + 角速度饱和联动降速
  /// （角速度饱和检查取远端/近场双点曲率中更尖锐的一个，避免远端曲率被前瞻
  /// 距离稀释而漏看近场路径突变）。
  void applyTrackingFeedbackConstraints(
    const nav_msgs::msg::Path & transformed_plan, double curvature, double near_curvature,
    double angle_to_heading, double & linear_vel, double & cross_track_error);

  // ---- 管线基础设施 ----
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::Logger logger_ {rclcpp::get_logger("RegulatedPurePursuitController")};
  rclcpp::Clock::SharedPtr clock_;
  double control_duration_;
  nav_msgs::msg::Path global_plan_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_path_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>>
  carrot_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> carrot_arc_pub_;
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>
  collision_checker_;
  std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  // ---- 前瞻与轨迹对接：carrot 点怎么找、期望速度从哪来 ----
  double lookahead_dist_;
  double min_lookahead_dist_;
  double max_lookahead_dist_;
  double lookahead_time_;
  bool use_velocity_scaled_lookahead_dist_;
  bool use_interpolation_;
  double max_robot_pose_search_dist_;
  tf2::Duration transform_tolerance_;
  double angular_gain_{1.0};
  double near_curvature_check_dist_{0.3};
  struct TrajSpeedPoint
  {
    double x;
    double y;
    double speed;
    double accel;
  };
  std::vector<TrajSpeedPoint> traj_speed_points_;
  std::string trajectory_frame_;
  bool trajectory_required_{false};
  bool trajectory_valid_{false};
  std::mutex traj_mutex_;
  double traj_speed_lookahead_{0.1};

  // ---- RPP 先验约束（applyConstraints，只降不升，基于静态几何/代价）----
  bool use_regulated_linear_velocity_scaling_;
  double regulated_linear_scaling_min_radius_;
  double regulated_linear_scaling_min_speed_;
  bool use_cost_regulated_linear_velocity_scaling_;
  double cost_scaling_dist_;
  double cost_scaling_gain_;
  double inflation_cost_scaling_factor_;
  double min_approach_linear_velocity_;
  double approach_velocity_scaling_dist_;

  // ---- RPP 反馈约束 ①②③（applyTrackingFeedbackConstraints，基于运行时实际状态）----
  double lateral_error_threshold_{0.15};
  double lateral_error_max_dist_{0.5};
  double min_lateral_speed_scale_{0.3};
  double heading_error_scaling_min_angle_{0.35};
  // ②号 taper 区间上界：独立于 rotate_to_path_entry_angle_——两者语义不同
  // （前者是"减速衰减到头"的角度，后者是"进入原地转模式"的角度），复用
  // 同一个值会让调其中一个时意外改变另一个的生效范围。
  double heading_error_scaling_max_angle_{0.785};
  double min_heading_speed_scale_{0.3};
  double max_angular_vel_{1.5};

  // ---- 速度环（trackLinearVelocity，恒生效）----
  double max_linear_accel_{0.5};
  nav2_controller_common::PIDController speed_pid_;
  rclcpp::Time last_speed_track_time_{0, 0, RCL_ROS_TIME};
  // 速度环指令积分器：锚定上一周期指令而非实测——实测只进误差项，
  // 锚定实测会让指令每周期被拉回 plant 的滞后响应（减速互锁、超速泄放
  // 被钳制）。与实测偏差超过 reseed 阈值（外力/急停/打滑）时重新对齐。
  bool last_speed_cmd_valid_{false};
  double last_speed_cmd_{0.0};
  double speed_cmd_reseed_error_{0.3};
  double desired_linear_vel_, base_desired_linear_vel_;

  // ---- 离散转向模式 ----
  // rotate_to_path（shouldRotateToPath 触发，中途朝向修正）与 rotate_to_goal
  // （shouldRotateToGoalHeading 触发，终点原地转对齐目标朝向）参数完全独立：
  // 两者物理场景不同（前者是行进中的小角度修正，后者可能是到达终点后的
  // 大角度掉头），共用同一套角速度/角度参数会导致调一个连带影响另一个。
  // 角速度命令是纯 P 控制（kp·angle_to_path，饱和于 angular_vel_max）——
  // entry_angle 只管 shouldRotateToPath 的进入/退出判定，不再兼作 P 控制
  // 的参考角，两件事彻底分开。
  bool use_rotate_to_heading_;
  double max_angular_accel_;  // 两者共用：机器人角加速度物理上限，非行为调参
  double goal_dist_tol_;      // 不是声明参数，每周期从 goal_checker 容差读取
  // rotate_to_path 专用：
  double rotate_to_path_entry_angle_;   // shouldRotateToPath 进入阈值
  double rotate_to_path_exit_angle_{0.35};  // 滞回退出阈值（须 < 进入阈值）
  double rotate_to_path_angular_vel_;   // 角速度饱和上限
  double rotate_to_path_angular_kp_{1.0};   // P 控制增益
  bool was_rotating_to_path_{false};
  // rotate_to_goal 专用（终点原地转对齐目标朝向）：
  double rotate_to_goal_angular_vel_{1.8};  // 角速度饱和上限，与 rotate_to_path 独立
  double rotate_to_goal_angular_kp_{1.0};   // P 控制增益，与 rotate_to_path 独立

  // ---- 碰撞检测 ----
  bool use_collision_detection_;
  double max_allowed_time_to_collision_up_to_carrot_;

  // ---- 其他 ----
  bool allow_reversing_;
};

}  // namespace nav2_regulated_pure_pursuit_controller

#endif  // NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__REGULATED_PURE_PURSUIT_CONTROLLER_HPP_
