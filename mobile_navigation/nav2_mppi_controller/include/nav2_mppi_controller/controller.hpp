// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#ifndef NAV2_MPPI_CONTROLLER__CONTROLLER_HPP_
#define NAV2_MPPI_CONTROLLER__CONTROLLER_HPP_

#include <array>
#include <cstddef>
#include <memory>
#include <string>

#include "nav2_mppi_controller/tools/path_handler.hpp"
#include "nav2_mppi_controller/optimizer.hpp"
#include "nav2_mppi_controller/tools/trajectory_visualizer.hpp"
#include "nav2_mppi_controller/models/constraints.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav_interfaces/msg/robot_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace nav2_mppi_controller
{

using namespace mppi;  // NOLINT

/**
 * @class mppi::MPPIController
 * @brief Main plugin controller for MPPI Controller
 */
// 三阶段枚举（消灭 magic index 0/1/2）：数组顺序 = 枚举值
enum Stage : size_t { STAGE_ALIGN = 0, STAGE_TRACK = 1, STAGE_APPROACH = 2 };

class MPPIController : public nav2_core::Controller
{
public:
  /**
    * @brief Constructor for mppi::MPPIController
    */
  MPPIController() = default;

  /**
    * @brief Configure controller on bringup
    * @param parent WeakPtr to node
    * @param name Name of plugin
    * @param tf TF buffer to use
    * @param costmap_ros Costmap2DROS object of environment
    */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
    * @brief Cleanup resources
    */
  void cleanup() override;

  /**
    * @brief Activate controller
    */
  void activate() override;

  /**
    * @brief Deactivate controller
    */
  void deactivate() override;

  /**
    * @brief Reset the controller state between tasks
    */
  void reset();

  /**
    * @brief Main method to compute velocities using the optimizer
    * @param robot_pose Robot pose
    * @param robot_speed Robot speed
    * @param goal_checker Pointer to the goal checker for awareness if completed task
    */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    nav2_core::GoalChecker * goal_checker) override;

  /**
    * @brief Set new reference path to track
    * @param path Path to track
    */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
    * @brief 消费速度规划轨迹（④ 产出）：每个控制周期取最近轨迹点的规划速度，
    *        作为采样器的前向 vx 上限（叠加在三阶段 profile 约束之上）。
    * @param trajectory 带逐点速度的时空轨迹
    */
  void setTrajectory(const nav_interfaces::msg::RobotTrajectory & trajectory) override;

  /**
    * @brief Set new speed limit from callback
    * @param speed_limit Speed limit to use
    * @param percentage Bool if the speed limit is absolute or relative
    */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /**
    * @brief Visualize trajectories
    * @param transformed_plan Transformed input plan
    */
  void visualize(nav_msgs::msg::Path transformed_plan);

  void configureStages();

  // 二期条件层（S1-S4）参数加载，见 controller.cpp configureConditionLayer() 注释。
  void configureConditionLayer();

  // 二期条件层：在 stage_weights（Layer0）+ 迟滞插值（Layer1）之上做第二次修正（Layer2），
  // 每个控制周期都调用，覆盖 optimizer 里的 profile/sampling_std。
  // S1 track 横向误差→PathAlign 权重+限速；S2 approach 横向误差→限速；
  // S3 approach 距离终点越近→采样标准差越小；S4 终点微调子区→线速度硬限制。
  void updateConditionLayer(double goal_distance, double e_lat, double key_lon, double key_lat);

  // 根据路径进度和到目标距离选择三阶段 profile，并把速度/行为代价平滑应用到 optimizer。
  void applyProfileForPathState(double path_progress, double goal_distance, double heading_path_angle);

  size_t selectStage(double goal_distance, double heading_path_angle) const;

  std::string stageName(size_t profile) const;

  MppiProfileSettings interpolateProfiles(
    const MppiProfileSettings & from,
    const MppiProfileSettings & to,
    double ratio) const;

  void updateProfileTransition();

  void publishProfileDebug(double path_progress, double goal_distance);

  /// 最近轨迹点的规划速度（机器人位姿先转到轨迹坐标系）；无轨迹/TF 失败返回 -1（不设帽）。
  double nearestTrajectorySpeed(const geometry_msgs::msg::PoseStamped & robot_pose) const;

  std::string name_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  std::unique_ptr<ParametersHandler> parameters_handler_;
  Optimizer optimizer_;
  PathHandler path_handler_;
  TrajectoryVisualizer trajectory_visualizer_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>>
  profile_debug_publisher_;

  bool visualize_;
  std::array<MppiProfileSettings, 3> stages_;
  MppiProfileSettings applied_profile_;
  MppiProfileSettings transition_start_profile_;
  MppiProfileSettings transition_target_profile_;
  rclcpp::Time profile_transition_start_time_;
  double profile_start_distance_{0.8};
  double approach_distance_{1.0};
  double profile_transition_duration_{0.0};
  double approach_hysteresis_{0.05};
  double align_enter_angle_{0.52};   // 对齐段进入阈值(rad,~30°)
  double align_exit_angle_{0.31};    // 对齐段退出阈值(rad,~18°,迟滞)
  size_t active_stage_{0};
  bool profile_transition_active_{false};

  double reset_period_;
  // Last time computeVelocityCommands was called
  rclcpp::Time last_time_called_;

  // ⭐ costmap 异步解耦：控制线程读快照副本，try_lock 刷新，永不阻塞等 costmap 更新锁
  nav2_costmap_2d::Costmap2D costmap_snapshot_;
  bool costmap_snapshot_initialized_{false};

  // 速度规划轨迹缓存（④ 每控制周期分发；前瞻窗内最大规划速度 → vx 采样上限）
  nav_interfaces::msg::RobotTrajectory reference_trajectory_;
  // 前瞻窗长度（m）：帽允许领先计划加速坡的距离；窗内至少含最近点的下一点（防死锁）
  double traj_speed_cap_lookahead_{0.2};

  // ── 二期条件层（S1-S4）：默认关闭，需 yaml condition_layer.enabled=true 显式开启 ──
  bool condition_layer_enabled_{false};

  // S1 track 阶段：横向误差→PathAlign 权重↑ + vx_max 收紧（约束层限速，DEC-003 Q5）
  double s1_enter_{0.05};          // 进入阈值(m)，约 5cm
  double s1_exit_{0.03};           // 退出阈值(m)，需 < enter，防抖
  double s1_weight_gain_{1.5};     // 横向误差达到 enter 时 PathAlign 权重放大倍数
  double s1_vx_scale_min_{0.5};    // 横向误差达到 enter 时 vx_max 收紧到的比例下限
  bool s1_active_{false};

  // S2 approach 阶段：横向误差→vx_max 收紧（复用 e_lat 与相同的迟滞/增益模式，独立阈值）
  double s2_enter_{0.05};
  double s2_exit_{0.03};
  double s2_vx_scale_min_{0.5};
  bool s2_active_{false};

  // S3 approach 阶段：距离终点越近→采样标准差越小（连续增益，无需迟滞）
  double s3_std_scale_min_{0.3};             // 到终点时缩到 yaml 基线的比例下限
  mppi::models::SamplingStd s3_base_sampling_std_{0.0f, 0.0f, 0.0f};  // configure() 时缓存的 yaml 基线

  // S4 终点微调子区：|key_lon|/|key_lat| 都小于 enter 时硬限制线速度（不是比例缩放）
  double s4_enter_{0.02};          // 进入阈值(m)，2cm
  double s4_exit_{0.03};           // 恢复阈值(m)，需 > enter，防止边界反复横跳
  double s4_vx_clamp_min_{0.02};
  double s4_vx_clamp_max_{0.05};
  bool s4_active_{false};
};

}  // namespace nav2_mppi_controller

#endif  // NAV2_MPPI_CONTROLLER__CONTROLLER_HPP_
