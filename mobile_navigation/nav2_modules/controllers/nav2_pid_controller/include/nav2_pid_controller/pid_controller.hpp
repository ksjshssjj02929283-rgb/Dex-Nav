// Copyright (c) 2023 Guo Linfeng (original ROS1 dcm_navigation)
// Copyright (c) 2026 dexforce (ROS2 / nav2_core port)
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

#ifndef NAV2_PID_CONTROLLER__PID_CONTROLLER_HPP_
#define NAV2_PID_CONTROLLER__PID_CONTROLLER_HPP_

#include <algorithm>
#include <atomic>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "nav2_core/controller.hpp"
#include "nav2_core/end_localization.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_core/trajectory_preview.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/node_utils.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_interfaces/msg/robot_trajectory.hpp"

#include "tf2_ros/buffer.h"

#include "nav2_controller_common/pid_controller.h"
#include "nav2_pid_controller/end_localization_lock.hpp"

namespace nav2_pid_controller
{

/**
 * @class nav2_pid_controller::PIDController
 * @brief Nav2 PID 局部控制器。
 *
 * 接近阶段跟踪规划速度，末端依次执行可选横向优化、纵向微调和停稳。
 * 到位标准由 GoalChecker 决定；同目标轨迹更新保留末端状态。
 * setTrajectory() 接收带速度的轨迹，setPlan() 接收普通几何路径。
 */
class PIDController
  : public nav2_core::Controller,
  public nav2_core::EndLocalizationCapability,
  public nav2_core::TrajectoryTrackingReceiptCapability
{
public:
  PIDController() = default;
  ~PIDController() override = default;

  /**
   * @brief Configure controller on bringup
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup controller state
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
   * @brief Set the geometric global plan (no speed information; uses default 0).
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief Set the time-space trajectory (L5 SpeedPlanner output). Consumes
   *        per-point longitudinal_velocity as desired speed and path_kappa as
   *        curvature, caching geometry + speed for computeVelocityCommands.
   * @param trajectory The robot trajectory
   */
  void setTrajectory(const nav_interfaces::msg::RobotTrajectory & trajectory) override;

  bool getLastTrajectoryTrackingTarget(
    double & velocity, double & acceleration) override;
  bool getLastTrajectoryTrackingReceipt(
    double & velocity, double & acceleration, double & relative_time) override;

  /**
   * @brief Compute the best command given the current pose and velocity
   * @param pose Current robot pose (in costmap global frame)
   * @param velocity Current robot velocity (in base frame)
   * @param goal_checker Goal checker for this task
   * @return Best command for the robot to drive
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  /**
   * @brief Limit the maximum linear speed of the robot.
   * @param speed_limit absolute (m/s) or percentage of max speed
   * @param percentage true if speed_limit is a percentage
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  /**
   * @brief Check if the goal pose has been achieved
   * @return True if achieved, false otherwise
   */
  bool isGoalReached();

  /**
   * @brief Enable / disable reverse (backward) driving.
   * @param enabled true to drive in reverse
   */
  void setReverseEnabled(const bool enabled) {reverse_enabled_ = enabled;}

  // nav2_core::EndLocalizationCapability：由ControllerServer传入权威终点并读取锁存结果。
  void resetEndLocalization(uint64_t goal_generation) override;
  void setEndLocalizationGoal(
    const geometry_msgs::msg::PoseStamped & live_goal,
    uint64_t goal_generation) override;
  nav2_core::EndLocalizationSnapshot getEndLocalizationSnapshot() const override;

protected:
  /**
   * @brief Single cached trajectory point: geometry + segment speed/curvature.
   */
  struct TrajPoint : public nav2_core::TrajectoryPreviewPoint
  {
    geometry_msgs::msg::PoseStamped pose;
    double curvature = 0.0;    // path_kappa
    std::string region_type;
  };

  /// PID内部互斥控制阶段；强类型枚举避免与整数或其他状态误比较。
  enum class ControllerStage
  {
    FINISH,
    APPROACHING,
    ROTATING_TO_GOAL,
    RETRY_ALIGN_TO_POSITION,
    RETRY_TRANSLATE
  };

    /// 子阶段单向推进，避免软目标附近的噪声触发反复优化。
    enum class FinalRotationPhase { LATERAL_REFINE, LONGITUDINAL_TRIM, SETTLING };

    /// 本周期的控制动作。
    enum class FinalRotationAction { HOLD_POSITION, TRANSLATE, RETRY_POSITION, INVALID };

    /// 复用本周期输入，决策时不再查询 TF。
    struct FinalRotationInput {
        double longitudinal_error = 0.0;
        double lateral_error = 0.0;
        double yaw_error = 0.0;
        double linear_velocity = 0.0;
        double angular_velocity = 0.0;
        double longitudinal_tolerance = 0.0;
        double lateral_tolerance = 0.0;
        double stopped_linear_velocity = 0.0;
        bool is_goal_valid = false;
        bool is_position_in_tolerance = false;
        bool is_position_in_hold = false;
    };

    struct FinalRotationDecision {
        FinalRotationAction action = FinalRotationAction::HOLD_POSITION;
        double heading_error = 0.0;
        double linear_error = 0.0;
        double linear_deadband = 0.0;
        bool should_use_point_heading = false;
        const char* reason = "none";
        // 仅诊断用，记录本周期结束横向优化的原因。
        const char* refinement_exit_reason = "none";
    };

  static inline void regularizeAngle(double & angle)
  {
    angle = angle - 2.0 * M_PI * std::floor((angle + M_PI) / (2.0 * M_PI));
  }

  double CalculateMean(const std::deque<double> & data);
  double CalculateVariance(const std::deque<double> & data, double mean);

    static const char* toString(FinalRotationPhase phase) {
        switch (phase) {
            case FinalRotationPhase::LATERAL_REFINE:
                return "LATERAL_REFINE";
            case FinalRotationPhase::LONGITUDINAL_TRIM:
                return "LONGITUDINAL_TRIM";
            case FinalRotationPhase::SETTLING:
                return "SETTLING";
            default:
                return "UNKNOWN";
        }
    }

    static const char* toString(FinalRotationAction action) {
        switch (action) {
            case FinalRotationAction::HOLD_POSITION:
                return "HOLD_POSITION";
            case FinalRotationAction::TRANSLATE:
                return "TRANSLATE";
            case FinalRotationAction::RETRY_POSITION:
                return "RETRY_POSITION";
            case FinalRotationAction::INVALID:
                return "INVALID";
            default:
                return "UNKNOWN";
        }
    }

    /// 小角度下预测纵向移动后的横向误差。
    static double predictedFinalLateralError(double lateral_error, double longitudinal_travel, double yaw_error) {
        return lateral_error + longitudinal_travel * std::tan(yaw_error);
    }

    /// 标量决策，不分配内存或查询 TF。
    FinalRotationDecision decideFinalRotation(const FinalRotationInput& input);
    void computeFinalRotationCommand(const FinalRotationInput& input,
                                     const FinalRotationDecision& decision,
                                     const geometry_msgs::msg::Twist& velocity,
                                     geometry_msgs::msg::Twist& command);
    /// 按减速度制动；无效输入立即归零。
    double brakeFinalLinearControl(const geometry_msgs::msg::Twist& velocity);
    /// 配置时校验软目标、死区和角度门槛。
    void validateFinalRotationParameters();

  /**
   * @brief Find the closest waypoint id on the cached trajectory.
   *        Ported from nav_util::getClosestWaypointID.
   */
  int getClosestWaypointID(
    const std::vector<TrajPoint> & traj,
    const geometry_msgs::msg::PoseStamped & pose,
    double ignore_dist = 0.0,
    bool ignore_first = false);

  /**
   * @brief Erase trajectory points behind the robot (within dist_behind_robot).
   */
  bool pruneGlobalPlan(
    const geometry_msgs::msg::PoseStamped & global_pose,
    std::vector<TrajPoint> & traj,
    double dist_behind_robot);

  /**
   * @brief Speed reference decoupled from the steering target index：genuine
   *        nearest-point search over controller_traj_（不受 final_target_lock_
   *        distance_ 的"距终点内直接锁定终点索引"影响），forward-window max
   *        excluding the nearest point itself（同 RPP，避免状态戳自引用）。
   */
  bool nearestTrajectorySpeed(
    const geometry_msgs::msg::PoseStamped & pose, double & traj_speed, double & traj_accel);

  bool shouldRotateToPath(
    const geometry_msgs::msg::PoseStamped & carrot_pose,
    double & angle_to_path);

  void rotateToHeading(
    double & linear_vel, double & angular_vel,
    const double & angle_to_path,
    const geometry_msgs::msg::Twist & curr_vel);

  /**
   * @brief Linear control: position error generates a signed target speed,
   *        then the speed PID tracks that target.
   * @param velocity current robot velocity (base frame)
   * @param e_dist signed longitudinal error used by the current control stage
   * @return linear velocity command
   */
  double LinearSpeedControl(
    const geometry_msgs::msg::Twist & velocity,
    double e_dist,
    double max_linear_speed = std::numeric_limits<double>::quiet_NaN(),
    double min_target_speed = 0.0,
    double position_deadband = 0.0,
    double feedforward_accel = 0.0);

  /**
   * @brief 检查目标系纵向、横向和径向误差；margin 仅用于保持窗口。
   */
  static bool isWithinPositionWindow(
    double longitudinal_error, double lateral_error, double radial_error,
    double longitudinal_tolerance, double lateral_tolerance, double margin = 0.0)
  {
    const double safe_margin = std::max(0.0, margin);
    const double longitudinal_limit = std::fabs(longitudinal_tolerance) + safe_margin;
    const double lateral_limit = std::fabs(lateral_tolerance) + safe_margin;
    const double radial_limit =
      std::max(std::fabs(longitudinal_tolerance), std::fabs(lateral_tolerance)) + safe_margin;
    return std::isfinite(longitudinal_error) && std::isfinite(lateral_error) &&
           std::isfinite(radial_error) &&
           std::fabs(longitudinal_error) < longitudinal_limit &&
           std::fabs(lateral_error) < lateral_limit &&
           std::fabs(radial_error) < radial_limit;
  }

  /**
   * @brief 横向合格时保持最终航向，否则选择转角较小的点位修正方向。
   * @param should_use_point_heading 强制对准目标点，避免重试重复不可行的微调。
   */
  static double selectPositionRetryHeading(
    double current_yaw, double goal_yaw, double goal_dx, double goal_dy,
    double lateral_tolerance, bool & reverse, bool & holds_goal_yaw,
    bool should_use_point_heading = false);

  bool confirmPositionRetryCapture(
    bool position_in_tolerance, double goal_yaw,
    double linear_velocity, double stopped_velocity);

  void beginPositionRetry(
    double current_yaw, double goal_yaw, double goal_dx, double goal_dy,
    double goal_distance, double lateral_tolerance,
    bool should_use_point_heading = false);

  /// 重置末端阶段和切入方向。
  void resetFinalRotationPhase();

  /**
   * @brief Angular PID control.
   * @param velocity current robot velocity (base frame)
   * @param e_theta heading error
   * @return angular velocity command
   */
  double AngularSpeedControl(
    const geometry_msgs::msg::Twist & velocity,
    double e_theta,
    bool final_rotation = false);

  /**
   * @brief Human-readable name for controller stage logging.
   */
  const char * controllerStageName(ControllerStage stage) const;

  /**
   * @brief 更新权威末端位置参考，并选择本周期控制目标。
   *
   * 权威目标暂时无效时允许轨迹终点作为控制回退值，但绝不写入锁存器。
   */
  nav2_core::EndLocalizationSnapshot selectEndLocalizationPositionReference(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::PoseStamped & trajectory_fallback_goal,
    geometry_msgs::msg::PoseStamped & effective_goal);

  /**
   * @brief Build the cached trajectory from a geometric path (speed = 0),
   *        transforming each pose into the costmap global frame.
   */
  bool cachePathAsTrajectory(const nav_msgs::msg::Path & path);

  /// 调用方必须持有plan_mutex_；任务切换时统一清理阶段、PID积分和指令基准。
  void resetTaskControlStateLocked(uint64_t goal_generation);

  // ---- ROS2 plumbing ----
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("PIDController")};
  rclcpp::Clock::SharedPtr clock_;
  std::string global_frame_;
  std::string base_frame_;

  // ---- plan / trajectory cache ----
  std::vector<TrajPoint> controller_traj_;
  std::mutex plan_mutex_;
  int plan_index_ = 0;
  geometry_msgs::msg::PoseStamped target_ps_;
  geometry_msgs::msg::PoseStamped current_ps_;

  // ---- tolerances / limits ----
  double long_precision_ = 0.02, lat_precision_ = 0.02, o_precision_ = 0.5;
  double angular_error_deadband_ = 0.008;
  double d_t_ = 0.05;  // control time step (1 / controller_frequency)

  double max_v_ = 0.5, min_v_ = 0.0;
  double max_linear_accel_ = 0.5, max_linear_decel_ = 0.5;
  double max_w_ = 1.57, min_w_ = 0.0, max_w_inc_ = 1.57;  // angular velocity

  double k_theta_ = 0.5;            // heading-weighting factor
  double switch_speed_ = 0.1;       // high/low speed switch threshold
  double default_desired_linear_vel_ = 0.15;  // fallback speed for geometric paths
  double desired_linear_vel_ = 0.0;  // per-point desired velocity
  // 进入最终位置保持使用 GoalChecker 的到点容差；进入后只有误差超过
  // “到点容差 + position_hold_hysteresis_”才退出，避免定位噪声反复切状态。
  double position_hold_hysteresis_ = 0.02;
  // PID末端旋转接近目标航向后，保留受限的纵向位置微调能力。
  double final_rotation_max_v_ = 0.05;
  double final_rotation_linear_deadband_ = 0.005;
  // 横向软优化目标，单位 m；0关闭横向优化，仍保留纵向微调和位置重试。
  double final_axis_lateral_tolerance_ = 0.010;
  double retry_position_max_v_ = 0.05;
  double retry_heading_tolerance_ = 0.03;
  int retry_position_capture_cycles_ = 3;
  double acc_max_ = 0.5;

  // upper speed bound from setSpeedLimit (taken min with trajectory speed)
  double speed_limit_ = std::numeric_limits<double>::infinity();

  bool reverse_enabled_ = false;

  // ---- state machine ----
  ControllerStage curr_controller_stage_ = ControllerStage::FINISH;
  ControllerStage prev_controller_stage_ = ControllerStage::FINISH;
  double retry_heading_ = 0.0;
  bool retry_heading_valid_ = false;
  bool retry_reverse_ = false;
  bool retry_holds_goal_yaw_ = false;
  int retry_position_capture_count_ = 0;
  int retry_position_outside_count_ = 0;
  FinalRotationPhase final_rotation_phase_ = FinalRotationPhase::LATERAL_REFINE;
  bool has_final_refinement_heading_ = false;
  bool is_final_refinement_reverse_ = false;
  double final_refinement_heading_offset_ = 0.0;
  // 位置闭环速度上限锁存：首次进入 5cm~2cm 用速度规划速度过渡，
  // 一旦进入 2cm 内或过冲即锁存为固定 final_rotation_max_v_，避免规划已停后卡住。
  bool approach_refine_limit_latched_ = false;

  std::deque<double> theta_errors_;
  std::deque<double> longitude_errors_;

  double last_linear_position_cmd_ = 0.0;
  double last_linear_target_v_ = 0.0;
  double last_linear_speed_error_ = 0.0;
  double last_linear_accel_cmd_ = 0.0;
  double last_linear_accel_limited_ = 0.0;
  double last_linear_limit_ = 0.0;
  double last_linear_min_target_v_ = 0.0;
  double last_linear_deadband_ = 0.0;

  // ---- speed-loop command integrator ----
  // 速度环输出锚定上一周期指令而非实测（实测只进误差项；锚定实测会让指令
  // 只能贴着执行值走、失去主动制动权）；指令与实测脱节超过 reseed 阈值时
  // 对齐实测（外力/打滑/急停兜底）。
  bool last_speed_cmd_valid_ = false;
  double last_speed_cmd_ = 0.0;
  double speed_cmd_reseed_error_ = 0.3;
  // NEGATIVE_FF_CONSISTENCY_GUARD_BEGIN
  // PID末端低速保护：落后目标时撤销不一致负前馈；实测超速时开放有限
  // 目标下探补刹。高速/RPP/目标0不受影响，保留完整主动制动权。
  double negative_ff_guard_error_{0.01};
  double negative_ff_max_target_speed_{0.10};
  double negative_ff_overspeed_undercut_gain_{0.50};
  double negative_ff_max_target_undercut_{0.03};
  // NEGATIVE_FF_CONSISTENCY_GUARD_END

  // ---- angular-loop command integrator（同上速度环模式）----
  bool last_angular_cmd_valid_ = false;
  double last_angular_cmd_ = 0.0;
  double last_angular_accel_ = 0.0;
  bool last_angular_accel_valid_ = false;
  double angular_cmd_reseed_error_ = 0.3;

  // ---- fixed-goal-yaw PID state ----
  // Final yaw is controlled directly from the fixed goal heading. The command
  // integrator provides the same bounded slew behavior as the robot baseline.

  // ---- approach-stage target lock ----
  // getClosestWaypointID 的 ignore_dist：机器人与轨迹终点距离小于此值时直接
  // 锁定终点索引，跳过最近点搜索。默认与 staged_control.approach_distance
  // 对齐（该控制器只在此距离内被调用），使 approach 全程瞄准固定终点，
  // 不再随机器人平移在候选点间跳变。
  double final_target_lock_distance_ = 0.3;
  // 距终点小于此值切换到位置闭环精修（位置PID生效，速度上限见锁存规则）。
  double final_refinement_distance_ = 0.05;

  // ---- terminal localization reference lock ----
  // 位置和航向阈值分别控制Odom位置锁存与最终航向锁存；0表示关闭对应通道。
  double end_localization_switch_distance_ = 0.0;
  double end_localization_switch_angle_ = 0.0;
  EndLocalizationLock end_localization_lock_;
  uint64_t last_end_localization_revision_{0};
  nav2_core::EndLocalizationStage last_end_localization_stage_{
    nav2_core::EndLocalizationStage::INACTIVE};

  // ---- speed reference lookahead（与上方 target lock 解耦）----
  // nearestTrajectorySpeed 的前方窗长度：最近点之后前方窗内取 max（不含
  // 最近点自身）。同 RPP 参数语义。
  double traj_speed_lookahead_ = 0.1;
  double traj_time_lookahead_ = 0.0;
  bool trajectory_time_valid_ = false;
  bool last_trajectory_tracking_target_valid_ = false;
  double last_trajectory_tracking_target_v_ = 0.0;
  double last_trajectory_tracking_target_a_ = 0.0;
  double last_trajectory_tracking_target_t_ = 0.0;
  double sampled_trajectory_relative_time_ = 0.0;
  std::string last_trajectory_region_type_;

  // ---- approach-stage rotation budget watchdog ----
  // 对机器人实际朝向变化量（非误差）逐周期积分；超预算强制停止。
  // 速度上限按朝向误差连续缩放只是阻尼，不能证明净旋转严格有界，
  // 这里独立兜底定位跳变等未预见场景。

  /**
   * @brief Continuous heading-based speed scale for the approach stage:
   *        1.0 at err_theta=0, 0.0 at |err_theta|>=90 deg. Replaces a hard
   *        on/off velocity gate so there is no command discontinuity while
   *        still damping the LOS bearing-rate blowup (dbeta/dt=v*sin(dtheta)/d)
   *        that drives unbounded orbiting when heading error is large.
   */
  static double headingSpeedScale(double err_theta)
  {
    return std::max(0.0, std::cos(err_theta));
  }

  /**
   * @brief 动态参数回调，目前只处理 verbose —— 调试开关必须能在机器人跑着的
   *        时候关掉/打开，否则要重启节点才能改，等于改不了。
   */
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  /// 0 关闭周期 INFO 诊断；>=1 按 100 ms 节流。原子读写支持动态开关。
  std::atomic<int> verbose_{1};

  // ---- linear PID controllers ----
  nav2_controller_common::PIDController station_nav2_pid_controller_;
  nav2_controller_common::PIDController speed_nav2_pid_controller_;

  nav2_controller_common::PidConf high_speed_station_pid_conf_;
  nav2_controller_common::PidConf low_speed_station_pid_conf_;
  nav2_controller_common::PidConf high_speed_pid_conf_;
  nav2_controller_common::PidConf low_speed_pid_conf_;
  nav2_controller_common::PidConf reverse_station_pid_conf_;
  nav2_controller_common::PidConf reverse_speed_pid_conf_;

  // ---- angular PID controller ----
  nav2_controller_common::PIDController angular_station_nav2_pid_controller_;
  nav2_controller_common::PidConf angular_station_pid_conf_;
};

}  // namespace nav2_pid_controller

#endif  // NAV2_PID_CONTROLLER__PID_CONTROLLER_HPP_
