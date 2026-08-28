// Copyright (c) 2019 Intel Corporation
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

#ifndef NAV2_CONTROLLER__CONTROLLER_SERVER_HPP_
#define NAV2_CONTROLLER__CONTROLLER_SERVER_HPP_

#include <atomic>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <mutex>

#include "nav2_core/controller.hpp"
#include "nav2_core/end_localization.hpp"
#include "nav2_core/progress_checker.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_core/local_planner.hpp"
#include "nav2_core/speed_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_speed_planner/core/initial_motion.hpp"
#include "nav_interfaces/msg/robot_trajectory.hpp"
#include "nav_interfaces/msg/navigation_status.hpp"
#include "tf2_ros/transform_listener.h"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "nav_2d_utils/odom_subscriber.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_controller
{

class ProgressChecker;
/**
 * @class nav2_controller::ControllerServer
 * @brief This class hosts variety of plugins of different algorithms to
 * complete control tasks from the exposed FollowPath action server.
 */
class ControllerServer : public nav2_util::LifecycleNode
{
public:
  using ControllerMap = std::unordered_map<std::string, nav2_core::Controller::Ptr>;
  using GoalCheckerMap = std::unordered_map<std::string, nav2_core::GoalChecker::Ptr>;
  using LocalPlannerMap = std::unordered_map<std::string, nav2_core::LocalPlanner::Ptr>;
  using SpeedPlannerMap = std::unordered_map<std::string, nav2_core::SpeedPlanner::Ptr>;
  enum class ControllerPhase { PREPARE, ALIGN, TRACK, APPROACH };
  enum class PrepareState
  {
    CHECK_PENDING,
    BACKING_UP,
    DONE
  };
  enum class PathUpdateType { NEW_GOAL, SAME_GOAL_GEOMETRY };

  /**
   * @brief Constructor for nav2_controller::ControllerServer
   * @param options Additional options to control creation of the node.
   */
  explicit ControllerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief Destructor for nav2_controller::ControllerServer
   */
  ~ControllerServer();

protected:
  /**
   * @brief Configures controller parameters and member variables
   *
   * Configures controller plugin and costmap; Initialize odom subscriber,
   * velocity publisher and follow path action server.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   * @throw pluginlib::PluginlibException When failed to initialize controller
   * plugin
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activates member variables
   *
   * Activates controller, costmap, velocity publisher and follow path action
   * server
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivates member variables
   *
   * Deactivates follow path action server, controller, costmap and velocity
   * publisher. Before calling deactivate state, velocity is being set to zero.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Calls clean up states and resets member variables.
   *
   * Controller and costmap clean up state is called, and resets rest of the
   * variables
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in Shutdown state
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  using Action = nav2_msgs::action::FollowPath;
  using ActionServer = nav2_util::SimpleActionServer<Action>;

  // Our action server implements the FollowPath action
  std::unique_ptr<ActionServer> action_server_;

  /**
   * @brief FollowPath action server callback. Handles action server updates and
   * spins server until goal is reached
   *
   * Provides global path to controller received from action client. Twist
   * velocities for the robot are calculated and published using controller at
   * the specified rate till the goal is reached.
   * @throw nav2_core::PlannerException
   */
  void computeControl();

  /**
   * @brief Find the valid controller ID name for the given request
   *
   * @param c_name The requested controller name
   * @param name Reference to the name to use for control if any valid available
   * @return bool Whether it found a valid controller to use
   */
  bool findControllerId(const std::string & c_name, std::string & name);

  /**
   * @brief Find the valid goal checker ID name for the specified parameter
   *
   * @param c_name The goal checker name
   * @param name Reference to the name to use for goal checking if any valid available
   * @return bool Whether it found a valid goal checker to use
   */
  bool findGoalCheckerId(const std::string & c_name, std::string & name);

  /**
   * @brief Assigns path to controller
   * @param path Path received from action server
   * @param update_type Whether this starts a new goal or only replaces geometry
   *        for the same controller, goal checker, and navigation goal
   */
  void setPlannerPath(
    const nav_msgs::msg::Path & path,
    PathUpdateType update_type);
  /**
   * @brief Commit path bookkeeping after controller/local-planner geometry is updated
   * @param path Path already assigned to controller and local-planner plugins
   * @param update_type Path update semantics used to reset or preserve navigation state
   */
  void commitPlannerPathState(
    const nav_msgs::msg::Path & path,
    PathUpdateType update_type);
  /**
   * @brief 规划级 pipeline（③④）：LocalPlanner → SpeedPlanner 生成
   *        RobotTrajectory，缓存并分发给 controller。
   *        在控制循环内每周期执行一次（控制计算之前）：全局重规划频率低，速度剖面
   *        必须从最新 odom 速度与最新 costmap 逐周期重算（receding-horizon 前提）。
   *        控制级升频后若需降低规划频率，可在调用处加分频。
   *        未配置 current_speed_planner（空）时直接返回——controller 走 setPlan 路径，
   *        全链行为与未集成前一致（灰度回退）。
   * @param path 规划级输入路径（global path）
   * @return true 表示可继续执行本控制周期；false 表示规划失败，本周期必须零速闭环
   */
  bool runPlanningPipeline(const nav_msgs::msg::Path & path);
  /**
   * @brief 执行可选的 ③ LocalPlanner。未配置时保持输入路径直通。
   * @return false 表示已配置插件失败或输出空路径，调用方不得继续消费旧轨迹
   */
  bool computeLocalPath(
    const nav_msgs::msg::Path & global_path,
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav_msgs::msg::Path & local_path);
  /**
   * @brief 把带速度轨迹以 MarkerArray 发布到 /navigation/speed_plan 供 RViz 观察：
   *        速度着色 LINE_STRIP（红=0 → 绿=峰值渐变）+ 抽样速度数值文本。
   *        轨迹为空时发布 DELETEALL 清除旧 marker。
   * @param trajectory 速度规划输出轨迹
   */
  void publishSpeedPlan(const nav_interfaces::msg::RobotTrajectory & trajectory);

  void publishNavigationStatus(int32_t status, const std::string & message = "");
  void publishSpeedPlanDiagnostics(
    bool control_succeeded,
    bool control_receipt_valid,
    double consumed_target_velocity,
    double consumed_target_acceleration,
    const std::string & control_result);

  /**
   * @brief 查询当前轨迹上距机器人最近点的规划速度（期望速度）。
   *        位姿与轨迹坐标系不同时按当周期 TF 对齐后再查询。
   * @return 轨迹为空或 TF 不可用时 false。
   */
  bool nearestPlannedSpeed(
    const geometry_msgs::msg::PoseStamped & pose, double & desired_v,
    size_t * nearest_index = nullptr);
  /// 仅当可行的障碍停车剖面已要求机器人保持静止时暂停进展检查。
  bool shouldPauseProgressCheckForObstacleStop(
    const geometry_msgs::msg::PoseStamped & pose);
  /// 停障等待时跳过进展检查，恢复后重新建立检查基线。
  bool checkProgress(geometry_msgs::msg::PoseStamped & pose);
  /// 预测本周期将运行的控制器是否能自行绕开参考路径上的局部障碍。
  bool activeControllerCanAvoidPathObstacles(
    const geometry_msgs::msg::PoseStamped & pose);
  nav2_core::InitialMotionState buildInitialMotionState(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::Twist & measured_velocity,
    bool should_stop_for_path_obstacle = true);
  /// Latch terminal braking only after the planner reaches the dynamic braking
  /// initial_motion and the cycle has either longitudinal continuity or non-zero
  /// braking motion. A diagnostic-only infeasible result must not poison a
  /// stationary goal's ALIGN -> TRACK startup.
  void updateTerminalBrakeLatch(
    const nav2_core::SpeedPlanningDiagnostics & diagnostics,
    bool longitudinal_continuity_valid);
  void updateSpeedPlanningContinuityAfterControl(
    bool control_succeeded, bool trajectory_target_consumed,
    double consumed_target_velocity, double consumed_target_acceleration);
  void invalidateSpeedPlanningContinuity();
  /**
   * @brief Calculates velocity and publishes to "cmd_vel" topic
   */
  void computeAndPublishVelocity();
  /**
   * @brief Select and execute the current four-stage controller phase
   * @param pose Current robot pose
   * @param twist Current robot twist after velocity thresholding
   * @return Velocity command for the selected phase
   */
  geometry_msgs::msg::TwistStamped computeStagedVelocity(
    const geometry_msgs::msg::PoseStamped & pose,
    const nav_2d_msgs::msg::Twist2D & twist);
  /**
   * @brief Compute velocity from a loaded controller plugin
   * @param controller_id Controller plugin ID
   * @param pose Current robot pose
   * @param twist Current robot twist after velocity thresholding
   * @return Velocity command from the selected plugin
   */
  geometry_msgs::msg::TwistStamped computeControllerVelocity(
    const std::string & controller_id,
    const geometry_msgs::msg::PoseStamped & pose,
    const nav_2d_msgs::msg::Twist2D & twist);
  /**
   * @brief 获取控制器可选的末端定位参考能力。
   * @return 控制器不存在或不支持该能力时返回nullptr。
   */
  nav2_core::EndLocalizationCapability * getEndLocalizationCapability(
    const std::string & controller_id);
  /// 新目标、取消、失败或成功时统一清理各控制器的旧末端参考状态。
  void resetEndLocalizationCapabilities(uint64_t goal_generation);
  /// 控制前把ControllerServer维护的权威终点转换到本周期控制坐标系。
  void updateEndLocalizationGoal(
    const std::string & controller_id,
    const geometry_msgs::msg::PoseStamped & pose);
  /// 读取指定控制器的末端参考快照并校验目标代次；调用方按用途检查目标有效性。
  bool getEndLocalizationSnapshot(
    const std::string & controller_id,
    nav2_core::EndLocalizationSnapshot & snapshot);
  /**
   * @brief 判断是否进入末端接近控制，并返回末端定位阶段是否已接管。
   *
   * 目标距离负责首次选择PID；末端定位阶段激活后负责保持PID控制权，
   * 位置锁存只决定参考来源。位置锁存释放或短时TF失败不会单独触发控制器切换。
   */
  bool shouldUseApproachController(
    double distance_to_goal,
    bool & end_localization_active);
  /**
   * @brief 用同代次的PID有效末端目标更新GoalChecker目标。
   *
   * 有效末端目标可能包含实时位置，以及满足门限后锁存的Odom航向。
   * @return 更新成功时返回true，否则保留调用方传入的原始目标。
   */
  bool applyEndLocalizationGoalToGoalChecker(
    const std::string & controller_id,
    geometry_msgs::msg::PoseStamped & goal_pose);
  /**
   * @brief Publish direct reverse command when prepare phase detects front obstacle
   * @return true when a reverse command should be used
   */
  bool shouldRunPrepareBackup();
  /// 同步 bt_navigator 发布的任务代次；只有代次变化才重新允许执行一次 PREPARE。
  void syncPrepareSession();
  /// 新任务是唯一允许重新武装 PREPARE 的业务边界。
  void armStagedControlForNewSession(uint64_t session_id);
  /// PREPARE 结束后清理其运行状态，并发布非阻塞重规划信号。
  void completePrepareAndRequestReplan(const char * reason);
  /// 发布当前任务需要重规划的事实，不修改任何四阶段状态。
  void publishReplanRequired(const char * reason);
  /// FollowPath 取消或异常结束时锁存当前会话，防止同会话恢复重试再次后退。
  void lockPrepareForCurrentSession(const char * reason);
  const char * prepareStateName(PrepareState state) const;
  /**
   * @brief Check local costmap cells in a front rectangle of the robot
   * @return true when obstacle cost exceeds configured prepare threshold
   */
  bool isObstacleNearFootprint();

  /// 后退方向安全检查（参考 nav2_behaviors::DriveOnHeading 的逐周期 costmap
  /// 校验，复用 prepare_obstacle_check_distance_ 作为检测距离）：后退启动前
  /// 与后退过程中每周期都查，避免盲目后退撞上身后障碍。
  bool isRearObstacleDetected();

  /// 前/左/右/后共用：查询单点是否落在障碍代价内（TF 变换 + costmap 采样）。
  bool isCostmapCellBlocked(
    nav2_costmap_2d::Costmap2D * costmap, unsigned char cost_threshold,
    double x, double y);
  /**
   * @brief Compute yaw error between robot heading and local path tangent
   * @param pose Current robot pose
   * @param yaw_error Output yaw error in radians
   * @return true when path tangent could be calculated
   */
  /// align 朝向误差 = 机器人当前朝向 与 "机器人→路径前方 align_target_distance_
  /// 处目标点" 方位角 之差（同 RPP carrot 点取向原理，非局部路径切向）。
  bool getTargetPointHeadingError(
    const geometry_msgs::msg::PoseStamped & pose, double & yaw_error);
  /**
   * @brief Compute Euclidean distance from robot pose to current goal
   * @param pose Current robot pose
   * @return Distance to goal in meters, or infinity on transform failure
   */
  double getDistanceToGoal(const geometry_msgs::msg::PoseStamped & pose);
  /**
   * @brief Update current phase and emit transition logs
   * @param phase New phase
   * @param reason Transition reason for logs
   */
  void setControllerPhase(ControllerPhase phase, const std::string & reason);
  /**
   * @brief Human-readable phase name
   */
  const char * controllerPhaseName(ControllerPhase phase) const;
  /**
   * @brief Return configured plugin type for a goal checker id
   */
  std::string getGoalCheckerType(const std::string & goal_checker_id) const;
  /**
   * @brief 判断新 path 是否仍指向当前目标点
   * @param path 待更新的全局路径
   * @return true 表示新 path 终点与当前目标点基本一致
   */
  bool isSamePathGoal(const nav_msgs::msg::Path & path);
  /**
   * @brief Calls setPlannerPath method with an updated path received from
   * action server
   */
  void updateGlobalPath();
  /**
   * @brief Calls velocity publisher to publish the velocity on "cmd_vel" topic
   * @param velocity Twist velocity to be published
   */
  void publishVelocity(const geometry_msgs::msg::TwistStamped & velocity);
  /**
   * @brief Calls velocity publisher to publish zero velocity
   */
  void publishZeroVelocity();
  /**
   * @brief Bypass normal smoothing and request an immediate zero velocity
   */
  void publishHardStopVelocity();
  /**
   * @brief Checks if goal is reached
   * @return true or false
   */
  bool isGoalReached();
  /**
   * @brief Obtain current pose of the robot
   * @param pose To store current pose of the robot
   * @return true if able to obtain current pose of the robot, else false
   */
  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief get the thresholded velocity
   * @param velocity The current velocity from odometry
   * @param threshold The minimum velocity to return non-zero
   * @return double velocity value
   */
  double getThresholdedVelocity(double velocity, double threshold)
  {
    return (std::abs(velocity) > threshold) ? velocity : 0.0;
  }

  /**
   * @brief get the thresholded Twist
   * @param Twist The current Twist from odometry
   * @return Twist Twist after thresholds applied
   */
  nav_2d_msgs::msg::Twist2D getThresholdedTwist(const nav_2d_msgs::msg::Twist2D & twist)
  {
    nav_2d_msgs::msg::Twist2D twist_thresh;
    twist_thresh.x = getThresholdedVelocity(twist.x, min_x_velocity_threshold_);
    twist_thresh.y = getThresholdedVelocity(twist.y, min_y_velocity_threshold_);
    twist_thresh.theta = getThresholdedVelocity(twist.theta, min_theta_velocity_threshold_);
    return twist_thresh;
  }

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  /// 本周期 current_trajectory_ 已分发给哪个 controller（空 = 尚未分发）。
  /// 每周期由 runPlanningPipeline 清空，由 computeControllerVelocity 按需填充。
  std::string trajectory_dispatched_to_;
  std::mutex dynamic_params_lock_;

  // The controller needs a costmap node
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_util::NodeThread> costmap_thread_;

  // Publishers and subscribers
  std::unique_ptr<nav_2d_utils::OdomSubscriber> odom_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Empty>::SharedPtr hard_stop_publisher_;
  std::string hard_stop_topic_{"cmd_vel_hard_stop"};
  // PREPARE 与行为树之间只传递单调编号，不跨进程共享可变状态。
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt64>::SharedPtr
    prepare_replan_request_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr prepare_session_sub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    goal_error_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    speed_plan_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    speed_plan_diagnostics_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    near_goal_odom_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav_interfaces::msg::NavigationStatus>::SharedPtr
    nav_status_pub_;
  rclcpp::Subscription<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_sub_;

  // nav_monitor 任务回显：setPlannerPath() 里每次真正生效的 global path 复述一份，
  // 供 nav2_nav_monitor 记录 planned_path_length/replanning_count；setControllerPhase()
  // 切换时复述阶段名，供 nav_monitor 记录 phase_transitions（track/approach 段时间分析）。
  // 默认关闭零开销
  bool enable_nav_monitor_echo_{false};
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr global_path_echo_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr phase_echo_pub_;
  // 期望速度回显：每控制周期发布当前位置最近轨迹点的规划速度
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr
    desired_vel_echo_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    controller_target_echo_pub_;

  // Progress Checker Plugin
  pluginlib::ClassLoader<nav2_core::ProgressChecker> progress_checker_loader_;
  nav2_core::ProgressChecker::Ptr progress_checker_;
  std::string default_progress_checker_id_;
  std::string default_progress_checker_type_;
  std::string progress_checker_id_;
  std::string progress_checker_type_;

  // Goal Checker Plugin
  pluginlib::ClassLoader<nav2_core::GoalChecker> goal_checker_loader_;
  GoalCheckerMap goal_checkers_;
  std::vector<std::string> default_goal_checker_ids_;
  std::vector<std::string> default_goal_checker_types_;
  std::vector<std::string> goal_checker_ids_;
  std::vector<std::string> goal_checker_types_;
  std::string goal_checker_ids_concat_, default_goal_checker_, current_goal_checker_;

  // Controller Plugins
  pluginlib::ClassLoader<nav2_core::Controller> lp_loader_;
  ControllerMap controllers_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> controller_ids_;
  std::vector<std::string> controller_types_;
  std::string controller_ids_concat_, default_controller_, current_controller_;

  // Local Planner Plugins（③ 规划级；current_local_planner 为空 = global path 直通）
  pluginlib::ClassLoader<nav2_core::LocalPlanner> local_planner_loader_;
  LocalPlannerMap local_planners_;
  std::vector<std::string> local_planner_ids_;
  std::vector<std::string> local_planner_types_;
  std::string local_planner_ids_concat_, current_local_planner_;

  // Speed Planner Plugins（④ 规划级；current_speed_planner 为空 = 禁用/灰度回退）
  pluginlib::ClassLoader<nav2_core::SpeedPlanner> speed_planner_loader_;
  SpeedPlannerMap speed_planners_;
  std::vector<std::string> speed_planner_ids_;
  std::vector<std::string> speed_planner_types_;
  std::string speed_planner_ids_concat_, current_speed_planner_;
  // 轨迹缓存：规划级产出、控制级消费；规划与控制同在 action 线程逐周期串行执行，
  // 无并发；将来拆独立规划线程时需加最小临界区保护
  nav_interfaces::msg::RobotTrajectory current_trajectory_;
  nav2_speed_planner::InitialMotionArbiter initial_motion_arbiter_;
  // Goal-lifecycle state owned by ControllerServer. Same-goal geometry updates
  // preserve it; a new goal / successful completion resets it.
  bool terminal_brake_latched_{false};
  // 控制器检测到碰撞后进入等待；等待期间保持零速并暂停进展检查。
  // 控制恢复、新目标或控制状态重置时清零。
  bool is_collision_stop_active_{false};
  bool is_progress_check_paused_for_obstacle_stop_{false};
  struct SpeedPlanCycleContext
  {
    bool active{false};
    uint64_t cycle_id{0};
    uint64_t path_id{0};
    bool planning_succeeded{false};
    std::string planning_result{"not_started"};
    std::string planner_id;
    size_t local_path_point_count{0};
    double local_path_length{0.0};
    nav2_core::InitialMotionState initial_motion;
    bool initial_motion_valid{false};
    std::string initial_motion_source{"not_computed"};
    // 初值裁决的两个输入与偏差（见 InitialMotionDecision 注释）：只记裁决结果
    // 时，无法区分 v0 跳变是"实测真的高"还是"计划采样点跳了"。
    double initial_motion_measured_v{std::numeric_limits<double>::quiet_NaN()};
    double initial_motion_sampled_v{std::numeric_limits<double>::quiet_NaN()};
    double initial_motion_deviation{std::numeric_limits<double>::quiet_NaN()};
    // 局部路径投影这一拍前进了多少弧长。与 odom 位移对账即可判定"裁剪是否
    // 超额消耗"——L 跳变与 v0 跳变是两个独立成因，必须能分开看。
    double local_path_progress_delta{std::numeric_limits<double>::quiet_NaN()};
    nav2_core::SpeedPlanningDiagnostics terminal;
    bool terminal_valid{false};
  };
  /// 上一拍的局部路径剩余长度与其 path_id，用于算 local_path_progress_delta。
  /// path_id 变了（重规划）说明比较基准换了，delta 无意义，置 NaN。
  double prev_local_path_length_{std::numeric_limits<double>::quiet_NaN()};
  uint64_t prev_local_path_id_{0};
  uint64_t speed_plan_cycle_id_{0};
  uint64_t speed_plan_path_id_{0};
  SpeedPlanCycleContext speed_plan_cycle_;
  // RViz 可视化节流（速度剖面每周期重算，但只以 ~2Hz 重画）
  rclcpp::Time last_speed_plan_viz_time_{0, 0, RCL_ROS_TIME};

  double controller_frequency_;
  double min_x_velocity_threshold_;
  double min_y_velocity_threshold_;
  double min_theta_velocity_threshold_;

  double failure_tolerance_;


  bool staged_control_enabled_{true};
  ControllerPhase controller_phase_{ControllerPhase::TRACK};
  std::string track_controller_{"MPPIController"};
  std::string approach_controller_{"PIDController"};
  double approach_distance_{0.3};
  double align_heading_tolerance_{0.5235987756};
  double align_angular_kp_{1.5};
  double align_target_distance_{0.3};
  double align_max_angular_velocity_{0.5};
  double align_min_angular_velocity_{0.08};
  bool prepare_enabled_{true};
  // 检查区域 = 机器人真实 footprint 前沿 ~ 前沿+该距离，宽度取 footprint 实际半宽
  // （非人工设定值，随机型 footprint 配置自动生效）。
  double prepare_obstacle_check_distance_{0.2};
  // 左右检测：footprint 全长（后沿~前沿）两侧各外扩该距离；<=0 关闭。
  // 单参数同时控制左右（对称）。
  double prepare_side_check_distance_{0.15};
  int prepare_obstacle_cost_threshold_{253};
  double prepare_backup_speed_{-0.08};
  double prepare_backup_duration_{0.6};
  // true：按 prepare_backup_distance_ 走完固定后退距离（用起点位移估计，
  // duration 兜底防跑飞）；false（默认）：按 duration 定时后退。
  bool prepare_backup_use_distance_{false};
  double prepare_backup_distance_{0.3};
  PrepareState prepare_state_{PrepareState::DONE};
  // bt_navigator 每个导航任务发布一个新代次；同任务重规划和 FollowPath 恢复沿用该代次。
  std::atomic<uint64_t> latest_prepare_session_id_{0};
  uint64_t active_prepare_session_id_{0};
  // align 阶段状态机：本次导航是否已完成过 align（含"收敛"与"无法计算朝向，
  // 视为跳过"两种终止方式）。仅在导航任务会话变化或到达目标时重置；
  // computeControl() 会因 BT 恢复重试被多次重新进入，不能在入口重置。
  // 一旦置真，本次导航剩余时间不再回退进 align——中途朝向误差变大交给
  // track 阶段的控制器自身处理（RPP/MPPI 均有各自的朝向纠偏机制）。
  bool align_done_{false};
  rclcpp::Time prepare_backup_start_time_;
  geometry_msgs::msg::PoseStamped prepare_backup_start_pose_;

  /// 安全清理各阶段运行状态并保持 PREPARE 为 DONE；只有新会话可重新武装。
  void resetStagedControlState();
  std::string active_controller_for_log_{"unset"};

  // Whether we've published the single controller warning yet
  geometry_msgs::msg::PoseStamped end_pose_;
  // 每个新目标递增；同目标重规划保持不变，用于隔离上一任务的锁存数据。
  uint64_t goal_generation_{0};

  // Last time the controller generated a valid command
  rclcpp::Time last_valid_cmd_time_;

  // Current path container
  nav_msgs::msg::Path current_path_;

private:
  /**
    * @brief Callback for speed limiting messages
    * @param msg Shared pointer to nav2_msgs::msg::SpeedLimit
    */
  void speedLimitCallback(const nav2_msgs::msg::SpeedLimit::SharedPtr msg);
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__CONTROLLER_SERVER_HPP_
