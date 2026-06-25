// 对接任务抽象（ROS1 docking_base DockingInterface 移植，DEC-011 Part2）
// 子类（DockingCharging/DockingExit）定义各对接任务的阶段状态机与执行；
// 每阶段产「指令包」：curr_goal + controller_id + goal_checker_id + 限速 + use_planner + reverse。
// DockingServer 双线程据此调 nav2_core::Controller/GoalChecker（P3）。
#ifndef DOCKING_BASE_CORE__DOCKING_INTERFACE_HPP_
#define DOCKING_BASE_CORE__DOCKING_INTERFACE_HPP_

#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  // isSamePose 用 getYaw(msg) → 需 fromMsg 定义
#include "tf2_ros/buffer.h"

#include "docking_base_core/perception_interface.hpp"

namespace docking_base_core
{

enum class DockingStatus
{
  FAILED = -1,
  UNKNOWN = 0,
  EXECUTING = 1,
  SUCCESS = 2,
};

/**
 * @class DockingInterface
 * @brief 对接任务抽象基类。子类实现 Init() + UpdateTaskStage() 阶段状态机。
 */
class DockingInterface
{
public:
  using Ptr = std::shared_ptr<DockingInterface>;

  virtual ~DockingInterface() = default;

  /** @brief 配置：宿主节点 / 实例名 / tf；perception 可选（解耦感知，Part1）。 */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf)
  {
    node_ = parent.lock();
    name_ = name;
    tf2_buffer_ = tf;
  }

  void setPerception(PerceptionInterface::Ptr perception) {perception_ = perception;}

  /** @brief 初始化任务（订阅/参数/复位阶段）。 */
  virtual bool Init() = 0;

  /** @brief 推进阶段状态机，返回任务状态。控制线程每周期调。 */
  virtual DockingStatus UpdateTaskStage() = 0;

  /**
   * @brief 复位基类公共状态（预加载复用实例时每次任务开始前调，再调 Init）。
   *        子类阶段等私有状态由各自 Init 复位。
   */
  void Reset()
  {
    std::lock_guard<std::recursive_mutex> lock(data_mutex_);
    target_info_updated_ = false;
    is_current_goal_reached_ = false;
    curr_goal_.reset();
    last_reached_goal_.reset();
    prev_check_point_.reset();
    robot_pose_.reset();
    curr_status_ = DockingStatus::UNKNOWN;
  }

  // ---- 外部输入 ----
  void UpdateRobotPose(const geometry_msgs::msg::PoseStamped & robot_pose)
  {
    std::lock_guard<std::recursive_mutex> lock(data_mutex_);
    robot_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>(robot_pose);
  }

  /** @brief 更新对接目标（dock）位姿（server 从 perception->getDockPose 喂入）。 */
  void UpdateTargetPose(const geometry_msgs::msg::PoseStamped & target_pose)
  {
    std::lock_guard<std::recursive_mutex> lock(data_mutex_);
    target_pose_ = target_pose;
    target_info_updated_ = true;
    GenerateCheckPoint();
  }

  /** @brief 当前阶段目标已到达（server 的 goal_checker 判定后调）。一个 goal 只 reach 一次。 */
  void SetCurrGoalReached()
  {
    std::lock_guard<std::recursive_mutex> lock(data_mutex_);
    if (!last_reached_goal_ || !curr_goal_ || !isSamePose(*last_reached_goal_, *curr_goal_)) {
      last_reached_goal_ = curr_goal_;
      is_current_goal_reached_ = true;
    }
  }

  // ---- 每阶段「指令包」getter（供 DockingServer 双线程使用）----
  std::string GetCurrControllerName() const {return curr_controller_;}
  std::string GetCurrGoalCheckerName() const {return curr_goal_checker_;}
  std::string GetCurrPlannerName() const {return curr_planner_;}
  std::shared_ptr<geometry_msgs::msg::PoseStamped> GetCurrGoal() const {return curr_goal_;}
  double GetLinearVelLimit() const {return linear_vel_limit_;}
  double GetAngularVelLimit() const {return angular_vel_limit_;}
  bool UseLocalPlanner() const {return use_local_planner_;}
  bool NeedEnableReverse() const {return need_enable_reverse_;}
  DockingStatus curr_status() const {return curr_status_;}

  /** @brief 段路径：prev_check_point_ → curr_goal_（供规划层/控制器跟踪）。 */
  nav_msgs::msg::Path GetCurrPath()
  {
    GenerateDockingPath();
    return docking_path_;
  }

protected:
  /** @brief 生成段路径（两点：上一检查点→当前目标）。 */
  virtual bool GenerateDockingPath()
  {
    std::lock_guard<std::recursive_mutex> lock(data_mutex_);
    if (!robot_pose_ || !curr_goal_) {return false;}
    docking_path_.header = robot_pose_->header;
    docking_path_.poses.clear();
    if (prev_check_point_) {
      prev_check_point_->header = robot_pose_->header;
      docking_path_.poses.push_back(*prev_check_point_);
    } else {
      docking_path_.poses.push_back(*robot_pose_);
    }
    curr_goal_->header = robot_pose_->header;
    docking_path_.poses.push_back(*curr_goal_);
    return true;
  }

  /** @brief 子类算检查点几何（相对 dock 位姿）。 */
  virtual void GenerateCheckPoint() {}

  static bool isSamePose(
    const geometry_msgs::msg::PoseStamped & a, const geometry_msgs::msg::PoseStamped & b,
    double xy_tol = 0.1, double yaw_tol = 0.05)
  {
    const double d = std::hypot(
      a.pose.position.x - b.pose.position.x, a.pose.position.y - b.pose.position.y);
    double dyaw = tf2::getYaw(a.pose.orientation) - tf2::getYaw(b.pose.orientation);
    while (dyaw > M_PI) {dyaw -= 2 * M_PI;}
    while (dyaw < -M_PI) {dyaw += 2 * M_PI;}
    return d < xy_tol && std::fabs(dyaw) < yaw_tol;
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  PerceptionInterface::Ptr perception_;
  std::string name_;

  geometry_msgs::msg::PoseStamped target_pose_;     // dock 位姿
  std::shared_ptr<geometry_msgs::msg::PoseStamped> robot_pose_;
  std::shared_ptr<geometry_msgs::msg::PoseStamped> prev_check_point_;
  std::shared_ptr<geometry_msgs::msg::PoseStamped> curr_goal_;
  std::shared_ptr<geometry_msgs::msg::PoseStamped> last_reached_goal_;
  nav_msgs::msg::Path docking_path_;

  // 每阶段指令包
  std::string curr_controller_;
  std::string curr_goal_checker_;
  std::string curr_planner_{"straight_line"};  // 规划层 id；use_local_planner_ 为 true 时生效
  double linear_vel_limit_{0.1};
  double angular_vel_limit_{0.3};
  bool use_local_planner_{true};
  bool need_enable_reverse_{false};

  DockingStatus curr_status_{DockingStatus::UNKNOWN};
  bool is_current_goal_reached_{false};
  bool target_info_updated_{false};

  std::recursive_mutex data_mutex_;
};

}  // namespace docking_base_core

#endif  // DOCKING_BASE_CORE__DOCKING_INTERFACE_HPP_
