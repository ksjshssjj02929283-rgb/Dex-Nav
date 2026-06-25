// Copyright (c) 2024 Open Navigation LLC
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

#include <atomic>
#include <thread>
#include <string>
#include <vector>
#include <memory>
#include <cmath>

#include "angles/angles.h"
#include "docking_base/docking_server.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace docking_base
{

DockingServer::DockingServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("docking_server", "", options),
  planner_loader_("docking_base_core", "docking_base_core::DockingPlanner"),
  controller_loader_("nav2_core", "nav2_core::Controller"),
  goal_checker_loader_("nav2_core", "nav2_core::GoalChecker"),
  task_loader_("docking_base_core", "docking_base_core::DockingInterface"),
  perception_loader_("docking_base_core", "docking_base_core::PerceptionInterface")
{
  RCLCPP_INFO(get_logger(), "Creating %s", get_name());

  declare_parameter("controller_frequency", 50.0);
  declare_parameter("initial_perception_timeout", 5.0);
  declare_parameter("wait_charge_timeout", 5.0);
  declare_parameter("dock_approach_timeout", 30.0);
  declare_parameter("rotate_to_dock_timeout", 10.0);
  declare_parameter("undock_linear_tolerance", 0.05);
  declare_parameter("undock_angular_tolerance", 0.05);
  declare_parameter("max_retries", 3);
  declare_parameter("base_frame", "base_link");
  declare_parameter("fixed_frame", "odom");
  declare_parameter("dock_backwards", false);
  declare_parameter("dock_prestaging_tolerance", 0.5);
  declare_parameter("rotation_angular_tolerance", 0.05);
  declare_parameter("rotate_to_dock", false);
  // staging 偏移（DEC-014：替代 ChargingDock::getStagingPose）
  declare_parameter("staging_x_offset", -0.7);
  declare_parameter("staging_yaw_offset", 0.0);

  // ROS1 范式：三类执行插件 id 列表 + 任务/感知默认插件 + 规划线程参数
  declare_parameter("plan_frequency", 10.0);
  declare_parameter("docking_task_timeout", 120.0);
  declare_parameter(
    "planner_plugins", std::vector<std::string>{"straight_line"});
  declare_parameter(
    "controller_plugins",
    std::vector<std::string>{"straight", "pure_rotation", "lat_lon", "pid"});
  declare_parameter(
    "goal_checker_plugins",
    std::vector<std::string>{"simple", "stopped", "rotation", "longitudinal", "position"});
  // 任务/感知插件 map（与三类执行 map 对称，on_configure 预加载）；
  // dock_type→task_id 由 <dock_type>.dock_task / .undock_task / .perception 解析（缺省 charging/exit/apriltag）
  declare_parameter("task_plugins", std::vector<std::string>{"charging", "exit"});
  declare_parameter("perception_plugins", std::vector<std::string>{"apriltag"});
}

nav2_util::CallbackReturn
DockingServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring %s", get_name());
  auto node = shared_from_this();

  get_parameter("controller_frequency", controller_frequency_);
  get_parameter("initial_perception_timeout", initial_perception_timeout_);
  get_parameter("wait_charge_timeout", wait_charge_timeout_);
  get_parameter("dock_approach_timeout", dock_approach_timeout_);
  get_parameter("rotate_to_dock_timeout", rotate_to_dock_timeout_);
  get_parameter("undock_linear_tolerance", undock_linear_tolerance_);
  get_parameter("undock_angular_tolerance", undock_angular_tolerance_);
  get_parameter("max_retries", max_retries_);
  get_parameter("base_frame", base_frame_);
  get_parameter("fixed_frame", fixed_frame_);
  get_parameter("dock_backwards", dock_backwards_);
  get_parameter("dock_prestaging_tolerance", dock_prestaging_tolerance_);
  get_parameter("rotation_angular_tolerance", rotation_angular_tolerance_);
  get_parameter("staging_x_offset", staging_x_offset_);
  get_parameter("staging_yaw_offset", staging_yaw_offset_);

  RCLCPP_INFO(get_logger(), "Controller frequency set to %.4fHz", controller_frequency_);

  vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  get_parameter("rotate_to_dock", rotate_to_dock_);
  if (rotate_to_dock_ && !dock_backwards_) {
    throw std::runtime_error{"Parameter rotate_to_dock is enabled but dock_backwards is not set."
            "Please set dock_backwards to true."};
  }

  double action_server_result_timeout;
  nav2_util::declare_parameter_if_not_declared(
    node, "action_server_result_timeout", rclcpp::ParameterValue(10.0));
  get_parameter("action_server_result_timeout", action_server_result_timeout);
  rcl_action_server_options_t server_options = rcl_action_server_get_default_options();
  server_options.result_timeout.nanoseconds = RCL_S_TO_NS(action_server_result_timeout);

  // Create the action servers for dock / undock
  docking_action_server_ = std::make_unique<DockingActionServer>(
    node, "dock_robot",
    std::bind(&DockingServer::dockRobot, this),
    nullptr, std::chrono::milliseconds(500),
    true, server_options);

  undocking_action_server_ = std::make_unique<UndockingActionServer>(
    node, "undock_robot",
    std::bind(&DockingServer::undockRobot, this),
    nullptr, std::chrono::milliseconds(500),
    true, server_options);

  // Create composed utilities（DEC-014：移除 Controller(graceful)/DockDatabase；保留 Navigator 导航到 staging）
  mutex_ = std::make_shared<std::mutex>();
  navigator_ = std::make_unique<Navigator>(node);

  // ROS1 范式：读规划/超时参数
  get_parameter("plan_frequency", plan_frequency_);
  get_parameter("docking_task_timeout", docking_task_timeout_);

  // 最小 costmap（仅供 controller/goal_checker 取 frame；plugins:[] 不避障）
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "docking_costmap", std::string{get_namespace()}, "docking_costmap");
  costmap_ros_->configure();
  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);

  // 加载 planner / controller / goal_checker 三类执行插件 map + 预加载 task/perception map
  loadPluginMaps();
  loadTaskMaps();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DockingServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating %s", get_name());

  auto node = shared_from_this();

  tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);
  navigator_->activate();
  costmap_ros_->activate();
  for (auto & kv : controllers_) {kv.second->activate();}
  for (auto & kv : planners_) {kv.second->activate();}
  vel_publisher_->on_activate();
  docking_action_server_->activate();
  undocking_action_server_->activate();
  curr_dock_type_.clear();

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&DockingServer::dynamicParametersCallback, this, _1));

  // Create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DockingServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating %s", get_name());

  docking_action_server_->deactivate();
  undocking_action_server_->deactivate();
  navigator_->deactivate();
  for (auto & kv : controllers_) {kv.second->deactivate();}
  for (auto & kv : planners_) {kv.second->deactivate();}
  if (costmap_ros_ && costmap_ros_->get_current_state().id() ==
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    costmap_ros_->deactivate();
  }
  vel_publisher_->on_deactivate();

  dyn_params_handler_.reset();
  tf2_listener_.reset();

  // Destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DockingServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up %s", get_name());
  // 三类执行插件清理
  for (auto & kv : controllers_) {kv.second->cleanup();}
  controllers_.clear();
  goal_checkers_.clear();
  for (auto & kv : planners_) {kv.second->cleanup();}
  planners_.clear();
  tasks_.clear();
  for (auto & kv : perceptions_) {kv.second->cleanup();}
  perceptions_.clear();
  if (costmap_ros_) {
    if (costmap_ros_->get_current_state().id() ==
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      costmap_ros_->cleanup();
    }
    costmap_thread_.reset();
    costmap_ros_.reset();
  }
  tf2_buffer_.reset();
  docking_action_server_.reset();
  undocking_action_server_.reset();
  navigator_.reset();
  curr_dock_type_.clear();
  vel_publisher_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DockingServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down %s", get_name());
  return nav2_util::CallbackReturn::SUCCESS;
}

template<typename ActionT>
void DockingServer::getPreemptedGoalIfRequested(
  typename std::shared_ptr<const typename ActionT::Goal> goal,
  const std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server)
{
  if (action_server->is_preempt_requested()) {
    goal = action_server->accept_pending_goal();
  }
}

template<typename ActionT>
bool DockingServer::checkAndWarnIfCancelled(
  std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server,
  const std::string & name)
{
  if (action_server->is_cancel_requested()) {
    RCLCPP_WARN(get_logger(), "Goal was cancelled. Cancelling %s action", name.c_str());
    return true;
  }
  return false;
}

template<typename ActionT>
bool DockingServer::checkAndWarnIfPreempted(
  std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server,
  const std::string & name)
{
  if (action_server->is_preempt_requested()) {
    RCLCPP_WARN(get_logger(), "Goal was preempted. Cancelling %s action", name.c_str());
    return true;
  }
  return false;
}

void DockingServer::dockRobot()
{
  std::lock_guard<std::mutex> lock(*mutex_);
  action_start_time_ = this->now();

  auto goal = docking_action_server_->get_current_goal();
  auto result = std::make_shared<DockRobot::Result>();
  result->success = false;

  if (!docking_action_server_ || !docking_action_server_->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return;
  }

  if (checkAndWarnIfCancelled(docking_action_server_, "dock_robot")) {
    docking_action_server_->terminate_all();
    return;
  }

  getPreemptedGoalIfRequested(goal, docking_action_server_);
  num_retries_ = 0;

  try {
    // DEC-014：dock 入口仅 dock_pose + dock_type（use_dock_id 随 DockDatabase 移除而废弃）
    if (goal->use_dock_id) {
      throw docking_base_core::DockNotValid(
              "use_dock_id 已不支持（DockDatabase 移除，DEC-014）；请用 dock_pose + dock_type");
    }
    const std::string dock_type = goal->dock_type;
    curr_dock_type_ = dock_type;
    RCLCPP_INFO(
      get_logger(), "对接 dock_type=%s @ (%.2f, %.2f)",
      dock_type.c_str(), goal->dock_pose.pose.position.x, goal->dock_pose.pose.position.y);

    // dock 位姿 → fixed_frame
    geometry_msgs::msg::PoseStamped dock_pose = goal->dock_pose;
    if (dock_pose.header.frame_id.empty()) {dock_pose.header.frame_id = fixed_frame_;}
    tf2_buffer_->transform(dock_pose, dock_pose, fixed_frame_);

    // 导航到 staging（staging = dock_pose ⊕ offset 参数，computeStagingPose 替代 ChargingDock::getStagingPose）
    publishDockingFeedback(DockRobot::Feedback::NAV_TO_STAGING_POSE);
    const auto staging_pose = computeStagingPose(dock_pose);
    const auto robot_pose = getRobotPoseInFrame(staging_pose.header.frame_id);
    if (!goal->navigate_to_staging_pose ||
      utils::l2Norm(robot_pose.pose, staging_pose.pose) < dock_prestaging_tolerance_)
    {
      RCLCPP_INFO(get_logger(), "已在 staging 容差内或跳过导航");
    } else {
      navigator_->goToPose(staging_pose, rclcpp::Duration::from_seconds(goal->max_staging_time));
      RCLCPP_INFO(get_logger(), "导航到 staging 完成");
    }

    // 精准对接：按 dock_type 选 DockingInterface 任务 + 感知，双线程执行
    auto task = resolveTask(dock_type, /*docking=*/true);
    if (!task) {throw docking_base_core::DockNotValid("未配置该 dock_type 的对接任务插件");}
    auto perception = resolvePerception(dock_type);
    if (perception) {task->setPerception(perception);}
    publishDockingFeedback(DockRobot::Feedback::CONTROLLING);

    if (runDockingTask(task, perception, docking_action_server_, "dock_robot", docking_task_timeout_)) {
      RCLCPP_INFO(get_logger(), "对接成功（DockingInterface 任务 SUCCESS）");
      result->success = true;
      result->num_retries = num_retries_;
      publishZeroVelocity();
      docking_action_server_->succeeded_current(result);
      return;
    }
    RCLCPP_WARN(get_logger(), "对接未成功（取消/失败/超时）");
    publishZeroVelocity();
    docking_action_server_->terminate_all(result);
    return;
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(get_logger(), "Transform error: %s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  } catch (docking_base_core::DockNotInDB & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::DOCK_NOT_IN_DB;
  } catch (docking_base_core::DockNotValid & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::DOCK_NOT_VALID;
  } catch (docking_base_core::FailedToStage & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::FAILED_TO_STAGE;
  } catch (docking_base_core::FailedToDetectDock & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::FAILED_TO_DETECT_DOCK;
  } catch (docking_base_core::FailedToControl & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::FAILED_TO_CONTROL;
  } catch (docking_base_core::FailedToCharge & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::FAILED_TO_CHARGE;
  } catch (docking_base_core::DockingException & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  } catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  }

  result->num_retries = num_retries_;
  publishZeroVelocity();
  docking_action_server_->terminate_current(result);
}


geometry_msgs::msg::PoseStamped DockingServer::computeStagingPose(
  const geometry_msgs::msg::PoseStamped & dock_pose)
{
  // staging = dock 位姿沿其 x 轴偏移 staging_x_offset + 朝向加 staging_yaw_offset
  // （DEC-014，替代已删除的 ChargingDock::getStagingPose）
  geometry_msgs::msg::PoseStamped staging = dock_pose;
  const double yaw = tf2::getYaw(dock_pose.pose.orientation);
  staging.pose.position.x += std::cos(yaw) * staging_x_offset_;
  staging.pose.position.y += std::sin(yaw) * staging_x_offset_;
  staging.pose.orientation =
    nav2_util::geometry_utils::orientationAroundZAxis(yaw + staging_yaw_offset_);
  return staging;
}

void DockingServer::undockRobot()
{
  std::lock_guard<std::mutex> lock(*mutex_);
  action_start_time_ = this->now();
  rclcpp::Rate loop_rate(controller_frequency_);

  auto goal = undocking_action_server_->get_current_goal();
  auto result = std::make_shared<UndockRobot::Result>();
  result->success = false;

  if (!undocking_action_server_ || !undocking_action_server_->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return;
  }

  if (checkAndWarnIfCancelled(undocking_action_server_, "undock_robot")) {
    undocking_action_server_->terminate_all(result);
    return;
  }

  getPreemptedGoalIfRequested(goal, undocking_action_server_);
  auto max_duration = rclcpp::Duration::from_seconds(goal->max_undocking_time);

  try {
    // ROS1 范式出桩：按 dock_type 选 DockingExit 任务，双线程执行（任务从外部传入）
    std::string dock_type = curr_dock_type_;
    if (!goal->dock_type.empty()) {
      dock_type = goal->dock_type;
    }
    RCLCPP_INFO(get_logger(), "Attempting to undock robot (dock_type=%s).", dock_type.c_str());

    auto task = resolveTask(dock_type, /*docking=*/false);
    if (!task) {throw docking_base_core::DockNotValid("未配置该 dock_type 的出桩任务插件");}
    if (runDockingTask(
        task, nullptr, undocking_action_server_, "undock_robot", goal->max_undocking_time))
    {
      RCLCPP_INFO(get_logger(), "出桩成功（DockingExit 任务 SUCCESS）");
      result->success = true;
      curr_dock_type_.clear();
      publishZeroVelocity();
      undocking_action_server_->succeeded_current(result);
      return;
    }
    RCLCPP_WARN(get_logger(), "出桩未成功（取消/失败/超时）");
    publishZeroVelocity();
    undocking_action_server_->terminate_all(result);
    return;
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(get_logger(), "Transform error: %s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  } catch (docking_base_core::DockNotValid & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::DOCK_NOT_VALID;
  } catch (docking_base_core::FailedToControl & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::FAILED_TO_CONTROL;
  } catch (docking_base_core::DockingException & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  } catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Internal error: %s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  }

  publishZeroVelocity();
  undocking_action_server_->terminate_current(result);
}

geometry_msgs::msg::PoseStamped DockingServer::getRobotPoseInFrame(const std::string & frame)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = base_frame_;
  robot_pose.header.stamp = rclcpp::Time(0);
  tf2_buffer_->transform(robot_pose, robot_pose, frame);
  return robot_pose;
}

void DockingServer::publishZeroVelocity()
{
  vel_publisher_->publish(geometry_msgs::msg::Twist());
}

void DockingServer::publishDockingFeedback(uint16_t state)
{
  auto feedback = std::make_shared<DockRobot::Feedback>();
  feedback->state = state;
  feedback->docking_time = this->now() - action_start_time_;
  feedback->num_retries = num_retries_;
  docking_action_server_->publish_feedback(feedback);
}

rcl_interfaces::msg::SetParametersResult
DockingServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> lock(*mutex_);

  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "controller_frequency") {
        controller_frequency_ = parameter.as_double();
      } else if (name == "initial_perception_timeout") {
        initial_perception_timeout_ = parameter.as_double();
      } else if (name == "wait_charge_timeout") {
        wait_charge_timeout_ = parameter.as_double();
      } else if (name == "undock_linear_tolerance") {
        undock_linear_tolerance_ = parameter.as_double();
      } else if (name == "undock_angular_tolerance") {
        undock_angular_tolerance_ = parameter.as_double();
      } else if (name == "rotation_angular_tolerance") {
        rotation_angular_tolerance_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == "base_frame") {
        base_frame_ = parameter.as_string();
      } else if (name == "fixed_frame") {
        fixed_frame_ = parameter.as_string();
      }
    } else if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == "max_retries") {
        max_retries_ = parameter.as_int();
      }
    }
  }

  result.successful = true;
  return result;
}

// ===== ROS1 范式：三类执行插件 + DockingInterface 任务驱动（双线程）=====

void DockingServer::loadPluginMaps()
{
  auto node = shared_from_this();
  get_parameter("planner_plugins", planner_ids_);
  get_parameter("controller_plugins", controller_ids_);
  get_parameter("goal_checker_plugins", goal_checker_ids_);

  // 短名→类型默认映射（yaml 可用 <id>.plugin 覆盖）；id 跨三类须唯一（同节点共享参数空间）
  auto declare_type = [&](const std::string & id, const std::string & type) {
      nav2_util::declare_parameter_if_not_declared(
        node, id + ".plugin", rclcpp::ParameterValue(type));
    };
  // planner（id straight_line）
  declare_type("straight_line", "docking_base_plugins::StraightLinePlanner");
  // controller（nav2_modules 控制器）
  declare_type("straight", "straight_controller::StraightController");
  declare_type("pure_rotation", "pure_rotation_controller::PureRotationController");
  declare_type("lat_lon", "lat_lon_controller::LatLonController");
  declare_type("pid", "pid_controller::PIDController");
  // goal_checker
  declare_type("simple", "simple_goal_checker::SimpleGoalChecker");
  declare_type("stopped", "stopped_goal_checker::StoppedGoalChecker");
  declare_type("rotation", "rotation_goal_checker::RotationGoalChecker");
  declare_type("longitudinal", "longitudinal_goal_checker::LongitudinalGoalChecker");
  declare_type("position", "position_goal_checker::PositionGoalChecker");
  declare_type("high_precision", "high_precision_goal_checker::HighPrecisionGoalChecker");

  for (const auto & id : planner_ids_) {
    std::string type;
    node->get_parameter(id + ".plugin", type);
    auto p = planner_loader_.createSharedInstance(type);
    p->configure(node, id, tf2_buffer_);
    planners_[id] = p;
    RCLCPP_INFO(get_logger(), "  planner[%s] = %s", id.c_str(), type.c_str());
  }
  for (const auto & id : controller_ids_) {
    std::string type;
    node->get_parameter(id + ".plugin", type);
    auto c = controller_loader_.createSharedInstance(type);
    c->configure(node, id, costmap_ros_->getTfBuffer(), costmap_ros_);
    controllers_[id] = c;
    RCLCPP_INFO(get_logger(), "  controller[%s] = %s", id.c_str(), type.c_str());
  }
  for (const auto & id : goal_checker_ids_) {
    std::string type;
    node->get_parameter(id + ".plugin", type);
    auto gc = goal_checker_loader_.createSharedInstance(type);
    gc->initialize(node, id, costmap_ros_);
    goal_checkers_[id] = gc;
    RCLCPP_INFO(get_logger(), "  goal_checker[%s] = %s", id.c_str(), type.c_str());
  }
}

void DockingServer::loadTaskMaps()
{
  auto node = shared_from_this();
  get_parameter("task_plugins", task_ids_);
  get_parameter("perception_plugins", perception_ids_);

  // 短名→类型默认映射（与 planner/controller/goal_checker 的 .plugin 参数同空间，id 须全局唯一）
  auto declare_type = [&](const std::string & id, const std::string & type) {
      nav2_util::declare_parameter_if_not_declared(
        node, id + ".plugin", rclcpp::ParameterValue(type));
    };
  declare_type("charging", "docking_base_plugins::DockingCharging");
  declare_type("exit", "docking_base_plugins::DockingExit");
  declare_type("apriltag", "docking_base_plugins::AprilTagPerception");

  // 预加载 task map（实例名=id，故任务参数在 <id>.* 命名空间）
  for (const auto & id : task_ids_) {
    std::string type;
    node->get_parameter(id + ".plugin", type);
    auto t = task_loader_.createSharedInstance(type);
    t->configure(node, id, tf2_buffer_);
    tasks_[id] = t;
    RCLCPP_INFO(get_logger(), "  task[%s] = %s", id.c_str(), type.c_str());
  }
  // 预加载 perception map
  for (const auto & id : perception_ids_) {
    std::string type;
    node->get_parameter(id + ".plugin", type);
    auto p = perception_loader_.createSharedInstance(type);
    p->configure(node, id, tf2_buffer_);
    perceptions_[id] = p;
    RCLCPP_INFO(get_logger(), "  perception[%s] = %s", id.c_str(), type.c_str());
  }
}

docking_base_core::DockingInterface::Ptr
DockingServer::resolveTask(const std::string & dock_type, bool docking)
{
  auto node = shared_from_this();
  const std::string suffix = docking ? ".dock_task" : ".undock_task";
  std::string task_id = docking ? "charging" : "exit";  // 缺省
  if (!dock_type.empty()) {
    const std::string param = dock_type + suffix;
    nav2_util::declare_parameter_if_not_declared(node, param, rclcpp::ParameterValue(task_id));
    node->get_parameter(param, task_id);
  }
  auto it = tasks_.find(task_id);
  if (it == tasks_.end()) {
    RCLCPP_ERROR(
      get_logger(), "任务插件未预加载: task_id=%s (dock_type=%s)",
      task_id.c_str(), dock_type.c_str());
    return nullptr;
  }
  RCLCPP_INFO(
    get_logger(), "选用任务[%s] (dock_type=%s, %s)",
    task_id.c_str(), dock_type.c_str(), docking ? "dock" : "undock");
  return it->second;
}

docking_base_core::PerceptionInterface::Ptr
DockingServer::resolvePerception(const std::string & dock_type)
{
  auto node = shared_from_this();
  std::string perc_id = "apriltag";  // 缺省（空字符串=该任务无感知）
  if (!dock_type.empty()) {
    const std::string param = dock_type + ".perception";
    nav2_util::declare_parameter_if_not_declared(node, param, rclcpp::ParameterValue(perc_id));
    node->get_parameter(param, perc_id);
  }
  if (perc_id.empty()) {return nullptr;}
  auto it = perceptions_.find(perc_id);
  if (it == perceptions_.end()) {
    RCLCPP_WARN(get_logger(), "感知插件未预加载: %s（按无感知处理）", perc_id.c_str());
    return nullptr;
  }
  return it->second;
}

bool DockingServer::getRobotPoseSafe(
  const std::string & frame, geometry_msgs::msg::PoseStamped & out)
{
  geometry_msgs::msg::PoseStamped robot;
  robot.header.frame_id = base_frame_;
  robot.header.stamp = rclcpp::Time(0);
  try {
    tf2_buffer_->transform(robot, out, frame);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "getRobotPoseSafe 失败: %s", ex.what());
    return false;
  }
}

template<typename ActionServerT>
bool DockingServer::runDockingTask(
  docking_base_core::DockingInterface::Ptr task,
  docking_base_core::PerceptionInterface::Ptr perception,
  std::unique_ptr<ActionServerT> & action_server,
  const std::string & action_name, double timeout_s)
{
  using docking_base_core::DockingStatus;
  task->Reset();   // 预加载复用实例：先复位基类公共状态，再 Init 复位子类阶段
  task->Init();
  if (perception) {perception->start();}

  // 规划线程：喂位姿 + 感知取目标 + 推阶段（不出 cmd_vel；控制由本线程负责）
  std::atomic_bool running{true};
  std::thread plan_thread(
    [this, task, perception, &running]() {
      rclcpp::Rate plan_rate(plan_frequency_);
      while (running && rclcpp::ok()) {
        geometry_msgs::msg::PoseStamped robot;
        if (getRobotPoseSafe(fixed_frame_, robot)) {task->UpdateRobotPose(robot);}
        if (perception) {
          geometry_msgs::msg::PoseStamped dp;
          if (perception->getDockPose(dp, fixed_frame_)) {task->UpdateTargetPose(dp);}
        }
        task->UpdateTaskStage();
        plan_rate.sleep();
      }
    });

  // 控制线程（本线程）：按阶段指令包选 planner/controller/goal_checker
  rclcpp::Rate rate(controller_frequency_);
  DockingStatus st = DockingStatus::EXECUTING;
  const auto t0 = now();
  std::string cur_ctrl, cur_gc;
  std::shared_ptr<geometry_msgs::msg::PoseStamped> planned_goal;
  geometry_msgs::msg::Twist last_vel;

  while (rclcpp::ok()) {
    if (checkAndWarnIfCancelled(action_server, action_name) ||
      checkAndWarnIfPreempted(action_server, action_name))
    {
      st = DockingStatus::FAILED;
      break;
    }
    st = task->curr_status();
    if (st == DockingStatus::SUCCESS || st == DockingStatus::FAILED) {break;}

    auto goal = task->GetCurrGoal();
    geometry_msgs::msg::PoseStamped robot;
    if (!goal || !getRobotPoseSafe(fixed_frame_, robot)) {
      publishZeroVelocity();
      rate.sleep();
      continue;
    }

    const std::string ctrl_id = task->GetCurrControllerName();
    const std::string gc_id = task->GetCurrGoalCheckerName();
    auto cit = controllers_.find(ctrl_id);
    auto git = goal_checkers_.find(gc_id);
    if (cit == controllers_.end() || git == goal_checkers_.end()) {
      RCLCPP_ERROR(
        get_logger(), "阶段控制器/检查器未注册: controller=%s goal_checker=%s",
        ctrl_id.c_str(), gc_id.c_str());
      st = DockingStatus::FAILED;
      break;
    }
    auto controller = cit->second;
    auto goal_checker = git->second;

    // 阶段/目标变化 → setSpeedLimit(先) → 有 planner 则出带速度剖面轨迹 setTrajectory，
    // 否则回退 setPlan（纯几何）→ 重置 goal_checker
    const double dgoal = planned_goal ?
      std::hypot(
      planned_goal->pose.position.x - goal->pose.position.x,
      planned_goal->pose.position.y - goal->pose.position.y) :
      1.0;
    if (ctrl_id != cur_ctrl || gc_id != cur_gc || !planned_goal || dgoal > 0.01) {
      const nav_msgs::msg::Path coarse = task->GetCurrPath();
      controller->setSpeedLimit(task->GetLinearVelLimit(), false);
      auto pit = task->UseLocalPlanner() ?
        planners_.find(task->GetCurrPlannerName()) : planners_.end();
      if (pit != planners_.end() && coarse.poses.size() >= 2) {
        // 规划/控制分离：planner 出带逐点纵向速度的 RobotTrajectory，controller 经 setTrajectory 消费
        auto traj = pit->second->createTrajectory(
          coarse, robot, task->GetLinearVelLimit(), task->NeedEnableReverse());
        controller->setTrajectory(traj);
      } else {
        // 无 planner（rotation 等纯几何阶段）：回退 setPlan，速度靠 setSpeedLimit
        controller->setPlan(coarse);
      }
      goal_checker->reset();
      cur_ctrl = ctrl_id;
      cur_gc = gc_id;
      planned_goal = goal;
    }

    try {
      auto cmd = controller->computeVelocityCommands(robot, last_vel, goal_checker.get());
      vel_publisher_->publish(cmd.twist);
      last_vel = cmd.twist;
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "computeVelocityCommands 异常: %s", ex.what());
      publishZeroVelocity();
      last_vel = geometry_msgs::msg::Twist();
    }

    if (goal_checker->isGoalReached(robot.pose, goal->pose, last_vel)) {
      task->SetCurrGoalReached();
    }

    if ((now() - t0).seconds() > timeout_s) {
      RCLCPP_ERROR(get_logger(), "%s 任务超时 (%.1fs)", action_name.c_str(), timeout_s);
      st = DockingStatus::FAILED;
      break;
    }
    rate.sleep();
  }

  running = false;
  if (plan_thread.joinable()) {plan_thread.join();}
  publishZeroVelocity();
  if (perception) {perception->stop();}
  return st == DockingStatus::SUCCESS;
}

}  // namespace docking_base

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(docking_base::DockingServer)
