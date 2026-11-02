// Copyright (c) 2019 Intel Corporation
// Copyright (c) 2022 Samsung Research America
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

#include "nav2_lifecycle_manager/lifecycle_manager.hpp"

#include <chrono>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

using lifecycle_msgs::msg::Transition;
using lifecycle_msgs::msg::State;
using nav2_util::LifecycleServiceClient;

namespace nav2_lifecycle_manager
{

LifecycleManager::LifecycleManager(const rclcpp::NodeOptions & options)
: Node("lifecycle_manager", options), diagnostics_updater_(this)
{
  RCLCPP_INFO(get_logger(), "Creating");

  // The list of names is parameterized, allowing this module to be used with a different set
  // of nodes
  declare_parameter("node_names", rclcpp::PARAMETER_STRING_ARRAY);
  declare_parameter("autostart", rclcpp::ParameterValue(false));
  declare_parameter("bond_timeout", 4.0);
  // 运行后断线等待节点恢复的观察窗口，秒；不负责重启节点进程。
  declare_parameter("bond_respawn_max_duration", 300.0);
  declare_parameter("attempt_respawn_reconnection", true);

  registerRclPreshutdownCallback();

  node_names_ = get_parameter("node_names").as_string_array();
  get_parameter("autostart", autostart_);
  double bond_timeout_s;
  get_parameter("bond_timeout", bond_timeout_s);
  bond_timeout_ = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(bond_timeout_s));

  double respawn_timeout_s;
  get_parameter("bond_respawn_max_duration", respawn_timeout_s);
  bond_respawn_max_duration_ = rclcpp::Duration::from_seconds(respawn_timeout_s);

  get_parameter("attempt_respawn_reconnection", attempt_respawn_reconnection_);

  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  manager_srv_ = create_service<ManageLifecycleNodes>(
    get_name() + std::string("/manage_nodes"),
    std::bind(&LifecycleManager::managerCallback, this, _1, _2, _3),
    rclcpp::ServicesQoS().get_rmw_qos_profile(),
    callback_group_);

  is_active_srv_ = create_service<std_srvs::srv::Trigger>(
    get_name() + std::string("/is_active"),
    std::bind(&LifecycleManager::isActiveCallback, this, _1, _2, _3),
    rclcpp::ServicesQoS().get_rmw_qos_profile(),
    callback_group_);

  transition_state_map_[Transition::TRANSITION_CONFIGURE] = State::PRIMARY_STATE_INACTIVE;
  transition_state_map_[Transition::TRANSITION_CLEANUP] = State::PRIMARY_STATE_UNCONFIGURED;
  transition_state_map_[Transition::TRANSITION_ACTIVATE] = State::PRIMARY_STATE_ACTIVE;
  transition_state_map_[Transition::TRANSITION_DEACTIVATE] = State::PRIMARY_STATE_INACTIVE;
  transition_state_map_[Transition::TRANSITION_UNCONFIGURED_SHUTDOWN] =
    State::PRIMARY_STATE_FINALIZED;

  transition_label_map_[Transition::TRANSITION_CONFIGURE] = std::string("Configuring ");
  transition_label_map_[Transition::TRANSITION_CLEANUP] = std::string("Cleaning up ");
  transition_label_map_[Transition::TRANSITION_ACTIVATE] = std::string("Activating ");
  transition_label_map_[Transition::TRANSITION_DEACTIVATE] = std::string("Deactivating ");
  transition_label_map_[Transition::TRANSITION_UNCONFIGURED_SHUTDOWN] =
    std::string("Shutting down ");

  init_timer_ = this->create_wall_timer(
    0s,
    [this]() -> void {
      init_timer_->cancel();
      createLifecycleServiceClients();
      if (autostart_) {
        init_timer_ = this->create_wall_timer(
          0s,
          [this]() -> void {
            init_timer_->cancel();
            startupWithCleanup();
          },
          callback_group_);
      }
      auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      executor->add_callback_group(callback_group_, get_node_base_interface());
      service_thread_ = std::make_unique<nav2_util::NodeThread>(executor);
    });
  diagnostics_updater_.setHardwareID("Nav2");
  diagnostics_updater_.add("Nav2 Health", this, &LifecycleManager::CreateActiveDiagnostic);
}

LifecycleManager::~LifecycleManager()
{
  RCLCPP_INFO(get_logger(), "Destroying %s", get_name());
  service_thread_.reset();
}

void
LifecycleManager::managerCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<ManageLifecycleNodes::Request> request,
  std::shared_ptr<ManageLifecycleNodes::Response> response)
{
  switch (request->command) {
    case ManageLifecycleNodes::Request::STARTUP:
      response->success = startupWithCleanup();
      break;
    case ManageLifecycleNodes::Request::RESET:
      response->success = reset();
      break;
    case ManageLifecycleNodes::Request::SHUTDOWN:
      response->success = shutdown();
      break;
    case ManageLifecycleNodes::Request::PAUSE:
      response->success = pause();
      break;
    case ManageLifecycleNodes::Request::RESUME:
      response->success = resume();
      break;
  }
}

void
LifecycleManager::isActiveCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  response->success = system_active_;
}

void
LifecycleManager::CreateActiveDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (system_active_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Nav2 is active");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Nav2 is inactive");
  }
}

void
LifecycleManager::createLifecycleServiceClients()
{
  message("Creating and initializing lifecycle service clients");
  for (auto & node_name : node_names_) {
    node_map_[node_name] =
      std::make_shared<LifecycleServiceClient>(node_name, shared_from_this());
  }
}

void
LifecycleManager::destroyLifecycleServiceClients()
{
  message("Destroying lifecycle service clients");
  for (auto & kv : node_map_) {
    kv.second.reset();
  }
}

bool
LifecycleManager::createBondConnection(const std::string & node_name)
{
  const double timeout_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(bond_timeout_).count();
  const double timeout_s = timeout_ns / 1e9;

  if (bond_map_.find(node_name) == bond_map_.end() && bond_timeout_.count() > 0.0) {
    bond_map_[node_name] =
      std::make_shared<bond::Bond>("bond", node_name, shared_from_this());
    // 底层默认只等待 10 秒，须与管理器窗口一致，避免较长配置被提前断开。
    bond_map_[node_name]->setConnectTimeout(timeout_s);
    bond_map_[node_name]->setHeartbeatTimeout(timeout_s);
    bond_map_[node_name]->setHeartbeatPeriod(0.10);
    bond_map_[node_name]->start();
    // 部分 bond 版本离开等待态就返回 true，连接超时进入断开态也不能算成功。
    if (
      !bond_map_[node_name]->waitUntilFormed(
        rclcpp::Duration(rclcpp::Duration::from_nanoseconds(timeout_ns))) ||
      bond_map_[node_name]->isBroken())
    {
      RCLCPP_ERROR(
        get_logger(),
        "Server %s was unable to be reached within %0.2fs by bond. "
        "This server may be misconfigured.",
        node_name.c_str(), timeout_s);
      bond_connection_failed_ = true;
      return false;
    }
    RCLCPP_INFO(get_logger(), "Server %s connected with bond.", node_name.c_str());
  }

  return true;
}

bool
LifecycleManager::changeStateForNode(const std::string & node_name, std::uint8_t transition)
{
  message(transition_label_map_[transition] + node_name);

  if (!node_map_[node_name]->change_state(transition) ||
    !(node_map_[node_name]->get_state() == transition_state_map_[transition]))
  {
    RCLCPP_ERROR(get_logger(), "Failed to change state for node: %s", node_name.c_str());
    return false;
  }

  if (transition == Transition::TRANSITION_ACTIVATE) {
    return createBondConnection(node_name);
  } else if (transition == Transition::TRANSITION_DEACTIVATE) {
    bond_map_.erase(node_name);
  }

  return true;
}

bool
LifecycleManager::changeStateForAllNodes(std::uint8_t transition, bool hard_change)
{
  // Hard change will continue even if a node fails
  if (transition == Transition::TRANSITION_CONFIGURE ||
    transition == Transition::TRANSITION_ACTIVATE)
  {
    for (auto & node_name : node_names_) {
      try {
        if (!changeStateForNode(node_name, transition) && !hard_change) {
          return false;
        }
      } catch (const std::runtime_error & e) {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to change state for node: %s. Exception: %s.", node_name.c_str(), e.what());
        return false;
      }
    }
  } else {
    std::vector<std::string>::reverse_iterator rit;
    for (rit = node_names_.rbegin(); rit != node_names_.rend(); ++rit) {
      try {
        if (!changeStateForNode(*rit, transition) && !hard_change) {
          return false;
        }
      } catch (const std::runtime_error & e) {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to change state for node: %s. Exception: %s.", (*rit).c_str(), e.what());
        return false;
      }
    }
  }
  return true;
}

void
LifecycleManager::shutdownAllNodes()
{
  message("Deactivate, cleanup, and shutdown nodes");
  changeStateForAllNodes(Transition::TRANSITION_DEACTIVATE);
  changeStateForAllNodes(Transition::TRANSITION_CLEANUP);
  changeStateForAllNodes(Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
}

bool
LifecycleManager::startupWithCleanup()
{
  // 外层可能再次请求启动，必须先完成上一次失败后的清理。
  if (startup_cleanup_pending_) {
    if (!cleanupStartup()) {
      RCLCPP_ERROR(get_logger(), "Startup blocked; cleanup is incomplete.");
      return false;
    }
    startup_cleanup_pending_ = false;
  }

  bond_connection_failed_ = false;
  if (startup()) {
    return true;
  }
  // 配置或激活失败沿用原处理，仅对 bond 建连超时回退状态。
  if (bond_connection_failed_) {
    startup_cleanup_pending_ = !cleanupStartup();
    if (startup_cleanup_pending_) {
      RCLCPP_ERROR(get_logger(), "Startup cleanup failed; cleanup is required before startup.");
    }
  }
  return false;
}

bool
LifecycleManager::cleanupStartup()
{
  // 清理开始后系统不再可用，避免保留激活标志及旧的心跳检查定时器。
  system_active_ = false;
  destroyBondTimer();

  // 每次查询或转换最多等待 5 秒，包含服务发现和响应时间。
  constexpr auto service_timeout = 5s;
  bool success = true;
  // 沿用先全部停用、再全部清理的逆序流程；单个失败不阻断其余节点。
  for (const auto transition :
    {Transition::TRANSITION_DEACTIVATE, Transition::TRANSITION_CLEANUP})
  {
    const auto source_state = transition == Transition::TRANSITION_DEACTIVATE ?
      State::PRIMARY_STATE_ACTIVE : State::PRIMARY_STATE_INACTIVE;
    for (auto it = node_names_.rbegin(); it != node_names_.rend(); ++it) {
      if (!rclcpp::ok()) {
        bond_map_.clear();
        return false;
      }
      try {
        const auto state = node_map_.at(*it)->get_state_with_timeout(service_timeout);
        if (state == State::PRIMARY_STATE_UNCONFIGURED ||
          (transition == Transition::TRANSITION_DEACTIVATE &&
          state == State::PRIMARY_STATE_INACTIVE))
        {
          continue;
        }
        if (state != source_state) {
          RCLCPP_ERROR(
            get_logger(), "Startup cleanup failed for %s; unexpected state %u.",
            it->c_str(), static_cast<unsigned int>(state));
          success = false;
          continue;
        }
        message(transition_label_map_[transition] + *it);
        if (!node_map_.at(*it)->change_state_with_timeout(transition, service_timeout) ||
          node_map_.at(*it)->get_state_with_timeout(service_timeout) !=
          transition_state_map_[transition])
        {
          RCLCPP_ERROR(
            get_logger(), "Startup cleanup failed for %s; transition failed.", it->c_str());
          success = false;
        } else if (transition == Transition::TRANSITION_DEACTIVATE) {
          bond_map_.erase(*it);
        }
      } catch (const std::exception & e) {
        // 统一由日志补全句号，避免异常文本末尾已有句号。
        std::string error = e.what();
        while (!error.empty() && error.back() == '.') {
          error.pop_back();
        }
        RCLCPP_ERROR(
          get_logger(), "Startup cleanup failed for %s; %s. Timeout budget: %.1f s.",
          it->c_str(), error.c_str(), std::chrono::duration<double>(service_timeout).count());
        success = false;
      }
    }
  }
  // 包括超时的连接，下一轮必须重新创建 bond。
  bond_map_.clear();
  return success;
}

bool
LifecycleManager::startup()
{
  message("Starting managed nodes bringup...");
  if (!changeStateForAllNodes(Transition::TRANSITION_CONFIGURE) ||
    !changeStateForAllNodes(Transition::TRANSITION_ACTIVATE))
  {
    RCLCPP_ERROR(get_logger(), "Failed to bring up all requested nodes. Aborting bringup.");
    return false;
  }
  message("Managed nodes are active");
  system_active_ = true;
  createBondTimer();
  return true;
}

bool
LifecycleManager::shutdown()
{
  system_active_ = false;
  destroyBondTimer();

  message("Shutting down managed nodes...");
  shutdownAllNodes();
  destroyLifecycleServiceClients();
  message("Managed nodes have been shut down");
  return true;
}

bool
LifecycleManager::reset(bool hard_reset)
{
  system_active_ = false;
  destroyBondTimer();

  message("Resetting managed nodes...");
  // Should transition in reverse order
  if (!changeStateForAllNodes(Transition::TRANSITION_DEACTIVATE, hard_reset) ||
    !changeStateForAllNodes(Transition::TRANSITION_CLEANUP, hard_reset))
  {
    if (!hard_reset) {
      RCLCPP_ERROR(get_logger(), "Failed to reset nodes: aborting reset");
      return false;
    }
  }

  message("Managed nodes have been reset");
  return true;
}

bool
LifecycleManager::pause()
{
  system_active_ = false;
  destroyBondTimer();

  message("Pausing managed nodes...");
  if (!changeStateForAllNodes(Transition::TRANSITION_DEACTIVATE)) {
    RCLCPP_ERROR(get_logger(), "Failed to pause nodes: aborting pause");
    return false;
  }

  message("Managed nodes have been paused");
  return true;
}

bool
LifecycleManager::resume()
{
  // 清理失败的节点可能仍为 inactive，不能通过 RESUME 绕过清理直接激活。
  if (startup_cleanup_pending_) {
    RCLCPP_ERROR(get_logger(), "Resume blocked; startup cleanup is incomplete.");
    return false;
  }

  message("Resuming managed nodes...");
  if (!changeStateForAllNodes(Transition::TRANSITION_ACTIVATE)) {
    RCLCPP_ERROR(get_logger(), "Failed to resume nodes: aborting resume");
    return false;
  }

  message("Managed nodes are active");
  system_active_ = true;
  createBondTimer();
  return true;
}

void
LifecycleManager::createBondTimer()
{
  if (bond_timeout_.count() <= 0) {
    return;
  }

  message("Creating bond timer...");
  bond_timer_ = this->create_wall_timer(
    200ms,
    std::bind(&LifecycleManager::checkBondConnections, this),
    callback_group_);
}

void
LifecycleManager::destroyBondTimer()
{
  if (bond_timer_) {
    message("Terminating bond timer...");
    bond_timer_->cancel();
    bond_timer_.reset();
  }
}

void
LifecycleManager::onRclPreshutdown()
{
  RCLCPP_INFO(
    get_logger(), "Running Nav2 LifecycleManager rcl preshutdown (%s)",
    this->get_name());

  destroyBondTimer();

  /*
   * Dropping the bond map is what we really need here, but we drop the others
   * to prevent the bond map being used. Likewise, squash the service thread.
   */
  service_thread_.reset();
  node_names_.clear();
  node_map_.clear();
  bond_map_.clear();
}

void
LifecycleManager::registerRclPreshutdownCallback()
{
  rclcpp::Context::SharedPtr context = get_node_base_interface()->get_context();

  context->add_pre_shutdown_callback(
    std::bind(&LifecycleManager::onRclPreshutdown, this)
  );
}

void
LifecycleManager::checkBondConnections()
{
  if (!system_active_ || !rclcpp::ok() || bond_map_.empty()) {
    return;
  }

  for (auto & node_name : node_names_) {
    if (!rclcpp::ok()) {
      return;
    }

    if (bond_map_[node_name]->isBroken()) {
      message(
        std::string(
          "Have not received a heartbeat from " + node_name + "."));

      // if one is down, bring them all down
      RCLCPP_ERROR(
        get_logger(),
        "CRITICAL FAILURE: SERVER %s IS DOWN after not receiving a heartbeat for %i ms."
        " Shutting down related nodes.",
        node_name.c_str(), static_cast<int>(bond_timeout_.count()));
      reset(true);  // hard reset to transition all still active down
      // if a server crashed, it won't get cleared due to failed transition, clear manually
      bond_map_.clear();

      // Initialize the bond respawn timer to check if server comes back online
      // after a failure, within a maximum timeout period.
      if (attempt_respawn_reconnection_) {
        bond_respawn_timer_ = this->create_wall_timer(
          1s,
          std::bind(&LifecycleManager::checkBondRespawnConnection, this),
          callback_group_);
      }
      return;
    }
  }
}

void
LifecycleManager::checkBondRespawnConnection()
{
  // First attempt in respawn, start maximum duration to respawn
  if (bond_respawn_start_time_.nanoseconds() == 0) {
    bond_respawn_start_time_ = now();
  }

  // Note: system_active_ is inverted since this should be in a failure
  // condition. If another outside user actives the system again, this should not process.
  if (system_active_ || !rclcpp::ok() || node_names_.empty()) {
    bond_respawn_start_time_ = rclcpp::Time(0);
    bond_respawn_timer_.reset();
    return;
  }

  // Check number of live connections after a bond failure
  int live_servers = 0;
  const int max_live_servers = node_names_.size();
  for (auto & node_name : node_names_) {
    if (!rclcpp::ok()) {
      return;
    }

    try {
      node_map_[node_name]->get_state();  // Only won't throw if the server exists
      live_servers++;
    } catch (...) {
      break;
    }
  }

  // If all are alive, kill timer and retransition system to active
  // Else, check if maximum timeout has occurred
  if (live_servers == max_live_servers) {
    message("Successfully re-established connections from server respawns, starting back up.");
    bond_respawn_start_time_ = rclcpp::Time(0);
    bond_respawn_timer_.reset();
    startup();
  } else if (now() - bond_respawn_start_time_ >= bond_respawn_max_duration_) {
    message("Failed to re-establish connection from a server crash after maximum timeout.");
    bond_respawn_start_time_ = rclcpp::Time(0);
    bond_respawn_timer_.reset();
  }
}

#define ANSI_COLOR_RESET    "\x1b[0m"
#define ANSI_COLOR_BLUE     "\x1b[34m"

void
LifecycleManager::message(const std::string & msg)
{
  RCLCPP_INFO(get_logger(), ANSI_COLOR_BLUE "\33[1m%s\33[0m" ANSI_COLOR_RESET, msg.c_str());
}

}  // namespace nav2_lifecycle_manager

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_lifecycle_manager::LifecycleManager)
