// Copyright (c) 2026 nav2_controller contributors
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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>

#include "nav2_controller/control_loop_timing.hpp"
#include "nav2_controller/split_control.hpp"
#include "nav2_controller/controller_server.hpp"

namespace nav2_controller
{

using detail::LoopTimingWindows;
using detail::TimingWindow;

detail::SplitControlRates detail::SplitControlRates::fromGoal(
  const nav2_msgs::action::FollowPath::Goal & goal, double default_control_hz)
{
  const auto valid_rate = [](double requested, double fallback) {
      const double rate = std::isfinite(requested) && requested > 0.0 ? requested : fallback;
      return std::clamp(rate, 1.0, 100.0);
    };
  const double control_hz = valid_rate(goal.control_frequency, default_control_hz);
  const double planning_hz = valid_rate(goal.planning_frequency, control_hz);
  const double max_plan_age =
    std::isfinite(goal.planning_result_max_age) && goal.planning_result_max_age > 0.0 ?
    goal.planning_result_max_age : 2.5 / planning_hz;
  return {control_hz, planning_hz, max_plan_age};
}

void ControllerServer::computeControlSplit()
{
  using Clock = std::chrono::steady_clock;
  const auto goal = action_server_->get_current_goal();
  const auto rates = detail::SplitControlRates::fromGoal(*goal, controller_frequency_);
  const double control_hz = rates.control_hz;
  const double planning_hz = rates.planning_hz;
  const double max_age = rates.max_plan_age;
  active_split_control_frequency_.store(control_hz);
  struct FrequencyReset
  {
    std::atomic<double> & value;
    ~FrequencyReset() {value.store(0.0);}
  } frequency_reset{active_split_control_frequency_};
  const auto planning_period = std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(1.0 / planning_hz));
  const auto planning_phase = std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(0.5 / control_hz));
  std::mutex wake_mutex;
  std::condition_variable wake;
  std::atomic<bool> stop{false};
  bool replan_now = false;
  bool plan_ready = false;
  uint64_t planned_path_id = 0;
  Clock::time_point plan_completed_at{};
  std::exception_ptr planning_error;
  bool continuity_reset_pending = false;
  bool invalidate_plan_pending = false;
  size_t busy_cycles = 0;
  size_t stale_cycles = 0;
  size_t invalid_cycles = 0;
  size_t control_cycles = 0;
  TimingWindow plan_age_ms;
  const auto report_cycle = [&]() {
      if (++control_cycles % TimingWindow::kCapacity == 0) {
        RCLCPP_INFO(
          get_logger(),
          "Split control last 100: busy=%zu stale=%zu invalid=%zu "
          "plan_age_p95=%.2f ms planning=%.2f Hz control=%.2f Hz.",
          busy_cycles, stale_cycles, invalid_cycles,
          plan_age_ms.percentile(0.95), planning_hz, control_hz);
        busy_cycles = stale_cycles = invalid_cycles = 0;
        plan_age_ms = TimingWindow();
      }
    };

  // Bypass continuity mutation while the planner owns the state mutex. The
  // next successful try_lock resets it before consuming a trajectory.
  const auto publish_safety_zero = [this, &continuity_reset_pending]() {
      continuity_reset_pending = true;
      geometry_msgs::msg::TwistStamped zero;
      zero.header.frame_id = costmap_ros_->getBaseFrameID();
      zero.header.stamp = now();
      publishVelocity(zero);
    };
  const auto publish_safety_hard_stop = [this, &publish_safety_zero]() {
      if (command_output_enabled_ && hard_stop_publisher_ &&
        hard_stop_publisher_->is_activated())
      {
        hard_stop_publisher_->publish(std::make_unique<std_msgs::msg::Empty>());
      }
      publish_safety_zero();
    };

  std::thread planning_worker([&]() {
      // Start between control ticks so ordinary short plans do not repeatedly
      // collide with the same control tick at harmonic frequencies.
      auto next_plan = Clock::now() + planning_phase;
      while (rclcpp::ok()) {
        {
          std::unique_lock<std::mutex> wake_lock(wake_mutex);
          wake.wait_until(wake_lock, next_plan, [&]() {return stop.load() || replan_now;});
          if (stop.load()) {return;}
          replan_now = false;
        }
        {
          std::lock_guard<std::mutex> state_lock(split_control_state_mutex_);
          if (stop.load()) {return;}
          if (costmap_ros_->isCurrent()) {
            try {
              const bool succeeded = runPlanningPipeline(current_path_);
              planned_path_id = speed_plan_path_id_;
              plan_completed_at = Clock::now();
              plan_ready = succeeded && costmap_ros_->isCurrent();
            } catch (...) {
              if (speed_plan_cycle_.active) {
                speed_plan_cycle_.planning_result = "planning_exception";
                publishSpeedPlanDiagnostics(
                  false, false, std::numeric_limits<double>::quiet_NaN(),
                  std::numeric_limits<double>::quiet_NaN(), "not_run");
              }
              plan_ready = false;
              planning_error = std::current_exception();
              return;
            }
          } else {
            plan_ready = false;
          }
        }
        do {
          next_plan += planning_period;
        } while (next_plan <= Clock::now());
      }
    });
  const auto stop_worker = [&]() {
      {
        std::lock_guard<std::mutex> wake_lock(wake_mutex);
        stop.store(true);
      }
      wake.notify_all();
      if (planning_worker.joinable()) {planning_worker.join();}
    };
  const auto request_replan = [&]() {
      {
        std::lock_guard<std::mutex> wake_lock(wake_mutex);
        replan_now = true;
      }
      wake.notify_one();
    };

  RCLCPP_INFO(
    get_logger(),
    "Split FollowPath started: planning=%.2f Hz control=%.2f Hz max_plan_age=%.3f s.",
    planning_hz, control_hz, max_age);
  rclcpp::WallRate loop_rate(control_hz);
  LoopTimingWindows loop_timing;
  const auto ms = [](Clock::time_point a, Clock::time_point b) {
      return std::chrono::duration<double, std::milli>(b - a).count();
    };
  try {
    while (rclcpp::ok()) {
      const auto cycle_start = Clock::now();
      if (action_server_ == nullptr || !action_server_->is_server_active()) {
        publish_safety_zero();
        stop_worker();
        lockPrepareForCurrentSession("action server inactive");
        resetEndLocalizationCapabilities(goal_generation_);
        return;
      }
      if (action_server_->is_cancel_requested()) {
        publish_safety_hard_stop();
        stop_worker();
        publishHardStopVelocity();
        publishNavigationStatus(nav_interfaces::msg::NavigationStatus::IDLE);
        lockPrepareForCurrentSession("goal canceled");
        resetEndLocalizationCapabilities(goal_generation_);
        action_server_->terminate_all();
        return;
      }
      if (!costmap_ros_->isCurrent()) {
        ++stale_cycles;
        invalidate_plan_pending = true;
        publish_safety_zero();
        const auto cycle_end = Clock::now();
        loop_timing.add(
          ms(cycle_start, cycle_end), ms(cycle_start, cycle_end), 0, 0, 0, 0,
          1000.0 / control_hz, get_logger());
        report_cycle();
        loop_rate.sleep();
        continue;
      }
      const auto costmap_ready = Clock::now();
      std::unique_lock<std::mutex> state_lock(split_control_state_mutex_, std::try_to_lock);
      if (!state_lock.owns_lock()) {
        ++busy_cycles;
        publish_safety_zero();
        const auto cycle_end = Clock::now();
        loop_timing.add(
          ms(cycle_start, cycle_end), ms(cycle_start, costmap_ready), 0, 0, 0, 0,
          1000.0 / control_hz, get_logger());
        report_cycle();
        loop_rate.sleep();
        continue;
      }
      if (continuity_reset_pending) {
        invalidateSpeedPlanningContinuity();
        continuity_reset_pending = false;
      }
      if (invalidate_plan_pending) {
        plan_ready = false;
        invalidate_plan_pending = false;
        request_replan();
      }
      const uint64_t old_path_id = speed_plan_path_id_;
      const auto path_start = Clock::now();
      updateGlobalPath();
      syncPrepareSession();
      const auto path_ready = Clock::now();
      if (speed_plan_path_id_ != old_path_id) {
        plan_ready = false;
        request_replan();
      }
      if (planning_error) {std::rethrow_exception(planning_error);}
      const double plan_age = planned_path_id == speed_plan_path_id_ ?
        std::chrono::duration<double>(path_ready - plan_completed_at).count() :
        std::numeric_limits<double>::infinity();
      if (std::isfinite(plan_age)) {plan_age_ms.add(plan_age * 1000.0);}
      double control_ms = 0.0;
      double goal_ms = 0.0;
      bool reached = false;
      if (plan_ready && plan_age <= max_age) {
        const auto control_start = Clock::now();
        computeAndPublishVelocity();
        const auto control_end = Clock::now();
        control_ms = ms(control_start, control_end);
        const auto goal_start = Clock::now();
        reached = isGoalReached();
        goal_ms = ms(goal_start, Clock::now());
      } else {
        ++invalid_cycles;
        publishZeroVelocity();
      }
      state_lock.unlock();
      const auto cycle_end = Clock::now();
      loop_timing.add(
        ms(cycle_start, cycle_end), ms(cycle_start, costmap_ready),
        ms(path_start, path_ready), 0, control_ms, goal_ms,
        1000.0 / control_hz, get_logger());
      report_cycle();
      if (reached) {
        stop_worker();
        resetStagedControlState();
        publishZeroVelocity();
        publishNavigationStatus(nav_interfaces::msg::NavigationStatus::FINISHED);
        action_server_->succeeded_current();
        publishNavigationStatus(nav_interfaces::msg::NavigationStatus::IDLE);
        return;
      }
      if (!loop_rate.sleep()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Split control loop late: target=%.2f Hz elapsed=%.2f ms busy=%zu stale=%zu.",
          control_hz, ms(cycle_start, cycle_end), busy_cycles, stale_cycles);
      }
    }
    publish_safety_zero();
    stop_worker();
  } catch (...) {
    publish_safety_hard_stop();
    stop_worker();
    throw;
  }
}

}  // namespace nav2_controller
