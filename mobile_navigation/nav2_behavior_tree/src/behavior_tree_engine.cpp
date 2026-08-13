// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Florian Gramss
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

#include "nav2_behavior_tree/behavior_tree_engine.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/utils/shared_library.h"

namespace nav2_behavior_tree
{

namespace
{

struct BtOverrunStats
{
  std::size_t count{0};
  std::size_t work_over_budget_count{0};
  std::size_t rate_schedule_lag_count{0};
  double max_work_ms{0.0};
  double max_work_over_budget_ms{0.0};
  double max_cancel_check_ms{0.0};
  double max_tick_root_ms{0.0};
  double max_on_loop_ms{0.0};
  double max_on_loop_other_ms{0.0};
  double max_feedback_other_ms{0.0};
  BtLoopTiming max_loop_timing;
  double worst_work_ms{0.0};
  double worst_work_tick_root_ms{0.0};
  double worst_work_on_loop_ms{0.0};
  BtLoopTiming worst_work_loop_timing;
  BtTickTiming worst_tick_root_timing;
};

void updateMax(double & maximum, const double value)
{
  maximum = std::max(maximum, value);
}

void updateMax(BtLoopTiming & maximum, const BtLoopTiming & value)
{
  updateMax(maximum.preempt_ms, value.preempt_ms);
  updateMax(maximum.topic_logger_flush_ms, value.topic_logger_flush_ms);
  updateMax(maximum.navigator_on_loop_ms, value.navigator_on_loop_ms);
  maximum.feedback_timing_valid = maximum.feedback_timing_valid || value.feedback_timing_valid;
  updateMax(maximum.feedback_tf_ms, value.feedback_tf_ms);
  updateMax(maximum.feedback_path_copy_ms, value.feedback_path_copy_ms);
  updateMax(maximum.feedback_path_calculation_ms, value.feedback_path_calculation_ms);
  updateMax(maximum.feedback_odom_eta_ms, value.feedback_odom_eta_ms);
  updateMax(maximum.feedback_finalize_ms, value.feedback_finalize_ms);
  updateMax(maximum.feedback_publish_ms, value.feedback_publish_ms);
  maximum.feedback_path_poses =
    std::max(maximum.feedback_path_poses, value.feedback_path_poses);
}

}  // namespace

BehaviorTreeEngine::BehaviorTreeEngine(const std::vector<std::string> & plugin_libraries)
{
  BT::SharedLibrary loader;
  for (const auto & p : plugin_libraries) {
    factory_.registerFromPlugin(loader.getOSName(p));
  }
}

BtStatus
BehaviorTreeEngine::run(
  BT::Tree * tree,
  std::function<void()> onLoop,
  std::function<bool()> cancelRequested,
  std::chrono::milliseconds loopTimeout)
{
  return run(tree, onLoop, cancelRequested, loopTimeout, nullptr);
}

BtStatus
BehaviorTreeEngine::run(
  BT::Tree * tree,
  std::function<void()> onLoop,
  std::function<bool()> cancelRequested,
  std::chrono::milliseconds loopTimeout,
  const BtLoopTiming * loopTiming)
{
  return run(tree, onLoop, cancelRequested, loopTimeout, loopTiming, nullptr);
}

BtStatus
BehaviorTreeEngine::run(
  BT::Tree * tree,
  std::function<void()> onLoop,
  std::function<bool()> cancelRequested,
  std::chrono::milliseconds loopTimeout,
  const BtLoopTiming * loopTiming,
  BtTickTiming * tickTiming)
{
  rclcpp::WallRate loopRate(loopTimeout);
  BT::NodeStatus result = BT::NodeStatus::RUNNING;
  const double loop_budget_ms = static_cast<double>(loopTimeout.count());
  constexpr auto report_interval = std::chrono::seconds(5);
  auto report_window_start = std::chrono::steady_clock::now();
  BtOverrunStats overrun_stats;

  const auto report_overruns = [&](const std::chrono::steady_clock::time_point report_time) {
      const double window_seconds =
        std::chrono::duration<double>(report_time - report_window_start).count();
      report_window_start = report_time;
      if (overrun_stats.count == 0) {
        return;
      }

      const auto & timing = overrun_stats.max_loop_timing;
      const auto & worst_loop = overrun_stats.worst_work_loop_timing;
      const auto & worst_tick = overrun_stats.worst_tick_root_timing;
      const auto node_name = [](const BtTickTiming::NodeTiming & node) {
          return node.name == nullptr ? "none" : node.name->c_str();
        };
      RCLCPP_WARN(
        rclcpp::get_logger("BehaviorTreeEngine"),
        "Behavior Tree tick rate %.2f was exceeded! 超周期汇总: window=%.2fs "
        "count=%zu work_over_budget_cycles=%zu rate_schedule_lag_cycles=%zu "
        "budget=%.3fms max [work=%.3fms over_budget=%.3fms "
        "cancel_check=%.3fms tick_root=%.3fms on_loop=%.3fms "
        "preempt=%.3fms bt_event_flush=%.3fms navigator=%.3fms "
        "on_loop_other=%.3fms] feedback_max [valid=%s tf=%.3fms path_copy=%.3fms "
        "path_calc=%.3fms odom_eta=%.3fms finalize=%.3fms publish=%.3fms "
        "other=%.3fms path_poses=%zu] worst_sample [work=%.3fms "
        "tick_root=%.3fms on_loop=%.3fms navigator=%.3fms] "
        "tick_root_worst_sample_inclusive [tick_root=%.3fms node1=%s/%.3fms "
        "node2=%s/%.3fms node3=%s/%.3fms bt_status_events=%.3fms/%zu] "
        "adaptive_inclusive [valid=%s total=%.3fms "
        "ports=%.3fms goal_update=%.3fms tf=%.3fms path_copy=%.3fms "
        "path_scan=%.3fms decision=%.3fms child=%.3fms other=%.3fms "
        "path_poses=%zu]",
        1.0 / (loopRate.period().count() * 1.0e-9), window_seconds,
        overrun_stats.count, overrun_stats.work_over_budget_count,
        overrun_stats.rate_schedule_lag_count, loop_budget_ms,
        overrun_stats.max_work_ms, overrun_stats.max_work_over_budget_ms,
        overrun_stats.max_cancel_check_ms, overrun_stats.max_tick_root_ms,
        overrun_stats.max_on_loop_ms, timing.preempt_ms,
        timing.topic_logger_flush_ms, timing.navigator_on_loop_ms,
        overrun_stats.max_on_loop_other_ms, timing.feedback_timing_valid ? "true" : "false",
        timing.feedback_tf_ms, timing.feedback_path_copy_ms,
        timing.feedback_path_calculation_ms, timing.feedback_odom_eta_ms,
        timing.feedback_finalize_ms, timing.feedback_publish_ms,
        overrun_stats.max_feedback_other_ms, timing.feedback_path_poses,
        overrun_stats.worst_work_ms, overrun_stats.worst_work_tick_root_ms,
        overrun_stats.worst_work_on_loop_ms, worst_loop.navigator_on_loop_ms,
        overrun_stats.max_tick_root_ms, node_name(worst_tick.slowest_nodes[0]),
        worst_tick.slowest_nodes[0].duration_ms,
        node_name(worst_tick.slowest_nodes[1]), worst_tick.slowest_nodes[1].duration_ms,
        node_name(worst_tick.slowest_nodes[2]), worst_tick.slowest_nodes[2].duration_ms,
        worst_tick.status_event_ms, worst_tick.status_event_count,
        worst_tick.adaptive.valid ? "true" : "false", worst_tick.adaptive.total_ms,
        worst_tick.adaptive.ports_ms, worst_tick.adaptive.goal_update_ms,
        worst_tick.adaptive.tf_ms, worst_tick.adaptive.path_copy_ms,
        worst_tick.adaptive.path_scan_ms, worst_tick.adaptive.decision_ms,
        worst_tick.adaptive.child_ms, worst_tick.adaptive.other_ms,
        worst_tick.adaptive.path_poses);
      overrun_stats = BtOverrunStats{};
      // 汇总日志是BT线程内的诊断操作，重置周期基准，避免把日志开销算成下一轮调度积压。
      loopRate.reset();
    };

  // Loop until something happens with ROS or the node completes
  try {
    while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
      const auto loop_start = std::chrono::steady_clock::now();
      if (tickTiming != nullptr) {
        tickTiming->reset();
      }
      if (cancelRequested()) {
        report_overruns(loop_start);
        tree->rootNode()->halt();
        return BtStatus::CANCELED;
      }
      const auto cancel_check_end = std::chrono::steady_clock::now();

      result = tree->tickRoot();
      const auto tick_root_end = std::chrono::steady_clock::now();

      onLoop();
      const auto on_loop_end = std::chrono::steady_clock::now();

      const bool rate_exceeded = !loopRate.sleep();
      const auto loop_end = std::chrono::steady_clock::now();
      if (rate_exceeded) {
        const auto to_ms = [](const auto & begin, const auto & end) {
            return std::chrono::duration<double, std::milli>(end - begin).count();
          };
        const double work_ms = to_ms(loop_start, on_loop_end);
        const double cancel_check_ms = to_ms(loop_start, cancel_check_end);
        const double tick_root_ms = to_ms(cancel_check_end, tick_root_end);
        const double on_loop_ms = to_ms(tick_root_end, on_loop_end);
        const double work_over_budget_ms =
          work_ms > loop_budget_ms ? work_ms - loop_budget_ms : 0.0;

        const BtLoopTiming empty_timing;
        const auto & timing = loopTiming == nullptr ? empty_timing : *loopTiming;
        const double on_loop_known_ms =
          timing.preempt_ms + timing.topic_logger_flush_ms + timing.navigator_on_loop_ms;
        const double on_loop_other_ms =
          on_loop_ms > on_loop_known_ms ? on_loop_ms - on_loop_known_ms : 0.0;
        const double feedback_known_ms =
          timing.feedback_tf_ms + timing.feedback_path_copy_ms +
          timing.feedback_path_calculation_ms + timing.feedback_odom_eta_ms +
          timing.feedback_finalize_ms + timing.feedback_publish_ms;
        const double feedback_other_ms =
          timing.navigator_on_loop_ms > feedback_known_ms ?
          timing.navigator_on_loop_ms - feedback_known_ms : 0.0;

        ++overrun_stats.count;
        if (work_ms >= loop_budget_ms) {
          ++overrun_stats.work_over_budget_count;
        } else {
          // 当前工作未超过预算，但WallRate仍无时间休眠，说明固定调度点已有积压。
          ++overrun_stats.rate_schedule_lag_count;
        }
        updateMax(overrun_stats.max_work_ms, work_ms);
        updateMax(overrun_stats.max_work_over_budget_ms, work_over_budget_ms);
        updateMax(overrun_stats.max_cancel_check_ms, cancel_check_ms);
        if (tick_root_ms > overrun_stats.max_tick_root_ms) {
          overrun_stats.max_tick_root_ms = tick_root_ms;
          overrun_stats.worst_tick_root_timing =
            tickTiming == nullptr ? BtTickTiming{} : *tickTiming;
        }
        updateMax(overrun_stats.max_on_loop_ms, on_loop_ms);
        updateMax(overrun_stats.max_on_loop_other_ms, on_loop_other_ms);
        updateMax(overrun_stats.max_feedback_other_ms, feedback_other_ms);
        updateMax(overrun_stats.max_loop_timing, timing);
        if (work_ms > overrun_stats.worst_work_ms) {
          overrun_stats.worst_work_ms = work_ms;
          overrun_stats.worst_work_tick_root_ms = tick_root_ms;
          overrun_stats.worst_work_on_loop_ms = on_loop_ms;
          overrun_stats.worst_work_loop_timing = timing;
        }
      }

      if (loop_end - report_window_start >= report_interval) {
        report_overruns(loop_end);
      }
    }
  } catch (const std::exception & ex) {
    report_overruns(std::chrono::steady_clock::now());
    RCLCPP_ERROR(
      rclcpp::get_logger("BehaviorTreeEngine"),
      "Behavior tree threw exception: %s. Exiting with failure.", ex.what());
    return BtStatus::FAILED;
  }

  report_overruns(std::chrono::steady_clock::now());
  return (result == BT::NodeStatus::SUCCESS) ? BtStatus::SUCCEEDED : BtStatus::FAILED;
}

BT::Tree
BehaviorTreeEngine::createTreeFromText(
  const std::string & xml_string,
  BT::Blackboard::Ptr blackboard)
{
  return factory_.createTreeFromText(xml_string, blackboard);
}

BT::Tree
BehaviorTreeEngine::createTreeFromFile(
  const std::string & file_path,
  BT::Blackboard::Ptr blackboard)
{
  return factory_.createTreeFromFile(file_path, blackboard);
}

// In order to re-run a Behavior Tree, we must be able to reset all nodes to the initial state
void
BehaviorTreeEngine::haltAllActions(BT::TreeNode * root_node)
{
  if (!root_node) {
    return;
  }

  // this halt signal should propagate through the entire tree.
  root_node->halt();

  // but, just in case...
  auto visitor = [](BT::TreeNode * node) {
      if (node->status() == BT::NodeStatus::RUNNING) {
        node->halt();
      }
    };
  BT::applyRecursiveVisitor(root_node, visitor);
}

}  // namespace nav2_behavior_tree
