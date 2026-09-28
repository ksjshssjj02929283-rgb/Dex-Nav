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

#ifndef NAV2_CONTROLLER__CONTROL_LOOP_TIMING_HPP_
#define NAV2_CONTROLLER__CONTROL_LOOP_TIMING_HPP_

#include <algorithm>
#include <array>
#include <cstddef>

#include "rclcpp/rclcpp.hpp"

namespace nav2_controller
{
namespace detail
{

// Fixed storage: timing collection must not allocate in the control loop.
struct TimingWindow
{
  static constexpr size_t kCapacity = 100;
  std::array<double, kCapacity> samples{};
  size_t count{0};
  size_t next{0};

  void add(double value)
  {
    samples[next] = value;
    next = (next + 1) % kCapacity;
    count = std::min(count + 1, kCapacity);
  }

  double percentile(double fraction) const
  {
    if (count == 0) {return 0.0;}
    auto sorted = samples;
    std::sort(sorted.begin(), sorted.begin() + count);
    return sorted[std::min(static_cast<size_t>(fraction * (count - 1)), count - 1)];
  }
};

struct PipelineTimingWindows
{
  TimingWindow total, pose, odom, local, speed_set_plan, speed_compute, viz;
  size_t cycles{0};

  void add(
    double total_ms, double pose_ms, double odom_ms, double local_ms,
    double set_plan_ms, double compute_ms, double viz_ms, const rclcpp::Logger & logger)
  {
    total.add(total_ms);
    pose.add(pose_ms);
    odom.add(odom_ms);
    local.add(local_ms);
    speed_set_plan.add(set_plan_ms);
    speed_compute.add(compute_ms);
    viz.add(viz_ms);
    if (++cycles % TimingWindow::kCapacity == 0) {
      RCLCPP_INFO(
        logger,
        "Planning timing last 100 (ms p50/p95): total=%.2f/%.2f pose=%.2f/%.2f "
        "odom=%.2f/%.2f local=%.2f/%.2f set_plan=%.2f/%.2f "
        "speed_compute=%.2f/%.2f viz=%.2f/%.2f",
        total.percentile(0.50), total.percentile(0.95),
        pose.percentile(0.50), pose.percentile(0.95),
        odom.percentile(0.50), odom.percentile(0.95),
        local.percentile(0.50), local.percentile(0.95),
        speed_set_plan.percentile(0.50), speed_set_plan.percentile(0.95),
        speed_compute.percentile(0.50), speed_compute.percentile(0.95),
        viz.percentile(0.50), viz.percentile(0.95));
    }
  }
};

struct ControlTimingWindows
{
  TimingWindow total, pose, progress, odom, compute, feedback, publish;
  size_t cycles{0};

  void add(
    double total_ms, double pose_ms, double progress_ms, double odom_ms,
    double compute_ms, double feedback_ms, double publish_ms,
    const rclcpp::Logger & logger)
  {
    total.add(total_ms);
    pose.add(pose_ms);
    progress.add(progress_ms);
    odom.add(odom_ms);
    compute.add(compute_ms);
    feedback.add(feedback_ms);
    publish.add(publish_ms);
    if (++cycles % TimingWindow::kCapacity == 0) {
      RCLCPP_INFO(
        logger,
        "Control timing last 100 (ms p50/p95): total=%.2f/%.2f pose=%.2f/%.2f "
        "progress=%.2f/%.2f odom=%.2f/%.2f compute=%.2f/%.2f "
        "feedback=%.2f/%.2f publish=%.2f/%.2f",
        total.percentile(0.50), total.percentile(0.95),
        pose.percentile(0.50), pose.percentile(0.95),
        progress.percentile(0.50), progress.percentile(0.95),
        odom.percentile(0.50), odom.percentile(0.95),
        compute.percentile(0.50), compute.percentile(0.95),
        feedback.percentile(0.50), feedback.percentile(0.95),
        publish.percentile(0.50), publish.percentile(0.95));
    }
  }
};

struct LoopTimingWindows
{
  TimingWindow total, costmap_wait, path_update, planning, control, goal;
  size_t cycles{0};
  size_t late_cycles{0};

  void add(
    double total_ms, double costmap_ms, double path_ms, double planning_ms,
    double control_ms, double goal_ms, double period_ms, const rclcpp::Logger & logger)
  {
    total.add(total_ms);
    costmap_wait.add(costmap_ms);
    path_update.add(path_ms);
    planning.add(planning_ms);
    control.add(control_ms);
    goal.add(goal_ms);
    late_cycles += total_ms > period_ms ? 1u : 0u;
    if (++cycles % TimingWindow::kCapacity == 0) {
      RCLCPP_INFO(
        logger,
        "FollowPath timing last 100 (ms p50/p95): total=%.2f/%.2f "
        "costmap_wait=%.2f/%.2f path=%.2f/%.2f planning=%.2f/%.2f "
        "control=%.2f/%.2f goal=%.2f/%.2f late=%zu/100 period=%.2f",
        total.percentile(0.50), total.percentile(0.95),
        costmap_wait.percentile(0.50), costmap_wait.percentile(0.95),
        path_update.percentile(0.50), path_update.percentile(0.95),
        planning.percentile(0.50), planning.percentile(0.95),
        control.percentile(0.50), control.percentile(0.95),
        goal.percentile(0.50), goal.percentile(0.95), late_cycles, period_ms);
      late_cycles = 0;
    }
  }
};

}  // namespace detail
}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__CONTROL_LOOP_TIMING_HPP_
