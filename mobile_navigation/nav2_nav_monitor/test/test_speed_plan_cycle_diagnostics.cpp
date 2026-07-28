// Copyright 2026 DEXE Robot Engineer
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
#include <cmath>
#include <string>

#include "gtest/gtest.h"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "nav2_nav_monitor/nav_monitor_node.hpp"

namespace nav2_nav_monitor
{
namespace
{

void addValue(
  diagnostic_msgs::msg::DiagnosticStatus & status,
  const std::string & key,
  const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  status.values.push_back(item);
}

diagnostic_msgs::msg::DiagnosticStatus makeStatus(bool receipt_valid)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "navigation_speed_plan";
  addValue(status, "schema_version", "1");
  addValue(status, "cycle_id", "42");
  addValue(status, "path_id", "7");
  addValue(status, "planning_succeeded", "true");
  addValue(status, "planning_result", "success");
  addValue(status, "planner", "SCurveSpeedPlanner");
  addValue(status, "local_path_point_count", "81");
  addValue(status, "local_path_length", "2.125");
  addValue(status, "trajectory_point_count", "81");
  addValue(status, "boundary_computed", "true");
  addValue(status, "boundary_reason", "inherited");
  addValue(status, "boundary_velocity", "0.083");
  addValue(status, "boundary_acceleration", "-0.210");
  addValue(status, "boundary_acceleration_valid", "true");
  addValue(status, "boundary_source_age", "0.101");
  addValue(status, "boundary_source_stamp_sec", "1234");
  addValue(status, "boundary_source_stamp_nanosec", "567000000");
  addValue(status, "terminal_valid", "true");
  addValue(status, "terminal_infeasible", "false");
  addValue(status, "control_succeeded", "true");
  addValue(status, "control_result", "success");
  addValue(status, "control_receipt_valid", receipt_valid ? "true" : "false");
  if (receipt_valid) {
    addValue(status, "control_target_velocity", "0.061");
    addValue(status, "control_target_acceleration", "-0.180");
  }
  addValue(status, "controller", "PIDController");
  addValue(status, "phase", "approach");
  return status;
}

void setValue(
  diagnostic_msgs::msg::DiagnosticStatus & status,
  const std::string & key,
  const std::string & value)
{
  for (auto & item : status.values) {
    if (item.key == key) {
      item.value = value;
      return;
    }
  }
  addValue(status, key, value);
}

void removeValue(
  diagnostic_msgs::msg::DiagnosticStatus & status,
  const std::string & key)
{
  status.values.erase(
    std::remove_if(
      status.values.begin(), status.values.end(),
      [&key](const auto & item) {return item.key == key;}),
    status.values.end());
}

TEST(SpeedPlanCycleDiagnostics, ParsesCompleteCycle)
{
  SpeedPlanCycleSample sample;
  ASSERT_TRUE(parseSpeedPlanCycleStatus(makeStatus(true), 3.25, sample));

  EXPECT_DOUBLE_EQ(sample.time, 3.25);
  EXPECT_EQ(sample.cycle_id, 42U);
  EXPECT_EQ(sample.path_id, 7U);
  EXPECT_TRUE(sample.planning_succeeded);
  EXPECT_EQ(sample.local_path_point_count, 81U);
  EXPECT_DOUBLE_EQ(sample.local_path_length, 2.125);
  EXPECT_TRUE(sample.boundary_computed);
  EXPECT_EQ(sample.boundary_reason, "inherited");
  EXPECT_DOUBLE_EQ(sample.boundary_velocity, 0.083);
  EXPECT_DOUBLE_EQ(sample.boundary_acceleration, -0.210);
  EXPECT_TRUE(sample.boundary_acceleration_valid);
  EXPECT_TRUE(sample.terminal_valid);
  EXPECT_FALSE(sample.terminal_infeasible);
  EXPECT_TRUE(sample.control_succeeded);
  EXPECT_TRUE(sample.control_receipt_valid);
  EXPECT_DOUBLE_EQ(sample.control_target_velocity, 0.061);
  EXPECT_DOUBLE_EQ(sample.control_target_acceleration, -0.180);
  EXPECT_EQ(sample.controller, "PIDController");
  EXPECT_EQ(sample.phase, "approach");
}

TEST(SpeedPlanCycleDiagnostics, KeepsInvalidReceiptWithoutInventingTarget)
{
  SpeedPlanCycleSample sample;
  ASSERT_TRUE(parseSpeedPlanCycleStatus(makeStatus(false), 0.5, sample));

  EXPECT_FALSE(sample.control_receipt_valid);
  EXPECT_TRUE(std::isnan(sample.control_target_velocity));
  EXPECT_TRUE(std::isnan(sample.control_target_acceleration));
}

TEST(SpeedPlanCycleDiagnostics, ParsesPlanningFailureWithoutBoundaryOrReceipt)
{
  auto status = makeStatus(false);
  setValue(status, "planning_succeeded", "false");
  setValue(status, "planning_result", "pose_lookup_failed");
  setValue(status, "boundary_computed", "false");
  setValue(status, "boundary_reason", "not_computed");
  setValue(status, "terminal_valid", "false");
  setValue(status, "control_succeeded", "false");
  setValue(status, "control_result", "not_run");
  setValue(status, "controller", "not_run");
  removeValue(status, "boundary_velocity");
  removeValue(status, "boundary_acceleration");
  removeValue(status, "boundary_acceleration_valid");
  removeValue(status, "boundary_source_age");
  removeValue(status, "boundary_source_stamp_sec");
  removeValue(status, "boundary_source_stamp_nanosec");

  SpeedPlanCycleSample sample;
  ASSERT_TRUE(parseSpeedPlanCycleStatus(status, 0.25, sample));
  EXPECT_FALSE(sample.planning_succeeded);
  EXPECT_EQ(sample.planning_result, "pose_lookup_failed");
  EXPECT_FALSE(sample.boundary_computed);
  EXPECT_FALSE(sample.terminal_valid);
  EXPECT_FALSE(sample.control_succeeded);
  EXPECT_FALSE(sample.control_receipt_valid);
  EXPECT_EQ(sample.controller, "not_run");
}

TEST(SpeedPlanCycleDiagnostics, RejectsUnsupportedOrIncompleteSchema)
{
  auto status = makeStatus(true);
  status.values.front().value = "2";
  SpeedPlanCycleSample sample;
  EXPECT_FALSE(parseSpeedPlanCycleStatus(status, 1.0, sample));

  status = makeStatus(true);
  status.values.erase(status.values.begin() + 1);  // cycle_id
  EXPECT_FALSE(parseSpeedPlanCycleStatus(status, 1.0, sample));
}

}  // namespace
}  // namespace nav2_nav_monitor
