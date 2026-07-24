// Copyright (c) 2026 Dexory
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

#ifndef NAV2_CONTROLLER__NEAR_GOAL_ODOM_CONTEXT_HPP_
#define NAV2_CONTROLLER__NEAR_GOAL_ODOM_CONTEXT_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/time.hpp"

namespace nav2_controller
{

enum class NearGoalOdomMode
{
  DISABLED,
  OBSERVE,
  CONTROL
};

enum class NearGoalOdomStage
{
  INACTIVE,
  ODOM_POSITION,
  MAP_YAW_COARSE,
  ODOM_YAW_FINE,
  STOP_CONFIRM,
  SUCCESS,
  INVALID
};

enum class NearGoalControlDirective
{
  NORMAL,
  HOLD_ZERO,
  TERMINATE
};

enum class NearGoalTransitionReason
{
  NONE,
  ENTRY_STABLE,
  POSITION_STABLE,
  FINE_YAW_STABLE,
  STOP_STABLE,
  POSITION_DRIFT,
  TF_FAILURE,
  ODOM_JUMP,
  GLOBAL_CORRECTION,
  YAW_LATCH_DISCONTINUITY,
  ILLEGAL_TRANSITION,
  MODE_CHANGED,
  GOAL_CHANGED
};

struct NearGoalOdomSnapshot
{
  uint64_t goal_generation{0};
  uint64_t control_revision{0};
  NearGoalOdomMode mode{NearGoalOdomMode::DISABLED};
  NearGoalOdomStage stage{NearGoalOdomStage::INACTIVE};
  NearGoalControlDirective control_directive{NearGoalControlDirective::NORMAL};
  NearGoalTransitionReason transition_reason{NearGoalTransitionReason::NONE};

  bool position_goal_latched{false};
  bool position_reached_latched{false};
  bool yaw_goal_latched{false};
  bool tf_valid{false};
  bool odom_continuous{false};
  bool global_sanity_ok{false};

  geometry_msgs::msg::Pose goal_in_odom;
  geometry_msgs::msg::Pose position_reached_pose_in_odom;
  double yaw_goal_in_odom{0.0};

  double odom_lon_error{0.0};
  double odom_lat_error{0.0};
  double odom_yaw_error{0.0};
  double odom_drift_since_position_reached{0.0};
  double live_map_position_error{0.0};
  double live_map_lon_error{0.0};
  double live_map_lat_error{0.0};
  double live_map_yaw_error{0.0};
  double yaw_error_before_latch{0.0};
  double yaw_error_after_latch{0.0};
  double map_to_odom_delta_x{0.0};
  double map_to_odom_delta_y{0.0};
  double map_to_odom_delta_yaw{0.0};
  double global_yaw_consistency_delta{0.0};
  double actual_vx{0.0};
  double actual_wz{0.0};
  double tf_age{0.0};
  rclcpp::Time snapshot_stamp;
};

class NearGoalOdomSnapshotProvider
{
public:
  virtual ~NearGoalOdomSnapshotProvider() = default;

  // 返回线程安全的状态副本，插件在一个控制周期内只读取一次。
  virtual NearGoalOdomSnapshot snapshot() const = 0;
};

class NearGoalOdomContextConsumer
{
public:
  virtual ~NearGoalOdomContextConsumer() = default;

  // 注入只读状态源；generation不一致时插件必须拒绝使用旧状态。
  virtual void setNearGoalOdomContext(
    std::shared_ptr<const NearGoalOdomSnapshotProvider> provider,
    uint64_t expected_generation) = 0;
};

inline const char * toString(NearGoalOdomMode mode)
{
  switch (mode) {
    case NearGoalOdomMode::DISABLED:
      return "DISABLED";
    case NearGoalOdomMode::OBSERVE:
      return "OBSERVE";
    case NearGoalOdomMode::CONTROL:
      return "CONTROL";
  }
  return "UNKNOWN";
}

inline const char * toString(NearGoalOdomStage stage)
{
  switch (stage) {
    case NearGoalOdomStage::INACTIVE:
      return "INACTIVE";
    case NearGoalOdomStage::ODOM_POSITION:
      return "ODOM_POSITION";
    case NearGoalOdomStage::MAP_YAW_COARSE:
      return "MAP_YAW_COARSE";
    case NearGoalOdomStage::ODOM_YAW_FINE:
      return "ODOM_YAW_FINE";
    case NearGoalOdomStage::STOP_CONFIRM:
      return "STOP_CONFIRM";
    case NearGoalOdomStage::SUCCESS:
      return "SUCCESS";
    case NearGoalOdomStage::INVALID:
      return "INVALID";
  }
  return "UNKNOWN";
}

inline const char * toString(NearGoalControlDirective directive)
{
  switch (directive) {
    case NearGoalControlDirective::NORMAL:
      return "NORMAL";
    case NearGoalControlDirective::HOLD_ZERO:
      return "HOLD_ZERO";
    case NearGoalControlDirective::TERMINATE:
      return "TERMINATE";
  }
  return "UNKNOWN";
}

inline const char * toString(NearGoalTransitionReason reason)
{
  switch (reason) {
    case NearGoalTransitionReason::NONE:
      return "NONE";
    case NearGoalTransitionReason::ENTRY_STABLE:
      return "ENTRY_STABLE";
    case NearGoalTransitionReason::POSITION_STABLE:
      return "POSITION_STABLE";
    case NearGoalTransitionReason::FINE_YAW_STABLE:
      return "FINE_YAW_STABLE";
    case NearGoalTransitionReason::STOP_STABLE:
      return "STOP_STABLE";
    case NearGoalTransitionReason::POSITION_DRIFT:
      return "POSITION_DRIFT";
    case NearGoalTransitionReason::TF_FAILURE:
      return "TF_FAILURE";
    case NearGoalTransitionReason::ODOM_JUMP:
      return "ODOM_JUMP";
    case NearGoalTransitionReason::GLOBAL_CORRECTION:
      return "GLOBAL_CORRECTION";
    case NearGoalTransitionReason::YAW_LATCH_DISCONTINUITY:
      return "YAW_LATCH_DISCONTINUITY";
    case NearGoalTransitionReason::ILLEGAL_TRANSITION:
      return "ILLEGAL_TRANSITION";
    case NearGoalTransitionReason::MODE_CHANGED:
      return "MODE_CHANGED";
    case NearGoalTransitionReason::GOAL_CHANGED:
      return "GOAL_CHANGED";
  }
  return "UNKNOWN";
}

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__NEAR_GOAL_ODOM_CONTEXT_HPP_
