/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "nav2_controller/plugins/precision_longitudinal_goal_checker.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include "angles/angles.h"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_controller
{
namespace
{
constexpr double kDefaultDesiredLonGoalTolerance = 0.01;
constexpr double kDefaultDesiredLatGoalTolerance = 0.01;
constexpr double kDefaultAcceptLonGoalTolerance = 0.02;
constexpr double kDefaultAcceptLatGoalTolerance = 0.02;
constexpr double kDefaultYawGoalTolerance = 0.02;
constexpr int kDefaultRefineCycles = 20;
constexpr int kDefaultStableWindowSize = 10;
constexpr bool kDefaultEnableHighPrecision = true;
constexpr bool kDefaultCheckStopped = false;
constexpr double kDefaultTransStoppedVelocity = 0.01;
constexpr double kDefaultRotStoppedVelocity = 0.02;
}  // namespace

PrecisionLongitudinalGoalChecker::PrecisionLongitudinalGoalChecker()
: desired_lon_goal_tolerance_(kDefaultDesiredLonGoalTolerance),
  desired_lat_goal_tolerance_(kDefaultDesiredLatGoalTolerance),
  accept_lon_goal_tolerance_(kDefaultAcceptLonGoalTolerance),
  accept_lat_goal_tolerance_(kDefaultAcceptLatGoalTolerance),
  yaw_goal_tolerance_(kDefaultYawGoalTolerance),
  refine_cycles_(kDefaultRefineCycles),
  stable_window_size_(kDefaultStableWindowSize),
  refine_count_(0),
  accept_refine_restart_used_(false),
  desired_refine_restart_used_(false),
  enable_high_precision_(kDefaultEnableHighPrecision),
  check_stopped_(kDefaultCheckStopped),
  trans_stopped_velocity_(kDefaultTransStoppedVelocity),
  rot_stopped_velocity_(kDefaultRotStoppedVelocity)
{
}

void PrecisionLongitudinalGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>/*costmap_ros*/)
{
  plugin_name_ = plugin_name;
  auto node = parent.lock();
  clock_ = node->get_clock();
  goal_error_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/navigation_goal_error",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());
  goal_error_pub_->on_activate();

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".desired_lon_goal_tolerance",
    rclcpp::ParameterValue(kDefaultDesiredLonGoalTolerance));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".desired_lat_goal_tolerance",
    rclcpp::ParameterValue(kDefaultDesiredLatGoalTolerance));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".accept_lon_goal_tolerance",
    rclcpp::ParameterValue(kDefaultAcceptLonGoalTolerance));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".accept_lat_goal_tolerance",
    rclcpp::ParameterValue(kDefaultAcceptLatGoalTolerance));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".yaw_goal_tolerance", rclcpp::ParameterValue(kDefaultYawGoalTolerance));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".refine_cycles", rclcpp::ParameterValue(kDefaultRefineCycles));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".stable_window_size", rclcpp::ParameterValue(kDefaultStableWindowSize));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".enable_high_precision",
    rclcpp::ParameterValue(kDefaultEnableHighPrecision));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".check_stopped", rclcpp::ParameterValue(kDefaultCheckStopped));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".trans_stopped_velocity",
    rclcpp::ParameterValue(kDefaultTransStoppedVelocity));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".rot_stopped_velocity",
    rclcpp::ParameterValue(kDefaultRotStoppedVelocity));

  node->get_parameter(plugin_name + ".desired_lon_goal_tolerance", desired_lon_goal_tolerance_);
  node->get_parameter(plugin_name + ".desired_lat_goal_tolerance", desired_lat_goal_tolerance_);
  node->get_parameter(plugin_name + ".accept_lon_goal_tolerance", accept_lon_goal_tolerance_);
  node->get_parameter(plugin_name + ".accept_lat_goal_tolerance", accept_lat_goal_tolerance_);
  node->get_parameter(plugin_name + ".yaw_goal_tolerance", yaw_goal_tolerance_);
  node->get_parameter(plugin_name + ".refine_cycles", refine_cycles_);
  node->get_parameter(plugin_name + ".stable_window_size", stable_window_size_);
  node->get_parameter(plugin_name + ".enable_high_precision", enable_high_precision_);
  node->get_parameter(plugin_name + ".check_stopped", check_stopped_);
  node->get_parameter(plugin_name + ".trans_stopped_velocity", trans_stopped_velocity_);
  node->get_parameter(plugin_name + ".rot_stopped_velocity", rot_stopped_velocity_);

  refine_cycles_ = std::max(0, refine_cycles_);
  stable_window_size_ = std::max(1, stable_window_size_);

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&PrecisionLongitudinalGoalChecker::dynamicParametersCallback, this, _1));
}

void PrecisionLongitudinalGoalChecker::reset()
{
  resetRefineState();
  accept_refine_restart_used_ = false;
  desired_refine_restart_used_ = false;
}

bool PrecisionLongitudinalGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose,
  const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & velocity)
{
  const double goal_yaw = tf2::getYaw(goal_pose.orientation);
  const double current_yaw = tf2::getYaw(query_pose.orientation);
  const double dx = query_pose.position.x - goal_pose.position.x;
  const double dy = query_pose.position.y - goal_pose.position.y;

  // 按目标点坐标系计算 current-goal 的纵向/横向误差，避免机器人当前朝向影响停车精度判断。
  const double lon_err = std::cos(goal_yaw) * dx + std::sin(goal_yaw) * dy;
  const double lat_err = -std::sin(goal_yaw) * dx + std::cos(goal_yaw) * dy;
  // yaw_err 与位置误差保持同一符号语义：current-goal。
  const double dyaw = angles::shortest_angular_distance(goal_yaw, current_yaw);
  const double abs_lon_err = std::fabs(lon_err);
  const double abs_lat_err = std::fabs(lat_err);
  const double abs_yaw_err = std::fabs(dyaw);

  const bool in_accept_window =
    abs_lon_err <= accept_lon_goal_tolerance_ &&
    abs_lat_err <= accept_lat_goal_tolerance_;

  if (!in_accept_window) {
    const double distance_err = std::hypot(lon_err, lat_err);
    if (distance_err <= 0.1) {
      RCLCPP_INFO(
        rclcpp::get_logger("PrecisionLongitudinalGoalChecker"),
        "lon_err=%.4f, lat_err=%.4f, "
        "yaw_err=%.4f，尚未进入接受窗口 accept=(%.4f, %.4f)",
        lon_err, lat_err, dyaw,
        accept_lon_goal_tolerance_, accept_lat_goal_tolerance_);
    }
    resetRefineState();
    return false;
  }

  recordAbsErrors(abs_lon_err, abs_lat_err, abs_yaw_err);

  const bool in_desired_window =
    abs_lon_err <= desired_lon_goal_tolerance_ &&
    abs_lat_err <= desired_lat_goal_tolerance_;
  const int stable_window_count = static_cast<int>(lon_errors_.size());
  const double avg_lon_err = lon_errors_.empty() ? 0.0 : meanAbs(lon_errors_);
  const double avg_lat_err = lat_errors_.empty() ? 0.0 : meanAbs(lat_errors_);
  const double avg_yaw_err = yaw_errors_.empty() ? 0.0 : meanAbs(yaw_errors_);
  const bool stable_window_ready =
    stable_window_count >= stable_window_size_ &&
    static_cast<int>(lat_errors_.size()) >= stable_window_size_ &&
    static_cast<int>(yaw_errors_.size()) >= stable_window_size_;
  const bool stable_desired =
    stable_window_ready &&
    avg_lon_err <= desired_lon_goal_tolerance_ &&
    avg_lat_err <= desired_lat_goal_tolerance_ &&
    avg_yaw_err <= yaw_goal_tolerance_;
  const int displayed_refine_count = std::min(refine_count_ + 1, refine_cycles_);

  const bool yaw_ok = abs_yaw_err <= yaw_goal_tolerance_;
  if (!yaw_ok) {
    const auto status = in_desired_window ? PrecisionStatus::YawUnstable :
      PrecisionStatus::AcceptOnly;
    logStatusTransition(
      status, lon_err, lat_err, dyaw, stable_desired,
      stable_window_count, avg_lon_err, avg_lat_err, avg_yaw_err);
  }

  bool stopped = true;
  if (check_stopped_) {
    stopped =
      std::hypot(velocity.linear.x, velocity.linear.y) <= trans_stopped_velocity_ &&
      std::fabs(velocity.angular.z) <= rot_stopped_velocity_;
  }

  auto log_stopped_unstable = [&]() {
      logStatusTransition(
        PrecisionStatus::StoppedUnstable, lon_err, lat_err, dyaw, stable_desired,
        stable_window_count, avg_lon_err, avg_lat_err, avg_yaw_err);
      RCLCPP_INFO(
        rclcpp::get_logger("PrecisionLongitudinalGoalChecker"),
        "位置和角度已满足成功条件，但速度尚未满足停止判定："
        "lon_err=%.4f, lat_err=%.4f, yaw_err=%.4f, refine_count=%d/%d, "
        "current_desired=%s, yaw_ok=%s, avg_stable=%s, "
        "avg_abs=(%.4f, %.4f, %.4f), window=%d/%d, restart_used=(accept=%s, desired=%s)",
        lon_err, lat_err, dyaw, displayed_refine_count, refine_cycles_,
        in_desired_window ? "true" : "false",
        yaw_ok ? "true" : "false",
        stable_desired ? "true" : "false",
        avg_lon_err, avg_lat_err, avg_yaw_err,
        stable_window_count, stable_window_size_,
        accept_refine_restart_used_ ? "true" : "false",
        desired_refine_restart_used_ ? "true" : "false");
    };

  if (yaw_ok) {
    logStatusTransition(
      in_desired_window ?
      (stable_desired ? PrecisionStatus::DesiredStable : PrecisionStatus::DesiredUnstable) :
      PrecisionStatus::AcceptOnly,
      lon_err, lat_err, dyaw, stable_desired,
      stable_window_count, avg_lon_err, avg_lat_err, avg_yaw_err);
  }

  if (!enable_high_precision_ && yaw_ok) {
    if (!stopped) {
      log_stopped_unstable();
      return false;
    }
    publishGoalError(
      query_pose, goal_pose, goal_yaw, current_yaw, lon_err, lat_err, dyaw,
      std::hypot(lon_err, lat_err), yaw_ok,
      in_desired_window, in_accept_window, stable_desired, "accept_direct");
    RCLCPP_INFO(
      rclcpp::get_logger("PrecisionLongitudinalGoalChecker"),
      "接受精度到位成功：高精度模式已关闭，当前误差满足接受精度，"
      "lon_err=%.4f, lat_err=%.4f, yaw_err=%.4f, accept=(%.4f, %.4f)",
      lon_err, lat_err, dyaw,
      accept_lon_goal_tolerance_, accept_lat_goal_tolerance_);
    return true;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("PrecisionLongitudinalGoalChecker"),
    "lon_err=%.4f, lat_err=%.4f, yaw_err=%.4f, refine_count=%d/%d, "
    "current_desired=%s, yaw_ok=%s, avg_stable=%s, "
    "avg_abs=(%.4f, %.4f, %.4f), window=%d/%d, restart_used=(accept=%s, desired=%s)",
    lon_err, lat_err, dyaw, displayed_refine_count, refine_cycles_,
    in_desired_window ? "true" : "false",
    yaw_ok ? "true" : "false",
    stable_desired ? "true" : "false",
    avg_lon_err, avg_lat_err, avg_yaw_err,
    stable_window_count, stable_window_size_,
    accept_refine_restart_used_ ? "true" : "false",
    desired_refine_restart_used_ ? "true" : "false");

  if (yaw_ok && !accept_refine_restart_used_ && refine_count_ > 0) {
    accept_refine_restart_used_ = true;
    refine_count_ = 0;
    lon_errors_.clear();
    lat_errors_.clear();
    yaw_errors_.clear();
    last_status_ = PrecisionStatus::OutsideAccept;
    RCLCPP_INFO(
      rclcpp::get_logger("PrecisionLongitudinalGoalChecker"),
      "位置误差和角度误差均已满足接受精度，重新从 0/%d 开始最终稳定计数；"
      "同时清空近 %d 次平均误差窗口，该重置每次目标只允许触发一次。"
      "lon_err=%.4f, lat_err=%.4f, yaw_err=%.4f",
      refine_cycles_, stable_window_size_, lon_err, lat_err, dyaw);
    return false;
  }

  if (in_desired_window && yaw_ok && !desired_refine_restart_used_ && refine_count_ > 0) {
    desired_refine_restart_used_ = true;
    refine_count_ = 0;
    lon_errors_.clear();
    lat_errors_.clear();
    yaw_errors_.clear();
    last_status_ = PrecisionStatus::OutsideAccept;
    RCLCPP_INFO(
      rclcpp::get_logger("PrecisionLongitudinalGoalChecker"),
      "位置误差首次进入期望精度且角度满足要求，重新从 0/%d 开始高精度稳定计数；"
      "同时清空近 %d 次平均误差窗口，该 1cm 重置每次目标只允许触发一次。"
      "lon_err=%.4f, lat_err=%.4f, yaw_err=%.4f",
      refine_cycles_, stable_window_size_, lon_err, lat_err, dyaw);
    return false;
  }

  if (in_desired_window && yaw_ok && stable_desired) {
    if (!stopped) {
      log_stopped_unstable();
      return false;
    }
    publishGoalError(
      query_pose, goal_pose, goal_yaw, current_yaw, lon_err, lat_err, dyaw,
      std::hypot(lon_err, lat_err), yaw_ok,
      in_desired_window, in_accept_window, stable_desired, "desired_stable");
    RCLCPP_INFO(
      rclcpp::get_logger("PrecisionLongitudinalGoalChecker"),
      "高精度到位成功：本次误差和近 %d 次平均误差均满足期望精度，lon_err=%.4f, "
      "lat_err=%.4f, yaw_err=%.4f, avg_abs=(%.4f, %.4f, %.4f), "
      "window=%d/%d, refine_count=%d",
      stable_window_size_, lon_err, lat_err, dyaw,
      avg_lon_err, avg_lat_err, avg_yaw_err,
      stable_window_count, stable_window_size_, displayed_refine_count);
    return true;
  }

  // 进入接受区后，继续给 controller 若干周期尝试收敛到期望精度。
  ++refine_count_;

  if (refine_count_ >= refine_cycles_) {
    if (!yaw_ok) {
      const int completed_refine_count = std::min(refine_count_, refine_cycles_);
      const int yaw_wait_count = std::max(0, refine_count_ - refine_cycles_);
      RCLCPP_INFO(
        rclcpp::get_logger("PrecisionLongitudinalGoalChecker"),
        "位置误差已在接受区内，基础精调已满 %d/%d 个周期，角度额外等待 %d 个周期；"
        "角度误差尚未满足 %.4f rad，继续等待角度收敛："
        "lon_err=%.4f, lat_err=%.4f, yaw_err=%.4f",
        completed_refine_count, refine_cycles_, yaw_wait_count,
        yaw_goal_tolerance_, lon_err, lat_err, dyaw);
      return false;
    }

    if (!stopped) {
      log_stopped_unstable();
      return false;
    }

    publishGoalError(
      query_pose, goal_pose, goal_yaw, current_yaw, lon_err, lat_err, dyaw,
      std::hypot(lon_err, lat_err), yaw_ok,
      in_desired_window, in_accept_window, stable_desired, "accept_fallback");
    RCLCPP_INFO(
      rclcpp::get_logger("PrecisionLongitudinalGoalChecker"),
      "退让接受成功：基础精调 %d/%d 个周期后当前误差仍在接受区内，"
      "lon_err=%.4f, lat_err=%.4f, yaw_err=%.4f, "
      "current_desired=%s, avg_stable=%s, avg_abs=(%.4f, %.4f, %.4f), "
      "window=%d/%d, accept=(%.4f, %.4f)",
      std::min(refine_count_, refine_cycles_), refine_cycles_,
      lon_err, lat_err, dyaw,
      in_desired_window ? "true" : "false",
      stable_desired ? "true" : "false",
      avg_lon_err, avg_lat_err, avg_yaw_err,
      stable_window_count, stable_window_size_,
      accept_lon_goal_tolerance_, accept_lat_goal_tolerance_);
    return true;
  }

  return false;
}

bool PrecisionLongitudinalGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  const double invalid_field = std::numeric_limits<double>::lowest();

  pose_tolerance.position.x = accept_lon_goal_tolerance_;
  pose_tolerance.position.y = accept_lat_goal_tolerance_;
  pose_tolerance.position.z = invalid_field;
  pose_tolerance.orientation =
    nav2_util::geometry_utils::orientationAroundZAxis(yaw_goal_tolerance_);

  vel_tolerance.linear.x = check_stopped_ ? trans_stopped_velocity_ : invalid_field;
  vel_tolerance.linear.y = check_stopped_ ? trans_stopped_velocity_ : invalid_field;
  vel_tolerance.linear.z = invalid_field;

  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = check_stopped_ ? rot_stopped_velocity_ : invalid_field;

  return true;
}

void PrecisionLongitudinalGoalChecker::resetRefineState()
{
  last_status_ = PrecisionStatus::OutsideAccept;
  refine_count_ = 0;
  lon_errors_.clear();
  lat_errors_.clear();
  yaw_errors_.clear();
}

void PrecisionLongitudinalGoalChecker::recordAbsErrors(
  double abs_lon_err, double abs_lat_err, double abs_yaw_err)
{
  lon_errors_.push_back(abs_lon_err);
  lat_errors_.push_back(abs_lat_err);
  yaw_errors_.push_back(abs_yaw_err);

  while (static_cast<int>(lon_errors_.size()) > stable_window_size_) {
    lon_errors_.pop_front();
  }
  while (static_cast<int>(lat_errors_.size()) > stable_window_size_) {
    lat_errors_.pop_front();
  }
  while (static_cast<int>(yaw_errors_.size()) > stable_window_size_) {
    yaw_errors_.pop_front();
  }
}

double PrecisionLongitudinalGoalChecker::meanAbs(const std::deque<double> & data) const
{
  double sum = 0.0;
  for (const auto value : data) {
    sum += value;
  }
  return sum / static_cast<double>(data.size());
}

void PrecisionLongitudinalGoalChecker::publishGoalError(
  const geometry_msgs::msg::Pose & query_pose,
  const geometry_msgs::msg::Pose & goal_pose,
  double goal_yaw,
  double current_yaw,
  double lon_err,
  double lat_err,
  double yaw_err,
  double distance_err,
  bool yaw_within_tolerance,
  bool in_desired_window,
  bool in_accept_window,
  bool stable_desired,
  const std::string & result_type)
{
  if (!goal_error_pub_ || !goal_error_pub_->is_activated()) {
    return;
  }

  diagnostic_msgs::msg::DiagnosticArray msg;
  msg.header.stamp = clock_->now();
  msg.header.frame_id = "map";

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.name = "navigation_goal_error";
  status.message = result_type;
  status.hardware_id = "nav2_controller";

  auto add_value = [&status](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      item.value = value;
      status.values.push_back(item);
    };
  auto add_double = [&add_value](const std::string & key, double value) {
      add_value(key, std::to_string(value));
    };
  auto add_bool = [&add_value](const std::string & key, bool value) {
      add_value(key, value ? "true" : "false");
    };

  add_value("result_type", result_type);
  add_double("goal_x", goal_pose.position.x);
  add_double("goal_y", goal_pose.position.y);
  add_double("goal_yaw", goal_yaw);
  add_double("current_x", query_pose.position.x);
  add_double("current_y", query_pose.position.y);
  add_double("current_yaw", current_yaw);
  add_double("longitudinal_error", lon_err);
  add_double("lateral_error", lat_err);
  add_double("distance_error", distance_err);
  add_double("yaw_error", yaw_err);
  add_double("desired_longitudinal_tolerance", desired_lon_goal_tolerance_);
  add_double("desired_lateral_tolerance", desired_lat_goal_tolerance_);
  add_double("accept_longitudinal_tolerance", accept_lon_goal_tolerance_);
  add_double("accept_lateral_tolerance", accept_lat_goal_tolerance_);
  add_double("yaw_tolerance", yaw_goal_tolerance_);
  add_bool("high_precision_enabled", enable_high_precision_);
  add_bool("within_desired_tolerance", in_desired_window && yaw_within_tolerance);
  add_bool("within_accept_tolerance", in_accept_window && yaw_within_tolerance);
  add_bool("desired_window_stable", stable_desired);

  msg.status.push_back(status);
  goal_error_pub_->publish(msg);
}

void PrecisionLongitudinalGoalChecker::logStatusTransition(
  PrecisionStatus status,
  double lon_err,
  double lat_err,
  double yaw_err,
  bool stable_desired,
  int stable_window_count,
  double avg_lon_err,
  double avg_lat_err,
  double avg_yaw_err)
{
  if (status == last_status_) {
    return;
  }

  last_status_ = status;
  const auto logger = rclcpp::get_logger("PrecisionLongitudinalGoalChecker");

  switch (status) {
    case PrecisionStatus::AcceptOnly:
      RCLCPP_INFO(
        logger,
        "当前已满足接受精度，但尚未满足期望精度或角度要求，继续精调："
        "lon_err=%.4f, lat_err=%.4f, yaw_err=%.4f, "
        "avg_abs=(%.4f, %.4f, %.4f), window=%d/%d",
        lon_err, lat_err, yaw_err,
        avg_lon_err, avg_lat_err, avg_yaw_err,
        stable_window_count, stable_window_size_);
      break;
    case PrecisionStatus::DesiredUnstable:
      RCLCPP_INFO(
        logger,
        "本次误差已满足期望精度，等待近 %d 次平均误差稳定："
        "lon_err=%.4f, lat_err=%.4f, yaw_err=%.4f, "
        "avg_abs=(%.4f, %.4f, %.4f), window=%d/%d, stable=%s, "
        "restart_used=(accept=%s, desired=%s)",
        stable_window_size_, lon_err, lat_err, yaw_err,
        avg_lon_err, avg_lat_err, avg_yaw_err,
        stable_window_count, stable_window_size_, stable_desired ? "true" : "false",
        accept_refine_restart_used_ ? "true" : "false",
        desired_refine_restart_used_ ? "true" : "false");
      break;
    case PrecisionStatus::DesiredStable:
      break;
    case PrecisionStatus::YawUnstable:
      RCLCPP_INFO(
        logger,
        "位置误差已进入期望范围，但角度误差尚未满足要求，继续调整朝向："
        "lon_err=%.4f, lat_err=%.4f, yaw_err=%.4f, "
        "avg_abs=(%.4f, %.4f, %.4f), window=%d/%d, "
        "restart_used=(accept=%s, desired=%s)",
        lon_err, lat_err, yaw_err,
        avg_lon_err, avg_lat_err, avg_yaw_err,
        stable_window_count, stable_window_size_,
        accept_refine_restart_used_ ? "true" : "false",
        desired_refine_restart_used_ ? "true" : "false");
      break;
    case PrecisionStatus::StoppedUnstable:
      RCLCPP_INFO(
        logger,
        "位置和角度已满足要求，但速度尚未满足停止判定，等待底盘停稳："
        "lon_err=%.4f, lat_err=%.4f, yaw_err=%.4f",
        lon_err, lat_err, yaw_err);
      break;
    case PrecisionStatus::OutsideAccept:
      break;
  }
}

rcl_interfaces::msg::SetParametersResult
PrecisionLongitudinalGoalChecker::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".desired_lon_goal_tolerance") {
        desired_lon_goal_tolerance_ = parameter.as_double();
      } else if (name == plugin_name_ + ".desired_lat_goal_tolerance") {
        desired_lat_goal_tolerance_ = parameter.as_double();
      } else if (name == plugin_name_ + ".accept_lon_goal_tolerance") {
        accept_lon_goal_tolerance_ = parameter.as_double();
      } else if (name == plugin_name_ + ".accept_lat_goal_tolerance") {
        accept_lat_goal_tolerance_ = parameter.as_double();
      } else if (name == plugin_name_ + ".yaw_goal_tolerance") {
        yaw_goal_tolerance_ = parameter.as_double();
      } else if (name == plugin_name_ + ".trans_stopped_velocity") {
        trans_stopped_velocity_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rot_stopped_velocity") {
        rot_stopped_velocity_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == plugin_name_ + ".refine_cycles") {
        refine_cycles_ = static_cast<int>(std::max<int64_t>(0, parameter.as_int()));
        resetRefineState();
      } else if (name == plugin_name_ + ".stable_window_size") {
        stable_window_size_ = static_cast<int>(std::max<int64_t>(1, parameter.as_int()));
        resetRefineState();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".enable_high_precision") {
        enable_high_precision_ = parameter.as_bool();
        resetRefineState();
      } else if (name == plugin_name_ + ".check_stopped") {
        check_stopped_ = parameter.as_bool();
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace nav2_controller

PLUGINLIB_EXPORT_CLASS(nav2_controller::PrecisionLongitudinalGoalChecker, nav2_core::GoalChecker)
