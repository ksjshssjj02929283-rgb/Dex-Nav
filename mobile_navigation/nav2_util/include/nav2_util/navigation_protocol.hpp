// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0

#ifndef NAV2_UTIL__NAVIGATION_PROTOCOL_HPP_
#define NAV2_UTIL__NAVIGATION_PROTOCOL_HPP_

#include "rclcpp/qos.hpp"

namespace nav2_util
{
namespace navigation_protocol
{

// 使用相对名称跟随 Nav2 namespace，避免多机器人之间共享状态。
inline constexpr char kPrepareSessionTopic[] = "controller_server/prepare_replan/session";
inline constexpr char kPrepareReplanTopic[] = "controller_server/prepare_replan/request";

// Session 和重规划信号都是当前导航任务状态，晚加入者需要获得最新值。
inline rclcpp::QoS stateTopicQoS()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

}  // namespace navigation_protocol
}  // namespace nav2_util

#endif  // NAV2_UTIL__NAVIGATION_PROTOCOL_HPP_
