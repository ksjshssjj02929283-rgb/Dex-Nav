// Copyright 2026 DEXE
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "nav2_controller/controller_server.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <condition_variable>
namespace nav2_controller {
using SnapshotClock = std::chrono::steady_clock;
struct ControllerServer::SnapshotRuntime
{
  struct Request {
    uint64_t session{}, revision{};
    nav_msgs::msg::Path path;
    geometry_msgs::msg::PoseStamped pose;
    geometry_msgs::msg::Twist velocity;
    nav2_msgs::msg::SpeedLimit limit;
    bool has_limit{};
    double acceleration{};
    bool acceleration_valid{};
  };
  struct Result {
    uint64_t session{}, revision{}, sequence{};
    nav_interfaces::msg::RobotTrajectory trajectory;
    nav2_core::SpeedPlanningDiagnostics diagnostics;
    SnapshotClock::time_point completed;
    bool success{};
  };
  struct Sensor {
    SnapshotClock::time_point received{};
    int64_t stamp{};
    bool valid{};
  };
  std::mutex mutex;
  std::condition_variable wake;
  std::thread worker;
  bool quit{false};
  uint64_t session{0}, revision{0}, limit_revision{0}, safety_revision{0};
  nav2_msgs::msg::SpeedLimit limit;
  bool has_limit{false};
  double feedback_acceleration{0};
  bool feedback_valid{false};
  SnapshotClock::time_point feedback_time{}, accepted_completed{};
  uint64_t accepted_revision{0}, accepted_session{0}, accepted_safety{0}, accepted_limit{0};
  double planning_hz{10.0}, max_age{0.3}, sensor_timeout{0.5};
  std::shared_ptr<const Request> request;
  std::shared_ptr<const Result> result;
  std::vector<Sensor> sensors;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> subscriptions;
  rclcpp_lifecycle::LifecyclePublisher<nav_interfaces::msg::RobotTrajectory>::SharedPtr publisher;
};

}
