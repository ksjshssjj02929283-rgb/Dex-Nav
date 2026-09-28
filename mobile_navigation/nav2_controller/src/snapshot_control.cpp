// Copyright 2026 DEXE
// SPDX-License-Identifier: Apache-2.0
#include "nav2_controller/controller_server.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav2_controller/snapshot_validation.hpp"
#include "nav2_controller/snapshot_runtime.hpp"
#include <chrono>
#include <condition_variable>
#include <functional>
#include <cmath>

namespace nav2_controller
{
using SnapshotClock = std::chrono::steady_clock;

void ControllerServer::configureSnapshotControl()
{
  snapshot_runtime_ = std::make_shared<SnapshotRuntime>();
  auto r = snapshot_runtime_;
  const auto param = [this](const std::string & name, double value) {
      nav2_util::declare_parameter_if_not_declared(shared_from_this(), name, rclcpp::ParameterValue(value));
      const double configured = get_parameter(name).as_double();
      if (!std::isfinite(configured) || configured <= 0) {throw std::invalid_argument(name);}
      return configured;
    };
  r->planning_hz = param("planning_frequency", 10.0);
  r->max_age = param("planning_result_max_age", 0.3);
  r->sensor_timeout = param("sensor_timeout", 0.5);
  nav2_util::declare_parameter_if_not_declared(shared_from_this(), "required_scan_topics",
    rclcpp::ParameterValue(std::vector<std::string>{"/laser_scan/front", "/laser_scan/rear",
      "/orbbec/front/laser_scan", "/orbbec/rear/laser_scan"}));
  const auto topics = get_parameter("required_scan_topics").as_string_array();
  if (topics.empty()) {throw std::invalid_argument("required_scan_topics must not be empty");}
  r->sensors.resize(topics.size());
  for (size_t i = 0; i < topics.size(); ++i) {
    r->subscriptions.push_back(create_subscription<sensor_msgs::msg::LaserScan>(topics[i],
      rclcpp::SensorDataQoS(), [this, weak = std::weak_ptr<SnapshotRuntime>(r), i](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {
        const auto r = weak.lock();
        if (!r) {return;}
        const auto stamp = rclcpp::Time(m->header.stamp).nanoseconds();
        const double age = (now().nanoseconds() - stamp) * 1e-9;
        const bool transform_valid = costmap_ros_->getTfBuffer() &&
          costmap_ros_->getTfBuffer()->canTransform(costmap_ros_->getBaseFrameID(),
            m->header.frame_id, tf2::TimePointZero);
        const bool content_valid = snapshot::validScan(m->ranges, m->range_min, m->range_max,
          m->angle_increment, transform_valid && !m->header.frame_id.empty(), stamp, 0,
          age, r->sensor_timeout);
        std::lock_guard<std::mutex> lock(r->mutex);
        auto & sensor = r->sensors[i];
        const bool valid = content_valid && stamp > sensor.stamp;
        if (!valid) {++r->safety_revision; r->result.reset();}
        sensor = {SnapshotClock::now(), stamp, valid};
      }));
  }
  r->publisher = create_publisher<nav_interfaces::msg::RobotTrajectory>("local_trajectory", 1);
}

bool ControllerServer::snapshotSensorsHealthy()
{
  auto r = snapshot_runtime_;
  if (!r) {return false;}
  std::lock_guard<std::mutex> lock(r->mutex);
  for (const auto & s : r->sensors) {
    if (!s.valid || std::chrono::duration<double>(SnapshotClock::now() - s.received).count() >
      r->sensor_timeout || (now().nanoseconds() - s.stamp) * 1e-9 > r->sensor_timeout)
    {return false;}
  }
  return true;
}

void ControllerServer::startSnapshotWorker()
{
  auto r = snapshot_runtime_;
  r->quit = false;
  r->publisher->on_activate();
  r->worker = std::thread([this, r]() {
    uint64_t last_session = 0, last_revision = 0, sequence = 0;
    bool terminal_latched = false;
    auto next = SnapshotClock::now();
    while (true) {
      std::shared_ptr<const SnapshotRuntime::Request> request;
      {
        std::unique_lock<std::mutex> lock(r->mutex);
        r->wake.wait(lock, [&]() {return r->quit || bool(r->request);});
        if (r->quit) {return;}
        r->wake.wait_until(lock, next, [&]() {
          return r->quit || !r->request || r->request->session != last_session ||
            r->request->revision != last_revision;
        });
        if (r->quit) {return;}
        request = r->request;
      }
      if (!request) {continue;}
      const auto planning_started = SnapshotClock::now();
      auto result = std::make_shared<SnapshotRuntime::Result>();
      result->session = request->session; result->revision = request->revision;
      result->sequence = ++sequence;
      const bool new_session = request->session != last_session;
      const bool new_path = new_session || request->revision != last_revision;
      last_session = request->session; last_revision = request->revision;
      try {
        if (new_session) {terminal_latched = false;}
        if (new_path) {
          for (auto & p : local_planners_) {p.second->setPlan(request->path);}
        }
        if (request->has_limit) {
          for (auto & p : local_planners_) {p.second->setSpeedLimit(request->limit.speed_limit, request->limit.percentage);}
          for (auto & p : speed_planners_) {p.second->setSpeedLimit(request->limit.speed_limit, request->limit.percentage);}
        }
        nav_msgs::msg::Path local = request->path;
        if (!current_local_planner_.empty() && !local_planners_.at(current_local_planner_)->computeLocalPlan(
            request->pose, request->velocity, local)) {throw std::runtime_error("local planning failed");}
        if (local.poses.empty() || current_speed_planner_.empty()) {
          throw std::runtime_error("a speed planner and nonempty local path are required");
        }
        nav2_core::InitialMotionState boundary;
        boundary.velocity = request->velocity;
        boundary.acceleration = request->acceleration;
        boundary.acceleration_valid = request->acceleration_valid;
        boundary.source_stamp = request->pose.header.stamp;
        boundary.terminal_brake_latched = terminal_latched;
        auto planner = speed_planners_.at(current_speed_planner_);
        planner->setPlan(local);
        result->trajectory = planner->computeSpeedPlan(local, boundary);
        result->trajectory.plan_id = std::to_string(request->session) + ":" + std::to_string(request->revision);
        result->diagnostics = planner->getLastPlanningDiagnostics();
        terminal_latched |= result->diagnostics.terminal_brake_required;
        result->success = !result->trajectory.points.empty() &&
          !result->trajectory.header.frame_id.empty() && !result->diagnostics.terminal_infeasible;
        double previous_time = -1;
        for (const auto & p : result->trajectory.points) {
          const double t = p.relative_time.sec + p.relative_time.nanosec * 1e-9;
          result->success &= std::isfinite(p.pose.pose.position.x) &&
            std::isfinite(p.pose.pose.position.y) && std::isfinite(p.longitudinal_velocity) &&
            std::isfinite(p.angular_velocity) && std::isfinite(p.acceleration) && t >= previous_time;
          const auto & q = p.pose.pose.orientation;
          const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
          result->success &= std::isfinite(norm) && std::fabs(norm - 1.0) < 0.01 &&
            std::isfinite(p.pose.pose.position.z) && std::isfinite(p.path_kappa) &&
            std::isfinite(p.jerk) && std::isfinite(p.speed_hint) &&
            (p.pose.header.frame_id.empty() || p.pose.header.frame_id == result->trajectory.header.frame_id);
          previous_time = t;
        }
      } catch (const std::exception & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Planning failed: %s", e.what());
        result->success = false;
      } catch (...) {
        result->success = false;
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Planning failed with unknown exception");
      }
      result->completed = SnapshotClock::now();
      {
        std::lock_guard<std::mutex> lock(r->mutex);
        if (r->request && r->request->session == result->session &&
          r->request->revision == result->revision) {r->result = result;}
      }
      next = planning_started + std::chrono::duration_cast<SnapshotClock::duration>(
        std::chrono::duration<double>(1.0 / r->planning_hz));
    }
  });
}

void ControllerServer::stopSnapshotWorker()
{
  auto r = snapshot_runtime_;
  if (!r) {return;}
  {
    std::lock_guard<std::mutex> lock(r->mutex);
    r->quit = true; r->request.reset(); r->result.reset();
  }
  r->wake.notify_all();
  if (r->worker.joinable()) {r->worker.join();}
  if (r->publisher->is_activated()) {r->publisher->on_deactivate();}
}

void ControllerServer::queueSnapshotSpeedLimit(const nav2_msgs::msg::SpeedLimit & limit)
{
  auto r = snapshot_runtime_;
  if (!r || !std::isfinite(limit.speed_limit) || limit.speed_limit < 0) {return;}
  std::lock_guard<std::mutex> lock(r->mutex);
  if (!r->has_limit || r->limit.speed_limit != limit.speed_limit || r->limit.percentage != limit.percentage) {
    r->limit = limit; r->has_limit = true; ++r->limit_revision; r->result.reset();
  }
}

bool ControllerServer::computeControlSnapshot()
{
  auto r = snapshot_runtime_;
  if (!r || !r->worker.joinable()) {throw std::runtime_error("Planning worker is not active");}
  {
    std::lock_guard<std::mutex> lock(r->mutex);
    ++r->session; ++r->revision; r->result.reset(); r->feedback_valid = false;
  }
  // Action termination never joins the planner. Late results cannot cross sessions.
  auto finish = std::shared_ptr<void>(nullptr, [r](void *) {
    std::lock_guard<std::mutex> lock(r->mutex);
    ++r->session; r->request.reset(); r->result.reset(); r->wake.notify_all();
  });
  uint64_t path_id = 0, limit_id = 0, safety_id = 0, accepted = 0;
  uint64_t observed_goal = goal_generation_;
  rclcpp::WallRate rate(controller_frequency_);
  while (rclcpp::ok()) {
    if (!action_server_->is_server_active() || action_server_->is_cancel_requested()) {
      publishHardStopVelocity();
      lockPrepareForCurrentSession("goal canceled or server inactive");
      resetEndLocalizationCapabilities(goal_generation_);
      publishNavigationStatus(nav_interfaces::msg::NavigationStatus::IDLE);
      action_server_->terminate_all(); return false;
    }
    updateGlobalPath(); syncPrepareSession();
    if (!costmap_ros_->isCurrent() || !snapshotSensorsHealthy()) {
      {
        std::lock_guard<std::mutex> lock(r->mutex);
        ++r->revision; r->request.reset(); r->result.reset();
      }
      publishHardStopVelocity();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Motion inhibited: stale/invalid lidar, RGBD or costmap");
      rate.sleep(); continue;
    }
    auto request = std::make_shared<SnapshotRuntime::Request>();
    request->path = current_path_;
    if (!nav2_util::getCurrentPose(request->pose, *costmap_ros_->getTfBuffer(),
        current_path_.header.frame_id, costmap_ros_->getBaseFrameID(), costmap_ros_->getTransformTolerance())) {
      {std::lock_guard<std::mutex> lock(r->mutex); ++r->revision; r->request.reset(); r->result.reset();}
      publishHardStopVelocity(); rate.sleep(); continue;
    }
    const auto twist = getThresholdedTwist(odom_sub_->getTwist());
    request->velocity.linear.x = twist.x; request->velocity.linear.y = twist.y;
    request->velocity.angular.z = twist.theta;
    std::shared_ptr<const SnapshotRuntime::Result> result;
    bool limit_changed = false;
    {
      std::lock_guard<std::mutex> lock(r->mutex);
      if (observed_goal != goal_generation_) {
        ++r->session; ++r->revision; r->result.reset(); r->feedback_valid = false;
        observed_goal = goal_generation_;
      }
      limit_changed = limit_id != r->limit_revision;
      if (path_id != speed_plan_path_id_ || limit_changed || safety_id != r->safety_revision) {
        ++r->revision; r->result.reset();
        path_id = speed_plan_path_id_; limit_id = r->limit_revision; safety_id = r->safety_revision;
      }
      request->session = r->session; request->revision = r->revision;
      request->limit = r->limit; request->has_limit = r->has_limit;
      request->acceleration = r->feedback_acceleration;
      request->acceleration_valid = r->feedback_valid &&
        std::chrono::duration<double>(SnapshotClock::now() - r->feedback_time).count() <= r->max_age;
      r->request = request; result = r->result;
    }
    r->wake.notify_one();
    if (limit_changed) {
      for (auto & c : controllers_) {c.second->setSpeedLimit(request->limit.speed_limit, request->limit.percentage);}
    }
    if (!result || !snapshot::usableResult(result->session, result->revision,
        request->session, request->revision, result->success,
        std::chrono::duration<double>(SnapshotClock::now() - result->completed).count(), r->max_age))
    {publishHardStopVelocity(); rate.sleep(); continue;}
    if (accepted != result->sequence) {
      current_trajectory_ = result->trajectory;
      trajectory_dispatched_to_.clear();
      speed_plan_cycle_ = SpeedPlanCycleContext();
      speed_plan_cycle_.terminal = result->diagnostics;
      r->publisher->publish(current_trajectory_);
      publishSpeedPlan(current_trajectory_);
      accepted = result->sequence;
      std::lock_guard<std::mutex> lock(r->mutex);
      r->accepted_revision = request->revision; r->accepted_session = request->session;
      r->accepted_safety = safety_id; r->accepted_limit = limit_id;
      r->accepted_completed = result->completed;
    }
    speed_plan_cycle_.active = true;
    speed_plan_cycle_.planning_succeeded = true;
    speed_plan_cycle_.terminal_valid = true;
    computeAndPublishVelocity();
    if (isGoalReached()) {resetStagedControlState(); publishHardStopVelocity(); return true;}
    rate.sleep();
  }
  publishHardStopVelocity(); return false;
}
}  // namespace nav2_controller

namespace nav2_controller
{
bool ControllerServer::snapshotMotionAllowed()
{
  if (!snapshotSensorsHealthy() || !costmap_ros_->isCurrent()) {return false;}
  auto r = snapshot_runtime_;
  std::lock_guard<std::mutex> lock(r->mutex);
  return r->request && r->result && r->result->success &&
    r->accepted_revision == r->revision && r->accepted_session == r->session &&
    r->accepted_safety == r->safety_revision && r->accepted_limit == r->limit_revision &&
    std::chrono::duration<double>(SnapshotClock::now() - r->accepted_completed).count() <= r->max_age;
}
void ControllerServer::snapshotControlFeedback(bool valid, double acceleration)
{
  auto r = snapshot_runtime_;
  if (!r) {return;}
  std::lock_guard<std::mutex> lock(r->mutex);
  r->feedback_valid = valid && std::isfinite(acceleration);
  r->feedback_acceleration = acceleration;
  r->feedback_time = SnapshotClock::now();
}
}
