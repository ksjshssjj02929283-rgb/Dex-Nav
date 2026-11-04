// Copyright 2026 DEXE
// SPDX-License-Identifier: Apache-2.0
#include "nav2_controller/controller_server.hpp"
#include "nav2_controller/snapshot_validation.hpp"
#include "nav2_controller/snapshot_runtime.hpp"
#include "nav2_core/exceptions.hpp"
#include <chrono>
#include <condition_variable>
#include <functional>
#include <cmath>
#include <limits>

namespace nav2_controller
{
using SnapshotClock = std::chrono::steady_clock;

/**
 * @brief 配置异步快照控制运行时参数与发布器。
 * 
 * 核心机制：
 * 1. planning_frequency (默认 10.0 Hz)：独立规划线程 LocalPlanner + SpeedPlanner 执行频率；
 * 2. planning_result_max_age (默认 0.3 s)：规划结果最大允许陈旧时间，超出后控制线程强制急停；
 * 3. local_trajectory 发布器：发布 SpeedPlanner 规划产出的带时空速度剖面的 RobotTrajectory。
 */
void ControllerServer::configureSnapshotControl()
{
  snapshot_runtime_ = std::make_shared<SnapshotRuntime>();
  auto r = snapshot_runtime_;
  const auto param = [this](const std::string & name, double value) {
      nav2_util::declare_parameter_if_not_declared(
        shared_from_this(), name,
        rclcpp::ParameterValue(value));
      const double configured = get_parameter(name).as_double();
      if (!std::isfinite(configured) || configured <= 0) {throw std::invalid_argument(name);}
      return configured;
    };
  r->planning_hz = param("planning_frequency", 25.0);
  r->max_age = param("planning_result_max_age", 0.5);
  r->local_control_stall_timeout = param("local_control_stall_timeout", 30.0);
  r->stall_linear_velocity_epsilon = param("stall_linear_velocity_epsilon", 0.01);
  r->stall_angular_velocity_epsilon = param("stall_angular_velocity_epsilon", 0.01);
  r->publisher = create_publisher<nav_interfaces::msg::RobotTrajectory>("local_trajectory", 1);
}

/**
 * @brief 启动持久化规划工作线程（Planning Worker）。
 * 
 * 规划与控制解耦核心：
 * 1. 规划线程常驻运行，通过条件变量 wake 与控制线程（25 Hz）异步解耦；
 * 2. 只有在有新请求且满足规划周期 interval (1/planning_hz) 时执行规划；
 * 3. 规划过程包含 LocalPlanner（几何剪裁与局部路径生成）与 SpeedPlanner（时空速度规划）；
 * 4. 产出 RobotTrajectory 并更新最新 Result，供控制线程以 25 Hz 高频无锁/极短锁快照读取。
 */
void ControllerServer::startSnapshotWorker()
{
  auto r = snapshot_runtime_;
  r->quit = false;
  r->publisher->on_activate();
  r->worker = std::thread(
    [this, r]() {
      uint64_t last_session = 0, last_revision = 0, sequence = 0;
      bool terminal_latched = false;
      auto next = SnapshotClock::now();
      while (true) {
        SnapshotRuntime::Request request;
        {
          std::unique_lock<std::mutex> lock(r->mutex);
          // 等待新请求或退出通知
          r->wake.wait(lock, [&]() {return r->quit || r->has_request;});
          if (r->quit) {return;}
          // 按 planning_hz 频率定时节流，若有新任务 session 或新路径 revision 则立即打断等待
          r->wake.wait_until(
            lock, next, [&]() {
              return r->quit || !r->has_request || r->request.session != last_session ||
              r->request.revision != last_revision;
            });
          if (r->quit) {return;}
          request = r->request;
        }
        if (!request.path) {continue;}
        const auto planning_started = SnapshotClock::now();
        auto result = std::make_shared<SnapshotRuntime::Result>();
        result->session = request.session; result->revision = request.revision;
        result->sequence = ++sequence;
        const bool new_session = request.session != last_session;
        const bool new_path = new_session || request.revision != last_revision;
        last_session = request.session; last_revision = request.revision;
        try {
          if (new_session) {terminal_latched = false;}
          // 1. 全局路径更新时下发给局部几何规划器
          if (new_path) {
            for (auto & p : local_planners_) {
              p.second->setPlan(*request.path);
            }
          }
          // 2. 限速参数分发
          if (request.has_limit) {
            for (auto & p : local_planners_) {
              p.second->setSpeedLimit(request.limit.speed_limit, request.limit.percentage);
            }
            for (auto & p : speed_planners_) {
              p.second->setSpeedLimit(request.limit.speed_limit, request.limit.percentage);
            }
          }
          // 3. 执行 LocalPlanner 几何规划（局部路径剪裁与避障搜索）
          nav_msgs::msg::Path local = *request.path;
          if (!current_local_planner_.empty() &&
          !local_planners_.at(current_local_planner_)->computeLocalPlan(
            request.pose, request.velocity, local))
          {
            throw std::runtime_error("local planning failed");
          }
          if (local.poses.empty() || current_speed_planner_.empty()) {
            throw std::runtime_error("a speed planner and nonempty local path are required");
          }

          // 4. 构建 SpeedPlanner 起点运动边界条件（InitialMotionState）
          nav2_core::InitialMotionState boundary;
          boundary.velocity = request.velocity;
          // 当控制侧连续跟踪正常且反馈新鲜时，若实测速度与上一周期规划目标未发生显著失配（<0.25m/s），
          // 纵向规划初值优先继承跟踪目标速度，消除里程计离散延迟和噪声引起的规划起点 Jerk 抖动；
          // 当出现打滑或外部强扰动失配时，自动回退到实测里程计速度。
          if (request.target_velocity_valid && std::isfinite(request.target_velocity)) {
            constexpr double kVelocityMismatchThreshold = 0.25;
            if (std::fabs(request.velocity.linear.x - request.target_velocity) < kVelocityMismatchThreshold) {
              boundary.velocity.linear.x = request.target_velocity;
            }
          }
          boundary.acceleration = request.acceleration;
          boundary.acceleration_valid = request.acceleration_valid;
          boundary.source_stamp = request.pose.header.stamp;
          boundary.terminal_brake_latched = terminal_latched;

          // 5. 执行 SpeedPlanner 速度剖面规划（S曲线/Ruckig/加减速生成）
          auto planner = speed_planners_.at(current_speed_planner_);
          planner->setPlan(local);
          result->trajectory = planner->computeSpeedPlan(local, boundary);
          result->trajectory.plan_id = std::to_string(request.session) + ":" +
          std::to_string(request.revision);
          result->diagnostics = planner->getLastPlanningDiagnostics();
          terminal_latched |= result->diagnostics.terminal_brake_required;

          // 6. 严苛校验输出轨迹：非空、坐标系合法、末端可行、时间单调递增及四元数归一化
          result->success = !result->trajectory.points.empty() &&
          !result->trajectory.header.frame_id.empty() && !result->diagnostics.terminal_infeasible;
          double previous_time = -1;
          for (const auto & p : result->trajectory.points) {
            const double t = p.relative_time.sec + p.relative_time.nanosec * 1e-9;
            result->success &= std::isfinite(p.pose.pose.position.x) &&
            std::isfinite(p.pose.pose.position.y) && std::isfinite(p.longitudinal_velocity) &&
            std::isfinite(p.angular_velocity) && std::isfinite(
              p.acceleration) && t >= previous_time;
            const auto & q = p.pose.pose.orientation;
            const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
            result->success &= std::isfinite(norm) && std::fabs(norm - 1.0) < 0.01 &&
            std::isfinite(p.pose.pose.position.z) && std::isfinite(p.path_kappa) &&
            std::isfinite(p.jerk) && std::isfinite(p.speed_hint) &&
            (p.pose.header.frame_id.empty() ||
            p.pose.header.frame_id == result->trajectory.header.frame_id);
            previous_time = t;
          }
        } catch (const std::exception & e) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Planning failed: %s", e.what());
          result->success = false;
        } catch (...) {
          result->success = false;
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Planning failed with unknown exception");
        }
        result->completed = SnapshotClock::now();
        {
          std::lock_guard<std::mutex> lock(r->mutex);
          if (r->has_request && r->request.session == result->session &&
          r->request.revision == result->revision) {r->result = result;}
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
    r->quit = true; r->has_request = false; r->result.reset();
  }
  r->wake.notify_all();
  // 协作式取消：向速度规划器发送取消请求以中断长耗时计算，加速工作线程退出
  for (auto & planner : speed_planners_) {
    planner.second->requestCancel();
  }
  if (r->worker.joinable()) {r->worker.join();}
  if (r->publisher->is_activated()) {r->publisher->on_deactivate();}
}

void ControllerServer::queueSnapshotSpeedLimit(const nav2_msgs::msg::SpeedLimit & limit)
{
  auto r = snapshot_runtime_;
  if (!r || !std::isfinite(limit.speed_limit) || limit.speed_limit < 0) {return;}
  std::lock_guard<std::mutex> lock(r->mutex);
  if (!r->has_limit || r->limit.speed_limit != limit.speed_limit ||
    r->limit.percentage != limit.percentage)
  {
    r->limit = limit; r->has_limit = true; ++r->limit_revision; r->result.reset();
  }
}

/**
 * @brief 执行高频快照控制主循环（25 Hz）。
 * 
 * 控制主循环职责：
 * 1. 25 Hz 固定频率驱动（controller_frequency_）；
 * 2. 检查代价地图时效性（costmap_ros_->isCurrent()）及机器人实时位姿 TF；
 * 3. 构造微型 Request（仅包含位姿、当前速度、限速与目标反馈），唤醒后台 10 Hz 规划线程；
 * 4. 快照式获取最新有效轨迹（通过 usableResult 校验版本与时效性）；
 * 5. 驱动下游 Controller 计算并下发 cmd_vel。
 */
bool ControllerServer::computeControlSnapshot()
{
  auto r = snapshot_runtime_;
  if (!r || !r->worker.joinable()) {throw std::runtime_error("Planning worker is not active");}
  {
    std::lock_guard<std::mutex> lock(r->mutex);
    ++r->session; ++r->revision; r->result.reset(); r->feedback_valid = false;
  }
  // 退出保护 RAII：Action 结束或异常退出时递增 session 并取消当前正在运行的规划
  auto finish = std::shared_ptr<void>(
    nullptr, [this, r](void *) {
      {
        std::lock_guard<std::mutex> lock(r->mutex);
        ++r->session; r->has_request = false; r->result.reset(); r->wake.notify_all();
      }
      for (auto & planner : speed_planners_) {
        planner.second->requestCancel();
      }
    });
  uint64_t path_id = std::numeric_limits<uint64_t>::max(), limit_id = 0, accepted = 0;
  std::shared_ptr<const nav_msgs::msg::Path> shared_path;
  uint64_t observed_goal = goal_generation_;
  rclcpp::WallRate rate(controller_frequency_);
  while (rclcpp::ok()) {
    // 1. 检查 Action 状态与取消请求
    if (!action_server_->is_server_active() || action_server_->is_cancel_requested()) {
      publishHardStopVelocity();
      lockPrepareForCurrentSession("goal canceled or server inactive");
      resetEndLocalizationCapabilities(goal_generation_);
      publishNavigationStatus(nav_interfaces::msg::NavigationStatus::IDLE);
      action_server_->terminate_all(); return false;
    }
    updateGlobalPath(); syncPrepareSession();

    // 2. 局部静止看门狗独立于 progress_checker：所有提前 continue 路径都会经过这里。
    //    progress_checker.movement_time_allowance 保持原有 5s 语义；连续静止的任务
    //    失败边界只由 local_control_stall_timeout（默认 30s）管理。
    const auto twist = getThresholdedTwist(odom_sub_->getTwist());
    const bool stationary =
      std::hypot(twist.x, twist.y) <= r->stall_linear_velocity_epsilon &&
      std::fabs(twist.theta) <= r->stall_angular_velocity_epsilon;
    const auto stall_now = SnapshotClock::now();
    if (!stationary) {
      if (r->local_stall_active) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Local control movement resumed; stall timeout cleared.");
        r->local_stall_active = false;
      }
    } else {
      if (!r->local_stall_active) {
        r->local_stall_active = true;
        r->local_stall_started = stall_now;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Local control stationary; task will fail after %.1f s without motion.",
          r->local_control_stall_timeout);
      }
      if (snapshot::stallTimedOut(
          r->local_stall_started, stall_now, r->local_control_stall_timeout))
      {
        throw nav2_core::PlannerException(
                "Local control remained stationary for " +
                std::to_string(r->local_control_stall_timeout) + " s.");
      }
    }

    // 3. 检查 Costmap 是否新鲜有效（过期时强制发硬刹停）
    if (!costmap_ros_ || !costmap_ros_->getLayeredCostmap() || !costmap_ros_->isCurrent()) {
      {
        std::lock_guard<std::mutex> lock(r->mutex);
        ++r->revision; r->has_request = false; r->result.reset();
      }
      publishHardStopVelocity();
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(), 2000, "Motion inhibited: costmap is not current");
      rate.sleep(); continue;
    }

    // 4. 获取机器人当前在全局路径坐标系下的位姿
    SnapshotRuntime::Request request;
    if (!nav2_util::getCurrentPose(
        request.pose, *costmap_ros_->getTfBuffer(),
        current_path_.header.frame_id, costmap_ros_->getBaseFrameID(),
        costmap_ros_->getTransformTolerance()))
    {
      {std::lock_guard<std::mutex> lock(r->mutex); ++r->revision; r->has_request = false;
        r->result.reset();}
      publishHardStopVelocity(); rate.sleep(); continue;
    }
    // 5. 获取当前底盘滤波里程计速度
    request.velocity.linear.x = twist.x; request.velocity.linear.y = twist.y;
    request.velocity.angular.z = twist.theta;

    std::shared_ptr<const SnapshotRuntime::Result> result;
    // 6. 路径几何变更时在锁外预构造不可变对象，避免持锁进行 Path 深拷贝与堆分配
    if (path_id != speed_plan_path_id_) {
      shared_path = std::make_shared<const nav_msgs::msg::Path>(current_path_);
    }
    bool limit_changed = false;
    {
      std::lock_guard<std::mutex> lock(r->mutex);
      if (observed_goal != goal_generation_) {
        ++r->session; ++r->revision; r->result.reset(); r->feedback_valid = false;
        observed_goal = goal_generation_;
      }
      limit_changed = limit_id != r->limit_revision;
      if (path_id != speed_plan_path_id_ || limit_changed) {
        ++r->revision;
        // 同一目标下的重规划/限速调整保留上一次的规划结果，避免在后台重规划耗时的极短窗口内（几十毫秒）
        // 瞬间触发 usableResult 校验失败导致机器人高速急刹顿挫；仅在真正换目标 (session 变更) 时才 reset。
        path_id = speed_plan_path_id_; limit_id = r->limit_revision;
      }
      request.path = shared_path;
      request.session = r->session; request.revision = r->revision;
      request.limit = r->limit; request.has_limit = r->has_limit;
      request.acceleration = r->feedback_acceleration;
      const bool feedback_fresh = r->feedback_valid &&
        std::chrono::duration<double>(SnapshotClock::now() - r->feedback_time).count() <=
        r->max_age;
      request.acceleration_valid = feedback_fresh;
      request.target_velocity = r->feedback_velocity;
      request.target_velocity_valid = feedback_fresh;
      r->request = request; r->has_request = true; result = r->result;
    }
    r->wake.notify_one();
    if (limit_changed) {
      for (auto & c : controllers_) {
        c.second->setSpeedLimit(request.limit.speed_limit, request.limit.percentage);
      }
    }
    // 7. 校验规划结果有效性与时效性（usableResult 检查会话匹配、版本一致、规划成功且未超出 max_age）
    if (!result || !snapshot::usableResult(*result, request.session, request.revision, r->max_age))
    {
      publishHardStopVelocity();
      rate.sleep();
      continue;
    }

    // 8. 若规划线程生成了新序号的轨迹，接受并更新当前轨迹与诊断信息，对外分发发布
    if (accepted != result->sequence) {
      current_trajectory_ = result->trajectory;
      trajectory_dispatched_to_.clear();
      speed_plan_cycle_ = SpeedPlanCycleContext();
      speed_plan_cycle_.terminal = result->diagnostics;
      r->publisher->publish(current_trajectory_);
      publishSpeedPlan(current_trajectory_);
      accepted = result->sequence;
      std::lock_guard<std::mutex> lock(r->mutex);
      r->accepted_revision = request.revision; r->accepted_session = request.session;
      r->accepted_limit = limit_id;
      r->accepted_completed = result->completed;
    }
    speed_plan_cycle_.active = true;
    speed_plan_cycle_.planning_succeeded = true;
    speed_plan_cycle_.terminal_valid = true;

    // 9. 驱动下游 Controller 计算速度指令并发布到 cmd_vel
    computeAndPublishVelocity();

    // 10. 到点判定：要求连续 goal_reached_required_frames_ 个控制拍 isGoalReached()
    //    均为真才确认到位，避免末端位姿噪声/瞬时掠过在容差边界上误判成功。
    //    任一拍未命中立即清零计数，重新开始累计。
    if (isGoalReached()) {
      ++goal_reached_streak_;
      if (goal_reached_streak_ >= goal_reached_required_frames_) {
        RCLCPP_INFO(
          get_logger(), "[GOAL][Reached] goal confirmed by %d consecutive frames, hard stop.",
          goal_reached_streak_);
        resetStagedControlState();
        publishHardStopVelocity();
        return true;
      }
      RCLCPP_DEBUG(
        get_logger(), "[GOAL][Armed] frame %d/%d within tolerance, awaiting confirmation.",
        goal_reached_streak_, goal_reached_required_frames_);
    } else if (goal_reached_streak_ != 0) {
      // 命中后立刻掉出容差：视为抖动，清零并继续跟踪，不锁存到位。
      RCLCPP_DEBUG(
        get_logger(), "[GOAL][Disarm] goal check fell out of tolerance after %d frame(s).",
        goal_reached_streak_);
      goal_reached_streak_ = 0;
    }
    rate.sleep();
  }
  publishHardStopVelocity(); return false;
}
}  // namespace nav2_controller

namespace nav2_controller
{
/**
 * @brief 校验当前快照运动状态是否允许非零速度输出（用于 publishVelocity 最终安全门禁）。
 * 
 * 门禁条件：
 * 1. 代价地图必须 current；
 * 2. 规划结果必须非空且标记 success；
 * 3. 已消费的 session、revision 和 limit_revision 必须与当前完全一致；
 * 4. 轨迹完成时间至今未超过 max_age（默认 0.3 秒）。
 */
bool ControllerServer::snapshotMotionAllowed()
{
  if (!costmap_ros_ || !costmap_ros_->getLayeredCostmap() || !costmap_ros_->isCurrent()) {return false;}
  auto r = snapshot_runtime_;
  if (!r) {return false;}
  std::lock_guard<std::mutex> lock(r->mutex);
  return r->has_request && r->result && r->result->success &&
         r->accepted_session == r->session &&
         r->accepted_revision <= r->revision &&
         r->accepted_limit <= r->limit_revision &&
         std::chrono::duration<double>(SnapshotClock::now() - r->accepted_completed).count() <=
         r->max_age;
}

/**
 * @brief 控制线程向规划线程回传上一控制周期实际消费的目标速度与加速度。
 * 
 * 作用：
 * 供后台 10 Hz 规划线程作为下一轮 SpeedPlanner 的初值边界（InitialMotionState），
 * 形成闭环平滑拼接，避免因直接采样底盘离散里程计而造成前向加速度与 Jerk 突变。
 * 
 * @param valid 反馈是否有效（跟踪成功且控制器消费了轨迹目标）
 * @param acceleration 控制器实际跟踪的目标加速度 (m/s^2)
 * @param velocity 控制器实际跟踪的目标速度 (m/s)
 */
void ControllerServer::snapshotControlFeedback(bool valid, double acceleration, double velocity)
{
  auto r = snapshot_runtime_;
  if (!r) {return;}
  std::lock_guard<std::mutex> lock(r->mutex);
  r->feedback_valid = valid && std::isfinite(acceleration) && std::isfinite(velocity);
  r->feedback_acceleration = acceleration;
  r->feedback_velocity = velocity;
  r->feedback_time = SnapshotClock::now();
}
}
