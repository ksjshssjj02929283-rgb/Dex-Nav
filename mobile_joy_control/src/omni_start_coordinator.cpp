#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "chassis_interfaces/msg/chassis_motor_feedback.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_interfaces/action/steering_angle_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int64.hpp"

namespace
{

constexpr std::size_t kWheelCount = 4;
constexpr double kPi = 3.14159265358979323846;

double wrapPi(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle <= -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double angleDistance(double target, double current)
{
  return std::abs(wrapPi(target - current));
}

double quaternionYaw(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double sin_yaw = 2.0 *
    (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cos_yaw = 1.0 - 2.0 *
    (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(sin_yaw, cos_yaw);
}

}  // namespace

class OmniStartCoordinator : public rclcpp::Node
{
public:
  using SteeringAngleControl = nav_interfaces::action::SteeringAngleControl;
  using GoalHandleSteeringAngleControl =
    rclcpp_action::ClientGoalHandle<SteeringAngleControl>;

  OmniStartCoordinator()
  : Node("omni_start_coordinator")
  {
    // [Layer 3A][Task Gate] 速度链、任务边界以及起步预测路径入口。
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/control/cmd_vel_nav_presteer");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/control/cmd_vel_nav");
    auto_mode_topic_ = declare_parameter<std::string>(
      "auto_mode_topic", "/control/nav_auto_mode");
    auto_task_id_topic_ = declare_parameter<std::string>(
      "auto_task_id_topic", "/control/nav_auto_task_id");
    steering_feedback_topic_ = declare_parameter<std::string>(
      "steering_feedback_topic", "/feedback/chassis_motor");
    action_name_ = declare_parameter<std::string>(
      "steering_action_name", "/steering_angle_control");
    preview_path_topic_ = declare_parameter<std::string>(
      "preview_path_topic", "/received_global_plan");

    // [Layer 3B][Initial Steering Solver] 与 region 当前几何和机械限位保持一致。
    chassis_length_ = std::max(
      0.0, declare_parameter<double>("chassis_length", 0.460));
    chassis_width_ = std::max(
      0.0, declare_parameter<double>("chassis_width", 0.345));
    steering_limit_rad_ = std::clamp(
      declare_parameter<double>("steering_limit_rad", 0.8 * kPi),
      0.0, kPi);
    command_linear_deadband_ = std::max(
      0.0, declare_parameter<double>("command_linear_deadband", 0.005));
    command_yaw_deadband_ = std::max(
      0.0, declare_parameter<double>("command_yaw_deadband", 0.008));
    action_timeout_sec_ = std::max(
      0.1, declare_parameter<double>("action_timeout_sec", 3.0));
    settle_tolerance_rad_ = std::max(
      0.001, declare_parameter<double>("settle_tolerance_rad", 0.08));
    steering_feedback_timeout_sec_ = std::max(
      0.01, declare_parameter<double>("steering_feedback_timeout_sec", 0.2));

    // [Layer 3B][Future Branch Prediction] 预测只在每个 AUTO 新任务起步时执行一次。
    prediction_enabled_ = declare_parameter<bool>("prediction_enabled", true);
    prediction_wait_timeout_sec_ = std::max(
      0.0, declare_parameter<double>("prediction_wait_timeout_sec", 0.30));
    prediction_path_max_age_sec_ = std::max(
      0.05, declare_parameter<double>("prediction_path_max_age_sec", 0.50));
    prediction_max_samples_ = static_cast<std::size_t>(std::max<std::int64_t>(
        2, declare_parameter<int>("prediction_max_samples", 120)));
    prediction_branch_switch_penalty_ = std::max(
      0.0, declare_parameter<double>("prediction_branch_switch_penalty", 10.0));
    prediction_min_segment_length_ = std::max(
      1e-4, declare_parameter<double>("prediction_min_segment_length", 0.02));
    prediction_max_yaw_rate_per_speed_ = std::max(
      0.0, declare_parameter<double>("prediction_max_yaw_rate_per_speed", 3.0));

    command_publisher_ =
      create_publisher<geometry_msgs::msg::Twist>(output_topic_, 10);
    command_subscription_ =
      create_subscription<geometry_msgs::msg::Twist>(
      input_topic_, 10,
      std::bind(
        &OmniStartCoordinator::commandCallback, this, std::placeholders::_1));
    auto_mode_subscription_ =
      create_subscription<std_msgs::msg::Bool>(
      auto_mode_topic_, 10,
      std::bind(
        &OmniStartCoordinator::autoModeCallback, this, std::placeholders::_1));
    auto_task_id_subscription_ =
      create_subscription<std_msgs::msg::UInt64>(
      auto_task_id_topic_, 10,
      std::bind(
        &OmniStartCoordinator::autoTaskIdCallback, this, std::placeholders::_1));
    preview_path_subscription_ =
      create_subscription<nav_msgs::msg::Path>(
      preview_path_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      std::bind(
        &OmniStartCoordinator::previewPathCallback, this, std::placeholders::_1));
    steering_feedback_subscription_ =
      create_subscription<chassis_interfaces::msg::ChassisMotorFeedback>(
      steering_feedback_topic_,
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
      std::bind(
        &OmniStartCoordinator::steeringFeedbackCallback,
        this, std::placeholders::_1));

    // [Layer 3C][Action Client] 只请求静止对舵，不拥有底盘电机发布权。
    steering_action_client_ =
      rclcpp_action::create_client<SteeringAngleControl>(this, action_name_);

    RCLCPP_INFO(
      get_logger(),
      "[Layer 3] omni start coordinator ready: %s -> %s, action=%s, preview=%s",
      input_topic_.c_str(), output_topic_.c_str(), action_name_.c_str(),
      preview_path_topic_.c_str());
  }

private:
  enum class GateState
  {
    kPassThrough,
    kWaitingForTask,
    kWaitingForCommand,
    kWaitingForServer,
    kSendingGoal,
    kAligning,
    kReady,
    kFailed
  };

  struct SteeringCandidate
  {
    double angle{0.0};
    int drive_sign{0};
    bool valid{false};
  };

  using CandidatePair = std::array<SteeringCandidate, 2>;

  static const char * stateName(GateState state)
  {
    switch (state) {
      case GateState::kPassThrough:
        return "PASS_THROUGH";
      case GateState::kWaitingForTask:
        return "WAITING_FOR_TASK";
      case GateState::kWaitingForCommand:
        return "WAITING_FOR_COMMAND";
      case GateState::kWaitingForServer:
        return "WAITING_FOR_SERVER";
      case GateState::kSendingGoal:
        return "SENDING_GOAL";
      case GateState::kAligning:
        return "ALIGNING";
      case GateState::kReady:
        return "READY";
      case GateState::kFailed:
        return "FAILED";
    }
    return "UNKNOWN";
  }

  bool isMeaningfulCommand(const geometry_msgs::msg::Twist & command) const
  {
    return std::hypot(command.linear.x, command.linear.y) >
           command_linear_deadband_ ||
           std::abs(command.angular.z) > command_yaw_deadband_;
  }

  void publishZero()
  {
    command_publisher_->publish(geometry_msgs::msg::Twist());
  }

  void cancelActiveGoal(const char * reason)
  {
    // [Layer 3E][Cancellation Safety] 模式退出或任务切换时，先撤销底盘对舵占用。
    if (active_goal_handle_) {
      RCLCPP_INFO(
        get_logger(), "[Layer 3E] cancel pre-alignment: %s", reason);
      steering_action_client_->async_cancel_goal(active_goal_handle_);
      active_goal_handle_.reset();
    }
  }

  void armTask(std::uint64_t task_id)
  {
    if (task_id == 0 || task_id == active_task_id_) {
      return;
    }

    cancelActiveGoal("new AUTO task");
    active_task_id_ = task_id;
    prepared_task_id_ = 0;
    preview_path_received_ = false;
    preview_path_task_id_ = 0;
    latest_preview_path_.poses.clear();
    task_armed_time_ = now();
    gate_state_ = GateState::kWaitingForCommand;
    publishZero();
    RCLCPP_INFO(
      get_logger(),
      "[Layer 3A] AUTO task=%llu armed; navigation command is gated at zero",
      static_cast<unsigned long long>(active_task_id_));
  }

  void autoModeCallback(const std_msgs::msg::Bool::SharedPtr message)
  {
    if (message->data == auto_mode_active_) {
      return;
    }

    auto_mode_active_ = message->data;
    if (auto_mode_active_) {
      // [Layer 3A][Task Gate] AUTO 标志先于 task_id 到达时也立即关闸，防止首帧漏过。
      gate_state_ = GateState::kWaitingForTask;
      active_task_id_ = 0;
      prepared_task_id_ = 0;
      publishZero();
      if (announced_task_id_ != 0) {
        armTask(announced_task_id_);
      }
      RCLCPP_INFO(get_logger(), "[Layer 3A] AUTO mode entered; start gate closed");
      return;
    }

    cancelActiveGoal("AUTO mode exited");
    active_task_id_ = 0;
    prepared_task_id_ = 0;
    announced_task_id_ = 0;
    preview_path_received_ = false;
    preview_path_task_id_ = 0;
    latest_preview_path_.poses.clear();
    gate_state_ = GateState::kPassThrough;
    RCLCPP_INFO(get_logger(), "[Layer 3A] AUTO mode exited; pass-through restored");
  }

  void autoTaskIdCallback(const std_msgs::msg::UInt64::SharedPtr message)
  {
    announced_task_id_ = message->data;
    if (auto_mode_active_) {
      armTask(announced_task_id_);
    }
  }

  void previewPathCallback(const nav_msgs::msg::Path::SharedPtr message)
  {
    // RPP 每个控制周期发布 base_link 下的局部路径。只缓存当前 AUTO 任务的版本，
    // armTask() 会清掉上一任务的路径，避免跨任务误用。
    if (!auto_mode_active_ || active_task_id_ == 0 || message->poses.size() < 2) {
      return;
    }
    latest_preview_path_ = *message;
    preview_path_received_ = true;
    preview_path_task_id_ = active_task_id_;
    preview_path_receive_time_ = now();
  }

  void steeringFeedbackCallback(
    const chassis_interfaces::msg::ChassisMotorFeedback::SharedPtr message)
  {
    if (message->position.size() < kWheelCount) {
      return;
    }

    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
      if (!std::isfinite(message->position[wheel])) {
        return;
      }
    }
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
      current_steering_angle_[wheel] =
        static_cast<double>(message->position[wheel]);
    }
    // [Layer 3B][Feedback Freshness] 起步角分支必须基于近期真实反馈选择。
    last_steering_feedback_time_ = now();
    steering_feedback_received_ = true;
  }

  bool wheelCandidates(
    const geometry_msgs::msg::Twist & command,
    std::size_t wheel,
    CandidatePair & candidates) const
  {
    const bool is_right = wheel == 1 || wheel == 3;
    const bool is_front = wheel == 0 || wheel == 1;
    const double width_sign = is_right ? 1.0 : -1.0;
    const double length_sign = is_front ? 1.0 : -1.0;
    const double wheel_vx =
      command.linear.x + width_sign * chassis_width_ * command.angular.z * 0.5;
    const double wheel_vy =
      command.linear.y + length_sign * chassis_length_ * command.angular.z * 0.5;
    if (!std::isfinite(wheel_vx) || !std::isfinite(wheel_vy) ||
      std::hypot(wheel_vx, wheel_vy) <= 1e-4)
    {
      return false;
    }

    const double candidate_a = wrapPi(std::atan2(wheel_vy, wheel_vx));
    const double candidate_b = wrapPi(candidate_a + kPi);
    candidates[0] = {
      candidate_a, 1, std::abs(candidate_a) <= steering_limit_rad_};
    candidates[1] = {
      candidate_b, -1, std::abs(candidate_b) <= steering_limit_rad_};
    return true;
  }

  bool computeInstantSteeringTargets(
    const geometry_msgs::msg::Twist & command,
    std::array<float, kWheelCount> & targets) const
  {
    // [Layer 3B][Initial Steering Solver]
    // 只求首条车体 Twist 对应的四轮角度；轮速和持续运动仍由 region 独立解算。
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
      CandidatePair candidates{};
      if (!wheelCandidates(command, wheel, candidates)) {
        targets[wheel] = static_cast<float>(current_steering_angle_[wheel]);
        continue;
      }

      const double cost_a = candidates[0].valid ?
        angleDistance(candidates[0].angle, current_steering_angle_[wheel]) :
        std::numeric_limits<double>::infinity();
      const double cost_b = candidates[1].valid ?
        angleDistance(candidates[1].angle, current_steering_angle_[wheel]) :
        std::numeric_limits<double>::infinity();
      if (!std::isfinite(cost_a) && !std::isfinite(cost_b)) {
        RCLCPP_ERROR(
          get_logger(),
          "[Layer 3B] wheel=%zu has no steering solution within +/-%.3f rad",
          wheel, steering_limit_rad_);
        return false;
      }
      targets[wheel] = static_cast<float>(
        cost_a <= cost_b ? candidates[0].angle : candidates[1].angle);
    }
    return true;
  }

  bool previewPathIsUsable() const
  {
    return prediction_enabled_ && preview_path_received_ &&
           preview_path_task_id_ == active_task_id_ &&
           latest_preview_path_.poses.size() >= 2 &&
           (now() - preview_path_receive_time_).seconds() <=
           prediction_path_max_age_sec_;
  }

  std::vector<geometry_msgs::msg::Twist> buildPredictionCommands() const
  {
    std::vector<geometry_msgs::msg::Twist> commands;
    if (!previewPathIsUsable()) {
      return commands;
    }

    const auto & poses = latest_preview_path_.poses;
    const std::size_t segment_count = poses.size() - 1;
    const std::size_t sample_count = std::min(
      prediction_max_samples_, segment_count);
    commands.reserve(sample_count);
    std::size_t previous_segment = segment_count;

    for (std::size_t sample = 0; sample < sample_count; ++sample) {
      const std::size_t segment = sample_count == 1 ? 0 :
        sample * (segment_count - 1) / (sample_count - 1);
      if (segment == previous_segment) {
        continue;
      }
      previous_segment = segment;

      const auto & start = poses[segment].pose;
      const auto & finish = poses[segment + 1].pose;
      const double dx = finish.position.x - start.position.x;
      const double dy = finish.position.y - start.position.y;
      const double ds = std::hypot(dx, dy);
      if (!std::isfinite(ds) || ds < prediction_min_segment_length_) {
        continue;
      }

      const double start_yaw = quaternionYaw(start.orientation);
      const double finish_yaw = quaternionYaw(finish.orientation);
      const double tangent_yaw = std::atan2(dy, dx);
      if (!std::isfinite(start_yaw) || !std::isfinite(finish_yaw) ||
        !std::isfinite(tangent_yaw))
      {
        continue;
      }

      // 以单位路径速度构造未来 Twist。舵角只依赖 vx:vy:wz 的比例，因此无需
      // 猜测 SpeedPlanner 的绝对速度；yaw/ds 则是稳态下 PID 应产生的角速度比例。
      const double body_translation_yaw = wrapPi(tangent_yaw - finish_yaw);
      const double yaw_rate_per_speed = std::max(
        -prediction_max_yaw_rate_per_speed_, std::min(
          prediction_max_yaw_rate_per_speed_,
          wrapPi(finish_yaw - start_yaw) / ds));
      geometry_msgs::msg::Twist command;
      command.linear.x = std::cos(body_translation_yaw);
      command.linear.y = std::sin(body_translation_yaw);
      command.angular.z = yaw_rate_per_speed;
      commands.push_back(command);
    }
    return commands;
  }

  bool computePredictedSteeringTargets(
    const geometry_msgs::msg::Twist & initial_command,
    std::array<float, kWheelCount> & targets,
    std::size_t & preview_samples,
    std::size_t & predicted_switches,
    double & total_cost) const
  {
    const auto future_commands = buildPredictionCommands();
    preview_samples = future_commands.size();
    predicted_switches = 0;
    total_cost = 0.0;
    if (future_commands.empty()) {
      return false;
    }

    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
      std::vector<CandidatePair> sequence;
      CandidatePair initial_candidates{};
      if (!wheelCandidates(initial_command, wheel, initial_candidates)) {
        targets[wheel] = static_cast<float>(current_steering_angle_[wheel]);
        continue;
      }
      sequence.push_back(initial_candidates);
      for (const auto & command : future_commands) {
        CandidatePair candidates{};
        if (wheelCandidates(command, wheel, candidates)) {
          sequence.push_back(candidates);
        }
      }

      std::vector<std::array<double, 2>> costs(sequence.size());
      std::vector<std::array<int, 2>> parents(sequence.size());
      const double infinity = std::numeric_limits<double>::infinity();
      for (std::size_t state = 0; state < 2; ++state) {
        costs[0][state] = sequence[0][state].valid ?
          angleDistance(
          sequence[0][state].angle, current_steering_angle_[wheel]) : infinity;
        parents[0][state] = -1;
      }

      for (std::size_t step = 1; step < sequence.size(); ++step) {
        for (std::size_t state = 0; state < 2; ++state) {
          costs[step][state] = infinity;
          parents[step][state] = -1;
          if (!sequence[step][state].valid) {
            continue;
          }
          for (std::size_t previous = 0; previous < 2; ++previous) {
            if (!std::isfinite(costs[step - 1][previous])) {
              continue;
            }
            const bool changes_drive_direction =
              sequence[step][state].drive_sign !=
              sequence[step - 1][previous].drive_sign;
            const double candidate_cost = costs[step - 1][previous] +
              angleDistance(
              sequence[step][state].angle,
              sequence[step - 1][previous].angle) +
              (changes_drive_direction ? prediction_branch_switch_penalty_ : 0.0);
            if (candidate_cost < costs[step][state]) {
              costs[step][state] = candidate_cost;
              parents[step][state] = static_cast<int>(previous);
            }
          }
        }
      }

      const std::size_t last = sequence.size() - 1;
      int state = costs[last][0] <= costs[last][1] ? 0 : 1;
      if (!std::isfinite(costs[last][state])) {
        return false;
      }
      total_cost += costs[last][state];
      std::vector<int> optimal_states(sequence.size(), 0);
      optimal_states[last] = state;
      for (std::size_t step = last; step > 0; --step) {
        state = parents[step][state];
        if (state < 0) {
          return false;
        }
        optimal_states[step - 1] = state;
      }
      for (std::size_t step = 1; step < optimal_states.size(); ++step) {
        if (sequence[step][optimal_states[step]].drive_sign !=
          sequence[step - 1][optimal_states[step - 1]].drive_sign)
        {
          ++predicted_switches;
        }
      }
      // sequence[0] 是速度规划刚起步时的瞬时 Twist。此时线速度尚在从零建立，
      // wz/v 往往异常偏大，若对齐这个角度，Action 执行期间线速度一增长就会立刻
      // 要求再次大幅换舵。DP 已经算出了未来分支，应对齐首个可用预览运动角，
      // 同时沿用该步的最优正反转分支；没有可用预览候选时才退回瞬时角。
      const std::size_t alignment_step = sequence.size() > 1 ? 1 : 0;
      targets[wheel] = static_cast<float>(
        sequence[alignment_step][optimal_states[alignment_step]].angle);
    }
    return true;
  }

  bool sendPreAlignmentGoal(const geometry_msgs::msg::Twist & command)
  {
    const bool steering_feedback_fresh =
      steering_feedback_received_ &&
      (now() - last_steering_feedback_time_).seconds() <=
      steering_feedback_timeout_sec_;
    if (!steering_feedback_fresh) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[Layer 3B] waiting for fresh steering feedback before pre-alignment");
      return false;
    }
    if (!steering_action_client_->action_server_is_ready()) {
      gate_state_ = GateState::kWaitingForServer;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[Layer 3C] waiting for %s action server", action_name_.c_str());
      return false;
    }

    std::array<float, kWheelCount> targets{};
    const auto prediction_start = std::chrono::steady_clock::now();
    std::size_t preview_samples = 0;
    std::size_t predicted_switches = 0;
    double prediction_cost = 0.0;
    const bool used_prediction = computePredictedSteeringTargets(
      command, targets, preview_samples, predicted_switches, prediction_cost);
    if (!used_prediction && !computeInstantSteeringTargets(command, targets)) {
      gate_state_ = GateState::kFailed;
      return false;
    }
    const double prediction_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - prediction_start).count();
    if (used_prediction) {
      RCLCPP_INFO(
        get_logger(),
        "[Layer 3B] future branch prediction: samples=%zu switches=%zu "
        "cost=%.3f compute=%.3f ms",
        preview_samples, predicted_switches, prediction_cost, prediction_ms);
    } else if (prediction_enabled_) {
      RCLCPP_WARN(
        get_logger(),
        "[Layer 3B] preview unavailable; using first-command steering solution");
    }

    SteeringAngleControl::Goal goal;
    goal.target_steer_angles_rad = targets;
    goal.timeout_sec = static_cast<float>(action_timeout_sec_);
    goal.settle_tolerance_rad = static_cast<float>(settle_tolerance_rad_);

    const std::uint64_t request_task_id = active_task_id_;
    auto options =
      rclcpp_action::Client<SteeringAngleControl>::SendGoalOptions();
    options.goal_response_callback =
      [this, request_task_id](GoalHandleSteeringAngleControl::SharedPtr goal_handle)
      {
        if (!goal_handle) {
          if (auto_mode_active_ && active_task_id_ == request_task_id) {
            gate_state_ = GateState::kFailed;
            RCLCPP_ERROR(
              get_logger(),
              "[Layer 3C] pre-alignment goal rejected for task=%llu",
              static_cast<unsigned long long>(request_task_id));
          }
          return;
        }

        if (!auto_mode_active_ || active_task_id_ != request_task_id) {
          steering_action_client_->async_cancel_goal(goal_handle);
          return;
        }
        active_goal_handle_ = goal_handle;
        gate_state_ = GateState::kAligning;
      };
    options.result_callback =
      [this, request_task_id](
      const GoalHandleSteeringAngleControl::WrappedResult & result)
      {
        if (!auto_mode_active_ || active_task_id_ != request_task_id) {
          return;
        }
        active_goal_handle_.reset();
        const bool succeeded =
          result.code == rclcpp_action::ResultCode::SUCCEEDED &&
          result.result && result.result->success;
        if (!succeeded) {
          gate_state_ = GateState::kFailed;
          RCLCPP_ERROR(
            get_logger(),
            "[Layer 3C] pre-alignment failed for task=%llu: %s",
            static_cast<unsigned long long>(request_task_id),
            result.result ? result.result->message.c_str() : "no result");
          publishZero();
          return;
        }

        // [Layer 3D][Command Release] Action 已退出控制，立即放行最新而非过期首帧。
        prepared_task_id_ = request_task_id;
        gate_state_ = GateState::kReady;
        command_publisher_->publish(latest_command_);
        RCLCPP_INFO(
          get_logger(),
          "[Layer 3D] task=%llu pre-aligned; latest navigation command released",
          static_cast<unsigned long long>(request_task_id));
      };

    gate_state_ = GateState::kSendingGoal;
    steering_action_client_->async_send_goal(goal, options);
    RCLCPP_INFO(
      get_logger(),
      "[Layer 3C] task=%llu pre-alignment targets=[%.3f, %.3f, %.3f, %.3f]",
      static_cast<unsigned long long>(request_task_id),
      targets[0], targets[1], targets[2], targets[3]);
    return true;
  }

  void commandCallback(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    latest_command_ = *message;

    if (!auto_mode_active_) {
      command_publisher_->publish(*message);
      return;
    }

    if (
      gate_state_ == GateState::kReady &&
      prepared_task_id_ != 0 &&
      prepared_task_id_ == active_task_id_)
    {
      command_publisher_->publish(*message);
      return;
    }

    // [Layer 3A][Task Gate] 对舵完成前每个输入周期都明确发布零速，避免 mux 超时。
    publishZero();
    const bool may_start =
      gate_state_ == GateState::kWaitingForCommand ||
      gate_state_ == GateState::kWaitingForServer;
    if (
      may_start && active_task_id_ != 0 &&
      isMeaningfulCommand(*message))
    {
      if (prediction_enabled_ && !previewPathIsUsable() &&
        (now() - task_armed_time_).seconds() < prediction_wait_timeout_sec_)
      {
        RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 500,
          "[Layer 3B] waiting briefly for RPP preview path");
        return;
      }
      sendPreAlignmentGoal(*message);
    }

    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[Layer 3A] AUTO command gated: state=%s task=%llu",
      stateName(gate_state_),
      static_cast<unsigned long long>(active_task_id_));
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string auto_mode_topic_;
  std::string auto_task_id_topic_;
  std::string steering_feedback_topic_;
  std::string action_name_;
  std::string preview_path_topic_;

  double chassis_length_{0.460};
  double chassis_width_{0.345};
  double steering_limit_rad_{0.8 * kPi};
  double command_linear_deadband_{0.005};
  double command_yaw_deadband_{0.008};
  double action_timeout_sec_{3.0};
  double settle_tolerance_rad_{0.08};
  double steering_feedback_timeout_sec_{0.2};
  bool prediction_enabled_{true};
  double prediction_wait_timeout_sec_{0.30};
  double prediction_path_max_age_sec_{0.50};
  std::size_t prediction_max_samples_{120};
  double prediction_branch_switch_penalty_{10.0};
  double prediction_min_segment_length_{0.02};
  double prediction_max_yaw_rate_per_speed_{3.0};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr auto_mode_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr auto_task_id_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr preview_path_subscription_;
  rclcpp::Subscription<chassis_interfaces::msg::ChassisMotorFeedback>::SharedPtr
    steering_feedback_subscription_;
  rclcpp_action::Client<SteeringAngleControl>::SharedPtr steering_action_client_;
  GoalHandleSteeringAngleControl::SharedPtr active_goal_handle_;

  GateState gate_state_{GateState::kPassThrough};
  bool auto_mode_active_{false};
  bool steering_feedback_received_{false};
  bool preview_path_received_{false};
  rclcpp::Time last_steering_feedback_time_;
  rclcpp::Time task_armed_time_;
  rclcpp::Time preview_path_receive_time_;
  std::uint64_t announced_task_id_{0};
  std::uint64_t active_task_id_{0};
  std::uint64_t prepared_task_id_{0};
  std::uint64_t preview_path_task_id_{0};
  geometry_msgs::msg::Twist latest_command_;
  nav_msgs::msg::Path latest_preview_path_;
  std::array<double, kWheelCount> current_steering_angle_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OmniStartCoordinator>());
  rclcpp::shutdown();
  return 0;
}
