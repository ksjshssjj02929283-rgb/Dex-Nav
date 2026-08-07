#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "chassis_interfaces/msg/chassis_motor_control.hpp"
#include "geometry_msgs/msg/twist.hpp"
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
    // [Layer 3A][Task Gate] 参数只描述速度链和任务边界，不包含地图或预测配置。
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/control/cmd_vel_nav_presteer");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/control/cmd_vel_nav");
    auto_mode_topic_ = declare_parameter<std::string>(
      "auto_mode_topic", "/control/nav_auto_mode");
    auto_task_id_topic_ = declare_parameter<std::string>(
      "auto_task_id_topic", "/control/nav_auto_task_id");
    steering_feedback_topic_ = declare_parameter<std::string>(
      "steering_feedback_topic", "/feedback/omni_chassis_motor");
    action_name_ = declare_parameter<std::string>(
      "steering_action_name", "/steering_angle_control");

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
    steering_feedback_subscription_ =
      create_subscription<chassis_interfaces::msg::ChassisMotorControl>(
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
      "[Layer 3] omni start coordinator ready: %s -> %s, action=%s",
      input_topic_.c_str(), output_topic_.c_str(), action_name_.c_str());
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

  void steeringFeedbackCallback(
    const chassis_interfaces::msg::ChassisMotorControl::SharedPtr message)
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

  bool computeInitialSteeringTargets(
    const geometry_msgs::msg::Twist & command,
    std::array<float, kWheelCount> & targets) const
  {
    // [Layer 3B][Initial Steering Solver]
    // 只求首条车体 Twist 对应的四轮角度；轮速和持续运动仍由 region 独立解算。
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
      const bool is_right = wheel == 1 || wheel == 3;
      const bool is_front = wheel == 0 || wheel == 1;
      const double width_sign = is_right ? 1.0 : -1.0;
      const double length_sign = is_front ? 1.0 : -1.0;
      const double wheel_vx =
        command.linear.x +
        width_sign * chassis_width_ * command.angular.z * 0.5;
      const double wheel_vy =
        command.linear.y +
        length_sign * chassis_length_ * command.angular.z * 0.5;

      if (std::hypot(wheel_vx, wheel_vy) <= 1e-4) {
        targets[wheel] = static_cast<float>(current_steering_angle_[wheel]);
        continue;
      }

      const double candidate_a = wrapPi(std::atan2(wheel_vy, wheel_vx));
      const double candidate_b = wrapPi(candidate_a + kPi);
      const bool valid_a = std::abs(candidate_a) <= steering_limit_rad_;
      const bool valid_b = std::abs(candidate_b) <= steering_limit_rad_;
      if (!valid_a && !valid_b) {
        RCLCPP_ERROR(
          get_logger(),
          "[Layer 3B] wheel=%zu has no steering solution within +/-%.3f rad",
          wheel, steering_limit_rad_);
        return false;
      }

      const double cost_a = valid_a ?
        angleDistance(candidate_a, current_steering_angle_[wheel]) :
        std::numeric_limits<double>::infinity();
      const double cost_b = valid_b ?
        angleDistance(candidate_b, current_steering_angle_[wheel]) :
        std::numeric_limits<double>::infinity();
      targets[wheel] = static_cast<float>(cost_a <= cost_b ? candidate_a : candidate_b);
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
    if (!computeInitialSteeringTargets(command, targets)) {
      gate_state_ = GateState::kFailed;
      return false;
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

  double chassis_length_{0.460};
  double chassis_width_{0.345};
  double steering_limit_rad_{0.8 * kPi};
  double command_linear_deadband_{0.005};
  double command_yaw_deadband_{0.008};
  double action_timeout_sec_{3.0};
  double settle_tolerance_rad_{0.08};
  double steering_feedback_timeout_sec_{0.2};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr auto_mode_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr auto_task_id_subscription_;
  rclcpp::Subscription<chassis_interfaces::msg::ChassisMotorControl>::SharedPtr
    steering_feedback_subscription_;
  rclcpp_action::Client<SteeringAngleControl>::SharedPtr steering_action_client_;
  GoalHandleSteeringAngleControl::SharedPtr active_goal_handle_;

  GateState gate_state_{GateState::kPassThrough};
  bool auto_mode_active_{false};
  bool steering_feedback_received_{false};
  rclcpp::Time last_steering_feedback_time_;
  std::uint64_t announced_task_id_{0};
  std::uint64_t active_task_id_{0};
  std::uint64_t prepared_task_id_{0};
  geometry_msgs::msg::Twist latest_command_;
  std::array<double, kWheelCount> current_steering_angle_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OmniStartCoordinator>());
  rclcpp::shutdown();
  return 0;
}
