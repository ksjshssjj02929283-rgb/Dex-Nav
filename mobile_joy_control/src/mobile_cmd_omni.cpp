// 轻量全向舵轮分配器
//
// 设计目标：
// 1. 保留真实舵角下的侧滑抑制与速度可行性；
// 2. 用一次固定 3x3 正定方程替代活动集枚举；
// 3. angle / angle+pi 选择带滞回，真正换支时整车先停稳再对舵；
// 4. 驻车、解除驻车、反馈超时均保持零轮速安全语义。

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include "chassis_interfaces/msg/chassis_motor_control.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_interfaces/action/steering_angle_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace
{

constexpr float kPi = 3.14159265358979323846f;
constexpr std::size_t kWheelCount = 4;
constexpr std::size_t kTwistDimension = 3;

using Vector3 = std::array<float, kTwistDimension>;
using Matrix3 = std::array<std::array<float, kTwistDimension>, kTwistDimension>;

// 机械参数。轮序统一为 FL、FR、RL、RR。
constexpr float kChassisLengthM = 0.460f;
constexpr float kChassisWidthM = 0.345f;
constexpr float kWheelDiameterM = 0.107f;
constexpr float kSteerAngleLimitRad = 5.0f * kPi / 6.0f;  // +/-150 deg
constexpr float kSteerLimitMarginRad = 0.08f;

constexpr float kControlPeriodS = 0.05f;  // 20 Hz
constexpr float kCommandTimeoutS = 0.50f;
constexpr float kSteeringFeedbackTimeoutS = 0.25f;
constexpr float kDriveFeedbackTimeoutS = 0.25f;

// 最终输出能力上限。
constexpr float kMaxBodySpeedX = 1.20f;
constexpr float kMaxBodySpeedY = 1.20f;
constexpr float kMaxBodyYawRate = 0.75f;
constexpr float kMaxWheelLinearSpeed = 1.50f;
constexpr float kMaxWheelLateralSpeed = 0.050f;

// 只保留一层加速度限幅，避免与上游 velocity_smoother 和优化器连续性项重复平滑。
constexpr float kMaxAccelX = 0.60f;
constexpr float kMaxAccelY = 0.60f;
constexpr float kMaxAccelYaw = 0.70f;
constexpr float kMaxBrakeX = 0.80f;
constexpr float kMaxBrakeY = 0.80f;
constexpr float kMaxBrakeYaw = 0.90f;

// 舵角误差小于 full 时全速；达到 stop 时四轮驱动严格为零。
constexpr float kAlignFullSpeedErrorRad = 0.08f;
constexpr float kAlignStopErrorRad = 0.35f;
constexpr float kSteerAlignToleranceRad = 0.08f;
constexpr int kSteerAlignStableCycles = 3;
constexpr float kMaxSteerTargetStepRad = 0.12f;

// 等价分支保持滞回，接近有效限位时允许提前换支。
constexpr float kBranchSwitchHysteresisRad = 0.20f;
constexpr float kSteerLimitGuardRad = 0.12f;
constexpr float kSteerLimitSoftZoneRad = 0.30f;
constexpr float kSteerLimitCostGain = 0.25f;
constexpr float kBranchSwitchMinimumTargetJumpRad = 0.35f;

// 轻量 QP（无约束加权最小二乘）权重。
constexpr float kTrackWeightX = 18.0f;
constexpr float kTrackWeightY = 18.0f;
constexpr float kTrackWeightYaw = 12.0f;
constexpr float kLateralSlipWeight = 8.0f;
constexpr float kRegularization = 0.01f;
constexpr float kLinearSolveTolerance = 1.0e-8f;

constexpr float kWheelDirectionUndefinedMps = 0.001f;
constexpr float kLinearCommandDeadbandMps = 0.002f;
constexpr float kYawCommandDeadbandRadps = 0.003f;

// 换支状态：先用固定周期降到零；有驱动反馈时还要连续确认实际停稳。
constexpr int kBranchBrakeCycles = 5;
constexpr float kStoppedRpm = 3.0f;
constexpr int kStoppedStableCycles = 3;
constexpr int kPostAlignmentDriveRampCycles = 5;

// 驻车成功后保持舵角；保护期结束后，明确的新运动命令才能解除。
constexpr float kParkPreemptGraceS = 0.80f;
constexpr float kParkPreemptLinearMps = 0.04f;
constexpr float kParkPreemptYawRadps = 0.04f;

// 只吞掉原地旋转命令中的数值级平移噪声。
constexpr float kRotationModeEnterYawRadps = 0.15f;
constexpr float kRotationModeExitYawRadps = 0.10f;
constexpr float kRotationModeEnterLinearMps = 0.02f;
constexpr float kRotationModeExitLinearMps = 0.04f;

float clampf(float value, float lower, float upper)
{
  return std::max(lower, std::min(upper, value));
}

float wrapPi(float angle)
{
  while (angle > kPi) {
    angle -= 2.0f * kPi;
  }
  while (angle <= -kPi) {
    angle += 2.0f * kPi;
  }
  return angle;
}

float angleDiff(float target, float current)
{
  return wrapPi(target - current);
}

float approachAngle(float target, float current, float maximum_step)
{
  const float delta = angleDiff(target, current);
  if (std::abs(delta) <= maximum_step) {
    return target;
  }
  return wrapPi(current + std::copysign(maximum_step, delta));
}

float approachValue(float target, float current, float maximum_step)
{
  return current + clampf(target - current, -maximum_step, maximum_step);
}

float speedToRpm(float speed_mps)
{
  return speed_mps * 60.0f / (kPi * kWheelDiameterM);
}

float dot(const Vector3 & lhs, const Vector3 & rhs)
{
  return lhs[0] * rhs[0] + lhs[1] * rhs[1] + lhs[2] * rhs[2];
}

void addOuterProduct(Matrix3 & matrix, const Vector3 & vector, float weight)
{
  for (std::size_t row = 0; row < kTwistDimension; ++row) {
    for (std::size_t column = 0; column < kTwistDimension; ++column) {
      matrix[row][column] += weight * vector[row] * vector[column];
    }
  }
}

float scaleFromSteeringError(float error)
{
  if (error <= kAlignFullSpeedErrorRad) {
    return 1.0f;
  }
  if (error >= kAlignStopErrorRad) {
    return 0.0f;
  }
  return clampf(
    (kAlignStopErrorRad - error) /
    (kAlignStopErrorRad - kAlignFullSpeedErrorRad),
    0.0f, 1.0f);
}

void applySymmetricBound(float value, float absolute_bound, float & scale)
{
  const float magnitude = std::abs(value);
  if (magnitude > absolute_bound && magnitude > 1.0e-7f) {
    scale = std::min(scale, absolute_bound / magnitude);
  }
}

bool finiteVector(const Vector3 & vector)
{
  return std::isfinite(vector[0]) &&
         std::isfinite(vector[1]) &&
         std::isfinite(vector[2]);
}

// Hessian 始终由正 tracking 对角项和半正定侧滑项组成，因此用固定 Cholesky
// 求解即可，不需要求逆、堆分配或活动集枚举。
bool solvePositiveDefinite(
  const Matrix3 & matrix, const Vector3 & rhs, Vector3 & solution)
{
  Matrix3 lower{};
  for (std::size_t row = 0; row < kTwistDimension; ++row) {
    for (std::size_t column = 0; column <= row; ++column) {
      float sum = matrix[row][column];
      for (std::size_t inner = 0; inner < column; ++inner) {
        sum -= lower[row][inner] * lower[column][inner];
      }
      if (row == column) {
        if (!(sum > kLinearSolveTolerance) || !std::isfinite(sum)) {
          return false;
        }
        lower[row][column] = std::sqrt(sum);
      } else {
        lower[row][column] = sum / lower[column][column];
      }
    }
  }

  Vector3 intermediate{};
  for (std::size_t row = 0; row < kTwistDimension; ++row) {
    float sum = rhs[row];
    for (std::size_t column = 0; column < row; ++column) {
      sum -= lower[row][column] * intermediate[column];
    }
    intermediate[row] = sum / lower[row][row];
  }

  for (int row = static_cast<int>(kTwistDimension) - 1; row >= 0; --row) {
    float sum = intermediate[static_cast<std::size_t>(row)];
    for (std::size_t column = static_cast<std::size_t>(row) + 1;
      column < kTwistDimension; ++column)
    {
      sum -= lower[column][static_cast<std::size_t>(row)] * solution[column];
    }
    solution[static_cast<std::size_t>(row)] =
      sum / lower[static_cast<std::size_t>(row)][static_cast<std::size_t>(row)];
  }
  return finiteVector(solution);
}

}  // namespace

class MobileOmniController : public rclcpp::Node
{
public:
  using SteeringAngleControl = nav_interfaces::action::SteeringAngleControl;
  using GoalHandleSteeringAngleControl =
    rclcpp_action::ServerGoalHandle<SteeringAngleControl>;

  MobileOmniController()
  : Node("mobile_cmd_omni")
  {
    current_steer_angle_.fill(0.0f);
    commanded_steer_angle_.fill(0.0f);
    current_drive_rpm_.fill(0.0f);
    previous_output_rpm_.fill(0.0f);
    brake_start_rpm_.fill(0.0f);
    pending_steer_angle_.fill(0.0f);
    pending_flipped_.fill(false);
    previous_flipped_.fill(false);
    park_targets_.fill(0.0f);

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/control/cmd_vel_omni", 10,
      std::bind(&MobileOmniController::cmdVelCallback, this, std::placeholders::_1));

    feedback_sub_ = create_subscription<chassis_interfaces::msg::ChassisMotorControl>(
      "/feedback/omni_chassis_motor",
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
      std::bind(&MobileOmniController::feedbackCallback, this, std::placeholders::_1));

    motor_control_pub_ =
      create_publisher<chassis_interfaces::msg::ChassisMotorControl>(
      "/control/omni_chassis_motors", 10);
    omni_debug_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>("/control/omni_debug", 10);

    steering_angle_control_server_ =
      rclcpp_action::create_server<SteeringAngleControl>(
      this, "steering_angle_control",
      std::bind(
        &MobileOmniController::handleParkGoal, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(
        &MobileOmniController::handleParkCancel, this, std::placeholders::_1),
      std::bind(
        &MobileOmniController::handleParkAccepted, this, std::placeholders::_1));

    last_cmd_time_ = now();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&MobileOmniController::publishControlCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "Mobile omni controller started: lightweight 3x3 QP + branch hysteresis + "
      "BRAKE/ALIGN + /steering_angle_control");
  }

private:
  enum class ControlState
  {
    kRun = 0,
    kBrake = 1,
    kAlign = 2,
    kUnpark = 3
  };

  struct SteerCandidate
  {
    float angle{0.0f};
    float cost{std::numeric_limits<float>::infinity()};
    bool flipped{false};
    bool valid{false};
  };

  struct SteeringSelection
  {
    std::array<float, kWheelCount> target_angle{};
    std::array<bool, kWheelCount> flipped{};
    float maximum_feedback_error{0.0f};
    bool success{false};
  };

  struct AllocationResult
  {
    Vector3 twist{};
    float feasibility_scale{0.0f};
    float alignment_scale{0.0f};
    float release_scale{0.0f};
    float total_scale{0.0f};
    float maximum_lateral_speed{0.0f};
    float maximum_steering_error{0.0f};
    double solve_time_us{0.0};
    bool success{false};
  };

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    latest_cmd_ = *message;
    last_cmd_time_ = now();
  }

  void feedbackCallback(
    const chassis_interfaces::msg::ChassisMotorControl::SharedPtr message)
  {
    if (message->position.size() >= kWheelCount) {
      bool valid = true;
      for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        valid = valid && std::isfinite(message->position[wheel]);
      }
      if (valid) {
        for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
          current_steer_angle_[wheel] =
            static_cast<float>(message->position[wheel]);
        }
        steering_feedback_received_ = true;
        last_steering_feedback_time_ = now();
      }
    }

    if (message->speed.size() >= kWheelCount) {
      bool valid = true;
      for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        valid = valid && std::isfinite(message->speed[wheel]);
      }
      if (valid) {
        for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
          // 与原控制链保持一致：反馈 speed 按 rpm 使用。
          current_drive_rpm_[wheel] = static_cast<float>(message->speed[wheel]);
        }
        drive_feedback_received_ = true;
        last_drive_feedback_time_ = now();
      }
    }
  }

  bool steeringFeedbackFresh() const
  {
    return steering_feedback_received_ &&
           (now() - last_steering_feedback_time_).seconds() <=
           kSteeringFeedbackTimeoutS;
  }

  bool driveFeedbackFresh() const
  {
    return drive_feedback_received_ &&
           (now() - last_drive_feedback_time_).seconds() <=
           kDriveFeedbackTimeoutS;
  }

  float maximumDriveFeedbackRpm() const
  {
    float maximum = 0.0f;
    for (const float rpm : current_drive_rpm_) {
      maximum = std::max(maximum, std::abs(rpm));
    }
    return maximum;
  }

  Vector3 wheelLongitudinalRow(
    std::size_t wheel_index, float steer_angle) const
  {
    const bool is_right = wheel_index == 1 || wheel_index == 3;
    const bool is_front = wheel_index == 0 || wheel_index == 1;
    const float width_sign = is_right ? 1.0f : -1.0f;
    const float length_sign = is_front ? 1.0f : -1.0f;
    const float cosine = std::cos(steer_angle);
    const float sine = std::sin(steer_angle);
    return Vector3{
      cosine,
      sine,
      cosine * width_sign * kChassisWidthM * 0.5f +
      sine * length_sign * kChassisLengthM * 0.5f};
  }

  Vector3 wheelLateralRow(
    std::size_t wheel_index, float steer_angle) const
  {
    const bool is_right = wheel_index == 1 || wheel_index == 3;
    const bool is_front = wheel_index == 0 || wheel_index == 1;
    const float width_sign = is_right ? 1.0f : -1.0f;
    const float length_sign = is_front ? 1.0f : -1.0f;
    const float cosine = std::cos(steer_angle);
    const float sine = std::sin(steer_angle);
    return Vector3{
      -sine,
      cosine,
      -sine * width_sign * kChassisWidthM * 0.5f +
      cosine * length_sign * kChassisLengthM * 0.5f};
  }

  Vector3 wheelVector(const Vector3 & command, std::size_t wheel_index) const
  {
    const bool is_right = wheel_index == 1 || wheel_index == 3;
    const bool is_front = wheel_index == 0 || wheel_index == 1;
    const float width_sign = is_right ? 1.0f : -1.0f;
    const float length_sign = is_front ? 1.0f : -1.0f;
    return Vector3{
      command[0] + width_sign * kChassisWidthM * command[2] * 0.5f,
      command[1] + length_sign * kChassisLengthM * command[2] * 0.5f,
      0.0f};
  }

  float steerCandidateCost(float angle, float reference) const
  {
    float cost = std::abs(angleDiff(angle, reference));
    const float effective_limit = kSteerAngleLimitRad - kSteerLimitMarginRad;
    const float soft_zone_start = effective_limit - kSteerLimitSoftZoneRad;
    if (std::abs(angle) > soft_zone_start) {
      const float normalized =
        (std::abs(angle) - soft_zone_start) / kSteerLimitSoftZoneRad;
      cost += kSteerLimitCostGain * normalized * normalized;
    }
    return cost;
  }

  std::array<SteerCandidate, 2> wheelCandidates(
    const Vector3 & command, std::size_t wheel_index, float reference) const
  {
    const Vector3 vector = wheelVector(command, wheel_index);
    const float base_angle = std::atan2(vector[1], vector[0]);
    const float effective_limit = kSteerAngleLimitRad - kSteerLimitMarginRad;

    std::array<SteerCandidate, 2> candidates{};
    candidates[0].angle = wrapPi(base_angle);
    candidates[0].flipped = false;
    candidates[0].valid = std::abs(candidates[0].angle) <= effective_limit;
    if (candidates[0].valid) {
      candidates[0].cost = steerCandidateCost(candidates[0].angle, reference);
    }

    candidates[1].angle = wrapPi(base_angle + kPi);
    candidates[1].flipped = true;
    candidates[1].valid = std::abs(candidates[1].angle) <= effective_limit;
    if (candidates[1].valid) {
      candidates[1].cost = steerCandidateCost(candidates[1].angle, reference);
    }
    return candidates;
  }

  SteeringSelection selectSteeringTargets(const Vector3 & command) const
  {
    SteeringSelection selection;
    const float effective_limit = kSteerAngleLimitRad - kSteerLimitMarginRad;

    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
      const Vector3 vector = wheelVector(command, wheel);
      const float reference = target_memory_initialized_ ?
        commanded_steer_angle_[wheel] : current_steer_angle_[wheel];

      if (std::hypot(vector[0], vector[1]) < kWheelDirectionUndefinedMps) {
        selection.target_angle[wheel] = reference;
        selection.flipped[wheel] =
          branch_memory_initialized_ ? previous_flipped_[wheel] : false;
        selection.maximum_feedback_error = std::max(
          selection.maximum_feedback_error,
          std::abs(angleDiff(reference, current_steer_angle_[wheel])));
        continue;
      }

      const auto candidates = wheelCandidates(command, wheel, reference);
      int selected_index = -1;
      if (candidates[0].valid || candidates[1].valid) {
        selected_index =
          (!candidates[1].valid ||
          (candidates[0].valid && candidates[0].cost <= candidates[1].cost)) ?
          0 : 1;
      }

      if (branch_memory_initialized_) {
        const int previous_index = previous_flipped_[wheel] ? 1 : 0;
        const int alternative_index = 1 - previous_index;
        if (candidates[previous_index].valid) {
          const bool previous_near_limit =
            std::abs(candidates[previous_index].angle) >
            effective_limit - kSteerLimitGuardRad;
          const bool alternative_clearly_better =
            candidates[alternative_index].valid &&
            candidates[alternative_index].cost + kBranchSwitchHysteresisRad <
            candidates[previous_index].cost;

          if (!previous_near_limit && !alternative_clearly_better) {
            selected_index = previous_index;
          } else if (candidates[alternative_index].valid) {
            selected_index = alternative_index;
          }
        }
      }

      if (selected_index < 0) {
        return selection;
      }
      selection.target_angle[wheel] = candidates[selected_index].angle;
      selection.flipped[wheel] = candidates[selected_index].flipped;
      selection.maximum_feedback_error = std::max(
        selection.maximum_feedback_error,
        std::abs(
          angleDiff(
            selection.target_angle[wheel], current_steer_angle_[wheel])));
    }

    selection.success = true;
    return selection;
  }

  bool selectionRequiresStop(const SteeringSelection & selection) const
  {
    if (!branch_memory_initialized_ || !target_memory_initialized_) {
      return false;
    }
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
      const float target_jump = std::abs(
        angleDiff(selection.target_angle[wheel], commanded_steer_angle_[wheel]));
      // 真实换支通常接近 pi；突发横移等同样可能让同一分支瞬间改变大角度。
      // 两类情况都先协调减速，连续曲线中每周期的小角度变化不会进入该状态。
      if (target_jump >= kBranchSwitchMinimumTargetJumpRad) {
        return true;
      }
    }
    return false;
  }

  void rememberSelection(const SteeringSelection & selection)
  {
    previous_flipped_ = selection.flipped;
    branch_memory_initialized_ = true;
  }

  void resetGovernor()
  {
    governed_twist_.fill(0.0f);
    governor_initialized_ = false;
  }

  float limitVelocity(
    float target, float current, float max_accel, float max_brake) const
  {
    const bool crossing_zero = target * current < 0.0f;
    const bool reducing_magnitude = std::abs(target) < std::abs(current);
    const float limit = (crossing_zero || reducing_magnitude) ?
      max_brake : max_accel;
    return approachValue(target, current, std::max(limit, 0.0f) * kControlPeriodS);
  }

  Vector3 governCommand(const Vector3 & reference)
  {
    const Vector3 bounded_reference{
      clampf(reference[0], -kMaxBodySpeedX, kMaxBodySpeedX),
      clampf(reference[1], -kMaxBodySpeedY, kMaxBodySpeedY),
      clampf(reference[2], -kMaxBodyYawRate, kMaxBodyYawRate)};

    if (!governor_initialized_) {
      governed_twist_.fill(0.0f);
      governor_initialized_ = true;
    }
    governed_twist_[0] = limitVelocity(
      bounded_reference[0], governed_twist_[0], kMaxAccelX, kMaxBrakeX);
    governed_twist_[1] = limitVelocity(
      bounded_reference[1], governed_twist_[1], kMaxAccelY, kMaxBrakeY);
    governed_twist_[2] = limitVelocity(
      bounded_reference[2], governed_twist_[2], kMaxAccelYaw, kMaxBrakeYaw);
    return governed_twist_;
  }

  void applyRotationDominantMode(Vector3 & command)
  {
    const float linear = std::hypot(command[0], command[1]);
    const float yaw = std::abs(command[2]);
    if (!rotation_dominant_mode_) {
      rotation_dominant_mode_ =
        yaw >= kRotationModeEnterYawRadps &&
        linear <= kRotationModeEnterLinearMps;
    } else if (
      yaw <= kRotationModeExitYawRadps ||
      linear >= kRotationModeExitLinearMps)
    {
      rotation_dominant_mode_ = false;
    }
    if (rotation_dominant_mode_) {
      command[0] = 0.0f;
      command[1] = 0.0f;
    }
  }

  float nextReleaseScale()
  {
    if (post_alignment_drive_ramp_cycle_ >= kPostAlignmentDriveRampCycles) {
      return 1.0f;
    }
    ++post_alignment_drive_ramp_cycle_;
    return static_cast<float>(post_alignment_drive_ramp_cycle_) /
           static_cast<float>(kPostAlignmentDriveRampCycles);
  }

  AllocationResult solveLightweightQp(
    const Vector3 & command, const SteeringSelection & selection)
  {
    const auto begin = std::chrono::steady_clock::now();
    AllocationResult result;

    Matrix3 hessian{};
    Vector3 rhs{};
    const Vector3 tracking_weight{
      kTrackWeightX, kTrackWeightY, kTrackWeightYaw};
    for (std::size_t dimension = 0;
      dimension < kTwistDimension; ++dimension)
    {
      hessian[dimension][dimension] =
        tracking_weight[dimension] + kRegularization;
      rhs[dimension] = tracking_weight[dimension] * command[dimension];
    }

    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
      const Vector3 lateral =
        wheelLateralRow(wheel, current_steer_angle_[wheel]);
      addOuterProduct(hessian, lateral, kLateralSlipWeight);
    }

    Vector3 unconstrained{};
    if (!solvePositiveDefinite(hessian, rhs, unconstrained)) {
      return result;
    }

    float feasibility_scale = 1.0f;
    applySymmetricBound(unconstrained[0], kMaxBodySpeedX, feasibility_scale);
    applySymmetricBound(unconstrained[1], kMaxBodySpeedY, feasibility_scale);
    applySymmetricBound(unconstrained[2], kMaxBodyYawRate, feasibility_scale);

    float maximum_lateral_before_scale = 0.0f;
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
      const Vector3 longitudinal =
        wheelLongitudinalRow(wheel, current_steer_angle_[wheel]);
      const Vector3 lateral =
        wheelLateralRow(wheel, current_steer_angle_[wheel]);
      const float longitudinal_speed = dot(longitudinal, unconstrained);
      const float lateral_speed = dot(lateral, unconstrained);
      applySymmetricBound(
        longitudinal_speed, kMaxWheelLinearSpeed, feasibility_scale);
      applySymmetricBound(
        lateral_speed, kMaxWheelLateralSpeed, feasibility_scale);
      maximum_lateral_before_scale = std::max(
        maximum_lateral_before_scale, std::abs(lateral_speed));
    }

    result.maximum_steering_error = selection.maximum_feedback_error;
    result.feasibility_scale = clampf(feasibility_scale, 0.0f, 1.0f);
    result.alignment_scale =
      scaleFromSteeringError(selection.maximum_feedback_error);
    result.release_scale = nextReleaseScale();
    result.total_scale =
      result.feasibility_scale * result.alignment_scale * result.release_scale;
    for (std::size_t dimension = 0;
      dimension < kTwistDimension; ++dimension)
    {
      result.twist[dimension] =
        unconstrained[dimension] * result.total_scale;
    }
    result.maximum_lateral_speed =
      maximum_lateral_before_scale * result.total_scale;
    result.solve_time_us = std::chrono::duration<double, std::micro>(
      std::chrono::steady_clock::now() - begin).count();
    result.success = finiteVector(result.twist);
    return result;
  }

  void holdCurrentSteering(
    std::array<float, kWheelCount> & output_angle,
    std::array<float, kWheelCount> & output_rpm)
  {
    output_angle = target_memory_initialized_ ?
      commanded_steer_angle_ : current_steer_angle_;
    output_rpm.fill(0.0f);
    previous_output_rpm_.fill(0.0f);
  }

  void beginBrake(const SteeringSelection & selection)
  {
    pending_steer_angle_ = selection.target_angle;
    pending_flipped_ = selection.flipped;
    brake_start_rpm_ = previous_output_rpm_;
    brake_cycle_ = 0;
    stopped_stable_cycle_ = 0;
    align_stable_cycle_ = 0;

    float maximum_commanded_rpm = 0.0f;
    for (const float rpm : previous_output_rpm_) {
      maximum_commanded_rpm = std::max(maximum_commanded_rpm, std::abs(rpm));
    }
    const bool actual_stop_known =
      !drive_feedback_received_ ||
      (driveFeedbackFresh() && maximumDriveFeedbackRpm() <= kStoppedRpm);
    if (maximum_commanded_rpm <= kStoppedRpm && actual_stop_known) {
      control_state_ = ControlState::kAlign;
      RCLCPP_INFO(
        get_logger(),
        "LIGHT_QP large steering change while stopped: ALIGN start");
      return;
    }

    control_state_ = ControlState::kBrake;
    RCLCPP_INFO(
      get_logger(),
      "LIGHT_QP coordinated steering change: BRAKE start, "
      "target=[%.2f, %.2f, %.2f, %.2f]",
      pending_steer_angle_[0], pending_steer_angle_[1],
      pending_steer_angle_[2], pending_steer_angle_[3]);
  }

  void tickBrake(
    std::array<float, kWheelCount> & output_angle,
    std::array<float, kWheelCount> & output_rpm)
  {
    output_angle = target_memory_initialized_ ?
      commanded_steer_angle_ : current_steer_angle_;
    ++brake_cycle_;
    const float scale = clampf(
      1.0f - static_cast<float>(brake_cycle_) /
      static_cast<float>(kBranchBrakeCycles),
      0.0f, 1.0f);
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
      output_rpm[wheel] = brake_start_rpm_[wheel] * scale;
    }
    previous_output_rpm_ = output_rpm;

    if (brake_cycle_ < kBranchBrakeCycles) {
      return;
    }
    output_rpm.fill(0.0f);
    previous_output_rpm_.fill(0.0f);

    bool stopped = false;
    if (!drive_feedback_received_) {
      // 某些底盘固件不回传驱动轮速度；已连续下发零速后使用固定周期兜底。
      stopped = true;
    } else if (driveFeedbackFresh()) {
      stopped = maximumDriveFeedbackRpm() <= kStoppedRpm;
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "LIGHT_QP drive feedback stale during branch switch; holding zero speed");
    }

    stopped_stable_cycle_ = stopped ? stopped_stable_cycle_ + 1 : 0;
    if (stopped_stable_cycle_ >= kStoppedStableCycles) {
      control_state_ = ControlState::kAlign;
      align_stable_cycle_ = 0;
      RCLCPP_INFO(
        get_logger(), "LIGHT_QP branch switch: vehicle stopped, ALIGN start");
    }
  }

  void completeAlignment()
  {
    commanded_steer_angle_ = pending_steer_angle_;
    previous_flipped_ = pending_flipped_;
    target_memory_initialized_ = true;
    branch_memory_initialized_ = true;
    previous_output_rpm_.fill(0.0f);
    post_alignment_drive_ramp_cycle_ = 0;
    resetGovernor();
    control_state_ = ControlState::kRun;
  }

  void tickAlignment(
    bool unpark,
    std::array<float, kWheelCount> & output_angle,
    std::array<float, kWheelCount> & output_rpm)
  {
    output_angle = pending_steer_angle_;
    output_rpm.fill(0.0f);
    previous_output_rpm_.fill(0.0f);

    if (!steeringFeedbackFresh()) {
      align_stable_cycle_ = 0;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "LIGHT_QP steering feedback stale during alignment; holding zero speed");
      return;
    }

    float maximum_error = 0.0f;
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
      maximum_error = std::max(
        maximum_error,
        std::abs(
          angleDiff(pending_steer_angle_[wheel], current_steer_angle_[wheel])));
    }
    align_stable_cycle_ =
      maximum_error <= kSteerAlignToleranceRad ?
      align_stable_cycle_ + 1 : 0;
    if (align_stable_cycle_ >= kSteerAlignStableCycles) {
      completeAlignment();
      RCLCPP_INFO(
        get_logger(), "LIGHT_QP %s alignment completed; drive ramp released",
        unpark ? "unpark" : "branch");
    }
  }

  void tickTransition(
    std::array<float, kWheelCount> & output_angle,
    std::array<float, kWheelCount> & output_rpm)
  {
    switch (control_state_) {
      case ControlState::kBrake:
        tickBrake(output_angle, output_rpm);
        return;
      case ControlState::kAlign:
        tickAlignment(false, output_angle, output_rpm);
        return;
      case ControlState::kUnpark:
        tickAlignment(true, output_angle, output_rpm);
        return;
      case ControlState::kRun:
        return;
    }
  }

  void cancelTransition()
  {
    control_state_ = ControlState::kRun;
    brake_cycle_ = 0;
    stopped_stable_cycle_ = 0;
    align_stable_cycle_ = 0;
    previous_output_rpm_.fill(0.0f);
  }

  void beginUnparkAlignment(const Vector3 & reference)
  {
    const SteeringSelection selection = selectSteeringTargets(reference);
    if (!selection.success) {
      RCLCPP_ERROR(
        get_logger(),
        "[SteeringAngleControl] cannot select unpark steering targets; "
        "holding current steering");
      cancelTransition();
      return;
    }
    pending_steer_angle_ = selection.target_angle;
    pending_flipped_ = selection.flipped;
    align_stable_cycle_ = 0;
    control_state_ = ControlState::kUnpark;
    resetGovernor();
    previous_output_rpm_.fill(0.0f);
    RCLCPP_INFO(
      get_logger(),
      "[SteeringAngleControl] unpark ALIGN targets=[%.2f, %.2f, %.2f, %.2f]",
      pending_steer_angle_[0], pending_steer_angle_[1],
      pending_steer_angle_[2], pending_steer_angle_[3]);
  }

  void computeRunOutput(
    const Vector3 & command,
    std::array<float, kWheelCount> & output_angle,
    std::array<float, kWheelCount> & output_rpm)
  {
    const bool command_is_zero =
      std::hypot(command[0], command[1]) < kLinearCommandDeadbandMps &&
      std::abs(command[2]) < kYawCommandDeadbandRadps;
    if (command_is_zero) {
      holdCurrentSteering(output_angle, output_rpm);
      last_allocation_ = AllocationResult{};
      return;
    }

    if (!target_memory_initialized_) {
      commanded_steer_angle_ = current_steer_angle_;
      target_memory_initialized_ = true;
    }

    const SteeringSelection selection = selectSteeringTargets(command);
    if (!selection.success) {
      holdCurrentSteering(output_angle, output_rpm);
      last_allocation_ = AllocationResult{};
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "LIGHT_QP steering target selection failed; holding zero speed");
      return;
    }

    if (selectionRequiresStop(selection)) {
      beginBrake(selection);
      tickTransition(output_angle, output_rpm);
      last_allocation_ = AllocationResult{};
      return;
    }

    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
      output_angle[wheel] = clampf(
        approachAngle(
          selection.target_angle[wheel], commanded_steer_angle_[wheel],
          kMaxSteerTargetStepRad),
        -(kSteerAngleLimitRad - kSteerLimitMarginRad),
        kSteerAngleLimitRad - kSteerLimitMarginRad);
    }

    const AllocationResult allocation =
      solveLightweightQp(command, selection);
    last_allocation_ = allocation;
    if (!allocation.success) {
      holdCurrentSteering(output_angle, output_rpm);
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "LIGHT_QP 3x3 solve failed; holding zero speed");
      return;
    }

    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
      const Vector3 longitudinal =
        wheelLongitudinalRow(wheel, current_steer_angle_[wheel]);
      output_rpm[wheel] = speedToRpm(dot(longitudinal, allocation.twist));
    }

    commanded_steer_angle_ = output_angle;
    previous_output_rpm_ = output_rpm;
    rememberSelection(selection);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "LIGHT_QP cmd=(%.3f,%.3f,%.3f) alloc=(%.3f,%.3f,%.3f) "
      "steer_err=%.3f scale=%.3f lateral=%.3f solve=%.1fus",
      command[0], command[1], command[2],
      allocation.twist[0], allocation.twist[1], allocation.twist[2],
      allocation.maximum_steering_error, allocation.total_scale,
      allocation.maximum_lateral_speed, allocation.solve_time_us);
  }

  rclcpp_action::GoalResponse handleParkGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const SteeringAngleControl::Goal> goal)
  {
    if (park_active_.load()) {
      RCLCPP_WARN(
        get_logger(), "[SteeringAngleControl] reject: action already active");
      return rclcpp_action::GoalResponse::REJECT;
    }

    const float maximum_target =
      kSteerAngleLimitRad - kSteerLimitMarginRad;
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
      const float target = goal->target_steer_angles_rad[wheel];
      if (!std::isfinite(target) || std::abs(target) > maximum_target) {
        RCLCPP_WARN(
          get_logger(),
          "[SteeringAngleControl] reject wheel=%zu target=%.3f outside "
          "[%.3f, %.3f]",
          wheel, target, -maximum_target, maximum_target);
        return rclcpp_action::GoalResponse::REJECT;
      }
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleParkCancel(
    const std::shared_ptr<GoalHandleSteeringAngleControl>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleParkAccepted(
    const std::shared_ptr<GoalHandleSteeringAngleControl> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    const float timeout = goal->timeout_sec > 0.0f ?
      goal->timeout_sec : 3.0f;
    const float tolerance = goal->settle_tolerance_rad > 0.0f ?
      goal->settle_tolerance_rad : 0.08f;

    {
      std::lock_guard<std::mutex> lock(park_mutex_);
      park_targets_ = goal->target_steer_angles_rad;
      park_tolerance_ = tolerance;
      park_start_time_ = now();
      park_deadline_ =
        park_start_time_ + rclcpp::Duration::from_seconds(timeout);
      park_goal_handle_ = goal_handle;
      park_hold_active_.store(false);
      park_active_.store(true);
    }

    cancelTransition();
    resetGovernor();
    branch_memory_initialized_ = false;
    previous_output_rpm_.fill(0.0f);
    RCLCPP_INFO(
      get_logger(),
      "[SteeringAngleControl] start targets=[%.2f, %.2f, %.2f, %.2f] "
      "tol=%.3f timeout=%.2f",
      park_targets_[0], park_targets_[1], park_targets_[2], park_targets_[3],
      tolerance, timeout);
  }

  void finishPark(bool success, const std::string & message)
  {
    std::shared_ptr<GoalHandleSteeringAngleControl> goal_handle;
    rclcpp::Time start_time;
    {
      std::lock_guard<std::mutex> lock(park_mutex_);
      goal_handle = park_goal_handle_;
      start_time = park_start_time_;
      park_goal_handle_.reset();
      park_active_.store(false);
      park_hold_active_.store(success);
    }
    if (!goal_handle) {
      return;
    }

    auto result = std::make_shared<SteeringAngleControl::Result>();
    result->success = success;
    result->message = message;
    result->total_elapsed_time = now() - start_time;

    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
    } else if (success) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }

  bool tickParkMode(chassis_interfaces::msg::ChassisMotorControl & output)
  {
    std::shared_ptr<GoalHandleSteeringAngleControl> goal_handle;
    std::array<float, kWheelCount> targets{};
    float tolerance = 0.08f;
    rclcpp::Time deadline;
    {
      std::lock_guard<std::mutex> lock(park_mutex_);
      if (!park_active_.load() && !park_hold_active_.load()) {
        return false;
      }
      goal_handle = park_goal_handle_;
      targets = park_targets_;
      tolerance = park_tolerance_;
      deadline = park_deadline_;
    }

    output.position.resize(kWheelCount);
    output.speed.assign(kWheelCount, 0.0);
    float maximum_error = 0.0f;
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
      output.position[wheel] = targets[wheel];
      maximum_error = std::max(
        maximum_error,
        std::abs(angleDiff(targets[wheel], current_steer_angle_[wheel])));
    }
    output.estop = 0;
    output.neutral = 0;
    commanded_steer_angle_ = targets;
    target_memory_initialized_ = true;
    previous_output_rpm_.fill(0.0f);

    if (park_hold_active_.load() && !park_active_.load()) {
      return true;
    }
    if (goal_handle && goal_handle->is_canceling()) {
      finishPark(false, "canceled");
      return true;
    }

    auto feedback = std::make_shared<SteeringAngleControl::Feedback>();
    feedback->max_steer_error = maximum_error;
    const double time_left_seconds =
      std::max(0.0, (deadline - now()).seconds());
    feedback->time_left =
      rclcpp::Duration::from_seconds(time_left_seconds);
    if (goal_handle) {
      goal_handle->publish_feedback(feedback);
    }

    if (steeringFeedbackFresh() && maximum_error <= tolerance) {
      finishPark(true, "wheels settled in requested pose");
    } else if (now() >= deadline) {
      finishPark(false, "timeout waiting for steering settle");
    }
    return true;
  }

  void releaseParkForMotion(const Vector3 & reference)
  {
    const bool park_or_hold_active =
      park_active_.load() || park_hold_active_.load();
    if (!park_or_hold_active) {
      return;
    }

    rclcpp::Time park_start;
    {
      std::lock_guard<std::mutex> lock(park_mutex_);
      park_start = park_start_time_;
    }
    const bool grace_elapsed =
      (now() - park_start).seconds() > kParkPreemptGraceS;
    const bool meaningful_motion =
      std::hypot(reference[0], reference[1]) > kParkPreemptLinearMps ||
      std::abs(reference[2]) > kParkPreemptYawRadps;
    if (!grace_elapsed || !meaningful_motion) {
      return;
    }

    if (park_active_.load()) {
      finishPark(false, "preempted by meaningful cmd_vel");
    } else {
      park_hold_active_.store(false);
    }
    beginUnparkAlignment(reference);
  }

  void publishDebug(
    const chassis_interfaces::msg::ChassisMotorControl & output,
    bool command_timed_out)
  {
    float maximum_commanded_rpm = 0.0f;
    for (const double rpm : output.speed) {
      maximum_commanded_rpm = std::max(
        maximum_commanded_rpm, static_cast<float>(std::abs(rpm)));
    }

    // 固定字段：
    // 0=状态(0 RUN,1 BRAKE,2 ALIGN,3 UNPARK,4 PARK)
    // 1=cmd 超时，2=舵角反馈新鲜，3=驱动反馈新鲜，
    // 4=最大指令 rpm，5=最大反馈 rpm，6=最大目标舵角误差，
    // 7=可行性缩放，8=对舵缩放，9=总缩放，
    // 10=最大横向速度，11=3x3 求解耗时(us)。
    const int state = (park_active_.load() || park_hold_active_.load()) ?
      4 : static_cast<int>(control_state_);
    auto debug = std::make_unique<std_msgs::msg::Float64MultiArray>();
    debug->data = {
      static_cast<double>(state),
      command_timed_out ? 1.0 : 0.0,
      steeringFeedbackFresh() ? 1.0 : 0.0,
      driveFeedbackFresh() ? 1.0 : 0.0,
      maximum_commanded_rpm,
      maximumDriveFeedbackRpm(),
      last_allocation_.maximum_steering_error,
      last_allocation_.feasibility_scale,
      last_allocation_.alignment_scale,
      last_allocation_.total_scale,
      last_allocation_.maximum_lateral_speed,
      last_allocation_.solve_time_us
    };
    omni_debug_pub_->publish(std::move(debug));
  }

  void publishControlCallback()
  {
    geometry_msgs::msg::Twist message;
    rclcpp::Time command_time;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      message = latest_cmd_;
      command_time = last_cmd_time_;
    }

    bool command_timed_out =
      (now() - command_time).seconds() > kCommandTimeoutS;
    Vector3 reference{
      static_cast<float>(message.linear.x),
      static_cast<float>(message.linear.y),
      static_cast<float>(message.angular.z)};
    if (!finiteVector(reference)) {
      command_timed_out = true;
      reference.fill(0.0f);
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "cmd_vel contains NaN/Inf; commanding zero wheel speed");
    }

    chassis_interfaces::msg::ChassisMotorControl output;
    output.header.stamp = now();
    output.header.frame_id = "base_link";

    if (!command_timed_out) {
      releaseParkForMotion(reference);
    }
    if (tickParkMode(output)) {
      motor_control_pub_->publish(output);
      publishDebug(output, command_timed_out);
      return;
    }

    std::array<float, kWheelCount> output_angle{};
    std::array<float, kWheelCount> output_rpm{};

    if (command_timed_out) {
      rotation_dominant_mode_ = false;
      cancelTransition();
      resetGovernor();
      holdCurrentSteering(output_angle, output_rpm);
      last_allocation_ = AllocationResult{};
    } else if (!steeringFeedbackFresh()) {
      cancelTransition();
      resetGovernor();
      holdCurrentSteering(output_angle, output_rpm);
      last_allocation_ = AllocationResult{};
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "steering feedback missing/stale; holding steering and zero wheel speed");
    } else if (control_state_ != ControlState::kRun) {
      tickTransition(output_angle, output_rpm);
      last_allocation_ = AllocationResult{};
    } else {
      applyRotationDominantMode(reference);
      const Vector3 governed_command = governCommand(reference);
      computeRunOutput(governed_command, output_angle, output_rpm);
    }

    output.position.resize(kWheelCount);
    output.speed.resize(kWheelCount);
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
      output.position[wheel] = output_angle[wheel];
      output.speed[wheel] = output_rpm[wheel];
    }
    output.estop = 0;
    output.neutral = 0;
    motor_control_pub_->publish(output);
    publishDebug(output, command_timed_out);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<chassis_interfaces::msg::ChassisMotorControl>::SharedPtr
    feedback_sub_;
  rclcpp::Publisher<chassis_interfaces::msg::ChassisMotorControl>::SharedPtr
    motor_control_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr omni_debug_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp_action::Server<SteeringAngleControl>::SharedPtr
    steering_angle_control_server_;
  std::mutex park_mutex_;
  std::atomic<bool> park_active_{false};
  std::atomic<bool> park_hold_active_{false};
  std::array<float, kWheelCount> park_targets_{};
  float park_tolerance_{0.08f};
  rclcpp::Time park_deadline_;
  rclcpp::Time park_start_time_;
  std::shared_ptr<GoalHandleSteeringAngleControl> park_goal_handle_;

  std::mutex command_mutex_;
  geometry_msgs::msg::Twist latest_cmd_;
  rclcpp::Time last_cmd_time_;

  std::array<float, kWheelCount> current_steer_angle_{};
  bool steering_feedback_received_{false};
  rclcpp::Time last_steering_feedback_time_;

  std::array<float, kWheelCount> current_drive_rpm_{};
  bool drive_feedback_received_{false};
  rclcpp::Time last_drive_feedback_time_;

  std::array<float, kWheelCount> commanded_steer_angle_{};
  std::array<bool, kWheelCount> previous_flipped_{};
  bool target_memory_initialized_{false};
  bool branch_memory_initialized_{false};

  Vector3 governed_twist_{};
  bool governor_initialized_{false};
  bool rotation_dominant_mode_{false};

  ControlState control_state_{ControlState::kRun};
  std::array<float, kWheelCount> pending_steer_angle_{};
  std::array<bool, kWheelCount> pending_flipped_{};
  std::array<float, kWheelCount> brake_start_rpm_{};
  std::array<float, kWheelCount> previous_output_rpm_{};
  int brake_cycle_{0};
  int stopped_stable_cycle_{0};
  int align_stable_cycle_{0};
  int post_alignment_drive_ramp_cycle_{kPostAlignmentDriveRampCycles};

  AllocationResult last_allocation_{};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MobileOmniController>());
  rclcpp::shutdown();
  return 0;
}
