#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>

#include "chassis_interfaces/msg/chassis_motor_control.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr std::size_t kWheelCount = 4;

// =============================================================================
// 机械参数
// =============================================================================
// NEED CALIBRATION: 按轮轴中心实测尺寸填写，不要使用外壳尺寸。
constexpr float kChassisLengthM = 0.460f;
constexpr float kChassisWidthM = 0.345f;
constexpr float kWheelDiameterM = 0.107f;

// NEED CALIBRATION: 舵角机械硬限位。应比真实硬限位小，并保留安全余量。
constexpr float kSteerAngleLimitRad = 2.50f;
constexpr float kSteerLimitMarginRad = 0.08f;

constexpr float kControlPeriodS = 0.05f;  // 20 Hz
constexpr float kCommandTimeoutS = 0.50f;

// =============================================================================
// cmd_vel 变化率限制
// =============================================================================
// 这里只保留一层底盘速度变化率限制，不再叠加独立 jerk 和 RPM 低通。
// NEED CALIBRATION: 满载测试，以不打滑、不明显冲击且响应可接受为准。
constexpr bool kCommandGovernorEnabled = true;
constexpr float kMaxAccelX = 0.6f;    // m/s^2
constexpr float kMaxAccelY = 0.5f;    // m/s^2
constexpr float kMaxAccelYaw = 0.75f;  // rad/s^2
constexpr float kMaxBrakeX = 0.70f;    // m/s^2
constexpr float kMaxBrakeY = 0.55f;    // m/s^2
constexpr float kMaxBrakeYaw = 1.20f;  // rad/s^2

// =============================================================================
// DLS 参数
// =============================================================================
// lambda 越大越稳定，但越容易让复合运动变钝。
// NEED CALIBRATION: 若正常复合运动仍明显迟钝，优先减小 kDlsLambdaMax，
// 再根据日志中的 mu 调整 kDlsMu0。
constexpr float kDlsLambdaMin = 0.02f;
constexpr float kDlsLambdaMax = 0.50f;
constexpr float kDlsMu0 = 0.008f;       // m^2
constexpr float kDlsLambdaPower = 2.0f;

// 单轴指令绕过完整 3-DOF 可操作性判断，避免纯前进或纯横移被 DLS 误抑制。
constexpr float kSimpleAxisCrossSpeedMax = 0.02f;
constexpr float kSimpleAxisYawMax = 0.02f;

// =============================================================================
// 舵角候选选择
// =============================================================================
// NEED CALIBRATION: 权重越大，越不容易切换 angle / angle+pi 候选，
// 但过大会使底盘不愿采用更短的新舵向。
constexpr float kCandidateSwitchCost = 0.35f;
constexpr float kCandidateReverseCost = 0.20f;
constexpr float kLimitSoftZoneRad = 0.30f;
constexpr float kLimitCostGain = 0.25f;

// =============================================================================
// 统一舵角过渡保护
// =============================================================================
// 原代码同时使用全局门控、单轮门控、换向保持状态机和 RPM 平滑，容易重复降速。
// 本版本只保留：连续舵角误差缩放 + 全局缩放限斜率。
//
// NEED CALIBRATION:
// - kGlobalFullSpeedErrRad：小于该误差时整车不因舵角降速。
// - kGlobalCrawlErrRad：达到该误差时整车降到最小爬行比例。
// - kGlobalMinScale：大换向时整车保留的速度比例；过大可能摩地，过小会停顿。
// - kWheelFullSpeedErrRad / kWheelCrawlErrRad：单轮局部降速区间。
constexpr float kGlobalFullSpeedErrRad = 0.15f;  // 约 8.6 deg
constexpr float kGlobalCrawlErrRad = 0.90f;      // 约 51.6 deg
constexpr float kGlobalMinScale = 0.10f;

constexpr float kWheelFullSpeedErrRad = 0.10f;  // 约 5.7 deg
constexpr float kWheelCrawlErrRad = 0.75f;      // 约 43.0 deg
constexpr float kWheelMinScale = 0.04f;

// 每个控制周期允许全局驱动比例变化的最大值。
// NEED CALIBRATION: drop 越大保护介入越快；rise 越大舵角对齐后恢复越快。
constexpr float kScaleMaxDropPerCycle = 0.30f;
constexpr float kScaleMaxRisePerCycle = 0.18f;

// =============================================================================
// 低速稳定参数
// =============================================================================
// NEED CALIBRATION: 低于该值时方向信息不可靠，保持当前舵角，避免 atan2 抖动。
constexpr float kWheelVectorDeadbandMps = 0.03f;
constexpr float kLinearCommandDeadbandMps = 0.002f;
constexpr float kYawCommandDeadbandRadps = 0.003f;

float clampf(float value, float lower, float upper) {
  return std::max(lower, std::min(upper, value));
}

float wrapPi(float angle) {
  while (angle > kPi) {
    angle -= 2.0f * kPi;
  }
  while (angle <= -kPi) {
    angle += 2.0f * kPi;
  }
  return angle;
}

float angleDiff(float target, float current) {
  return wrapPi(target - current);
}

float speedToRpm(float speed_mps) {
  return speed_mps * 60.0f / (kPi * kWheelDiameterM);
}

float linearScaleFromError(float error, float full_speed_error,
                           float crawl_error, float minimum_scale) {
  if (error <= full_speed_error) {
    return 1.0f;
  }
  if (error >= crawl_error) {
    return minimum_scale;
  }

  const float ratio = (crawl_error - error) / (crawl_error - full_speed_error);
  return minimum_scale + (1.0f - minimum_scale) * clampf(ratio, 0.0f, 1.0f);
}

float limitVelocityRate(float target, float current, float max_accel,
                        float max_brake, float dt) {
  const bool crossing_zero = target * current < 0.0f;
  const bool reducing_magnitude = std::abs(target) < std::abs(current);
  const float rate = (crossing_zero || reducing_magnitude) ? max_brake : max_accel;
  const float max_delta = std::max(rate, 0.0f) * dt;
  const float delta = target - current;

  if (std::abs(delta) <= max_delta) {
    return target;
  }
  return current + std::copysign(max_delta, delta);
}

float determinant3x3(const float matrix[3][3]) {
  return matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) -
         matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) +
         matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
}

bool invert3x3(const float input[3][3], float inverse[3][3]) {
  const float determinant = determinant3x3(input);
  if (std::abs(determinant) < 1e-9f) {
    return false;
  }

  inverse[0][0] =  (input[1][1] * input[2][2] - input[1][2] * input[2][1]) / determinant;
  inverse[0][1] = -(input[0][1] * input[2][2] - input[0][2] * input[2][1]) / determinant;
  inverse[0][2] =  (input[0][1] * input[1][2] - input[0][2] * input[1][1]) / determinant;
  inverse[1][0] = -(input[1][0] * input[2][2] - input[1][2] * input[2][0]) / determinant;
  inverse[1][1] =  (input[0][0] * input[2][2] - input[0][2] * input[2][0]) / determinant;
  inverse[1][2] = -(input[0][0] * input[1][2] - input[0][2] * input[1][0]) / determinant;
  inverse[2][0] =  (input[1][0] * input[2][1] - input[1][1] * input[2][0]) / determinant;
  inverse[2][1] = -(input[0][0] * input[2][1] - input[0][1] * input[2][0]) / determinant;
  inverse[2][2] =  (input[0][0] * input[1][1] - input[0][1] * input[1][0]) / determinant;
  return true;
}

float computeDlsLambda(float manipulability) {
  const float ratio = clampf(manipulability / kDlsMu0, 0.0f, 1.0f);
  const float attenuation = std::pow(1.0f - ratio, kDlsLambdaPower);
  return kDlsLambdaMin + (kDlsLambdaMax - kDlsLambdaMin) * attenuation;
}

bool isSimpleLongitudinalCommand(float vx, float vy, float yaw_rate) {
  return std::abs(vx) > kLinearCommandDeadbandMps &&
         std::abs(vy) < kSimpleAxisCrossSpeedMax &&
         std::abs(yaw_rate) < kSimpleAxisYawMax;
}

bool isSimpleLateralCommand(float vx, float vy, float yaw_rate) {
  return std::abs(vy) > kLinearCommandDeadbandMps &&
         std::abs(vx) < kSimpleAxisCrossSpeedMax &&
         std::abs(yaw_rate) < kSimpleAxisYawMax;
}

}  // namespace

class MobileOmniController : public rclcpp::Node {
public:
  MobileOmniController() : Node("mobile_cmd_omni") {
    current_steer_angle_.fill(0.0f);
    previous_candidate_flipped_.fill(false);
    previous_drive_sign_.fill(1.0f);

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/control/cmd_vel_omni", 10,
      std::bind(&MobileOmniController::cmdVelCallback, this, std::placeholders::_1));

    feedback_sub_ = create_subscription<chassis_interfaces::msg::ChassisMotorControl>(
      "/feedback/omni_chassis_motor",
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
      std::bind(&MobileOmniController::feedbackCallback, this, std::placeholders::_1));

    motor_control_pub_ = create_publisher<chassis_interfaces::msg::ChassisMotorControl>(
      "/control/omni_chassis_motors", 10);

    last_cmd_time_ = now();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&MobileOmniController::publishControlCallback, this));

    RCLCPP_INFO(get_logger(),
      "Mobile omni controller started: DLS clean A, unified steering transition protection");
  }

private:
  struct SteerCandidate {
    float angle{0.0f};
    float drive_sign{1.0f};
    float cost{1e6f};
    bool flipped{false};
    bool valid{false};
  };

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr message) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    latest_cmd_ = *message;
    last_cmd_time_ = now();
  }

  void feedbackCallback(
      const chassis_interfaces::msg::ChassisMotorControl::SharedPtr message) {
    if (message->position.size() < kWheelCount) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Steering feedback contains %zu positions; expected at least %zu",
        message->position.size(), kWheelCount);
      return;
    }

    for (std::size_t i = 0; i < kWheelCount; ++i) {
      current_steer_angle_[i] = static_cast<float>(message->position[i]);
    }
    steering_feedback_received_ = true;
  }

  void resetCommandGovernor() {
    governed_vx_ = 0.0f;
    governed_vy_ = 0.0f;
    governed_yaw_rate_ = 0.0f;
    command_governor_initialized_ = false;
  }

  void applyCommandGovernor(float vx_ref, float vy_ref, float yaw_rate_ref,
                            float & vx_cmd, float & vy_cmd, float & yaw_rate_cmd) {
    if (!kCommandGovernorEnabled) {
      vx_cmd = vx_ref;
      vy_cmd = vy_ref;
      yaw_rate_cmd = yaw_rate_ref;
      return;
    }

    if (!command_governor_initialized_) {
      governed_vx_ = 0.0f;
      governed_vy_ = 0.0f;
      governed_yaw_rate_ = 0.0f;
      command_governor_initialized_ = true;
    }

    governed_vx_ = limitVelocityRate(
      vx_ref, governed_vx_, kMaxAccelX, kMaxBrakeX, kControlPeriodS);
    governed_vy_ = limitVelocityRate(
      vy_ref, governed_vy_, kMaxAccelY, kMaxBrakeY, kControlPeriodS);
    governed_yaw_rate_ = limitVelocityRate(
      yaw_rate_ref, governed_yaw_rate_, kMaxAccelYaw, kMaxBrakeYaw, kControlPeriodS);

    vx_cmd = governed_vx_;
    vy_cmd = governed_vy_;
    yaw_rate_cmd = governed_yaw_rate_;
  }

  float limitProximityCost(float angle) const {
    const float effective_limit =
      std::max(kSteerAngleLimitRad - kSteerLimitMarginRad, 0.1f);
    const float remaining_margin = effective_limit - std::abs(angle);

    if (remaining_margin >= kLimitSoftZoneRad) {
      return 0.0f;
    }
    if (remaining_margin <= 0.0f) {
      return 1.0f;
    }
    return 1.0f - remaining_margin / kLimitSoftZoneRad;
  }

  float candidateCost(std::size_t wheel_index,
                      const SteerCandidate & candidate) const {
    if (!candidate.valid) {
      return 1e6f;
    }

    float cost = std::abs(angleDiff(
      candidate.angle, current_steer_angle_[wheel_index]));

    if (candidate_memory_initialized_ &&
        candidate.flipped != previous_candidate_flipped_[wheel_index]) {
      cost += kCandidateSwitchCost;
    }

    if (candidate_memory_initialized_ &&
        candidate.drive_sign * previous_drive_sign_[wheel_index] < 0.0f) {
      cost += kCandidateReverseCost;
    }

    cost += kLimitCostGain * limitProximityCost(candidate.angle);
    return cost;
  }

  SteerCandidate selectSteerCandidate(std::size_t wheel_index,
                                      float raw_angle) const {
    const float effective_limit =
      std::max(kSteerAngleLimitRad - kSteerLimitMarginRad, 0.1f);

    SteerCandidate normal;
    normal.angle = wrapPi(raw_angle);
    normal.drive_sign = 1.0f;
    normal.flipped = false;
    normal.valid = std::abs(normal.angle) <= effective_limit;
    normal.cost = candidateCost(wheel_index, normal);

    SteerCandidate flipped;
    flipped.angle = wrapPi(raw_angle + kPi);
    flipped.drive_sign = -1.0f;
    flipped.flipped = true;
    flipped.valid = std::abs(flipped.angle) <= effective_limit;
    flipped.cost = candidateCost(wheel_index, flipped);

    if (normal.valid || flipped.valid) {
      return normal.cost <= flipped.cost ? normal : flipped;
    }

    // 正常配置下两个等价方向至少有一个可行。此分支仅防止参数或零位异常越界。
    SteerCandidate fallback;
    fallback.angle = clampf(normal.angle, -effective_limit, effective_limit);
    fallback.drive_sign = 1.0f;
    fallback.flipped = false;
    fallback.valid = true;
    fallback.cost = 1e5f;
    return fallback;
  }

  bool solveAchievableTwist(
      const std::array<float, kWheelCount> & steering_angle,
      const std::array<float, kWheelCount> & wheel_x_sign,
      const std::array<float, kWheelCount> & wheel_y_sign,
      const std::array<float, kWheelCount> & requested_corner_vx,
      const std::array<float, kWheelCount> & requested_corner_vy,
      float & achievable_vx, float & achievable_vy, float & achievable_yaw_rate,
      float & manipulability, float & lambda) const {
    float h[kWheelCount][3]{};
    float projected_wheel_speed[kWheelCount]{};

    for (std::size_t i = 0; i < kWheelCount; ++i) {
      const float cosine = std::cos(steering_angle[i]);
      const float sine = std::sin(steering_angle[i]);

      projected_wheel_speed[i] =
        requested_corner_vx[i] * cosine + requested_corner_vy[i] * sine;

      h[i][0] = cosine;
      h[i][1] = sine;
      h[i][2] = cosine * wheel_x_sign[i] * kChassisWidthM * 0.5f +
                sine * wheel_y_sign[i] * kChassisLengthM * 0.5f;
    }

    float h_transpose_h[3][3]{};
    float h_transpose_speed[3]{};

    for (std::size_t i = 0; i < kWheelCount; ++i) {
      for (int row = 0; row < 3; ++row) {
        h_transpose_speed[row] += h[i][row] * projected_wheel_speed[i];
        for (int column = 0; column < 3; ++column) {
          h_transpose_h[row][column] += h[i][row] * h[i][column];
        }
      }
    }

    manipulability = std::sqrt(std::max(determinant3x3(h_transpose_h), 0.0f));
    lambda = computeDlsLambda(manipulability);

    float regularized_matrix[3][3]{};
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        regularized_matrix[row][column] = h_transpose_h[row][column];
      }
      regularized_matrix[row][row] += lambda * lambda;
    }

    float inverse[3][3]{};
    if (!invert3x3(regularized_matrix, inverse)) {
      achievable_vx = 0.0f;
      achievable_vy = 0.0f;
      achievable_yaw_rate = 0.0f;
      return false;
    }

    achievable_vx = inverse[0][0] * h_transpose_speed[0] +
                    inverse[0][1] * h_transpose_speed[1] +
                    inverse[0][2] * h_transpose_speed[2];
    achievable_vy = inverse[1][0] * h_transpose_speed[0] +
                    inverse[1][1] * h_transpose_speed[1] +
                    inverse[1][2] * h_transpose_speed[2];
    achievable_yaw_rate = inverse[2][0] * h_transpose_speed[0] +
                          inverse[2][1] * h_transpose_speed[1] +
                          inverse[2][2] * h_transpose_speed[2];
    return true;
  }

  float filterGlobalDriveScale(float requested_scale) {
    requested_scale = clampf(requested_scale, 0.0f, 1.0f);

    if (!drive_scale_filter_initialized_) {
      filtered_global_drive_scale_ = requested_scale;
      drive_scale_filter_initialized_ = true;
      return filtered_global_drive_scale_;
    }

    const float delta = requested_scale - filtered_global_drive_scale_;
    if (delta > kScaleMaxRisePerCycle) {
      filtered_global_drive_scale_ += kScaleMaxRisePerCycle;
    } else if (delta < -kScaleMaxDropPerCycle) {
      filtered_global_drive_scale_ -= kScaleMaxDropPerCycle;
    } else {
      filtered_global_drive_scale_ = requested_scale;
    }

    filtered_global_drive_scale_ =
      clampf(filtered_global_drive_scale_, 0.0f, 1.0f);
    return filtered_global_drive_scale_;
  }

  void computeWheels(float vx, float vy, float yaw_rate,
                     std::array<float, kWheelCount> & output_angle,
                     std::array<float, kWheelCount> & output_rpm) {
    const bool command_is_zero =
      std::hypot(vx, vy) < kLinearCommandDeadbandMps &&
      std::abs(yaw_rate) < kYawCommandDeadbandRadps;

    if (command_is_zero) {
      output_angle = current_steer_angle_;
      output_rpm.fill(0.0f);
      drive_scale_filter_initialized_ = false;
      return;
    }

    std::array<float, kWheelCount> target_angle{};
    std::array<float, kWheelCount> requested_corner_vx{};
    std::array<float, kWheelCount> requested_corner_vy{};
    std::array<float, kWheelCount> wheel_x_sign{};
    std::array<float, kWheelCount> wheel_y_sign{};
    std::array<float, kWheelCount> steering_error{};
    std::array<bool, kWheelCount> low_vector_hold{};
    std::array<bool, kWheelCount> selected_flipped{};
    std::array<float, kWheelCount> selected_drive_sign{};

    for (std::size_t i = 0; i < kWheelCount; ++i) {
      const bool is_right = i == 1 || i == 3;
      const bool is_front = i == 0 || i == 1;

      wheel_x_sign[i] = is_right ? 1.0f : -1.0f;
      wheel_y_sign[i] = is_front ? 1.0f : -1.0f;

      requested_corner_vx[i] =
        vx + wheel_x_sign[i] * kChassisWidthM * yaw_rate * 0.5f;
      requested_corner_vy[i] =
        vy + wheel_y_sign[i] * kChassisLengthM * yaw_rate * 0.5f;

      const float corner_speed =
        std::hypot(requested_corner_vx[i], requested_corner_vy[i]);

      if (corner_speed < kWheelVectorDeadbandMps) {
        target_angle[i] = current_steer_angle_[i];
        requested_corner_vx[i] = 0.0f;
        requested_corner_vy[i] = 0.0f;
        low_vector_hold[i] = true;
        selected_drive_sign[i] = previous_drive_sign_[i];
        selected_flipped[i] = previous_candidate_flipped_[i];
        continue;
      }

      const SteerCandidate selected = selectSteerCandidate(
        i, std::atan2(requested_corner_vy[i], requested_corner_vx[i]));
      target_angle[i] = selected.angle;
      selected_drive_sign[i] = selected.drive_sign;
      selected_flipped[i] = selected.flipped;
    }

    candidate_memory_initialized_ = true;
    for (std::size_t i = 0; i < kWheelCount; ++i) {
      if (!low_vector_hold[i]) {
        previous_candidate_flipped_[i] = selected_flipped[i];
        previous_drive_sign_[i] = selected_drive_sign[i];
      }
    }

    float maximum_steering_error = 0.0f;
    for (std::size_t i = 0; i < kWheelCount; ++i) {
      steering_error[i] = std::abs(angleDiff(target_angle[i], current_steer_angle_[i]));
      maximum_steering_error = std::max(maximum_steering_error, steering_error[i]);
    }

    const float requested_global_scale = linearScaleFromError(
      maximum_steering_error,
      kGlobalFullSpeedErrRad,
      kGlobalCrawlErrRad,
      kGlobalMinScale);
    const float global_scale = filterGlobalDriveScale(requested_global_scale);

    float achievable_vx = vx;
    float achievable_vy = vy;
    float achievable_yaw_rate = yaw_rate;
    float manipulability = 0.0f;
    float lambda = kDlsLambdaMin;
    bool dls_bypassed = false;

    if (isSimpleLongitudinalCommand(vx, vy, yaw_rate) ||
        isSimpleLateralCommand(vx, vy, yaw_rate)) {
      dls_bypassed = true;
    } else if (!solveAchievableTwist(
                 current_steer_angle_, wheel_x_sign, wheel_y_sign,
                 requested_corner_vx, requested_corner_vy,
                 achievable_vx, achievable_vy, achievable_yaw_rate,
                 manipulability, lambda)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "DLS solve failed; holding steering and commanding zero wheel speed");
      output_angle = current_steer_angle_;
      output_rpm.fill(0.0f);
      return;
    }

    for (std::size_t i = 0; i < kWheelCount; ++i) {
      output_angle[i] = target_angle[i];

      const float actual_angle = current_steer_angle_[i];
      const float achievable_corner_vx =
        achievable_vx + wheel_x_sign[i] * kChassisWidthM * achievable_yaw_rate * 0.5f;
      const float achievable_corner_vy =
        achievable_vy + wheel_y_sign[i] * kChassisLengthM * achievable_yaw_rate * 0.5f;
      const float wheel_speed =
        achievable_corner_vx * std::cos(actual_angle) +
        achievable_corner_vy * std::sin(actual_angle);

      const float local_scale = linearScaleFromError(
        steering_error[i],
        kWheelFullSpeedErrRad,
        kWheelCrawlErrRad,
        kWheelMinScale);

      // 使用 min 而不是 global * local，避免两层保护相乘造成重复衰减。
      const float final_scale = std::min(global_scale, local_scale);
      output_rpm[i] = speedToRpm(wheel_speed) * final_scale;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "OMNI_A: cmd=(%.3f,%.3f,%.3f) ach=(%.3f,%.3f,%.3f) "
      "mu=%.5f lambda=%.3f dls_bypass=%d steer_err_max=%.3f scale=%.2f",
      vx, vy, yaw_rate,
      achievable_vx, achievable_vy, achievable_yaw_rate,
      manipulability, lambda, dls_bypassed ? 1 : 0,
      maximum_steering_error, global_scale);
  }

  void publishControlCallback() {
    geometry_msgs::msg::Twist command;
    rclcpp::Time command_time;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      command = latest_cmd_;
      command_time = last_cmd_time_;
    }

    const bool command_timed_out =
      (now() - command_time).seconds() > kCommandTimeoutS;

    float vx = 0.0f;
    float vy = 0.0f;
    float yaw_rate = 0.0f;

    if (command_timed_out) {
      resetCommandGovernor();
    } else {
      applyCommandGovernor(
        static_cast<float>(command.linear.x),
        static_cast<float>(command.linear.y),
        static_cast<float>(command.angular.z),
        vx, vy, yaw_rate);
    }

    std::array<float, kWheelCount> wheel_angle{};
    std::array<float, kWheelCount> wheel_rpm{};

    if (!steering_feedback_received_) {
      wheel_angle.fill(0.0f);
      wheel_rpm.fill(0.0f);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for steering feedback; wheel speed command is held at zero");
    } else {
      computeWheels(vx, vy, yaw_rate, wheel_angle, wheel_rpm);
    }

    chassis_interfaces::msg::ChassisMotorControl output;
    output.header.stamp = now();
    output.header.frame_id = "base_link";
    output.position.resize(kWheelCount);
    output.speed.resize(kWheelCount);

    for (std::size_t i = 0; i < kWheelCount; ++i) {
      output.position[i] = wheel_angle[i];
      output.speed[i] = wheel_rpm[i];
    }

    output.estop = 0;
    output.neutral = 0;
    motor_control_pub_->publish(output);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<chassis_interfaces::msg::ChassisMotorControl>::SharedPtr feedback_sub_;
  rclcpp::Publisher<chassis_interfaces::msg::ChassisMotorControl>::SharedPtr motor_control_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex command_mutex_;
  geometry_msgs::msg::Twist latest_cmd_;
  rclcpp::Time last_cmd_time_;

  std::array<float, kWheelCount> current_steer_angle_{};
  bool steering_feedback_received_{false};

  float governed_vx_{0.0f};
  float governed_vy_{0.0f};
  float governed_yaw_rate_{0.0f};
  bool command_governor_initialized_{false};

  std::array<bool, kWheelCount> previous_candidate_flipped_{};
  std::array<float, kWheelCount> previous_drive_sign_{};
  bool candidate_memory_initialized_{false};

  float filtered_global_drive_scale_{1.0f};
  bool drive_scale_filter_initialized_{false};
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MobileOmniController>());
  rclcpp::shutdown();
  return 0;
}
