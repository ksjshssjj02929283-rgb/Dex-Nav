// B 方案：连续舵角选择 + 单次 QP Swerve Allocator
//
// 核心原则：
// 1. 每轮先用上一周期目标角选择 angle / angle+pi 连续分支，再统一求一次 QP；
// 2. 分支接近软限位、必须跨越大角度时，整车先减速到零，再停车摆舵；
// 3. 原地旋转占主导时抑制很小的 vx/vy，避免 atan2 目标方向反复摆动；
// 4. QP 统一处理速度跟踪、连续性、侧向拖拽惩罚、轮速与车体速度约束。
//
// 不依赖 OSQP、qpOASES 等外部库；三变量 QP 使用主动集穷举求解。

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
#include <vector>

#include "chassis_interfaces/msg/chassis_motor_control.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "mobile_joy_control/action/park_wheels.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace
{

constexpr float kPi = 3.14159265358979323846f;
constexpr std::size_t kWheelCount = 4;
constexpr std::size_t kTwistDimension = 3;

using Vector3 = std::array<float, kTwistDimension>;
using Matrix3 = std::array<std::array<float, kTwistDimension>, kTwistDimension>;

// =============================================================================
// 机械参数
// =============================================================================
// NEED CALIBRATION: 按轮轴中心实测，不要使用车体外壳尺寸。
constexpr float kChassisLengthM = 0.460f;
constexpr float kChassisWidthM = 0.345f;
constexpr float kWheelDiameterM = 0.107f;

// NEED CALIBRATION: 软限位应小于真实机械硬限位，并预留安装与零位误差。
constexpr float kSteerAngleLimitRad = 2.50f;
constexpr float kSteerLimitMarginRad = 0.08f;

constexpr float kControlPeriodS = 0.05f;  // 20 Hz
constexpr float kCommandTimeoutS = 0.50f;

// 导航刚结束时 velocity_smoother 仍可能短暂发布减速尾巴，不能把它误判为新任务
// 而立即取消 X 驻车。超过保护时间且速度明显非零时，才允许遥控/新导航抢占。
// 保护时间覆盖 velocity_smoother 的正常减速尾巴；保护期后按“收到的目标指令”
// 判断新任务，而不是按尚未爬升的治理后速度判断。起点原地对齐最低约 0.08 rad/s，
// 因此解除阈值必须明显低于它，否则 X 驻车与起点对齐会互相等待形成死锁。
constexpr float kParkPreemptGraceS = 0.80f;
constexpr float kParkPreemptLinearMps = 0.04f;
constexpr float kParkPreemptYawRadps = 0.04f;

// X 驻车解除后先零轮速快速对舵，再从零起步。对舵目标直接发送最终角度，
// 不经过普通行驶的每周期舵角目标限幅；实际转速仍由底层舵机驱动器限制。
constexpr float kUnparkAlignToleranceRad = 0.08f;
constexpr int kUnparkAlignStableCycles = 3;
constexpr float kUnparkRetargetAngleRad = 0.35f;
constexpr float kUnparkRetargetConsistencyRad = 0.10f;
constexpr int kUnparkRetargetStableCycles = 3;
constexpr float kSteeringFeedbackTimeoutS = 0.25f;
constexpr float kDriveFeedbackTimeoutS = 0.25f;

// =============================================================================
// 最终速度的加速度与 jerk 限制
// =============================================================================
// QP 负责可实现性与轮胎侧向拖拽权衡；这里限制最终交给 QP 的速度变化过程。
// NEED CALIBRATION: 满载测试，以不打滑、不过度冲击且响应可接受为准。
constexpr bool kCommandGovernorEnabled = true;
constexpr float kMaxAccelX = 0.60f;    // m/s^2
constexpr float kMaxAccelY = 0.50f;    // m/s^2
constexpr float kMaxAccelYaw = 0.70f;  // rad/s^2
constexpr float kMaxBrakeX = 0.60f;    // m/s^2
constexpr float kMaxBrakeY = 0.50f;    // m/s^2
constexpr float kMaxBrakeYaw = 0.70f;  // rad/s^2

// 最终下发电机前必须再限制一次 jerk：导航经过 velocity_smoother 后未必仍
// 保持 MPPI 的 jerk 约束，遥控指令也会绕过 MPPI。这里的阈值略高于 MPPI，
// 让 MPPI 仍负责正常导航轨迹，只拦截上游阶跃、快速松杆和模式切换冲击。
// 20 Hz 下每周期最大加速度变化分别约为 0.06、0.05、0.09。
constexpr float kMaxJerkX = 1.20f;    // m/s^3
constexpr float kMaxJerkY = 1.00f;    // m/s^3
constexpr float kMaxJerkYaw = 1.80f;  // rad/s^3

// =============================================================================
// QP 硬约束
// =============================================================================
// NEED CALIBRATION: 按底盘、电机和减速器真实能力填写。
constexpr float kMaxBodySpeedX = 1.20f;       // m/s
constexpr float kMaxBodySpeedY = 1.20f;       // m/s
constexpr float kMaxBodyYawRate = 0.75f;      // rad/s
constexpr float kMaxWheelLinearSpeed = 1.50f; // m/s

// 舵角误差越大，当前轮向可安全输出的纵向轮速上限越低。
// QP 内保留硬约束，QP 后再按四轮最大“最终目标角误差”统一缩放轮速，
// 防止低速指令小于 QP 上限时限速实际不生效。
// NEED CALIBRATION:
// - full：误差小于该值时不限制轮速；
// - stop：误差达到该值时四轮驱动严格为零，不再保留爬行速度。
constexpr float kAlignFullSpeedErrorRad = 0.08f;  // 约 4.6 度
constexpr float kAlignStopErrorRad = 0.35f;       // 约 20.1 度
constexpr float kAlignMinimumSpeedScale = 0.0f;

// =============================================================================
// QP 目标函数权重
// =============================================================================
// 优化变量 x = [vx, vy, yaw_rate]。
// 目标函数同时考虑：命令跟踪、上一周期连续性、轮胎侧向速度、轮速跟踪和正则化。
//
// NEED CALIBRATION:
// 1) 底盘迟钝：优先增大 tracking，或减小 smooth / lateral-slip；
// 2) 换向摩地：优先增大 lateral-slip，或减小 align minimum scale；
// 3) 速度跳变：增大 smooth，但过大会拖慢响应；
// 4) yaw 跟踪弱：单独增大 kQpTrackYaw。
constexpr float kQpTrackX = 18.0f;
constexpr float kQpTrackY = 18.0f;
constexpr float kQpTrackYaw = 12.0f;

constexpr float kQpSmoothX = 0.35f;
constexpr float kQpSmoothY = 0.35f;
constexpr float kQpSmoothYaw = 0.25f;

constexpr float kQpLateralSlipWeight = 8.0f;
constexpr float kQpWheelSpeedTrackWeight = 0.60f;
constexpr float kQpRegularization = 0.01f;

// =============================================================================
// 舵角连续性与整车协调换向
// =============================================================================
// 在真实软限位内再保留一段预警区。当前分支进入预警区时，优先选择另一套
// 等价轮向，不等到越过 2.42 rad 后才突然跳变。
constexpr float kSteerLimitGuardRad = 0.12f;
constexpr float kLimitSoftZoneRad = 0.30f;
constexpr float kLimitCostGain = 0.25f;

// 普通小角度调整每 50 ms 最多变化约 6.9 度；但最终目标角与真实反馈角
// 相差超过约 20 度时，进入整车协调停车换向，不能用单周期临时目标掩盖误差。
constexpr float kMaxSteerTargetStepRad = 0.12f;
constexpr float kCoordinatedSwitchTriggerRad = 0.35f;
constexpr int kCoordinatedDecelerationCycles = 5;  // 约 0.25 s 整车减到零
constexpr float kCoordinatedStoppedRpm = 3.0f;
constexpr int kCoordinatedStoppedStableCycles = 3;
constexpr float kCoordinatedAlignToleranceRad = 0.10f;
constexpr int kCoordinatedAlignStableCycles = 3;
// 对舵期间 S 曲线仍在跟踪上游目标，完成后保留其稳定方向；轮速单独用统一比例
// 从零释放，避免重置 S 曲线后瞬时方向失真并立即触发第二次停车换向。
constexpr int kPostAlignmentDriveRampCycles = 5;  // 20 Hz 下约 0.25 s
// MPPI 的 vx/vy/wz 每周期都会小幅变化，直接用 0.02 rad 更新舵角目标会使
// “连续到位”计数长期归零。协调换向期间锁存当前目标；只有新方向相差超过
// 约 20 度，并连续 3 个周期保持在约 6 度以内，才确认用户/规划器确实改向。
constexpr float kCoordinatedRetargetAngleRad = 0.35f;
constexpr float kCoordinatedRetargetConsistencyRad = 0.10f;
constexpr int kCoordinatedRetargetStableCycles = 3;

// 原地旋转模式只允许吞掉接近数值噪声的 vx/vy。旧阈值 0.10 m/s 会把终点阶段
// “仍需平移定位，同时缓慢调 yaw”的有效全向指令误判为纯旋转，导致机器人提前
// 停车并把四轮摆成旋转阵型。起点对齐和 final_yaw 的 vx/vy 本来就是严格零，
// 因此收紧线速度阈值不会影响这两个真正需要原地旋转的阶段。
constexpr float kRotationModeEnterYawRadps = 0.15f;
constexpr float kRotationModeExitYawRadps = 0.10f;
constexpr float kRotationModeEnterLinearMps = 0.02f;
constexpr float kRotationModeExitLinearMps = 0.04f;

// =============================================================================
// 低速稳定与求解器参数
// =============================================================================
// 单轮合成速度只有真正接近数学零点时，方向 atan2 才没有意义。
// 这里不能使用原来的 0.03 m/s 普通速度死区：低速原地旋转时每个轮子的
// 线速度本来就小，原阈值会把正确的 ±53° 舵角吞掉，却仍然输出差速轮速，
// 造成四轮保持 0° 时硬拧车体。1e-3 只过滤数值噪声和真正的瞬时旋转中心轮。
constexpr float kWheelDirectionUndefinedMps = 0.001f;
constexpr float kLinearCommandDeadbandMps = 0.002f;
constexpr float kYawCommandDeadbandRadps = 0.003f;

constexpr float kQpFeasibilityTolerance = 1e-5f;
constexpr float kQpMultiplierTolerance = 1e-5f;
constexpr float kLinearSolveTolerance = 1e-8f;

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

float speedToRpm(float speed_mps)
{
  return speed_mps * 60.0f / (kPi * kWheelDiameterM);
}

float linearScaleFromError(
  float error, float full_speed_error,
  float crawl_error, float minimum_scale)
{
  if (error <= full_speed_error) {
    return 1.0f;
  }
  if (error >= crawl_error) {
    return minimum_scale;
  }

  const float ratio = (crawl_error - error) /
    (crawl_error - full_speed_error);
  return minimum_scale + (1.0f - minimum_scale) *
         clampf(ratio, 0.0f, 1.0f);
}

float limitVelocityJerk(
  float target, float current, float & current_accel,
  float max_accel, float max_brake, float max_jerk, float dt)
{
  if (dt <= 1e-6f) {
    current_accel = 0.0f;
    return target;
  }

  const bool crossing_zero = target * current < 0.0f;
  const bool reducing_magnitude = std::abs(target) < std::abs(current);
  const float acceleration_limit = std::max(
    (crossing_zero || reducing_magnitude) ? max_brake : max_accel, 0.0f);
  const float delta = target - current;

  if (std::abs(delta) <= 1e-6f && std::abs(current_accel) <= 1e-6f) {
    current_accel = 0.0f;
    return target;
  }

  // sqrt(2 * jerk * |dv|) 是接近目标速度时的加速度收敛包络。
  // 如果始终保持最大加速度直到最后一个周期，再直接贴到目标速度，最后一步
  // 仍会形成加速度阶跃；提前压低期望加速度可显著减小这种末端冲击。
  const float jerk = std::max(max_jerk, 0.0f);
  const float convergence_accel = jerk > 1e-6f ?
    std::sqrt(2.0f * jerk * std::abs(delta)) : acceleration_limit;
  const float desired_accel = std::copysign(
    std::min(acceleration_limit, convergence_accel), delta);

  float next_accel = desired_accel;
  if (jerk > 1e-6f) {
    const float maximum_accel_delta = jerk * dt;
    const float accel_delta = clampf(
      desired_accel - current_accel,
      -maximum_accel_delta, maximum_accel_delta);
    next_accel = current_accel + accel_delta;
  }
  next_accel = clampf(next_accel, -acceleration_limit, acceleration_limit);
  const float next_velocity = current + next_accel * dt;

  // 离散周期最后仍可能有极小越界。仅在真正跨过目标时贴到目标并清零加速度，
  // 上面的收敛包络会保证此处的加速度已经很小，避免来回越界形成低速顿挫。
  if (delta * (target - next_velocity) <= 0.0f) {
    current_accel = 0.0f;
    return target;
  }

  current_accel = next_accel;
  return next_velocity;
}

float dot(const Vector3 & lhs, const Vector3 & rhs)
{
  return lhs[0] * rhs[0] + lhs[1] * rhs[1] + lhs[2] * rhs[2];
}

Vector3 multiply(const Matrix3 & matrix, const Vector3 & vector)
{
  Vector3 result{};
  for (std::size_t row = 0; row < kTwistDimension; ++row) {
    for (std::size_t column = 0; column < kTwistDimension; ++column) {
      result[row] += matrix[row][column] * vector[column];
    }
  }
  return result;
}

void addOuterProduct(Matrix3 & matrix, const Vector3 & vector, float weight)
{
  for (std::size_t row = 0; row < kTwistDimension; ++row) {
    for (std::size_t column = 0; column < kTwistDimension; ++column) {
      matrix[row][column] += weight * vector[row] * vector[column];
    }
  }
}

bool solveLinearSystem(
  int dimension, float matrix[3][3], float rhs[3],
  float solution[3])
{
  float augmented[3][4]{};
  for (int row = 0; row < dimension; ++row) {
    for (int column = 0; column < dimension; ++column) {
      augmented[row][column] = matrix[row][column];
    }
    augmented[row][dimension] = rhs[row];
  }

  for (int pivot = 0; pivot < dimension; ++pivot) {
    int best_row = pivot;
    float best_value = std::abs(augmented[pivot][pivot]);
    for (int row = pivot + 1; row < dimension; ++row) {
      const float value = std::abs(augmented[row][pivot]);
      if (value > best_value) {
        best_value = value;
        best_row = row;
      }
    }

    if (best_value < kLinearSolveTolerance) {
      return false;
    }

    if (best_row != pivot) {
      for (int column = pivot; column <= dimension; ++column) {
        std::swap(augmented[pivot][column], augmented[best_row][column]);
      }
    }

    const float pivot_value = augmented[pivot][pivot];
    for (int column = pivot; column <= dimension; ++column) {
      augmented[pivot][column] /= pivot_value;
    }

    for (int row = 0; row < dimension; ++row) {
      if (row == pivot) {
        continue;
      }
      const float factor = augmented[row][pivot];
      for (int column = pivot; column <= dimension; ++column) {
        augmented[row][column] -= factor * augmented[pivot][column];
      }
    }
  }

  for (int row = 0; row < dimension; ++row) {
    solution[row] = augmented[row][dimension];
  }
  return true;
}

bool invertMatrix3(const Matrix3 & input, Matrix3 & inverse)
{
  for (std::size_t column = 0; column < kTwistDimension; ++column) {
    float matrix[3][3]{};
    float rhs[3]{};
    float solution[3]{};

    for (std::size_t row = 0; row < kTwistDimension; ++row) {
      for (std::size_t inner = 0; inner < kTwistDimension; ++inner) {
        matrix[row][inner] = input[row][inner];
      }
    }
    rhs[column] = 1.0f;

    if (!solveLinearSystem(3, matrix, rhs, solution)) {
      return false;
    }

    for (std::size_t row = 0; row < kTwistDimension; ++row) {
      inverse[row][column] = solution[row];
    }
  }
  return true;
}

}  // namespace

class MobileOmniController : public rclcpp::Node
{
public:
  using ParkWheels = mobile_joy_control::action::ParkWheels;
  using GoalHandleParkWheels = rclcpp_action::ServerGoalHandle<ParkWheels>;

  MobileOmniController()
  : Node("mobile_cmd_omni")
  {
    current_steer_angle_.fill(0.0f);
    commanded_steer_angle_.fill(0.0f);
    pending_switch_angle_.fill(0.0f);
    retarget_candidate_angle_.fill(0.0f);
    unpark_retarget_candidate_angle_.fill(0.0f);
    transition_start_rpm_.fill(0.0f);
    previous_output_rpm_.fill(0.0f);
    current_drive_rpm_.fill(0.0f);
    previous_candidate_flipped_.fill(false);
    previous_drive_sign_.fill(1.0f);
    previous_allocated_twist_.fill(0.0f);
    park_targets_.fill(0.0f);

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/control/cmd_vel_omni", 10,
      std::bind(&MobileOmniController::cmdVelCallback, this, std::placeholders::_1));

    feedback_sub_ = create_subscription<chassis_interfaces::msg::ChassisMotorControl>(
      "/feedback/omni_chassis_motor",
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
      std::bind(&MobileOmniController::feedbackCallback, this, std::placeholders::_1));

    motor_control_pub_ = create_publisher<chassis_interfaces::msg::ChassisMotorControl>(
      "/control/omni_chassis_motors", 10);

    // [ParkWheels] BT 导航结束后调用 /park_wheels；必须由本节点提供，避免与电机话题双写。
    // 成功后锁存 X 姿态；只有超过保护时间的明确非零新指令才能解除，避免平滑器减速尾巴误抢占。
    park_wheels_server_ = rclcpp_action::create_server<ParkWheels>(
      this, "park_wheels",
      std::bind(
        &MobileOmniController::handleParkGoal, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(&MobileOmniController::handleParkCancel, this, std::placeholders::_1),
      std::bind(&MobileOmniController::handleParkAccepted, this, std::placeholders::_1));

    last_cmd_time_ = now();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&MobileOmniController::publishControlCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "Mobile omni controller started: continuous steering + single-QP allocator + "
      "coordinated switch + /park_wheels");
  }

private:
  enum class CoordinatedSwitchState
  {
    kNormal,
    kDecelerating,
    kWaitingForStop,
    kSteering,
    kAligning
  };

  enum class UnparkState
  {
    kInactive,
    kSteering
  };

  struct SteerCandidate
  {
    float angle{0.0f};
    float drive_sign{1.0f};
    float discrete_cost{0.0f};
    bool flipped{false};
    bool valid{false};
  };

  struct SteeringSelection
  {
    std::array<float, kWheelCount> target_angle{};
    std::array<bool, kWheelCount> flipped{};
    std::array<float, kWheelCount> drive_sign{};
    float maximum_target_change{0.0f};
    int mode_index{0};
    bool success{false};
  };

  struct LinearConstraint
  {
    Vector3 coefficients{};
    float upper_bound{0.0f};
  };

  struct QpProblem
  {
    Matrix3 hessian{};
    Vector3 gradient{};
    std::vector<LinearConstraint> constraints;
  };

  struct QpResult
  {
    Vector3 solution{};
    float objective{std::numeric_limits<float>::infinity()};
    bool success{false};
    int active_constraint_count{0};
  };

  struct AllocationResult
  {
    Vector3 twist{};
    std::array<float, kWheelCount> target_angle{};
    std::array<float, kWheelCount> steering_error{};
    std::array<bool, kWheelCount> flipped{};
    std::array<float, kWheelCount> drive_sign{};
    float qp_objective{std::numeric_limits<float>::infinity()};
    float total_score{std::numeric_limits<float>::infinity()};
    float max_lateral_speed{0.0f};
    float max_steering_error{0.0f};
    int mode_index{-1};
    int active_constraint_count{0};
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
    last_steering_feedback_time_ = now();

    // 驱动轮实际转速用于协调换向的“已停车”闭环确认。不能只看本节点已经
    // 发出 0 rpm，因为重载底盘仍可能受惯性影响继续运动。
    if (message->speed.size() >= kWheelCount) {
      for (std::size_t i = 0; i < kWheelCount; ++i) {
        current_drive_rpm_[i] = static_cast<float>(message->speed[i]);
      }
      drive_feedback_received_ = true;
      last_drive_feedback_time_ = now();
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "驱动轮反馈只有 %zu 个速度值，期望至少 %zu 个；快速换向将保持零速等待",
        message->speed.size(), kWheelCount);
    }
  }

  void resetCommandGovernor()
  {
    governed_vx_ = 0.0f;
    governed_vy_ = 0.0f;
    governed_yaw_rate_ = 0.0f;
    governed_ax_ = 0.0f;
    governed_ay_ = 0.0f;
    governed_yaw_accel_ = 0.0f;
    command_governor_initialized_ = false;
    previous_allocated_twist_.fill(0.0f);
  }

  void applyCommandGovernor(
    float vx_ref, float vy_ref, float yaw_rate_ref,
    float & vx_cmd, float & vy_cmd, float & yaw_rate_cmd)
  {
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
      governed_ax_ = 0.0f;
      governed_ay_ = 0.0f;
      governed_yaw_accel_ = 0.0f;
      command_governor_initialized_ = true;
    }

    governed_vx_ = limitVelocityJerk(
      vx_ref, governed_vx_, governed_ax_,
      kMaxAccelX, kMaxBrakeX, kMaxJerkX, kControlPeriodS);
    governed_vy_ = limitVelocityJerk(
      vy_ref, governed_vy_, governed_ay_,
      kMaxAccelY, kMaxBrakeY, kMaxJerkY, kControlPeriodS);
    governed_yaw_rate_ = limitVelocityJerk(
      yaw_rate_ref, governed_yaw_rate_, governed_yaw_accel_,
      kMaxAccelYaw, kMaxBrakeYaw, kMaxJerkYaw, kControlPeriodS);

    vx_cmd = governed_vx_;
    vy_cmd = governed_vy_;
    yaw_rate_cmd = governed_yaw_rate_;
  }

  void applyRotationDominantMode(
    float & vx_ref, float & vy_ref, float yaw_rate_ref)
  {
    const float linear_speed = std::hypot(vx_ref, vy_ref);
    const float yaw_speed = std::abs(yaw_rate_ref);

    if (!rotation_dominant_mode_ &&
      yaw_speed >= kRotationModeEnterYawRadps &&
      linear_speed <= kRotationModeEnterLinearMps)
    {
      rotation_dominant_mode_ = true;
      RCLCPP_INFO(
        get_logger(),
        "进入纯旋转模式：vx/vy 已接近零，稳定四轮切线方向");
    } else if (rotation_dominant_mode_ &&
      (yaw_speed <= kRotationModeExitYawRadps ||
      linear_speed >= kRotationModeExitLinearMps))
    {
      rotation_dominant_mode_ = false;
      RCLCPP_INFO(
        get_logger(),
        "退出纯旋转模式：恢复完整全向速度指令");
    }

    if (rotation_dominant_mode_) {
      vx_ref = 0.0f;
      vy_ref = 0.0f;
    }
  }

  float limitProximityCost(float angle) const
  {
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

  std::array<SteerCandidate, 2> buildSteerCandidates(
    float raw_angle) const
  {
    const float effective_limit =
      std::max(kSteerAngleLimitRad - kSteerLimitMarginRad, 0.1f);

    std::array<SteerCandidate, 2> candidates{};

    candidates[0].angle = wrapPi(raw_angle);
    candidates[0].drive_sign = 1.0f;
    candidates[0].flipped = false;
    candidates[0].valid = std::abs(candidates[0].angle) <= effective_limit;

    candidates[1].angle = wrapPi(raw_angle + kPi);
    candidates[1].drive_sign = -1.0f;
    candidates[1].flipped = true;
    candidates[1].valid = std::abs(candidates[1].angle) <= effective_limit;

    // 正常软限位大于 90 度时，两个等价解至少有一个可行。
    // 若配置异常导致都不可行，提供一个夹紧后的安全候选。
    if (!candidates[0].valid && !candidates[1].valid) {
      candidates[0].angle = clampf(
        candidates[0].angle,
        -effective_limit, effective_limit);
      candidates[0].drive_sign = 1.0f;
      candidates[0].flipped = false;
      candidates[0].valid = true;
      candidates[0].discrete_cost = 1e4f;
    }

    return candidates;
  }

  SteeringSelection selectSteeringTargets(const Vector3 & command) const
  {
    SteeringSelection selection;
    const float effective_limit =
      std::max(kSteerAngleLimitRad - kSteerLimitMarginRad, 0.1f);
    const float preferred_limit =
      std::max(effective_limit - kSteerLimitGuardRad, 0.1f);

    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
      const bool is_right = wheel == 1 || wheel == 3;
      const bool is_front = wheel == 0 || wheel == 1;
      const float x_sign = is_right ? 1.0f : -1.0f;
      const float y_sign = is_front ? 1.0f : -1.0f;
      const float corner_vx =
        command[0] + x_sign * kChassisWidthM * command[2] * 0.5f;
      const float corner_vy =
        command[1] + y_sign * kChassisLengthM * command[2] * 0.5f;
      const float corner_speed = std::hypot(corner_vx, corner_vy);
      const float reference_angle = target_memory_initialized_ ?
        commanded_steer_angle_[wheel] : current_steer_angle_[wheel];

      if (corner_speed < kWheelDirectionUndefinedMps) {
        // 该轮速度向量真正接近零时方向没有定义，沿用上一目标，不重新选分支。
        selection.target_angle[wheel] = reference_angle;
        selection.flipped[wheel] = previous_candidate_flipped_[wheel];
        selection.drive_sign[wheel] = previous_drive_sign_[wheel];
        continue;
      }

      const auto candidates = buildSteerCandidates(std::atan2(corner_vy, corner_vx));
      const bool has_guarded_candidate =
        (candidates[0].valid && std::abs(candidates[0].angle) <= preferred_limit) ||
        (candidates[1].valid && std::abs(candidates[1].angle) <= preferred_limit);

      int best_index = -1;
      float best_cost = std::numeric_limits<float>::infinity();
      for (int candidate_index = 0; candidate_index < 2; ++candidate_index) {
        const auto & candidate = candidates[candidate_index];
        if (!candidate.valid) {
          continue;
        }
        // 只要存在不在预警区内的等价解，就不继续沿贴近软限位的分支运行。
        if (has_guarded_candidate && std::abs(candidate.angle) > preferred_limit) {
          continue;
        }

        const float continuity_cost = std::abs(
          angleDiff(
            candidate.angle, reference_angle));
        const float cost = continuity_cost +
          kLimitCostGain * limitProximityCost(candidate.angle);
        if (cost < best_cost) {
          best_cost = cost;
          best_index = candidate_index;
        }
      }

      if (best_index < 0) {
        return selection;
      }

      const auto & selected = candidates[best_index];
      selection.target_angle[wheel] = selected.angle;
      selection.flipped[wheel] = selected.flipped;
      selection.drive_sign[wheel] = selected.drive_sign;
      selection.maximum_target_change = std::max(
        selection.maximum_target_change,
        std::abs(angleDiff(selected.angle, reference_angle)));
      selection.mode_index |= best_index << wheel;
    }

    selection.success = true;
    return selection;
  }

  Vector3 wheelLongitudinalRow(
    std::size_t wheel_index,
    float steer_angle) const
  {
    const bool is_right = wheel_index == 1 || wheel_index == 3;
    const bool is_front = wheel_index == 0 || wheel_index == 1;
    const float x_sign = is_right ? 1.0f : -1.0f;
    const float y_sign = is_front ? 1.0f : -1.0f;
    const float cosine = std::cos(steer_angle);
    const float sine = std::sin(steer_angle);

    return Vector3{
      cosine,
      sine,
      cosine * x_sign * kChassisWidthM * 0.5f +
      sine * y_sign * kChassisLengthM * 0.5f};
  }

  Vector3 wheelLateralRow(
    std::size_t wheel_index,
    float steer_angle) const
  {
    const bool is_right = wheel_index == 1 || wheel_index == 3;
    const bool is_front = wheel_index == 0 || wheel_index == 1;
    const float x_sign = is_right ? 1.0f : -1.0f;
    const float y_sign = is_front ? 1.0f : -1.0f;
    const float cosine = std::cos(steer_angle);
    const float sine = std::sin(steer_angle);

    return Vector3{
      -sine,
      cosine,
      -sine * x_sign * kChassisWidthM * 0.5f +
      cosine * y_sign * kChassisLengthM * 0.5f};
  }

  void addQuadraticReference(
    QpProblem & problem, std::size_t dimension,
    float weight, float reference) const
  {
    problem.hessian[dimension][dimension] += weight;
    problem.gradient[dimension] -= weight * reference;
  }

  void addSymmetricBound(
    QpProblem & problem, const Vector3 & coefficients,
    float absolute_bound) const
  {
    const float bound = std::max(absolute_bound, 0.0f);
    problem.constraints.push_back(LinearConstraint{coefficients, bound});
    problem.constraints.push_back(
      LinearConstraint{
      Vector3{-coefficients[0], -coefficients[1], -coefficients[2]}, bound});
  }

  QpProblem buildQpProblem(
    const Vector3 & command,
    const std::array<float, kWheelCount> & target_angle,
    const std::array<float, kWheelCount> & steering_error) const
  {
    QpProblem problem;
    problem.constraints.reserve(14);

    addQuadraticReference(problem, 0, kQpTrackX, command[0]);
    addQuadraticReference(problem, 1, kQpTrackY, command[1]);
    addQuadraticReference(problem, 2, kQpTrackYaw, command[2]);

    addQuadraticReference(problem, 0, kQpSmoothX, previous_allocated_twist_[0]);
    addQuadraticReference(problem, 1, kQpSmoothY, previous_allocated_twist_[1]);
    addQuadraticReference(problem, 2, kQpSmoothYaw, previous_allocated_twist_[2]);

    for (std::size_t dimension = 0; dimension < kTwistDimension; ++dimension) {
      problem.hessian[dimension][dimension] += kQpRegularization;
    }

    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
      // 实际舵角决定当前轮胎能够无侧滑执行的方向。
      const Vector3 longitudinal = wheelLongitudinalRow(
        wheel, current_steer_angle_[wheel]);
      const Vector3 lateral = wheelLateralRow(
        wheel, current_steer_angle_[wheel]);

      // 侧向速度平方进入代价函数，减少重载轮胎横向拖拽。
      addOuterProduct(problem.hessian, lateral, kQpLateralSlipWeight);

      // 轻量跟踪命令在当前轮向上的投影，改善低权重方向的响应。
      const float requested_wheel_speed = dot(longitudinal, command);
      addOuterProduct(problem.hessian, longitudinal, kQpWheelSpeedTrackWeight);
      for (std::size_t dimension = 0; dimension < kTwistDimension; ++dimension) {
        problem.gradient[dimension] -=
          kQpWheelSpeedTrackWeight * requested_wheel_speed * longitudinal[dimension];
      }

      const float alignment_scale = linearScaleFromError(
        steering_error[wheel],
        kAlignFullSpeedErrorRad,
        kAlignStopErrorRad,
        kAlignMinimumSpeedScale);
      const float wheel_speed_limit = kMaxWheelLinearSpeed * alignment_scale;
      addSymmetricBound(problem, longitudinal, wheel_speed_limit);
    }

    addSymmetricBound(problem, Vector3{1.0f, 0.0f, 0.0f}, kMaxBodySpeedX);
    addSymmetricBound(problem, Vector3{0.0f, 1.0f, 0.0f}, kMaxBodySpeedY);
    addSymmetricBound(problem, Vector3{0.0f, 0.0f, 1.0f}, kMaxBodyYawRate);

    (void)target_angle;  // 目标角已通过 steering_error 进入约束。
    return problem;
  }

  float evaluateObjective(
    const QpProblem & problem,
    const Vector3 & solution) const
  {
    const Vector3 hessian_times_solution = multiply(problem.hessian, solution);
    return 0.5f * dot(solution, hessian_times_solution) +
           dot(problem.gradient, solution);
  }

  bool isFeasible(const QpProblem & problem, const Vector3 & solution) const
  {
    for (const LinearConstraint & constraint : problem.constraints) {
      if (dot(constraint.coefficients, solution) >
        constraint.upper_bound + kQpFeasibilityTolerance)
      {
        return false;
      }
    }
    return true;
  }

  bool solveForActiveSet(
    const QpProblem & problem,
    const Matrix3 & inverse_hessian,
    const Vector3 & unconstrained_solution,
    const std::array<int, 3> & active_indices,
    int active_count,
    Vector3 & solution) const
  {
    if (active_count == 0) {
      solution = unconstrained_solution;
      return isFeasible(problem, solution);
    }

    float schur_matrix[3][3]{};
    float rhs[3]{};
    float multipliers[3]{};

    std::array<Vector3, 3> inverse_hessian_times_a{};
    for (int row = 0; row < active_count; ++row) {
      const LinearConstraint & constraint = problem.constraints[active_indices[row]];
      inverse_hessian_times_a[row] = multiply(inverse_hessian, constraint.coefficients);
      rhs[row] = dot(constraint.coefficients, unconstrained_solution) -
        constraint.upper_bound;
    }

    for (int row = 0; row < active_count; ++row) {
      const LinearConstraint & row_constraint = problem.constraints[active_indices[row]];
      for (int column = 0; column < active_count; ++column) {
        schur_matrix[row][column] = dot(
          row_constraint.coefficients,
          inverse_hessian_times_a[column]);
      }
    }

    if (!solveLinearSystem(active_count, schur_matrix, rhs, multipliers)) {
      return false;
    }

    for (int i = 0; i < active_count; ++i) {
      if (multipliers[i] < -kQpMultiplierTolerance) {
        return false;
      }
    }

    solution = unconstrained_solution;
    for (int active = 0; active < active_count; ++active) {
      for (std::size_t dimension = 0; dimension < kTwistDimension; ++dimension) {
        solution[dimension] -=
          inverse_hessian_times_a[active][dimension] * multipliers[active];
      }
    }

    return isFeasible(problem, solution);
  }

  void considerActiveSet(
    const QpProblem & problem,
    const Matrix3 & inverse_hessian,
    const Vector3 & unconstrained_solution,
    const std::array<int, 3> & active_indices,
    int active_count,
    QpResult & best_result) const
  {
    Vector3 candidate{};
    if (!solveForActiveSet(
        problem, inverse_hessian, unconstrained_solution,
        active_indices, active_count, candidate))
    {
      return;
    }

    const float objective = evaluateObjective(problem, candidate);
    if (!best_result.success || objective < best_result.objective) {
      best_result.solution = candidate;
      best_result.objective = objective;
      best_result.success = true;
      best_result.active_constraint_count = active_count;
    }
  }

  QpResult solveQp(const QpProblem & problem) const
  {
    QpResult result;

    Matrix3 inverse_hessian{};
    if (!invertMatrix3(problem.hessian, inverse_hessian)) {
      return result;
    }

    const Vector3 negative_gradient{
      -problem.gradient[0],
      -problem.gradient[1],
      -problem.gradient[2]};
    const Vector3 unconstrained_solution = multiply(
      inverse_hessian, negative_gradient);

    std::array<int, 3> active_indices{-1, -1, -1};
    considerActiveSet(
      problem, inverse_hessian, unconstrained_solution,
      active_indices, 0, result);

    const int constraint_count = static_cast<int>(problem.constraints.size());
    for (int first = 0; first < constraint_count; ++first) {
      active_indices[0] = first;
      considerActiveSet(
        problem, inverse_hessian, unconstrained_solution,
        active_indices, 1, result);
    }

    for (int first = 0; first < constraint_count; ++first) {
      for (int second = first + 1; second < constraint_count; ++second) {
        active_indices[0] = first;
        active_indices[1] = second;
        considerActiveSet(
          problem, inverse_hessian, unconstrained_solution,
          active_indices, 2, result);
      }
    }

    for (int first = 0; first < constraint_count; ++first) {
      for (int second = first + 1; second < constraint_count; ++second) {
        for (int third = second + 1; third < constraint_count; ++third) {
          active_indices[0] = first;
          active_indices[1] = second;
          active_indices[2] = third;
          considerActiveSet(
            problem, inverse_hessian, unconstrained_solution,
            active_indices, 3, result);
        }
      }
    }

    return result;
  }

  AllocationResult solveAllocation(
    const Vector3 & command,
    const SteeringSelection & selection) const
  {
    AllocationResult result;

    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
      result.target_angle[wheel] = selection.target_angle[wheel];
      result.flipped[wheel] = selection.flipped[wheel];
      result.drive_sign[wheel] = selection.drive_sign[wheel];
      result.steering_error[wheel] = std::abs(
        angleDiff(
          selection.target_angle[wheel], current_steer_angle_[wheel]));
      result.max_steering_error = std::max(
        result.max_steering_error, result.steering_error[wheel]);
    }

    // 候选分支已经通过连续性和软限位预警确定，这里只求一次三变量 QP。
    // rosbag 中旧实现运动时会从 20 Hz 降到约 16 Hz，主要负载就是最多 16 次重复求解。
    const QpProblem problem = buildQpProblem(
      command, result.target_angle, result.steering_error);
    const QpResult qp_result = solveQp(problem);
    if (!qp_result.success) {
      return result;
    }

    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
      const Vector3 lateral = wheelLateralRow(
        wheel, current_steer_angle_[wheel]);
      result.max_lateral_speed = std::max(
        result.max_lateral_speed,
        std::abs(dot(lateral, qp_result.solution)));
    }

    result.twist = qp_result.solution;
    result.qp_objective = qp_result.objective;
    result.total_score = qp_result.objective;
    result.mode_index = selection.mode_index;
    result.active_constraint_count = qp_result.active_constraint_count;
    result.success = true;
    return result;
  }

  const char * coordinatedSwitchStateName() const
  {
    switch (coordinated_switch_state_) {
      case CoordinatedSwitchState::kNormal:
        return "NORMAL";
      case CoordinatedSwitchState::kDecelerating:
        return "DECELERATING";
      case CoordinatedSwitchState::kWaitingForStop:
        return "WAITING_FOR_STOP";
      case CoordinatedSwitchState::kSteering:
        return "STEERING";
      case CoordinatedSwitchState::kAligning:
        return "ALIGNING";
    }
    return "UNKNOWN";
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

  float maximumActualDriveRpm() const
  {
    float maximum_rpm = 0.0f;
    for (const float rpm : current_drive_rpm_) {
      maximum_rpm = std::max(maximum_rpm, std::abs(rpm));
    }
    return maximum_rpm;
  }

  float maximumSteeringFeedbackError(
    const std::array<float, kWheelCount> & target_angle) const
  {
    float maximum_error = 0.0f;
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
      maximum_error = std::max(
        maximum_error,
        std::abs(angleDiff(target_angle[wheel], current_steer_angle_[wheel])));
    }
    return maximum_error;
  }

  bool steeringReferenceIsValid(const Vector3 & steering_reference) const
  {
    return std::hypot(steering_reference[0], steering_reference[1]) >=
           kLinearCommandDeadbandMps ||
           std::abs(steering_reference[2]) >= kYawCommandDeadbandRadps;
  }

  float driveScaleFromFinalSteeringError(float maximum_error) const
  {
    return linearScaleFromError(
      maximum_error, kAlignFullSpeedErrorRad,
      kAlignStopErrorRad, kAlignMinimumSpeedScale);
  }

  float nextPostAlignmentDriveScale()
  {
    if (post_alignment_drive_ramp_cycle_ >= kPostAlignmentDriveRampCycles) {
      return 1.0f;
    }
    ++post_alignment_drive_ramp_cycle_;
    return static_cast<float>(post_alignment_drive_ramp_cycle_) /
           static_cast<float>(kPostAlignmentDriveRampCycles);
  }

  void beginUnparkAlignment()
  {
    unpark_state_ = UnparkState::kSteering;
    unpark_targets_initialized_ = false;
    unpark_align_stable_cycle_ = 0;
    unpark_retarget_stable_cycle_ = 0;
    unpark_retarget_candidate_initialized_ = false;

    // 驻车期间驱动轮一直是零速，解除时可以直接进入停车对舵。
    coordinated_switch_state_ = CoordinatedSwitchState::kNormal;
    previous_output_rpm_.fill(0.0f);
    previous_allocated_twist_.fill(0.0f);
    target_memory_initialized_ = false;
    resetCommandGovernor();

    RCLCPP_INFO(
      get_logger(),
      "[Unpark] 收到新运动指令：保持四轮零速，先按启动方向快速对舵");
  }

  bool tickUnparkAlignment(
    const Vector3 & steering_reference,
    std::array<float, kWheelCount> & output_angle,
    std::array<float, kWheelCount> & output_rpm)
  {
    if (unpark_state_ == UnparkState::kInactive) {
      return false;
    }

    // 解锁对舵全过程严格禁止驱动轮输出，避免舵轮未到位时摩地。
    output_rpm.fill(0.0f);
    previous_output_rpm_.fill(0.0f);
    previous_allocated_twist_.fill(0.0f);

    const bool reference_is_valid =
      std::hypot(steering_reference[0], steering_reference[1]) >=
      kLinearCommandDeadbandMps ||
      std::abs(steering_reference[2]) >= kYawCommandDeadbandRadps;

    if (reference_is_valid) {
      const SteeringSelection selection = selectSteeringTargets(steering_reference);
      if (selection.success) {
        if (!unpark_targets_initialized_) {
          unpark_target_angle_ = selection.target_angle;
          unpark_target_flipped_ = selection.flipped;
          unpark_target_drive_sign_ = selection.drive_sign;
          commanded_steer_angle_ = unpark_target_angle_;
          target_memory_initialized_ = true;
          unpark_targets_initialized_ = true;
          unpark_align_stable_cycle_ = 0;
        } else {
          float maximum_target_change = 0.0f;
          for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
            maximum_target_change = std::max(
              maximum_target_change,
              std::abs(
                angleDiff(
                  selection.target_angle[wheel], unpark_target_angle_[wheel])));
          }

          if (maximum_target_change <= kUnparkRetargetAngleRad) {
            // 解锁期间也锁存首次运动方向；MPPI 的小幅 vx/vy 抖动不能持续追着摆舵。
            unpark_retarget_stable_cycle_ = 0;
            unpark_retarget_candidate_initialized_ = false;
          } else {
            float consistency_error = 0.0f;
            if (unpark_retarget_candidate_initialized_) {
              for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
                consistency_error = std::max(
                  consistency_error,
                  std::abs(
                    angleDiff(
                      selection.target_angle[wheel],
                      unpark_retarget_candidate_angle_[wheel])));
              }
            }

            if (!unpark_retarget_candidate_initialized_ ||
              consistency_error > kUnparkRetargetConsistencyRad)
            {
              unpark_retarget_candidate_angle_ = selection.target_angle;
              unpark_retarget_candidate_flipped_ = selection.flipped;
              unpark_retarget_candidate_drive_sign_ = selection.drive_sign;
              unpark_retarget_candidate_initialized_ = true;
              unpark_retarget_stable_cycle_ = 1;
            } else {
              ++unpark_retarget_stable_cycle_;
              if (unpark_retarget_stable_cycle_ >= kUnparkRetargetStableCycles) {
                unpark_target_angle_ = unpark_retarget_candidate_angle_;
                unpark_target_flipped_ = unpark_retarget_candidate_flipped_;
                unpark_target_drive_sign_ = unpark_retarget_candidate_drive_sign_;
                commanded_steer_angle_ = unpark_target_angle_;
                unpark_align_stable_cycle_ = 0;
                unpark_retarget_stable_cycle_ = 0;
                unpark_retarget_candidate_initialized_ = false;
                RCLCPP_INFO(
                  get_logger(),
                  "[Unpark] 确认持续改向：更新锁存启动舵角并重新等待连续到位");
              }
            }
          }
        }
      } else {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[Unpark] 无法根据启动指令选择舵角；继续保持四轮零速");
      }
    }

    output_angle = unpark_targets_initialized_ ?
      unpark_target_angle_ : current_steer_angle_;

    const bool feedback_fresh = steering_feedback_received_ &&
      (now() - last_steering_feedback_time_).seconds() <=
      kSteeringFeedbackTimeoutS;
    if (!feedback_fresh) {
      unpark_align_stable_cycle_ = 0;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[Unpark] 舵角反馈超时；继续保持四轮零速，不允许起步");
      return true;
    }

    float maximum_feedback_error = 0.0f;
    if (unpark_targets_initialized_) {
      for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        maximum_feedback_error = std::max(
          maximum_feedback_error,
          std::abs(
            angleDiff(
              unpark_target_angle_[wheel], current_steer_angle_[wheel])));
      }
    }

    if (unpark_targets_initialized_ &&
      maximum_feedback_error <= kUnparkAlignToleranceRad)
    {
      ++unpark_align_stable_cycle_;
    } else {
      unpark_align_stable_cycle_ = 0;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "[Unpark] 零速对舵中：最大反馈误差 %.3f rad，稳定周期 %d/%d",
      maximum_feedback_error, unpark_align_stable_cycle_,
      kUnparkAlignStableCycles);

    if (unpark_align_stable_cycle_ >= kUnparkAlignStableCycles) {
      previous_candidate_flipped_ = unpark_target_flipped_;
      previous_drive_sign_ = unpark_target_drive_sign_;
      candidate_memory_initialized_ = true;
      unpark_state_ = UnparkState::kInactive;
      unpark_retarget_stable_cycle_ = 0;
      unpark_retarget_candidate_initialized_ = false;
      // 对舵期间治理器已经连续跟踪上游目标，保留该稳定方向；驱动轮由独立斜坡
      // 从零释放，避免治理器重置后的 vx/vy/wz 比例失真再次触发换向。
      previous_allocated_twist_.fill(0.0f);
      post_alignment_drive_ramp_cycle_ = 0;
      RCLCPP_INFO(
        get_logger(),
        "[Unpark] 四轮舵角已连续确认到位；下一周期按稳定方向平滑释放轮速");
    }

    return true;
  }

  void beginCoordinatedSwitch(const SteeringSelection & selection)
  {
    if (!target_memory_initialized_) {
      commanded_steer_angle_ = current_steer_angle_;
      target_memory_initialized_ = true;
    }

    pending_switch_angle_ = selection.target_angle;
    pending_switch_flipped_ = selection.flipped;
    pending_switch_drive_sign_ = selection.drive_sign;
    transition_start_rpm_ = previous_output_rpm_;
    coordinated_deceleration_cycle_ = 0;
    coordinated_stop_stable_cycle_ = 0;
    coordinated_align_stable_cycle_ = 0;
    coordinated_retarget_stable_cycle_ = 0;
    retarget_candidate_initialized_ = false;
    const float maximum_start_rpm = *std::max_element(
      transition_start_rpm_.begin(), transition_start_rpm_.end(),
      [](float lhs, float rhs) {return std::abs(lhs) < std::abs(rhs);});
    coordinated_switch_state_ = std::abs(maximum_start_rpm) <= 1.0f ?
      CoordinatedSwitchState::kWaitingForStop :
      CoordinatedSwitchState::kDecelerating;

    const float maximum_feedback_error =
      maximumSteeringFeedbackError(selection.target_angle);

    RCLCPP_WARN(
      get_logger(),
      "触发整车协调换向：最终目标相对真实舵角误差 %.3f rad；"
      "当前最大指令轮速 %.1f rpm，先停车、确认实际轮速为零，再快速对舵",
      maximum_feedback_error, std::abs(maximum_start_rpm));
  }

  void updateCoordinatedSwitchTarget(const Vector3 & steering_reference)
  {
    if (!steeringReferenceIsValid(steering_reference)) {
      return;
    }

    const SteeringSelection selection = selectSteeringTargets(steering_reference);
    if (!selection.success) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "协调换向期间无法根据最新速度指令更新舵角目标；继续保持四轮零速");
      return;
    }

    float maximum_target_change = 0.0f;
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
      maximum_target_change = std::max(
        maximum_target_change,
        std::abs(
          angleDiff(
            selection.target_angle[wheel], pending_switch_angle_[wheel])));
    }

    if (maximum_target_change <= kCoordinatedRetargetAngleRad) {
      // 小于迟滞阈值视为 MPPI 正常采样抖动：继续追锁存目标，不能每周期改舵角。
      coordinated_retarget_stable_cycle_ = 0;
      retarget_candidate_initialized_ = false;
      return;
    }

    float candidate_consistency_error = 0.0f;
    if (retarget_candidate_initialized_) {
      for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        candidate_consistency_error = std::max(
          candidate_consistency_error,
          std::abs(
            angleDiff(
              selection.target_angle[wheel], retarget_candidate_angle_[wheel])));
      }
    }

    if (!retarget_candidate_initialized_ ||
      candidate_consistency_error > kCoordinatedRetargetConsistencyRad)
    {
      retarget_candidate_angle_ = selection.target_angle;
      retarget_candidate_flipped_ = selection.flipped;
      retarget_candidate_drive_sign_ = selection.drive_sign;
      retarget_candidate_initialized_ = true;
      coordinated_retarget_stable_cycle_ = 1;
      return;
    }

    ++coordinated_retarget_stable_cycle_;
    if (coordinated_retarget_stable_cycle_ >= kCoordinatedRetargetStableCycles) {
      pending_switch_angle_ = retarget_candidate_angle_;
      pending_switch_flipped_ = retarget_candidate_flipped_;
      pending_switch_drive_sign_ = retarget_candidate_drive_sign_;
      coordinated_align_stable_cycle_ = 0;
      coordinated_retarget_stable_cycle_ = 0;
      retarget_candidate_initialized_ = false;
      RCLCPP_INFO(
        get_logger(),
        "协调换向确认持续改向：更新锁存舵角目标，并重新等待四轮连续到位");
    }
  }

  bool tickCoordinatedSwitch(
    const Vector3 & steering_reference,
    std::array<float, kWheelCount> & output_angle,
    std::array<float, kWheelCount> & output_rpm)
  {
    if (coordinated_switch_state_ == CoordinatedSwitchState::kNormal) {
      return false;
    }

    output_rpm.fill(0.0f);
    updateCoordinatedSwitchTarget(steering_reference);

    if (coordinated_switch_state_ == CoordinatedSwitchState::kDecelerating) {
      // 四个轮子使用同一个缩放比例，禁止“单轮停、其他轮继续推”造成横向摩地。
      ++coordinated_deceleration_cycle_;
      const float scale = std::max(
        0.0f,
        1.0f - static_cast<float>(coordinated_deceleration_cycle_) /
        static_cast<float>(kCoordinatedDecelerationCycles));
      output_angle = commanded_steer_angle_;
      for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
        output_rpm[wheel] = transition_start_rpm_[wheel] * scale;
      }
      previous_output_rpm_ = output_rpm;

      if (coordinated_deceleration_cycle_ >= kCoordinatedDecelerationCycles) {
        coordinated_switch_state_ = CoordinatedSwitchState::kWaitingForStop;
        coordinated_stop_stable_cycle_ = 0;
        previous_allocated_twist_.fill(0.0f);
      }
      return true;
    }

    if (coordinated_switch_state_ == CoordinatedSwitchState::kWaitingForStop) {
      // 已经发出零轮速后仍保持原舵角，等待驱动轮真实转速连续归零。
      // 在底盘尚有惯性时摆舵，会把滚动摩擦变成横向滑动摩擦。
      output_angle = commanded_steer_angle_;
      previous_output_rpm_.fill(0.0f);

      if (!driveFeedbackFresh()) {
        coordinated_stop_stable_cycle_ = 0;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "协调换向等待驱动轮反馈；反馈缺失或超时，保持四轮零速且不摆舵");
        return true;
      }

      const float maximum_actual_rpm = maximumActualDriveRpm();
      if (maximum_actual_rpm <= kCoordinatedStoppedRpm) {
        ++coordinated_stop_stable_cycle_;
      } else {
        coordinated_stop_stable_cycle_ = 0;
      }

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "协调换向等待实际停车：最大反馈轮速 %.2f rpm，稳定周期 %d/%d",
        maximum_actual_rpm, coordinated_stop_stable_cycle_,
        kCoordinatedStoppedStableCycles);

      if (coordinated_stop_stable_cycle_ >= kCoordinatedStoppedStableCycles) {
        coordinated_switch_state_ = CoordinatedSwitchState::kSteering;
      }
      return true;
    }

    if (coordinated_switch_state_ == CoordinatedSwitchState::kSteering) {
      // 车体已经由实际轮速确认停止，此时直接发送最终舵角，取消普通行驶的
      // 0.12 rad/周期软件限幅，让锁车/解锁/快速换向都尽快完成对舵。
      commanded_steer_angle_ = pending_switch_angle_;
      output_angle = commanded_steer_angle_;
      previous_output_rpm_.fill(0.0f);
      coordinated_switch_state_ = CoordinatedSwitchState::kAligning;
      coordinated_align_stable_cycle_ = 0;
      return true;
    }

    // 目标指令已到位后继续保持整车零速，并依据真实舵角反馈连续确认。
    output_angle = pending_switch_angle_;
    commanded_steer_angle_ = pending_switch_angle_;
    previous_output_rpm_.fill(0.0f);

    if (!steeringFeedbackFresh()) {
      coordinated_align_stable_cycle_ = 0;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "协调换向等待舵角反馈；反馈缺失或超时，保持四轮零速，不允许起步");
      return true;
    }

    const float maximum_feedback_error =
      maximumSteeringFeedbackError(pending_switch_angle_);

    if (maximum_feedback_error <= kCoordinatedAlignToleranceRad) {
      ++coordinated_align_stable_cycle_;
    } else {
      coordinated_align_stable_cycle_ = 0;
    }

    if (coordinated_align_stable_cycle_ >= kCoordinatedAlignStableCycles) {
      previous_candidate_flipped_ = pending_switch_flipped_;
      previous_drive_sign_ = pending_switch_drive_sign_;
      candidate_memory_initialized_ = true;
      previous_output_rpm_.fill(0.0f);
      coordinated_switch_state_ = CoordinatedSwitchState::kNormal;
      coordinated_retarget_stable_cycle_ = 0;
      retarget_candidate_initialized_ = false;
      // 不重置已经收敛的 S 曲线方向；只把 QP 连续性参考清零，并用统一轮速
      // 比例从零释放，防止下一周期再次因瞬时方向变化进入协调换向。
      previous_allocated_twist_.fill(0.0f);
      post_alignment_drive_ramp_cycle_ = 0;
      RCLCPP_INFO(
        get_logger(),
        "整车协调换向完成：舵角已连续确认对齐，按稳定方向平滑释放轮速");
    }
    return true;
  }

  void computeWheels(
    float vx, float vy, float yaw_rate,
    const Vector3 & startup_steering_reference,
    std::array<float, kWheelCount> & output_angle,
    std::array<float, kWheelCount> & output_rpm)
  {
    // X 驻车刚解除时治理器会从零爬升，必须用上游完整目标提前完成零速对舵。
    if (tickUnparkAlignment(startup_steering_reference, output_angle, output_rpm)) {
      return;
    }

    // 除 Unpark 外，舵角选择必须与真正交给 QP 的 S 曲线治理后速度一致。
    // 旧实现用原始 MPPI 指令选舵、用治理后速度算轮速；低速时 vy/wz 抖动会让
    // 舵角方向先跳变并连续触发停车换向，表现为刚起步反复“一顿一顿”。
    const Vector3 command{vx, vy, yaw_rate};
    if (tickCoordinatedSwitch(command, output_angle, output_rpm)) {
      return;
    }

    // 正常行驶也必须检查舵角反馈时效。历史上只要收到过一次反馈就继续运行，
    // 一旦反馈卡住便会使用旧角度计算轮速；现在反馈超时立即停止四轮驱动。
    if (!steeringFeedbackFresh()) {
      output_angle = target_memory_initialized_ ?
        commanded_steer_angle_ : current_steer_angle_;
      output_rpm.fill(0.0f);
      previous_output_rpm_.fill(0.0f);
      previous_allocated_twist_.fill(0.0f);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "正常行驶舵角反馈缺失或超时；保持舵角并停止四轮驱动");
      return;
    }

    const SteeringSelection desired_selection = selectSteeringTargets(command);
    if (!desired_selection.success) {
      output_angle = target_memory_initialized_ ?
        commanded_steer_angle_ : current_steer_angle_;
      output_rpm.fill(0.0f);
      previous_output_rpm_.fill(0.0f);
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "舵角候选选择失败；保持当前目标并停止四轮驱动");
      return;
    }

    const float maximum_final_steering_error =
      maximumSteeringFeedbackError(desired_selection.target_angle);
    if (maximum_final_steering_error >= kCoordinatedSwitchTriggerRad) {
      beginCoordinatedSwitch(desired_selection);
      (void)tickCoordinatedSwitch(
        command, output_angle, output_rpm);
      return;
    }

    const bool command_is_zero =
      std::hypot(vx, vy) < kLinearCommandDeadbandMps &&
      std::abs(yaw_rate) < kYawCommandDeadbandRadps;
    if (command_is_zero) {
      output_angle = target_memory_initialized_ ?
        commanded_steer_angle_ : current_steer_angle_;
      output_rpm.fill(0.0f);
      previous_output_rpm_.fill(0.0f);
      previous_allocated_twist_.fill(0.0f);
      return;
    }

    if (!target_memory_initialized_) {
      commanded_steer_angle_ = current_steer_angle_;
      target_memory_initialized_ = true;
    }

    SteeringSelection limited_selection = desired_selection;
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
      limited_selection.target_angle[wheel] = approachAngle(
        desired_selection.target_angle[wheel], commanded_steer_angle_[wheel],
        kMaxSteerTargetStepRad);
    }

    const AllocationResult allocation = solveAllocation(command, limited_selection);

    if (!allocation.success) {
      output_angle = commanded_steer_angle_;
      output_rpm.fill(0.0f);
      previous_output_rpm_.fill(0.0f);
      previous_allocated_twist_.fill(0.0f);
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "QP allocation failed; holding steering and commanding zero wheel speed");
      return;
    }

    const float drive_scale =
      driveScaleFromFinalSteeringError(maximum_final_steering_error);
    const float release_scale = nextPostAlignmentDriveScale();
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
      output_angle[wheel] = allocation.target_angle[wheel];

      // 轮速按实际反馈舵角投影，避免舵轮未到位时按理论目标角硬推。
      // 四轮再统一乘最终舵角误差比例，禁止某些轮子先推、其他轮子仍在摆舵。
      const Vector3 longitudinal = wheelLongitudinalRow(
        wheel, current_steer_angle_[wheel]);
      const float wheel_speed = dot(longitudinal, allocation.twist);
      output_rpm[wheel] = speedToRpm(wheel_speed) * drive_scale * release_scale;
    }

    commanded_steer_angle_ = allocation.target_angle;
    previous_output_rpm_ = output_rpm;
    previous_allocated_twist_ = allocation.twist;
    candidate_memory_initialized_ = true;
    previous_candidate_flipped_ = allocation.flipped;
    previous_drive_sign_ = allocation.drive_sign;

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "OMNI_B_QP: cmd=(%.3f,%.3f,%.3f) alloc=(%.3f,%.3f,%.3f) "
      "mode=%d active=%d steer_err_max=%.3f drive_scale=%.2f "
      "lateral_max=%.3f state=%s",
      command[0], command[1], command[2],
      allocation.twist[0], allocation.twist[1], allocation.twist[2],
      allocation.mode_index, allocation.active_constraint_count,
      maximum_final_steering_error, drive_scale, allocation.max_lateral_speed,
      coordinatedSwitchStateName());
  }

  // [ParkWheels] 轮序：0=FL, 1=FR, 2=RL, 3=RR；朝里 X：[+a,-a,-a,+a]
  static void fillInwardParkTargets(float magnitude, std::array<float, kWheelCount> & out)
  {
    const float a = std::min(std::abs(magnitude), kSteerAngleLimitRad - kSteerLimitMarginRad);
    out[0] = +a;
    out[1] = -a;
    out[2] = -a;
    out[3] = +a;
  }

  rclcpp_action::GoalResponse handleParkGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ParkWheels::Goal>)
  {
    if (park_active_.load()) {
      RCLCPP_WARN(get_logger(), "[ParkWheels] reject: already parking");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleParkCancel(
    const std::shared_ptr<GoalHandleParkWheels>)
  {
    RCLCPP_INFO(get_logger(), "[ParkWheels] cancel requested");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleParkAccepted(const std::shared_ptr<GoalHandleParkWheels> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    float a = goal->steer_angle_rad;
    float timeout = goal->timeout_sec;
    float tol = goal->settle_tolerance_rad;
    if (!(a > 0.0f)) {
      a = 0.785f;
    }
    if (!(timeout > 0.0f)) {
      timeout = 3.0f;
    }
    if (!(tol > 0.0f)) {
      tol = 0.08f;
    }

    {
      std::lock_guard<std::mutex> lock(park_mutex_);
      fillInwardParkTargets(a, park_targets_);
      park_tolerance_ = tol;
      park_deadline_ = now() + rclcpp::Duration::from_seconds(timeout);
      park_start_time_ = now();
      park_goal_handle_ = goal_handle;
      park_hold_active_.store(false);
      park_active_.store(true);
    }

    // 驻车优先级高于正在进行的协调换向；从此周期起四轮统一零速。
    unpark_state_ = UnparkState::kInactive;
    unpark_targets_initialized_ = false;
    unpark_align_stable_cycle_ = 0;
    coordinated_switch_state_ = CoordinatedSwitchState::kNormal;
    previous_output_rpm_.fill(0.0f);
    previous_allocated_twist_.fill(0.0f);
    resetCommandGovernor();

    RCLCPP_INFO(
      get_logger(),
      "[ParkWheels] start targets=[%.2f,%.2f,%.2f,%.2f] tol=%.3f timeout=%.2f",
      park_targets_[0], park_targets_[1], park_targets_[2], park_targets_[3],
      tol, timeout);
  }

  void finishPark(bool success, const std::string & message)
  {
    std::shared_ptr<GoalHandleParkWheels> goal_handle;
    rclcpp::Time start;
    {
      std::lock_guard<std::mutex> lock(park_mutex_);
      goal_handle = park_goal_handle_;
      start = park_start_time_;
      park_goal_handle_.reset();
      park_active_.store(false);
      park_hold_active_.store(success);
    }
    if (!goal_handle) {
      return;
    }

    auto result = std::make_shared<ParkWheels::Result>();
    result->success = success;
    result->message = message;
    result->total_elapsed_time = now() - start;

    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      RCLCPP_INFO(get_logger(), "[ParkWheels] canceled: %s", message.c_str());
      return;
    }
    if (success) {
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "[ParkWheels] succeeded: %s", message.c_str());
    } else {
      goal_handle->abort(result);
      RCLCPP_WARN(get_logger(), "[ParkWheels] aborted: %s", message.c_str());
    }
  }

  // 返回 true：本周期已写好驻车电机指令，调用方直接 publish，不要再跑 QP。
  bool tickParkMode(chassis_interfaces::msg::ChassisMotorControl & output)
  {
    std::shared_ptr<GoalHandleParkWheels> goal_handle;
    std::array<float, kWheelCount> targets{};
    float tol = 0.08f;
    rclcpp::Time deadline;
    {
      std::lock_guard<std::mutex> lock(park_mutex_);
      if (!park_active_.load() && !park_hold_active_.load()) {
        return false;
      }
      targets = park_targets_;
      tol = park_tolerance_;
      deadline = park_deadline_;
      goal_handle = park_goal_handle_;
    }

    output.position.resize(kWheelCount);
    output.speed.resize(kWheelCount);
    float max_err = 0.0f;
    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
      output.position[wheel] = targets[wheel];
      output.speed[wheel] = 0.0;
      max_err = std::max(
        max_err, std::abs(angleDiff(targets[wheel], current_steer_angle_[wheel])));
    }
    output.estop = 0;
    output.neutral = 0;

    // Action 成功后的锁存阶段只持续发布 X 姿态，不再重复发送 feedback/result。
    if (park_hold_active_.load() && !park_active_.load()) {
      commanded_steer_angle_ = targets;
      target_memory_initialized_ = true;
      previous_output_rpm_.fill(0.0f);
      return true;
    }

    if (goal_handle && goal_handle->is_canceling()) {
      finishPark(false, "canceled");
      return true;
    }

    auto feedback = std::make_shared<ParkWheels::Feedback>();
    feedback->max_steer_error = max_err;
    feedback->time_left = deadline - now();
    if (goal_handle) {
      goal_handle->publish_feedback(feedback);
    }

    if (max_err <= tol) {
      finishPark(true, "wheels settled in park pose");
      commanded_steer_angle_ = targets;
      target_memory_initialized_ = true;
      return true;
    }
    if (now() >= deadline) {
      finishPark(false, "timeout waiting for park steer settle");
      return true;
    }
    return true;
  }

  void publishControlCallback()
  {
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
    float vx_ref = 0.0f;
    float vy_ref = 0.0f;
    float yaw_rate_ref = 0.0f;

    if (command_timed_out) {
      rotation_dominant_mode_ = false;
      resetCommandGovernor();
    } else {
      vx_ref = static_cast<float>(command.linear.x);
      vy_ref = static_cast<float>(command.linear.y);
      yaw_rate_ref = static_cast<float>(command.angular.z);
      applyRotationDominantMode(vx_ref, vy_ref, yaw_rate_ref);
      applyCommandGovernor(
        vx_ref, vy_ref, yaw_rate_ref,
        vx, vy, yaw_rate);
    }

    // [ParkWheels] 保护期后只有明确的新运动才解除驻车。导航结束时平滑器的
    // 小幅减速尾巴不会再让刚启动的 X 驻车立即失败。
    const bool park_or_hold_active =
      park_active_.load() || park_hold_active_.load();
    const bool park_grace_elapsed = park_or_hold_active &&
      (now() - park_start_time_).seconds() > kParkPreemptGraceS;
    const bool meaningful_motion =
      std::hypot(vx_ref, vy_ref) > kParkPreemptLinearMps ||
      std::abs(yaw_rate_ref) > kParkPreemptYawRadps;
    if (park_or_hold_active && park_grace_elapsed && meaningful_motion) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[ParkWheels] 由明确非零 cmd_vel 解除驻车：reference linear=%.3f yaw=%.3f",
        std::hypot(vx_ref, vy_ref), std::abs(yaw_rate_ref));
      park_hold_active_.store(false);
      if (park_active_.load()) {
        finishPark(false, "preempted by meaningful cmd_vel");
      }
      beginUnparkAlignment();
      vx = 0.0f;
      vy = 0.0f;
      yaw_rate = 0.0f;
    }

    chassis_interfaces::msg::ChassisMotorControl output;
    output.header.stamp = now();
    output.header.frame_id = "base_link";

    if (tickParkMode(output)) {
      motor_control_pub_->publish(output);
      return;
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
      computeWheels(
        vx, vy, yaw_rate, Vector3{vx_ref, vy_ref, yaw_rate_ref},
        wheel_angle, wheel_rpm);
    }

    output.position.resize(kWheelCount);
    output.speed.resize(kWheelCount);

    for (std::size_t wheel = 0; wheel < kWheelCount; ++wheel) {
      output.position[wheel] = wheel_angle[wheel];
      output.speed[wheel] = wheel_rpm[wheel];
    }

    output.estop = 0;
    output.neutral = 0;
    motor_control_pub_->publish(output);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<chassis_interfaces::msg::ChassisMotorControl>::SharedPtr feedback_sub_;
  rclcpp::Publisher<chassis_interfaces::msg::ChassisMotorControl>::SharedPtr motor_control_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // [ParkWheels]
  rclcpp_action::Server<ParkWheels>::SharedPtr park_wheels_server_;
  std::mutex park_mutex_;
  std::atomic<bool> park_active_{false};
  std::atomic<bool> park_hold_active_{false};
  std::array<float, kWheelCount> park_targets_{};
  float park_tolerance_{0.08f};
  rclcpp::Time park_deadline_;
  rclcpp::Time park_start_time_;
  std::shared_ptr<GoalHandleParkWheels> park_goal_handle_;

  std::mutex command_mutex_;
  geometry_msgs::msg::Twist latest_cmd_;
  rclcpp::Time last_cmd_time_;

  std::array<float, kWheelCount> current_steer_angle_{};
  bool steering_feedback_received_{false};
  rclcpp::Time last_steering_feedback_time_;

  // 底盘反馈中的驱动轮实际转速，用于确认车体已经停止后才允许摆舵。
  std::array<float, kWheelCount> current_drive_rpm_{};
  bool drive_feedback_received_{false};
  rclcpp::Time last_drive_feedback_time_;

  // 上一周期实际发送给舵机的目标角，而不是反馈角。候选连续性必须以它为基准，
  // 否则舵机跟随滞后时，优化器可能误判另一套 angle+pi 分支更优。
  std::array<float, kWheelCount> commanded_steer_angle_{};
  bool target_memory_initialized_{false};

  float governed_vx_{0.0f};
  float governed_vy_{0.0f};
  float governed_yaw_rate_{0.0f};
  // jerk 限制器必须跨周期保存当前加速度，不能只保存上一周期速度。
  float governed_ax_{0.0f};
  float governed_ay_{0.0f};
  float governed_yaw_accel_{0.0f};
  bool command_governor_initialized_{false};
  int post_alignment_drive_ramp_cycle_{kPostAlignmentDriveRampCycles};

  Vector3 previous_allocated_twist_{};

  bool rotation_dominant_mode_{false};

  // X 驻车退出状态：发送启动方向对应的完整舵角，但驱动轮严格保持零速，
  // 直到四轮真实反馈连续到位。该状态只由 ParkWheels 的新运动抢占触发。
  UnparkState unpark_state_{UnparkState::kInactive};
  std::array<float, kWheelCount> unpark_target_angle_{};
  std::array<bool, kWheelCount> unpark_target_flipped_{};
  std::array<float, kWheelCount> unpark_target_drive_sign_{};
  std::array<float, kWheelCount> unpark_retarget_candidate_angle_{};
  std::array<bool, kWheelCount> unpark_retarget_candidate_flipped_{};
  std::array<float, kWheelCount> unpark_retarget_candidate_drive_sign_{};
  bool unpark_targets_initialized_{false};
  bool unpark_retarget_candidate_initialized_{false};
  int unpark_align_stable_cycle_{0};
  int unpark_retarget_stable_cycle_{0};

  // 整车协调换向状态：任何单轮需要跨大角度换分支时，四轮统一减速到零，
  // 停车摆舵并等待真实反馈对齐，避免单轮停转却被其他轮拖着横向摩地。
  CoordinatedSwitchState coordinated_switch_state_{
    CoordinatedSwitchState::kNormal};
  std::array<float, kWheelCount> pending_switch_angle_{};
  std::array<bool, kWheelCount> pending_switch_flipped_{};
  std::array<float, kWheelCount> pending_switch_drive_sign_{};
  // 协调换向期间的候选新方向。先经过“角度迟滞 + 连续周期确认”，再替换
  // pending_switch_*，避免 MPPI 的 vx/vy 小抖动让四轮永远无法确认到位。
  std::array<float, kWheelCount> retarget_candidate_angle_{};
  std::array<bool, kWheelCount> retarget_candidate_flipped_{};
  std::array<float, kWheelCount> retarget_candidate_drive_sign_{};
  bool retarget_candidate_initialized_{false};
  std::array<float, kWheelCount> transition_start_rpm_{};
  std::array<float, kWheelCount> previous_output_rpm_{};
  int coordinated_deceleration_cycle_{0};
  int coordinated_stop_stable_cycle_{0};
  int coordinated_align_stable_cycle_{0};
  int coordinated_retarget_stable_cycle_{0};

  std::array<bool, kWheelCount> previous_candidate_flipped_{};
  std::array<float, kWheelCount> previous_drive_sign_{};
  bool candidate_memory_initialized_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MobileOmniController>());
  rclcpp::shutdown();
  return 0;
}
