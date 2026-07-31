#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "chassis_interfaces/msg/chassis_motor_control.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_interfaces/action/steering_angle_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// 全向底盘参数定义（单位：mm，角度：rad）
#define CHASSIS_SIZE_L 460.0f  // 前后轮间距（wheelbase）
#define CHASSIS_SIZE_W 345.0f  // 左右轮间距（track）

#define WHEEL_DIAM 107.0f  // 轮径（mm）

#define SPEED_TO_RPM(x) ((x)*60.0f / (3.141592f * WHEEL_DIAM * 0.001f))

#define PI_CHASSIS 3.141592f
#define FRONT_DIRECTION_ANGLE 1.570796f  // 车头方向相对舵角零位偏移
#define ZERO_ANGLE_THETA 0.090625f       // 舵机零位机械偏置
#define STEER_ANGLE_LIMIT (PI_CHASSIS * 0.8f)  // 舵角限位（rad，默认90度）

#define STEER_MAX_STEP 6.28f  // rad / cycle
#define SPEED_RAMP_RATE 0.8f
#define ANGLE_EPS 1.0f

// 驻车完成后先保持一小段时间，避免导航速度平滑器残留的减速指令立即解除驻车。
static constexpr float PARK_PREEMPT_GRACE_SEC = 0.8f;
// 只有明确的新运动指令超过以下阈值，才认为新任务需要解除驻车。
static constexpr float PARK_PREEMPT_LINEAR_MPS = 0.001f;
static constexpr float PARK_PREEMPT_YAW_RADPS = 0.001f;
// 真正解除驻车保持后，必须先零轮速对舵。0.08rad 约为 4.6°，
// 与 SteeringAngleControl 默认到位容差一致，避免原 ANGLE_EPS=1rad
// 导致 45° 未对正就起步。
static constexpr float UNPARK_ALIGN_TOLERANCE_RAD = 0.08f;
// 连续多个控制周期到位才恢复轮速，过滤单帧反馈抖动。
static constexpr int UNPARK_ALIGN_STABLE_CYCLES = 3;

// static inline float wrap_pi(float ang)
// {
//     // wrap to (-pi, pi]
//     while (ang > PI_CHASSIS) ang -= 2.0f * PI_CHASSIS;
//     while (ang <= -PI_CHASSIS) ang += 2.0f * PI_CHASSIS;
//     return ang;
// }

static float wrap_pi(float a) {
    while (a > PI_CHASSIS) a -= 2.f * PI_CHASSIS;
    while (a <= -PI_CHASSIS) a += 2.f * PI_CHASSIS;
    return a;
}

static float angle_diff(float a, float b) { return wrap_pi(a - b); }

static float ramp_angle(float target, float current, float max_step) {
    float d = angle_diff(target, current);
    if (std::abs(d) <= max_step) return target;
    return current + std::copysign(max_step, d);
}

class MobileOmniController : public rclcpp::Node {
public:
    using SteeringAngleControl = nav_interfaces::action::SteeringAngleControl;
    using GoalHandleSteeringAngleControl =
        rclcpp_action::ServerGoalHandle<SteeringAngleControl>;

    MobileOmniController() : Node("mobile_cmd_omni") {
        RCLCPP_INFO(this->get_logger(), "Mobile Omni Controller initialized");

        // 初始化前一帧的状态（用于连续性判断）
        for (int i = 0; i < 4; ++i) {
            prev_angle_[i] = 0.0f;
            prev_speed_rpm_[i] = 0.0f;
        }
        prev_vx_ = 0.0f;

        // 订阅 /control/cmd_vel
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/control/cmd_vel_omni", 10, std::bind(&MobileOmniController::cmdVelCallback, this, std::placeholders::_1));

        // 订阅底盘反馈：用于用真实反馈作为 prev_angle_/prev_speed_rpm_ 的平滑基准
        feedback_sub_ = this->create_subscription<chassis_interfaces::msg::ChassisMotorControl>(
            "/feedback/omni_chassis_motor", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
            std::bind(&MobileOmniController::feedbackCallback, this, std::placeholders::_1));

        // 发布 /control/omni_chassis_motors
        motor_control_pub_ =
            this->create_publisher<chassis_interfaces::msg::ChassisMotorControl>("/control/omni_chassis_motors", 10);

        // 提供 /steering_angle_control Action Server。
        // 驻车指令和正常底盘指令由同一节点发布，
        // 避免两个节点同时写 /control/omni_chassis_motors 导致指令互相覆盖。
        steering_angle_control_server_ =
            rclcpp_action::create_server<SteeringAngleControl>(
            this, "steering_angle_control",
            std::bind(&MobileOmniController::handleParkGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MobileOmniController::handleParkCancel, this, std::placeholders::_1),
            std::bind(&MobileOmniController::handleParkAccepted, this, std::placeholders::_1));

        // 初始化缓存指令与时间戳
        latest_cmd_ = geometry_msgs::msg::Twist();
        last_cmd_time_ = this->now();

        // 20Hz 定时器：定时解算并发布电机控制指令
        using namespace std::chrono_literals;
        timer_ = this->create_wall_timer(50ms, std::bind(&MobileOmniController::publishControlCallback, this));
        RCLCPP_INFO(
            this->get_logger(),
            "[ControlTiming] 已启用底盘控制耗时统计，车辆运动时每 1 秒输出一次平均值和最大值");
    }

private:
    void feedbackCallback(const chassis_interfaces::msg::ChassisMotorControl::SharedPtr msg) {
        if (msg->position.size() >= 4) {
            for (int i = 0; i < 4; ++i) {
                prev_angle_[i] = static_cast<float>(msg->position[i]);
            }
            steering_feedback_received_ = true;
        }
        if (msg->speed.size() >= 4) {
            for (int i = 0; i < 4; ++i) {
                // 反馈话题里的 speed 通常是轮线速度(m/s) 或 rpm，需与控制侧保持一致。
                // 这里按 rpm 使用；若你的反馈是 m/s，请改为 SPEED_TO_RPM(msg->speed[i]).
                prev_speed_rpm_[i] = static_cast<float>(msg->speed[i]);
            }
        }
    }

    rclcpp_action::GoalResponse handleParkGoal(
        const rclcpp_action::GoalUUID&,
        std::shared_ptr<const SteeringAngleControl::Goal> goal) {
        {
            std::lock_guard<std::mutex> lock(park_mutex_);
            if (park_active_) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "[SteeringAngleControl] 已有驻车任务正在执行，拒绝新任务");
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        // 驻车目标同样必须满足本文件统一配置的舵角机械限位。
        for (std::size_t wheel = 0; wheel < park_targets_.size(); ++wheel) {
            const float target = goal->target_steer_angles_rad[wheel];
            if (!std::isfinite(target) || std::abs(target) > STEER_ANGLE_LIMIT) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "[SteeringAngleControl] 第 %zu 个车轮目标角 %.3f rad "
                    "超出舵角限位 [%.3f, %.3f]，拒绝任务",
                    wheel, target, -STEER_ANGLE_LIMIT, STEER_ANGLE_LIMIT);
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleParkCancel(
        const std::shared_ptr<GoalHandleSteeringAngleControl>) {
        RCLCPP_INFO(
            this->get_logger(), "[SteeringAngleControl] 收到取消驻车请求");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleParkAccepted(
        const std::shared_ptr<GoalHandleSteeringAngleControl> goal_handle) {
        const auto goal = goal_handle->get_goal();
        float timeout = goal->timeout_sec;
        float tolerance = goal->settle_tolerance_rad;
        if (!(timeout > 0.0f)) {
            timeout = 3.0f;
        }
        if (!(tolerance > 0.0f)) {
            tolerance = 0.08f;
        }

        {
            std::lock_guard<std::mutex> lock(park_mutex_);
            park_targets_ = goal->target_steer_angles_rad;
            park_tolerance_ = tolerance;
            park_start_time_ = this->now();
            park_deadline_ = park_start_time_ + rclcpp::Duration::from_seconds(timeout);
            park_goal_handle_ = goal_handle;
            park_hold_active_ = false;
            park_active_ = true;
            // 新的 SteeringAngleControl 指令优先级高于正在进行的驻车恢复对舵。
            unpark_align_active_ = false;
            unpark_align_stable_cycles_ = 0;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "[SteeringAngleControl] 开始执行，"
            "目标=[%.3f, %.3f, %.3f, %.3f] rad，"
            "容差=%.3f rad，超时=%.2f s",
            goal->target_steer_angles_rad[0], goal->target_steer_angles_rad[1],
            goal->target_steer_angles_rad[2], goal->target_steer_angles_rad[3],
            tolerance, timeout);
    }

    void finishPark(bool success, const std::string& message) {
        std::shared_ptr<GoalHandleSteeringAngleControl> goal_handle;
        rclcpp::Time start_time;
        {
            std::lock_guard<std::mutex> lock(park_mutex_);
            goal_handle = park_goal_handle_;
            start_time = park_start_time_;
            park_goal_handle_.reset();
            park_active_ = false;
            // 驻车成功后始终锁存零轮速与目标舵角，直到明确的新运动指令解除。
            park_hold_active_ = success;
        }

        if (!goal_handle) {
            return;
        }

        auto result = std::make_shared<SteeringAngleControl::Result>();
        result->success = success;
        result->message = message;
        result->total_elapsed_time = this->now() - start_time;

        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
            RCLCPP_INFO(
                this->get_logger(), "[SteeringAngleControl] 任务已取消：%s",
                message.c_str());
        } else if (success) {
            goal_handle->succeed(result);
            RCLCPP_INFO(
                this->get_logger(), "[SteeringAngleControl] 任务完成：%s",
                message.c_str());
        } else {
            goal_handle->abort(result);
            RCLCPP_WARN(
                this->get_logger(), "[SteeringAngleControl] 驻车失败：%s",
                message.c_str());
        }
    }

    // 返回 true 表示本周期已生成驻车指令，调用方应直接发布，不再执行正常运动解算。
    bool tickParkMode(chassis_interfaces::msg::ChassisMotorControl& output) {
        std::array<float, 4> targets{};
        std::shared_ptr<GoalHandleSteeringAngleControl> goal_handle;
        float tolerance = 0.08f;
        rclcpp::Time deadline;
        bool park_active = false;
        bool park_hold_active = false;
        {
            std::lock_guard<std::mutex> lock(park_mutex_);
            park_active = park_active_;
            park_hold_active = park_hold_active_;
            if (!park_active && !park_hold_active) {
                return false;
            }
            targets = park_targets_;
            tolerance = park_tolerance_;
            deadline = park_deadline_;
            goal_handle = park_goal_handle_;
        }

        output.position.resize(4);
        output.speed.resize(4);
        float max_error = steering_feedback_received_
                              ? 0.0f
                              : std::numeric_limits<float>::infinity();
        for (std::size_t wheel = 0; wheel < targets.size(); ++wheel) {
            // 驻车期间始终直接下发目标舵角，同时强制四个驱动轮转速为零。
            output.position[wheel] = targets[wheel];
            output.speed[wheel] = 0.0;
            if (steering_feedback_received_) {
                max_error = std::max(
                    max_error, std::abs(angle_diff(targets[wheel], prev_angle_[wheel])));
            }
        }
        output.estop = 0;
        output.neutral = 0;

        // Action 成功后进入保持状态；保持阶段只持续输出驻车姿态，
        // 不再重复发送反馈和结果。
        if (park_hold_active && !park_active) {
            return true;
        }

        if (goal_handle && goal_handle->is_canceling()) {
            finishPark(false, "驻车任务被取消");
            return true;
        }

        auto feedback = std::make_shared<SteeringAngleControl::Feedback>();
        feedback->max_steer_error = max_error;
        feedback->time_left = deadline - this->now();
        if (goal_handle) {
            goal_handle->publish_feedback(feedback);
        }

        if (steering_feedback_received_ && max_error <= tolerance) {
            finishPark(true, "四轮舵角已到达驻车位置");
        } else if (this->now() >= deadline) {
            finishPark(false, "等待驻车舵角到位超时");
        }
        return true;
    }

    static bool isMeaningfulMotion(float vx, float vy, float vw) {
        return std::hypot(vx, vy) > PARK_PREEMPT_LINEAR_MPS ||
               std::abs(vw) > PARK_PREEMPT_YAW_RADPS;
    }

    // 真正驻车解除后的独立恢复状态机：
    // 1. 按解除驻车的运动方向解算四轮目标角；
    // 2. 对舵期间四个驱动轮始终保持零速；
    // 3. 真实舵角反馈连续到位后，本周期仍发布零速，下一周期才恢复正常轮速。
    //
    // 该状态由驻车保持被新的运动指令解除时触发。
    bool tickUnparkAlignment(
        float vx, float vy, float vw,
        chassis_interfaces::msg::ChassisMotorControl& output) {
        geometry_msgs::msg::Twist reference_cmd;
        {
            std::lock_guard<std::mutex> lock(park_mutex_);
            if (!unpark_align_active_) {
                return false;
            }

            // 遥控器或导航在对舵期间可能改变运动方向。只有当前指令仍是明确
            // 运动指令时才更新参考方向；短暂回零时沿用触发解除驻车的方向，
            // 避免目标舵角在对舵过程中突然跳回 atan2(0,0) 的零角。
            if (isMeaningfulMotion(vx, vy, vw)) {
                unpark_reference_cmd_ = geometry_msgs::msg::Twist();
                unpark_reference_cmd_.linear.x = vx;
                unpark_reference_cmd_.linear.y = vy;
                unpark_reference_cmd_.angular.z = vw;
            }
            reference_cmd = unpark_reference_cmd_;
        }

        float target_angle[4];
        float ignored_speed_rpm[4];
        compute_wheels(
            static_cast<float>(reference_cmd.linear.x),
            static_cast<float>(reference_cmd.linear.y),
            static_cast<float>(reference_cmd.angular.z),
            target_angle,
            ignored_speed_rpm);

        output.position.resize(4);
        output.speed.resize(4);
        float max_error = steering_feedback_received_
                              ? 0.0f
                              : std::numeric_limits<float>::infinity();
        for (int wheel = 0; wheel < 4; ++wheel) {
            output.position[wheel] = target_angle[wheel];
            // 驻车恢复最重要的安全约束：舵角没有稳定到位前绝不输出驱动轮速度。
            output.speed[wheel] = 0.0f;
            if (steering_feedback_received_) {
                max_error = std::max(
                    max_error,
                    std::abs(angle_diff(target_angle[wheel], prev_angle_[wheel])));
            }
        }
        output.estop = 0;
        output.neutral = 0;

        bool alignment_complete = false;
        int stable_cycles = 0;
        {
            std::lock_guard<std::mutex> lock(park_mutex_);
            if (steering_feedback_received_ &&
                max_error <= UNPARK_ALIGN_TOLERANCE_RAD) {
                ++unpark_align_stable_cycles_;
            } else {
                unpark_align_stable_cycles_ = 0;
            }
            stable_cycles = unpark_align_stable_cycles_;
            if (unpark_align_stable_cycles_ >= UNPARK_ALIGN_STABLE_CYCLES) {
                unpark_align_active_ = false;
                unpark_align_stable_cycles_ = 0;
                alignment_complete = true;
            }
        }

        if (alignment_complete) {
            RCLCPP_INFO(
                this->get_logger(),
                "[Unpark] 四轮已连续 %d 个周期对正，最大误差=%.3f rad；"
                "本周期保持零轮速，下一周期恢复正常解算",
                UNPARK_ALIGN_STABLE_CYCLES, max_error);
        } else {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 500,
                "[Unpark] 零轮速对舵中：最大误差=%.3f rad，稳定周期=%d/%d",
                max_error, stable_cycles, UNPARK_ALIGN_STABLE_CYCLES);
        }
        return true;
    }

    // 驻车后只有明确的新运动指令才允许恢复，防止导航结束时的速度尾巴误解除。
    void releaseParkForNewMotion(float vx, float vy, float vw) {
        bool should_release = false;
        bool action_in_progress = false;
        bool start_unpark_alignment = false;
        {
            std::lock_guard<std::mutex> lock(park_mutex_);
            const bool parking = park_active_ || park_hold_active_;
            const bool grace_elapsed =
                parking && (this->now() - park_start_time_).seconds() > PARK_PREEMPT_GRACE_SEC;
            const bool meaningful_motion = isMeaningfulMotion(vx, vy, vw);
            should_release = grace_elapsed && meaningful_motion;
            action_in_progress = park_active_;
            if (should_release) {
                // 驻车保持或正在执行的驻车被新运动解除时，先零速对舵。
                start_unpark_alignment = parking;
                if (!action_in_progress) {
                    park_hold_active_ = false;
                }
                if (start_unpark_alignment) {
                    unpark_align_active_ = true;
                    unpark_align_stable_cycles_ = 0;
                    unpark_reference_cmd_ = geometry_msgs::msg::Twist();
                    unpark_reference_cmd_.linear.x = vx;
                    unpark_reference_cmd_.linear.y = vy;
                    unpark_reference_cmd_.angular.z = vw;
                }
            }
        }

        if (!should_release) {
            return;
        }

        if (action_in_progress) {
            finishPark(false, "驻车被新的运动指令抢占");
        }

        if (start_unpark_alignment) {
            RCLCPP_INFO(
                this->get_logger(),
                "[Unpark] 检测到新的运动指令，解除真实驻车并进入零轮速对舵："
                "linear=%.3f m/s，yaw=%.3f rad/s，容差=%.3f rad",
                std::hypot(vx, vy), std::abs(vw),
                UNPARK_ALIGN_TOLERANCE_RAD);
        } else {
            RCLCPP_INFO(
                this->get_logger(),
                "[SteeringAngleControl] 驻车被运动指令抢占，恢复正常控制："
                "linear=%.3f m/s，yaw=%.3f rad/s",
                std::hypot(vx, vy), std::abs(vw));
        }
    }

    // 常量定义
    // static constexpr float SPEED_RAMP_RATE = 0.8f;         // 速度加速度系数 (0-1)，越小过渡越平滑
    // static constexpr float ANGLE_TOLERANCE = 0.01f;        // 角度容差（rad），判断是否接近目标角度

    // 计算两个角度之间的最小差值（考虑角度的周期性）
    static float angle_diff(float target, float current) {
        float diff = target - current;
        // 将差值规范化到 [-pi, pi]
        while (diff > PI_CHASSIS) diff -= 2.0f * PI_CHASSIS;
        while (diff < -PI_CHASSIS) diff += 2.0f * PI_CHASSIS;
        return diff;
    }

    // 选择最优角度（考虑与前一帧的连续性）
    // 当vx接近0时，atan2(vy, vx)可能返回±pi/2，需要选择与前一帧最接近的
    static float select_optimal_angle(float angle_candidate, float prev_angle, float vx, float vy) {
        // 只有当角度接近±pi/2时才需要考虑多值性
        // 判断是否接近±pi/2（容差0.1）
        float abs_angle = std::abs(angle_candidate);
        if (abs_angle > STEER_ANGLE_LIMIT - 0.1f && abs_angle < STEER_ANGLE_LIMIT + 0.1f) {
            // 接近±pi/2，考虑两个候选值
            float candidate1 = STEER_ANGLE_LIMIT;
            float candidate2 = -STEER_ANGLE_LIMIT;

            // 计算两个候选角度与前一帧的差值
            float diff1 = std::abs(angle_diff(candidate1, prev_angle));
            float diff2 = std::abs(angle_diff(candidate2, prev_angle));

            // 选择差值较小的角度（保持连续性）
            if (diff2 < diff1) {
                return candidate2;
            } else {
                return candidate1;
            }
        }

        // 不接近±pi/2，直接返回
        return angle_candidate;
    }

    // 检查是否跨过x=0（x方向速度符号改变）
    // 返回值：true 表示需要平滑过渡，false 表示可以直接执行
    static bool is_crossing_zero(float prev_vx, float curr_vx) {
        // 当x速度从正到0或从负到0，或直接越过0（正到负、负到正）都视为跨过0
        if ((prev_vx > 0.0f && curr_vx <= 0.0f) || (prev_vx < 0.0f && curr_vx >= 0.0f)) {
            return true;
        }
        return false;
    }

    void compute_wheels(float vx, float vy, float vw, float out_angle[4], float out_speed_rpm[4]) {
        const float L = CHASSIS_SIZE_L * 0.001f;
        const float W = CHASSIS_SIZE_W * 0.001f;

        float target_ang[4];
        float target_rpm[4];
        float ang_err[4];

        /* ===============================
         * Phase 0：统一解算目标（纯几何）
         * =============================== */
        for (int i = 0; i < 4; ++i) {
            const bool is_right = (i == 1 || i == 3);
            const bool is_front = (i == 0 || i == 1);

            const float kx = is_right ? +1.f : -1.f;
            const float ky = is_front ? +1.f : -1.f;

            float vx_i = vx + kx * W * vw * 0.5f;
            float vy_i = vy + ky * L * vw * 0.5f;

            float speed = std::hypot(vx_i, vy_i);
            float ang = std::atan2(vy_i, vx_i);

            /* π 等价两解 */
            float angA = wrap_pi(ang);
            float spdA = speed;

            float angB = wrap_pi(ang + PI_CHASSIS);
            float spdB = -speed;

            bool validA = (std::abs(angA) <= STEER_ANGLE_LIMIT);
            bool validB = (std::abs(angB) <= STEER_ANGLE_LIMIT);

            float costA = validA ? std::abs(angle_diff(angA, prev_angle_[i])) : 1e6f;
            float costB = validB ? std::abs(angle_diff(angB, prev_angle_[i])) : 1e6f;

            if (costA <= costB) {
                target_ang[i] = angA;
                target_rpm[i] = SPEED_TO_RPM(spdA);
            } else {
                target_ang[i] = angB;
                target_rpm[i] = SPEED_TO_RPM(spdB);
            }

            ang_err[i] = std::abs(angle_diff(target_ang[i], prev_angle_[i]));
        }

        /* =========================================
         * Phase 1：全局仲裁（是否允许产生线速度）
         * ========================================= */
        bool steering_done = true;
        for (int i = 0; i < 4; ++i) {
            if (ang_err[i] > ANGLE_EPS) {
                steering_done = false;
                break;
            }
        }

        /* =========================================
         * Phase 2：统一输出（Two-Phase 核心）
         * ========================================= */
        for (int i = 0; i < 4; ++i) {
            /* 舵角：始终允许转 */
            out_angle[i] = ramp_angle(target_ang[i], prev_angle_[i],
                                      STEER_MAX_STEP  // 20Hz
            );

            if (!steering_done) {
                /* 🚫 转向阶段：所有轮强制停速 */
                out_speed_rpm[i] = 0.0f;
            } else {
                /* ✅ 行走阶段：所有轮一起加速 */
                float diff = target_rpm[i] - prev_speed_rpm_[i];
                out_speed_rpm[i] = prev_speed_rpm_[i] + diff * SPEED_RAMP_RATE;
            }

            // 如需右侧轮反向，在这里统一处理
            // if (i == 1 || i == 3) out_speed_rpm[i] = -out_speed_rpm[i];
        }
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // 订阅回调中只缓存最新指令，不做解算与发布
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        latest_cmd_ = *msg;
        last_cmd_time_ = this->now();
    }

    // 单项耗时统计器。使用 steady_clock 统计真实经过时间，不受 ROS 时间跳变影响。
    struct TimingMetric {
        double sum_ms{0.0};
        double max_ms{0.0};
        std::uint64_t count{0};

        void add(double value_ms) {
            if (!std::isfinite(value_ms)) {
                return;
            }
            sum_ms += value_ms;
            max_ms = std::max(max_ms, value_ms);
            ++count;
        }

        double average() const {
            return count > 0 ? sum_ms / static_cast<double>(count) : 0.0;
        }
    };

    struct ControlTimingStatistics {
        TimingMetric command_age;
        TimingMetric callback_period;
        TimingMetric wheel_solver;
        TimingMetric publish;
        TimingMetric callback_total;
    };

    void outputControlTimingStatistics(const char* window_reason) {
        if (timing_statistics_.wheel_solver.count == 0) {
            return;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "[ControlTiming] 窗口=%s，样本=%llu，"
            "指令等待(收到cmd->控制回调) avg/max=%.3f/%.3f ms，"
            "控制周期 avg/max=%.3f/%.3f ms，compute_wheels解算 avg/max=%.3f/%.3f ms，"
            "publish调用 avg/max=%.3f/%.3f ms，控制回调总耗时 avg/max=%.3f/%.3f ms",
            window_reason,
            static_cast<unsigned long long>(timing_statistics_.wheel_solver.count),
            timing_statistics_.command_age.average(),
            timing_statistics_.command_age.max_ms,
            timing_statistics_.callback_period.average(),
            timing_statistics_.callback_period.max_ms,
            timing_statistics_.wheel_solver.average(),
            timing_statistics_.wheel_solver.max_ms,
            timing_statistics_.publish.average(),
            timing_statistics_.publish.max_ms,
            timing_statistics_.callback_total.average(),
            timing_statistics_.callback_total.max_ms);
        timing_statistics_ = ControlTimingStatistics{};
    }

    // 只在车辆收到有效非零运动指令时统计，并按 1 秒窗口输出平均/最大耗时。
    // 不逐周期打印日志，避免日志 IO 本身干扰 20Hz 控制回调的测试结果。
    void recordControlTiming(
        bool motion_command_active,
        double command_age_ms,
        double callback_period_ms,
        double wheel_solver_ms,
        double publish_ms,
        double callback_total_ms) {
        const auto now = std::chrono::steady_clock::now();
        if (!motion_command_active) {
            if (timing_window_active_) {
                // 短时测试可能不足 1 秒；运动结束时也输出最后一个不完整窗口，
                // 确保“只发一小会速度”仍然能看到耗时结果。
                outputControlTimingStatistics("运动结束");
            }
            timing_window_active_ = false;
            timing_statistics_ = ControlTimingStatistics{};
            return;
        }

        if (!timing_window_active_) {
            timing_window_active_ = true;
            timing_window_start_ = now;
            timing_statistics_ = ControlTimingStatistics{};
        }

        timing_statistics_.command_age.add(command_age_ms);
        timing_statistics_.callback_period.add(callback_period_ms);
        timing_statistics_.wheel_solver.add(wheel_solver_ms);
        timing_statistics_.publish.add(publish_ms);
        timing_statistics_.callback_total.add(callback_total_ms);

        const double window_ms =
            std::chrono::duration<double, std::milli>(
                now - timing_window_start_).count();
        if (window_ms < 1000.0) {
            return;
        }

        outputControlTimingStatistics("1秒周期");
        timing_window_start_ = now;
    }

    void publishControlCallback() {
        const auto callback_begin = std::chrono::steady_clock::now();
        double callback_period_ms = std::numeric_limits<double>::quiet_NaN();
        if (last_control_callback_valid_) {
            callback_period_ms =
                std::chrono::duration<double, std::milli>(
                    callback_begin - last_control_callback_begin_).count();
        }
        last_control_callback_begin_ = callback_begin;
        last_control_callback_valid_ = true;

        // 判断舵机是否到位
        bool steering_done = true;

        // 20Hz 定时解算并发布
        geometry_msgs::msg::Twist cmd;
        rclcpp::Time cmd_time;
        {
            std::lock_guard<std::mutex> lk(cmd_mutex_);
            cmd = latest_cmd_;
            cmd_time = last_cmd_time_;
        }

        // 指令等待时间用于判断延迟发生在解算前，还是 compute_wheels 内部。
        const rclcpp::Time control_ros_time = this->now();
        const double command_age_ms =
            (control_ros_time - cmd_time).seconds() * 1000.0;

        // 简单超时保护：超过 0.5s 未收到新指令，则自动置零
        const double timeout_sec = 0.5;
        const bool command_timed_out =
            command_age_ms > timeout_sec * 1000.0;
        if (command_timed_out) {
            cmd = geometry_msgs::msg::Twist();
        }

        const float vx = static_cast<float>(cmd.linear.x);
        const float vy = static_cast<float>(cmd.linear.y);
        const float vw = static_cast<float>(cmd.angular.z);

        // 新任务的明确非零速度可以解除驻车；驻车保护期内的减速尾巴不会触发解除。
        releaseParkForNewMotion(vx, vy, vw);

        auto motor_control_msg = chassis_interfaces::msg::ChassisMotorControl();
        motor_control_msg.header.stamp = this->now();
        motor_control_msg.header.frame_id = "base_link";

        // 驻车优先级高于正常速度解算。驻车期间直接发布零轮速和指定舵角。
        if (tickParkMode(motor_control_msg)) {
            motor_control_pub_->publish(motor_control_msg);
            // 驻车路径没有进入正常 compute_wheels，清空统计窗口，避免与正常
            // 行驶样本混在同一组平均值中。
            recordControlTiming(false, 0.0, 0.0, 0.0, 0.0, 0.0);
            return;
        }

        // 真正驻车解除后先进入独立恢复状态：按当前运动方向对舵，但轮速保持为零。
        // 对舵稳定完成的下一周期才会落入后面的正常解算。
        if (tickUnparkAlignment(vx, vy, vw, motor_control_msg)) {
            motor_control_pub_->publish(motor_control_msg);
            // 驻车恢复属于零轮速对舵状态，不计入正常行驶解算耗时。
            recordControlTiming(false, 0.0, 0.0, 0.0, 0.0, 0.0);
            return;
        }

        float angle[4];
        float speed_rpm[4];

        // 正常底盘解算前后使用单调时钟打点，单独统计 compute_wheels 的耗时。
        const auto wheel_solver_begin = std::chrono::steady_clock::now();
        compute_wheels(vx, vy, vw, angle, speed_rpm);
        const auto wheel_solver_end = std::chrono::steady_clock::now();
        const double wheel_solver_ms =
            std::chrono::duration<double, std::milli>(
                wheel_solver_end - wheel_solver_begin).count();

        motor_control_msg.position.resize(4);
        motor_control_msg.speed.resize(4);
        for (int i = 0; i < 4; ++i) {
            motor_control_msg.position[i] = angle[i];
            motor_control_msg.speed[i] = speed_rpm[i];
        }

        motor_control_msg.estop = 0;
        motor_control_msg.neutral = 0;

        const auto publish_begin = std::chrono::steady_clock::now();
        motor_control_pub_->publish(motor_control_msg);
        const auto callback_end = std::chrono::steady_clock::now();

        const double publish_ms =
            std::chrono::duration<double, std::milli>(
                callback_end - publish_begin).count();
        const double callback_total_ms =
            std::chrono::duration<double, std::milli>(
                callback_end - callback_begin).count();
        const bool motion_command_active =
            !command_timed_out &&
            (std::hypot(vx, vy) > 1.0e-4f || std::abs(vw) > 1.0e-4f);
        recordControlTiming(
            motion_command_active,
            command_age_ms,
            callback_period_ms,
            wheel_solver_ms,
            publish_ms,
            callback_total_ms);
    }

    // 成员变量
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<chassis_interfaces::msg::ChassisMotorControl>::SharedPtr feedback_sub_;
    rclcpp::Publisher<chassis_interfaces::msg::ChassisMotorControl>::SharedPtr motor_control_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 底盘解算性能统计状态。只影响日志，不参与控制结果计算。
    std::chrono::steady_clock::time_point last_control_callback_begin_{};
    bool last_control_callback_valid_{false};
    std::chrono::steady_clock::time_point timing_window_start_{};
    bool timing_window_active_{false};
    ControlTimingStatistics timing_statistics_{};

    // /steering_angle_control Action 状态。执行中负责等待舵角反馈，
    // 成功后持续保持目标姿态。
    rclcpp_action::Server<SteeringAngleControl>::SharedPtr
        steering_angle_control_server_;
    std::mutex park_mutex_;
    bool park_active_{false};
    bool park_hold_active_{false};
    std::array<float, 4> park_targets_{};
    float park_tolerance_{0.08f};
    rclcpp::Time park_deadline_;
    rclcpp::Time park_start_time_;
    std::shared_ptr<GoalHandleSteeringAngleControl> park_goal_handle_;
    // 驻车保持被新运动解除后，先零轮速对舵再恢复驱动。
    bool unpark_align_active_{false};
    int unpark_align_stable_cycles_{0};
    geometry_msgs::msg::Twist unpark_reference_cmd_;

    // 缓存最新的速度指令
    std::mutex cmd_mutex_;
    geometry_msgs::msg::Twist latest_cmd_;
    rclcpp::Time last_cmd_time_;

    // 历史状态：用于保持角度和速度的连续性
    bool steering_feedback_received_{false};  // 收到真实舵角反馈后才允许判定驻车完成
    float prev_angle_[4];      // 前一帧的舵角
    float prev_speed_rpm_[4];  // 前一帧的速度
    float prev_vx_;            // 前一帧的x方向速度（用于检测是否跨过0）
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MobileOmniController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
