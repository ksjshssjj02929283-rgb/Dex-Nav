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
#define STEER_ANGLE_LIMIT (PI_CHASSIS * 0.8f)  // 舵角限位（rad，约144度）

#define STEER_MAX_STEP 6.28f  // rad / cycle
#define SPEED_RAMP_RATE 0.8f
#define ANGLE_EPS 1.0f

// [Layer 1][Zero Command Contract]
// region 对“零 Twist”的解释仅是安全停轮并保持当前舵角，不包含驻车策略。
static constexpr float ZERO_CMD_LINEAR_EPS = 1.0e-4f;
static constexpr float ZERO_CMD_YAW_EPS = 1.0e-4f;

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

        // [Layer 1][Steering Primitive]
        // 提供一次性 /steering_angle_control Action Server。
        // 静止对舵和正常底盘指令由同一节点发布，
        // 避免两个节点同时写 /control/omni_chassis_motors 导致指令互相覆盖。
        steering_angle_control_server_ =
            rclcpp_action::create_server<SteeringAngleControl>(
            this, "steering_angle_control",
            std::bind(&MobileOmniController::handleSteeringGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MobileOmniController::handleSteeringCancel, this, std::placeholders::_1),
            std::bind(&MobileOmniController::handleSteeringAccepted, this, std::placeholders::_1));

        // 初始化缓存指令与时间戳
        latest_cmd_ = geometry_msgs::msg::Twist();
        last_cmd_time_ = this->now();

        // [Layer 1][Region Control Frequency]
        // 100Hz 定时器：降低 Twist 缓存等待时间；不改变底盘几何解算职责。
        using namespace std::chrono_literals;
        timer_ = this->create_wall_timer(10ms, std::bind(&MobileOmniController::publishControlCallback, this));
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
            last_steering_feedback_time_ = this->now();
        }
        if (msg->speed.size() >= 4) {
            for (int i = 0; i < 4; ++i) {
                // 反馈话题里的 speed 通常是轮线速度(m/s) 或 rpm，需与控制侧保持一致。
                // 这里按 rpm 使用；若你的反馈是 m/s，请改为 SPEED_TO_RPM(msg->speed[i]).
                prev_speed_rpm_[i] = static_cast<float>(msg->speed[i]);
            }
        }
    }

    // [Layer 1][Steering Primitive] 只校验执行条件，不判断起步或驻车用途。
    rclcpp_action::GoalResponse handleSteeringGoal(
        const rclcpp_action::GoalUUID&,
        std::shared_ptr<const SteeringAngleControl::Goal> goal) {
        {
            std::lock_guard<std::mutex> lock(steering_action_mutex_);
            if (steering_action_active_) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "[SteeringAngleControl] 已有静止对舵任务正在执行，拒绝新任务");
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        // Action 目标与普通解算共用同一机械舵角限位。
        for (std::size_t wheel = 0; wheel < steering_action_targets_.size(); ++wheel) {
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

    rclcpp_action::CancelResponse handleSteeringCancel(
        const std::shared_ptr<GoalHandleSteeringAngleControl>) {
        RCLCPP_INFO(
            this->get_logger(), "[SteeringAngleControl] 收到取消静止对舵请求");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleSteeringAccepted(
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
            std::lock_guard<std::mutex> lock(steering_action_mutex_);
            steering_action_targets_ = goal->target_steer_angles_rad;
            steering_action_tolerance_ = tolerance;
            steering_action_start_time_ = this->now();
            steering_action_deadline_ =
                steering_action_start_time_ + rclcpp::Duration::from_seconds(timeout);
            steering_action_goal_handle_ = goal_handle;
            // 新 Action 接管此前可能存在的目标保持；执行期间仍由 Action 分支输出。
            steering_action_hold_active_ = false;
            steering_action_active_ = true;
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

    void finishSteeringAction(bool success, const std::string& message) {
        std::shared_ptr<GoalHandleSteeringAngleControl> goal_handle;
        rclcpp::Time start_time;
        bool target_hold_enabled = false;
        {
            std::lock_guard<std::mutex> lock(steering_action_mutex_);
            goal_handle = steering_action_goal_handle_;
            start_time = steering_action_start_time_;
            steering_action_goal_handle_.reset();
            steering_action_active_ = false;
            // [Layer 1][Steering Primitive Handoff]
            // 只有真正成功的目标才进入零速保持。超时或取消不能锁存一个尚未到位的
            // 角度。保持只负责电机输出连续性，首条新非零 Twist 会自动解除。
            target_hold_enabled =
                success && goal_handle && !goal_handle->is_canceling();
            steering_action_hold_active_ = target_hold_enabled;
            if (target_hold_enabled) {
                steering_action_hold_start_time_ = this->now();
            }
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
                this->get_logger(),
                "[SteeringAngleControl] 任务完成并锁存目标舵角：%s；"
                "收到新的非零 Twist 后自动解除",
                message.c_str());
        } else {
            goal_handle->abort(result);
            RCLCPP_WARN(
                this->get_logger(), "[SteeringAngleControl] 静止对舵失败：%s",
                message.c_str());
        }
    }

    // [Layer 1][Steering Primitive]
    // 返回 true 表示本周期由 Action 生成“目标舵角 + 四轮零速”，
    // 调用方直接发布，不执行普通 Twist 解算。
    bool tickSteeringAction(chassis_interfaces::msg::ChassisMotorControl& output) {
        std::array<float, 4> targets{};
        std::shared_ptr<GoalHandleSteeringAngleControl> goal_handle;
        float tolerance = 0.08f;
        rclcpp::Time deadline;
        rclcpp::Time start_time;
        {
            std::lock_guard<std::mutex> lock(steering_action_mutex_);
            if (!steering_action_active_) {
                return false;
            }
            targets = steering_action_targets_;
            tolerance = steering_action_tolerance_;
            deadline = steering_action_deadline_;
            start_time = steering_action_start_time_;
            goal_handle = steering_action_goal_handle_;
        }

        output.position.resize(4);
        output.speed.resize(4);
        // 必须使用 Action 接受之后到达的新反馈，避免用历史舵角误判到位。
        const bool feedback_after_goal =
            steering_feedback_received_ &&
            (last_steering_feedback_time_ - start_time).seconds() >= 0.0;
        float max_error = feedback_after_goal
                              ? 0.0f
                              : std::numeric_limits<float>::infinity();
        for (std::size_t wheel = 0; wheel < targets.size(); ++wheel) {
            // Action 执行期间始终直接下发目标舵角，并强制四个驱动轮为零速。
            output.position[wheel] = targets[wheel];
            output.speed[wheel] = 0.0;
            if (feedback_after_goal) {
                max_error = std::max(
                    max_error, std::abs(angle_diff(targets[wheel], prev_angle_[wheel])));
            }
        }
        output.estop = 0;
        output.neutral = 0;

        if (goal_handle && goal_handle->is_canceling()) {
            finishSteeringAction(false, "静止对舵任务被取消");
            return true;
        }

        auto feedback = std::make_shared<SteeringAngleControl::Feedback>();
        feedback->max_steer_error = max_error;
        feedback->time_left = deadline - this->now();
        if (goal_handle) {
            goal_handle->publish_feedback(feedback);
        }

        if (feedback_after_goal && max_error <= tolerance) {
            finishSteeringAction(true, "四轮舵角已到达目标位置，零速目标保持已启用");
        } else if (this->now() >= deadline) {
            finishSteeringAction(false, "等待舵角到位超时");
        }
        return true;
    }

    // [Layer 1][Steering Primitive Handoff]
    // Action 成功以后、上层仍给零 Twist 时继续发布原理论目标，防止把机械回弹后的
    // 反馈角逐帧当作新目标而慢慢漂回零位。只接受“锁存开始之后到达”的新非零
    // Twist 作为解除条件，避免 Action 前遗留的缓存速度误解除。
    bool tickSteeringTargetHold(
        chassis_interfaces::msg::ChassisMotorControl& output,
        float vx, float vy, float vw,
        const rclcpp::Time& command_time,
        bool command_timed_out) {
        std::array<float, 4> targets{};
        bool release_for_motion = false;
        {
            std::lock_guard<std::mutex> lock(steering_action_mutex_);
            if (!steering_action_hold_active_) {
                return false;
            }

            const bool nonzero_command =
                std::hypot(vx, vy) > ZERO_CMD_LINEAR_EPS ||
                std::abs(vw) > ZERO_CMD_YAW_EPS;
            const bool command_arrived_after_hold =
                (command_time - steering_action_hold_start_time_).seconds() > 0.0;
            release_for_motion =
                !command_timed_out && nonzero_command && command_arrived_after_hold;
            if (release_for_motion) {
                steering_action_hold_active_ = false;
            } else {
                targets = steering_action_targets_;
            }
        }

        if (release_for_motion) {
            RCLCPP_INFO(
                this->get_logger(),
                "[SteeringAngleControl] 收到新的非零 Twist，解除目标舵角锁存并恢复正常解算");
            return false;
        }

        output.position.resize(4);
        output.speed.resize(4);
        for (std::size_t wheel = 0; wheel < targets.size(); ++wheel) {
            output.position[wheel] = targets[wheel];
            output.speed[wheel] = 0.0;
        }
        output.estop = 0;
        output.neutral = 0;
        return true;
    }

    // 常量定义
    // static constexpr float SPEED_RAMP_RATE = 0.8f;         // 速度加速度系数 (0-1)，越小过渡越平滑
    // static constexpr float ANGLE_TOLERANCE = 0.01f;        // 角度容差（rad），判断是否接近目标角度

    // 计算两个角度之间的最小差值（考虑角度的周期性）
    //static float angle_diff(float target, float current) {
    //    float diff = target - current;
    //    // 将差值规范化到 [-pi, pi]
    //    while (diff > PI_CHASSIS) diff -= 2.0f * PI_CHASSIS;
    //    while (diff < -PI_CHASSIS) diff += 2.0f * PI_CHASSIS;
    //    return diff;
    //}

    void compute_wheels(float vx, float vy, float vw, float out_angle[4], float out_speed_rpm[4]) {
        // [Layer 1][Zero Command Contract]
        // 零速度不重新求 atan2(0, 0)，直接保持真实反馈对应的当前舵角。
        // 驻车上层只需持续给零 Twist；region 不需要知道“是否驻车”。
        const bool command_is_zero =
            std::hypot(vx, vy) <= ZERO_CMD_LINEAR_EPS &&
            std::abs(vw) <= ZERO_CMD_YAW_EPS;
        if (command_is_zero) {
            for (int i = 0; i < 4; ++i) {
                out_angle[i] = prev_angle_[i];
                out_speed_rpm[i] = 0.0f;
            }
            return;
        }

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
                if(ang_err[i] >= STEER_ANGLE_LIMIT){
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "[compute_wheels] 第 %d 个车轮舵角误差 %.3f rad 超出限位 %.3f rad，"
                        "可能存在舵机故障或反馈异常，强制停速",
                        i, ang_err[i], STEER_ANGLE_LIMIT);
                } else {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "[compute_wheels] 第 %d 个车轮舵角误差 %.3f rad 超过阈值 %.3f rad，"
                        "本周期强制停速",
                        i, ang_err[i], ANGLE_EPS);
                }
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
                                      STEER_MAX_STEP  // 100Hz
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

        // [Layer 1][Region Control Frequency] 100Hz 定时解算并发布。
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

        auto motor_control_msg = chassis_interfaces::msg::ChassisMotorControl();
        motor_control_msg.header.stamp = this->now();
        motor_control_msg.header.frame_id = "base_link";

        // [Layer 1][Steering Primitive] Action 执行期间由同一电机出口静止对舵。
        if (tickSteeringAction(motor_control_msg)) {
            motor_control_pub_->publish(motor_control_msg);
            // 静止对舵不计入正常行驶解算耗时窗口。
            recordControlTiming(false, 0.0, 0.0, 0.0, 0.0, 0.0);
            return;
        }

        // Action 成功后的零速交接期继续保持理论目标角；新的非零 Twist 会在本函数
        // 内解除锁存，并让同一控制周期继续进入正常轮向解算。
        if (tickSteeringTargetHold(
            motor_control_msg, vx, vy, vw, cmd_time, command_timed_out)) {
            motor_control_pub_->publish(motor_control_msg);
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

    // [Layer 1][Steering Primitive] /steering_angle_control 保存当前执行目标；成功后
    // 仅在零 Twist 下保持同一目标，第一条新非零 Twist 到达即恢复正常解算。
    rclcpp_action::Server<SteeringAngleControl>::SharedPtr
        steering_angle_control_server_;
    std::mutex steering_action_mutex_;
    bool steering_action_active_{false};
    bool steering_action_hold_active_{false};
    std::array<float, 4> steering_action_targets_{};
    float steering_action_tolerance_{0.08f};
    rclcpp::Time steering_action_deadline_;
    rclcpp::Time steering_action_start_time_;
    rclcpp::Time steering_action_hold_start_time_;
    std::shared_ptr<GoalHandleSteeringAngleControl> steering_action_goal_handle_;

    // 缓存最新的速度指令
    std::mutex cmd_mutex_;
    geometry_msgs::msg::Twist latest_cmd_;
    rclcpp::Time last_cmd_time_;

    // 历史状态：用于保持角度和速度的连续性
    bool steering_feedback_received_{false};
    rclcpp::Time last_steering_feedback_time_;
    float prev_angle_[4];      // 前一帧的舵角
    float prev_speed_rpm_[4];  // 前一帧的速度
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MobileOmniController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
