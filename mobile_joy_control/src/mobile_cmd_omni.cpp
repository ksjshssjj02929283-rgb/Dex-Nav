#include <cmath>
#include <mutex>
#include <vector>

#include "chassis_interfaces/msg/chassis_motor_control.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

// 全向底盘参数定义（单位：mm，角度：rad）
#define CHASSIS_SIZE_L 460.0f  // 前后轮间距（wheelbase）
#define CHASSIS_SIZE_W 345.0f  // 左右轮间距（track）

#define WHEEL_DIAM 107.0f  // 轮径（mm）

#define SPEED_TO_RPM(x) ((x)*60.0f / (3.141592f * WHEEL_DIAM * 0.001f))

#define PI_CHASSIS 3.141592f
#define FRONT_DIRECTION_ANGLE 1.570796f  // 车头方向相对舵角零位偏移
#define ZERO_ANGLE_THETA 0.090625f       // 舵机零位机械偏置

#define STEER_MAX_STEP 6.28f  // rad / cycle
#define SPEED_RAMP_RATE 0.8f
#define ANGLE_EPS 0.1f

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

        // 初始化缓存指令与时间戳
        latest_cmd_ = geometry_msgs::msg::Twist();
        last_cmd_time_ = this->now();

        // 20Hz 定时器：定时解算并发布电机控制指令
        using namespace std::chrono_literals;
        timer_ = this->create_wall_timer(50ms, std::bind(&MobileOmniController::publishControlCallback, this));
    }

private:
    void feedbackCallback(const chassis_interfaces::msg::ChassisMotorControl::SharedPtr msg) {
        if (msg->position.size() >= 4) {
            for (int i = 0; i < 4; ++i) {
                prev_angle_[i] = static_cast<float>(msg->position[i]);
            }
        }
        if (msg->speed.size() >= 4) {
            for (int i = 0; i < 4; ++i) {
                // 反馈话题里的 speed 通常是轮线速度(m/s) 或 rpm，需与控制侧保持一致。
                // 这里按 rpm 使用；若你的反馈是 m/s，请改为 SPEED_TO_RPM(msg->speed[i]).
                prev_speed_rpm_[i] = static_cast<float>(msg->speed[i]);
            }
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
        if (abs_angle > PI_CHASSIS / 2.0f - 0.1f && abs_angle < PI_CHASSIS / 2.0f + 0.1f) {
            // 接近±pi/2，考虑两个候选值
            float candidate1 = PI_CHASSIS / 2.0f;   // +pi/2
            float candidate2 = -PI_CHASSIS / 2.0f;  // -pi/2

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

            bool validA = (std::abs(angA) <= PI_CHASSIS / 2.f);
            bool validB = (std::abs(angB) <= PI_CHASSIS / 2.f);

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

    void publishControlCallback() {
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

        // 简单超时保护：超过 0.5s 未收到新指令，则自动置零
        const double timeout_sec = 0.5;
        if ((this->now() - cmd_time).seconds() > timeout_sec) {
            cmd = geometry_msgs::msg::Twist();
        }

        const float vx = static_cast<float>(cmd.linear.x);
        const float vy = static_cast<float>(cmd.linear.y);
        const float vw = static_cast<float>(cmd.angular.z);

        float angle[4];
        float speed_rpm[4];
        compute_wheels(vx, vy, vw, angle, speed_rpm);

        auto motor_control_msg = chassis_interfaces::msg::ChassisMotorControl();
        motor_control_msg.header.stamp = this->now();
        motor_control_msg.header.frame_id = "base_link";

        motor_control_msg.position.resize(4);
        motor_control_msg.speed.resize(4);
        for (int i = 0; i < 4; ++i) {
            motor_control_msg.position[i] = angle[i];
            motor_control_msg.speed[i] = speed_rpm[i];
        }

        motor_control_msg.estop = 0;
        motor_control_msg.neutral = 0;

        motor_control_pub_->publish(motor_control_msg);
    }

    // 成员变量
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<chassis_interfaces::msg::ChassisMotorControl>::SharedPtr feedback_sub_;
    rclcpp::Publisher<chassis_interfaces::msg::ChassisMotorControl>::SharedPtr motor_control_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 缓存最新的速度指令
    std::mutex cmd_mutex_;
    geometry_msgs::msg::Twist latest_cmd_;
    rclcpp::Time last_cmd_time_;

    // 历史状态：用于保持角度和速度的连续性
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
