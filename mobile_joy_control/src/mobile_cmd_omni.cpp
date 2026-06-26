#include <algorithm>
#include <cmath>
#include <mutex>
#include <vector>

#include "chassis_interfaces/msg/chassis_motor_control.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

// 全向底盘参数定义（CHASSIS_SIZE_* 单位为 mm；解算时 *0.001 转为 m）
#define CHASSIS_SIZE_L 460.0f  // 前后轮间距 wheelbase (mm)
#define CHASSIS_SIZE_W 345.0f  // 左右轮间距 track (mm)

#define WHEEL_DIAM 107.0f  // 轮径 (mm)

#define SPEED_TO_RPM(x) ((x)*60.0f / (3.141592f * WHEEL_DIAM * 0.001f))

#define PI_CHASSIS 3.141592f
#define FRONT_DIRECTION_ANGLE 1.570796f  // 车头方向相对舵角零位偏移
#define ZERO_ANGLE_THETA 0.090625f       // 舵机零位机械偏置

// 转向限速：从反馈角 Δθ/Δt 在线估计（实机约 45 RPM ≈ 4.71 rad/s）
#define CONTROL_PERIOD_S 0.05f   // 50ms -> 20Hz，与 timer_ 一致
#define STEER_STEP_MIN 0.02f       // rad/周期，冷启动下限
#define STEER_STEP_MAX 0.25f       // rad/周期，≈48 RPM 上限
#define W_STEER_EST_ALPHA 0.3f     // w_est 低通系数

// 动态 DLS：μ=√det(H^T H)，λ 随可操作性变化  【当前启用】
// 注意：L/W 传入 H 时必须为 m，μ 的量纲为 m²，DLS_MU0 也必须是 m² 量纲下的阈值
#define DLS_LAMBDA_MIN 0.02f
#define DLS_LAMBDA_MAX 0.50f
#define DLS_MU0 0.08f              // 初始阈值，可用日志里的 suggest_mu0 标定
#define DLS_LAMBDA_POWER 2.f

// --- [备选 · 固定 λ] 不用动态 μ 时，按下面步骤切换（勿与上面动态方案同时启用）---
// 1) 注释掉上方 DLS_LAMBDA_MIN / DLS_LAMBDA_MAX / DLS_LAMBDA_POWER（动态专用，固定方案不需要）
// 2) 取消下面 1 行注释，设定固定阻尼：
// #define DLS_LAMBDA_FIXED 0.15f
// 3) 在 compute_dls_lambda() 内改为 return DLS_LAMBDA_FIXED;（见函数下方注释块）
//    或在 solve_achievable_twist() 里把
//      lambda_out = compute_dls_lambda(mu_out);
//    换成
//      lambda_out = DLS_LAMBDA_FIXED;  // mu_out 仍计算并打印，但不参与 λ
// 4) 日志里 ratio 仅作参考；标定 DLS_LAMBDA_FIXED 建议 0.10~0.25，越大越稳、越钝
// 5) 若仍想保留 DLS_MU0 仅用于日志对比，可继续保留 #define DLS_MU0

#define SPEED_RAMP_RATE 0.8f

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

static float clampf(float v, float lo, float hi) { return std::max(lo, std::min(hi, v)); }

static float det3x3(const float M[3][3]) {
    return M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
         - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
         + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
}

static float compute_dls_lambda(float mu) {
    const float ratio = std::min(mu / DLS_MU0, 1.f);
    if (ratio >= 1.f) return DLS_LAMBDA_MIN;
    const float t = 1.f - ratio;
    return DLS_LAMBDA_MIN + (DLS_LAMBDA_MAX - DLS_LAMBDA_MIN) * std::pow(t, DLS_LAMBDA_POWER);

    // --- [备选 · 固定 λ] 启用固定方案时，注释掉上面 4 行，改用下面 1 行 ---
    // return DLS_LAMBDA_FIXED;
}

// --- [备选 · 固定 λ] 完整固定版 compute_dls_lambda（二选一，勿与上方同时保留）---
// static float compute_dls_lambda(float /*mu*/) {
//     return DLS_LAMBDA_FIXED;
// }

static bool invert3x3(const float A[3][3], float invA[3][3]) {
    const float det = det3x3(A);
    if (std::abs(det) < 1e-9f) return false;
    invA[0][0] =  (A[1][1] * A[2][2] - A[1][2] * A[2][1]) / det;
    invA[0][1] = -(A[0][1] * A[2][2] - A[0][2] * A[2][1]) / det;
    invA[0][2] =  (A[0][1] * A[1][2] - A[0][2] * A[1][1]) / det;
    invA[1][0] = -(A[1][0] * A[2][2] - A[1][2] * A[2][0]) / det;
    invA[1][1] =  (A[0][0] * A[2][2] - A[0][2] * A[2][0]) / det;
    invA[1][2] = -(A[0][0] * A[1][2] - A[0][2] * A[1][0]) / det;
    invA[2][0] =  (A[1][0] * A[2][1] - A[1][1] * A[2][0]) / det;
    invA[2][1] = -(A[0][0] * A[2][1] - A[0][1] * A[2][0]) / det;
    invA[2][2] =  (A[0][0] * A[1][1] - A[0][1] * A[1][0]) / det;
    return true;
}

// W, L 必须为 m；μ=√det(H^T H)，量纲 m²
static bool solve_achievable_twist(const float theta[4], const float kx[4], const float ky[4],
                                   const float corner_vx[4], const float corner_vy[4], float W, float L,
                                   float& vx_a, float& vy_a, float& vw_a, float& mu_out, float& lambda_out) {
    float s_cmd[4];
    float H[4][3];
    for (int i = 0; i < 4; ++i) {
        const float c = std::cos(theta[i]);
        const float s = std::sin(theta[i]);
        s_cmd[i] = corner_vx[i] * c + corner_vy[i] * s;
        H[i][0] = c;
        H[i][1] = s;
        H[i][2] = c * kx[i] * W * 0.5f + s * ky[i] * L * 0.5f;
    }

    float HtH[3][3] = {};
    float Hts[3] = {};
    for (int i = 0; i < 4; ++i) {
        for (int r = 0; r < 3; ++r) {
            Hts[r] += H[i][r] * s_cmd[i];
            for (int c = 0; c < 3; ++c) {
                HtH[r][c] += H[i][r] * H[i][c];
            }
        }
    }

    mu_out = std::sqrt(std::max(det3x3(HtH), 0.f));
    lambda_out = compute_dls_lambda(mu_out);
    // --- [备选 · 固定 λ] 不经过 μ 映射时，注释上一行，改用下一行 ---
    // lambda_out = DLS_LAMBDA_FIXED;
    const float lambda_sq = lambda_out * lambda_out;

    float A[3][3];
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            A[r][c] = HtH[r][c];
        }
        A[r][r] += lambda_sq;
    }

    float Ainv[3][3];
    if (!invert3x3(A, Ainv)) {
        vx_a = vy_a = vw_a = 0.f;
        return false;
    }

    vx_a = Ainv[0][0] * Hts[0] + Ainv[0][1] * Hts[1] + Ainv[0][2] * Hts[2];
    vy_a = Ainv[1][0] * Hts[0] + Ainv[1][1] * Hts[1] + Ainv[1][2] * Hts[2];
    vw_a = Ainv[2][0] * Hts[0] + Ainv[2][1] * Hts[1] + Ainv[2][2] * Hts[2];
    return true;
}

class MobileOmniController : public rclcpp::Node {
public:
    MobileOmniController() : Node("mobile_cmd_omni") {
        RCLCPP_INFO(this->get_logger(), "Mobile Omni Controller initialized (方案2 DLS + 做法B)");

        for (int i = 0; i < 4; ++i) {
            prev_angle_[i] = 0.0f;
            prev_speed_rpm_[i] = 0.0f;
            prev_angle_last_[i] = 0.f;
            w_est_[i] = 0.f;
        }
        steer_rate_initialized_ = false;
        prev_vx_ = 0.0f;

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/control/cmd_vel_omni", 10, std::bind(&MobileOmniController::cmdVelCallback, this, std::placeholders::_1));

        feedback_sub_ = this->create_subscription<chassis_interfaces::msg::ChassisMotorControl>(
            "/feedback/omni_chassis_motor", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
            std::bind(&MobileOmniController::feedbackCallback, this, std::placeholders::_1));

        motor_control_pub_ =
            this->create_publisher<chassis_interfaces::msg::ChassisMotorControl>("/control/omni_chassis_motors", 10);

        latest_cmd_ = geometry_msgs::msg::Twist();
        last_cmd_time_ = this->now();

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
                prev_speed_rpm_[i] = static_cast<float>(msg->speed[i]);
            }
        }
    }

    void update_steer_rate_from_feedback() {
        for (int i = 0; i < 4; ++i) {
            if (!steer_rate_initialized_) {
                prev_angle_last_[i] = prev_angle_[i];
                w_est_[i] = 0.f;
                continue;
            }
            const float dtheta = std::abs(angle_diff(prev_angle_[i], prev_angle_last_[i]));
            const float w_meas = dtheta / CONTROL_PERIOD_S;
            w_est_[i] = (1.f - W_STEER_EST_ALPHA) * w_est_[i] + W_STEER_EST_ALPHA * w_meas;
            prev_angle_last_[i] = prev_angle_[i];
        }
        steer_rate_initialized_ = true;
    }

    float steer_max_step_for_wheel(int i) const {
        float step = w_est_[i] * CONTROL_PERIOD_S;
        if (!steer_rate_initialized_) {
            step = STEER_STEP_MAX;
        }
        return clampf(step, STEER_STEP_MIN, STEER_STEP_MAX);
    }

    void update_mu_calibration(float mu, float vx, float vy, float vw) {
        const float cmd_mag = std::hypot(vx, std::hypot(vy, vw));
        if (cmd_mag < 0.02f || mu < 1e-6f) {
            return;
        }
        if (mu_ema_ <= 0.f) {
            mu_ema_ = mu;
        } else {
            mu_ema_ = 0.98f * mu_ema_ + 0.02f * mu;
        }
        if (mu < mu_min_under_cmd_) {
            mu_min_under_cmd_ = mu;
        }
        // 建议阈值：典型 μ 的 ~35%，或观测最小值略放大，取较大者更保守
        mu_suggest_ = std::max(mu_ema_ * 0.35f, mu_min_under_cmd_ * 1.2f);
    }

    void log_dls_diagnostics(float mu, float lambda, float vx_a, float vy_a, float vw_a, float vx, float vy,
                             float vw) {
        update_mu_calibration(mu, vx, vy, vw);
        const float ratio = (DLS_MU0 > 1e-9f) ? std::min(mu / DLS_MU0, 1.f) : 0.f;

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "DLS: mu=%.5f (m^2) lambda=%.3f ratio=%.2f | cfg DLS_MU0=%.3f suggest_DLS_MU0=%.3f "
            "(mu_ema=%.5f mu_min=%.5f) | u_ach=(%.3f,%.3f,%.3f) cmd=(%.3f,%.3f,%.3f) w_est=[%.2f,%.2f,%.2f,%.2f]",
            mu, lambda, ratio, DLS_MU0, mu_suggest_, mu_ema_, mu_min_under_cmd_, vx_a, vy_a, vw_a, vx, vy, vw,
            w_est_[0], w_est_[1], w_est_[2], w_est_[3]);
    }

    void compute_wheels(float vx, float vy, float vw, float out_angle[4], float out_speed_rpm[4]) {
        // mm -> m；H 矩阵与 μ 均基于米制，CHASSIS_SIZE_* 为 mm 不影响 μ（此处已换算）
        const float L = CHASSIS_SIZE_L * 0.001f;
        const float W = CHASSIS_SIZE_W * 0.001f;

        update_steer_rate_from_feedback();

        float target_ang[4];
        float corner_vx[4];
        float corner_vy[4];
        float kx_tab[4];
        float ky_tab[4];

        for (int i = 0; i < 4; ++i) {
            const bool is_right = (i == 1 || i == 3);
            const bool is_front = (i == 0 || i == 1);

            const float kx = is_right ? +1.f : -1.f;
            const float ky = is_front ? +1.f : -1.f;
            kx_tab[i] = kx;
            ky_tab[i] = ky;

            const float vx_i = vx + kx * W * vw * 0.5f;
            const float vy_i = vy + ky * L * vw * 0.5f;
            corner_vx[i] = vx_i;
            corner_vy[i] = vy_i;

            const float speed = std::hypot(vx_i, vy_i);
            const float ang = std::atan2(vy_i, vx_i);

            const float angA = wrap_pi(ang);
            const float angB = wrap_pi(ang + PI_CHASSIS);

            const bool validA = (std::abs(angA) <= PI_CHASSIS / 2.f);
            const bool validB = (std::abs(angB) <= PI_CHASSIS / 2.f);

            const float costA = validA ? std::abs(angle_diff(angA, prev_angle_[i])) : 1e6f;
            const float costB = validB ? std::abs(angle_diff(angB, prev_angle_[i])) : 1e6f;

            if (costA <= costB) {
                target_ang[i] = angA;
            } else {
                target_ang[i] = angB;
            }
        }

        float vx_a = 0.f;
        float vy_a = 0.f;
        float vw_a = 0.f;
        float mu = 0.f;
        float lambda = 0.f;
        if (!solve_achievable_twist(prev_angle_, kx_tab, ky_tab, corner_vx, corner_vy, W, L, vx_a, vy_a, vw_a, mu,
                                    lambda)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "DLS invert failed, mu=%.5f lambda=%.3f — zeroing wheel speeds", mu, lambda);
            for (int i = 0; i < 4; ++i) {
                out_angle[i] = prev_angle_[i];
                out_speed_rpm[i] = 0.f;
            }
            return;
        }

        log_dls_diagnostics(mu, lambda, vx_a, vy_a, vw_a, vx, vy, vw);

        for (int i = 0; i < 4; ++i) {
            const float max_step = steer_max_step_for_wheel(i);
            out_angle[i] = ramp_angle(target_ang[i], prev_angle_[i], max_step);

            const float theta = prev_angle_[i];
            const float vx_i_a = vx_a + kx_tab[i] * W * vw_a * 0.5f;
            const float vy_i_a = vy_a + ky_tab[i] * L * vw_a * 0.5f;
            const float v_wheel = vx_i_a * std::cos(theta) + vy_i_a * std::sin(theta);

            const float target_rpm_i = SPEED_TO_RPM(v_wheel);
            const float diff = target_rpm_i - prev_speed_rpm_[i];
            out_speed_rpm[i] = prev_speed_rpm_[i] + diff * SPEED_RAMP_RATE;
        }
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        latest_cmd_ = *msg;
        last_cmd_time_ = this->now();
    }

    void publishControlCallback() {
        geometry_msgs::msg::Twist cmd;
        rclcpp::Time cmd_time;
        {
            std::lock_guard<std::mutex> lk(cmd_mutex_);
            cmd = latest_cmd_;
            cmd_time = last_cmd_time_;
        }

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

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<chassis_interfaces::msg::ChassisMotorControl>::SharedPtr feedback_sub_;
    rclcpp::Publisher<chassis_interfaces::msg::ChassisMotorControl>::SharedPtr motor_control_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex cmd_mutex_;
    geometry_msgs::msg::Twist latest_cmd_;
    rclcpp::Time last_cmd_time_;

    float prev_angle_[4];
    float prev_speed_rpm_[4];
    float prev_vx_;

    float prev_angle_last_[4];
    float w_est_[4];
    bool steer_rate_initialized_{false};

    float mu_ema_{0.f};
    float mu_min_under_cmd_{1e9f};
    float mu_suggest_{DLS_MU0};

    // =========================================================================
    // DLS λ 方案切换（二选一）
    //
    // 【A · 动态 λ】当前默认：compute_dls_lambda(mu) 随 μ 变化，见文件顶部常量
    //
    // 【B · 固定 λ】步骤：
    //   1. #define DLS_LAMBDA_FIXED 0.15f
    //   2. compute_dls_lambda() 改为 return DLS_LAMBDA_FIXED;（或 solve 里直接赋值）
    //   3. mu 仍会通过 log_dls_diagnostics 打印，便于对比；suggest_DLS_MU0 可忽略
    //
    // 固定 λ 更简单；动态 λ 在四轮退化时自动加大阻尼。其余方案2/做法B 逻辑不变。
    // =========================================================================
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MobileOmniController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
