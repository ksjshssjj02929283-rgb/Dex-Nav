#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
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

#define STEER_ANGLE_LIMIT 2.50f // 90度，2.50f:143.23945度

// 转向限速：从反馈角 Δθ/Δt 在线估计（实机约 45 RPM ≈ 4.71 rad/s）
#define CONTROL_PERIOD_S 0.05f   // 50ms -> 20Hz，与 timer_ 一致
#define STEER_STEP_MIN 0.02f       // rad/周期，冷启动下限
#define STEER_STEP_MAX 0.25f       // rad/周期，≈48 RPM 上限
#define W_STEER_EST_ALPHA 0.3f     // w_est 低通系数

// 动态 DLS：μ=√det(H^T H)，λ 随可操作性变化  【当前启用】
// 注意：L/W 传入 H 时必须为 m，μ 的量纲为 m²，DLS_MU0 也必须是 m² 量纲下的阈值
#define DLS_LAMBDA_MIN 0.02f
#define DLS_LAMBDA_MAX 0.50f
// [调参改动1] DLS_MU0 按实机诊断重新标定。
// 原来 0.080f 对这台车明显偏大：横移稳定时 mu≈0.0065，却仍被判成 DLS_HEAVY。
// 这里先取略高于横移 P20 的 0.008f，让正常横移/斜向运动不被过度阻尼。
#define DLS_MU0 0.005f
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

// ============================================================================
// MPPI 后级轻量速度约束器参数
// ----------------------------------------------------------------------------
// 这一层不是重新做路径跟踪，也不是替代 Nav2 MPPI。
// 它只把 MPPI/遥控器给出的 cmd_vel 变成底盘机械更容易执行的速度指令。
//
// 【需要实车标定】
// - CMD_GOV_MAX_AX / AY / AW：负载 70kg 时，底盘不打滑、不明显点头/抖动的最大加速度。
// - CMD_GOV_MAX_BRAKE_*：松杆或停车时的最大减速度。太大容易急刹抖，太小会拖尾。
// - CMD_GOV_JERK_LIMIT_ENABLE：如果 MPPI 已经配置 jerk，建议默认 0，避免双重 jerk 限制。
//   若遥控器/上游 mux 可能绕过 MPPI 直接给突变 cmd_vel，再改成 1。
// - CMD_GOV_MAX_JERK_*：启用后建议设成 MPPI jerk 的 1.1~1.5 倍，
//   让 MPPI 做主要约束，底盘层只拦截遥控突变、通信抖动或异常速度跳变。
// ============================================================================
#define CMD_GOVERNOR_ENABLE 1
#define CMD_GOV_JERK_LIMIT_ENABLE 0
#define CMD_GOV_MAX_AX 0.60f      // m/s^2，缺少实测参数：建议从 0.30~0.60 试
#define CMD_GOV_MAX_AY 0.60f      // m/s^2，缺少实测参数：横移负载下通常要比 vx 小
#define CMD_GOV_MAX_AW 0.84f      // rad/s^2，缺少实测参数：边走边转 180deg 时重点调它
#define CMD_GOV_MAX_BRAKE_X 0.60f // m/s^2，缺少实测参数：停止时若抖，降低这个值
#define CMD_GOV_MAX_BRAKE_Y 0.65f // m/s^2，缺少实测参数：横向急停摩地则降低
#define CMD_GOV_MAX_BRAKE_W 0.84f // rad/s^2，缺少实测参数：旋转急停抖则降低
#define CMD_GOV_MAX_JERK_X 1.80f  // m/s^3，缺少实测参数：建议略大于 MPPI jerk_x
#define CMD_GOV_MAX_JERK_Y 1.40f  // m/s^3，缺少实测参数：横移 jerk 通常小于 vx
#define CMD_GOV_MAX_JERK_W 3.50f  // rad/s^3，缺少实测参数：建议略大于 MPPI jerk_w

// ============================================================================
// 舵轮候选角优化参数
// ----------------------------------------------------------------------------
// 普通反解只会在 angle 和 angle+pi 两个等价解里选离当前角最近的。
// 这里额外加入：
// 1) 限位余量：不要贴着机械限位跑；
// 2) 候选保持：不要 A/B 两个等价解来回跳；
// 3) 轮速正反切换惩罚：避免轮子方向频繁反向。
//
// 【需要实车标定】
// - STEER_LIMIT_MARGIN：机械限位安全余量。编码器零位、机构弹性、安装误差越大，余量越大。
// - CANDIDATE_*_COST：这些是“代价权重”，不是物理量。现值偏保守，先用于减少频繁换向。
// ============================================================================
#define STEER_LIMIT_MARGIN 0.08f          // rad，约 4.6deg，缺少实测参数：建议 0.05~0.12
#define CANDIDATE_SWITCH_COST 0.35f       // rad 等效代价，缺少实测参数：越大越不愿 A/B 切换
#define CANDIDATE_REVERSE_COST 0.20f      // rad 等效代价，缺少实测参数：越大越不愿轮速正反切
#define CANDIDATE_LIMIT_SOFT_ZONE 0.20f   // rad，靠近限位多少范围内开始加代价
#define CANDIDATE_LIMIT_COST_GAIN 0.25f   // rad 等效代价，缺少实测参数：越大越远离限位

// ============================================================================
// 单轮舵角误差轮速缩放参数
// ----------------------------------------------------------------------------
// 原代码用 max_steer_err 统一压低四个轮速，安全但可能一卡一卡。
// 新版本保留大误差互锁，同时给每个轮子单独算 wheel_scale：
// - 对准的轮子可以保留更多力；
// - 没对准的轮子少推，减少横向摩擦。
//
// 【需要实车标定】
// - STEER_GLOBAL_*：整车级缩放，建议比单轮缩放宽松；
// - STEER_ERR_FULL_SPEED / FULL_STOP：单轮缩放沿用原参数。
// ============================================================================
#define PER_WHEEL_STEER_SCALE_ENABLE 1
#define STEER_GLOBAL_FULL_SPEED 0.30f // rad，整车级开始降速阈值，缺少实测参数
#define STEER_GLOBAL_FULL_STOP 0.90f  // rad，整车级最低缩放阈值，缺少实测参数
#define STEER_GLOBAL_MIN_SCALE 0.45f  // 非互锁状态下整车最低比例，缺少实测参数

// [诊断改动1] 实时诊断文件输出路径。
// latest 每次有效控制周期覆盖一次；CSV 每次有效控制周期追加一行历史样本。
#define DLS_DIAG_LATEST_PATH "/tmp/omni_dls_latest.txt"
#define DLS_DIAG_CSV_PATH "/tmp/omni_dls_samples.csv"
#define DLS_DIAG_MAX_SAMPLES 3000

// 指令接近 0 时保持当前舵角，避免 atan2(0,0) 把轮子拉回某个无意义角度
#define CMD_LINEAR_DEADBAND 0.002f   // m/s
#define CMD_ANGULAR_DEADBAND 0.003f  // rad/s

// [改动1] 纯前进/后退判定阈值：
// 四轮都接近 0 deg 时，mu 会因为完整三自由度退化而接近 0，
// 但纯 vx 运动本身是安全可实现的，所以这种指令不应该触发动态 DLS 重阻尼。
#define SIMPLE_LONGITUDINAL_VY_MAX 0.02f  // m/s
#define SIMPLE_LONGITUDINAL_VW_MAX 0.02f  // rad/s

// [调参改动2] 纯横移判定阈值：
// 四轮都接近 ±90 deg 时，完整三自由度可操作性 mu 也会偏低；
// 但如果当前指令只是 vy 横移，这个姿态对该指令是完全可实现的，不应触发重 DLS。
#define SIMPLE_LATERAL_VX_MAX 0.02f  // m/s
#define SIMPLE_LATERAL_VW_MAX 0.02f  // rad/s

// 单轮合成速度很小时方向不可靠，保持当前舵角，避免低速 atan2 方向跳变
#define WHEEL_VECTOR_DEADBAND 0.03f  // m/s

// 舵角还没对准时的整车轮速门控：基于实际反馈角，而不是理论目标角
// [软重载保护改动1] 不再用很小误差就硬停的策略，避免正常转弯一卡一卡。
// 误差约 4.6deg 内满速，约 28.6deg 以上目标速度降到 0，中间线性降速。
#define STEER_ERR_FULL_SPEED 0.08f  // rad
#define STEER_ERR_FULL_STOP 0.50f   // rad

// [软重载保护改动2] 大换向互锁只在明显大角度时进入。
// 原先 0.22rad 太敏感，正常限制角范围内转弯也容易触发停顿。
#define STEER_ALIGN_ENTER_ERR 0.55f    // rad, 约 31.5deg
#define STEER_ALIGN_RELEASE_ERR 0.12f  // rad, 约 6.9deg
#define STEER_ALIGN_HOLD_CYCLES 2      // 20Hz 下约 100ms

// [软重载保护改动3] drive_scale 限斜率，避免 rpm_cmd 因上层保护瞬间掉到 0 引起底层急刹抖动。
// 下降稍快是为了防摩地，恢复稍慢是为了避免刚对齐又猛冲。
#define DRIVE_SCALE_MAX_DROP_PER_CYCLE 0.20f
#define DRIVE_SCALE_MAX_RISE_PER_CYCLE 0.12f
#define STEER_ALIGN_HOLD_TARGET_SCALE 0.03f

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

// [改动2] 判断当前指令是不是“只要求前进/后退”。
// 这种情况下即使 mu=0，也不代表当前 vx 指令危险；DLS 不应把 lambda 推到最大。
static bool is_simple_longitudinal_cmd(float vx, float vy, float vw) {
    return std::abs(vx) > CMD_LINEAR_DEADBAND &&
           std::abs(vy) < SIMPLE_LONGITUDINAL_VY_MAX &&
           std::abs(vw) < SIMPLE_LONGITUDINAL_VW_MAX;
}

// [调参改动3] 判断当前指令是不是“只要求横移”。
// 纯横移与纯前进一样，是单自由度明确指令；即使完整 3DOF 的 mu 偏低，也不代表该指令危险。
static bool is_simple_lateral_cmd(float vx, float vy, float vw) {
    return std::abs(vy) > CMD_LINEAR_DEADBAND &&
           std::abs(vx) < SIMPLE_LATERAL_VX_MAX &&
           std::abs(vw) < SIMPLE_LATERAL_VW_MAX;
}

static float drive_scale_from_steer_error(float max_err) {
    if (max_err <= STEER_ERR_FULL_SPEED) return 1.f;
    if (max_err >= STEER_ERR_FULL_STOP) return 0.f;
    const float span = STEER_ERR_FULL_STOP - STEER_ERR_FULL_SPEED;
    return clampf((STEER_ERR_FULL_STOP - max_err) / span, 0.f, 1.f);
}

static float global_drive_scale_from_steer_error(float max_err) {
    if (max_err <= STEER_GLOBAL_FULL_SPEED) return 1.f;
    if (max_err >= STEER_GLOBAL_FULL_STOP) return STEER_GLOBAL_MIN_SCALE;
    const float span = STEER_GLOBAL_FULL_STOP - STEER_GLOBAL_FULL_SPEED;
    const float t = (STEER_GLOBAL_FULL_STOP - max_err) / span;
    return STEER_GLOBAL_MIN_SCALE + (1.f - STEER_GLOBAL_MIN_SCALE) * clampf(t, 0.f, 1.f);
}

#if !CMD_GOV_JERK_LIMIT_ENABLE
static float limit_velocity_rate(float target, float current, float max_accel, float max_brake, float dt) {
    const bool crossing_zero = (target * current) < 0.f;
    const bool reducing_magnitude = std::abs(target) < std::abs(current);
    const float max_step = (crossing_zero || reducing_magnitude ? max_brake : max_accel) * dt;
    const float delta = target - current;
    if (std::abs(delta) <= max_step) return target;
    return current + std::copysign(max_step, delta);
}
#endif

#if CMD_GOV_JERK_LIMIT_ENABLE
static float limit_velocity_jerk(float target, float current, float& prev_accel,
                                 float max_accel, float max_brake, float max_jerk, float dt) {
    if (dt <= 1e-6f) return target;

    const float raw_accel = (target - current) / dt;
    const bool crossing_zero = (target * current) < 0.f;
    const bool reducing_magnitude = std::abs(target) < std::abs(current);
    const float accel_limit = crossing_zero || reducing_magnitude ? max_brake : max_accel;
    const float desired_accel = clampf(raw_accel, -accel_limit, accel_limit);

    const float max_accel_delta = std::max(max_jerk, 0.f) * dt;
    float accel_cmd = desired_accel;
    if (max_accel_delta > 1e-6f) {
        const float accel_delta = desired_accel - prev_accel;
        if (accel_delta > max_accel_delta) {
            accel_cmd = prev_accel + max_accel_delta;
        } else if (accel_delta < -max_accel_delta) {
            accel_cmd = prev_accel - max_accel_delta;
        }
    }

    float next = current + accel_cmd * dt;

    // 防止接近目标时因为历史加速度惯性越过目标，越过则直接贴到目标并清加速度。
    if ((target - current) * (target - next) <= 0.f) {
        next = target;
        prev_accel = 0.f;
        return next;
    }

    prev_accel = accel_cmd;
    return next;
}
#endif

static float limit_proximity_cost(float angle) {
    const float effective_limit = std::max(STEER_ANGLE_LIMIT - STEER_LIMIT_MARGIN, 0.1f);
    const float margin = effective_limit - std::abs(angle);
    if (margin >= CANDIDATE_LIMIT_SOFT_ZONE) return 0.f;
    if (margin <= 0.f) return 1.f;
    return 1.f - margin / CANDIDATE_LIMIT_SOFT_ZONE;
}

// [诊断改动2] 诊断文件和滑动窗口分位数工具。
static bool file_is_empty_or_missing(const char* path) {
    std::ifstream f(path);
    return !f.good() || f.peek() == std::ifstream::traits_type::eof();
}

// [调参改动5] CSV 增加 dls_bypass_reason 列后，旧表头会导致后续数据列错位。
// 如果检测到旧 CSV 没有这一列，就自动重建 CSV，只保留新格式数据。
static bool csv_header_missing_column(const char* path, const char* column_name) {
    std::ifstream f(path);
    if (!f.good()) return false;
    std::string header;
    std::getline(f, header);
    return header.find(column_name) == std::string::npos;
}

static float percentile_of_samples(std::vector<float> samples, float p) {
    if (samples.empty()) return 0.f;
    std::sort(samples.begin(), samples.end());
    const float pos = clampf(p, 0.f, 1.f) * static_cast<float>(samples.size() - 1);
    const size_t lo = static_cast<size_t>(std::floor(pos));
    const size_t hi = static_cast<size_t>(std::ceil(pos));
    if (lo == hi) return samples[lo];
    const float a = pos - static_cast<float>(lo);
    return samples[lo] * (1.f - a) + samples[hi] * a;
}

static void push_limited_sample(std::vector<float>& samples, float value) {
    if (!std::isfinite(value)) return;
    samples.push_back(value);
    if (samples.size() > DLS_DIAG_MAX_SAMPLES) {
        samples.erase(samples.begin(), samples.begin() + (samples.size() - DLS_DIAG_MAX_SAMPLES));
    }
}

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
        RCLCPP_INFO(this->get_logger(), "Mobile Omni Controller initialized (方案4 MPPI jerk-aware governor + DLS + steer candidate memory)");

        for (int i = 0; i < 4; ++i) {
            prev_angle_[i] = 0.0f;
            prev_speed_rpm_[i] = 0.0f;
            prev_angle_last_[i] = 0.f;
            w_est_[i] = 0.f;
            prev_candidate_flipped_[i] = false;
            prev_drive_sign_[i] = 1.f;
        }
        steer_rate_initialized_ = false;
        cmd_governor_initialized_ = false;
        prev_cmd_vx_ = 0.0f;
        prev_cmd_vy_ = 0.0f;
        prev_cmd_vw_ = 0.0f;
        prev_cmd_ax_ = 0.0f;
        prev_cmd_ay_ = 0.0f;
        prev_cmd_aw_ = 0.0f;

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

    void reset_cmd_vel_governor() {
        prev_cmd_vx_ = 0.f;
        prev_cmd_vy_ = 0.f;
        prev_cmd_vw_ = 0.f;
        prev_cmd_ax_ = 0.f;
        prev_cmd_ay_ = 0.f;
        prev_cmd_aw_ = 0.f;
        cmd_governor_initialized_ = false;
    }

    // MPPI 后级轻量速度约束器：
    // 只限制 cmd_vel 的变化率，不看地图、不改路径跟踪目标。
    // 这样可以减少前进->横移、边走边转、刹车时的突变冲击。
    void apply_cmd_vel_governor(float vx_ref, float vy_ref, float vw_ref,
                                float& vx_cmd, float& vy_cmd, float& vw_cmd) {
#if CMD_GOVERNOR_ENABLE
        if (!cmd_governor_initialized_) {
            prev_cmd_vx_ = 0.f;
            prev_cmd_vy_ = 0.f;
            prev_cmd_vw_ = 0.f;
            prev_cmd_ax_ = 0.f;
            prev_cmd_ay_ = 0.f;
            prev_cmd_aw_ = 0.f;
            cmd_governor_initialized_ = true;
        }

#if CMD_GOV_JERK_LIMIT_ENABLE
        vx_cmd = limit_velocity_jerk(vx_ref, prev_cmd_vx_, prev_cmd_ax_, CMD_GOV_MAX_AX, CMD_GOV_MAX_BRAKE_X,
                                     CMD_GOV_MAX_JERK_X, CONTROL_PERIOD_S);
        vy_cmd = limit_velocity_jerk(vy_ref, prev_cmd_vy_, prev_cmd_ay_, CMD_GOV_MAX_AY, CMD_GOV_MAX_BRAKE_Y,
                                     CMD_GOV_MAX_JERK_Y, CONTROL_PERIOD_S);
        vw_cmd = limit_velocity_jerk(vw_ref, prev_cmd_vw_, prev_cmd_aw_, CMD_GOV_MAX_AW, CMD_GOV_MAX_BRAKE_W,
                                     CMD_GOV_MAX_JERK_W, CONTROL_PERIOD_S);
#else
        vx_cmd = limit_velocity_rate(vx_ref, prev_cmd_vx_, CMD_GOV_MAX_AX, CMD_GOV_MAX_BRAKE_X, CONTROL_PERIOD_S);
        vy_cmd = limit_velocity_rate(vy_ref, prev_cmd_vy_, CMD_GOV_MAX_AY, CMD_GOV_MAX_BRAKE_Y, CONTROL_PERIOD_S);
        vw_cmd = limit_velocity_rate(vw_ref, prev_cmd_vw_, CMD_GOV_MAX_AW, CMD_GOV_MAX_BRAKE_W, CONTROL_PERIOD_S);
#endif

        prev_cmd_vx_ = vx_cmd;
        prev_cmd_vy_ = vy_cmd;
        prev_cmd_vw_ = vw_cmd;
#else
        vx_cmd = vx_ref;
        vy_cmd = vy_ref;
        vw_cmd = vw_ref;
#endif
    }

    struct SteerCandidate {
        float angle{0.f};
        float drive_sign{1.f};
        bool flipped{false};
        bool valid{false};
        float cost{1e6f};
    };

    float candidate_cost(int i, const SteerCandidate& cand) const {
        if (!cand.valid) return 1e6f;

        // 基础代价：当前舵角到候选舵角的角度距离。
        float cost = std::abs(angle_diff(cand.angle, prev_angle_[i]));

        // 记忆代价：上一周期选 A，这一周期不要轻易切 B，反之亦然。
        // 这项主要针对 ±150deg 限位下边走边转时的反复换向。
        if (candidate_memory_initialized_ && cand.flipped != prev_candidate_flipped_[i]) {
            cost += CANDIDATE_SWITCH_COST;
        }

        // 轮速正反切换代价：angle+pi 等价解会让轮速方向反过来，频繁切会抖。
        if (candidate_memory_initialized_ && cand.drive_sign * prev_drive_sign_[i] < 0.f) {
            cost += CANDIDATE_REVERSE_COST;
        }

        // 限位代价：靠近有效限位时增加代价，给机械限位留余量。
        cost += CANDIDATE_LIMIT_COST_GAIN * limit_proximity_cost(cand.angle);
        return cost;
    }

    SteerCandidate select_steer_candidate(int i, float raw_angle) const {
        const float effective_limit = std::max(STEER_ANGLE_LIMIT - STEER_LIMIT_MARGIN, 0.1f);

        SteerCandidate a;
        a.angle = wrap_pi(raw_angle);
        a.drive_sign = 1.f;
        a.flipped = false;
        a.valid = std::abs(a.angle) <= effective_limit;
        a.cost = candidate_cost(i, a);

        SteerCandidate b;
        b.angle = wrap_pi(raw_angle + PI_CHASSIS);
        b.drive_sign = -1.f;
        b.flipped = true;
        b.valid = std::abs(b.angle) <= effective_limit;
        b.cost = candidate_cost(i, b);

        if (!a.valid && !b.valid) {
            // 理论上 effective_limit > 90deg 时，A/B 至少有一个可行。
            // 如果机械限位或零位参数错误导致都不可行，则夹到安全范围，避免发出越界角。
            SteerCandidate fallback;
            fallback.angle = clampf(a.angle, -effective_limit, effective_limit);
            fallback.drive_sign = 1.f;
            fallback.flipped = false;
            fallback.valid = true;
            fallback.cost = 1e5f;
            return fallback;
        }

        return (a.cost <= b.cost) ? a : b;
    }

    // [诊断改动3] 仅用于 mu0 统计分类：把横移、旋转、复合运动视为复杂样本。
    bool is_complex_cmd(float vx, float vy, float vw) const {
        return std::abs(vy) > 0.03f || std::abs(vw) > 0.10f ||
               (std::abs(vx) > 0.03f && std::abs(vw) > 0.05f);
    }

    void update_mu_calibration(float mu, float vx, float vy, float vw) {
        const float cmd_mag = std::hypot(vx, std::hypot(vy, vw));
        if (cmd_mag < 0.02f || mu < 1e-6f) {
            return;
        }

        push_limited_sample(mu_samples_active_, mu);
        if (is_complex_cmd(vx, vy, vw)) {
            push_limited_sample(mu_samples_complex_, mu);
        }

        if (mu_ema_ <= 0.f) {
            mu_ema_ = mu;
        } else {
            mu_ema_ = 0.98f * mu_ema_ + 0.02f * mu;
        }
        if (mu < mu_min_under_cmd_) {
            mu_min_under_cmd_ = mu;
        }

        const float p20_complex = percentile_of_samples(mu_samples_complex_, 0.20f);
        const float p20_active = percentile_of_samples(mu_samples_active_, 0.20f);
        if (mu_samples_complex_.size() >= 30) {
            mu_suggest_ = p20_complex;
        } else if (mu_samples_active_.size() >= 30) {
            mu_suggest_ = p20_active;
        } else {
            // 样本还少时保留旧估计，避免前几十帧建议值跳动。
            mu_suggest_ = std::max(mu_ema_ * 0.35f, mu_min_under_cmd_ * 1.2f);
        }
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

    // [软重载保护改动4] 换向等待状态机 + drive_scale 限斜率。
    // 注意这里用的是实际反馈角 prev_angle_ 与目标角 target_ang 的误差，不看理论指令本身。
    // 大换向时会把目标比例压低，但最终输出通过限斜率变化，避免底层速度环急刹导致车体抖动。
    float apply_heavy_load_steer_interlock(float raw_drive_scale, float max_steer_err) {
        if (max_steer_err > STEER_ALIGN_ENTER_ERR) {
            steer_align_hold_active_ = true;
            steer_align_ok_cycles_ = 0;
        }

        float target_drive_scale = raw_drive_scale;
        if (steer_align_hold_active_) {
            if (max_steer_err < STEER_ALIGN_RELEASE_ERR) {
                ++steer_align_ok_cycles_;
            } else {
                steer_align_ok_cycles_ = 0;
            }

            if (steer_align_ok_cycles_ >= STEER_ALIGN_HOLD_CYCLES) {
                steer_align_hold_active_ = false;
                target_drive_scale = raw_drive_scale;
            } else {
                target_drive_scale = std::min(raw_drive_scale, STEER_ALIGN_HOLD_TARGET_SCALE);
            }
        }

        if (!drive_scale_filter_initialized_) {
            drive_scale_filtered_ = target_drive_scale;
            drive_scale_filter_initialized_ = true;
            return drive_scale_filtered_;
        }

        const float delta = target_drive_scale - drive_scale_filtered_;
        if (delta > DRIVE_SCALE_MAX_RISE_PER_CYCLE) {
            drive_scale_filtered_ += DRIVE_SCALE_MAX_RISE_PER_CYCLE;
        } else if (delta < -DRIVE_SCALE_MAX_DROP_PER_CYCLE) {
            drive_scale_filtered_ -= DRIVE_SCALE_MAX_DROP_PER_CYCLE;
        } else {
            drive_scale_filtered_ = target_drive_scale;
        }

        drive_scale_filtered_ = clampf(drive_scale_filtered_, 0.f, 1.f);
        return drive_scale_filtered_;
    }

    // [诊断改动4] 把当前控制状态归类，写入 latest 文件方便直接看。
    const char* diagnose_state(float lambda, float drive_scale, float max_steer_err,
                               const char* dls_bypass_state, bool steer_align_hold_active) const {
        if (steer_align_hold_active) return "STEER_ALIGN_HOLD";
        if (dls_bypass_state != nullptr) return dls_bypass_state;
        if (drive_scale <= 0.05f) return "STEER_GATE_STOP";
        if (drive_scale < 0.80f) return "STEER_GATE_SLOW";
        if (max_steer_err > 0.50f) return "STEER_ERROR_HIGH";
        if (lambda > (DLS_LAMBDA_MIN + 0.60f * (DLS_LAMBDA_MAX - DLS_LAMBDA_MIN))) return "DLS_HEAVY";
        return "NORMAL";
    }

    const char* diagnose_advice(float lambda, float drive_scale, float max_steer_err, float ratio,
                                const char* dls_bypass_advice, bool steer_align_hold_active) const {
        if (steer_align_hold_active) return "waiting_for_actual_steer_alignment";
        if (dls_bypass_advice != nullptr) return dls_bypass_advice;
        if (drive_scale < 0.80f && max_steer_err > STEER_ERR_FULL_SPEED) {
            return "steer_error_gate_dominant";
        }
        if (lambda > (DLS_LAMBDA_MIN + 0.60f * (DLS_LAMBDA_MAX - DLS_LAMBDA_MIN)) && max_steer_err < 0.25f) {
            return "dls_maybe_too_conservative";
        }
        if (ratio > 0.95f && max_steer_err > 0.60f) {
            return "dls_not_main_limit_check_steer_or_gate";
        }
        return "balanced_or_insufficient_evidence";
    }

    // [诊断改动5] 写 /tmp/omni_dls_latest.txt 和 /tmp/omni_dls_samples.csv。
    void write_diagnostic_files(float vx, float vy, float vw, float vx_a, float vy_a, float vw_a, float mu,
                                float lambda, float drive_scale, float max_steer_err, const float steer_err[4],
                                const float target_ang[4], const bool low_vector_hold[4],
                                const float out_speed_rpm[4], const char* dls_bypass_state,
                                const char* dls_bypass_advice, bool steer_align_hold_active,
                                int steer_align_ok_cycles) {
        update_mu_calibration(mu, vx, vy, vw);

        const float ratio = (DLS_MU0 > 1e-9f) ? std::min(mu / DLS_MU0, 1.f) : 0.f;
        const float p10_active = percentile_of_samples(mu_samples_active_, 0.10f);
        const float p20_active = percentile_of_samples(mu_samples_active_, 0.20f);
        const float p30_active = percentile_of_samples(mu_samples_active_, 0.30f);
        const float p10_complex = percentile_of_samples(mu_samples_complex_, 0.10f);
        const float p20_complex = percentile_of_samples(mu_samples_complex_, 0.20f);
        const float p30_complex = percentile_of_samples(mu_samples_complex_, 0.30f);
        const bool complex = is_complex_cmd(vx, vy, vw);
        const bool dls_bypassed = (dls_bypass_state != nullptr);
        const char* state = diagnose_state(lambda, drive_scale, max_steer_err, dls_bypass_state,
                                           steer_align_hold_active);
        const char* advice = diagnose_advice(lambda, drive_scale, max_steer_err, ratio, dls_bypass_advice,
                                             steer_align_hold_active);
        const double now_sec = this->now().seconds();

        {
            std::ofstream latest(std::string(DLS_DIAG_LATEST_PATH) + ".tmp", std::ios::trunc);
            latest << std::fixed << std::setprecision(4);
            latest << "time_sec: " << now_sec << "\n";
            latest << "state: " << state << "\n";
            latest << "advice: " << advice << "\n";
            latest << "cmd: vx=" << vx << " vy=" << vy << " vw=" << vw
                   << " complex=" << (complex ? 1 : 0) << " dls_bypassed=" << (dls_bypassed ? 1 : 0) << "\n";
            latest << "cmd_governor: enable=" << CMD_GOVERNOR_ENABLE
                   << " jerk_enable=" << CMD_GOV_JERK_LIMIT_ENABLE
                   << " ref_vx=" << last_cmd_vx_ref_ << " ref_vy=" << last_cmd_vy_ref_
                   << " ref_vw=" << last_cmd_vw_ref_
                   << " governed_vx=" << vx << " governed_vy=" << vy << " governed_vw=" << vw
                   << " accel_x=" << prev_cmd_ax_ << " accel_y=" << prev_cmd_ay_
                   << " accel_w=" << prev_cmd_aw_ << "\n";
            latest << "dls_bypass_reason: " << (dls_bypassed ? state : "none") << "\n";
            latest << "achievable: vx=" << vx_a << " vy=" << vy_a << " vw=" << vw_a << "\n";
            latest << "dls: mu=" << mu << " ratio=" << ratio << " lambda=" << lambda
                   << " cfg_mu0=" << DLS_MU0 << " suggested_mu0=" << mu_suggest_ << "\n";
            latest << "gate: drive_scale=" << drive_scale << " max_steer_err=" << max_steer_err << "\n";
            latest << "steer_align_hold: active=" << (steer_align_hold_active ? 1 : 0)
                   << " ok_cycles=" << steer_align_ok_cycles
                   << " enter_err=" << STEER_ALIGN_ENTER_ERR
                   << " release_err=" << STEER_ALIGN_RELEASE_ERR
                   << " hold_cycles=" << STEER_ALIGN_HOLD_CYCLES << "\n";
            latest << "drive_scale_filter: filtered=" << drive_scale_filtered_
                   << " max_drop_per_cycle=" << DRIVE_SCALE_MAX_DROP_PER_CYCLE
                   << " max_rise_per_cycle=" << DRIVE_SCALE_MAX_RISE_PER_CYCLE
                   << " hold_target_scale=" << STEER_ALIGN_HOLD_TARGET_SCALE << "\n";
            latest << "steer_err: [" << steer_err[0] << ", " << steer_err[1] << ", " << steer_err[2]
                   << ", " << steer_err[3] << "]\n";
            latest << "target_angle: [" << target_ang[0] << ", " << target_ang[1] << ", " << target_ang[2]
                   << ", " << target_ang[3] << "]\n";
            latest << "actual_angle: [" << prev_angle_[0] << ", " << prev_angle_[1] << ", " << prev_angle_[2]
                   << ", " << prev_angle_[3] << "]\n";
            latest << "rpm_cmd: [" << out_speed_rpm[0] << ", " << out_speed_rpm[1] << ", "
                   << out_speed_rpm[2] << ", " << out_speed_rpm[3] << "]\n";
            latest << "low_vector_hold: [" << low_vector_hold[0] << ", " << low_vector_hold[1] << ", "
                   << low_vector_hold[2] << ", " << low_vector_hold[3] << "]\n";
            latest << "mu_samples_active: n=" << mu_samples_active_.size() << " p10=" << p10_active
                   << " p20=" << p20_active << " p30=" << p30_active << "\n";
            latest << "mu_samples_complex: n=" << mu_samples_complex_.size() << " p10=" << p10_complex
                   << " p20=" << p20_complex << " p30=" << p30_complex << "\n";
            latest << "mu0_rule: use complex P20 when complex sample count >= 30; otherwise active P20.\n";
        }
        std::rename((std::string(DLS_DIAG_LATEST_PATH) + ".tmp").c_str(), DLS_DIAG_LATEST_PATH);

        const bool need_header = file_is_empty_or_missing(DLS_DIAG_CSV_PATH) ||
                                 csv_header_missing_column(DLS_DIAG_CSV_PATH, "dls_bypass_reason") ||
                                 csv_header_missing_column(DLS_DIAG_CSV_PATH, "steer_align_hold");
        std::ofstream csv(DLS_DIAG_CSV_PATH, need_header ? std::ios::trunc : std::ios::app);
        if (need_header) {
            csv << "time_sec,state,advice,complex,dls_bypassed,dls_bypass_reason,vx,vy,vw,vx_a,vy_a,vw_a,mu,ratio,lambda,cfg_mu0,"
                   "suggested_mu0,drive_scale,max_steer_err,steer_align_hold,steer_align_ok_cycles,err0,err1,err2,err3,target0,target1,target2,"
                   "target3,actual0,actual1,actual2,actual3,rpm0,rpm1,rpm2,rpm3,low0,low1,low2,low3,"
                   "active_n,active_p10,active_p20,active_p30,complex_n,complex_p10,complex_p20,complex_p30\n";
        }
        csv << std::fixed << std::setprecision(6)
            << now_sec << "," << state << "," << advice << "," << (complex ? 1 : 0) << ","
            << (dls_bypassed ? 1 : 0) << "," << (dls_bypassed ? state : "none") << ","
            << vx << "," << vy << "," << vw << "," << vx_a << "," << vy_a << "," << vw_a << ","
            << mu << "," << ratio << "," << lambda << "," << DLS_MU0 << "," << mu_suggest_ << ","
            << drive_scale << "," << max_steer_err << ","
            << (steer_align_hold_active ? 1 : 0) << "," << steer_align_ok_cycles << ","
            << steer_err[0] << "," << steer_err[1] << "," << steer_err[2] << "," << steer_err[3] << ","
            << target_ang[0] << "," << target_ang[1] << "," << target_ang[2] << "," << target_ang[3] << ","
            << prev_angle_[0] << "," << prev_angle_[1] << "," << prev_angle_[2] << "," << prev_angle_[3] << ","
            << out_speed_rpm[0] << "," << out_speed_rpm[1] << "," << out_speed_rpm[2] << ","
            << out_speed_rpm[3] << ","
            << low_vector_hold[0] << "," << low_vector_hold[1] << "," << low_vector_hold[2] << ","
            << low_vector_hold[3] << ","
            << mu_samples_active_.size() << "," << p10_active << "," << p20_active << "," << p30_active << ","
            << mu_samples_complex_.size() << "," << p10_complex << "," << p20_complex << "," << p30_complex
            << "\n";
    }

    void compute_wheels(float vx, float vy, float vw, float out_angle[4], float out_speed_rpm[4]) {
        // mm -> m；H 矩阵与 μ 均基于米制，CHASSIS_SIZE_* 为 mm 不影响 μ（此处已换算）
        const float L = CHASSIS_SIZE_L * 0.001f;
        const float W = CHASSIS_SIZE_W * 0.001f;

        update_steer_rate_from_feedback();

        const bool cmd_is_zero =
            (std::hypot(vx, vy) < CMD_LINEAR_DEADBAND) && (std::abs(vw) < CMD_ANGULAR_DEADBAND);
        if (cmd_is_zero) {
            steer_align_hold_active_ = false;
            steer_align_ok_cycles_ = 0;
            drive_scale_filtered_ = 0.f;
            drive_scale_filter_initialized_ = true;
            for (int i = 0; i < 4; ++i) {
                out_angle[i] = prev_angle_[i];
                out_speed_rpm[i] = 0.f;
            }
            return;
        }

        float target_ang[4];
        float corner_vx[4];
        float corner_vy[4];
        float kx_tab[4];
        float ky_tab[4];
        bool low_vector_hold[4] = {};
        float steer_err[4] = {};
        bool selected_flipped[4] = {};
        float selected_drive_sign[4] = {};

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

            if (speed < WHEEL_VECTOR_DEADBAND) {
                target_ang[i] = prev_angle_[i];
                corner_vx[i] = 0.f;
                corner_vy[i] = 0.f;
                low_vector_hold[i] = true;
                continue;
            }

            const float ang = std::atan2(vy_i, vx_i);

            const SteerCandidate selected = select_steer_candidate(i, ang);
            target_ang[i] = selected.angle;
            selected_flipped[i] = selected.flipped;
            selected_drive_sign[i] = selected.drive_sign;
        }

        candidate_memory_initialized_ = true;
        for (int i = 0; i < 4; ++i) {
            if (!low_vector_hold[i]) {
                prev_candidate_flipped_[i] = selected_flipped[i];
                prev_drive_sign_[i] = selected_drive_sign[i];
            }
        }

        float max_steer_err = 0.f;
        bool any_low_vector_hold = false;
        for (int i = 0; i < 4; ++i) {
            steer_err[i] = std::abs(angle_diff(target_ang[i], prev_angle_[i]));
            max_steer_err = std::max(max_steer_err, steer_err[i]);
            any_low_vector_hold = any_low_vector_hold || low_vector_hold[i];
        }
        const float raw_drive_scale = global_drive_scale_from_steer_error(max_steer_err);
        const float drive_scale = apply_heavy_load_steer_interlock(raw_drive_scale, max_steer_err);

        float vx_a = 0.f;
        float vy_a = 0.f;
        float vw_a = 0.f;
        float mu = 0.f;
        float lambda = 0.f;

        // [改动3] 纯前进/后退时绕过动态 DLS。
        // 原因：四个轮子都朝前时，H 对完整 (vx,vy,vw) 是退化的，mu≈0；
        // 但如果当前指令只有 vx，这个退化并不影响前后运动。继续用动态 DLS 会得到
        // lambda=DLS_LAMBDA_MAX，导致纯前进/后退也被不必要地压钝。
        const bool bypass_dls_for_simple_vx = is_simple_longitudinal_cmd(vx, vy, vw);

        // [调参改动4] 纯横移时也绕过动态 DLS。
        // 原因与纯 vx 类似：四轮接近 ±90 deg 时完整 3DOF 的 mu 可能偏小，
        // 但当前只要求 vy，横移本身是可实现的。这里交给舵角误差门控保护轮胎，不让 DLS 误降速。
        const bool bypass_dls_for_simple_vy = is_simple_lateral_cmd(vx, vy, vw);
        const bool bypass_dls_for_simple_axis = bypass_dls_for_simple_vx || bypass_dls_for_simple_vy;
        const char* dls_bypass_state = bypass_dls_for_simple_vx ? "SIMPLE_VX_DLS_BYPASS"
                                     : bypass_dls_for_simple_vy ? "SIMPLE_VY_DLS_BYPASS"
                                                                : nullptr;
        const char* dls_bypass_advice = bypass_dls_for_simple_vx ? "simple_vx_dls_bypassed"
                                      : bypass_dls_for_simple_vy ? "simple_vy_dls_bypassed"
                                                                 : nullptr;

        if (bypass_dls_for_simple_axis) {
            vx_a = vx;
            vy_a = vy;
            vw_a = vw;
            mu = 0.f;  // 这里保留为 0，仅表示当前姿态对完整三自由度退化
            lambda = DLS_LAMBDA_MIN;
        } else {
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
        }

        log_dls_diagnostics(mu, lambda, vx_a, vy_a, vw_a, vx, vy, vw);

        if (bypass_dls_for_simple_vx) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "DLS_BYPASS_SIMPLE_VX: cmd=(%.3f,%.3f,%.3f) u_ach=(%.3f,%.3f,%.3f) lambda=%.3f "
                "max_steer_err=%.3f drive_scale=%.2f",
                vx, vy, vw, vx_a, vy_a, vw_a, lambda, max_steer_err, drive_scale);
        }
        if (bypass_dls_for_simple_vy) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "DLS_BYPASS_SIMPLE_VY: cmd=(%.3f,%.3f,%.3f) u_ach=(%.3f,%.3f,%.3f) lambda=%.3f "
                "max_steer_err=%.3f drive_scale=%.2f",
                vx, vy, vw, vx_a, vy_a, vw_a, lambda, max_steer_err, drive_scale);
        }

        if (drive_scale < 0.999f || any_low_vector_hold) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 500,
                "STEER_GATE: max_err=%.3f rad raw_scale=%.2f scale=%.2f align_hold=%d ok_cycles=%d low_vec=[%d,%d,%d,%d] target=[%.2f,%.2f,%.2f,%.2f] actual=[%.2f,%.2f,%.2f,%.2f]",
                max_steer_err, raw_drive_scale, drive_scale, steer_align_hold_active_ ? 1 : 0,
                steer_align_ok_cycles_, low_vector_hold[0], low_vector_hold[1], low_vector_hold[2],
                low_vector_hold[3], target_ang[0], target_ang[1], target_ang[2], target_ang[3], prev_angle_[0],
                prev_angle_[1], prev_angle_[2], prev_angle_[3]);
        }

        for (int i = 0; i < 4; ++i) {
            const float max_step = steer_max_step_for_wheel(i);
            //out_angle[i] = ramp_angle(target_ang[i], prev_angle_[i], max_step);
            out_angle[i] = target_ang[i];  // 直接使用目标角，避免舵角滞后导致的轮速门控

            const float theta = prev_angle_[i];
            const float vx_i_a = vx_a + kx_tab[i] * W * vw_a * 0.5f;
            const float vy_i_a = vy_a + ky_tab[i] * L * vw_a * 0.5f;
            const float v_wheel = vx_i_a * std::cos(theta) + vy_i_a * std::sin(theta);

            // 整车 drive_scale 处理大换向/整体未对准；
            // 单轮 wheel_scale 只压低没对准的那个轮子，减少轮胎横向拖拽。
#if PER_WHEEL_STEER_SCALE_ENABLE
            const float wheel_scale = drive_scale_from_steer_error(steer_err[i]);
#else
            const float wheel_scale = 1.f;
#endif
            const float target_rpm_i = SPEED_TO_RPM(v_wheel) * drive_scale * wheel_scale;
            if (drive_scale <= 1e-3f) {
                out_speed_rpm[i] = 0.f;
            } else {
                const float diff = target_rpm_i - prev_speed_rpm_[i];
                out_speed_rpm[i] = prev_speed_rpm_[i] + diff * SPEED_RAMP_RATE;
            }
        }

        // [诊断改动6] 只有完成一次有效非零控制计算后才更新文件；
        // cmd 归零时会保留 latest 的上一次有效分析。
        write_diagnostic_files(vx, vy, vw, vx_a, vy_a, vw_a, mu, lambda, drive_scale, max_steer_err, steer_err,
                               target_ang, low_vector_hold, out_speed_rpm, dls_bypass_state, dls_bypass_advice,
                               steer_align_hold_active_, steer_align_ok_cycles_);
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
        const bool cmd_timeout = (this->now() - cmd_time).seconds() > timeout_sec;
        if (cmd_timeout) {
            cmd = geometry_msgs::msg::Twist();
            reset_cmd_vel_governor();
        }

        const float vx_ref = static_cast<float>(cmd.linear.x);
        const float vy_ref = static_cast<float>(cmd.linear.y);
        const float vw_ref = static_cast<float>(cmd.angular.z);
        last_cmd_vx_ref_ = vx_ref;
        last_cmd_vy_ref_ = vy_ref;
        last_cmd_vw_ref_ = vw_ref;

        float vx = 0.f;
        float vy = 0.f;
        float vw = 0.f;
        if (cmd_timeout) {
            // 通信超时时保持原来的安全策略：立即发 0。
            // 正常松杆/MPPI 输出 0 时，会走 apply_cmd_vel_governor() 做受控减速。
            vx = 0.f;
            vy = 0.f;
            vw = 0.f;
        } else {
            apply_cmd_vel_governor(vx_ref, vy_ref, vw_ref, vx, vy, vw);
        }

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

    float prev_cmd_vx_;
    float prev_cmd_vy_;
    float prev_cmd_vw_;
    float prev_cmd_ax_;
    float prev_cmd_ay_;
    float prev_cmd_aw_;
    bool cmd_governor_initialized_{false};
    float last_cmd_vx_ref_{0.f};
    float last_cmd_vy_ref_{0.f};
    float last_cmd_vw_ref_{0.f};

    float prev_angle_last_[4];
    float w_est_[4];
    bool steer_rate_initialized_{false};

    bool prev_candidate_flipped_[4];
    float prev_drive_sign_[4];
    bool candidate_memory_initialized_{false};

    float mu_ema_{0.f};
    float mu_min_under_cmd_{1e9f};
    float mu_suggest_{DLS_MU0};
    std::vector<float> mu_samples_active_;
    std::vector<float> mu_samples_complex_;

    // [软重载保护改动5] 舵角到位互锁和 drive_scale 滤波状态。
    // active=1 时不会瞬间急刹，而是把目标 drive_scale 压低，再通过限斜率平滑变化。
    bool steer_align_hold_active_{false};
    int steer_align_ok_cycles_{0};
    float drive_scale_filtered_{0.f};
    bool drive_scale_filter_initialized_{false};

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
    // 固定 λ 更简单；动态 λ 在四轮退化时自动加大阻尼。
    // 方案4 额外增加 MPPI 后级 jerk-aware 速度约束、舵角候选记忆、单轮舵角误差缩放。
    // =========================================================================
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MobileOmniController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
