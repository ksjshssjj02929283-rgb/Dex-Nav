#include "path_follower/pid_controller.hpp"

#include <algorithm>
#include <cmath>

namespace path_follower
{
    void PidController::configure(
        const PidAxisConfig& x_cfg,
        const PidAxisConfig& y_cfg,
        const PidAxisConfig& theta_cfg,
        const std::array<double, kGainLevelCount>& vel_levels,
        double max_correct_vy,
        double max_correct_w)
    {
        x_cfg_ = x_cfg;
        y_cfg_ = y_cfg;
        theta_cfg_ = theta_cfg;
        vel_levels_ = vel_levels;
        max_correct_vy_ = max_correct_vy;
        max_correct_w_ = max_correct_w;
        reset();
    }

    void PidController::reset()
    {
        y_state_ = AxisState{};
        theta_state_ = AxisState{};
    }

    std::size_t PidController::selectLevel(double current_linear_velocity) const
    {
        // 忠实复现 PID_Parameters_Speed_Choose：按 |v| 落入 vel_levels 区间选档。
        const double v = std::fabs(current_linear_velocity);
        for (std::size_t i = 0; i < kGainLevelCount; ++i)
        {
            if (v < vel_levels_[i])
            {
                return i;
            }
        }
        return kGainLevelCount - 1; // 超过最高档阈值 -> 最高档
    }

    double PidController::stepIncrement(
        const AxisState& prev,
        const PidGainTable& gains,
        std::size_t level,
        double error)
    {
        // 增量式：Δout = Kp·(e_k - e_{k-1}) + Kd·(e_k - 2·e_{k-1} + e_{k-2}) + Ki·e_k
        const double kp = gains.kp[level];
        const double ki = gains.ki[level];
        const double kd = gains.kd[level];

        const double delta_p = error - prev.err1;
        const double delta_d = error - 2.0 * prev.err1 + prev.err2;

        return kp * delta_p + kd * delta_d + ki * error;
    }

    CorrectionVelocity PidController::compute(
        const TrackingError& error,
        double current_linear_velocity,
        double reference_linear_v,
        double reference_w,
        bool have_vy,
        double diff_lateral_gain,
        double max_percentage_vy_to_vx) const
    {
        // 差速折叠与 Vy/Vx 限幅由外层 PathTracker 处理；这里保留参数以明确接口。
        (void)diff_lateral_gain;
        (void)max_percentage_vy_to_vx;

        CorrectionVelocity out;

        std::size_t level = selectLevel(current_linear_velocity);

        // ---- 横向 y 增量式纠偏（轨迹坐标系）----
        if (have_vy)
        {
            // 全向：直接对横向误差做增量式 PID
            y_state_.err2 = y_state_.err1;
            y_state_.err1 = y_state_.err0;
            y_state_.err0 = error.delta_y;

            const double delta_out =
                stepIncrement(y_state_, y_cfg_.gains, level, error.delta_y);
            y_state_.out += delta_out;
            y_state_.out = std::clamp(y_state_.out, -max_correct_vy_, max_correct_vy_);
            out.vy = y_state_.out;
        }
        else
        {
            // 差速：横向误差经 FollowPath 非线性律折叠到航向（由外层调用处理），
            // 这里把横向贡献叠加到参考航向速度，vy 置零。
            // （外层 PathTracker 已把折叠后的航向作为 delta_theta 传入，
            //  故此处不再单独处理。）
            out.vy = 0.0;
            y_state_ = AxisState{};
        }

        // ---- 航向 θ 增量式纠偏（轨迹坐标系） ----
        theta_state_.err2 = theta_state_.err1;
        theta_state_.err1 = theta_state_.err0;
        theta_state_.err0 = error.delta_theta;

        level = selectLevel(reference_w);
        const double theta_delta_out =
            stepIncrement(theta_state_, theta_cfg_.gains, level, error.delta_theta);
        theta_state_.out += theta_delta_out;
        theta_state_.out = std::clamp(theta_state_.out, -max_correct_w_, max_correct_w_);
        out.wz = theta_state_.out;

        // ---- 纵向 x：参考速度前馈（外层速度规划提供），不在此做 PID ----
        out.vx = reference_linear_v;

        // 纠偏速度（不含参考）+ 参考角速度（差速必须加上）
        // out.wz += reference_w;
        // out.wz = reference_w;

        return out;
    }
} // namespace path_follower