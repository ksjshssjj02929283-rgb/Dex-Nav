#include "path_follower/velocity_step_model.hpp"

#include <algorithm>
#include <cmath>

namespace path_follower
{
    namespace
    {
        constexpr double kEpsilon = 1e-9;
    } // namespace

    double stepVelocityTowards(
        double v_prev, double v_target, double dt, const VelocityStepParams& params)
    {
        const double diff = v_target - v_prev;
        if (std::fabs(diff) <= kEpsilon || dt <= 0.0)
        {
            return v_target;
        }

        // 提速还是降速按"速度幅值"判断：幅值变大用 max_accel，幅值变小（含穿越 0 反向）
        // 一律按 max_decel 处理——反向意味着先要把当前速度杀到 0，用减速度限更保守也更合理。
        const bool speeding_up = std::fabs(v_target) > std::fabs(v_prev);
        const double a_max = std::max(speeding_up ? params.max_accel : params.max_decel, kEpsilon);
        const double decay_t = std::max(params.accel_decay_time, kEpsilon);

        // jerk 由加速度衰减时间派生；delta_v 是衰减段自身能吃掉的速度差（三角形面积）。
        const double jerk = a_max / decay_t;
        const double delta_v = 0.5 * a_max * decay_t;

        const double remain = std::fabs(diff);
        // 阶段A：硬顶最大加/减速度；阶段B：按能量关系反解衰减段的瞬时加速度。
        // remain == delta_v 时两式都等于 a_max，边界连续；remain -> 0 时 a -> 0，收尾平滑。
        const double a = (remain > delta_v) ? a_max : std::sqrt(2.0 * jerk * remain);

        double v_new = v_prev + std::copysign(a * dt, diff);
        // 离散步进下不允许越过目标：收尾段 a 很小但 a*dt 相对 remain 仍偏大，
        // 靠这一步把最后一点残差直接吃掉，保证有限步收敛而不是无限逼近。
        if ((diff > 0.0 && v_new > v_target) || (diff < 0.0 && v_new < v_target))
        {
            v_new = v_target;
        }
        return v_new;
    }
} // namespace path_follower
