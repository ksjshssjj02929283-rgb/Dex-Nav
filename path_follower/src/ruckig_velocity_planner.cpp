#include "path_follower/ruckig_velocity_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <ruckig/ruckig.hpp>

namespace path_follower
{
    namespace
    {
        constexpr double kEpsilon = 1e-9;
    }

    bool planRuckigVelocityProfile(
        const std::vector<double>& dist,
        const RuckigSeed& seed,
        double v_max, double v_target,
        double a_max, double d_max, double jerk_max,
        double ctrl_dt,
        std::vector<double>& v_out,
        std::vector<double>& a_out)
    {
        const std::size_t n = dist.size();
        v_out.assign(n, seed.v);
        a_out.assign(n, seed.a);
        if (n == 0)
        {
            return true;
        }

        a_max = std::max(a_max, kEpsilon);
        d_max = std::max(d_max, kEpsilon);
        jerk_max = std::max(jerk_max, kEpsilon);
        ctrl_dt = std::max(ctrl_dt, kEpsilon);
        // 注意：v_max 允许小于 seed.v（current_velocity 超过 max_velocity 是 Ruckig 合法输入，
        // 且正是 SLOWDOWN/STOP 触发快速减速的设计意图——已实测验证会立即按 jerk/a_max 减速，
        // 不是"温和收加速度"），这里绝不能把 v_max 兜底抬到 seed.v。只兜底 v_max 不能低于
        // target_velocity（Ruckig 要求 |target_velocity| <= max_velocity）或退化到 <=0。
        v_max = std::max({v_max, std::fabs(v_target), kEpsilon});

        if (dist.back() < 1e-6)
        {
            // 剩余距离几乎为 0（比如重规划时只剩最后 1-2 个点）：无路可规划，直接给目标速度。
            std::fill(v_out.begin(), v_out.end(), v_target);
            std::fill(a_out.begin(), a_out.end(), 0.0);
            return true;
        }
        if (n == 1)
        {
            v_out[0] = seed.v;
            a_out[0] = seed.a;
            return true;
        }

        ruckig::Ruckig<1> otg(ctrl_dt);
        ruckig::InputParameter<1> in;
        ruckig::OutputParameter<1> out;
        in.current_position = {0.0};
        in.current_velocity = {seed.v};
        in.current_acceleration = {std::clamp(seed.a, -d_max, a_max)};
        in.target_position = {dist.back()};
        in.target_velocity = {v_target};
        in.target_acceleration = {0.0};
        in.max_velocity = {v_max};
        // min_velocity/min_acceleration 是 Ruckig 的可选非对称边界（std::optional<array>），
        // 需要显式构造 std::array 再赋值，直接花括号初始化对 optional 做不到模板推导。
        // 注意：ROS2 Humble 链接的 ruckig 版本里 validate_input() 要求 min_velocity 严格小于 0
        // （检查是 vMin >= -std::numeric_limits<double>::min()，即不接受精确的 0.0），传 0.0
        // 会导致 update() 第一次调用就返回 ErrorInvalidInput、整段退化成种子速度常量——
        // 这里用一个数值上无意义的极小负数，既满足校验又等价于"禁止倒退"。
        constexpr double kNegligibleReverseVelocity = -1.0e-6;
        in.min_velocity = std::array<double, 1>{kNegligibleReverseVelocity};
        in.max_acceleration = {a_max};
        in.min_acceleration = std::array<double, 1>{-d_max};
        in.max_jerk = {jerk_max};

        std::vector<double> rs{0.0}, rv{seed.v}, ra{seed.a};
        int steps_left = 200000;
        bool failed = false;
        ruckig::Result res = ruckig::Result::Working;
        while (res == ruckig::Result::Working && steps_left-- > 0)
        {
            res = otg.update(in, out);
            if (res == ruckig::Result::Error || res == ruckig::Result::ErrorInvalidInput)
            {
                failed = true;
                break;
            }
            rs.push_back(out.new_position[0]);
            rv.push_back(out.new_velocity[0]);
            ra.push_back(out.new_acceleration[0]);
            out.pass_to_input(in);
        }

        if (res == ruckig::Result::Working)
        {
            // 200000 步内没能收敛到 Finished（也不是显式 Error）：不能当成成功继续插值，
            // 否则会拿一条没走完、状态可能已经发散的 rs/rv/ra 去插值，产生垃圾输出。
            RCLCPP_ERROR(
                rclcpp::get_logger("velocity_planner"),
                "Ruckig position planning DID NOT CONVERGE within step budget: "
                "res=Working steps_used=%d rs.size()=%zu seed_v=%.4f seed_a=%.4f "
                "target_position=%.4f target_v=%.4f v_max=%.4f a_max=%.4f d_max=%.4f jerk=%.4f",
                200000, rs.size(), seed.v, seed.a,
                dist.back(), v_target, v_max, a_max, d_max, jerk_max);
            failed = true;
        }

        if (failed)
        {
            // 规划失败：退化为全程保持种子速度/加速度，不让控制环拿到未初始化数据。
            std::fill(v_out.begin(), v_out.end(), seed.v);
            std::fill(a_out.begin(), a_out.end(), seed.a);
            return false;
        }

        // 按 dist 网格线性插值（rs/dist 都单调递增，双指针一次扫描）。
        std::size_t k = 0;
        for (std::size_t i = 0; i < n; ++i)
        {
            while (k + 1 < rs.size() && rs[k + 1] < dist[i])
            {
                ++k;
            }
            if (k + 1 >= rs.size())
            {
                v_out[i] = rv.back();
                a_out[i] = ra.back();
            }
            else
            {
                const double denom = rs[k + 1] - rs[k];
                const double f = (denom > kEpsilon)
                    ? std::clamp((dist[i] - rs[k]) / denom, 0.0, 1.0)
                    : 0.0;
                v_out[i] = rv[k] + f * (rv[k + 1] - rv[k]);
                a_out[i] = ra[k] + f * (ra[k + 1] - ra[k]);
            }
        }
        return true;
    }

} // namespace path_follower
