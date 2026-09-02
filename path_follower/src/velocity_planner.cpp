#include "path_follower/velocity_planner.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <angles/angles.h>
#include <rclcpp/rclcpp.hpp>

#include "path_follower/ruckig_velocity_planner.hpp"
#include "path_follower/velocity_envelope.hpp"

namespace path_follower
{
    namespace
    {
        constexpr double kEpsilon = 1e-9;

        /// 短路径阈值 m：短于此长度的路径，起步加速度取 0（原因见 planPosition）。
        /// 阈值需要覆盖"把起步加速度压回 0 所需的行程" v0·(a0/jerk) + a0³/(3·jerk²)：
        /// 按默认参数（a0 = max_linear_acceleration/2 = 0.2、jerk = 0.2）该行程约 0.087m，
        /// 0.4m 留了足够余量。若大幅调高 max_linear_acceleration 或调低 max_linear_jerk，
        /// 需要重新评估这个阈值。
        constexpr double kShortPathDistance = 0.4;

        /// 短转角阈值 rad：小于此转角的朝向规划，起步角加速度取 0（原因同 kShortPathDistance）。
        /// 同一个公式换成角量：w0·(α0/jerk) + α0³/(3·jerk²)，按默认参数
        /// （w0 = min_angular_velocity = 0.02、α0 = max_angular_acceleration/2 = 0.3、
        /// jerk = max_angular_jerk = 0.3）约 0.12 rad，取 0.6 rad（≈34°）留出与线量那边
        /// 相当的余量（3.3x vs 3.5x）。若调整 max_angular_acceleration 或 max_angular_jerk，
        /// 需要按上式重新评估这个阈值。
        constexpr double kShortRotationAngle = 0.9;
    } // namespace

    VelocityPlanner::VelocityPlanner(const VelocityPlannerParams& params)
        : params_(params)
    {
    }

    void VelocityPlanner::computeArcLengthAndHeading(Route& route, bool trust_input_heading) const
    {
        if (route.empty())
        {
            return;
        }
        route.points[0].current_s = 0.0;

        // 弧长：两种模式都一样，逐点累加弦长。
        double s = 0.0;
        for (std::size_t i = 1; i < route.points.size(); ++i)
        {
            const double dx = route.points[i].x - route.points[i - 1].x;
            const double dy = route.points[i].y - route.points[i - 1].y;
            s += std::hypot(dx, dy);
            route.points[i].current_s = s;
        }
        route.total_s = s;

        if (trust_input_heading)
        {
            // 采信上游填入的解析切向，只做 unwrap：逐点的 atan2 结果落在 (-pi, pi]，
            // 过 ±pi 会跳 2pi，直接差分求曲率会出一个巨大的假尖峰。unwrap 之后
            // heading_angle 变成连续量（可能超出 ±pi），消费端凡是要比较角度的地方
            // 本来就走 angles::shortest_angular_distance，不受影响。
            for (std::size_t i = 1; i < route.points.size(); ++i)
            {
                route.points[i].heading_angle =
                    route.points[i - 1].heading_angle +
                    angles::shortest_angular_distance(route.points[i - 1].heading_angle,
                                                      route.points[i].heading_angle);
            }
            return;
        }

        // 自算：段航向 = 该点指向下一点的弦向（末点用倒数第二点的值）。
        // 注意这个值天然滞后半格（真实切向约在 heading + kappa*ds/2），
        // 采信上游解析切向时没有这个滞后。
        if (route.points.size() > 1)
        {
            route.points[0].heading_angle = std::atan2(
                route.points[1].y - route.points[0].y,
                route.points[1].x - route.points[0].x);
        }
        else
        {
            route.points[0].heading_angle = 0.0;
            return;
        }
        for (std::size_t i = 0; i < route.points.size(); ++i)
        {
            if (i + 1 < route.points.size())
            {
                const double ndx = route.points[i + 1].x - route.points[i].x;
                const double ndy = route.points[i + 1].y - route.points[i].y;
                route.points[i].heading_angle = std::atan2(ndy, ndx);
            }
            else
            {
                route.points[i].heading_angle = route.points[i - 1].heading_angle;
            }
        }
    }

    void VelocityPlanner::computeCurvature(Route& route, bool from_heading) const
    {
        if (route.size() < 3)
        {
            return;
        }

        if (from_heading)
        {
            // kappa = dtheta/ds，中心差分。heading_angle 已被 unwrap 成连续量，
            // 直接相减即可，不需要再取最短角差。
            for (std::size_t i = 1; i + 1 < route.points.size(); ++i)
            {
                const double ds =
                    route.points[i + 1].current_s - route.points[i - 1].current_s;
                route.points[i].curvature =
                    (ds > kEpsilon)
                        ? (route.points[i + 1].heading_angle - route.points[i - 1].heading_angle) / ds
                        : 0.0;
            }
            route.points.front().curvature = route.points[1].curvature;
            route.points.back().curvature = route.points[route.size() - 2].curvature;
            return;
        }

        for (std::size_t i = 1; i + 1 < route.points.size(); ++i)
        {
            const auto& a = route.points[i - 1];
            const auto& b = route.points[i];
            const auto& c = route.points[i + 1];
            // 三点外接圆半径法
            const double abx = b.x - a.x;
            const double aby = b.y - a.y;
            const double bcx = c.x - b.x;
            const double bcy = c.y - b.y;
            const double cross = abx * bcy - aby * bcx;
            const double a1 = std::hypot(b.x - a.x, b.y - a.y);
            const double a2 = std::hypot(c.x - b.x, c.y - b.y);
            const double a3 = std::hypot(c.x - a.x, c.y - a.y);
            const double s = 0.5 * (a1 + a2 + a3);
            const double area_sq = std::max(s * (s - a1) * (s - a2) * (s - a3), 0.0);
            const double area = std::sqrt(area_sq);
            if (area < kEpsilon || std::fabs(cross) < kEpsilon)
            {
                route.points[i].curvature = 0.0;
            }
            else
            {
                const double curvature = 4.0 * area / (a1 * a2 * a3);
                route.points[i].curvature = curvature * std::copysign(1.0, cross);
            }
        }
        route.points.front().curvature = route.points[1].curvature;
        route.points.back().curvature = route.points[route.size() - 2].curvature;
    }

    void VelocityPlanner::applyLegacyProfile(Route& route) const
    {
        const std::size_t n = route.points.size();
        std::vector<double> arc(n), kappa(n), v(n), a(n);
        for (std::size_t i = 0; i < n; ++i)
        {
            arc[i] = route.points[i].current_s;
            kappa[i] = route.points[i].curvature;
            v[i] = route.points[i].linear_v;
            a[i] = route.points[i].linear_a;
        }

        if (params_.curvature_deceleration)
        {
            legacyCurvatureClamp(kappa, v, a,
                                 params_.curvature_dec_gain,
                                 params_.curvature_dec_ratio_min,
                                 params_.min_linear_velocity);
        }
        // 曲率限速之后再叠一层加减速包络。两个分支都要走：
        // 曲率减速开着时，它负责把 dec_ratio 造成的速度断崖摊成可执行的减速段；
        // 关掉时 Ruckig 的输出本就满足加减速约束，扫描退化为恒等操作，无副作用。
        if (params_.acceleration_envelope)
        {
            legacyAccelEnvelope(arc, v, a,
                                params_.max_linear_acceleration,
                                params_.max_linear_deceleration);
        }

        for (std::size_t i = 0; i < n; ++i)
        {
            route.points[i].linear_v = v[i];
            route.points[i].linear_a = a[i];
            route.points[i].curvature_omega = v[i] * kappa[i];
        }
    }

    void VelocityPlanner::applyToppraProfile(Route& route, bool kappa_trustworthy) const
    {
        const std::size_t n = route.points.size();
        std::vector<double> arc(n), kappa(n), v(n), a(n);
        for (std::size_t i = 0; i < n; ++i)
        {
            arc[i] = route.points[i].current_s;
            kappa[i] = route.points[i].curvature;
            v[i] = route.points[i].linear_v;
            a[i] = route.points[i].linear_a;
        }

        // kappa' 只在曲率来自上游解析切向时才有意义；否则留空 = 关掉角加速度约束。
        // 曲率若是三点外接圆算的，再差分一次噪声还要放大 1/ds，是在噪声上做优化。
        std::vector<double> kappa_rate;
        if (kappa_trustworthy)
        {
            kappa_rate = computeCurvatureRate(arc, kappa);
        }
        else
        {
            RCLCPP_WARN_ONCE(
                rclcpp::get_logger("velocity_planner"),
                "velocity_profile_mode=toppra but path carries no analytic heading; "
                "angular acceleration constraint disabled (curvature from 3-point circle "
                "is too noisy to differentiate). Fill poses.orientation upstream to enable it.");
        }

        EnvelopeLimits lim;
        lim.w_max = params_.max_angular_velocity;
        lim.a_max = params_.max_linear_acceleration;
        lim.d_max = params_.max_linear_deceleration;
        lim.alpha_max = params_.max_angular_acceleration *
                        std::clamp(params_.angular_accel_margin, 0.0, 1.0);
        lim.min_v = params_.min_linear_velocity;

        toppraEnvelope(arc, kappa, kappa_rate, v, a, lim);

        for (std::size_t i = 0; i < n; ++i)
        {
            route.points[i].linear_v = v[i];
            route.points[i].linear_a = a[i];
            route.points[i].curvature_omega = v[i] * kappa[i];
        }
    }

    void VelocityPlanner::fillPositionVelocityProfile(Route& route, const RuckigSeed& seed,
                                                      bool kappa_trustworthy) const
    {
        const std::size_t n = route.points.size();
        if (n == 0)
        {
            return;
        }
        const double s0 = route.points.front().current_s;
        std::vector<double> dist(n);
        for (std::size_t i = 0; i < n; ++i)
        {
            dist[i] = route.points[i].current_s - s0;
        }

        std::vector<double> v_out, a_out;
        if (!planRuckigVelocityProfile(
                dist, seed, params_.max_linear_velocity, params_.min_linear_velocity,
                params_.max_linear_acceleration, params_.max_linear_deceleration,
                params_.max_linear_jerk, params_.ruckig_control_dt, v_out, a_out))
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("velocity_planner"),
                "Ruckig position planning FAILED (fell back to flat seed velocity): "
                "n=%zu dist_back=%.4f seed_v=%.4f seed_a=%.4f v_max=%.4f v_target=%.4f "
                "a_max=%.4f d_max=%.4f jerk=%.4f dt=%.4f",
                n, dist.back(), seed.v, seed.a,
                params_.max_linear_velocity, params_.min_linear_velocity,
                params_.max_linear_acceleration, params_.max_linear_deceleration,
                params_.max_linear_jerk, params_.ruckig_control_dt);
        }

        for (std::size_t i = 0; i < n; ++i)
        {
            route.points[i].linear_v = v_out[i];
            route.points[i].linear_a = a_out[i];
        }

        // 至此 v_out 是"只看弧长"的 Ruckig 全局剖面，作为逐点速度上限交给包络。
        // 两种包络都只会往下压，不会超过这条线。
        if (params_.velocity_profile_mode == "toppra")
        {
            applyToppraProfile(route, kappa_trustworthy);
        }
        else
        {
            applyLegacyProfile(route);
        }
    }

    void VelocityPlanner::fillHeadingVelocityProfile(HeadingRoute& route, const RuckigSeed& seed) const
    {
        const std::size_t n = route.points.size();
        if (n == 0)
        {
            return;
        }
        // current_a 是有符号累计角度（可能递减）；Ruckig 需要非负单调"距离"，用 sign 把
        // 整段弧度转成无符号幅值空间规划，w_max/w_target 全部按幅值传入，
        // 只有 seed（调用方按旋转方向带符号给出）需要先乘 sign 转成幅值空间。
        const double a0 = route.points.front().current_a;
        const double span = route.points.back().current_a - a0;
        const double sign = (span < 0.0) ? -1.0 : 1.0;

        std::vector<double> dist(n);
        for (std::size_t i = 0; i < n; ++i)
        {
            dist[i] = sign * (route.points[i].current_a - a0);
        }

        // 幅值空间的种子（sign 已把"沿剩余路径前进"统一成正方向）。
        const RuckigSeed mag_seed{sign * seed.v, sign * seed.a};

        std::vector<double> v_out, a_out;
        if (!planRuckigVelocityProfile(
                dist, mag_seed,
                params_.max_angular_velocity, params_.min_angular_velocity,
                params_.max_angular_acceleration, params_.max_angular_deceleration,
                params_.max_angular_jerk, params_.ruckig_control_dt, v_out, a_out))
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("velocity_planner"),
                "Ruckig heading planning FAILED (fell back to flat seed velocity): "
                "n=%zu dist_back=%.4f seed_w(mag)=%.4f w_max=%.4f w_target=%.4f "
                "a_max=%.4f d_max=%.4f jerk=%.4f dt=%.4f",
                n, dist.back(), mag_seed.v,
                params_.max_angular_velocity, params_.min_angular_velocity,
                params_.max_angular_acceleration, params_.max_angular_deceleration,
                params_.max_angular_jerk, params_.ruckig_control_dt);
        }

        for (std::size_t i = 0; i < n; ++i)
        {
            route.points[i].heading_omega = sign * v_out[i];
            route.points[i].heading_alpha = sign * a_out[i];
        }
    }

    HeadingRoute VelocityPlanner::planHeading(double start_theta, double goal_theta) const
    {
        HeadingRoute hr;
        const double delta = angles::normalize_angle(goal_theta - start_theta); // 最短有向角差
        hr.total_a = delta;

        const double a_res = std::max(params_.angle_resolution, kEpsilon);
        const double abs_total = std::fabs(delta);

        // 起末朝向相同：不旋转，两点都=起点朝向，角速度全 0。
        if (abs_total < kEpsilon)
        {
            hr.points.assign(2, HeadingPoint{});
            hr.points.front().theta = start_theta;
            hr.points.back().theta = start_theta;
            return hr;
        }

        // 0.2° 离散：首=起点朝向，末=终点朝向，n = ceil(span/res) + 1。
        const double sign = (delta > 0.0) ? 1.0 : -1.0;
        const std::size_t n = static_cast<std::size_t>(std::ceil(abs_total / a_res)) + 1u;
        hr.points.resize(std::max<std::size_t>(n, 2u));
        for (std::size_t i = 0; i < hr.points.size(); ++i)
        {
            const double r = static_cast<double>(i) / static_cast<double>(hr.points.size() - 1);
            const double a = delta * r; // 有符号累计角度（unwrap 单调）
            hr.points[i].theta = start_theta + a;
            hr.points[i].current_a = a;
        }

        // 整段规划：起点种子角速度固定为 sign*min_angular_velocity（覆盖底盘死区），
        // 终点目标幅值同为 min_angular_velocity（末端速度语义与旧梯形规划一致）。
        // 起步角加速度按转角大小选，语义与 planPosition 的 seed_accel 完全一致：
        // 转角小到"还没停止加速就已经转过头"时，Ruckig 的点到点问题无解，把 α0 取成 0
        // 让它重新可解（见 kShortRotationAngle）。
        const double seed_alpha = (abs_total < kShortRotationAngle)
            ? 0.0
            : params_.max_angular_acceleration / 2.0;

        fillHeadingVelocityProfile(
            hr, RuckigSeed{sign * params_.min_angular_velocity, sign * seed_alpha});
        return hr;
    }

    Route VelocityPlanner::planPosition(const std::vector<TrajectoryPoint>& raw_points,
                                        bool trust_input_heading) const
    {
        Route route;
        route.mode = RouteMode::kDispersedCurve;
        route.points = raw_points;
        const double gap_threshold = 2.0 * params_.trajectory_resolution;
        for (std::size_t i = 1; i < raw_points.size(); ++i)
        {
            const double dx = raw_points[i].x - raw_points[i - 1].x;
            const double dy = raw_points[i].y - raw_points[i - 1].y;
            const double gap = std::hypot(dx, dy);
            if (gap > gap_threshold)
            {
                RCLCPP_WARN(
                    rclcpp::get_logger("velocity_planner"),
                    "Raw path gap too large at segment %zu->%zu: gap=%.4fm > 2*resolution=%.4fm",
                    i - 1, i, gap, gap_threshold);
            }
        }

        computeArcLengthAndHeading(route, trust_input_heading);
        computeCurvature(route, trust_input_heading);

        // 整段规划：起点种子速度固定为 min_linear_velocity（覆盖底盘死区），终点目标同为
        // min_linear_velocity（真正到点停车由 path_follower 的 reach_distance 判定独立
        // 触发，与此处规划值无关）。曲率减速在 fillPositionVelocityProfile 内部对全程生效。
        //
        // 起步加速度按路径长度选：jerk 有限，正的起步加速度没法立刻消掉，把 a0 压回 0
        // 至少要走 v0·(a0/jerk) + a0³/(3·jerk²) 的距离，期间速度还在继续涨。路径比这
        // 还短就意味着"还没停止加速就已经越过终点"，而 min_velocity≈0 禁止倒退、Ruckig
        // 退不回来，这个点到点问题无解，只会吐出一条无意义的曲线（实测表现为参考速度
        // 莫名衰减到 0）。短路径把 a0 取成 0，上述行程退化为 0，问题重新可解——
        // 于是速度输出始终只有 Ruckig 这一个来源，不需要额外的兜底填充分支。
        const double seed_accel = (route.total_s < kShortPathDistance)
            ? 0.0
            : params_.max_linear_acceleration / 2.0;

        fillPositionVelocityProfile(
            route, RuckigSeed{params_.min_linear_velocity, seed_accel}, trust_input_heading);

        return route;
    }
} // namespace path_follower
