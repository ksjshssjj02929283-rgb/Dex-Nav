#include "schedule_planner/path_segment_sampler.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace schedule_planner
{
    namespace
    {
        constexpr double kEps = 1e-9;
    } // namespace

    long long PathSegmentSampler::binomial(int n, int k)
    {
        if (k < 0 || k > n)
        {
            return 0;
        }
        if (k == 0 || k == n)
        {
            return 1;
        }
        // 取较小的一侧减少乘法
        if (k > n - k)
        {
            k = n - k;
        }
        long long v = 1;
        for (int i = 0; i < k; ++i)
        {
            v = v * (n - i) / (i + 1);
        }
        return v;
    }

    PathSegmentSampler::Pt PathSegmentSampler::bezierEval(const std::vector<Pt>& cp, double t)
    {
        const int n = static_cast<int>(cp.size()) - 1;
        const double u = 1.0 - t;
        double x = 0.0;
        double y = 0.0;
        for (int i = 0; i <= n; ++i)
        {
            const double b = static_cast<double>(binomial(n, i)) *
                             std::pow(u, n - i) * std::pow(t, i);
            x += b * cp[i].first;
            y += b * cp[i].second;
        }
        return {x, y};
    }

    PathSegmentSampler::Pt PathSegmentSampler::bezierDeriv(const std::vector<Pt>& cp, double t)
    {
        // B'(t) 是一条 n-1 阶贝塞尔，控制点为 n * (P_{i+1} - P_i)（hodograph）。
        const int n = static_cast<int>(cp.size()) - 1;
        if (n < 1)
        {
            return {0.0, 0.0};
        }
        const double u = 1.0 - t;
        const int m = n - 1;
        double dx = 0.0;
        double dy = 0.0;
        for (int i = 0; i <= m; ++i)
        {
            const double b = static_cast<double>(binomial(m, i)) *
                             std::pow(u, m - i) * std::pow(t, i);
            dx += b * static_cast<double>(n) * (cp[i + 1].first - cp[i].first);
            dy += b * static_cast<double>(n) * (cp[i + 1].second - cp[i].second);
        }
        return {dx, dy};
    }

    std::vector<PathSegmentSampler::Sample> PathSegmentSampler::sampleLine(
        const Pt& a, const Pt& b, double res)
    {
        const double dx = b.first - a.first;
        const double dy = b.second - a.second;
        const double dist = std::hypot(dx, dy);
        std::vector<Sample> pts;
        if (dist < kEps)
        {
            // 退化段（首末重合）：方向无定义，留 0，由 discretize 的兜底逻辑接手。
            pts.push_back({a.first, a.second, 0.0});
            pts.push_back({b.first, b.second, 0.0});
            return pts;
        }
        const double yaw = std::atan2(dy, dx); // 直线段切向全程常数
        const double r = std::max(res, kEps);
        const std::size_t n = static_cast<std::size_t>(std::ceil(dist / r));
        const std::size_t m = std::max<std::size_t>(2, n);
        pts.reserve(m + 1);
        for (std::size_t i = 0; i <= m; ++i)
        {
            const double t = static_cast<double>(i) / static_cast<double>(m);
            pts.push_back({a.first + t * dx, a.second + t * dy, yaw});
        }
        return pts;
    }

    std::vector<PathSegmentSampler::Sample> PathSegmentSampler::sampleBezier(
        const std::vector<Pt>& cp, double res)
    {
        std::vector<Sample> pts;
        if (cp.size() < 2)
        {
            return pts;
        }
        // 1) 细描曲线，求累积弧长
        const int fine = std::max(200, static_cast<int>(cp.size()) * 50);
        std::vector<Pt> fine_pts(static_cast<std::size_t>(fine) + 1);
        std::vector<double> cum(static_cast<std::size_t>(fine) + 1, 0.0);
        fine_pts[0] = bezierEval(cp, 0.0);
        for (int s = 1; s <= fine; ++s)
        {
            const double t = static_cast<double>(s) / static_cast<double>(fine);
            fine_pts[static_cast<std::size_t>(s)] = bezierEval(cp, t);
            const auto& prev = fine_pts[static_cast<std::size_t>(s - 1)];
            const auto& cur = fine_pts[static_cast<std::size_t>(s)];
            cum[static_cast<std::size_t>(s)] =
                cum[static_cast<std::size_t>(s - 1)] + std::hypot(cur.first - prev.first, cur.second - prev.second);
        }
        const double total = cum.back();
        if (total < kEps)
        {
            // 退化（控制点重合）：返回首末，方向留 0 交给兜底
            pts.push_back({cp.front().first, cp.front().second, 0.0});
            pts.push_back({cp.back().first, cp.back().second, 0.0});
            return pts;
        }
        // 2) 按弧长间距重采样
        const double r = std::max(res, kEps);
        const std::size_t n = static_cast<std::size_t>(std::ceil(total / r));
        const std::size_t m = std::max<std::size_t>(2, n);
        pts.reserve(m + 1);
        std::size_t j = 0;
        const std::size_t fine_n = static_cast<std::size_t>(fine);
        for (std::size_t k = 0; k <= m; ++k)
        {
            // k==m 时 (total*m)/m 浮点上可能略大于 total，夹一下避免越过末点。
            const double target_s =
                std::min(total, total * static_cast<double>(k) / static_cast<double>(m));
            // 守卫用 j+1 < fine_n，保证下面的 cum[j + 1] 恒在界内。
            while (j + 1 < fine_n && cum[j + 1] < target_s)
            {
                ++j;
            }
            const double seg_len = cum[j + 1] - cum[j];
            const double ratio = (seg_len > kEps) ? (target_s - cum[j]) / seg_len : 0.0;
            const double t = (static_cast<double>(j) + ratio) / static_cast<double>(fine);
            const Pt pos = bezierEval(cp, t);
            // 解析切向。|B'| 退化（重控制点造成的尖点）时置 0，由 discretize 兜底成弦向。
            const Pt d1 = bezierDeriv(cp, t);
            const double speed = std::hypot(d1.first, d1.second);
            const double yaw = (speed > kEps) ? std::atan2(d1.second, d1.first) : 0.0;
            pts.push_back({pos.first, pos.second, yaw});
        }
        return pts;
    }

    uint8_t PathSegmentSampler::resolveMode(
        const nav_interfaces::msg::PathSegment& seg, uint8_t default_mode)
    {
        // seg.mode（高级属性，rviz 不暴露，默认 0）显式指定时原样透传（MODE_OMNI_TRANSLATION
        // 例外：hold_direction 为 NaN 时无法平移保向，降级为 MODE_DIFF 并告警）；
        // seg.mode==0 时走既有默认逻辑：backward 显式 -> DIFF；hold_direction 显式 -> OMNI_TRANSLATION；
        // 否则 default_mode。
        switch (seg.mode)
        {
        case nav_interfaces::msg::PathSegment::MODE_OMNI_TRANS_ROTATION:
            return nav_interfaces::msg::TrackingPath::MODE_OMNI_TRANS_ROTATION;
        case nav_interfaces::msg::PathSegment::MODE_OMNI_TRANSLATION:
            if (std::isnan(seg.hold_direction))
            {
                RCLCPP_WARN(
                    rclcpp::get_logger("path_segment_sampler"),
                    "PathSegment.mode=MODE_OMNI_TRANSLATION but hold_direction is NaN, "
                    "downgrading to MODE_DIFF");
                return nav_interfaces::msg::TrackingPath::MODE_DIFF;
            }
            return nav_interfaces::msg::TrackingPath::MODE_OMNI_TRANSLATION;
        case nav_interfaces::msg::PathSegment::MODE_DIFF:
            return nav_interfaces::msg::TrackingPath::MODE_DIFF;
        default: // 0：未显式指定，走既有默认逻辑
            if (seg.backward)
            {
                return nav_interfaces::msg::TrackingPath::MODE_DIFF;
            }
            if (!std::isnan(seg.hold_direction))
            {
                return nav_interfaces::msg::TrackingPath::MODE_OMNI_TRANSLATION;
            }
            return default_mode;
        }
    }

    nav_interfaces::msg::TrackingPath PathSegmentSampler::discretize(
        const nav_interfaces::msg::PathSegment& seg,
        double resolution, const std::string& frame_id,
        uint8_t default_mode) const
    {
        nav_interfaces::msg::TrackingPath tp;
        tp.header.frame_id = frame_id;
        // header.stamp 由节点发布前用 now() 覆盖。

        if (seg.control_points.size() < 2)
        {
            return tp; // 非法段
        }

        std::vector<Pt> cp;
        cp.reserve(seg.control_points.size());
        for (const auto& p : seg.control_points)
        {
            cp.emplace_back(p.x, p.y);
        }

        std::vector<Sample> pts;
        if (seg.type == nav_interfaces::msg::PathSegment::BEZIER)
        {
            pts = sampleBezier(cp, resolution);
        }
        else // LINE 或未知一律按直线处理
        {
            pts = sampleLine(cp.front(), cp.back(), resolution);
        }

        if (pts.size() < 2)
        {
            return tp;
        }

        const std::size_t n = pts.size();

        // 兜底：采样器在退化处（重合控制点造成的尖点）把 yaw 留成 0，这里用弦向补上。
        // 判据是"整条采样的 yaw 全为 0"或"逐点为 0"，两种都只在退化时出现，正常段不会命中。
        for (std::size_t i = 0; i < n; ++i)
        {
            if (pts[i].yaw != 0.0)
            {
                continue;
            }
            const std::size_t a = (i + 1 < n) ? i : i - 1;
            const std::size_t b = (i + 1 < n) ? i + 1 : i;
            const double dx = pts[b].x - pts[a].x;
            const double dy = pts[b].y - pts[a].y;
            if (std::hypot(dx, dy) > kEps)
            {
                pts[i].yaw = std::atan2(dy, dx);
            }
        }

        tp.poses.reserve(n);
        for (std::size_t i = 0; i < n; ++i)
        {
            geometry_msgs::msg::Pose p;
            p.position.x = pts[i].x;
            p.position.y = pts[i].y;
            p.position.z = 0.0;
            // 朝向 = 该点解析切向（行进方向）。下游据此直接求曲率，
            // 不再从 x,y 做二次差分。倒车段的 +pi 由 path_follower 在消费端处理。
            const double half = 0.5 * pts[i].yaw;
            p.orientation.x = 0.0;
            p.orientation.y = 0.0;
            p.orientation.z = std::sin(half);
            p.orientation.w = std::cos(half);
            tp.poses.push_back(std::move(p));
        }

        // 段级跟踪参数映射。mode 的解析（含 mode==0 的默认逻辑与
        // MODE_OMNI_TRANSLATION 缺 hold_direction 的降级）统一走 resolveMode，
        // 合并判定（segment_merger）要用同一份结果。
        tp.mode = resolveMode(seg, default_mode);
        tp.backward = seg.backward
            ? nav_interfaces::msg::TrackingPath::REVERSE
            : nav_interfaces::msg::TrackingPath::FORWARD;
        tp.hold_direction = seg.hold_direction;
        tp.max_speed = seg.max_speed;
        tp.padding = seg.padding;
        tp.target_yaw = seg.target_yaw; // float64 -> float32，NaN 保留
        return tp;
    }

    // ---------------------------------------------------------------------------
    // 组合并：折线 -> 直线 + 四次贝塞尔倒角 + 直线
    // ---------------------------------------------------------------------------
    std::vector<PathSegmentSampler::Pt> PathSegmentSampler::cornerControlPoints(
        const Pt& p_prev, const Pt& p_corner, const Pt& p_next,
        double off_in, double off_out)
    {
        // 四次锚点方案：中间锚点取两条边的中点 + 拐角点本身，首末各三点共线，
        // 于是曲线在 A、B 处与原直线段一阶且二阶相切（kappa(0)=kappa(1)=0），
        // 拼接后曲率连续、无冲击。与 scripts/corner_bezier_path_generator.py 一致。
        double ix = p_prev.first - p_corner.first;
        double iy = p_prev.second - p_corner.second;
        double ox = p_next.first - p_corner.first;
        double oy = p_next.second - p_corner.second;
        const double ni = std::max(std::hypot(ix, iy), kEps);
        const double no = std::max(std::hypot(ox, oy), kEps);
        ix /= ni; iy /= ni; ox /= no; oy /= no;

        const Pt a{p_corner.first + off_in * ix, p_corner.second + off_in * iy};
        const Pt b{p_corner.first + off_out * ox, p_corner.second + off_out * oy};
        return {
            a,
            {0.5 * (a.first + p_corner.first), 0.5 * (a.second + p_corner.second)},
            p_corner,
            {0.5 * (p_corner.first + b.first), 0.5 * (p_corner.second + b.second)},
            b,
        };
    }

    std::vector<PathSegmentSampler::Sample> PathSegmentSampler::smoothedPolyline(
        const std::vector<Pt>& wps, double anchor_spacing, int degree, double dense_step)
    {
        std::vector<Sample> dense;
        const std::size_t n = wps.size();
        if (n < 2)
        {
            return dense;
        }
        const double nominal = static_cast<double>(degree - 2) * anchor_spacing;
        const double step = std::max(dense_step, kEps);

        std::vector<double> seg_len(n - 1);
        for (std::size_t i = 0; i + 1 < n; ++i)
        {
            seg_len[i] = std::hypot(wps[i + 1].first - wps[i].first,
                                    wps[i + 1].second - wps[i].second);
        }

        // 每个内部拐角在前/后两条边上的实际退让距离：首尾边可整段使用，
        // 中间边被两个拐角共享各取一半，保证相邻两段贝塞尔不重叠。
        std::vector<double> offsets;
        offsets.reserve(n > 2 ? n - 2 : 0);
        for (std::size_t i = 1; i + 1 < n; ++i)
        {
            const double in_budget = seg_len[i - 1] * ((i == 1) ? 1.0 : 0.5);
            const double out_budget = seg_len[i] * ((i + 2 == n) ? 1.0 : 0.5);
            offsets.push_back(std::min({nominal, in_budget, out_budget}));
        }

        auto pushLine = [&](const Pt& p0, const Pt& p1) {
            const double len = std::hypot(p1.first - p0.first, p1.second - p0.second);
            if (len < kEps)
            {
                return;
            }
            const double yaw = std::atan2(p1.second - p0.second, p1.first - p0.first);
            const std::size_t m =
                std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(len / step)));
            for (std::size_t k = 0; k <= m; ++k)
            {
                const double t = static_cast<double>(k) / static_cast<double>(m);
                dense.push_back({p0.first + t * (p1.first - p0.first),
                                 p0.second + t * (p1.second - p0.second), yaw});
            }
        };

        Pt cursor = wps.front();
        for (std::size_t idx = 0, i = 1; i + 1 < n; ++i, ++idx)
        {
            const double off = offsets[idx];
            const auto cp = cornerControlPoints(wps[i - 1], wps[i], wps[i + 1], off, off);
            pushLine(cursor, cp.front());

            // 贝塞尔按 u 细分；后续统一按弧长重采样，这里只要足够密。
            double chord = 0.0;
            for (std::size_t k = 0; k + 1 < cp.size(); ++k)
            {
                chord += std::hypot(cp[k + 1].first - cp[k].first,
                                    cp[k + 1].second - cp[k].second);
            }
            const std::size_t m = std::max<std::size_t>(
                64, static_cast<std::size_t>(std::ceil(chord / step)) + 1);
            for (std::size_t k = 0; k <= m; ++k)
            {
                const double t = static_cast<double>(k) / static_cast<double>(m);
                const Pt pos = bezierEval(cp, t);
                const Pt d1 = bezierDeriv(cp, t);
                const double speed = std::hypot(d1.first, d1.second);
                // |B'| 退化（重合控制点造成的尖点）时沿用上一点的朝向，由 resample 前的
                // unwrap 兜住；正常拐角不会命中。
                const double yaw = (speed > kEps)
                                       ? std::atan2(d1.second, d1.first)
                                       : (dense.empty() ? 0.0 : dense.back().yaw);
                dense.push_back({pos.first, pos.second, yaw});
            }
            cursor = cp.back();
        }
        pushLine(cursor, wps.back());
        return dense;
    }

    std::vector<PathSegmentSampler::Sample> PathSegmentSampler::resampleByArcLength(
        const std::vector<Sample>& dense, double resolution)
    {
        std::vector<Sample> out;
        if (dense.size() < 2)
        {
            return dense;
        }
        // 去掉拼接处的重复点，并把 yaw unwrap 成连续量：跨 ±pi 不 unwrap 会在插值时
        // 反向绕一圈，下游按它求曲率会出巨大假尖峰。
        std::vector<Sample> pts;
        std::vector<double> cum;
        pts.reserve(dense.size());
        cum.reserve(dense.size());
        pts.push_back(dense.front());
        cum.push_back(0.0);
        for (std::size_t i = 1; i < dense.size(); ++i)
        {
            const double d = std::hypot(dense[i].x - pts.back().x, dense[i].y - pts.back().y);
            if (d < 1e-9)
            {
                continue;
            }
            Sample s = dense[i];
            double dy = s.yaw - pts.back().yaw;
            while (dy > M_PI) { dy -= 2.0 * M_PI; }
            while (dy < -M_PI) { dy += 2.0 * M_PI; }
            s.yaw = pts.back().yaw + dy;
            cum.push_back(cum.back() + d);
            pts.push_back(s);
        }
        const double total = cum.back();
        if (total < kEps)
        {
            return {pts.front()};
        }
        const double r = std::max(resolution, kEps);
        const std::size_t m =
            std::max<std::size_t>(2, static_cast<std::size_t>(std::ceil(total / r)));
        out.reserve(m + 1);
        std::size_t j = 0;
        for (std::size_t k = 0; k <= m; ++k)
        {
            const double target = std::min(total, total * static_cast<double>(k) /
                                                      static_cast<double>(m));
            while (j + 2 < cum.size() && cum[j + 1] < target)
            {
                ++j;
            }
            const double span = cum[j + 1] - cum[j];
            const double t = (span > kEps) ? (target - cum[j]) / span : 0.0;
            out.push_back({pts[j].x + t * (pts[j + 1].x - pts[j].x),
                           pts[j].y + t * (pts[j + 1].y - pts[j].y),
                           pts[j].yaw + t * (pts[j + 1].yaw - pts[j].yaw)});
        }
        return out;
    }

    nav_interfaces::msg::TrackingPath PathSegmentSampler::discretizeGroup(
        const std::vector<nav_interfaces::msg::PathSegment>& segs,
        std::size_t begin, std::size_t end,
        double resolution, double anchor_spacing, int degree,
        const std::string& frame_id, uint8_t default_mode) const
    {
        nav_interfaces::msg::TrackingPath tp;
        tp.header.frame_id = frame_id;
        if (end <= begin || end > segs.size())
        {
            return tp;
        }
        if (end - begin == 1)
        {
            return discretize(segs[begin], resolution, frame_id, default_mode);
        }

        // 折线顶点：首段起点 + 每段终点。相邻段的端点本应重合（都是图上的边），
        // 不重合说明上游解析有问题，告警但仍按后一段的起点续上，避免整组失败。
        std::vector<Pt> wps;
        wps.reserve(end - begin + 1);
        for (std::size_t i = begin; i < end; ++i)
        {
            if (segs[i].type != nav_interfaces::msg::PathSegment::LINE ||
                segs[i].control_points.size() < 2)
            {
                RCLCPP_ERROR(rclcpp::get_logger("path_segment_sampler"),
                             "discretizeGroup got a non-LINE segment at %zu, refusing to merge.",
                             i);
                return tp;
            }
            const auto& f = segs[i].control_points.front();
            const auto& b = segs[i].control_points.back();
            if (wps.empty())
            {
                wps.push_back({f.x, f.y});
            }
            else if (std::hypot(f.x - wps.back().first, f.y - wps.back().second) > 1e-3)
            {
                RCLCPP_WARN(rclcpp::get_logger("path_segment_sampler"),
                            "Segment %zu start does not match previous end (gap %.4fm), "
                            "bridging with a straight line.",
                            i, std::hypot(f.x - wps.back().first, f.y - wps.back().second));
                wps.push_back({f.x, f.y});
            }
            wps.push_back({b.x, b.y});
        }
        if (wps.size() < 2)
        {
            return tp;
        }

        // 细描步长要明显细于重采样间距，否则折线化的贝塞尔会给切向留下台阶。
        const auto dense = smoothedPolyline(wps, anchor_spacing, degree,
                                            std::min(resolution / 5.0, 1e-3));
        const auto pts = resampleByArcLength(dense, resolution);
        if (pts.size() < 2)
        {
            return tp;
        }

        tp.poses.reserve(pts.size());
        for (const auto& s : pts)
        {
            geometry_msgs::msg::Pose p;
            p.position.x = s.x;
            p.position.y = s.y;
            p.position.z = 0.0;
            const double half = 0.5 * s.yaw;
            p.orientation.x = 0.0;
            p.orientation.y = 0.0;
            p.orientation.z = std::sin(half);
            p.orientation.w = std::cos(half);
            tp.poses.push_back(std::move(p));
        }

        // 段级参数：backward/padding/mode 由 sameTrackingAttributes 保证组内一致，取首段即可。
        const auto& head = segs[begin];
        const auto& tail = segs[end - 1];
        tp.mode = resolveMode(head, default_mode);
        tp.backward = head.backward ? nav_interfaces::msg::TrackingPath::REVERSE
                                    : nav_interfaces::msg::TrackingPath::FORWARD;
        tp.padding = head.padding;
        tp.hold_direction = head.hold_direction;
        // target_yaw 取末段（调用方仍按「非末组置 NaN」处理）。
        // xy/th_tolerance 是 PathSegment 独有字段，TrackingPath 里没有，
        // 现有单段 discretize 也不转发，这里保持一致。
        tp.target_yaw = tail.target_yaw;

        // max_speed 不参与合并判定，组内可能不一致：取最小值（NaN=不限速），
        // 保证没有任何一段跑得比它自己的限速快。不一致时告警，免得现场以为限速丢了。
        double v_min = std::numeric_limits<double>::quiet_NaN();
        bool speed_differs = false;
        for (std::size_t i = begin; i < end; ++i)
        {
            const double v = segs[i].max_speed;
            if (std::isnan(v))
            {
                continue;
            }
            if (std::isnan(v_min))
            {
                v_min = v;
            }
            else if (std::fabs(v - v_min) > 1e-6)
            {
                speed_differs = true;
                v_min = std::min(v_min, v);
            }
        }
        if (speed_differs)
        {
            RCLCPP_WARN(rclcpp::get_logger("path_segment_sampler"),
                        "Merged group [%zu,%zu) has differing max_speed, using min %.3f m/s.",
                        begin, end, v_min);
        }
        tp.max_speed = static_cast<float>(v_min);

        if (std::isnan(head.hold_direction) != std::isnan(tail.hold_direction))
        {
            RCLCPP_WARN(rclcpp::get_logger("path_segment_sampler"),
                        "Merged group [%zu,%zu) has inconsistent hold_direction, using the first.",
                        begin, end);
        }
        return tp;
    }

} // namespace schedule_planner
