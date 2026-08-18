#include "schedule_planner/path_segment_sampler.hpp"

#include <algorithm>
#include <cmath>
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

    std::vector<PathSegmentSampler::Pt> PathSegmentSampler::sampleLine(
        const Pt& a, const Pt& b, double res)
    {
        const double dx = b.first - a.first;
        const double dy = b.second - a.second;
        const double dist = std::hypot(dx, dy);
        std::vector<Pt> pts;
        if (dist < kEps)
        {
            pts.push_back(a);
            pts.push_back(b);
            return pts;
        }
        const double r = std::max(res, kEps);
        const std::size_t n = static_cast<std::size_t>(std::ceil(dist / r));
        const std::size_t m = std::max<std::size_t>(2, n);
        pts.reserve(m + 1);
        for (std::size_t i = 0; i <= m; ++i)
        {
            const double t = static_cast<double>(i) / static_cast<double>(m);
            pts.emplace_back(a.first + t * dx, a.second + t * dy);
        }
        return pts;
    }

    std::vector<PathSegmentSampler::Pt> PathSegmentSampler::sampleBezier(
        const std::vector<Pt>& cp, double res)
    {
        std::vector<Pt> pts;
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
            // 退化（控制点重合）：返回首末
            pts.push_back(cp.front());
            pts.push_back(cp.back());
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
            pts.push_back(bezierEval(cp, t));
        }
        return pts;
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

        std::vector<Pt> pts;
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
        tp.poses.reserve(n);
        for (std::size_t i = 0; i < n; ++i)
        {
            geometry_msgs::msg::Pose p;
            p.position.x = pts[i].first;
            p.position.y = pts[i].second;
            p.position.z = 0.0;
            // poses 朝向暂时无意义，置 0（path_follower 只用 position）。
            tp.poses.push_back(std::move(p));
        }

        // 段级跟踪参数映射。
        // seg.mode（高级属性，rviz 不暴露，默认 0）显式指定时原样透传（MODE_OMNI_TRANSLATION
        // 例外：hold_direction 为 NaN 时无法平移保向，降级为 MODE_DIFF 并告警）；
        // seg.mode==0 时走既有默认逻辑：backward 显式 -> DIFF；hold_direction 显式 -> OMNI_TRANSLATION；
        // 否则 default_mode。
        switch (seg.mode)
        {
        case nav_interfaces::msg::PathSegment::MODE_OMNI_TRANS_ROTATION:
            tp.mode = nav_interfaces::msg::TrackingPath::MODE_OMNI_TRANS_ROTATION;
            break;
        case nav_interfaces::msg::PathSegment::MODE_OMNI_TRANSLATION:
            if (std::isnan(seg.hold_direction))
            {
                RCLCPP_WARN(
                    rclcpp::get_logger("path_segment_sampler"),
                    "PathSegment.mode=MODE_OMNI_TRANSLATION but hold_direction is NaN, "
                    "downgrading to MODE_DIFF");
                tp.mode = nav_interfaces::msg::TrackingPath::MODE_DIFF;
            }
            else
            {
                tp.mode = nav_interfaces::msg::TrackingPath::MODE_OMNI_TRANSLATION;
            }
            break;
        case nav_interfaces::msg::PathSegment::MODE_DIFF:
            tp.mode = nav_interfaces::msg::TrackingPath::MODE_DIFF;
            break;
        default: // 0：未显式指定，走既有默认逻辑
            if (seg.backward)
            {
                tp.mode = nav_interfaces::msg::TrackingPath::MODE_DIFF;
            }
            else if (!std::isnan(seg.hold_direction))
            {
                tp.mode = nav_interfaces::msg::TrackingPath::MODE_OMNI_TRANSLATION;
            }
            else
            {
                tp.mode = default_mode;
            }
            break;
        }
        tp.backward = seg.backward
            ? nav_interfaces::msg::TrackingPath::REVERSE
            : nav_interfaces::msg::TrackingPath::FORWARD;
        tp.hold_direction = seg.hold_direction;
        tp.max_speed = seg.max_speed;
        tp.padding = seg.padding;
        tp.target_yaw = seg.target_yaw; // float64 -> float32，NaN 保留
        return tp;
    }
} // namespace schedule_planner
