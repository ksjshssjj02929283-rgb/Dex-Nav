#include "schedule_planner/segment_merger.hpp"

#include <algorithm>
#include <cmath>

#include "schedule_planner/path_segment_sampler.hpp"

namespace schedule_planner
{
    namespace
    {
        constexpr double kEps = 1e-9;
        /// padding 等 float32 字段的相等判据。停障膨胀是厘米级量，1mm 足够区分。
        constexpr double kPaddingTol = 1e-3;

        /// NaN 感知的相等比较：两个都是 NaN 视为相等（都表示「用默认值」）。
        bool sameOptional(double a, double b, double tol)
        {
            const bool na = std::isnan(a);
            const bool nb = std::isnan(b);
            if (na || nb)
            {
                return na && nb;
            }
            return std::fabs(a - b) <= tol;
        }
    } // namespace

    double segmentLength(const nav_interfaces::msg::PathSegment& seg)
    {
        if (seg.control_points.size() < 2)
        {
            return 0.0;
        }
        const auto& a = seg.control_points.front();
        const auto& b = seg.control_points.back();
        return std::hypot(b.x - a.x, b.y - a.y);
    }

    double junctionDirectionCos(
        const nav_interfaces::msg::PathSegment& prev,
        const nav_interfaces::msg::PathSegment& next)
    {
        if (prev.control_points.size() < 2 || next.control_points.size() < 2)
        {
            return 1.0;
        }
        const auto& p0 = prev.control_points.front();
        const auto& p1 = prev.control_points.back();
        const auto& n0 = next.control_points.front();
        const auto& n1 = next.control_points.back();
        const double px = p1.x - p0.x, py = p1.y - p0.y;
        const double nx = n1.x - n0.x, ny = n1.y - n0.y;
        const double hp = std::hypot(px, py);
        const double hn = std::hypot(nx, ny);
        if (hp < kEps || hn < kEps)
        {
            return 1.0; // 方向退化，不在这里拦（长度门槛会先把它拦掉）
        }
        // 单位方向向量点积；数值上夹到 [-1, 1] 免得后续 acos 越界。
        return std::clamp((px * nx + py * ny) / (hp * hn), -1.0, 1.0);
    }

    bool sameTrackingAttributes(
        const nav_interfaces::msg::PathSegment& a,
        const nav_interfaces::msg::PathSegment& b,
        uint8_t default_mode)
    {
        if (a.backward != b.backward)
        {
            return false;
        }
        if (!sameOptional(a.padding, b.padding, kPaddingTol))
        {
            return false;
        }
        return PathSegmentSampler::resolveMode(a, default_mode) ==
               PathSegmentSampler::resolveMode(b, default_mode);
    }

    std::vector<MergeGroup> planMergeGroups(
        const std::vector<nav_interfaces::msg::PathSegment>& segs,
        std::size_t first_mergeable,
        uint8_t default_mode,
        const MergeParams& params)
    {
        std::vector<MergeGroup> groups;
        const std::size_t n = segs.size();
        if (n == 0)
        {
            return groups;
        }

        // 关掉合并、或阶数不是支持的 4 次时，退回一段一组（与改造前行为完全一致）。
        const bool merging = params.enable && params.bezier_degree == 4;
        const std::size_t start = merging ? std::min(first_mergeable, n) : n;

        // 归位段（以及关闭合并时的全部段）单独成组。
        for (std::size_t i = 0; i < start; ++i)
        {
            groups.push_back(MergeGroup{i, i + 1});
        }

        const double min_end = params.minEndLength();
        const double min_interior = params.minInteriorLength();
        // 转角判据取余弦：cos 在 [0, pi] 上单调减，故
        //     dot(unit_prev, unit_next) > cos(max_turn)  <=>  转角 < max_turn
        const double min_cos = std::cos(params.max_turn_angle_deg * M_PI / 180.0);

        auto lineSeg = [](const nav_interfaces::msg::PathSegment& s) {
            return s.type == nav_interfaces::msg::PathSegment::LINE &&
                   s.control_points.size() >= 2;
        };

        std::size_t i = start;
        while (i < n)
        {
            std::size_t end = i + 1;
            // 贝塞尔段不参与合并：锚点倒角需要两条直边交于一个顶点，曲线段的端点
            // 已有自己的切向，没有可退让的顶点。
            if (lineSeg(segs[i]))
            {
                while (end < n)
                {
                    const std::size_t last = end - 1; // 当前组的末段，即接缝的前一段
                    if (!lineSeg(segs[end]))
                    {
                        break;
                    }
                    if (!sameTrackingAttributes(segs[last], segs[end], default_mode))
                    {
                        break;
                    }
                    if (junctionDirectionCos(segs[last], segs[end]) <= min_cos)
                    {
                        break;
                    }
                    // 接缝两侧的长度门槛。segs[last] 加入新接缝后角色会变：
                    // 它原本是组的末段（只服务一个拐角），扩组后若不再是首段就成了
                    // 中间段（服务两个拐角），门槛从 min_end 抬到 min_interior。
                    const double len_last = segmentLength(segs[last]);
                    const double need_last = (last == i) ? min_end : min_interior;
                    if (len_last < need_last)
                    {
                        break;
                    }
                    // 新末段至少要够一个拐角的退让。长度落在 [min_end, min_interior)
                    // 的段能进来当末段，但下一轮它的中间段门槛不过，组就在这里自动封口。
                    if (segmentLength(segs[end]) < min_end)
                    {
                        break;
                    }
                    ++end;
                }
            }
            groups.push_back(MergeGroup{i, end});
            i = end;
        }
        return groups;
    }
} // namespace schedule_planner
