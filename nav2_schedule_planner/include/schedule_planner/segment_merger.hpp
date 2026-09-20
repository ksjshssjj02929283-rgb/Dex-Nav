#ifndef SCHEDULE_PLANNER__SEGMENT_MERGER_HPP_
#define SCHEDULE_PLANNER__SEGMENT_MERGER_HPP_

#include <cstddef>
#include <vector>

#include <nav_interfaces/msg/path_segment.hpp>

namespace schedule_planner
{
    /**
     * @file
     * @brief 把属性一致、长度够的相邻 PathSegment 分成若干「组」，每组合并成一条带拐角
     *        平滑的轨迹一次性下发，省掉组内每个途经点的「停车 + 原地旋转」。
     *
     * 分组只做判定、不碰几何；几何在 PathSegmentSampler::discretizeGroup 里做。
     * 纯函数，不依赖 rclcpp，便于单测。
     */

    /// 分组参数。长度阈值不单独配置，由 anchor_spacing 推导——改倒角参数时自动跟着走。
    struct MergeParams
    {
        bool enable{true};
        /// 拐角贝塞尔阶数（当前只支持 4，与 path_follower 侧的锚点方案一致）。
        int bezier_degree{4};
        /// 锚点沿边间距 m；退让距离 d = (degree - 2) * anchor_spacing。
        double anchor_spacing{0.25};
        /// 端段长度余量 m：端段只需 d，留一点余量避免刚好卡死。
        double end_margin{0.1};
        /// 相邻两段允许的最大转角（deg），判定用方向向量的余弦：
        ///     dot(unit(前段行进方向), unit(后段行进方向)) > cos(max_turn_angle_deg)
        /// 直行时余弦为 1，转 90 度为 0，掉头为 -1；转得越急越小。
        /// 默认 120 度即 cos > -0.5，直角拐角（cos = 0）在内。
        /// 取余弦比取角度好：省掉 atan2 与跨 ±pi 的归一化，也没有分支。
        ///
        /// 注意必须用**行进方向**向量，不能用「两条直线」作为无向直线的夹角：
        /// 150 度急掉头的两条直线夹角只有 30 度，按无向算会被误放行。
        double max_turn_angle_deg{120.0};

        /// 退让距离 d：拐角前后各在两条边上退让这么多。
        double retreat() const
        {
            return static_cast<double>(bezier_degree - 2) * anchor_spacing;
        }
        /// 端段（组内首段或末段）最小长度：只服务一个拐角，需要 d + 余量。
        double minEndLength() const { return retreat() + end_margin; }
        /// 中间段最小长度：被前后两个拐角共享，需要 2d。
        double minInteriorLength() const { return 2.0 * retreat(); }
    };

    /// 一组连续段 [begin, end)。end - begin == 1 时退化为原来的单段下发。
    struct MergeGroup
    {
        std::size_t begin{0};
        std::size_t end{0};
        std::size_t size() const { return end - begin; }
    };

    /**
     * @brief 判断两段的跟踪属性是否一致（可否合并成一条轨迹）。
     *
     * 只比 backward / padding / **解析后的** mode 三项：
     *   - TrackingPath 每个字段只有一份，这三项决定 path_follower 的分派与停障距离，
     *     不一致就必须拆开；
     *   - mode 必须比「解析后」的值：PathSegment.mode == 0 时实际模式还取决于 backward
     *     与 hold_direction 是否为 NaN（见 PathSegmentSampler::resolveMode），
     *     直接比原始字段会把两段实际行为不同的段判成一致；
     *   - max_speed / xy_tolerance / th_tolerance / hold_direction 不参与判定，
     *     组内取值规则见 discretizeGroup；
     *   - target_yaw 不用比：非末段本来就被置 NaN。
     */
    bool sameTrackingAttributes(
        const nav_interfaces::msg::PathSegment& a,
        const nav_interfaces::msg::PathSegment& b,
        uint8_t default_mode);

    /**
     * @brief 规划分组。返回的区间**覆盖全部段且不重叠**，顺序与输入一致。
     *
     * 贪心：从 first_mergeable 开始，能吃就往后吃。每段在组内的「角色」决定它的长度门槛——
     *   组内首段 / 末段：只服务一个拐角，len >= minEndLength()
     *   组内中间段：      被两个拐角共享，len >= minInteriorLength()
     * 于是「长度在 [minEnd, minInterior) 的段只能收尾」这条截断规则不用特判，
     * 它就是角色门槛的自然结果：想把它变成中间段时中间段门槛不过，组自动封口。
     *
     * 另外三条硬性条件：两段属性一致、都是 LINE 段（贝塞尔段的端点没有可退让的顶点，
     * 先拆开单独下发）、接缝处两段行进方向的夹角余弦 > cos(max_turn_angle_deg)。
     *
     * @param segs            解析出来的全部段
     * @param first_mergeable 从第几段开始允许合并。map_asset 在车离起点太远时会在最前面
     *                        插一条归位边（segments.size() == nav_point_names.size()），
     *                        那一段必须单独成组先跑完，此时传 1；否则传 0。
     * @param default_mode    PathSegment.mode == 0 时的默认跟踪方式，用于解析 mode
     */
    std::vector<MergeGroup> planMergeGroups(
        const std::vector<nav_interfaces::msg::PathSegment>& segs,
        std::size_t first_mergeable,
        uint8_t default_mode,
        const MergeParams& params);

    /// 段长度：control_points 首末点的直线距离。LINE 段精确；分组只对 LINE 生效，
    /// 所以不需要先离散求弧长。
    double segmentLength(const nav_interfaces::msg::PathSegment& seg);

    /// 两段接缝处**行进方向**的夹角余弦：dot(unit(prev), unit(next))。
    /// 直行 = 1，转 60 度 = 0.5，转 90 度 = 0，转 120 度 = -0.5，掉头 = -1。
    /// 任一段方向退化（首末点重合）时返回 1（不因此拦截，长度门槛会先把它拦掉）。
    double junctionDirectionCos(
        const nav_interfaces::msg::PathSegment& prev,
        const nav_interfaces::msg::PathSegment& next);
} // namespace schedule_planner

#endif // SCHEDULE_PLANNER__SEGMENT_MERGER_HPP_
