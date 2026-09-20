#ifndef SCHEDULE_PLANNER__PATH_SEGMENT_SAMPLER_HPP_
#define SCHEDULE_PLANNER__PATH_SEGMENT_SAMPLER_HPP_

#include <string>
#include <utility>
#include <vector>

#include <nav_interfaces/msg/tracking_path.hpp>
#include <nav_interfaces/msg/path_segment.hpp>

namespace schedule_planner
{
    /**
     * @brief 把一段已解析的 PathSegment 离散成固定间距的 TrackingPath（含段级跟踪参数）。
     *
     * - LINE  : 两端点间等距采样。
     * - BEZIER: 一般 Bernstein 曲线，先细描再按累积弧长重采样到指定间距
     *           （与 scripts/bezier_path_generator.py 的 sample_bezier_arc_length 等价，
     *            并兼容 route_graph_manager 的一般阶 Bernstein 约定）。
     *
     * poses 朝向 = 该点的**解析切向角**（行进方向），写入 orientation 的 yaw 分量：
     *   LINE   : 首末点连线方向，全段常数。
     *   BEZIER : theta(t) = atan2(B'_y(t), B'_x(t))，B' 由控制点一次降阶（hodograph）得到。
     * 下游 path_follower 据此直接算曲率 kappa = dtheta/ds，避免从 5mm~1cm 的 x,y 二次差分
     * 反推几何（那样对位置噪声的放大系数是 4/ds^2，1cm 下约 4e4，曲率会被噪声淹没）。
     * 倒车段不在这里做 +pi：control_points 按行进方向序给出，切向即行进方向，
     * 车头反向由 path_follower 在消费端处理。
     * 段级参数 mode/backward/hold_direction/max_speed/padding/target_yaw 由 seg 字段映射：
     *   seg.mode == MODE_OMNI_TRANS_ROTATION(3) -> 原样透传
     *   seg.mode == MODE_OMNI_TRANSLATION(2)    -> hold_direction 为 NaN 时降级为 MODE_DIFF(1)
     *                                              （打印 warning），否则原样透传
     *   seg.mode == MODE_DIFF(1)                -> 原样透传
     *   seg.mode == 0（未显式指定，高级属性，rviz 不暴露）：
     *       mode = backward?DIFF : (isnan(hold_direction)?default_mode:OMNI_TRANSLATION)
     *   target_yaw 透传（NaN=到点不旋转）；max_speed/padding/hold_direction 透传（NaN=默认）。
     *
     * @param default_mode PathSegment.mode==0 时的默认跟踪方式
     *            （TrackingPath::MODE_DIFF=1 / MODE_OMNI_TRANS_ROTATION=3）。
     */
    class PathSegmentSampler
    {
    public:
        PathSegmentSampler() = default;

        /**
         * @brief 离散一段路径。
         * @param seg 已解析段（control_points 按行进方向序）
         * @param resolution 弧长采样间距 m（如 0.005 = 5mm）
         * @param frame_id 输出 TrackingPath 的坐标系
         * @param default_mode PathSegment 无显式 mode 时的默认跟踪方式
         * @return nav_interfaces/TrackingPath；seg 非法（control_points<2）时返回空 poses
         */
        nav_interfaces::msg::TrackingPath discretize(
            const nav_interfaces::msg::PathSegment& seg,
            double resolution, const std::string& frame_id,
            uint8_t default_mode = nav_interfaces::msg::TrackingPath::MODE_DIFF) const;

        /**
         * @brief 把连续若干段 LINE 合并离散成**一条**轨迹，接缝处用四次贝塞尔倒角。
         *
         * 折线顶点 = segs[begin].front, segs[begin].back(=segs[begin+1].front), ...,
         * segs[end-1].back。每个内部顶点按锚点方案退让 d = (degree-2)*anchor_spacing，
         * 并按「首尾边可整段使用、中间边被两个拐角共享各取一半」夹取，保证相邻两个拐角
         * 的贝塞尔段不重叠。整体先细描再按弧长统一重采样到 resolution，避免拼接处间距不均。
         *
         * 段级参数的组内取值（只有 backward/padding/mode 保证一致，见 sameTrackingAttributes）：
         *   max_speed      取组内**最小**（NaN=不限速），保守，不会让某段超过它自己的限速
         *   hold_direction 取首段（mode 一致时通常相同；不同则告警）
         *   target_yaw     取末段（调用方仍按「非末组置 NaN」处理）
         * （xy/th_tolerance 是 PathSegment 独有字段，TrackingPath 没有，不转发）
         *
         * @param segs        全部段
         * @param begin,end   组区间 [begin, end)，要求 end > begin
         * @param anchor_spacing 锚点间距 m
         * @param degree      贝塞尔阶数（只支持 4）
         * @return 合并后的 TrackingPath；组内出现非 LINE 段或顶点数不足时返回空 poses
         */
        nav_interfaces::msg::TrackingPath discretizeGroup(
            const std::vector<nav_interfaces::msg::PathSegment>& segs,
            std::size_t begin, std::size_t end,
            double resolution, double anchor_spacing, int degree,
            const std::string& frame_id,
            uint8_t default_mode = nav_interfaces::msg::TrackingPath::MODE_DIFF) const;

        /// 解析 PathSegment.mode -> TrackingPath.mode。mode==0 时的默认逻辑与
        /// MODE_OMNI_TRANSLATION 缺 hold_direction 的降级都在这里，合并判定要用同一份
        /// 结果，否则两段实际行为不同却被判成一致。
        static uint8_t resolveMode(
            const nav_interfaces::msg::PathSegment& seg, uint8_t default_mode);

    private:
        using Pt = std::pair<double, double>;

        /// 采样点：位置 + 该点的解析切向角 yaw（rad）。
        struct Sample
        {
            double x{0.0};
            double y{0.0};
            double yaw{0.0};
        };

        static std::vector<Sample> sampleLine(const Pt& a, const Pt& b, double res);
        /// 折线 -> 拐角四次贝塞尔平滑后的稠密采样（含解析切向），与
        /// scripts/corner_bezier_path_generator.py 的 build_smoothed_polyline_with_yaw 同一套方案。
        static std::vector<Sample> smoothedPolyline(
            const std::vector<Pt>& wps, double anchor_spacing, int degree, double dense_step);
        /// 拐角控制点（四次锚点方案）：[A, (A+P)/2, P, (P+B)/2, B]。
        static std::vector<Pt> cornerControlPoints(
            const Pt& p_prev, const Pt& p_corner, const Pt& p_next,
            double off_in, double off_out);
        /// 按弧长等距重采样，yaw 一并插值（先 unwrap 保证连续）。
        static std::vector<Sample> resampleByArcLength(
            const std::vector<Sample>& dense, double resolution);
        static std::vector<Sample> sampleBezier(const std::vector<Pt>& cp, double res);
        static Pt bezierEval(const std::vector<Pt>& cp, double t);
        /// 一阶导 B'(t)：把控制点降阶成 hodograph 后按 Bernstein 求值。
        static Pt bezierDeriv(const std::vector<Pt>& cp, double t);
        static long long binomial(int n, int k);
    };
} // namespace schedule_planner

#endif // SCHEDULE_PLANNER__PATH_SEGMENT_SAMPLER_HPP_
