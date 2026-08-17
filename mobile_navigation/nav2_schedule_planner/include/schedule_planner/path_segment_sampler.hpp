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
     * poses 朝向暂时无意义（path_follower 只用 position），统一置 0。
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

    private:
        using Pt = std::pair<double, double>;

        static std::vector<Pt> sampleLine(const Pt& a, const Pt& b, double res);
        static std::vector<Pt> sampleBezier(const std::vector<Pt>& cp, double res);
        static Pt bezierEval(const std::vector<Pt>& cp, double t);
        static long long binomial(int n, int k);
    };
} // namespace schedule_planner

#endif // SCHEDULE_PLANNER__PATH_SEGMENT_SAMPLER_HPP_
