#include "map_manager_rviz_plugin/grid_edit_draft.hpp"

#include <cmath>

namespace map_manager_rviz_plugin {

namespace {

// 自由绘制采样间距与折线简化容差（米）。方案要求相邻采样点不超过一个栅格（常见 0.05 m）、
// 简化容差取半格；这里取更细的值，形状只会更贴近手势。
constexpr double kSampleStepM = 0.02;
constexpr double kSimplifyTolM = 0.01;

double PointToSegment(const std::pair<double, double>& p, const std::pair<double, double>& a,
                      const std::pair<double, double>& b) {
    const double dx = b.first - a.first, dy = b.second - a.second;
    const double l2 = dx * dx + dy * dy;
    double t = l2 > 0 ? ((p.first - a.first) * dx + (p.second - a.second) * dy) / l2 : 0.0;
    t = std::max(0.0, std::min(1.0, t));
    return std::hypot(a.first + t * dx - p.first, a.second + t * dy - p.second);
}

// Douglas-Peucker 折线简化，保留首尾点。
void Simplify(const std::vector<std::pair<double, double>>& in, size_t i0, size_t i1,
              std::vector<bool>* keep) {
    if (i1 <= i0 + 1) return;
    double best = -1.0;
    size_t idx = i0;
    for (size_t i = i0 + 1; i < i1; ++i) {
        const double d = PointToSegment(in[i], in[i0], in[i1]);
        if (d > best) { best = d; idx = i; }
    }
    if (best > kSimplifyTolM) {
        (*keep)[idx] = true;
        Simplify(in, i0, idx, keep);
        Simplify(in, idx, i1, keep);
    }
}

}  // namespace

GridEditDraft& GridEditDraft::instance() {
    static GridEditDraft draft;
    return draft;
}

void GridEditDraft::beginStroke(double x, double y) {
    current_ = DraftStroke{};
    current_.tool = tool;
    current_.shape = (mode == DrawMode::kRect) ? kShapeRect : kShapePolyline;
    current_.width_m = width_m;
    current_.points = {{x, y}};
    if (mode != DrawMode::kFree) current_.points.push_back({x, y});  // 直线 / 矩形：第二点随鼠标移动
    active_ = true;
    Q_EMIT changed();
}

void GridEditDraft::moveStroke(double x, double y, bool snap45) {
    if (!active_) return;
    if (mode == DrawMode::kFree) {
        const auto& last = current_.points.back();
        if (std::hypot(x - last.first, y - last.second) < kSampleStepM) return;
        current_.points.push_back({x, y});
    } else {
        if (mode == DrawMode::kLine && snap45) {
            const auto& s = current_.points.front();
            const double len = std::hypot(x - s.first, y - s.second);
            const double ang = std::round(std::atan2(y - s.second, x - s.first) / (M_PI / 4.0)) * (M_PI / 4.0);
            x = s.first + len * std::cos(ang);
            y = s.second + len * std::sin(ang);
        }
        current_.points.back() = {x, y};
    }
    Q_EMIT changed();
}

void GridEditDraft::endStroke() {
    if (!active_) return;
    active_ = false;
    auto& pts = current_.points;
    if (current_.shape == kShapePolyline) {
        if (pts.size() == 2 && pts[0] == pts[1]) pts.pop_back();  // 直线模式单击 = 圆点
        if (pts.size() > 2) {
            std::vector<bool> keep(pts.size(), false);
            keep.front() = keep.back() = true;
            Simplify(pts, 0, pts.size() - 1, &keep);
            std::vector<std::pair<double, double>> out;
            for (size_t i = 0; i < pts.size(); ++i) if (keep[i]) out.push_back(pts[i]);
            pts.swap(out);
        }
    } else if (pts.size() == 2 && pts[0] == pts[1]) {
        Q_EMIT changed();  // 零面积矩形丢弃
        return;
    }
    strokes_.push_back(current_);
    Q_EMIT changed();
}

void GridEditDraft::undo() {
    if (active_) active_ = false;
    else if (!strokes_.empty()) strokes_.pop_back();
    Q_EMIT changed();
}

void GridEditDraft::clear() {
    strokes_.clear();
    active_ = false;
    Q_EMIT changed();
}

}  // namespace map_manager_rviz_plugin
