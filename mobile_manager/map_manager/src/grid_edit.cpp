#include "map_manager/grid_edit.hpp"

#include <algorithm>
#include <cmath>

namespace map_manager {

namespace {

constexpr double kMaxBrushWidthM = 10.0;   // 前端范围 0.05~3.00 m，后端留余量只挡明显错误
constexpr size_t kMaxStrokes = 10000;
constexpr size_t kMaxTotalPoints = 200000;
constexpr double kEps = 1e-9;

double OccPixelBound(const GridThresholds& th) { return 255.0 * (1.0 - th.occupied_thresh); }
double FreePixelBound(const GridThresholds& th) { return 255.0 * (1.0 - th.free_thresh); }

// 点到线段的距离（米）。
double DistToSegment(double px, double py, double ax, double ay, double bx, double by) {
    const double dx = bx - ax, dy = by - ay;
    const double len2 = dx * dx + dy * dy;
    double t = 0.0;
    if (len2 > 0.0) t = std::clamp(((px - ax) * dx + (py - ay) * dy) / len2, 0.0, 1.0);
    const double cx = ax + t * dx - px, cy = ay + t * dy - py;
    return std::sqrt(cx * cx + cy * cy);
}

// 米 → 格（未裁剪）。col = floor((x − ox)/res)，row = (H − 1) − floor((y − oy)/res)。
int ColOf(double x, const GridGeometry& g) { return static_cast<int>(std::floor((x - g.origin_x) / g.resolution)); }
int RowOf(double y, const GridGeometry& g) {
    return (g.height - 1) - static_cast<int>(std::floor((y - g.origin_y) / g.resolution));
}
double CenterX(int col, const GridGeometry& g) { return g.origin_x + (col + 0.5) * g.resolution; }
double CenterY(int row, const GridGeometry& g) {
    return g.origin_y + ((g.height - 1 - row) + 0.5) * g.resolution;
}

void ApplyPixel(uint8_t* p, uint8_t tool, const GridThresholds& th, uint8_t unknown_gray) {
    const CellClass c = ClassifyPixel(*p, th);
    switch (tool) {
        case kToolWall:    if (c != CellClass::kOccupied) *p = 0; break;
        case kToolErase:   if (c == CellClass::kOccupied) *p = 255; break;
        case kToolUnknown: if (c != CellClass::kUnknown) *p = unknown_gray; break;
        default: break;
    }
}

// 对米制包围盒内（已外扩）的格子逐个判断 covered(center_x, center_y)。
template <typename Covered>
void ForCellsInBox(cv::Mat* gray, const GridGeometry& g, double xmin, double xmax, double ymin,
                   double ymax, uint8_t tool, const GridThresholds& th, uint8_t unknown_gray,
                   Covered covered) {
    const int c0 = std::max(0, ColOf(xmin, g)), c1 = std::min(g.width - 1, ColOf(xmax, g));
    const int r0 = std::max(0, RowOf(ymax, g)), r1 = std::min(g.height - 1, RowOf(ymin, g));
    for (int r = r0; r <= r1; ++r) {
        uint8_t* row = gray->ptr<uint8_t>(r);
        const double cy = CenterY(r, g);
        for (int c = c0; c <= c1; ++c) {
            if (covered(CenterX(c, g), cy)) ApplyPixel(&row[c], tool, th, unknown_gray);
        }
    }
}

}  // namespace

CellClass ClassifyPixel(const uint8_t p, const GridThresholds& th) {
    if (p > FreePixelBound(th)) return CellClass::kFree;
    if (p < OccPixelBound(th)) return CellClass::kOccupied;
    return CellClass::kUnknown;
}

bool ValidThresholds(const GridThresholds& th) {
    if (!(th.free_thresh > 0.0 && th.free_thresh < th.occupied_thresh && th.occupied_thresh < 1.0)) {
        return false;
    }
    return UnknownGray(th) >= 0;
}

int UnknownGray(const GridThresholds& th) {
    const int lo = static_cast<int>(std::ceil(OccPixelBound(th)));    // 最小的「非障碍」灰度
    const int hi = static_cast<int>(std::floor(FreePixelBound(th)));  // 最大的「非自由」灰度
    if (lo > hi || lo < 0 || hi > 255) return -1;
    if (205 >= lo && 205 <= hi) return 205;
    return (lo + hi) / 2;
}

std::string ValidateStrokes(const std::vector<GridStrokeSpec>& strokes) {
    if (strokes.empty()) return "strokes is empty";
    if (strokes.size() > kMaxStrokes) return "too many strokes (max " + std::to_string(kMaxStrokes) + ")";
    size_t total_points = 0;
    for (size_t i = 0; i < strokes.size(); ++i) {
        const auto& s = strokes[i];
        const std::string at = "stroke " + std::to_string(i) + ": ";
        if (s.tool != kToolWall && s.tool != kToolErase && s.tool != kToolUnknown) {
            return at + "invalid tool " + std::to_string(s.tool);
        }
        if (s.shape != kShapePolyline && s.shape != kShapeRect) {
            return at + "invalid shape " + std::to_string(s.shape);
        }
        if (s.points.empty()) return at + "no points";
        for (const auto& p : s.points) {
            if (!std::isfinite(p.first) || !std::isfinite(p.second)) return at + "non-finite point";
        }
        if (s.shape == kShapeRect) {
            if (s.points.size() != 2) return at + "rect needs exactly 2 points";
        } else if (!std::isfinite(s.width_m) || s.width_m <= 0.0 || s.width_m > kMaxBrushWidthM) {
            return at + "width_m must be in (0, 10] meters";
        }
        total_points += s.points.size();
    }
    if (total_points > kMaxTotalPoints) return "too many points (max " + std::to_string(kMaxTotalPoints) + ")";
    return "";
}

int ApplyStrokes(cv::Mat* gray, const GridGeometry& geo, const GridThresholds& th,
                 const std::vector<GridStrokeSpec>& strokes) {
    const cv::Mat before = gray->clone();
    const uint8_t unknown_gray = static_cast<uint8_t>(UnknownGray(th));
    for (const auto& s : strokes) {
        if (s.shape == kShapeRect) {
            const double x0 = std::min(s.points[0].first, s.points[1].first);
            const double x1 = std::max(s.points[0].first, s.points[1].first);
            const double y0 = std::min(s.points[0].second, s.points[1].second);
            const double y1 = std::max(s.points[0].second, s.points[1].second);
            ForCellsInBox(gray, geo, x0, x1, y0, y1, s.tool, th, unknown_gray,
                          [&](double cx, double cy) {
                              return cx >= x0 - kEps && cx <= x1 + kEps && cy >= y0 - kEps && cy <= y1 + kEps;
                          });
            continue;
        }
        // 折线：每段一个胶囊，半径 = 笔刷半径 + 半格（保证再细的笔也至少覆盖一格）
        const double r = s.width_m / 2.0 + geo.resolution / 2.0;
        const size_t n = s.points.size();
        for (size_t i = 0; i < (n == 1 ? 1 : n - 1); ++i) {
            const auto& a = s.points[i];
            const auto& b = (n == 1) ? s.points[0] : s.points[i + 1];
            ForCellsInBox(gray, geo, std::min(a.first, b.first) - r, std::max(a.first, b.first) + r,
                          std::min(a.second, b.second) - r, std::max(a.second, b.second) + r, s.tool, th,
                          unknown_gray, [&](double cx, double cy) {
                              return DistToSegment(cx, cy, a.first, a.second, b.first, b.second) <= r + kEps;
                          });
        }
    }
    return cv::countNonZero(before != *gray);
}

}  // namespace map_manager
