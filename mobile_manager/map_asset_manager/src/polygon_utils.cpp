#include "map_asset_manager/polygon_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace map_asset_manager::polygon_utils {

namespace {

// 点到线段的最短距离（用于 distanceToPolygonBoundary）
double distancePointToSegment(double px, double py,
                              double x1, double y1,
                              double x2, double y2) {
    const double dx = x2 - x1;
    const double dy = y2 - y1;
    const double len_sq = dx * dx + dy * dy;
    if (len_sq < 1e-12) {
        const double ex = px - x1;
        const double ey = py - y1;
        return std::sqrt(ex * ex + ey * ey);
    }
    double t = ((px - x1) * dx + (py - y1) * dy) / len_sq;
    t = std::clamp(t, 0.0, 1.0);
    const double cx = x1 + t * dx;
    const double cy = y1 + t * dy;
    const double ex = px - cx;
    const double ey = py - cy;
    return std::sqrt(ex * ex + ey * ey);
}

}  // namespace

bool isPointInPolygon(double x, double y, const Polygon& poly) {
    const size_t n = poly.size();
    if (n < 3) return false;
    bool inside = false;
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        const double xi = poly[i].x;
        const double yi = poly[i].y;
        const double xj = poly[j].x;
        const double yj = poly[j].y;
        const bool crosses = ((yi > y) != (yj > y)) &&
                             (x < (xj - xi) * (y - yi) / (yj - yi + 1e-30) + xi);
        if (crosses) inside = !inside;
    }
    return inside;
}

bool isPointInPolygon(const PolygonPoint& p, const Polygon& poly) {
    return isPointInPolygon(p.x, p.y, poly);
}

PolygonPoint polygonCentroid(const Polygon& poly) {
    PolygonPoint c;
    const size_t n = poly.size();
    if (n == 0) return c;
    if (n < 3) {
        double sx = 0.0, sy = 0.0;
        for (const auto& pt : poly) { sx += pt.x; sy += pt.y; }
        c.x = sx / static_cast<double>(n);
        c.y = sy / static_cast<double>(n);
        return c;
    }
    double sa = 0.0, cx = 0.0, cy = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const auto& p0 = poly[i];
        const auto& p1 = poly[(i + 1) % n];
        const double cross = p0.x * p1.y - p1.x * p0.y;
        sa += cross;
        cx += (p0.x + p1.x) * cross;
        cy += (p0.y + p1.y) * cross;
    }
    if (std::fabs(sa) < 1e-12) {
        // 退化为顶点平均
        double sx = 0.0, sy = 0.0;
        for (const auto& pt : poly) { sx += pt.x; sy += pt.y; }
        c.x = sx / static_cast<double>(n);
        c.y = sy / static_cast<double>(n);
        return c;
    }
    const double factor = 1.0 / (3.0 * sa);
    c.x = cx * factor;
    c.y = cy * factor;
    return c;
}

double polygonArea(const Polygon& poly) {
    const size_t n = poly.size();
    if (n < 3) return 0.0;
    double sa = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const auto& p0 = poly[i];
        const auto& p1 = poly[(i + 1) % n];
        sa += p0.x * p1.y - p1.x * p0.y;
    }
    return 0.5 * sa;  // CCW > 0, CW < 0
}

double signedDistanceToPolygonBoundary(const PolygonPoint& p, const Polygon& poly) {
    const double d = distanceToPolygonBoundary(p, poly);
    return isPointInPolygon(p, poly) ? -d : d;
}

double distanceToPolygonBoundary(const PolygonPoint& p, const Polygon& poly) {
    const size_t n = poly.size();
    if (n < 2) return std::numeric_limits<double>::infinity();
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < n; ++i) {
        const auto& p0 = poly[i];
        const auto& p1 = poly[(i + 1) % n];
        const double d = distancePointToSegment(p.x, p.y, p0.x, p0.y, p1.x, p1.y);
        if (d < best) best = d;
    }
    return best;
}

AABB polygonBoundingBox(const Polygon& poly) {
    AABB box;
    if (poly.empty()) return box;
    box.min_x = box.max_x = poly[0].x;
    box.min_y = box.max_y = poly[0].y;
    box.min_z = box.max_z = poly[0].z;
    for (const auto& pt : poly) {
        box.min_x = std::min(box.min_x, pt.x);
        box.min_y = std::min(box.min_y, pt.y);
        box.min_z = std::min(box.min_z, pt.z);
        box.max_x = std::max(box.max_x, pt.x);
        box.max_y = std::max(box.max_y, pt.y);
        box.max_z = std::max(box.max_z, pt.z);
    }
    return box;
}

bool isPolygonClosed(const Polygon& poly, double eps) {
    if (poly.size() < 3) return false;
    const auto& a = poly.front();
    const auto& b = poly.back();
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return (dx * dx + dy * dy) < (eps * eps);
}

}  // namespace map_asset_manager::polygon_utils
