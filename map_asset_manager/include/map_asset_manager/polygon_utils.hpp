#ifndef MAP_ASSET_MANAGER__POLYGON_UTILS_HPP_
#define MAP_ASSET_MANAGER__POLYGON_UTILS_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <vector>

namespace map_asset_manager::polygon_utils {

using PolygonPoint = geometry_msgs::msg::Point;
using Polygon = std::vector<PolygonPoint>;

struct AABB {
    double min_x{0.0};
    double min_y{0.0};
    double min_z{0.0};
    double max_x{0.0};
    double max_y{0.0};
    double max_z{0.0};
};

bool isPointInPolygon(double x, double y, const Polygon& poly);
bool isPointInPolygon(const PolygonPoint& p, const Polygon& poly);

PolygonPoint polygonCentroid(const Polygon& poly);

double polygonArea(const Polygon& poly);

double signedDistanceToPolygonBoundary(const PolygonPoint& p, const Polygon& poly);
double distanceToPolygonBoundary(const PolygonPoint& p, const Polygon& poly);

AABB polygonBoundingBox(const Polygon& poly);

bool isPolygonClosed(const Polygon& poly, double eps = 1e-6);

}  // namespace map_asset_manager::polygon_utils

#endif  // MAP_ASSET_MANAGER__POLYGON_UTILS_HPP_
