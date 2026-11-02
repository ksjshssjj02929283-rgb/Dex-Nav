#include "map_asset_manager/asset_transform.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>

#include <rclcpp/logging.hpp>

namespace fs = std::filesystem;

namespace map_asset_manager {

namespace {

double NormalizeAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
}

struct Rigid2D {
    explicit Rigid2D(const OriginChange& t)
        : ox(t.x), oy(t.y), yaw(t.yaw), c(std::cos(t.yaw)), s(std::sin(t.yaw)) {}

    // 旧坐标 → 新坐标：先平移到新原点，再旋转 -yaw
    void apply(double x, double y, double& nx, double& ny) const {
        const double dx = x - ox;
        const double dy = y - oy;
        nx = c * dx + s * dy;
        ny = -s * dx + c * dy;
    }
    double heading(double a) const { return NormalizeAngle(a - yaw); }

    double ox, oy, yaw, c, s;
};

// 四元数左乘绕 z 轴旋转 -yaw：只改航向，保留横滚 / 俯仰
void RotateQuaternionYaw(geometry_msgs::msg::Quaternion& q, double yaw) {
    const double hz = std::sin(-yaw / 2.0);
    const double hw = std::cos(-yaw / 2.0);
    const double x = q.x, y = q.y, z = q.z, w = q.w;
    q.w = hw * w - hz * z;
    q.x = hw * x - hz * y;
    q.y = hw * y + hz * x;
    q.z = hw * z + hz * w;
}

}  // namespace

void TransformAssetsInPlace(const OriginChange& t,
                            std::unordered_map<std::string, NavPoint>& points,
                            std::unordered_map<std::string, Area>& areas,
                            std::vector<RouteJunction>& junctions,
                            std::vector<RouteEdge>& edges,
                            std::unordered_map<std::string, Room>& rooms,
                            AssetTransformStats* stats) {
    const Rigid2D tf(t);
    AssetTransformStats st;

    for (auto& [name, pt] : points) {
        tf.apply(pt.x, pt.y, pt.x, pt.y);
        pt.yaw = tf.heading(pt.yaw);
        ++st.points;
    }

    for (auto& [name, area] : areas) {
        for (auto& p : area.polygon) tf.apply(p.x, p.y, p.x, p.y);
        ++st.areas;
    }

    for (auto& j : junctions) {
        double nx, ny;
        tf.apply(j.x, j.y, nx, ny);
        j.x = static_cast<float>(nx);
        j.y = static_cast<float>(ny);
        j.yaw = static_cast<float>(tf.heading(j.yaw));
        ++st.junctions;
    }

    for (auto& e : edges) {
        bool changed = false;
        for (auto& cp : e.control_points) {
            double nx, ny;
            tf.apply(cp.first, cp.second, nx, ny);
            cp.first = static_cast<float>(nx);
            cp.second = static_cast<float>(ny);
            changed = true;
        }
        // 平移朝向是地图系下的车体朝向（缺省时 path_follower 取起步朝向），随坐标系一起转
        if (e.hold_direction) {
            e.hold_direction = static_cast<float>(tf.heading(*e.hold_direction));
            changed = true;
        }
        if (changed) ++st.edges;
    }

    for (auto& [name, r] : rooms) {
        for (auto& p : r.polygon) tf.apply(p.x, p.y, p.x, p.y);
        tf.apply(r.pose.position.x, r.pose.position.y, r.pose.position.x, r.pose.position.y);
        RotateQuaternionYaw(r.pose.orientation, t.yaw);
        if (!r.polygon.empty()) {
            double minx = r.polygon[0].x, miny = r.polygon[0].y;
            double maxx = minx, maxy = miny;
            for (const auto& p : r.polygon) {
                minx = std::min(minx, p.x);
                miny = std::min(miny, p.y);
                maxx = std::max(maxx, p.x);
                maxy = std::max(maxy, p.y);
            }
            // z 范围保持原值，只重算平面外包框
            r.bounding_box[0] = minx;
            r.bounding_box[1] = miny;
            r.bounding_box[3] = maxx;
            r.bounding_box[4] = maxy;
        }
        ++st.rooms;
    }

    if (stats) *stats = st;
}

std::string ResolveAssetsFile(const std::string& map_dir) {
    const fs::path dir(map_dir);
    std::error_code ec;
    const bool legacy_layout = !fs::exists(dir / "slam", ec) &&
                               (fs::exists(dir / "map_assets.json", ec) ||
                                fs::exists(dir / "map.yaml", ec));
    return legacy_layout ? (dir / "map_assets.json").string()
                         : (dir / "assets" / "map_assets.json").string();
}

AssetFileTransformResult TransformAssetsFile(const std::string& assets_file, const OriginChange& t,
                                             FileManager& file_manager, const rclcpp::Logger& logger) {
    AssetFileTransformResult out;
    out.assets_file = assets_file;

    std::error_code ec;
    if (!fs::exists(assets_file, ec)) {
        out.ok = true;
        out.no_assets = true;
        out.message = "no assets file, nothing to transform: " + assets_file;
        return out;
    }

    const AssetFileLock lock(assets_file);
    if (!lock.locked()) {
        out.message = "failed to lock assets file: " + assets_file;
        return out;
    }
    out.version_ts = CreateAssetVersion(assets_file, logger, AssetVersionSource::TRANSFORM);
    if (out.version_ts.empty()) {
        out.message = "failed to save asset version before transform: " + assets_file;
        return out;
    }

    std::unordered_map<std::string, NavPoint> points;
    std::unordered_map<std::string, Route> routes;
    std::unordered_map<std::string, Area> areas;
    std::vector<RouteJunction> junctions;
    std::vector<RouteEdge> edges;
    std::unordered_map<std::string, Room> rooms;
    if (!file_manager.loadAllAssetsFromFile(assets_file, points, routes, areas, junctions, edges, rooms)) {
        out.message = "failed to load assets: " + assets_file;
        return out;
    }

    TransformAssetsInPlace(t, points, areas, junctions, edges, rooms, &out.stats);

    if (!file_manager.saveAllAssetsToFile(assets_file, points, routes, areas, junctions, edges, rooms)) {
        out.message = "failed to save transformed assets: " + assets_file;
        return out;
    }

    out.ok = true;
    out.message = "transformed: points=" + std::to_string(out.stats.points) +
                  " areas=" + std::to_string(out.stats.areas) +
                  " junctions=" + std::to_string(out.stats.junctions) +
                  " edges=" + std::to_string(out.stats.edges) +
                  " rooms=" + std::to_string(out.stats.rooms) +
                  "; asset version " + out.version_ts;
    return out;
}

}  // namespace map_asset_manager
