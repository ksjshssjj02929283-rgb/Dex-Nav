#include "map_asset_manager/asset_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "map_asset_manager/polygon_utils.hpp"
#include "map_asset_manager/types.hpp"

namespace fs = std::filesystem;

namespace map_asset_manager {

namespace {

std::string Trim(std::string s) {
    const auto b = s.find_first_not_of(" \t\r\"'");
    if (b == std::string::npos) return "";
    const auto e = s.find_last_not_of(" \t\r\"'");
    return s.substr(b, e - b + 1);
}

// map_server 格式的 map.yaml：只取栅格化需要的键
struct GridYaml {
    std::string image;
    double resolution{0.0};
    double origin[3]{0.0, 0.0, 0.0};
    bool negate{false};
    double occupied_thresh{0.65};
    double free_thresh{0.196};
};

bool ParseGridYaml(const std::string& path, GridYaml& y, std::string& err) {
    std::ifstream in(path);
    if (!in) {
        err = "cannot open " + path;
        return false;
    }
    std::string line;
    while (std::getline(in, line)) {
        const auto hash = line.find('#');
        if (hash != std::string::npos) line = line.substr(0, hash);
        const auto colon = line.find(':');
        if (colon == std::string::npos) continue;
        const std::string key = Trim(line.substr(0, colon));
        const std::string val = Trim(line.substr(colon + 1));
        try {
            if (key == "image") y.image = val;
            else if (key == "resolution") y.resolution = std::stod(val);
            else if (key == "negate") y.negate = (val == "1" || val == "true" || val == "True");
            else if (key == "occupied_thresh") y.occupied_thresh = std::stod(val);
            else if (key == "free_thresh") y.free_thresh = std::stod(val);
            else if (key == "origin") {
                std::string v = val;
                std::replace(v.begin(), v.end(), '[', ' ');
                std::replace(v.begin(), v.end(), ']', ' ');
                std::replace(v.begin(), v.end(), ',', ' ');
                std::istringstream ss(v);
                ss >> y.origin[0] >> y.origin[1] >> y.origin[2];
            }
        } catch (const std::exception&) {
            err = "bad value for '" + key + "' in " + path;
            return false;
        }
    }
    if (y.image.empty() || y.resolution <= 0.0) {
        err = "map.yaml lacks image or resolution: " + path;
        return false;
    }
    return true;
}

std::string FileStamp(const fs::path& p) {
    std::error_code ec;
    const auto t = fs::last_write_time(p, ec);
    if (ec) return "missing";
    const auto size = fs::file_size(p, ec);
    return std::to_string(t.time_since_epoch().count()) + ":" + std::to_string(ec ? 0 : size);
}

// 路网沿线的问题按严重程度取最重的一个
int CodeRank(const std::string& c) {
    static const char* order[] = {GeoCode::OUT_OF_MAP,          GeoCode::CROSSES_OBSTACLE,
                                  GeoCode::CROSSES_RESTRICTED,  GeoCode::CROSSES_UNKNOWN,
                                  GeoCode::TOO_CLOSE_OBSTACLE,  GeoCode::TOO_CLOSE_RESTRICTED,
                                  GeoCode::TOO_CLOSE_UNKNOWN,   GeoCode::TOO_CLOSE_BOUNDARY};
    for (int i = 0; i < static_cast<int>(sizeof(order) / sizeof(order[0])); ++i)
        if (c == order[i]) return i;
    return 100;
}

double Cross(double ax, double ay, double bx, double by, double cx, double cy) {
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

bool OnSegment(double ax, double ay, double bx, double by, double px, double py) {
    return std::min(ax, bx) - 1e-12 <= px && px <= std::max(ax, bx) + 1e-12 && std::min(ay, by) - 1e-12 <= py &&
           py <= std::max(ay, by) + 1e-12;
}

bool SegmentsIntersect(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b,
                       const geometry_msgs::msg::Point& c, const geometry_msgs::msg::Point& d) {
    const double d1 = Cross(c.x, c.y, d.x, d.y, a.x, a.y);
    const double d2 = Cross(c.x, c.y, d.x, d.y, b.x, b.y);
    const double d3 = Cross(a.x, a.y, b.x, b.y, c.x, c.y);
    const double d4 = Cross(a.x, a.y, b.x, b.y, d.x, d.y);
    if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) return true;
    if (std::abs(d1) < 1e-12 && OnSegment(c.x, c.y, d.x, d.y, a.x, a.y)) return true;
    if (std::abs(d2) < 1e-12 && OnSegment(c.x, c.y, d.x, d.y, b.x, b.y)) return true;
    if (std::abs(d3) < 1e-12 && OnSegment(a.x, a.y, b.x, b.y, c.x, c.y)) return true;
    if (std::abs(d4) < 1e-12 && OnSegment(a.x, a.y, b.x, b.y, d.x, d.y)) return true;
    return false;
}

// 多边形首尾相连；相邻边共享顶点不算相交
bool PolygonSelfIntersects(const std::vector<geometry_msgs::msg::Point>& p) {
    const size_t n = p.size();
    if (n < 4) return false;
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            if (j == i + 1 || (i == 0 && j == n - 1)) continue;
            if (SegmentsIntersect(p[i], p[(i + 1) % n], p[j], p[(j + 1) % n])) return true;
        }
    }
    return false;
}

}  // namespace

bool GridMap::toCell(double x, double y, int& col, int& row) const {
    if (resolution <= 0.0) return false;
    double dx = x - origin_x, dy = y - origin_y;
    if (origin_yaw != 0.0) {
        const double c = std::cos(origin_yaw), s = std::sin(origin_yaw);
        const double rx = c * dx + s * dy, ry = -s * dx + c * dy;
        dx = rx;
        dy = ry;
    }
    col = static_cast<int>(std::floor(dx / resolution));
    row = static_cast<int>(std::floor(dy / resolution));
    return col >= 0 && row >= 0 && col < width && row < height;
}

double GridMap::distToBoundary(double x, double y) const {
    double dx = x - origin_x, dy = y - origin_y;
    if (origin_yaw != 0.0) {
        const double c = std::cos(origin_yaw), s = std::sin(origin_yaw);
        const double rx = c * dx + s * dy, ry = -s * dx + c * dy;
        dx = rx;
        dy = ry;
    }
    return std::min({dx, dy, width * resolution - dx, height * resolution - dy});
}

std::string ResolveGridYaml(const std::string& map_dir) {
    std::error_code ec;
    const fs::path layered = fs::path(map_dir) / "slam" / "map.yaml";
    if (fs::exists(layered, ec)) return layered.string();
    return (fs::path(map_dir) / "map.yaml").string();
}

std::shared_ptr<const GridMap> LoadGridMap(const std::string& yaml_path, std::string& err) {
    GridYaml y;
    if (!ParseGridYaml(yaml_path, y, err)) return nullptr;
    fs::path image = y.image;
    if (image.is_relative()) image = fs::path(yaml_path).parent_path() / image;
    const cv::Mat img = cv::imread(image.string(), cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        err = "cannot read map image " + image.string();
        return nullptr;
    }

    auto g = std::make_shared<GridMap>();
    g->width = img.cols;
    g->height = img.rows;
    g->resolution = y.resolution;
    g->origin_x = y.origin[0];
    g->origin_y = y.origin[1];
    g->origin_yaw = y.origin[2];
    g->cells.resize(static_cast<size_t>(g->width) * g->height);

    // 图像第 0 行是地图 y 最大处；三值判定与 map_server trinary 一致
    cv::Mat not_occ(img.rows, img.cols, CV_8U), not_unk(img.rows, img.cols, CV_8U);
    for (int r = 0; r < img.rows; ++r) {
        const uint8_t* src = img.ptr<uint8_t>(r);
        uint8_t* no = not_occ.ptr<uint8_t>(r);
        uint8_t* nu = not_unk.ptr<uint8_t>(r);
        for (int c = 0; c < img.cols; ++c) {
            const double occ = y.negate ? src[c] / 255.0 : (255.0 - src[c]) / 255.0;
            uint8_t cell = GridMap::UNKNOWN;
            if (occ > y.occupied_thresh) cell = GridMap::OCCUPIED;
            else if (occ < y.free_thresh) cell = GridMap::FREE;
            g->cells[static_cast<size_t>(img.rows - 1 - r) * g->width + c] = cell;
            no[c] = (cell == GridMap::OCCUPIED) ? 0 : 255;
            nu[c] = (cell == GridMap::UNKNOWN) ? 0 : 255;
        }
    }

    // 距离变换：格中心到最近障碍 / 未知格中心的像素距离，换算成到格边缘的米数
    cv::Mat d_occ, d_unk;
    cv::distanceTransform(not_occ, d_occ, cv::DIST_L2, cv::DIST_MASK_PRECISE);
    cv::distanceTransform(not_unk, d_unk, cv::DIST_L2, cv::DIST_MASK_PRECISE);
    g->dist_occupied.resize(g->cells.size());
    g->dist_unknown.resize(g->cells.size());
    const float half = static_cast<float>(y.resolution * 0.5);
    for (int r = 0; r < img.rows; ++r) {
        const float* po = d_occ.ptr<float>(r);
        const float* pu = d_unk.ptr<float>(r);
        const size_t base = static_cast<size_t>(img.rows - 1 - r) * g->width;
        for (int c = 0; c < img.cols; ++c) {
            g->dist_occupied[base + c] = std::max(0.0f, po[c] * static_cast<float>(y.resolution) - half);
            g->dist_unknown[base + c] = std::max(0.0f, pu[c] * static_cast<float>(y.resolution) - half);
        }
    }
    return g;
}

std::shared_ptr<const GridMap> GridMapCache::get(const std::string& map_dir, std::string& err) {
    const std::string yaml = ResolveGridYaml(map_dir);
    GridYaml y;
    std::string image_stamp = "missing";
    if (ParseGridYaml(yaml, y, err)) {
        fs::path image = y.image;
        if (image.is_relative()) image = fs::path(yaml).parent_path() / image;
        image_stamp = FileStamp(image);
    } else {
        std::lock_guard<std::mutex> lock(mutex_);
        entries_.erase(yaml);
        return nullptr;
    }
    const std::string stamp = FileStamp(yaml) + "|" + image_stamp;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = entries_.find(yaml);
        if (it != entries_.end() && it->second.stamp == stamp) return it->second.grid;
    }
    auto grid = LoadGridMap(yaml, err);
    std::lock_guard<std::mutex> lock(mutex_);
    if (grid) entries_[yaml] = Entry{stamp, grid};
    else entries_.erase(yaml);
    return grid;
}

std::vector<std::pair<double, double>> SampleEdgePath(const RouteEdge& edge, double ax, double ay, double bx,
                                                      double by, double step) {
    std::vector<std::pair<double, double>> ctrl;
    if (edge.edge_type == "BEZIER" && edge.control_points.size() >= 2) {
        for (const auto& cp : edge.control_points) ctrl.emplace_back(cp.first, cp.second);
        ctrl.front() = {ax, ay};
        ctrl.back() = {bx, by};
    } else {
        ctrl = {{ax, ay}, {bx, by}};
    }
    double len = 0.0;
    for (size_t i = 1; i < ctrl.size(); ++i) len += std::hypot(ctrl[i].first - ctrl[i - 1].first, ctrl[i].second - ctrl[i - 1].second);
    const int n = std::max(1, static_cast<int>(std::ceil(len / std::max(step, 1e-3))));
    std::vector<std::pair<double, double>> out;
    out.reserve(n + 1);
    std::vector<std::pair<double, double>> tmp(ctrl.size());
    for (int i = 0; i <= n; ++i) {
        const double t = static_cast<double>(i) / n;
        tmp = ctrl;
        for (size_t k = tmp.size() - 1; k > 0; --k)
            for (size_t m = 0; m < k; ++m)
                tmp[m] = {tmp[m].first + (tmp[m + 1].first - tmp[m].first) * t,
                          tmp[m].second + (tmp[m + 1].second - tmp[m].second) * t};
        out.push_back(tmp[0]);
    }
    return out;
}

GeometryChecker::GeometryChecker(std::shared_ptr<const GridMap> grid, const AssetSet& set, GeometryParams params)
    : grid_(std::move(grid)), params_(params) {
    for (const auto& kv : set.areas)
        if (kv.second.type == AreaType::RESTRICTED_AREA && kv.second.polygon.size() >= 3)
            restricted_.push_back(kv.second.polygon);
}

std::string GeometryChecker::checkSample(double x, double y, double clearance, bool path) const {
    double d_boundary = std::numeric_limits<double>::infinity();
    double d_occupied = d_boundary, d_unknown = d_boundary, d_restricted = d_boundary;
    if (grid_) {
        int col, row;
        if (!grid_->toCell(x, y, col, row)) return GeoCode::OUT_OF_MAP;
        const size_t i = static_cast<size_t>(row) * grid_->width + col;
        if (grid_->cells[i] == GridMap::OCCUPIED) return path ? GeoCode::CROSSES_OBSTACLE : GeoCode::IN_OBSTACLE;
        if (grid_->cells[i] == GridMap::UNKNOWN) return path ? GeoCode::CROSSES_UNKNOWN : GeoCode::IN_UNKNOWN;
        d_boundary = grid_->distToBoundary(x, y);
        d_occupied = grid_->dist_occupied[i];
        d_unknown = grid_->dist_unknown[i];
    }
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    for (const auto& poly : restricted_) {
        if (polygon_utils::isPointInPolygon(x, y, poly)) return path ? GeoCode::CROSSES_RESTRICTED : GeoCode::IN_RESTRICTED;
        d_restricted = std::min(d_restricted, polygon_utils::distanceToPolygonBoundary(p, poly));
    }
    const double m = std::min({d_boundary, d_occupied, d_unknown, d_restricted});
    if (m >= clearance) return "";
    if (m == d_occupied) return GeoCode::TOO_CLOSE_OBSTACLE;
    if (m == d_restricted) return GeoCode::TOO_CLOSE_RESTRICTED;
    if (m == d_unknown) return GeoCode::TOO_CLOSE_UNKNOWN;
    return GeoCode::TOO_CLOSE_BOUNDARY;
}

std::string GeometryChecker::checkPose(double x, double y) const { return checkSample(x, y, params_.clearance_m, false); }

std::string GeometryChecker::checkArea(const Area& area) const {
    const bool wall = (area.type == AreaType::VIRTUAL_WALL);
    if (area.polygon.size() < (wall ? 2u : 3u)) return GeoCode::TOO_FEW_VERTICES;
    if (!wall && PolygonSelfIntersects(area.polygon)) return GeoCode::SELF_INTERSECTING;
    if (grid_) {
        int c, r;
        for (const auto& v : area.polygon)
            if (!grid_->toCell(v.x, v.y, c, r)) return GeoCode::OUT_OF_MAP;
    }
    return "";
}

bool GeometryChecker::endpointPose(const RouteEndpoint& ep, const AssetSet& set, double& x, double& y) const {
    if (ep.type == "nav") {
        auto it = set.points.find(ep.id);
        if (it == set.points.end()) return false;
        x = it->second.x;
        y = it->second.y;
        return true;
    }
    if (ep.type == "junction") {
        auto it = set.junctions.find(ep.id);
        if (it == set.junctions.end()) return false;
        x = it->second.x;
        y = it->second.y;
        return true;
    }
    return false;
}

std::string GeometryChecker::checkEdge(const RouteEdge& edge, const AssetSet& set) const {
    double ax, ay, bx, by;
    if (!endpointPose(edge.from, set, ax, ay) || !endpointPose(edge.to, set, bx, by)) return GeoCode::ENDPOINT_UNAVAILABLE;
    if (!checkPose(ax, ay).empty() || !checkPose(bx, by).empty()) return GeoCode::ENDPOINT_UNAVAILABLE;
    if (edge.edge_type == "BEZIER" && edge.control_points.size() < 3) return GeoCode::MISSING_CONTROL_POINT;
    if (edge.lane_width > 0.0f && edge.lane_width < 2.0 * params_.clearance_m - 1e-6) return GeoCode::WIDTH_TOO_SMALL;

    const double half = edge.lane_width > 0.0f ? edge.lane_width * 0.5 : params_.clearance_m;
    const double step = grid_ ? grid_->resolution * 0.5 : 0.05;
    std::string worst;
    for (const auto& s : SampleEdgePath(edge, ax, ay, bx, by, step)) {
        const std::string c = checkSample(s.first, s.second, half, true);
        if (!c.empty() && (worst.empty() || CodeRank(c) < CodeRank(worst))) {
            worst = c;
            if (CodeRank(worst) == 0) break;
        }
    }
    return worst;
}

std::vector<nav_interfaces::msg::AssetIssue> GeometryChecker::computeIssues(const AssetSet& set) const {
    std::vector<nav_interfaces::msg::AssetIssue> out;
    auto add = [&](const char* kind, const std::string& key, const std::string& code, const std::string& detail) {
        nav_interfaces::msg::AssetIssue i;
        i.kind = kind;
        i.key = key;
        i.code = code;
        i.detail = detail;
        out.push_back(std::move(i));
    };
    std::vector<std::string> names;
    for (const auto& kv : set.points) names.push_back(kv.first);
    std::sort(names.begin(), names.end());
    std::unordered_map<std::string, bool> point_ok;
    for (const auto& n : names) {
        const auto& p = set.points.at(n);
        const std::string c = checkPose(p.x, p.y);
        point_ok[n] = c.empty();
        if (!c.empty()) add("point", n, c, "point " + n + ": " + c);
    }
    std::vector<std::string> ids;
    for (const auto& kv : set.edges) ids.push_back(kv.first);
    std::sort(ids.begin(), ids.end());
    for (const auto& id : ids) {
        const std::string c = checkEdge(set.edges.at(id), set);
        if (!c.empty()) add("edge", id, c, "edge " + id + ": " + c);
    }
    names.clear();
    for (const auto& kv : set.routes) names.push_back(kv.first);
    std::sort(names.begin(), names.end());
    for (const auto& n : names) {
        for (const auto& pn : set.routes.at(n).point_names) {
            auto it = point_ok.find(pn);
            if (it == point_ok.end() || !it->second) {
                add("route", n, GeoCode::POINT_UNAVAILABLE, "route " + n + ": point " + pn + " unavailable");
                break;
            }
        }
    }
    return out;
}

}  // namespace map_asset_manager
