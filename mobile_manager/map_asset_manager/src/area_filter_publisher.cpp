#include "map_asset_manager/area_filter_publisher.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace map_asset_manager {

namespace {

constexpr int8_t kFree = 0;
constexpr int8_t kKeepout = 100;
constexpr uint8_t kKeepoutFilterType = 0;
constexpr uint8_t kSpeedFilterAbsoluteType = 2;

template <typename T>
void HashCombine(size_t* seed, const T& v) {
    *seed ^= std::hash<T>()(v) + 0x9e3779b97f4a7c15ULL + (*seed << 6) + (*seed >> 2);
}

void HashQ(size_t* seed, double v, double res) { HashCombine(seed, static_cast<int64_t>(std::llround(v / res))); }

}  // namespace

AreaFilterPublisher::AreaFilterPublisher(rclcpp::Node* node) : node_(node) {
    const std::string map_topic = node_->declare_parameter<std::string>("map_topic", "/map");
    keepout_mask_topic_ = node_->declare_parameter<std::string>("keepout_mask_topic", "/keepout_filter_mask");
    speed_mask_topic_ = node_->declare_parameter<std::string>("speed_mask_topic", "/speed_filter_mask");
    const std::string keepout_info_topic =
        node_->declare_parameter<std::string>("keepout_filter_info_topic", "/keepout_costmap_filter_info");
    const std::string speed_info_topic =
        node_->declare_parameter<std::string>("speed_filter_info_topic", "/speed_costmap_filter_info");
    speed_base_ = node_->declare_parameter<double>("speed_base", 0.0);
    speed_multiplier_ = node_->declare_parameter<double>("speed_multiplier", 0.01);
    virtual_wall_width_ = node_->declare_parameter<double>("virtual_wall_width", 0.10);

    auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic, latched, std::bind(&AreaFilterPublisher::onMap, this, std::placeholders::_1));
    keepout_mask_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(keepout_mask_topic_, latched);
    speed_mask_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(speed_mask_topic_, latched);
    keepout_info_pub_ = node_->create_publisher<nav2_msgs::msg::CostmapFilterInfo>(keepout_info_topic, latched);
    speed_info_pub_ = node_->create_publisher<nav2_msgs::msg::CostmapFilterInfo>(speed_info_topic, latched);
}

void AreaFilterPublisher::setAreas(const std::vector<Area>& areas) {
    areas_ = areas;
    std::sort(areas_.begin(), areas_.end(), [](const Area& a, const Area& b) { return a.name < b.name; });
    has_areas_ = true;
    renderAndPublish();
}

void AreaFilterPublisher::onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    map_ = *msg;
    has_map_ = true;
    renderAndPublish();
}

// 输入（底图几何 + 区域内容）不变时不重发：Nav2 filter 每收到一次 latched 更新都会整体替换
size_t AreaFilterPublisher::renderHash() const {
    size_t seed = 0;
    const auto& info = map_.info;
    HashCombine(&seed, map_.header.frame_id);
    HashCombine(&seed, info.width);
    HashCombine(&seed, info.height);
    HashQ(&seed, info.resolution, 1e-6);
    HashQ(&seed, info.origin.position.x, 1e-3);
    HashQ(&seed, info.origin.position.y, 1e-3);
    HashQ(&seed, info.origin.orientation.z, 1e-6);
    HashQ(&seed, info.origin.orientation.w, 1e-6);
    HashCombine(&seed, areas_.size());
    for (const auto& a : areas_) {
        HashCombine(&seed, a.name);
        HashCombine(&seed, a.type);
        HashQ(&seed, a.max_speed, 1e-3);
        for (const auto& p : a.polygon) {
            HashQ(&seed, p.x, 1e-3);
            HashQ(&seed, p.y, 1e-3);
        }
    }
    return seed;
}

void AreaFilterPublisher::renderAndPublish() {
    if (!has_map_ || !has_areas_) return;
    const size_t h = renderHash();
    if (has_last_hash_ && h == last_hash_) return;

    const auto stamp = node_->now();
    keepout_mask_ = map_;
    speed_mask_ = map_;
    keepout_mask_.header.stamp = stamp;
    speed_mask_.header.stamp = stamp;
    keepout_mask_.data.assign(static_cast<size_t>(map_.info.width) * map_.info.height, kFree);
    speed_mask_.data.assign(keepout_mask_.data.size(), kFree);
    speed_limits_.assign(keepout_mask_.data.size(), std::numeric_limits<float>::infinity());

    // 地图原点带 yaw 时，世界坐标先转到地图局部系（原点平移 + 旋转 -yaw）；
    // 局部系里原点为 0、无旋转，下面的栅格化与 worldToMap 都按局部系算
    const double oyaw = 2.0 * std::atan2(map_.info.origin.orientation.z, map_.info.origin.orientation.w);
    const double ox = map_.info.origin.position.x, oy = map_.info.origin.position.y;
    const double c = std::cos(oyaw), sn = std::sin(oyaw);
    auto to_local = [&](std::vector<geometry_msgs::msg::Point> poly) {
        for (auto& p : poly) {
            const double dx = p.x - ox, dy = p.y - oy;
            p.x = c * dx + sn * dy;
            p.y = -sn * dx + c * dy;
        }
        return poly;
    };
    nav_msgs::msg::MapMetaData local_info = map_.info;
    local_info.origin.position.x = 0.0;
    local_info.origin.position.y = 0.0;
    local_info_ = local_info;

    Stats st;
    st.total = areas_.size();
    for (const auto& area : areas_) {
        Area a = area;
        a.polygon = to_local(area.polygon);
        if (a.type == AreaType::VIRTUAL_WALL && a.polygon.size() >= 2) {
            rasterizeVirtualWall(a.polygon);
            ++st.keepout;
        } else if (a.type == AreaType::RESTRICTED_AREA && a.polygon.size() >= 3) {
            fillPolygonCells(a.polygon, local_info_, [this](int mx, int my) { setKeepoutCell(mx, my); });
            ++st.keepout;
        } else if (a.type == AreaType::SPEED_LIMIT_AREA && a.polygon.size() >= 3 && a.max_speed > 0.0f &&
                   encodeSpeedLimit(a.max_speed) > 0) {
            const float limit = a.max_speed;
            const int8_t code = encodeSpeedLimit(limit);
            fillPolygonCells(a.polygon, local_info_, [&](int mx, int my) {
                const size_t i = static_cast<size_t>(my) * speed_mask_.info.width + mx;
                if (limit < speed_limits_[i]) {
                    speed_limits_[i] = limit;
                    speed_mask_.data[i] = code;
                }
            });
            ++st.speed;
        } else {
            ++st.ignored;  // 其它类型不映射 costmap filter（由各自消费方处理）
        }
    }

    nav2_msgs::msg::CostmapFilterInfo keepout_info;
    keepout_info.header.stamp = stamp;
    keepout_info.header.frame_id = map_.header.frame_id;
    keepout_info.type = kKeepoutFilterType;
    keepout_info.filter_mask_topic = keepout_mask_topic_;
    keepout_info.base = 0.0;
    keepout_info.multiplier = 1.0;
    nav2_msgs::msg::CostmapFilterInfo speed_info = keepout_info;
    speed_info.type = kSpeedFilterAbsoluteType;
    speed_info.filter_mask_topic = speed_mask_topic_;
    speed_info.base = speed_base_;
    speed_info.multiplier = speed_multiplier_;
    keepout_info_pub_->publish(keepout_info);
    speed_info_pub_->publish(speed_info);
    keepout_mask_pub_->publish(keepout_mask_);
    speed_mask_pub_->publish(speed_mask_);

    st.published = true;
    last_stats_ = st;
    last_hash_ = h;
    has_last_hash_ = true;
    RCLCPP_INFO(node_->get_logger(), "Area filter masks published: areas=%zu keepout=%zu speed=%zu ignored=%zu",
                st.total, st.keepout, st.speed, st.ignored);
}

// 输入为地图局部系坐标（见 renderAndPublish）
bool AreaFilterPublisher::worldToMap(double wx, double wy, int* mx, int* my) const {
    const auto& info = local_info_;
    if (info.resolution <= 0.0f) return false;
    const int x = static_cast<int>(std::floor((wx - info.origin.position.x) / info.resolution));
    const int y = static_cast<int>(std::floor((wy - info.origin.position.y) / info.resolution));
    if (x < 0 || y < 0 || x >= static_cast<int>(info.width) || y >= static_cast<int>(info.height)) return false;
    *mx = x;
    *my = y;
    return true;
}

void AreaFilterPublisher::setKeepoutCell(int mx, int my) {
    const auto& info = keepout_mask_.info;
    if (mx < 0 || my < 0 || mx >= static_cast<int>(info.width) || my >= static_cast<int>(info.height)) return;
    keepout_mask_.data[static_cast<size_t>(my) * info.width + mx] = kKeepout;
}

void AreaFilterPublisher::inflateKeepoutCell(int cx, int cy, double radius) {
    const double res = keepout_mask_.info.resolution;
    const int rc = std::max(0, static_cast<int>(std::ceil(radius / res)));
    for (int dy = -rc; dy <= rc; ++dy)
        for (int dx = -rc; dx <= rc; ++dx)
            if (std::hypot(dx * res, dy * res) <= radius + res * 0.5) setKeepoutCell(cx + dx, cy + dy);
}

void AreaFilterPublisher::rasterizeVirtualWall(const std::vector<geometry_msgs::msg::Point>& pts) {
    const double res = keepout_mask_.info.resolution;
    if (res <= 0.0) return;
    const double radius = std::max(virtual_wall_width_, res) * 0.5;
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        const double len = std::hypot(pts[i + 1].x - pts[i].x, pts[i + 1].y - pts[i].y);
        const int steps = std::max(1, static_cast<int>(std::ceil(len / (res * 0.5))));
        for (int k = 0; k <= steps; ++k) {
            const double t = static_cast<double>(k) / steps;
            int mx, my;
            if (worldToMap(pts[i].x + (pts[i + 1].x - pts[i].x) * t, pts[i].y + (pts[i + 1].y - pts[i].y) * t, &mx, &my))
                inflateKeepoutCell(mx, my, radius);
        }
    }
}

// 扫描线填充：格中心落在多边形内的格子
void AreaFilterPublisher::fillPolygonCells(const std::vector<geometry_msgs::msg::Point>& polygon,
                                           const nav_msgs::msg::MapMetaData& info,
                                           const std::function<void(int, int)>& fill) const {
    if (polygon.size() < 3 || info.resolution <= 0.0f) return;
    double min_y = std::numeric_limits<double>::max(), max_y = std::numeric_limits<double>::lowest();
    for (const auto& p : polygon) {
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
    }
    const double res = info.resolution;
    const int min_my = std::max(0, static_cast<int>(std::ceil((min_y - info.origin.position.y) / res - 0.5)));
    const int max_my = std::min(static_cast<int>(info.height) - 1,
                                static_cast<int>(std::floor((max_y - info.origin.position.y) / res - 0.5)));
    std::vector<double> xs;
    for (int my = min_my; my <= max_my; ++my) {
        const double y = info.origin.position.y + (my + 0.5) * res;
        xs.clear();
        for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
            const auto& p1 = polygon[j];
            const auto& p2 = polygon[i];
            if ((p1.y <= y && p2.y > y) || (p2.y <= y && p1.y > y))
                xs.push_back(p1.x + (y - p1.y) / (p2.y - p1.y) * (p2.x - p1.x));
        }
        std::sort(xs.begin(), xs.end());
        for (size_t i = 0; i + 1 < xs.size(); i += 2) {
            const int a = std::max(0, static_cast<int>(std::ceil((xs[i] - info.origin.position.x) / res - 0.5)));
            const int b = std::min(static_cast<int>(info.width) - 1,
                                   static_cast<int>(std::floor((xs[i + 1] - info.origin.position.x) / res - 0.5)));
            for (int mx = a; mx <= b; ++mx) fill(mx, my);
        }
    }
}

int8_t AreaFilterPublisher::encodeSpeedLimit(float speed_limit) const {
    if (speed_multiplier_ <= 0.0) return 0;
    const double encoded = std::round((static_cast<double>(speed_limit) - speed_base_) / speed_multiplier_);
    if (encoded < 1.0 || encoded > 100.0) return 0;
    return static_cast<int8_t>(encoded);
}

}  // namespace map_asset_manager
