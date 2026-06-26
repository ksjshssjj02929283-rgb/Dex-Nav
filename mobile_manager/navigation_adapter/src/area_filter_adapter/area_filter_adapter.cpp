#include "navigation_adapter/area_filter_adapter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <sstream>

namespace navigation_adapter {

namespace {

constexpr int8_t kOccupancyFree = 0;
constexpr int8_t kOccupancyKeepout = 100;
constexpr uint8_t kKeepoutFilterType = 0;
constexpr uint8_t kSpeedFilterAbsoluteType = 2;
constexpr double kHashPositionResolution = 0.001;      // 1 mm
constexpr double kHashAngleResolution = 0.0001;        // about 0.006 degree
constexpr double kHashSpeedResolution = 0.001;         // 1 mm/s
constexpr double kHashMapResolutionResolution = 1e-6;  // 0.001 mm
constexpr double kHashUnitlessResolution = 1e-6;

template <typename T>
void hashCombine(size_t* seed, const T& value) {
    std::hash<T> hasher;
    *seed ^= hasher(value) + 0x9e3779b97f4a7c15ULL + (*seed << 6) + (*seed >> 2);
}

int64_t quantizeDouble(double value, double resolution) {
    return static_cast<int64_t>(std::llround(value / resolution));
}

void hashQuantized(size_t* seed, double value, double resolution) {
    hashCombine(seed, quantizeDouble(value, resolution));
}

void hashPose2D(size_t* seed, const geometry_msgs::msg::Pose2D& pose) {
    hashQuantized(seed, pose.x, kHashPositionResolution);
    hashQuantized(seed, pose.y, kHashPositionResolution);
    hashQuantized(seed, pose.theta, kHashAngleResolution);
}

}  // namespace

// ──────────────────────────────────────────────────────────────────────
// 构造与初始化
// ──────────────────────────────────────────────────────────────────────

AreaFilterAdapter::AreaFilterAdapter(rclcpp::Node* node) : node_(node) {
    node_->declare_parameter<std::string>("map_topic", "/map");
    node_->declare_parameter<std::string>("areas_topic", "/map_asset_manager/areas");
    node_->declare_parameter<std::string>("map_state_topic", "map_manager/map_state");
    node_->declare_parameter<std::string>("keepout_mask_topic", "/keepout_filter_mask");
    node_->declare_parameter<std::string>("speed_mask_topic", "/speed_filter_mask");
    node_->declare_parameter<std::string>("keepout_filter_info_topic", "/keepout_costmap_filter_info");
    node_->declare_parameter<std::string>("speed_filter_info_topic", "/speed_costmap_filter_info");
    node_->declare_parameter<double>("speed_base", 0.0);
    node_->declare_parameter<double>("speed_multiplier", 0.01);
    node_->declare_parameter<double>("virtual_wall_width", 0.10);
    node_->declare_parameter<double>("publish_status_period_sec", 5.0);

    map_topic_ = node_->get_parameter("map_topic").as_string();
    areas_topic_ = node_->get_parameter("areas_topic").as_string();
    map_state_topic_ = node_->get_parameter("map_state_topic").as_string();
    keepout_mask_topic_ = node_->get_parameter("keepout_mask_topic").as_string();
    speed_mask_topic_ = node_->get_parameter("speed_mask_topic").as_string();
    keepout_filter_info_topic_ = node_->get_parameter("keepout_filter_info_topic").as_string();
    speed_filter_info_topic_ = node_->get_parameter("speed_filter_info_topic").as_string();
    speed_base_ = node_->get_parameter("speed_base").as_double();
    speed_multiplier_ = node_->get_parameter("speed_multiplier").as_double();
    virtual_wall_width_ = node_->get_parameter("virtual_wall_width").as_double();
    const double status_period = node_->get_parameter("publish_status_period_sec").as_double();

    auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

    map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, latched_qos, std::bind(&AreaFilterAdapter::onMap, this, std::placeholders::_1));
    areas_sub_ = node_->create_subscription<nav_interfaces::msg::AreaInfoArray>(
        areas_topic_, latched_qos, std::bind(&AreaFilterAdapter::onAreas, this, std::placeholders::_1));
    map_state_sub_ = node_->create_subscription<nav_interfaces::msg::MapState>(
        map_state_topic_, latched_qos, std::bind(&AreaFilterAdapter::onMapState, this, std::placeholders::_1));

    keepout_mask_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(keepout_mask_topic_, latched_qos);
    speed_mask_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(speed_mask_topic_, latched_qos);
    keepout_filter_info_pub_ =
        node_->create_publisher<nav2_msgs::msg::CostmapFilterInfo>(keepout_filter_info_topic_, latched_qos);
    speed_filter_info_pub_ =
        node_->create_publisher<nav2_msgs::msg::CostmapFilterInfo>(speed_filter_info_topic_, latched_qos);
    status_pub_ = node_->create_publisher<std_msgs::msg::String>("~/status", 10);

    status_timer_ = node_->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(status_period)),
        std::bind(&AreaFilterAdapter::onStatusTimer, this));

    RCLCPP_INFO(node_->get_logger(), "AreaFilterAdapter started");
    RCLCPP_INFO(node_->get_logger(), "  map_topic: %s", map_topic_.c_str());
    RCLCPP_INFO(node_->get_logger(), "  areas_topic: %s", areas_topic_.c_str());
    RCLCPP_INFO(node_->get_logger(), "  map_state_topic: %s", map_state_topic_.c_str());
}

// ──────────────────────────────────────────────────────────────────────
// 输入回调
// ──────────────────────────────────────────────────────────────────────

void AreaFilterAdapter::onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    latest_map_ = *msg;
    has_map_ = true;

    if (!has_areas_) {
        state_ = LifecycleState::WAITING_AREAS;
        return;
    }

    renderAndPublish();
}

void AreaFilterAdapter::onAreas(const nav_interfaces::msg::AreaInfoArray::SharedPtr msg) {
    latest_areas_ = *msg;
    has_areas_ = true;

    if (!has_map_) {
        state_ = LifecycleState::WAITING_MAP;
        return;
    }

    renderAndPublish();
}

void AreaFilterAdapter::onMapState(const nav_interfaces::msg::MapState::SharedPtr msg) {
    if (msg->map_name.empty()) {
        return;
    }
    // 按 epoch 比对（不依赖名字）：epoch 变即地图变更，含同名重载/回退，根治同名漏检
    if (msg->epoch == last_epoch_) {
        return;
    }

    const bool first = (last_epoch_ == 0);
    last_epoch_ = msg->epoch;
    current_map_name_ = msg->map_name;

    if (first) {
        // 首次：仅记录并在就绪时渲染
        if (readyToRender()) {
            renderAndPublish();
        }
        return;
    }

    // epoch 变（含同名）→ 标记 STALE 并清旧 mask，等待新 map/areas 重渲染
    state_ = LifecycleState::STALE;
    const bool had_map = has_map_;

    if (had_map) {
        clearMasksFromMap(latest_map_);
        publishFilterInfo();
        keepout_mask_pub_->publish(keepout_mask_);
        speed_mask_pub_->publish(speed_mask_);
        last_stats_ = RenderStats{};
    }

    has_last_render_hash_ = false;
    has_map_ = false;
    has_areas_ = false;
}

// ──────────────────────────────────────────────────────────────────────
// 状态发布与 Filter 输出
// ──────────────────────────────────────────────────────────────────────

void AreaFilterAdapter::onStatusTimer() {
    std_msgs::msg::String msg;
    std::ostringstream oss;
    oss << "state=" << stateToString() << " map=" << current_map_name_ << " areas=" << last_stats_.total_areas
        << " keepout=" << last_stats_.keepout_areas << " speed=" << last_stats_.speed_areas
        << " ignored=" << last_stats_.ignored_areas;
    const std::string breakdown = formatIgnoredBreakdown(last_stats_);
    if (!breakdown.empty()) {
        oss << " ignored_by_type=" << breakdown;
    }
    msg.data = oss.str();
    status_pub_->publish(msg);
}

// 基于最新 map 与 areas 全量重建 mask，并只在输入 hash 变化时发布。
// 这样 map_asset_manager 周期发布相同区域时，不会反复触发 Nav2 filter 更新。
void AreaFilterAdapter::renderAndPublish() {
    if (!readyToRender()) {
        return;
    }

    const size_t render_hash = buildRenderHash();
    if (has_last_render_hash_ && render_hash == last_render_hash_) {
        // Nav2 costmap filters treat every latched update as a real replacement.
        // Avoid re-publishing when map geometry and area assets are unchanged.
        state_ = LifecycleState::ACTIVE;
        return;
    }

    clearMasksFromMap(latest_map_);
    last_stats_ = renderAreas();
    publishFilterInfo();
    keepout_mask_pub_->publish(keepout_mask_);
    speed_mask_pub_->publish(speed_mask_);
    const std::string breakdown = formatIgnoredBreakdown(last_stats_);
    RCLCPP_INFO(
        node_->get_logger(), "Area filters updated: map='%s' areas=%zu keepout=%zu speed=%zu ignored=%zu%s%s hash=%zu",
        current_map_name_.c_str(), last_stats_.total_areas, last_stats_.keepout_areas, last_stats_.speed_areas,
        last_stats_.ignored_areas, breakdown.empty() ? "" : " ignored_by_type=", breakdown.c_str(), render_hash);
    state_ = LifecycleState::ACTIVE;
    last_render_hash_ = render_hash;
    has_last_render_hash_ = true;
}

void AreaFilterAdapter::publishFilterInfo() {
    if (!has_map_) {
        return;
    }

    const auto stamp = node_->now();

    nav2_msgs::msg::CostmapFilterInfo keepout_info;
    keepout_info.header.stamp = stamp;
    keepout_info.header.frame_id = latest_map_.header.frame_id;
    keepout_info.type = kKeepoutFilterType;
    keepout_info.filter_mask_topic = keepout_mask_topic_;
    keepout_info.base = 0.0;
    keepout_info.multiplier = 1.0;
    keepout_filter_info_pub_->publish(keepout_info);

    nav2_msgs::msg::CostmapFilterInfo speed_info;
    speed_info.header.stamp = stamp;
    speed_info.header.frame_id = latest_map_.header.frame_id;
    speed_info.type = kSpeedFilterAbsoluteType;
    speed_info.filter_mask_topic = speed_mask_topic_;
    speed_info.base = speed_base_;
    speed_info.multiplier = speed_multiplier_;
    speed_filter_info_pub_->publish(speed_info);
}

bool AreaFilterAdapter::readyToRender() const { return has_map_ && has_areas_; }

void AreaFilterAdapter::clearMasksFromMap(const nav_msgs::msg::OccupancyGrid& map) {
    keepout_mask_ = map;
    speed_mask_ = map;
    keepout_mask_.data.assign(static_cast<size_t>(keepout_mask_.info.width) * keepout_mask_.info.height,
                              kOccupancyFree);
    speed_mask_.data.assign(static_cast<size_t>(speed_mask_.info.width) * speed_mask_.info.height, kOccupancyFree);
    speed_limits_grid_.assign(speed_mask_.data.size(), std::numeric_limits<float>::infinity());

    const auto stamp = node_->now();
    keepout_mask_.header.stamp = stamp;
    speed_mask_.header.stamp = stamp;
}

// ──────────────────────────────────────────────────────────────────────
// 区域分类与栅格化
// ──────────────────────────────────────────────────────────────────────

// 按 AreaInfo.type 分类区域，将 keepout 和 speed 区域分别写入对应 mask。
// 显式枚举 AreaInfo 当前已知的 9 个类型常量：
//   - VIRTUAL_WALL / RESTRICTED_AREA -> keepout mask
//   - SPEED_LIMIT_AREA               -> speed mask
//   - DETOUR_AREA / STOP_OBSTACLE_AREA / PARKING_AREA / CHARGING_AREA /
//     ELEVATOR_AREA / BUMPY_AREA     -> 本层不映射 Nav2 costmap filter，
//                                       仅按类型计数，由其它适配层消费。
// default 分支用于上游 enum 演进时落入未知类型的兜底，保持可观测性。
AreaFilterAdapter::RenderStats AreaFilterAdapter::renderAreas() {
    RenderStats stats;
    stats.total_areas = latest_areas_.areas.size();

    auto markIgnored = [&](uint32_t area_type) {
        ++stats.ignored_areas;
        const size_t index = (area_type < kIgnoredByTypeSize) ? static_cast<size_t>(area_type) : 0U;
        ++stats.ignored_by_type[index];
    };

    for (const auto& area : latest_areas_.areas) {
        switch (area.type) {
            case nav_interfaces::msg::AreaInfo::VIRTUAL_WALL:
            case nav_interfaces::msg::AreaInfo::RESTRICTED_AREA:
                if (renderKeepoutArea(area)) {
                    ++stats.keepout_areas;
                } else {
                    markIgnored(area.type);
                }
                break;
            case nav_interfaces::msg::AreaInfo::SPEED_LIMIT_AREA:
                if (renderSpeedArea(area)) {
                    ++stats.speed_areas;
                } else {
                    markIgnored(area.type);
                }
                break;
            // 已知但不属于 Nav2 costmap filter 范畴的类型：明确计数，不静默吞掉。
            case nav_interfaces::msg::AreaInfo::DETOUR_AREA:
            case nav_interfaces::msg::AreaInfo::STOP_OBSTACLE_AREA:
            case nav_interfaces::msg::AreaInfo::PARKING_AREA:
            case nav_interfaces::msg::AreaInfo::CHARGING_AREA:
            case nav_interfaces::msg::AreaInfo::ELEVATOR_AREA:
            case nav_interfaces::msg::AreaInfo::BUMPY_AREA:
                markIgnored(area.type);
                break;
            default:
                // 未知类型（上游 AreaInfo 新增常量但本层未跟进），落入 ignored_by_type[0]。
                RCLCPP_WARN(node_->get_logger(), "Unknown area type %u for area '%s', counted as unknown ignored",
                            area.type, area.name.c_str());
                markIgnored(0U);
                break;
        }
    }

    return stats;
}

bool AreaFilterAdapter::renderKeepoutArea(const nav_interfaces::msg::AreaInfo& area) {
    const auto points = toPoints(area.polygon);

    if (area.type == nav_interfaces::msg::AreaInfo::VIRTUAL_WALL) {
        if (points.size() < 2U) {
            RCLCPP_WARN(node_->get_logger(), "Ignore virtual wall '%s': need at least 2 points", area.name.c_str());
            return false;
        }
        return rasterizeVirtualWall(points);
    }

    if (points.size() < 3U) {
        RCLCPP_WARN(node_->get_logger(), "Ignore restricted area '%s': need at least 3 points", area.name.c_str());
        return false;
    }
    return rasterizeRestrictedPolygon(points);
}

bool AreaFilterAdapter::renderSpeedArea(const nav_interfaces::msg::AreaInfo& area) {
    if (area.max_speed <= 0.0f) {
        RCLCPP_WARN(node_->get_logger(), "Ignore speed area '%s': max_speed must be > 0", area.name.c_str());
        return false;
    }

    const auto points = toPoints(area.polygon);
    if (points.size() < 3U) {
        RCLCPP_WARN(node_->get_logger(), "Ignore speed area '%s': need at least 3 points", area.name.c_str());
        return false;
    }
    return rasterizeSpeedPolygon(points, area.max_speed);
}

bool AreaFilterAdapter::rasterizeRestrictedPolygon(const std::vector<geometry_msgs::msg::Point>& polygon) {
    fillPolygonCells(polygon, keepout_mask_.info, [this](int mx, int my) { setKeepoutCell(mx, my); });
    return true;
}

bool AreaFilterAdapter::rasterizeVirtualWall(const std::vector<geometry_msgs::msg::Point>& points) {
    const auto& info = keepout_mask_.info;
    const double resolution = static_cast<double>(info.resolution);
    if (resolution <= 0.0) {
        return false;
    }
    const double radius = std::max(virtual_wall_width_, resolution) * 0.5;

    for (size_t i = 0; i + 1 < points.size(); ++i) {
        const auto& p1 = points[i];
        const auto& p2 = points[i + 1];
        const double length = std::hypot(p2.x - p1.x, p2.y - p1.y);
        const int steps = std::max(1, static_cast<int>(std::ceil(length / (resolution * 0.5))));
        int last_mx = std::numeric_limits<int>::min();
        int last_my = std::numeric_limits<int>::min();

        for (int step = 0; step <= steps; ++step) {
            const double ratio = static_cast<double>(step) / static_cast<double>(steps);
            const double wx = p1.x + (p2.x - p1.x) * ratio;
            const double wy = p1.y + (p2.y - p1.y) * ratio;
            int mx;
            int my;
            if (!worldToMap(wx, wy, &mx, &my)) {
                continue;
            }
            if (mx == last_mx && my == last_my) {
                continue;
            }
            inflateKeepoutCell(mx, my, radius);
            last_mx = mx;
            last_my = my;
        }
    }

    return true;
}

bool AreaFilterAdapter::rasterizeSpeedPolygon(const std::vector<geometry_msgs::msg::Point>& polygon,
                                              float speed_limit) {
    const int8_t encoded = encodeSpeedLimit(speed_limit);
    if (encoded <= 0) {
        RCLCPP_WARN(node_->get_logger(),
                    "Ignore speed area: speed %.3f can not be encoded with base %.3f multiplier %.4f", speed_limit,
                    speed_base_, speed_multiplier_);
        return false;
    }

    fillPolygonCells(polygon, speed_mask_.info,
                     [this, speed_limit, encoded](int mx, int my) { setSpeedCell(mx, my, speed_limit, encoded); });

    return true;
}

// ──────────────────────────────────────────────────────────────────────
// 几何与消息转换工具
// ──────────────────────────────────────────────────────────────────────

bool AreaFilterAdapter::computeBounds(const std::vector<geometry_msgs::msg::Point>& points,
                                      double* min_x,
                                      double* min_y,
                                      double* max_x,
                                      double* max_y) const {
    if (points.empty()) {
        return false;
    }

    *min_x = std::numeric_limits<double>::max();
    *min_y = std::numeric_limits<double>::max();
    *max_x = std::numeric_limits<double>::lowest();
    *max_y = std::numeric_limits<double>::lowest();

    for (const auto& point : points) {
        *min_x = std::min(*min_x, point.x);
        *min_y = std::min(*min_y, point.y);
        *max_x = std::max(*max_x, point.x);
        *max_y = std::max(*max_y, point.y);
    }
    return true;
}

bool AreaFilterAdapter::worldToMap(double wx, double wy, int* mx, int* my) const {
    const auto& info = keepout_mask_.info;
    if (info.resolution <= 0.0f) {
        return false;
    }

    const int x = static_cast<int>(std::floor((wx - info.origin.position.x) / info.resolution));
    const int y = static_cast<int>(std::floor((wy - info.origin.position.y) / info.resolution));
    if (x < 0 || y < 0 || x >= static_cast<int>(info.width) || y >= static_cast<int>(info.height)) {
        return false;
    }

    *mx = x;
    *my = y;
    return true;
}

void AreaFilterAdapter::setKeepoutCell(int mx, int my) {
    const auto& info = keepout_mask_.info;
    if (mx < 0 || my < 0 || mx >= static_cast<int>(info.width) || my >= static_cast<int>(info.height)) {
        return;
    }
    keepout_mask_.data[static_cast<size_t>(my) * info.width + static_cast<size_t>(mx)] = kOccupancyKeepout;
}

void AreaFilterAdapter::setSpeedCell(int mx, int my, float speed_limit, int8_t encoded) {
    const auto& info = speed_mask_.info;
    if (mx < 0 || my < 0 || mx >= static_cast<int>(info.width) || my >= static_cast<int>(info.height)) {
        return;
    }

    const size_t index = static_cast<size_t>(my) * info.width + static_cast<size_t>(mx);
    if (speed_limit < speed_limits_grid_[index]) {
        speed_limits_grid_[index] = speed_limit;
        speed_mask_.data[index] = encoded;
    }
}

void AreaFilterAdapter::fillPolygonCells(const std::vector<geometry_msgs::msg::Point>& polygon,
                                         const nav_msgs::msg::MapMetaData& info,
                                         const std::function<void(int, int)>& fill_cell) const {
    double min_x;
    double min_y;
    double max_x;
    double max_y;
    if (!computeBounds(polygon, &min_x, &min_y, &max_x, &max_y)) {
        return;
    }

    const double resolution = static_cast<double>(info.resolution);
    if (resolution <= 0.0) {
        return;
    }

    const int min_my = std::max(0, static_cast<int>(std::ceil((min_y - info.origin.position.y) / resolution - 0.5)));
    const int max_my = std::min(static_cast<int>(info.height) - 1,
                                static_cast<int>(std::floor((max_y - info.origin.position.y) / resolution - 0.5)));
    if (min_my > max_my) {
        return;
    }

    std::vector<double> intersections;
    intersections.reserve(polygon.size());

    for (int my = min_my; my <= max_my; ++my) {
        const double y = info.origin.position.y + (static_cast<double>(my) + 0.5) * resolution;
        intersections.clear();

        for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
            const auto& p1 = polygon[j];
            const auto& p2 = polygon[i];
            if ((p1.y <= y && p2.y > y) || (p2.y <= y && p1.y > y)) {
                const double ratio = (y - p1.y) / (p2.y - p1.y);
                intersections.push_back(p1.x + ratio * (p2.x - p1.x));
            }
        }

        if (intersections.size() < 2U) {
            continue;
        }

        std::sort(intersections.begin(), intersections.end());
        for (size_t i = 0; i + 1 < intersections.size(); i += 2) {
            const double start_x = intersections[i];
            const double end_x = intersections[i + 1];
            int min_mx = static_cast<int>(std::ceil((start_x - info.origin.position.x) / resolution - 0.5));
            int max_mx = static_cast<int>(std::floor((end_x - info.origin.position.x) / resolution - 0.5));
            min_mx = std::max(0, min_mx);
            max_mx = std::min(static_cast<int>(info.width) - 1, max_mx);

            for (int mx = min_mx; mx <= max_mx; ++mx) {
                fill_cell(mx, my);
            }
        }
    }
}

void AreaFilterAdapter::inflateKeepoutCell(int center_mx, int center_my, double radius) {
    const auto& info = keepout_mask_.info;
    const double resolution = static_cast<double>(info.resolution);
    if (resolution <= 0.0) {
        return;
    }

    const int radius_cells = std::max(0, static_cast<int>(std::ceil(radius / resolution)));
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            const double distance =
                std::hypot(static_cast<double>(dx) * resolution, static_cast<double>(dy) * resolution);
            if (distance <= radius + resolution * 0.5) {
                setKeepoutCell(center_mx + dx, center_my + dy);
            }
        }
    }
}

int8_t AreaFilterAdapter::encodeSpeedLimit(float speed_limit) const {
    if (speed_multiplier_ <= 0.0) {
        return 0;
    }

    const double encoded = std::round((static_cast<double>(speed_limit) - speed_base_) / speed_multiplier_);
    if (encoded < 1.0 || encoded > 100.0) {
        return 0;
    }
    return static_cast<int8_t>(encoded);
}

std::vector<geometry_msgs::msg::Point> AreaFilterAdapter::toPoints(
    const std::vector<geometry_msgs::msg::Pose2D>& polygon) const {
    std::vector<geometry_msgs::msg::Point> points;
    points.reserve(polygon.size());
    for (const auto& pose : polygon) {
        geometry_msgs::msg::Point point;
        point.x = pose.x;
        point.y = pose.y;
        point.z = 0.0;
        points.push_back(point);
    }
    return points;
}

// 构造渲染输入 hash，忽略消息时间戳，只比较地图几何与区域资产内容。
// 浮点字段先量化再参与 hash，避免微小数值抖动触发无意义重建。
// 区域先按名称和类型排序，避免发布顺序变化造成误判。
size_t AreaFilterAdapter::buildRenderHash() const {
    size_t seed = 0U;
    const auto& info = latest_map_.info;
    hashCombine(&seed, latest_map_.header.frame_id);
    hashCombine(&seed, info.width);
    hashCombine(&seed, info.height);
    hashQuantized(&seed, static_cast<double>(info.resolution), kHashMapResolutionResolution);
    hashQuantized(&seed, info.origin.position.x, kHashPositionResolution);
    hashQuantized(&seed, info.origin.position.y, kHashPositionResolution);
    hashQuantized(&seed, info.origin.position.z, kHashPositionResolution);
    hashQuantized(&seed, info.origin.orientation.x, kHashUnitlessResolution);
    hashQuantized(&seed, info.origin.orientation.y, kHashUnitlessResolution);
    hashQuantized(&seed, info.origin.orientation.z, kHashUnitlessResolution);
    hashQuantized(&seed, info.origin.orientation.w, kHashUnitlessResolution);

    std::vector<const nav_interfaces::msg::AreaInfo*> areas;
    areas.reserve(latest_areas_.areas.size());
    for (const auto& area : latest_areas_.areas) {
        areas.push_back(&area);
    }

    std::sort(areas.begin(), areas.end(), [](const auto* lhs, const auto* rhs) {
        if (lhs->name != rhs->name) {
            return lhs->name < rhs->name;
        }
        return lhs->type < rhs->type;
    });

    hashCombine(&seed, areas.size());
    for (const auto* area : areas) {
        hashCombine(&seed, area->name);
        hashCombine(&seed, area->type);
        hashQuantized(&seed, static_cast<double>(area->max_speed), kHashSpeedResolution);
        hashQuantized(&seed, static_cast<double>(area->stop_distance), kHashPositionResolution);
        hashQuantized(&seed, static_cast<double>(area->depth_min_height), kHashPositionResolution);
        hashCombine(&seed, area->manual_linked_point_names.size());
        for (const auto& linked_point_name : area->manual_linked_point_names) {
            hashCombine(&seed, linked_point_name);
        }
        hashCombine(&seed, area->charger_type);
        hashCombine(&seed, area->attribute);
        hashCombine(&seed, area->polygon.size());

        for (const auto& pose : area->polygon) {
            hashPose2D(&seed, pose);
        }
    }

    return seed;
}

std::string AreaFilterAdapter::stateToString() const {
    switch (state_) {
        case LifecycleState::WAITING_MAP:
            return "waiting_map";
        case LifecycleState::WAITING_AREAS:
            return "waiting_areas";
        case LifecycleState::STALE:
            return "stale";
        case LifecycleState::ACTIVE:
            return "active";
        default:
            return "unknown";
    }
}

// 把 ignored_by_type 数组渲染为 "virtual_wall:1,parking:2,unknown:1" 风格的可读字符串。
// 仅输出 count > 0 的条目，便于排障时快速分辨"为什么这些区域被忽略"。
std::string AreaFilterAdapter::formatIgnoredBreakdown(const RenderStats& stats) const {
    std::ostringstream oss;
    bool first = true;
    auto append = [&](const char* label, size_t count) {
        if (count == 0U) {
            return;
        }
        if (!first) {
            oss << ",";
        }
        oss << label << ":" << count;
        first = false;
    };

    using AreaInfo = nav_interfaces::msg::AreaInfo;
    append("virtual_wall", stats.ignored_by_type[AreaInfo::VIRTUAL_WALL]);
    append("restricted", stats.ignored_by_type[AreaInfo::RESTRICTED_AREA]);
    append("speed_limit", stats.ignored_by_type[AreaInfo::SPEED_LIMIT_AREA]);
    append("detour", stats.ignored_by_type[AreaInfo::DETOUR_AREA]);
    append("stop_obstacle", stats.ignored_by_type[AreaInfo::STOP_OBSTACLE_AREA]);
    append("parking", stats.ignored_by_type[AreaInfo::PARKING_AREA]);
    append("charging", stats.ignored_by_type[AreaInfo::CHARGING_AREA]);
    append("elevator", stats.ignored_by_type[AreaInfo::ELEVATOR_AREA]);
    append("bumpy", stats.ignored_by_type[AreaInfo::BUMPY_AREA]);
    append("unknown", stats.ignored_by_type[0]);

    return oss.str();
}

}  // namespace navigation_adapter
