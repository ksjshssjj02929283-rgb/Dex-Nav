#include "map_asset_manager/area_manager/area_manager.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

namespace map_asset_manager {

// ──────────────────────────────────────────────────────────────────────
// 构造与初始化
// ──────────────────────────────────────────────────────────────────────

AreaManager::AreaManager(rclcpp::Node* node,
                         std::shared_ptr<AreaStore> area_store,
                         std::shared_ptr<DataEventBus> event_bus,
                         std::function<void()> save_fn)
    : node_(node),
      area_store_(area_store),
      event_bus_(event_bus),
      save_fn_(std::move(save_fn)) {

    // Publishers
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "~/area_markers", rclcpp::QoS(10).transient_local());

    // Services
    RCLCPP_INFO(node_->get_logger(), "AreaManager: initialized (assets via set_assets)");
}

// ──────────────────────────────────────────────────────────────────────
// Service 回调
// ──────────────────────────────────────────────────────────────────────



// ──────────────────────────────────────────────────────────────────────
// 发布
// ──────────────────────────────────────────────────────────────────────

void AreaManager::publishMarkers() {
    auto all_areas = area_store_->getAllAreas();
    visualization_msgs::msg::MarkerArray arr;

    int new_count = static_cast<int>(all_areas.size()) * 2;

    // 删除多余的旧 marker
    for (int i = new_count; i < prev_marker_count_; ++i) {
        visualization_msgs::msg::Marker del;
        del.header.frame_id = "map";
        del.header.stamp = node_->now();
        del.ns = "areas";
        del.id = i;
        del.action = visualization_msgs::msg::Marker::DELETE;
        arr.markers.push_back(del);
    }
    prev_marker_count_ = new_count;

    int id = 0;
    for (const auto& area : all_areas) {
        auto col = typeColor(area.type);

        // 多边形轮廓（LINE_STRIP）
        visualization_msgs::msg::Marker lm;
        lm.header.frame_id = "map";
        lm.header.stamp = node_->now();
        lm.ns = "areas";
        lm.id = id++;
        lm.type = visualization_msgs::msg::Marker::LINE_STRIP;
        lm.action = visualization_msgs::msg::Marker::ADD;
        lm.scale.x = 0.08;
        lm.color.r = col.r;
        lm.color.g = col.g;
        lm.color.b = col.b;
        lm.color.a = 0.9f;
        for (const auto& pt : area.polygon) {
            geometry_msgs::msg::Point p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = 0.02;
            lm.points.push_back(p);
        }
        // 多边形闭合（虚拟墙 type=VIRTUAL_WALL 是 LINE，不闭合）
        if (!area.polygon.empty() && area.type != AreaType::VIRTUAL_WALL) {
            geometry_msgs::msg::Point pf;
            pf.x = area.polygon[0].x;
            pf.y = area.polygon[0].y;
            pf.z = 0.02;
            lm.points.push_back(pf);
        }
        arr.markers.push_back(lm);

        // 区域名称标签（TEXT_VIEW_FACING）
        visualization_msgs::msg::Marker tm;
        tm.header.frame_id = "map";
        tm.header.stamp = node_->now();
        tm.ns = "areas";
        tm.id = id++;
        tm.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        tm.action = visualization_msgs::msg::Marker::ADD;

        if (!area.polygon.empty()) {
            double cx = 0, cy = 0;
            for (const auto& pt : area.polygon) {
                cx += pt.x;
                cy += pt.y;
            }
            cx /= static_cast<double>(area.polygon.size());
            cy /= static_cast<double>(area.polygon.size());
            tm.pose.position.x = cx;
            tm.pose.position.y = cy;
            tm.pose.position.z = 0.3;
        }
        tm.pose.orientation.w = 1.0;
        tm.scale.z = 0.25;
        tm.color.r = col.r;
        tm.color.g = col.g;
        tm.color.b = col.b;
        tm.color.a = 1.0f;
        tm.text = area.name + "\n" + AreaType::toString(area.type);
        arr.markers.push_back(tm);
    }

    marker_pub_->publish(arr);
}

// ──────────────────────────────────────────────────────────────────────
// 消息转换
// ──────────────────────────────────────────────────────────────────────

Area AreaManager::fromMsg(const nav_interfaces::msg::AreaInfo& msg) {
    Area a;
    a.name              = msg.name;
    a.type              = msg.type;
    a.max_speed         = msg.max_speed;
    a.stop_distance     = msg.stop_distance;
    a.charger_type      = msg.charger_type;
    a.depth_min_height     = msg.depth_min_height;
    a.attribute         = msg.attribute;

    // 关联点（复数，支持多个）
    a.manual_linked_point_names.assign(msg.manual_linked_point_names.begin(),
                                msg.manual_linked_point_names.end());

    a.polygon.reserve(msg.polygon.size());
    for (const auto& pose : msg.polygon) {
        geometry_msgs::msg::Point p;
        p.x = pose.x;
        p.y = pose.y;
        p.z = 0.0;
        a.polygon.push_back(p);
    }

    // 自动归属字段：从 msg 来的视为初始值，SpatialIndexer 会覆盖
    a.related_nav_point_names.assign(msg.related_nav_point_names.begin(),
                                  msg.related_nav_point_names.end());
    a.related_junction_ids.assign(msg.related_junction_ids.begin(),
                               msg.related_junction_ids.end());
    return a;
}

nav_interfaces::msg::AreaInfo AreaManager::toMsg(const Area& area) {
    nav_interfaces::msg::AreaInfo msg;
    msg.name              = area.name;
    msg.type              = area.type;
    msg.max_speed         = area.max_speed;
    msg.stop_distance     = area.stop_distance;
    msg.charger_type      = area.charger_type;
    msg.depth_min_height     = area.depth_min_height;
    msg.attribute         = area.attribute;

    msg.manual_linked_point_names.assign(area.manual_linked_point_names.begin(),
                                  area.manual_linked_point_names.end());

    msg.polygon.reserve(area.polygon.size());
    for (const auto& pt : area.polygon) {
        geometry_msgs::msg::Pose2D pose;
        pose.x     = pt.x;
        pose.y     = pt.y;
        pose.theta = 0.0;
        msg.polygon.push_back(pose);
    }

    // 自动归属（只读字段，SpatialIndexer 维护）
    msg.related_nav_point_names.assign(area.related_nav_point_names.begin(),
                                    area.related_nav_point_names.end());
    msg.related_junction_ids.assign(area.related_junction_ids.begin(),
                                 area.related_junction_ids.end());
    return msg;
}

// ──────────────────────────────────────────────────────────────────────
// 颜色映射
// ──────────────────────────────────────────────────────────────────────

AreaManager::Color AreaManager::typeColor(uint32_t type) {
    switch (type) {
        case AreaType::VIRTUAL_WALL:       return {0.5f, 0.5f, 0.5f};  // 灰黑
        case AreaType::RESTRICTED_AREA:    return {1.0f, 0.0f, 0.0f};  // 红
        case AreaType::SPEED_LIMIT_AREA:   return {1.0f, 0.8f, 0.0f};  // 黄
        case AreaType::DETOUR_AREA:        return {0.0f, 0.9f, 0.9f};  // 青
        case AreaType::STOP_OBSTACLE_AREA: return {1.0f, 0.5f, 0.0f};  // 橙
        case AreaType::PARKING_AREA:       return {0.0f, 0.8f, 0.0f};  // 绿
        case AreaType::CHARGING_AREA:      return {0.2f, 0.5f, 1.0f};  // 蓝
        case AreaType::ELEVATOR_AREA:      return {0.7f, 0.3f, 1.0f};  // 紫
        case AreaType::BUMPY_AREA:         return {0.9f, 0.5f, 0.1f};  // 深橙
        default:                           return {0.6f, 0.6f, 0.6f};  // 灰
    }
}

}  // namespace map_asset_manager
