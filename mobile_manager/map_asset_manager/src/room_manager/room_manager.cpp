#include "map_asset_manager/room_manager/room_manager.hpp"

#include <algorithm>
#include <cmath>

#include "map_asset_manager/polygon_utils.hpp"

namespace map_asset_manager {

// ──────────────────────────────────────────────────────────────────────
// 构造与初始化
// ──────────────────────────────────────────────────────────────────────

RoomManager::RoomManager(rclcpp::Node* node,
                         std::shared_ptr<RoomStore> room_store,
                         std::shared_ptr<DataEventBus> event_bus,
                         std::function<void()> save_fn)
    : node_(node),
      room_store_(room_store),
      event_bus_(event_bus),
      save_fn_(std::move(save_fn)) {

    // Publishers
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "~/room_markers", rclcpp::QoS(10).transient_local());

    // Services
    RCLCPP_INFO(node_->get_logger(),
                "RoomManager: initialized (assets via set_assets)");
}

// ──────────────────────────────────────────────────────────────────────
// Service 回调
// ──────────────────────────────────────────────────────────────────────



// ──────────────────────────────────────────────────────────────────────
// 发布
// ──────────────────────────────────────────────────────────────────────

void RoomManager::publishMarkers() {
    auto all_rooms = room_store_->getAllRooms();
    visualization_msgs::msg::MarkerArray arr;

    // 每个 room 生成 3 个 marker：LINE_STRIP（轮廓）+ TRIANGLE_LIST（半透明填充）+ TEXT
    int new_count = static_cast<int>(all_rooms.size()) * 3;

    for (int i = new_count; i < prev_marker_count_; ++i) {
        visualization_msgs::msg::Marker del;
        del.header.frame_id = "map";
        del.header.stamp = node_->now();
        del.ns = "rooms";
        del.id = i;
        del.action = visualization_msgs::msg::Marker::DELETE;
        arr.markers.push_back(del);
    }
    prev_marker_count_ = new_count;

    int id = 0;
    for (const auto& room : all_rooms) {
        auto col = typeColor(room.type);

        // 1) 多边形轮廓
        visualization_msgs::msg::Marker lm;
        lm.header.frame_id = "map";
        lm.header.stamp = node_->now();
        lm.ns = "rooms";
        lm.id = id++;
        lm.type = visualization_msgs::msg::Marker::LINE_STRIP;
        lm.action = visualization_msgs::msg::Marker::ADD;
        lm.scale.x = 0.06;
        lm.color.r = col.r;
        lm.color.g = col.g;
        lm.color.b = col.b;
        lm.color.a = 0.95f;
        lm.pose.orientation.w = 1.0;
        for (const auto& pt : room.polygon) {
            geometry_msgs::msg::Point p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = 0.04;
            lm.points.push_back(p);
        }
        if (!room.polygon.empty()) {
            geometry_msgs::msg::Point pf;
            pf.x = room.polygon[0].x;
            pf.y = room.polygon[0].y;
            pf.z = 0.04;
            lm.points.push_back(pf);
        }
        arr.markers.push_back(lm);

        // 2) 半透明填充（TRIANGLE_LIST，扇形三角化，凸多边形足够）
        visualization_msgs::msg::Marker fm;
        fm.header.frame_id = "map";
        fm.header.stamp = node_->now();
        fm.ns = "rooms";
        fm.id = id++;
        fm.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        fm.action = visualization_msgs::msg::Marker::ADD;
        fm.scale.x = 1.0;
        fm.scale.y = 1.0;
        fm.scale.z = 1.0;
        fm.color.r = col.r;
        fm.color.g = col.g;
        fm.color.b = col.b;
        fm.color.a = 0.15f;
        fm.pose.orientation.w = 1.0;
        if (room.polygon.size() >= 3) {
            for (size_t i2 = 1; i2 + 1 < room.polygon.size(); ++i2) {
                geometry_msgs::msg::Point a, b, c;
                a.x = room.polygon[0].x;       a.y = room.polygon[0].y;       a.z = 0.02;
                b.x = room.polygon[i2].x;      b.y = room.polygon[i2].y;      b.z = 0.02;
                c.x = room.polygon[i2 + 1].x;  c.y = room.polygon[i2 + 1].y;  c.z = 0.02;
                fm.points.push_back(a);
                fm.points.push_back(b);
                fm.points.push_back(c);
            }
        }
        arr.markers.push_back(fm);

        // 3) 名称标签
        visualization_msgs::msg::Marker tm;
        tm.header.frame_id = "map";
        tm.header.stamp = node_->now();
        tm.ns = "rooms";
        tm.id = id++;
        tm.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        tm.action = visualization_msgs::msg::Marker::ADD;

        const auto centroid = polygon_utils::polygonCentroid(room.polygon);
        tm.pose.position.x = centroid.x;
        tm.pose.position.y = centroid.y;
        tm.pose.position.z = 0.5;
        tm.pose.orientation.w = 1.0;
        tm.scale.z = 0.3;
        tm.color.r = col.r;
        tm.color.g = col.g;
        tm.color.b = col.b;
        tm.color.a = 1.0f;
        const std::string label = room.display_label.empty() ? room.name : room.display_label;
        tm.text = label + "\n" + RoomType::toString(room.type);
        arr.markers.push_back(tm);
    }

    marker_pub_->publish(arr);
}

// ──────────────────────────────────────────────────────────────────────
// 消息转换 + 字段补全
// ──────────────────────────────────────────────────────────────────────

Room RoomManager::fromMsg(const nav_interfaces::msg::RoomInfo& msg) {
    Room r;
    r.id                   = msg.id;
    r.source               = msg.source;
    r.created_at_ns        = rclcpp::Time(msg.created_at).nanoseconds();
    r.updated_at_ns        = rclcpp::Time(msg.updated_at).nanoseconds();

    r.name                 = msg.name;
    r.type                 = msg.type;
    r.semantic_label       = msg.semantic_label;
    r.display_label        = msg.display_label;

    r.pose                 = msg.pose;
    r.polygon.reserve(msg.polygon.size());
    for (const auto& pose : msg.polygon) {
        geometry_msgs::msg::Point p;
        p.x = pose.x;
        p.y = pose.y;
        p.z = 0.0;
        r.polygon.push_back(p);
    }
    for (size_t i = 0; i < 6; ++i) r.bounding_box[i] = msg.bounding_box[i];

    // 行为约束（Room 内嵌约束子集，对应 Area 部分功能）
    r.behavior.max_speed_m_s        = msg.behavior.max_speed_m_s;
    r.behavior.restricted           = msg.behavior.restricted;
    r.behavior.detour_enabled       = msg.behavior.detour_enabled;
    r.behavior.stop_distance_m      = msg.behavior.stop_distance_m;
    r.behavior.depth_min_height_m   = msg.behavior.depth_min_height_m;
    r.behavior.additional_area_names.assign(msg.behavior.additional_area_names.begin(),
                                             msg.behavior.additional_area_names.end());

    // 锚点（复数，支持多个）
    r.manual_anchor_point_names.assign(msg.manual_anchor_point_names.begin(),
                                msg.manual_anchor_point_names.end());

    r.attribute            = msg.attribute;
    r.external_dsg_node_id = msg.external_dsg_node_id;

    // 自动归属字段（业务侧传值视为初始，SpatialIndexer 会覆盖）
    r.related_nav_point_names.assign(msg.related_nav_point_names.begin(),
                                  msg.related_nav_point_names.end());
    r.related_junction_ids.assign(msg.related_junction_ids.begin(),
                               msg.related_junction_ids.end());
    return r;
}

nav_interfaces::msg::RoomInfo RoomManager::toMsg(const Room& room) {
    nav_interfaces::msg::RoomInfo msg;
    msg.id                   = room.id;
    msg.source               = room.source;
    msg.created_at           = rclcpp::Time(room.created_at_ns);
    msg.updated_at           = rclcpp::Time(room.updated_at_ns);

    msg.name                 = room.name;
    msg.type                 = room.type;
    msg.semantic_label       = room.semantic_label;
    msg.display_label        = room.display_label;

    msg.pose                 = room.pose;
    msg.polygon.reserve(room.polygon.size());
    for (const auto& pt : room.polygon) {
        geometry_msgs::msg::Pose2D pose;
        pose.x = pt.x;
        pose.y = pt.y;
        pose.theta = 0.0;
        msg.polygon.push_back(pose);
    }
    for (size_t i = 0; i < 6; ++i) msg.bounding_box[i] = room.bounding_box[i];

    // 行为约束（内嵌约束子集）
    msg.behavior.max_speed_m_s        = room.behavior.max_speed_m_s;
    msg.behavior.restricted           = room.behavior.restricted;
    msg.behavior.detour_enabled       = room.behavior.detour_enabled;
    msg.behavior.stop_distance_m      = room.behavior.stop_distance_m;
    msg.behavior.depth_min_height_m   = room.behavior.depth_min_height_m;
    msg.behavior.additional_area_names.assign(room.behavior.additional_area_names.begin(),
                                               room.behavior.additional_area_names.end());

    msg.manual_anchor_point_names.assign(room.manual_anchor_point_names.begin(),
                                  room.manual_anchor_point_names.end());

    msg.attribute            = room.attribute;
    msg.external_dsg_node_id = room.external_dsg_node_id;

    // 自动归属（只读字段，SpatialIndexer 维护）
    msg.related_nav_point_names.assign(room.related_nav_point_names.begin(),
                                    room.related_nav_point_names.end());
    msg.related_junction_ids.assign(room.related_junction_ids.begin(),
                                 room.related_junction_ids.end());
    return msg;
}

Room RoomManager::enrich(Room room, const std::string& source, const rclcpp::Time& now) {
    room.source = source.empty() ? std::string(DSGSource::MANUAL) : source;

    if (room.created_at_ns == 0) room.created_at_ns = now.nanoseconds();
    room.updated_at_ns = now.nanoseconds();

    room.id = buildId(room.name, room.source);

    // bounding_box（从 polygon 推导）
    if (!room.polygon.empty()) {
        const auto aabb = polygon_utils::polygonBoundingBox(room.polygon);
        room.bounding_box = {aabb.min_x, aabb.min_y, aabb.min_z,
                             aabb.max_x, aabb.max_y, aabb.max_z};
    }

    // pose（centroid + 默认朝向）：业务侧未填时自动推导
    const bool pose_empty = (room.pose.position.x == 0.0 && room.pose.position.y == 0.0 &&
                             room.pose.position.z == 0.0 &&
                             room.pose.orientation.x == 0.0 &&
                             room.pose.orientation.y == 0.0 &&
                             room.pose.orientation.z == 0.0 &&
                             room.pose.orientation.w == 0.0);
    if (pose_empty && !room.polygon.empty()) {
        const auto c = polygon_utils::polygonCentroid(room.polygon);
        room.pose.position.x = c.x;
        room.pose.position.y = c.y;
        room.pose.position.z = 0.0;
        room.pose.orientation.w = 1.0;
    } else if (room.pose.orientation.x == 0.0 && room.pose.orientation.y == 0.0 &&
               room.pose.orientation.z == 0.0 && room.pose.orientation.w == 0.0) {
        // pose 已填位置但四元数全 0，补单位四元数
        room.pose.orientation.w = 1.0;
    }

    return room;
}

std::string RoomManager::sanitizeIdSegment(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (char c : s) {
        if (std::isalnum(static_cast<unsigned char>(c))) {
            out.push_back(c);
        } else if (c == '_' || c == '-' || c == '.') {
            out.push_back(c);
        } else {
            out.push_back('_');
        }
    }
    if (out.empty()) out = "unnamed";
    return out;
}

std::string RoomManager::buildId(const std::string& name, const std::string& source) {
    return std::string("room:") + sanitizeIdSegment(name) + ":" +
           sanitizeIdSegment(source.empty() ? std::string(DSGSource::MANUAL) : source);
}

// ──────────────────────────────────────────────────────────────────────
// 颜色映射
// ──────────────────────────────────────────────────────────────────────

RoomManager::Color RoomManager::typeColor(uint32_t type) {
    switch (type) {
        case RoomType::MEETING_ROOM: return {0.20f, 0.55f, 0.95f};  // 蓝
        case RoomType::PANTRY:       return {0.95f, 0.75f, 0.25f};  // 黄
        case RoomType::CAFE:         return {0.65f, 0.45f, 0.25f};  // 棕
        case RoomType::CORRIDOR:     return {0.70f, 0.70f, 0.70f};  // 灰
        case RoomType::OFFICE:       return {0.30f, 0.80f, 0.40f};  // 绿
        case RoomType::LOBBY:        return {0.95f, 0.55f, 0.30f};  // 橙
        case RoomType::RESTROOM:     return {0.80f, 0.30f, 0.60f};  // 粉紫
        case RoomType::STORAGE:      return {0.50f, 0.50f, 0.30f};  // 橄榄
        default:                     return {0.60f, 0.60f, 0.60f};  // 灰
    }
}

}  // namespace map_asset_manager
