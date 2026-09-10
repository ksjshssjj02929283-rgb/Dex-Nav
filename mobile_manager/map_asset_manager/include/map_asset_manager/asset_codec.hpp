#ifndef MAP_ASSET_MANAGER__ASSET_CODEC_HPP_
#define MAP_ASSET_MANAGER__ASSET_CODEC_HPP_
// 六类资产 内部 struct ↔ nav_interfaces msg 的转换（header-only）。
// 图相关的 6 个函数原为 route_graph_manager.cpp 的 static，收口到此供 GetAssets / SetAssets / AssetSnapshot 复用。
// Area / Room 的转换仍由 AreaManager::toMsg/fromMsg、RoomManager::toMsg/fromMsg 提供（已改为 public static）。

#include <cmath>
#include <limits>
#include <optional>

#include <geometry_msgs/msg/point.hpp>
#include <nav_interfaces/msg/nav_point.hpp>
#include <nav_interfaces/msg/route.hpp>
#include <nav_interfaces/msg/route_edge.hpp>
#include <nav_interfaces/msg/route_endpoint.hpp>
#include <nav_interfaces/msg/route_junction.hpp>

#include "map_asset_manager/types.hpp"

namespace map_asset_manager {

// ---------- NavPoint ----------
inline nav_interfaces::msg::NavPoint toMsg(const NavPoint& pt) {
    nav_interfaces::msg::NavPoint msg;
    msg.name      = pt.name;
    msg.type      = pt.type;
    msg.x         = static_cast<float>(pt.x);
    msg.y         = static_cast<float>(pt.y);
    msg.theta     = static_cast<float>(pt.yaw);
    msg.tolerance = static_cast<float>(pt.tolerance);
    return msg;
}
inline NavPoint fromMsg(const nav_interfaces::msg::NavPoint& msg) {
    NavPoint pt;
    pt.name      = msg.name;
    pt.type      = msg.type;
    pt.x         = msg.x;
    pt.y         = msg.y;
    pt.yaw       = msg.theta;
    pt.tolerance = msg.tolerance;
    return pt;
}

// ---------- Route（命名路线）----------
inline nav_interfaces::msg::Route toMsg(const Route& r) {
    nav_interfaces::msg::Route msg;
    msg.name        = r.name;
    msg.point_names = r.point_names;
    return msg;
}
inline Route fromMsg(const nav_interfaces::msg::Route& msg) {
    Route r;
    r.name        = msg.name;
    r.point_names = msg.point_names;
    return r;
}

// ---------- RouteGraph ----------
inline nav_interfaces::msg::RouteEndpoint toMsg(const RouteEndpoint& ep) {
    nav_interfaces::msg::RouteEndpoint msg;
    msg.id   = ep.id;
    msg.type = ep.type;
    return msg;
}

inline nav_interfaces::msg::RouteJunction toMsg(const RouteJunction& jct) {
    nav_interfaces::msg::RouteJunction msg;
    msg.id  = jct.id;
    msg.x   = jct.x;
    msg.y   = jct.y;
    msg.yaw = jct.yaw;
    return msg;
}

inline nav_interfaces::msg::RouteEdge toMsg(const RouteEdge& e) {
    nav_interfaces::msg::RouteEdge msg;
    msg.id            = e.id;
    msg.from_ep       = toMsg(e.from);
    msg.to_ep         = toMsg(e.to);
    msg.edge_type     = e.edge_type;
    msg.bidirectional = e.bidirectional;
    msg.max_speed     = e.max_speed;
    msg.xy_tolerance  = e.xy_tolerance;
    msg.th_tolerance  = e.th_tolerance;
    msg.lane_width    = e.lane_width;
    msg.obs_policy    = e.obs_policy;
    for (const auto& cp : e.control_points) {
        geometry_msgs::msg::Point p;
        p.x = cp.first;
        p.y = cp.second;
        p.z = 0.0;
        msg.control_points.push_back(p);
    }
    msg.hold_direction = e.hold_direction.value_or(std::numeric_limits<float>::quiet_NaN());
    msg.backward = e.backward;
    msg.padding = e.padding.value_or(std::numeric_limits<float>::quiet_NaN());
    msg.mode = e.mode;
    return msg;
}

inline RouteEndpoint fromMsg(const nav_interfaces::msg::RouteEndpoint& msg) {
    return {msg.id, msg.type};
}

inline RouteJunction fromMsg(const nav_interfaces::msg::RouteJunction& msg) {
    RouteJunction jct;
    jct.id  = msg.id;
    jct.x   = msg.x;
    jct.y   = msg.y;
    jct.yaw = msg.yaw;
    return jct;
}

inline RouteEdge fromMsg(const nav_interfaces::msg::RouteEdge& msg) {
    RouteEdge e;
    e.id           = msg.id;
    e.from         = fromMsg(msg.from_ep);
    e.to           = fromMsg(msg.to_ep);
    e.edge_type    = msg.edge_type;
    e.bidirectional= msg.bidirectional;
    e.max_speed    = msg.max_speed;
    e.xy_tolerance = msg.xy_tolerance;
    e.th_tolerance = msg.th_tolerance;
    e.lane_width   = msg.lane_width;
    e.obs_policy   = msg.obs_policy;
    for (const auto& cp : msg.control_points) {
        e.control_points.emplace_back(static_cast<float>(cp.x),
                                      static_cast<float>(cp.y));
    }
    e.hold_direction = std::isnan(msg.hold_direction)
                           ? std::optional<float>{}
                           : std::optional<float>(static_cast<float>(msg.hold_direction));
    e.backward = msg.backward;
    e.padding = std::isnan(msg.padding) ? std::optional<float>{} : std::optional<float>(msg.padding);
    e.mode = msg.mode;
    return e;
}

}  // namespace map_asset_manager

#endif  // MAP_ASSET_MANAGER__ASSET_CODEC_HPP_
