#include "map_asset_manager/route_manager/route_manager.hpp"

#include <cmath>
#include <thread>

namespace map_asset_manager {

RouteManager::RouteManager(rclcpp::Node* node,
                                 std::shared_ptr<RouteStore> route_store,
                                 std::shared_ptr<PointStore> point_store,
                                 std::shared_ptr<DataEventBus> event_bus,
                                 std::function<void()> save_fn)
    : node_(node),
      route_store_(route_store),
      point_store_(point_store),
      event_bus_(event_bus),
      save_fn_(std::move(save_fn)) {
    route_marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "~/route_markers", rclcpp::QoS(10).transient_local());

    get_route_srv_ = node_->create_service<nav_interfaces::srv::GetRoute>(
        "~/get_route",
        std::bind(&RouteManager::getRouteCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(node_->get_logger(), "RouteManager: services registered");
}

// ──────────────────────────────────────────────────────────────────────
// EventBus 响应
// ──────────────────────────────────────────────────────────────────────

void RouteManager::onPointRemoved(const std::string& point_name) {
    auto modified = route_store_->onPointRemoved(point_name);
    if (!modified.empty()) {
        RCLCPP_INFO(node_->get_logger(),
                    "RouteManager: cascaded point removal '%s' to %zu routes",
                    point_name.c_str(), modified.size());
        saveToFile();
        publishAllRouteMarkers();
    }
}

void RouteManager::onPointRenamed(const std::string& old_name, const std::string& new_name) {
    auto modified = route_store_->onPointRenamed(old_name, new_name);
    if (!modified.empty()) {
        RCLCPP_INFO(node_->get_logger(),
                    "RouteManager: cascaded rename '%s'->'%s' to %zu routes",
                    old_name.c_str(), new_name.c_str(), modified.size());
        saveToFile();
        publishAllRouteMarkers();
    }
}

// ──────────────────────────────────────────────────────────────────────
// Service 回调
// ──────────────────────────────────────────────────────────────────────

void RouteManager::getRouteCallback(
    const std::shared_ptr<nav_interfaces::srv::GetRoute::Request> req,
    std::shared_ptr<nav_interfaces::srv::GetRoute::Response> res) {
    auto opt = route_store_->getRoute(req->name);
    if (opt) {
        res->success     = true;
        res->name        = opt->name;
        res->point_names = opt->point_names;
    } else {
        res->success = false;
        RCLCPP_WARN(node_->get_logger(), "Route not found: %s", req->name.c_str());
    }
}

// ──────────────────────────────────────────────────────────────────────
// 辅助
// ──────────────────────────────────────────────────────────────────────

void RouteManager::saveToFile() {
    if (save_fn_) save_fn_();
}

void RouteManager::publishAllRouteMarkers() {
    auto all_routes = route_store_->getAllRoutes();
    visualization_msgs::msg::MarkerArray arr;
    auto now = node_->now();

    // 每条路线生成：LINE_STRIP + 每个点一个 ARROW + 路线名 TEXT
    // 每条路线 marker 数 = 1(线) + N(箭头) + 1(路线名文本)，总数不固定
    // 用 DELETEALL 清空旧 marker，再重新发布
    {
        visualization_msgs::msg::Marker del_all;
        del_all.header.frame_id = "map";
        del_all.header.stamp    = now;
        del_all.ns              = "route_static";
        del_all.id              = -1;
        del_all.action          = visualization_msgs::msg::Marker::DELETEALL;
        arr.markers.push_back(del_all);
    }

    int id = 0;
    for (const auto& route : all_routes) {
        // 解析路线中每个点的坐标
        std::vector<geometry_msgs::msg::Point> pts;
        std::vector<NavPoint> nav_pts;
        bool valid = true;
        for (const auto& name : route.point_names) {
            auto opt = point_store_->getPoint(name);
            if (!opt) { valid = false; break; }
            geometry_msgs::msg::Point p;
            p.x = opt->x;  p.y = opt->y;  p.z = 0.05;
            pts.push_back(p);
            nav_pts.push_back(*opt);
        }
        if (!valid || pts.empty()) continue;

        // ① 连线（LINE_STRIP）
        visualization_msgs::msg::Marker line;
        line.header.frame_id = "map";
        line.header.stamp    = now;
        line.ns              = "route_static";
        line.id              = id++;
        line.type            = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action          = visualization_msgs::msg::Marker::ADD;
        line.scale.x         = 0.05;
        line.color.r = 0.1f; line.color.g = 0.9f; line.color.b = 0.2f; line.color.a = 0.85f;
        line.points          = pts;
        line.lifetime        = rclcpp::Duration::from_seconds(0);
        arr.markers.push_back(line);

        // ② 每个航点：箭头（显示朝向）
        for (size_t i = 0; i < nav_pts.size(); ++i) {
            const auto& np = nav_pts[i];
            visualization_msgs::msg::Marker arrow;
            arrow.header.frame_id = "map";
            arrow.header.stamp    = now;
            arrow.ns              = "route_static";
            arrow.id              = id++;
            arrow.type            = visualization_msgs::msg::Marker::ARROW;
            arrow.action          = visualization_msgs::msg::Marker::ADD;
            arrow.pose.position.x = np.x;
            arrow.pose.position.y = np.y;
            arrow.pose.position.z = 0.05;
            {
                double half_yaw = np.yaw * 0.5;
                arrow.pose.orientation.z = std::sin(half_yaw);
                arrow.pose.orientation.w = std::cos(half_yaw);
            }
            arrow.scale.x = 0.35; arrow.scale.y = 0.07; arrow.scale.z = 0.07;
            arrow.color.r = 0.0f; arrow.color.g = 0.75f; arrow.color.b = 0.1f; arrow.color.a = 1.0f;
            arrow.lifetime = rclcpp::Duration::from_seconds(0);
            arr.markers.push_back(arrow);

            // 顺序+点名文本
            visualization_msgs::msg::Marker text;
            text.header.frame_id = "map";
            text.header.stamp    = now;
            text.ns              = "route_static";
            text.id              = id++;
            text.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text.action          = visualization_msgs::msg::Marker::ADD;
            text.pose.position.x = np.x;
            text.pose.position.y = np.y;
            text.pose.position.z = 0.45;
            text.pose.orientation.w = 1.0;
            text.scale.z         = 0.2;
            text.color.r = 0.9f; text.color.g = 1.0f; text.color.b = 0.9f; text.color.a = 1.0f;
            text.text = route.name + "\n" + std::to_string(i + 1) + ". " + np.name;
            text.lifetime = rclcpp::Duration::from_seconds(0);
            arr.markers.push_back(text);
        }
    }

    route_marker_pub_->publish(arr);
}

}  // namespace map_asset_manager
