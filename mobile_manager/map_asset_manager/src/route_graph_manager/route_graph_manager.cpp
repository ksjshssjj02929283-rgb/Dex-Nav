#include "map_asset_manager/route_graph_manager/route_graph_manager.hpp"

#include <cmath>
#include <unordered_set>
#include <visualization_msgs/msg/marker.hpp>

namespace map_asset_manager {

static nav_interfaces::msg::RouteEndpoint toMsg(const RouteEndpoint& ep) {
    nav_interfaces::msg::RouteEndpoint msg;
    msg.id = ep.id;
    msg.type = ep.type;
    return msg;
}

static nav_interfaces::msg::RouteJunction toMsg(const RouteJunction& jct) {
    nav_interfaces::msg::RouteJunction msg;
    msg.id = jct.id;
    msg.x = jct.x;
    msg.y = jct.y;
    msg.yaw = jct.yaw;
    return msg;
}

static nav_interfaces::msg::RouteEdge toMsg(const RouteEdge& e) {
    nav_interfaces::msg::RouteEdge msg;
    msg.id = e.id;
    msg.from_ep = toMsg(e.from);
    msg.to_ep = toMsg(e.to);
    msg.edge_type = e.edge_type;
    msg.bidirectional = e.bidirectional;
    msg.max_speed = e.max_speed;
    msg.xy_tolerance = e.xy_tolerance;
    msg.th_tolerance = e.th_tolerance;
    msg.lane_width = e.lane_width;
    msg.obs_policy = e.obs_policy;
    for (const auto& cp : e.control_points) {
        geometry_msgs::msg::Point p;
        p.x = cp.first;
        p.y = cp.second;
        p.z = 0.0;
        msg.control_points.push_back(p);
    }
    return msg;
}

static RouteEndpoint fromMsg(const nav_interfaces::msg::RouteEndpoint& msg) { return {msg.id, msg.type}; }

static RouteJunction fromMsg(const nav_interfaces::msg::RouteJunction& msg) {
    RouteJunction jct;
    jct.id = msg.id;
    jct.x = msg.x;
    jct.y = msg.y;
    jct.yaw = msg.yaw;
    return jct;
}

static RouteEdge fromMsg(const nav_interfaces::msg::RouteEdge& msg) {
    RouteEdge e;
    e.id = msg.id;
    e.from = fromMsg(msg.from_ep);
    e.to = fromMsg(msg.to_ep);
    e.edge_type = msg.edge_type;
    e.bidirectional = msg.bidirectional;
    e.max_speed = msg.max_speed;
    e.xy_tolerance = msg.xy_tolerance;
    e.th_tolerance = msg.th_tolerance;
    e.lane_width = msg.lane_width;
    e.obs_policy = msg.obs_policy;
    for (const auto& cp : msg.control_points) {
        e.control_points.emplace_back(static_cast<float>(cp.x), static_cast<float>(cp.y));
    }
    return e;
}

// ──────────────────────────────────────────────────────────────────────
// Constructor
// ──────────────────────────────────────────────────────────────────────

RouteGraphManager::RouteGraphManager(rclcpp::Node* node,
                                     std::shared_ptr<JunctionStore> junction_store,
                                     std::shared_ptr<RouteEdgeStore> edge_store,
                                     std::shared_ptr<PointStore> point_store,
                                     std::function<void()> save_fn)
    : node_(node),
      junction_store_(junction_store),
      edge_store_(edge_store),
      point_store_(point_store),
      save_fn_(save_fn) {
    add_junction_srv_ = node->create_service<nav_interfaces::srv::AddRouteJunction>(
        "~/add_route_junction",
        std::bind(&RouteGraphManager::addJunctionCallback, this, std::placeholders::_1, std::placeholders::_2));

    remove_junction_srv_ = node->create_service<nav_interfaces::srv::RemoveRouteJunction>(
        "~/remove_route_junction",
        std::bind(&RouteGraphManager::removeJunctionCallback, this, std::placeholders::_1, std::placeholders::_2));

    add_edge_srv_ = node->create_service<nav_interfaces::srv::AddRouteEdge>(
        "~/add_route_edge",
        std::bind(&RouteGraphManager::addEdgeCallback, this, std::placeholders::_1, std::placeholders::_2));

    remove_edge_srv_ = node->create_service<nav_interfaces::srv::RemoveRouteEdge>(
        "~/remove_route_edge",
        std::bind(&RouteGraphManager::removeEdgeCallback, this, std::placeholders::_1, std::placeholders::_2));

    update_edge_srv_ = node->create_service<nav_interfaces::srv::UpdateRouteEdge>(
        "~/update_route_edge",
        std::bind(&RouteGraphManager::updateEdgeCallback, this, std::placeholders::_1, std::placeholders::_2));

    get_graph_srv_ = node->create_service<nav_interfaces::srv::GetRouteGraph>(
        "~/get_route_graph",
        std::bind(&RouteGraphManager::getRouteGraphCallback, this, std::placeholders::_1, std::placeholders::_2));

    auto qos = rclcpp::QoS(1).transient_local().reliable();
    marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("~/route_graph_markers", qos);

    RCLCPP_INFO(node->get_logger(), "RouteGraphManager initialized");
}

// ──────────────────────────────────────────────────────────────────────
// Junction CRUD
// ──────────────────────────────────────────────────────────────────────

void RouteGraphManager::addJunctionCallback(const std::shared_ptr<nav_interfaces::srv::AddRouteJunction::Request> req,
                                            std::shared_ptr<nav_interfaces::srv::AddRouteJunction::Response> res) {
    if (req->junction.id.empty()) {
        res->success = false;
        res->message = "Junction id must not be empty";
        return;
    }
    auto jct = fromMsg(req->junction);
    if (!junction_store_->add(jct)) {
        res->success = false;
        res->message = "Junction id '" + jct.id + "' already exists";
        return;
    }
    save_fn_();
    publishMarkers();
    res->success = true;
    res->message = "Added junction '" + jct.id + "'";
    RCLCPP_INFO(node_->get_logger(), "RouteGraph: added junction '%s'", jct.id.c_str());
}

void RouteGraphManager::removeJunctionCallback(
    const std::shared_ptr<nav_interfaces::srv::RemoveRouteJunction::Request> req,
    std::shared_ptr<nav_interfaces::srv::RemoveRouteJunction::Response> res) {
    if (!junction_store_->has(req->id)) {
        res->success = false;
        res->message = "Junction '" + req->id + "' not found";
        return;
    }
    auto removed_edges = edge_store_->removeEdgesContainingJunction(req->id);
    junction_store_->remove(req->id);
    save_fn_();
    publishMarkers();
    res->success = true;
    res->message = "Removed junction '" + req->id + "' and " + std::to_string(removed_edges.size()) + " edges";
    RCLCPP_INFO(node_->get_logger(), "RouteGraph: removed junction '%s' + %zu edges", req->id.c_str(),
                removed_edges.size());
}

// ──────────────────────────────────────────────────────────────────────
// Edge CRUD
// ──────────────────────────────────────────────────────────────────────

void RouteGraphManager::addEdgeCallback(const std::shared_ptr<nav_interfaces::srv::AddRouteEdge::Request> req,
                                        std::shared_ptr<nav_interfaces::srv::AddRouteEdge::Response> res) {
    auto edge = fromMsg(req->edge);
    if (edge.id.empty()) {
        res->success = false;
        res->message = "Edge id must not be empty";
        return;
    }
    if (!endpointExists(edge.from)) {
        res->success = false;
        res->message = "Edge 'from' endpoint not found: " + edge.from.type + ":" + edge.from.id;
        return;
    }
    if (!endpointExists(edge.to)) {
        res->success = false;
        res->message = "Edge 'to' endpoint not found: " + edge.to.type + ":" + edge.to.id;
        return;
    }
    if (!edge_store_->add(edge)) {
        res->success = false;
        res->message = "Edge id '" + edge.id + "' already exists";
        return;
    }
    save_fn_();
    publishMarkers();
    res->success = true;
    res->message = "Added edge '" + edge.id + "'";
    RCLCPP_INFO(node_->get_logger(), "RouteGraph: added edge '%s'", edge.id.c_str());
}

void RouteGraphManager::removeEdgeCallback(const std::shared_ptr<nav_interfaces::srv::RemoveRouteEdge::Request> req,
                                           std::shared_ptr<nav_interfaces::srv::RemoveRouteEdge::Response> res) {
    if (!edge_store_->remove(req->id)) {
        res->success = false;
        res->message = "Edge '" + req->id + "' not found";
        return;
    }
    save_fn_();
    publishMarkers();
    res->success = true;
    res->message = "Removed edge '" + req->id + "'";
}

void RouteGraphManager::updateEdgeCallback(const std::shared_ptr<nav_interfaces::srv::UpdateRouteEdge::Request> req,
                                           std::shared_ptr<nav_interfaces::srv::UpdateRouteEdge::Response> res) {
    auto edge = fromMsg(req->edge);
    if (!endpointExists(edge.from)) {
        res->success = false;
        res->message = "Edge 'from' endpoint not found: " + edge.from.type + ":" + edge.from.id;
        return;
    }
    if (!endpointExists(edge.to)) {
        res->success = false;
        res->message = "Edge 'to' endpoint not found: " + edge.to.type + ":" + edge.to.id;
        return;
    }
    if (!edge_store_->update(edge)) {
        res->success = false;
        res->message = "Edge '" + edge.id + "' not found";
        return;
    }
    save_fn_();
    publishMarkers();
    res->success = true;
    res->message = "Updated edge '" + edge.id + "'";
}

void RouteGraphManager::getRouteGraphCallback(
    const std::shared_ptr<nav_interfaces::srv::GetRouteGraph::Request> /*req*/,
    std::shared_ptr<nav_interfaces::srv::GetRouteGraph::Response> res) {
    for (const auto& jct : junction_store_->getAll()) {
        res->junctions.push_back(toMsg(jct));
    }

    // ── D034: 收集被边引用的 NavPoint 名称（去重）──
    // 边响应只存端点 id（不冗余坐标，避免 NavPoint 移动后边内坐标失效）；
    // 此处一次性把被引用的 NavPoint 坐标补到 nav_points[]，消费方一次请求即可建图。
    std::unordered_set<std::string> referenced_nav_names;
    const auto all_edges = edge_store_->getAll();
    for (const auto& e : all_edges) {
        res->edges.push_back(toMsg(e));
        if (e.from.type == "nav") referenced_nav_names.insert(e.from.id);
        if (e.to.type == "nav") referenced_nav_names.insert(e.to.id);
    }

    // ── D034: 从 PointStore 查坐标，按 nav_interfaces/NavPoint 组装 nav_points ──
    for (const auto& name : referenced_nav_names) {
        auto pt_opt = point_store_->getPoint(name);
        if (!pt_opt.has_value()) {
            // 被边引用但 NavPoint 已不存在：理论上 EventBus POINT_REMOVED 级联会清掉，
            // 此处仅作防御；不阻塞响应，但日志记录便于排查脏数据
            RCLCPP_WARN(node_->get_logger(), "GetRouteGraph: edge references missing NavPoint '%s' (skipped)",
                        name.c_str());
            continue;
        }
        nav_interfaces::msg::NavPoint np;
        np.name = pt_opt->name;
        np.type = pt_opt->type;
        np.x = pt_opt->x;
        np.y = pt_opt->y;
        np.theta = pt_opt->yaw;
        np.tolerance = pt_opt->tolerance;
        // np.id 在 PointStore 中无对应字段；msg.id 为可选展示用字段，保持默认 0
        res->nav_points.push_back(np);
    }

    res->success = true;
}

// ──────────────────────────────────────────────────────────────────────
// EventBus 响应
// ──────────────────────────────────────────────────────────────────────

void RouteGraphManager::onNavPointRemoved(const std::string& nav_name) {
    auto removed = edge_store_->removeEdgesContainingNav(nav_name);
    if (!removed.empty()) {
        save_fn_();
        publishMarkers();
        RCLCPP_INFO(node_->get_logger(), "RouteGraph: cascaded removed %zu edges referencing nav '%s'", removed.size(),
                    nav_name.c_str());
    }
}

// ──────────────────────────────────────────────────────────────────────
// Marker 发布
// ──────────────────────────────────────────────────────────────────────

void RouteGraphManager::publishMarkers() {
    visualization_msgs::msg::MarkerArray arr;

    // 清除旧 markers（ns 必须为空字符串，DELETEALL 才会删除所有 namespace 下的 marker）
    // ns="route_graph" 只删除该 namespace，而实际 marker 的 ns 是
    // "route_graph_junctions" / "route_graph_edges"，导致旧 marker 无法清除
    visualization_msgs::msg::Marker del_all;
    del_all.action = visualization_msgs::msg::Marker::DELETEALL;
    del_all.ns = "";  // 空字符串 = 删除所有 namespace
    arr.markers.push_back(del_all);

    int id = 0;

    // Junction markers：圆柱体 + ID 文字
    for (const auto& jct : junction_store_->getAll()) {
        // 球形（junction）
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "map";
            m.header.stamp = node_->get_clock()->now();
            m.ns = "route_graph_junctions";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = jct.x;
            m.pose.position.y = jct.y;
            m.pose.position.z = 0.125;
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.20;
            m.scale.y = 0.20;
            m.scale.z = 0.20;
            m.color.r = 0.6f;
            m.color.g = 0.6f;
            m.color.b = 0.6f;
            m.color.a = 0.8f;
            arr.markers.push_back(m);
        }
        // ID 文字
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "map";
            m.header.stamp = node_->get_clock()->now();
            m.ns = "route_graph_junctions";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = jct.x;
            m.pose.position.y = jct.y;
            m.pose.position.z = 0.35;
            m.pose.orientation.w = 1.0;
            m.scale.z = 0.18;  // 字体高度
            m.color.r = 1.0f;
            m.color.g = 1.0f;
            m.color.b = 1.0f;
            m.color.a = 0.9f;
            m.text = jct.id;
            arr.markers.push_back(m);
        }
    }

    // Nav point markers：紫色球形 + 名称文字
    for (const auto& pt : point_store_->getAllPoints()) {
        // 紫色球形
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "map";
            m.header.stamp = node_->get_clock()->now();
            m.ns = "route_graph_nav_points";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = pt.x;
            m.pose.position.y = pt.y;
            m.pose.position.z = 0.10;
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.20;
            m.scale.y = 0.20;
            m.scale.z = 0.20;
            m.color.r = 0.7f;
            m.color.g = 0.1f;
            m.color.b = 0.9f;
            m.color.a = 0.85f;
            arr.markers.push_back(m);
        }
        // 名称文字
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "map";
            m.header.stamp = node_->get_clock()->now();
            m.ns = "route_graph_nav_points";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = pt.x;
            m.pose.position.y = pt.y;
            m.pose.position.z = 0.35;
            m.pose.orientation.w = 1.0;
            m.scale.z = 0.15;
            m.color.r = 0.9f;
            m.color.g = 0.6f;
            m.color.b = 1.0f;
            m.color.a = 0.9f;
            m.text = pt.name;
            arr.markers.push_back(m);
        }
    }

    // Edge markers
    // 单向：细线（0.04m）+ 中点箭头
    // 双向：粗线（0.08m）+ 两端各一个小箭头
    // BEZIER 边：沿控制点序列采样贝塞尔曲线，其余同上
    for (const auto& e : edge_store_->getAll()) {
        // 获取起终点坐标
        float fx = 0, fy = 0, tx = 0, ty = 0;
        bool found_from = false, found_to = false;

        if (e.from.type == "nav") {
            auto pt = point_store_->getPoint(e.from.id);
            if (pt) {
                fx = static_cast<float>(pt->x);
                fy = static_cast<float>(pt->y);
                found_from = true;
            }
        } else {
            auto jct = junction_store_->get(e.from.id);
            if (jct) {
                fx = jct->x;
                fy = jct->y;
                found_from = true;
            }
        }

        if (e.to.type == "nav") {
            auto pt = point_store_->getPoint(e.to.id);
            if (pt) {
                tx = static_cast<float>(pt->x);
                ty = static_cast<float>(pt->y);
                found_to = true;
            }
        } else {
            auto jct = junction_store_->get(e.to.id);
            if (jct) {
                tx = jct->x;
                ty = jct->y;
                found_to = true;
            }
        }

        if (!found_from || !found_to) continue;

        const float z_line = 0.06f;
        const float z_arrow = 0.08f;

        // ── 构建路径点序列（直线或贝塞尔采样） ──────────────────────
        std::vector<std::pair<float, float>> path;
        if (e.edge_type == "BEZIER" && e.control_points.size() >= 2) {
            // Bernstein 贝塞尔采样（40 段）
            const auto& cp = e.control_points;
            const int n = static_cast<int>(cp.size()) - 1;
            const int segs = 40;
            auto binom = [](int nn, int k) -> float {
                if (k == 0 || k == nn) return 1.0f;
                float v = 1.0f;
                for (int i = 0; i < k; ++i) v = v * (nn - i) / (i + 1);
                return v;
            };
            for (int s = 0; s <= segs; ++s) {
                float t = static_cast<float>(s) / segs;
                float bx = 0, by = 0;
                for (int i = 0; i <= n; ++i) {
                    float b = binom(n, i) * std::pow(1.0f - t, n - i) * std::pow(t, i);
                    bx += b * static_cast<float>(cp[i].first);
                    by += b * static_cast<float>(cp[i].second);
                }
                path.push_back({bx, by});
            }
        } else {
            path.push_back({fx, fy});
            path.push_back({tx, ty});
        }

        // ── 颜色与线宽（单向/双向） ──────────────────────────────────
        const float line_width = e.bidirectional ? 0.08f : 0.04f;
        const float cr = 0.2f, cg = 0.75f, cb = 1.0f;  // 青蓝色

        // ── LINE_STRIP（路径主线） ────────────────────────────────────
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "map";
            m.header.stamp = node_->get_clock()->now();
            m.ns = "route_graph_edges";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::LINE_STRIP;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.orientation.w = 1.0;
            m.scale.x = line_width;
            m.color.r = cr;
            m.color.g = cg;
            m.color.b = cb;
            m.color.a = 0.9f;
            for (const auto& [px, py] : path) {
                geometry_msgs::msg::Point p;
                p.x = px;
                p.y = py;
                p.z = z_line;
                m.points.push_back(p);
            }
            arr.markers.push_back(m);
        }

        // ── 计算路径几何中点和切线方向（箭头+文字共用） ─────────────
        std::pair<float, float> mid_pt;
        float tangent_yaw;
        {
            const int last = static_cast<int>(path.size()) - 1;
            if (path.size() <= 2) {
                // 直线：精确几何中点
                mid_pt = {(path[0].first + path[last].first) / 2.0f, (path[0].second + path[last].second) / 2.0f};
                tangent_yaw = std::atan2(path[last].second - path[0].second, path[last].first - path[0].first);
            } else {
                // 曲线：数组中点
                const int mi = static_cast<int>(path.size()) / 2;
                mid_pt = path[mi];
                tangent_yaw =
                    std::atan2(path[mi + 1].second - path[mi - 1].second, path[mi + 1].first - path[mi - 1].first);
            }
        }

        // ── 在中点放方向箭头（quaternion 姿态，显示稳定） ────────────
        auto addArrowAt = [&](float px, float py, float yaw) {
            const float cy = std::cos(yaw * 0.5f);
            const float sy = std::sin(yaw * 0.5f);
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "map";
            m.header.stamp = node_->get_clock()->now();
            m.ns = "route_graph_edges";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::ARROW;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = px;
            m.pose.position.y = py;
            m.pose.position.z = z_arrow;
            m.pose.orientation.x = 0.0f;
            m.pose.orientation.y = 0.0f;
            m.pose.orientation.z = sy;
            m.pose.orientation.w = cy;
            m.scale.x = e.bidirectional ? 0.12f : 0.12f;  // 箭头总长度
            m.scale.y = e.bidirectional ? 0.04f : 0.03f;  // 箭头轴径
            m.scale.z = e.bidirectional ? 0.04f : 0.02f;  // 箭头头长
            m.color.r = cr;
            m.color.g = cg;
            m.color.b = cb;
            m.color.a = 0.95f;
            arr.markers.push_back(m);
        };

        if (e.bidirectional) {
            // 双向：中点处两个方向相反的箭头
            addArrowAt(mid_pt.first, mid_pt.second, tangent_yaw);
            addArrowAt(mid_pt.first, mid_pt.second, tangent_yaw + static_cast<float>(M_PI));
        } else {
            addArrowAt(mid_pt.first, mid_pt.second, tangent_yaw);
        }

        // ── 边 ID 文字（中点正上方） ──────────────────────────────────
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "map";
            m.header.stamp = node_->get_clock()->now();
            m.ns = "route_graph_edges";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = mid_pt.first;
            m.pose.position.y = mid_pt.second;
            m.pose.position.z = z_line + 0.25;
            m.pose.orientation.w = 1.0;
            m.scale.z = 0.15;
            m.color.r = 0.9f;
            m.color.g = 0.9f;
            m.color.b = 0.3f;
            m.color.a = 0.9f;
            m.text = e.id;
            arr.markers.push_back(m);
        }
    }

    marker_pub_->publish(arr);
}

// ──────────────────────────────────────────────────────────────────────
// 辅助
// ──────────────────────────────────────────────────────────────────────

bool RouteGraphManager::endpointExists(const RouteEndpoint& ep) const {
    if (ep.type == "nav") {
        return point_store_->getPoint(ep.id).has_value();
    } else if (ep.type == "junction") {
        return junction_store_->has(ep.id);
    }
    return false;
}

}  // namespace map_asset_manager
