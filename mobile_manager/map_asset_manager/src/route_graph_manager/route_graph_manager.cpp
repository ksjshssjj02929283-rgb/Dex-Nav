#include "map_asset_manager/route_graph_manager/route_graph_manager.hpp"
#include "map_asset_manager/asset_codec.hpp"

#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <limits>
#include <tf2/exceptions.h>
#include <unordered_set>
#include <utility>
#include <visualization_msgs/msg/marker.hpp>

namespace map_asset_manager {

// ──────────────────────────────────────────────────────────────────────
// Constructor
// ──────────────────────────────────────────────────────────────────────

RouteGraphManager::RouteGraphManager(
    rclcpp::Node*                    node,
    std::shared_ptr<JunctionStore>   junction_store,
    std::shared_ptr<RouteEdgeStore>  edge_store,
    std::shared_ptr<PointStore>      point_store,
    std::shared_ptr<DataEventBus>    event_bus,
    std::function<void()>            save_fn)
    : node_(node),
      junction_store_(junction_store),
      edge_store_(edge_store),
      point_store_(point_store),
      event_bus_(event_bus),
      save_fn_(save_fn)
{

    // 路网数据的读取统一走 get_assets；这里只保留路径解析这一计算服务
    resolve_nav_points_srv_ = node->create_service<nav_interfaces::srv::ResolveNavPoints>(
        "~/resolve_nav_points",
        std::bind(&RouteGraphManager::resolveNavPointsCallback, this, std::placeholders::_1, std::placeholders::_2));

    auto qos = rclcpp::QoS(1).transient_local().reliable();
    marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "~/route_graph_markers", qos);

    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, node);

    start_tolerance_ = node->declare_parameter<double>("route_start_tolerance", 0.1);
    narrow_turnaround_tolerance_ =
        node->declare_parameter<double>("narrow_turnaround_tolerance", 0.3);

    RCLCPP_INFO(node->get_logger(),
                "RouteGraphManager initialized (route_start_tolerance=%.3fm, "
                "narrow_turnaround_tolerance=%.3fm)",
                start_tolerance_, narrow_turnaround_tolerance_);
}

// ──────────────────────────────────────────────────────────────────────
// Junction CRUD
// ──────────────────────────────────────────────────────────────────────


// ──────────────────────────────────────────────────────────────────────
// Edge CRUD
// ──────────────────────────────────────────────────────────────────────

// ──────────────────────────────────────────────────────────────────────
// EventBus 响应
// ──────────────────────────────────────────────────────────────────────

void RouteGraphManager::onNavPointRemoved(const std::string& nav_name) {
    auto removed = edge_store_->removeEdgesContainingNav(nav_name);
    if (!removed.empty()) {
        save_fn_();
        publishMarkers();
        RCLCPP_INFO(node_->get_logger(),
                    "RouteGraph: cascaded removed %zu edges referencing nav '%s'",
                    removed.size(), nav_name.c_str());
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
    del_all.ns     = "";  // 空字符串 = 删除所有 namespace
    arr.markers.push_back(del_all);

    int id = 0;

    // Junction markers：圆柱体 + ID 文字
    for (const auto& jct : junction_store_->getAll()) {
        // 球形（junction）
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "map";
            m.header.stamp    = node_->get_clock()->now();
            m.ns              = "route_graph_junctions";
            m.id              = id++;
            m.type            = visualization_msgs::msg::Marker::SPHERE;
            m.action          = visualization_msgs::msg::Marker::ADD;
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
            m.header.stamp    = node_->get_clock()->now();
            m.ns              = "route_graph_junctions";
            m.id              = id++;
            m.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            m.action          = visualization_msgs::msg::Marker::ADD;
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
            m.header.stamp    = node_->get_clock()->now();
            m.ns              = "route_graph_nav_points";
            m.id              = id++;
            m.type            = visualization_msgs::msg::Marker::SPHERE;
            m.action          = visualization_msgs::msg::Marker::ADD;
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
            m.header.stamp    = node_->get_clock()->now();
            m.ns              = "route_graph_nav_points";
            m.id              = id++;
            m.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            m.action          = visualization_msgs::msg::Marker::ADD;
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
            if (pt) { fx = static_cast<float>(pt->x); fy = static_cast<float>(pt->y); found_from = true; }
        } else {
            auto jct = junction_store_->get(e.from.id);
            if (jct) { fx = jct->x; fy = jct->y; found_from = true; }
        }

        if (e.to.type == "nav") {
            auto pt = point_store_->getPoint(e.to.id);
            if (pt) { tx = static_cast<float>(pt->x); ty = static_cast<float>(pt->y); found_to = true; }
        } else {
            auto jct = junction_store_->get(e.to.id);
            if (jct) { tx = jct->x; ty = jct->y; found_to = true; }
        }

        if (!found_from || !found_to) continue;

        const float z_line  = 0.06f;
        const float z_arrow = 0.08f;

        // ── 构建路径点序列（直线或贝塞尔采样） ──────────────────────
        std::vector<std::pair<float,float>> path;
        if (e.edge_type == "BEZIER" && e.control_points.size() >= 2) {
            // Bernstein 贝塞尔采样（40 段）
            const auto& cp = e.control_points;
            const int   n  = static_cast<int>(cp.size()) - 1;
            const int   segs = 40;
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
            m.header.stamp    = node_->get_clock()->now();
            m.ns              = "route_graph_edges";
            m.id              = id++;
            m.type            = visualization_msgs::msg::Marker::LINE_STRIP;
            m.action          = visualization_msgs::msg::Marker::ADD;
            m.pose.orientation.w = 1.0;
            m.scale.x = line_width;
            m.color.r = cr; m.color.g = cg; m.color.b = cb; m.color.a = 0.9f;
            for (const auto& [px, py] : path) {
                geometry_msgs::msg::Point p;
                p.x = px; p.y = py; p.z = z_line;
                m.points.push_back(p);
            }
            arr.markers.push_back(m);
        }

        // ── 计算路径几何中点和切线方向（箭头+文字共用） ─────────────
        std::pair<float,float> mid_pt;
        float tangent_yaw;
        {
            const int last = static_cast<int>(path.size()) - 1;
            if (path.size() <= 2) {
                // 直线：精确几何中点
                mid_pt = {(path[0].first  + path[last].first)  / 2.0f,
                          (path[0].second + path[last].second) / 2.0f};
                tangent_yaw = std::atan2(path[last].second - path[0].second,
                                         path[last].first  - path[0].first);
            } else {
                // 曲线：数组中点
                const int mi = static_cast<int>(path.size()) / 2;
                mid_pt = path[mi];
                tangent_yaw = std::atan2(path[mi + 1].second - path[mi - 1].second,
                                         path[mi + 1].first  - path[mi - 1].first);
            }
        }

        // ── 在中点放方向箭头（quaternion 姿态，显示稳定） ────────────
        auto addArrowAt = [&](float px, float py, float yaw) {
            const float cy = std::cos(yaw * 0.5f);
            const float sy = std::sin(yaw * 0.5f);
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "map";
            m.header.stamp    = node_->get_clock()->now();
            m.ns              = "route_graph_edges";
            m.id              = id++;
            m.type            = visualization_msgs::msg::Marker::ARROW;
            m.action          = visualization_msgs::msg::Marker::ADD;
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
            m.color.r = cr; m.color.g = cg; m.color.b = cb; m.color.a = 0.95f;
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
            m.header.stamp    = node_->get_clock()->now();
            m.ns              = "route_graph_edges";
            m.id              = id++;
            m.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            m.action          = visualization_msgs::msg::Marker::ADD;
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
    std::pair<float, float> position;
    return endpointPosition(ep, position);
}

bool RouteGraphManager::endpointPosition(
    const RouteEndpoint& ep, std::pair<float, float>& position) const
{
    if (ep.type == "nav") {
        const auto point = point_store_->getPoint(ep.id);
        if (!point.has_value()) return false;
        position = {
            static_cast<float>(point->x), static_cast<float>(point->y)};
        return true;
    } else if (ep.type == "junction") {
        const auto junction = junction_store_->get(ep.id);
        if (!junction.has_value()) return false;
        position = {junction->x, junction->y};
        return true;
    }
    return false;
}

void RouteGraphManager::snapEdgeEndpoints(RouteEdge& edge) const {
    if (edge.edge_type != "BEZIER") return;

    std::pair<float, float> from_position;
    std::pair<float, float> to_position;
    if (!endpointPosition(edge.from, from_position) ||
        !endpointPosition(edge.to, to_position)) {
        return;
    }

    if (edge.control_points.empty()) {
        edge.control_points = {from_position, to_position};
    } else if (edge.control_points.size() == 1) {
        const auto middle = edge.control_points.front();
        edge.control_points = {from_position, middle, to_position};
    } else {
        edge.control_points.front() = from_position;
        edge.control_points.back() = to_position;
    }
}

// ──────────────────────────────────────────────────────────────────────
// ResolveNavPoints：nav_point_names → PathSegment[]
// ──────────────────────────────────────────────────────────────────────

std::optional<std::pair<float, float>> RouteGraphManager::endpointCoord(const RouteEndpoint& ep) const {
    if (ep.type == "nav") {
        const auto p = point_store_->getPoint(ep.id);
        if (!p) return std::nullopt;
        return std::make_pair(static_cast<float>(p->x), static_cast<float>(p->y));
    }
    if (ep.type == "junction") {
        const auto j = junction_store_->get(ep.id);
        if (!j) return std::nullopt;
        return std::make_pair(j->x, j->y);
    }
    return std::nullopt;
}

std::optional<RouteEdge> RouteGraphManager::findDirectEdge(
    const RouteEndpoint& a, const RouteEndpoint& b, bool& reversed) const {
    reversed = false;
    for (const auto& e : edge_store_->getAll()) {
        // 正向：from=a, to=b
        if (e.from.id == a.id && e.from.type == a.type &&
            e.to.id == b.id && e.to.type == b.type) {
            reversed = false;
            return e;
        }
        // 反向：双向边 from=b, to=a
        if (e.bidirectional &&
            e.from.id == b.id && e.from.type == b.type &&
            e.to.id == a.id && e.to.type == a.type) {
            reversed = true;
            return e;
        }
    }
    return std::nullopt;
}

std::optional<RouteEdge> RouteGraphManager::findEdgeTo(
    const RouteEndpoint& target, bool& reversed) const {
    reversed = false;
    for (const auto& e : edge_store_->getAll()) {
        // 直达：to == target
        if (e.to.id == target.id && e.to.type == target.type) {
            reversed = false;
            return e;
        }
        // 双向边反向：from == target，反向行进后终点为 target
        if (e.bidirectional &&
            e.from.id == target.id && e.from.type == target.type) {
            reversed = true;
            return e;
        }
    }
    return std::nullopt;
}

std::optional<std::pair<double, double>> RouteGraphManager::robotXY() const {
    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(node_->get_logger(), "robotXY: TF lookup failed: %s", ex.what());
        return std::nullopt;
    }
    return std::make_pair(tf.transform.translation.x, tf.transform.translation.y);
}

std::vector<std::pair<double, double>> RouteGraphManager::edgePolyline(const RouteEdge& e) const {
    std::vector<std::pair<double, double>> poly;

    if (e.edge_type == "BEZIER" && e.control_points.size() >= 2) {
        // Bernstein 采样，段数与 publishMarkers() 保持一致（40 段，约 2.5cm 级精度）
        const auto& cp   = e.control_points;
        const int   n    = static_cast<int>(cp.size()) - 1;
        const int   segs = 40;
        auto binom = [](int nn, int k) -> double {
            if (k == 0 || k == nn) return 1.0;
            double v = 1.0;
            for (int i = 0; i < k; ++i) v = v * (nn - i) / (i + 1);
            return v;
        };
        poly.reserve(static_cast<std::size_t>(segs) + 1);
        for (int s = 0; s <= segs; ++s) {
            const double t = static_cast<double>(s) / segs;
            double bx = 0.0, by = 0.0;
            for (int i = 0; i <= n; ++i) {
                const double b = binom(n, i) * std::pow(1.0 - t, n - i) * std::pow(t, i);
                bx += b * static_cast<double>(cp[i].first);
                by += b * static_cast<double>(cp[i].second);
            }
            poly.emplace_back(bx, by);
        }
        return poly;
    }

    const auto fc = endpointCoord(e.from);
    const auto tc = endpointCoord(e.to);
    if (!fc || !tc) return poly;  // 端点坐标缺失：无法算距离，交给调用方按 +inf 处理
    poly.emplace_back(static_cast<double>(fc->first), static_cast<double>(fc->second));
    poly.emplace_back(static_cast<double>(tc->first), static_cast<double>(tc->second));
    return poly;
}

double RouteGraphManager::distanceToPolyline(
    const std::pair<double, double>& p,
    const std::vector<std::pair<double, double>>& poly) {
    if (poly.empty()) return std::numeric_limits<double>::infinity();
    if (poly.size() == 1) return std::hypot(p.first - poly[0].first, p.second - poly[0].second);

    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i + 1 < poly.size(); ++i) {
        const double ax = poly[i].first,     ay = poly[i].second;
        const double dx = poly[i + 1].first - ax, dy = poly[i + 1].second - ay;
        const double len2 = dx * dx + dy * dy;
        double d;
        if (len2 < 1e-12) {
            d = std::hypot(p.first - ax, p.second - ay);
        } else {
            double t = ((p.first - ax) * dx + (p.second - ay) * dy) / len2;
            t = std::max(0.0, std::min(1.0, t));
            d = std::hypot(p.first - (ax + t * dx), p.second - (ay + t * dy));
        }
        best = std::min(best, d);
    }
    return best;
}

std::optional<RouteEdge> RouteGraphManager::findNearestEdgeTo(
    const RouteEndpoint& target, const std::pair<double, double>& robot,
    bool& reversed, double& distance) const {
    std::optional<RouteEdge> best;
    reversed = false;
    distance = std::numeric_limits<double>::infinity();

    for (const auto& e : edge_store_->getAll()) {
        bool rev = false;
        if (e.to.id == target.id && e.to.type == target.type) {
            rev = false;  // 直达：to == target
        } else if (e.bidirectional &&
                   e.from.id == target.id && e.from.type == target.type) {
            rev = true;   // 双向边反向行进后终点为 target
        } else {
            continue;
        }
        const double d = distanceToPolyline(robot, edgePolyline(e));
        if (d < distance) {
            distance = d;
            reversed = rev;
            best     = e;
        }
    }
    return best;
}

std::optional<std::pair<RouteEdge, RouteEdge>> RouteGraphManager::findNarrowTurnaround(
    const RouteEndpoint& p) const {
    // 先收 P 的全部关联边（无向意义上"挂在 P 上"的边），用来卡死「恰好两条」这个前提。
    // 窄道端点就该只有一进一出；多于两条说明它是个普通路口，退出方向不唯一，不能替上层做主。
    std::vector<RouteEdge> incident;
    for (const auto& e : edge_store_->getAll()) {
        const bool from_p = (e.from.id == p.id && e.from.type == p.type);
        const bool to_p   = (e.to.id == p.id && e.to.type == p.type);
        if (from_p || to_p) incident.push_back(e);
    }
    if (incident.size() != 2) {
        if (incident.size() > 2) {
            RCLCPP_WARN(node_->get_logger(),
                        "ResolveNavPoints: nav point '%s' has %zu incident edges (>2), "
                        "not a narrow turnaround point; falling back to the normal "
                        "single-point path.",
                        p.id.c_str(), incident.size());
        }
        return std::nullopt;
    }

    // 两条边里试着配出 out/in（两种配法都试，路网里的存储顺序不作数）：
    //   out：单向、from==P、backward==true            —— 倒车退出去的那条
    //   in ：单向、to==P、from==out.to                —— 正向开进来的那条
    // 最后一条是关键：退出去的终点必须正好是进来那条的起点，否则退出去以后接不回原路。
    for (int i = 0; i < 2; ++i) {
        const RouteEdge& out = incident[i];
        const RouteEdge& in  = incident[1 - i];
        if (out.bidirectional || !out.backward) continue;
        if (!(out.from.id == p.id && out.from.type == p.type)) continue;
        if (in.bidirectional) continue;
        if (!(in.to.id == p.id && in.to.type == p.type)) continue;
        if (!(in.from.id == out.to.id && in.from.type == out.to.type)) continue;
        return std::make_pair(out, in);
    }
    return std::nullopt;
}

nav_interfaces::msg::PathSegment RouteGraphManager::edgeToSegment(const RouteEdge& e, bool reversed) const {
    nav_interfaces::msg::PathSegment seg;

    // 按行进方向序的控制点
    std::vector<std::pair<float, float>> cps;
    const bool is_bezier = (e.edge_type == "BEZIER" && e.control_points.size() >= 2);
    if (is_bezier) {
        cps = e.control_points;
        if (reversed) std::reverse(cps.begin(), cps.end());
    } else {
        const auto fc = endpointCoord(e.from);
        const auto tc = endpointCoord(e.to);
        const std::pair<float, float> f = fc.value_or(std::make_pair(0.0f, 0.0f));
        const std::pair<float, float> t = tc.value_or(std::make_pair(0.0f, 0.0f));
        cps = reversed ? std::vector<std::pair<float, float>>{t, f}
                       : std::vector<std::pair<float, float>>{f, t};
    }

    seg.type = is_bezier ? nav_interfaces::msg::PathSegment::BEZIER : nav_interfaces::msg::PathSegment::LINE;
    seg.control_points.reserve(cps.size());
    for (const auto& cp : cps) {
        geometry_msgs::msg::Point p;
        p.x = static_cast<double>(cp.first);
        p.y = static_cast<double>(cp.second);
        p.z = 0.0;
        seg.control_points.push_back(p);
    }

    // target_yaw = 末点切向（倒数第二 → 末点）
    if (cps.size() >= 2) {
        const auto& a = cps[cps.size() - 2];
        const auto& b = cps[cps.size() - 1];
        seg.target_yaw = std::atan2(static_cast<double>(b.second - a.second),
                                    static_cast<double>(b.first - a.first));
    } else {
        seg.target_yaw = 0.0;
    }

    // 逐段运动参数透传（v1 path_follower 暂未支持逐段覆盖，schedule_planner 接收但暂不生效）
    seg.backward = e.backward;
    seg.max_speed = e.max_speed;
    seg.xy_tolerance = e.xy_tolerance;
    seg.th_tolerance = e.th_tolerance;
    seg.hold_direction = e.hold_direction.value_or(std::numeric_limits<float>::quiet_NaN());
    seg.padding = e.padding.value_or(std::numeric_limits<float>::quiet_NaN());
    seg.mode = e.mode;
    return seg;
}

// 控制点被改写后重算段末朝向（算法与 edgeToSegment 一致：倒数第二点 → 末点切向）。
static void refreshTargetYaw(nav_interfaces::msg::PathSegment& seg) {
    const auto& cps = seg.control_points;
    if (cps.size() < 2) return;
    const auto& a = cps[cps.size() - 2];
    const auto& b = cps[cps.size() - 1];
    seg.target_yaw = std::atan2(b.y - a.y, b.x - a.x);
}

void RouteGraphManager::resolveNavPointsCallback(
    const std::shared_ptr<nav_interfaces::srv::ResolveNavPoints::Request> req,
    std::shared_ptr<nav_interfaces::srv::ResolveNavPoints::Response> res) {
    const auto& names = req->nav_point_names;
    if (names.empty()) {
        res->success = false;
        res->message = "need >=1 nav_point_names, got 0";
        return;
    }

    // 校验所有导航点存在
    for (const auto& id : names) {
        if (!point_store_->hasPoint(id)) {
            res->success = false;
            res->message = "nav point not found: " + id;
            return;
        }
    }

    // ── 预查找 ────────────────────────────────────────────────────────
    // 取列表首点 + 当前车体坐标，在所有"终点为首点"的边里挑几何上离车最近的一条作
    // 参考输入边，并记下车体到它的最近距离；下面的分支据此决定起点吸附 / 先归位 / 原样下发。
    // TF 拿不到时退化成旧行为（任意一条入边），距离视为未知（+inf），所有改造分支自动跳过。
    const RouteEndpoint first_ep{names[0], "nav"};
    const auto first_pt = point_store_->getPoint(names[0]);  // 上面已校验存在
    const auto robot    = robotXY();

    bool                     ref_reversed = false;
    double                   ref_dist     = std::numeric_limits<double>::infinity();
    std::optional<RouteEdge> ref_edge;
    if (robot) {
        ref_edge = findNearestEdgeTo(first_ep, *robot, ref_reversed, ref_dist);
    } else {
        RCLCPP_WARN(node_->get_logger(),
                    "ResolveNavPoints: robot pose unavailable, falling back to "
                    "first matching incoming edge without geometry adjustment.");
        ref_edge = findEdgeTo(first_ep, ref_reversed);
    }

    const double robot_to_first =
        (robot && first_pt)
            ? std::hypot(robot->first - static_cast<double>(first_pt->x),
                         robot->second - static_cast<double>(first_pt->y))
            : std::numeric_limits<double>::infinity();

    std::vector<nav_interfaces::msg::PathSegment> segments;
    std::string decision;

    if (names.size() == 1) {
        // 单点：目标点即终点，路径就是那条参考输入边
        if (!ref_edge) {
            res->success = false;
            res->message = "no edge ending at nav point " + names[0];
            return;
        }
        // ── 窄道点位优先 ──────────────────────────────────────────────
        // 车已经停在只能倒车退出的死胡同端点上时，不管它对得准不准，都必须
        // 「先沿倒车边退到 Q，再沿入边正向开回 P」：就地重新对准会让差速车原地旋转，
        // 而这个点位之所以被配成一进一出 + 出去那条是倒车，就是因为那儿没有旋转空间。
        // 只在车确实压在 P 上（< narrow_turnaround_tolerance_）时才成立——车还在别处时
        // 下发一条起点在 P 的路径没有意义。TF 拿不到时 robot_to_first = +inf，自动跳过。
        std::optional<std::pair<RouteEdge, RouteEdge>> narrow;
        if (robot_to_first < narrow_turnaround_tolerance_) {
            narrow = findNarrowTurnaround(first_ep);
        }

        if (narrow) {
            // 两段都按边自身方向下发（out: P→Q 倒车，in: Q→P 正向），不做任何起点吸附：
            // 车头在整个过程里始终指向 Q→P，两段的 phase1 对准误差都是 0。
            const RouteEdge& out_edge = narrow->first;
            const RouteEdge& in_edge  = narrow->second;
            segments.push_back(edgeToSegment(out_edge, /*reversed=*/false));
            segments.push_back(edgeToSegment(in_edge, /*reversed=*/false));
            decision = "narrow turnaround at " + names[0] + " (robot dist " +
                       std::to_string(robot_to_first) + "m < " +
                       std::to_string(narrow_turnaround_tolerance_) + "m): reverse out via " +
                       out_edge.id + ", then re-enter via " + in_edge.id;
        } else {
            auto seg = edgeToSegment(*ref_edge, ref_reversed);
            if (robot_to_first < start_tolerance_) {
                // 车已经压在目标点上：整条跑参考输入边等于先退回边起点再走一遍，
                // 改成"车体当前位置 → 目标点"的短直线，段级参数（mode/backward/padding…）沿用该边。
                geometry_msgs::msg::Point p0;
                p0.x = robot->first;
                p0.y = robot->second;
                p0.z = 0.0;
                geometry_msgs::msg::Point p1;
                p1.x = static_cast<double>(first_pt->x);
                p1.y = static_cast<double>(first_pt->y);
                p1.z = 0.0;
                seg.type           = nav_interfaces::msg::PathSegment::LINE;
                seg.control_points = {p0, p1};
                refreshTargetYaw(seg);
                decision = "at goal (" + std::to_string(robot_to_first) + "m), rewrote edge " +
                           ref_edge->id + " as short LINE from robot pose";
            } else {
                decision = "nearest incoming edge " + ref_edge->id + " (robot-edge dist " +
                           std::to_string(ref_dist) + "m)";
            }
            segments.push_back(std::move(seg));
        }
    } else {
        // 多点：相邻两点各取一条直接边，首段即规划起始边
        std::optional<RouteEdge> start_edge;
        for (std::size_t i = 0; i + 1 < names.size(); ++i) {
            const RouteEndpoint a{names[i], "nav"};
            const RouteEndpoint b{names[i + 1], "nav"};
            bool reversed = false;
            const auto edge = findDirectEdge(a, b, reversed);
            if (!edge) {
                res->success = false;
                res->message = "no direct edge " + names[i] + " -> " + names[i + 1];
                return;
            }
            if (i == 0) start_edge = edge;
            segments.push_back(edgeToSegment(*edge, reversed));
        }

        // 车体到规划起始边（首段）的最近距离：车已经压在起始边上就不需要归位，
        // 直接从当前位置切入原路径即可。
        const double robot_to_start_edge =
            (robot && start_edge)
                ? distanceToPolyline(*robot, edgePolyline(*start_edge))
                : std::numeric_limits<double>::infinity();

        if (robot_to_first < start_tolerance_) {
            // 车已在规划起点附近：起始边的起点挪到车体当前位置，免得起步先横切回起点。
            // BEZIER 段只替换 P0，不强改直线（曲线形状由后面的控制点决定）。
            // edgeToSegment() 恒给出 >=2 个控制点，故 front() 安全。
            segments.front().control_points.front().x = robot->first;
            segments.front().control_points.front().y = robot->second;
            refreshTargetYaw(segments.front());
            decision = "start point snapped to robot pose (" +
                       std::to_string(robot_to_first) + "m)";
        } else if (robot_to_start_edge < start_tolerance_) {
            // 车没到起点，但已经压在规划起始边上：本来就在要走的边上，不必绕回参考输入边，
            // 按原路径直接下发即可。
            decision = "already on start edge (dist " + std::to_string(robot_to_start_edge) +
                       "m), publishing planned path as-is";
        } else if (ref_edge && ref_dist < start_tolerance_) {
            // 车既没到起点、也不在起始边上，但还压在某条入边上：
            // 先整条跑该参考输入边归位，再接原路径。
            segments.insert(segments.begin(), edgeToSegment(*ref_edge, ref_reversed));
            decision = "prepended homing edge " + ref_edge->id + " (robot-ref-edge dist " +
                       std::to_string(ref_dist) + "m, robot-start dist " +
                       std::to_string(robot_to_first) + "m, robot-start-edge dist " +
                       std::to_string(robot_to_start_edge) + "m)";
        } else {
            // 车既不在起点也不在起始边/任何入边上：保持原行为，按原路径直接下发。
            decision = "robot off graph (start dist " + std::to_string(robot_to_first) +
                       "m, start-edge dist " + std::to_string(robot_to_start_edge) +
                       "m, ref-edge dist " + std::to_string(ref_dist) +
                       "m), publishing planned path as-is";
        }
    }

    // 终点朝向优先取该 NavPoint 自身配置的 yaw，而不是 edgeToSegment() 算出的路径末段几何切向：
    // 请求的真正终点是 names.back()，其 yaw 代表用户在该点显式配置的期望朝向（如充电桩/工作台
    // 对接角度），可能与到达它的路径切向不一致；只有最后一段的 target_yaw 会被 schedule_planner
    // 保留（其余段被置 NaN），故只需覆盖 segments.back()。
    if (!segments.empty()) {
        if (const auto dest = point_store_->getPoint(names.back())) {
            segments.back().target_yaw = dest->yaw;
        }
    }

    res->success = true;
    res->message = "resolved " + std::to_string(names.size()) + " points -> " +
                   std::to_string(segments.size()) + " segments; " + decision;
    res->segments = std::move(segments);

    // 拼装点位名称列表字符串
    std::string names_str;
    for (std::size_t i = 0; i < names.size(); ++i) {
        if (i) names_str += " -> ";
        names_str += names[i];
    }

    RCLCPP_INFO(node_->get_logger(), "ResolveNavPoints: %s", res->message.c_str());
    RCLCPP_INFO(node_->get_logger(), "ResolveNavPoints points: %s", names_str.c_str());
}

}  // namespace map_asset_manager
