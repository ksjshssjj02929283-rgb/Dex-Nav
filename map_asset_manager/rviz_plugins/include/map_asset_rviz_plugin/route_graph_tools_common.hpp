#pragma once

#include <chrono>
#include <cmath>
#include <chrono>
#include <functional>
#include <limits>
#include <mutex>
#include <string>
#include <vector>
#include <utility>

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QObject>
#include <QWidget>

#include <OgreManualObject.h>
#include <OgreRenderOperation.h>

#include <rclcpp/rclcpp.hpp>
#include <nav_interfaces/srv/get_all_nav_points.hpp>
#include <nav_interfaces/srv/get_route_graph.hpp>
#include <nav_interfaces/msg/route_edge.hpp>
#include <nav_interfaces/srv/get_all_nav_points.hpp>
#include <nav_interfaces/srv/get_route_graph.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>

namespace map_asset_rviz_plugin {

// ---------------------------------------------------------------------------
// Endpoint info (nav point or junction) for snap logic
// ---------------------------------------------------------------------------
struct EndpointInfo {
    std::string id;
    std::string type;  // "nav" | "junction"
    float x{0.0f};
    float y{0.0f};
};

// ---------------------------------------------------------------------------
// Edge configuration collected from the config dialog
// ---------------------------------------------------------------------------
struct EdgeConfig {
    bool        accepted{false};  ///< true if user clicked OK in the dialog
    std::string edge_id;
    std::string edge_type{"LINE"};
    bool        bidirectional{false};
    float       max_speed{0.8f};
    float       xy_tolerance{0.3f};
    float       th_tolerance{0.5f};
    float       lane_width{0.0f};
    std::string obs_policy{"STOP"};
    bool backward{false};                        ///< 差速移动车头反向（vx<0）
    std::optional<float> hold_direction;         ///< 全向车平移朝向 rad；nullopt=差速方式
    std::optional<float> padding;                ///< 车体膨胀大小 m；nullopt=系统默认
};

/// 弹框中四个运动参数控件（倒车 / 平移朝向 / 膨胀大小 / 最大速度）的句柄，供 AddEdgeTool/AddCurveEdgeTool 复用
struct OptionalEdgeFieldWidgets {
    QCheckBox* backward_cb{nullptr};
    QCheckBox* hold_dir_cb{nullptr};
    QDoubleSpinBox* hold_dir_spin{nullptr};
    QCheckBox* padding_cb{nullptr};
    QDoubleSpinBox* padding_spin{nullptr};
    QCheckBox* max_speed_cb{nullptr};
    QDoubleSpinBox* max_speed_spin{nullptr};
};

/// 在 form 的“单向通行”行之后追加四行：倒车、平移朝向（勾选启用，否则差速）、
/// 膨胀大小（勾选启用，否则系统默认）、最大速度（勾选启用，否则系统默认）
inline OptionalEdgeFieldWidgets addOptionalEdgeFieldRows(QFormLayout* form) {
    OptionalEdgeFieldWidgets w;

    w.backward_cb = new QCheckBox("倒车（backward）");

    w.hold_dir_cb = new QCheckBox("平移朝向（holdDirection）");
    w.hold_dir_spin = new QDoubleSpinBox();
    w.hold_dir_spin->setRange(-6.2831853, 6.2831853);
    w.hold_dir_spin->setDecimals(4);
    w.hold_dir_spin->setValue(0.0);
    w.hold_dir_spin->setSuffix(" rad");
    w.hold_dir_spin->setEnabled(false);
    QObject::connect(w.hold_dir_cb, &QCheckBox::toggled, w.hold_dir_spin, &QDoubleSpinBox::setEnabled);
    auto* hold_dir_w = new QWidget;
    auto* hold_dir_lay = new QHBoxLayout(hold_dir_w);
    hold_dir_lay->setContentsMargins(0, 0, 0, 0);
    hold_dir_lay->addWidget(w.hold_dir_cb);
    hold_dir_lay->addWidget(w.hold_dir_spin);

    w.padding_cb = new QCheckBox("膨胀大小（padding）");
    w.padding_spin = new QDoubleSpinBox();
    w.padding_spin->setRange(0.0, 5.0);
    w.padding_spin->setDecimals(3);
    w.padding_spin->setSingleStep(0.05);
    w.padding_spin->setValue(0.1);
    w.padding_spin->setSuffix(" m");
    w.padding_spin->setEnabled(false);
    QObject::connect(w.padding_cb, &QCheckBox::toggled, w.padding_spin, &QDoubleSpinBox::setEnabled);
    auto* padding_w = new QWidget;
    auto* padding_lay = new QHBoxLayout(padding_w);
    padding_lay->setContentsMargins(0, 0, 0, 0);
    padding_lay->addWidget(w.padding_cb);
    padding_lay->addWidget(w.padding_spin);

    w.max_speed_cb = new QCheckBox("最大速度（maxSpeed）");
    w.max_speed_spin = new QDoubleSpinBox();
    w.max_speed_spin->setRange(0.0, 10.0);
    w.max_speed_spin->setDecimals(2);
    w.max_speed_spin->setSingleStep(0.1);
    w.max_speed_spin->setValue(0.5);
    w.max_speed_spin->setSuffix(" m/s");
    w.max_speed_spin->setEnabled(false);
    QObject::connect(w.max_speed_cb, &QCheckBox::toggled, w.max_speed_spin, &QDoubleSpinBox::setEnabled);
    auto* max_speed_w = new QWidget;
    auto* max_speed_lay = new QHBoxLayout(max_speed_w);
    max_speed_lay->setContentsMargins(0, 0, 0, 0);
    max_speed_lay->addWidget(w.max_speed_cb);
    max_speed_lay->addWidget(w.max_speed_spin);

    form->addRow("", w.backward_cb);
    form->addRow("", hold_dir_w);
    form->addRow("", padding_w);
    form->addRow("", max_speed_w);
    return w;
}

/// 从弹框控件读取四个运动参数到 EdgeConfig
inline void readOptionalEdgeFields(const OptionalEdgeFieldWidgets& w, EdgeConfig& cfg) {
    cfg.backward = w.backward_cb->isChecked();
    if (w.hold_dir_cb->isChecked()) {
        cfg.hold_direction = static_cast<float>(w.hold_dir_spin->value());
    }
    if (w.padding_cb->isChecked()) {
        cfg.padding = static_cast<float>(w.padding_spin->value());
    }
    // 未勾选保持 EdgeConfig::max_speed 的默认值 0.0f（=系统默认，与 Area/RouteEdge 持久化约定一致）
    if (w.max_speed_cb->isChecked()) {
        cfg.max_speed = static_cast<float>(w.max_speed_spin->value());
    }
}

/// 把 EdgeConfig 的运动参数写入 RouteEdge 消息（hold_direction/padding：nullopt → NaN 哨兵；
/// max_speed：EdgeConfig 本身已是 0=默认 语义，直接透传）
inline void applyOptionalEdgeFields(nav_interfaces::msg::RouteEdge& edge, const EdgeConfig& cfg) {
    edge.backward = cfg.backward;
    edge.hold_direction = cfg.hold_direction.value_or(std::numeric_limits<float>::quiet_NaN());
    edge.padding = cfg.padding.value_or(std::numeric_limits<float>::quiet_NaN());
    edge.max_speed = cfg.max_speed;
}

/// Snap radius in meters
static constexpr float kSnapRadius = 0.5f;

/// Z height for preview geometry
static constexpr float kPreviewZ = 0.08f;

/// 路网编辑模式标志（全局共享，同一 so 内所有工具共享此变量）
/// RouteGraphStartTool::activate() 置 true，RouteGraphSaveTool::activate() 置 false
/// 所有路网编辑工具在 false 时拒绝写入操作（对标 create_waypoint::is_new_waypoint）
inline bool s_route_graph_editing = false;

// ---------------------------------------------------------------------------
// Load all endpoints (nav points + junctions) synchronously
// ---------------------------------------------------------------------------
inline std::vector<EndpointInfo> loadEndpoints(
    rclcpp::Node::SharedPtr node, const std::string& prefix)
{
    std::vector<EndpointInfo> eps;

    // --- nav points ---
    auto pts_cli = node->create_client<nav_interfaces::srv::GetAllNavPoints>(
        prefix + "/get_all_nav_points");
    if (pts_cli->wait_for_service(std::chrono::milliseconds(800))) {
        auto req = std::make_shared<nav_interfaces::srv::GetAllNavPoints::Request>();
        auto fut = pts_cli->async_send_request(req);
        if (fut.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
            for (const auto& pt : fut.get()->points) {
                EndpointInfo ep;
                ep.id   = pt.name;
                ep.type = "nav";
                ep.x    = static_cast<float>(pt.x);
                ep.y    = static_cast<float>(pt.y);
                eps.push_back(ep);
            }
        }
    }

    // --- junctions ---
    auto grph_cli = node->create_client<nav_interfaces::srv::GetRouteGraph>(
        prefix + "/get_route_graph");
    if (grph_cli->wait_for_service(std::chrono::milliseconds(800))) {
        auto req = std::make_shared<nav_interfaces::srv::GetRouteGraph::Request>();
        auto fut = grph_cli->async_send_request(req);
        if (fut.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
            for (const auto& jct : fut.get()->junctions) {
                EndpointInfo ep;
                ep.id   = jct.id;
                ep.type = "junction";
                ep.x    = jct.x;
                ep.y    = jct.y;
                eps.push_back(ep);
            }
        }
    }

    return eps;
}

// ---------------------------------------------------------------------------
// Load all endpoints asynchronously — non-blocking, callback called from
// a ROS executor thread once BOTH services have responded.
// Caller is responsible for dispatching the callback to the Qt main thread
// (via QMetaObject::invokeMethod with Qt::QueuedConnection) if needed.
// ---------------------------------------------------------------------------
inline void loadEndpointsAsync(
    rclcpp::Node::SharedPtr node,
    const std::string& prefix,
    std::function<void(std::vector<EndpointInfo>)> on_done)
{
    // Shared state between two independent async service calls
    struct State {
        std::mutex mu;
        std::vector<EndpointInfo> navpts;
        std::vector<EndpointInfo> junctions;
        int done_count{0};
        std::function<void(std::vector<EndpointInfo>)> cb;

        void maybeFinish() {
            // Called under mu lock; only fires when both responses are ready
            if (done_count < 2) return;
            std::vector<EndpointInfo> all;
            all.insert(all.end(), navpts.begin(), navpts.end());
            all.insert(all.end(), junctions.begin(), junctions.end());
            cb(std::move(all));
        }
    };
    auto state = std::make_shared<State>();
    state->cb  = std::move(on_done);

    // ---- nav points ----
    auto pts_cli = node->create_client<nav_interfaces::srv::GetAllNavPoints>(
        prefix + "/get_all_nav_points");
    if (pts_cli->service_is_ready()) {
        pts_cli->async_send_request(
            std::make_shared<nav_interfaces::srv::GetAllNavPoints::Request>(),
            [state, pts_cli](
                rclcpp::Client<nav_interfaces::srv::GetAllNavPoints>::SharedFuture fut) {
                std::vector<EndpointInfo> eps;
                try {
                    for (const auto& pt : fut.get()->points) {
                        eps.push_back({pt.name, "nav",
                                       static_cast<float>(pt.x),
                                       static_cast<float>(pt.y)});
                    }
                } catch (...) {}
                std::lock_guard<std::mutex> lk(state->mu);
                state->navpts = std::move(eps);
                ++state->done_count;
                state->maybeFinish();
            });
    } else {
        std::lock_guard<std::mutex> lk(state->mu);
        ++state->done_count;
        state->maybeFinish();
    }

    // ---- junctions ----
    auto grph_cli = node->create_client<nav_interfaces::srv::GetRouteGraph>(
        prefix + "/get_route_graph");
    if (grph_cli->service_is_ready()) {
        grph_cli->async_send_request(
            std::make_shared<nav_interfaces::srv::GetRouteGraph::Request>(),
            [state, grph_cli](
                rclcpp::Client<nav_interfaces::srv::GetRouteGraph>::SharedFuture fut) {
                std::vector<EndpointInfo> eps;
                try {
                    for (const auto& jct : fut.get()->junctions) {
                        eps.push_back({jct.id, "junction", jct.x, jct.y});
                    }
                } catch (...) {}
                std::lock_guard<std::mutex> lk(state->mu);
                state->junctions = std::move(eps);
                ++state->done_count;
                state->maybeFinish();
            });
    } else {
        std::lock_guard<std::mutex> lk(state->mu);
        ++state->done_count;
        state->maybeFinish();
    }
}

// ---------------------------------------------------------------------------
// Load only junctions
// ---------------------------------------------------------------------------
inline std::vector<EndpointInfo> loadJunctions(
    rclcpp::Node::SharedPtr node, const std::string& prefix)
{
    std::vector<EndpointInfo> eps;
    auto cli = node->create_client<nav_interfaces::srv::GetRouteGraph>(
        prefix + "/get_route_graph");
    if (cli->wait_for_service(std::chrono::milliseconds(800))) {
        auto req = std::make_shared<nav_interfaces::srv::GetRouteGraph::Request>();
        auto fut = cli->async_send_request(req);
        if (fut.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
            for (const auto& jct : fut.get()->junctions) {
                EndpointInfo ep;
                ep.id   = jct.id;
                ep.type = "junction";
                ep.x    = jct.x;
                ep.y    = jct.y;
                eps.push_back(ep);
            }
        }
    }
    return eps;
}

// ---------------------------------------------------------------------------
// Find nearest endpoint within radius (optionally exclude one id)
// ---------------------------------------------------------------------------
inline const EndpointInfo* findNearest(
    double x, double y,
    const std::vector<EndpointInfo>& eps,
    float radius = kSnapRadius,
    const std::string& exclude_id = "")
{
    float best2 = radius * radius;
    const EndpointInfo* best = nullptr;
    for (const auto& ep : eps) {
        if (!exclude_id.empty() && ep.id == exclude_id) continue;
        float dx = ep.x - static_cast<float>(x);
        float dy = ep.y - static_cast<float>(y);
        float d2 = dx * dx + dy * dy;
        if (d2 < best2) { best2 = d2; best = &ep; }
    }
    return best;
}

// ---------------------------------------------------------------------------
// Bezier curve computation (Bernstein basis)
// ---------------------------------------------------------------------------
inline std::vector<std::pair<float, float>> computeBezier(
    const std::vector<std::pair<float, float>>& ctrl_pts, int segments = 40)
{
    std::vector<std::pair<float, float>> result;
    if (ctrl_pts.size() < 2) return result;

    const int n = static_cast<int>(ctrl_pts.size()) - 1;
    auto binom  = [](int nn, int k) -> float {
        if (k == 0 || k == nn) return 1.0f;
        float val = 1.0f;
        for (int i = 0; i < k; ++i) val = val * (nn - i) / (i + 1);
        return val;
    };

    for (int s = 0; s <= segments; ++s) {
        float t = static_cast<float>(s) / segments;
        float bx = 0.0f, by = 0.0f;
        for (int i = 0; i <= n; ++i) {
            float b = binom(n, i) * std::pow(1.0f - t, n - i) * std::pow(t, i);
            bx += b * ctrl_pts[i].first;
            by += b * ctrl_pts[i].second;
        }
        result.push_back({bx, by});
    }
    return result;
}

// ---------------------------------------------------------------------------
// Ogre drawing helpers  (inline, called inside ManualObject clear/begin/end)
// ---------------------------------------------------------------------------

/// Draw a circle ring (one ManualObject section)
inline void drawRing(Ogre::ManualObject* obj,
                     float cx, float cy, float cz, float radius,
                     float r, float g, float b, int segs = 20)
{
    obj->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);
    obj->colour(r, g, b, 1.0f);
    for (int i = 0; i <= segs; ++i) {
        float a = static_cast<float>(i) / segs * 2.0f * static_cast<float>(M_PI);
        obj->position(cx + radius * std::cos(a), cy + radius * std::sin(a), cz);
    }
    obj->end();
}

/// Draw a line segment (one ManualObject section)
inline void drawSegment(Ogre::ManualObject* obj,
                        float x0, float y0, float x1, float y1, float z,
                        float r, float g, float b)
{
    obj->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
    obj->colour(r, g, b, 1.0f);
    obj->position(x0, y0, z);
    obj->colour(r, g, b, 1.0f);
    obj->position(x1, y1, z);
    obj->end();
}

/// Draw a bezier curve through ctrl_pts (one ManualObject section)
inline void drawBezierCurve(Ogre::ManualObject* obj,
                            const std::vector<std::pair<float, float>>& ctrl_pts,
                            float z, float r, float g, float b)
{
    auto pts = computeBezier(ctrl_pts);
    if (pts.size() < 2) return;
    obj->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);
    obj->colour(r, g, b, 1.0f);
    for (const auto& p : pts) {
        obj->position(p.first, p.second, z);
    }
    obj->end();
}

/// Draw a small X mark (two crossed lines)
inline void drawCross(Ogre::ManualObject* obj, float cx, float cy, float cz,
                      float half, float r, float g, float b)
{
    obj->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
    obj->colour(r, g, b, 1.0f);
    obj->position(cx - half, cy - half, cz);
    obj->colour(r, g, b, 1.0f);
    obj->position(cx + half, cy + half, cz);
    obj->colour(r, g, b, 1.0f);
    obj->position(cx + half, cy - half, cz);
    obj->colour(r, g, b, 1.0f);
    obj->position(cx - half, cy + half, cz);
    obj->end();
}

}  // namespace map_asset_rviz_plugin
