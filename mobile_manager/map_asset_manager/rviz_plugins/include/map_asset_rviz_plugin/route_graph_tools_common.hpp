#pragma once

#include <cmath>
#include <chrono>
#include <functional>
#include <mutex>
#include <string>
#include <vector>
#include <utility>

#include <OgreManualObject.h>
#include <OgreRenderOperation.h>

#include <rclcpp/rclcpp.hpp>
#include <nav_interfaces/srv/get_all_nav_points.hpp>
#include <nav_interfaces/srv/get_route_graph.hpp>

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
};

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
