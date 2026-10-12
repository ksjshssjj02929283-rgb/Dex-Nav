#include "map_asset_manager/asset_editor.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_set>

#include <nav_interfaces/msg/asset_snapshot.hpp>

#include "map_asset_manager/area_manager/area_manager.hpp"
#include "map_asset_manager/asset_codec.hpp"
#include "map_asset_manager/room_manager/room_manager.hpp"

namespace map_asset_manager {

void FollowEndpoints(RouteEdge& edge, double old_ax, double old_ay, double old_bx, double old_by, double new_ax,
                     double new_ay, double new_bx, double new_by) {
    // 复数相似变换：p' = a' + s (p - a)，s = (b' - a') / (b - a)；旧起终点重合时退化为平移
    const double vx = old_bx - old_ax, vy = old_by - old_ay;
    const double ux = new_bx - new_ax, uy = new_by - new_ay;
    const double n2 = vx * vx + vy * vy;
    double sr = 1.0, si = 0.0;
    if (n2 > 1e-12) {
        sr = (ux * vx + uy * vy) / n2;
        si = (uy * vx - ux * vy) / n2;
    }
    for (auto& cp : edge.control_points) {
        const double px = cp.first - old_ax, py = cp.second - old_ay;
        cp.first = static_cast<float>(new_ax + sr * px - si * py);
        cp.second = static_cast<float>(new_ay + si * px + sr * py);
    }
    if (edge.control_points.size() >= 2) {
        edge.control_points.front() = {static_cast<float>(new_ax), static_cast<float>(new_ay)};
        edge.control_points.back() = {static_cast<float>(new_bx), static_cast<float>(new_by)};
    }
}

ApplyOutcome ApplySetAssets(const nav_interfaces::srv::SetAssets::Request& req, AssetSet& set_io,
                            const rclcpp::Time& now, const GeometryParams* geometry,
                            std::shared_ptr<const GridMap> grid) {
    using SA = nav_interfaces::srv::SetAssets;
    using ST = nav_interfaces::msg::AssetSnapshot;
    ApplyOutcome out;
    auto fail = [&](int32_t code, const std::string& msg) {
        out.ok = false;
        out.error_code = code;
        out.message = msg;
        return out;
    };
    if (req.mode > SA::Request::MODE_REMOVE) {
        return fail(ST::E_ASSET_MODE_INVALID, "invalid mode " + std::to_string(req.mode) +
                                                  "; expected 0 REPLACE / 1 MERGE / 2 ADD_ONLY / 3 UPDATE_ONLY / 4 REMOVE");
    }

    // ── 组装目标集合 ──
    // REPLACE：kinds 列出的类型从空开始（请求缺席即删），未列出的类型保持当前不变；kinds=0 视为全部。
    // MERGE / ADD_ONLY / UPDATE_ONLY / REMOVE：六类都从当前开始，按 name/id 逐项处理。
    const bool replace = (req.mode == SA::Request::MODE_REPLACE);
    const bool add_only = (req.mode == SA::Request::MODE_ADD_ONLY);
    const bool update_only = (req.mode == SA::Request::MODE_UPDATE_ONLY);
    const bool remove = (req.mode == SA::Request::MODE_REMOVE);
    const uint8_t k = (req.kinds == 0) ? SA::Request::KIND_ALL : req.kinds;
    auto keep = [&](uint8_t bit) { return !replace || !(k & bit); };
    AssetSet s;
    if (keep(SA::Request::KIND_POINTS)) s.points = set_io.points;
    if (keep(SA::Request::KIND_ROUTES)) s.routes = set_io.routes;
    if (keep(SA::Request::KIND_AREAS)) s.areas = set_io.areas;
    if (keep(SA::Request::KIND_ROOMS)) s.rooms = set_io.rooms;
    if (keep(SA::Request::KIND_JUNCTIONS)) s.junctions = set_io.junctions;
    if (keep(SA::Request::KIND_EDGES)) s.edges = set_io.edges;
    auto& rej = out.rejected;
    bool guard_hit = false;
    // 单条模式守卫：ADD_ONLY 已存在 → 2203080；UPDATE_ONLY / REMOVE 不存在 → 2203070（按当前集合查主键）
    auto guard = [&](const std::string& kind, const std::string& key, bool exists) -> bool {
        if (add_only && exists) { rej.push_back(kind + ":" + key + ":exists"); guard_hit = true; return false; }
        if ((update_only || remove) && !exists) { rej.push_back(kind + ":" + key + ":not_found"); guard_hit = true; return false; }
        return true;
    };

    if (remove) {
        std::unordered_set<std::string> rm_pts, rm_jcts;
        for (const auto& m : req.points) { if (m.name.empty()) { rej.push_back("point::name is empty"); continue; } if (guard("point", m.name, s.points.count(m.name) > 0)) { s.points.erase(m.name); rm_pts.insert(m.name); } }
        for (const auto& m : req.areas) { if (m.name.empty()) { rej.push_back("area::name is empty"); continue; } if (guard("area", m.name, s.areas.count(m.name) > 0)) s.areas.erase(m.name); }
        for (const auto& m : req.rooms) { if (m.name.empty()) { rej.push_back("room::name is empty"); continue; } if (guard("room", m.name, s.rooms.count(m.name) > 0)) s.rooms.erase(m.name); }
        for (const auto& m : req.routes) { if (m.name.empty()) { rej.push_back("route::name is empty"); continue; } if (guard("route", m.name, s.routes.count(m.name) > 0)) s.routes.erase(m.name); }
        for (const auto& m : req.junctions) { if (m.id.empty()) { rej.push_back("junction::id is empty"); continue; } if (guard("junction", m.id, s.junctions.count(m.id) > 0)) { s.junctions.erase(m.id); rm_jcts.insert(m.id); } }
        for (const auto& m : req.edges) { if (m.id.empty()) { rej.push_back("edge::id is empty"); continue; } if (guard("edge", m.id, s.edges.count(m.id) > 0)) s.edges.erase(m.id); }
        if (!rej.empty()) return fail(guard_hit ? ST::E_ASSET_NOT_FOUND : ST::E_ASSET_VALIDATION, "remove rejected; nothing applied");
        // 级联：删点 → 路线剔点、清引用它的边、区域/房间关联剔除；删节点 → 清引用它的边
        auto ep_removed = [&](const RouteEndpoint& ep) {
            return (ep.type == "nav" && rm_pts.count(ep.id) > 0) || (ep.type == "junction" && rm_jcts.count(ep.id) > 0);
        };
        for (auto it = s.edges.begin(); it != s.edges.end();) {
            if (ep_removed(it->second.from) || ep_removed(it->second.to)) it = s.edges.erase(it); else ++it;
        }
        auto strip = [](std::vector<std::string>& v, const std::unordered_set<std::string>& gone) {
            v.erase(std::remove_if(v.begin(), v.end(), [&](const std::string& n) { return gone.count(n) > 0; }), v.end());
        };
        for (auto& kv : s.routes) strip(kv.second.point_names, rm_pts);
        for (auto& kv : s.areas) { strip(kv.second.related_nav_point_names, rm_pts); strip(kv.second.manual_linked_point_names, rm_pts); strip(kv.second.related_junction_ids, rm_jcts); }
        for (auto& kv : s.rooms) { strip(kv.second.related_nav_point_names, rm_pts); strip(kv.second.manual_anchor_point_names, rm_pts); strip(kv.second.related_junction_ids, rm_jcts); }
    } else {
        // 字段校验（同批内重复也算失败）+ ADD_ONLY / UPDATE_ONLY 守卫
        {
            std::unordered_set<std::string> seen;
            for (const auto& m : req.points) {
                if (m.name.empty()) { rej.push_back("point::name is empty"); continue; }
                if (!seen.insert(m.name).second) { rej.push_back("point:" + m.name + ":duplicate in request"); continue; }
                if (!guard("point", m.name, s.points.count(m.name) > 0)) continue;
                s.points[m.name] = fromMsg(m);
            }
        }
        {
            std::unordered_set<std::string> seen;
            for (const auto& m : req.areas) {
                if (m.name.empty()) { rej.push_back("area::name is empty"); continue; }
                if (!seen.insert(m.name).second) { rej.push_back("area:" + m.name + ":duplicate in request"); continue; }
                if (m.type < nav_interfaces::msg::AreaInfo::VIRTUAL_WALL || m.type > nav_interfaces::msg::AreaInfo::BUMPY_AREA) { rej.push_back("area:" + m.name + ":type out of range"); continue; }
                // 虚拟墙是折线，2 点即可；其余区域是多边形，至少 3 点（与几何校验、掩膜渲染同一口径）
                const size_t min_vertices = (m.type == nav_interfaces::msg::AreaInfo::VIRTUAL_WALL) ? 2u : 3u;
                if (m.polygon.size() < min_vertices) {
                    rej.push_back("area:" + m.name + ":polygon needs >= " + std::to_string(min_vertices) + " points");
                    continue;
                }
                if (!guard("area", m.name, s.areas.count(m.name) > 0)) continue;
                s.areas[m.name] = AreaManager::fromMsg(m);
            }
        }
        {
            std::unordered_set<std::string> seen;
            for (const auto& m : req.rooms) {
                if (m.name.empty()) { rej.push_back("room::name is empty"); continue; }
                if (!seen.insert(m.name).second) { rej.push_back("room:" + m.name + ":duplicate in request"); continue; }
                if (m.type > nav_interfaces::msg::RoomInfo::STORAGE) { rej.push_back("room:" + m.name + ":type out of range"); continue; }
                if (m.polygon.size() < 3) { rej.push_back("room:" + m.name + ":polygon needs >= 3 points"); continue; }
                if (!guard("room", m.name, s.rooms.count(m.name) > 0)) continue;
                Room room = RoomManager::fromMsg(m);
                const std::string src = room.source.empty() ? std::string(DSGSource::MANUAL) : room.source;
                s.rooms[m.name] = RoomManager::enrich(std::move(room), src, now);
            }
        }
        {
            std::unordered_set<std::string> seen;
            for (const auto& m : req.routes) {
                if (m.name.empty()) { rej.push_back("route::name is empty"); continue; }
                if (!seen.insert(m.name).second) { rej.push_back("route:" + m.name + ":duplicate in request"); continue; }
                if (m.point_names.empty()) { rej.push_back("route:" + m.name + ":point_names is empty"); continue; }
                if (!guard("route", m.name, s.routes.count(m.name) > 0)) continue;
                s.routes[m.name] = fromMsg(m);
            }
        }
        {
            std::unordered_set<std::string> seen;
            for (const auto& m : req.junctions) {
                if (m.id.empty()) { rej.push_back("junction::id is empty"); continue; }
                if (!seen.insert(m.id).second) { rej.push_back("junction:" + m.id + ":duplicate in request"); continue; }
                if (!guard("junction", m.id, s.junctions.count(m.id) > 0)) continue;
                s.junctions[m.id] = fromMsg(m);
            }
        }
        {
            std::unordered_set<std::string> seen;
            for (const auto& m : req.edges) {
                if (m.id.empty()) { rej.push_back("edge::id is empty"); continue; }
                if (!seen.insert(m.id).second) { rej.push_back("edge:" + m.id + ":duplicate in request"); continue; }
                if (!guard("edge", m.id, s.edges.count(m.id) > 0)) continue;
                s.edges[m.id] = fromMsg(m);
            }
        }
        if (!rej.empty()) {
            return fail(guard_hit ? (add_only ? ST::E_ASSET_EXISTS : ST::E_ASSET_NOT_FOUND) : ST::E_ASSET_VALIDATION,
                        "validation failed; nothing applied");
        }
    }

    // 引用校验：所有引用必须在「应用后」的集合中可解析
    auto has_point = [&](const std::string& n) { return s.points.count(n) > 0; };
    auto has_jct = [&](const std::string& i) { return s.junctions.count(i) > 0; };
    auto check_ep = [&](const std::string& eid, const RouteEndpoint& ep, const char* side) {
        if (ep.type == "nav") { if (!has_point(ep.id)) rej.push_back("edge:" + eid + ":" + side + " nav point '" + ep.id + "' not found"); }
        else if (ep.type == "junction") { if (!has_jct(ep.id)) rej.push_back("edge:" + eid + ":" + side + " junction '" + ep.id + "' not found"); }
        else rej.push_back("edge:" + eid + ":" + side + " endpoint type '" + ep.type + "' must be nav|junction");
    };
    for (const auto& kv : s.edges) { check_ep(kv.first, kv.second.from, "from"); check_ep(kv.first, kv.second.to, "to"); }
    for (const auto& kv : s.routes)
        for (const auto& n : kv.second.point_names)
            if (!has_point(n)) rej.push_back("route:" + kv.first + ":point '" + n + "' not found");
    for (const auto& kv : s.areas) {
        for (const auto& n : kv.second.related_nav_point_names)   if (!has_point(n)) rej.push_back("area:" + kv.first + ":related nav point '" + n + "' not found");
        for (const auto& n : kv.second.manual_linked_point_names) if (!has_point(n)) rej.push_back("area:" + kv.first + ":linked nav point '" + n + "' not found");
        for (const auto& i : kv.second.related_junction_ids)      if (!has_jct(i))   rej.push_back("area:" + kv.first + ":related junction '" + i + "' not found");
    }
    for (const auto& kv : s.rooms) {
        for (const auto& n : kv.second.related_nav_point_names)   if (!has_point(n)) rej.push_back("room:" + kv.first + ":related nav point '" + n + "' not found");
        for (const auto& n : kv.second.manual_anchor_point_names) if (!has_point(n)) rej.push_back("room:" + kv.first + ":anchor nav point '" + n + "' not found");
        for (const auto& i : kv.second.related_junction_ids)      if (!has_jct(i))   rej.push_back("room:" + kv.first + ":related junction '" + i + "' not found");
    }
    if (!rej.empty()) return fail(ST::E_ASSET_REF_UNRESOLVED, "unresolved references; nothing applied");

    // 端点坐标（应用前 / 应用后）
    auto ep_pos = [](const AssetSet& from, const RouteEndpoint& ep, double& x, double& y) -> bool {
        if (ep.type == "nav") { auto it = from.points.find(ep.id); if (it == from.points.end()) return false; x = it->second.x; y = it->second.y; return true; }
        if (ep.type == "junction") { auto it = from.junctions.find(ep.id); if (it == from.junctions.end()) return false; x = it->second.x; y = it->second.y; return true; }
        return false;
    };
    std::unordered_set<std::string> req_edges;
    for (const auto& m : req.edges) req_edges.insert(m.id);
    if (!remove) {
        for (auto& kv : s.edges) {
            auto& e = kv.second;
            if (e.edge_type != "BEZIER") continue;
            double na_x, na_y, nb_x, nb_y;
            if (!ep_pos(s, e.from, na_x, na_y) || !ep_pos(s, e.to, nb_x, nb_y)) continue;
            auto& cp = e.control_points;
            if (req_edges.count(kv.first) > 0) {
                // 本次提交的贝塞尔边：首尾控制点吸附到端点
                const std::pair<float, float> a{static_cast<float>(na_x), static_cast<float>(na_y)};
                const std::pair<float, float> b{static_cast<float>(nb_x), static_cast<float>(nb_y)};
                if (cp.empty()) cp = {a, b};
                else if (cp.size() == 1) { const auto mid = cp.front(); cp = {a, mid, b}; }
                else { cp.front() = a; cp.back() = b; }
                continue;
            }
            // 未提交的贝塞尔边：端点被移动 → 整体随动（形状不变）
            double oa_x, oa_y, ob_x, ob_y;
            if (!ep_pos(set_io, e.from, oa_x, oa_y) || !ep_pos(set_io, e.to, ob_x, ob_y)) continue;
            const bool moved = std::hypot(na_x - oa_x, na_y - oa_y) > 1e-6 || std::hypot(nb_x - ob_x, nb_y - ob_y) > 1e-6;
            if (!moved || cp.size() < 2) continue;
            FollowEndpoints(e, oa_x, oa_y, ob_x, ob_y, na_x, na_y, nb_x, nb_y);
            ++out.moved_curves;
        }
    }

    // 几何校验：只针对本次新增 / 修改的项；未提交的存量项进入失效清单，不拦本次保存
    if (geometry != nullptr && !remove) {
        const GeometryChecker checker(grid, s, *geometry);
        for (const auto& m : req.points) { const auto c = checker.checkPose(s.points.at(m.name).x, s.points.at(m.name).y); if (!c.empty()) rej.push_back("point:" + m.name + ":" + c); }
        for (const auto& m : req.junctions) { const auto& j = s.junctions.at(m.id); const auto c = checker.checkPose(j.x, j.y); if (!c.empty()) rej.push_back("junction:" + m.id + ":" + c); }
        for (const auto& m : req.areas) { const auto c = checker.checkArea(s.areas.at(m.name)); if (!c.empty()) rej.push_back("area:" + m.name + ":" + c); }
        for (const auto& m : req.edges) { const auto c = checker.checkEdge(s.edges.at(m.id), s); if (!c.empty()) rej.push_back("edge:" + m.id + ":" + c); }
        if (!rej.empty()) return fail(ST::E_ASSET_GEOMETRY_INVALID, "geometry check failed; nothing applied");
    }

    RecomputeRelated(s);
    set_io = std::move(s);
    out.ok = true;
    out.error_code = ST::E_NONE;
    static const char* kModeName[] = {"replaced", "merged", "added", "updated", "removed"};
    out.message = std::string(kModeName[req.mode]) + (replace ? "(kinds=" + std::to_string(k) + ")" : "") + ": " +
                  std::to_string(set_io.points.size()) + " points, " + std::to_string(set_io.areas.size()) + " areas, " +
                  std::to_string(set_io.rooms.size()) + " rooms, " + std::to_string(set_io.routes.size()) + " routes, " +
                  std::to_string(set_io.junctions.size()) + " junctions, " + std::to_string(set_io.edges.size()) + " edges";
    if (out.moved_curves > 0) out.message += ", " + std::to_string(out.moved_curves) + " curves followed endpoints";
    return out;
}

}  // namespace map_asset_manager
