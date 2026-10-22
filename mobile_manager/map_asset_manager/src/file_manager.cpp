#include "map_asset_manager/file_manager.hpp"

#include <sys/stat.h>

#include <ctime>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace map_asset_manager {

FileManager::FileManager(rclcpp::Logger logger) : logger_(logger) {}

// ──────────────────────────────────────────────────────────────────────
// 导航点
// ──────────────────────────────────────────────────────────────────────

bool FileManager::loadPointsFromFile(const std::string& file_path,
                                     std::unordered_map<std::string, NavPoint>& out) {
    std::lock_guard<std::mutex> lock(file_mutex_);
    try {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(logger_, "Failed to open points file: %s", file_path.c_str());
            return false;
        }
        json j;
        file >> j;
        out.clear();

        if (!j.contains("points") || !j["points"].is_array()) {
            RCLCPP_ERROR(logger_, "Invalid JSON: missing 'points' array in %s", file_path.c_str());
            return false;
        }

        for (const auto& item : j["points"]) {
            NavPoint pt;
            pt.name      = item["name"].get<std::string>();
            pt.type      = item["type"].get<std::string>();
            pt.tolerance = item["tolerance"].get<double>();
            pt.x         = item["pose"]["x"].get<double>();
            pt.y         = item["pose"]["y"].get<double>();
            pt.yaw       = item["pose"]["yaw"].get<double>();
            out[pt.name] = pt;
        }

        RCLCPP_INFO(logger_, "Loaded %zu points from %s", out.size(), file_path.c_str());
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Exception loading points: %s", e.what());
        return false;
    }
}

bool FileManager::savePointsToFile(const std::string& file_path,
                                   const std::unordered_map<std::string, NavPoint>& in) {
    std::lock_guard<std::mutex> lock(file_mutex_);
    try {
        json j;
        j["points"] = json::array();
        for (const auto& [name, pt] : in) {
            json item;
            item["name"]        = pt.name;
            item["type"]        = pt.type;
            item["tolerance"]   = pt.tolerance;
            item["pose"]["x"]   = pt.x;
            item["pose"]["y"]   = pt.y;
            item["pose"]["yaw"] = pt.yaw;
            j["points"].push_back(item);
        }
        std::ofstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(logger_, "Failed to open for writing: %s", file_path.c_str());
            return false;
        }
        file << j.dump(2);
        RCLCPP_INFO(logger_, "Saved %zu points to %s", in.size(), file_path.c_str());
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Exception saving points: %s", e.what());
        return false;
    }
}

// ──────────────────────────────────────────────────────────────────────
// 路径
// ──────────────────────────────────────────────────────────────────────

bool FileManager::loadRoutesFromFile(const std::string& file_path,
                                     std::unordered_map<std::string, Route>& out) {
    std::lock_guard<std::mutex> lock(file_mutex_);
    try {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_WARN(logger_, "Routes file not found: %s (optional)", file_path.c_str());
            return true;  // routes 是可选的
        }
        json j;
        file >> j;
        out.clear();

        if (!j.contains("routes") || !j["routes"].is_array()) {
            RCLCPP_WARN(logger_, "Invalid JSON: missing 'routes' array");
            return true;
        }

        for (const auto& item : j["routes"]) {
            Route r;
            r.name        = item["name"].get<std::string>();
            r.point_names = item["points"].get<std::vector<std::string>>();
            out[r.name]   = r;
        }

        RCLCPP_INFO(logger_, "Loaded %zu routes from %s", out.size(), file_path.c_str());
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Exception loading routes: %s", e.what());
        return false;
    }
}

bool FileManager::saveRoutesToFile(const std::string& file_path,
                                   const std::unordered_map<std::string, Route>& in) {
    std::lock_guard<std::mutex> lock(file_mutex_);
    try {
        json j;
        j["routes"] = json::array();
        for (const auto& [name, r] : in) {
            json item;
            item["name"]   = r.name;
            item["points"] = r.point_names;
            j["routes"].push_back(item);
        }
        std::ofstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(logger_, "Failed to open for writing: %s", file_path.c_str());
            return false;
        }
        file << j.dump(2);
        RCLCPP_INFO(logger_, "Saved %zu routes to %s", in.size(), file_path.c_str());
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Exception saving routes: %s", e.what());
        return false;
    }
}

// ──────────────────────────────────────────────────────────────────────
// 区域
// ──────────────────────────────────────────────────────────────────────

bool FileManager::loadAreasFromFile(const std::string& file_path,
                                    std::unordered_map<std::string, Area>& out) {
    std::lock_guard<std::mutex> lock(file_mutex_);
    try {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_WARN(logger_, "Areas file not found: %s (optional)", file_path.c_str());
            return true;
        }
        json j;
        file >> j;
        out.clear();

        if (!j.contains("areas") || !j["areas"].is_array()) {
            return true;
        }

        for (const auto& item : j["areas"]) {
            Area a;
            a.name              = item["name"].get<std::string>();
            a.type              = AreaType::fromString(item.value("type", std::string{}));
            a.max_speed         = item.value("max_speed", 0.0f);
            a.stop_distance     = item.value("stop_distance", 0.0f);
            if (item.contains("manual_linked_point_names") && item["manual_linked_point_names"].is_array()) {
                a.manual_linked_point_names = item["manual_linked_point_names"].get<std::vector<std::string>>();
            }
            a.charger_type      = item.value("charger_type", std::string{});
            a.depth_min_height     = item.value("depth_min_height", 0.1f);
            a.attribute         = item.value("attribute", std::string{});
            if (item.contains("polygon") && item["polygon"].is_array()) {
                for (const auto& pt : item["polygon"]) {
                    geometry_msgs::msg::Point p;
                    p.x = pt["x"].get<double>();
                    p.y = pt["y"].get<double>();
                    p.z = 0.0;
                    a.polygon.push_back(p);
                }
            }
            // 自动空间归属（持久化，启动后立即可用，SpatialIndexer 后续会重算覆盖）
            if (item.contains("related_nav_point_names") && item["related_nav_point_names"].is_array()) {
                a.related_nav_point_names = item["related_nav_point_names"].get<std::vector<std::string>>();
            }
            if (item.contains("related_junction_ids") && item["related_junction_ids"].is_array()) {
                a.related_junction_ids = item["related_junction_ids"].get<std::vector<std::string>>();
            }
            out[a.name] = a;
        }

        RCLCPP_INFO(logger_, "Loaded %zu areas from %s", out.size(), file_path.c_str());
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Exception loading areas: %s", e.what());
        return false;
    }
}

bool FileManager::saveAreasToFile(const std::string& file_path,
                                  const std::unordered_map<std::string, Area>& in) {
    std::lock_guard<std::mutex> lock(file_mutex_);
    try {
        json j;
        j["areas"] = json::array();
        for (const auto& [name, a] : in) {
            json item;
            item["name"]               = a.name;
            item["type"]               = AreaType::toString(a.type);
            item["max_speed"]          = a.max_speed;
            item["stop_distance"]      = a.stop_distance;
            item["manual_linked_point_names"] = a.manual_linked_point_names;
            item["charger_type"]       = a.charger_type;
            item["attribute"]          = a.attribute;
            item["polygon"]            = json::array();
            for (const auto& pt : a.polygon) {
                json p;
                p["x"] = pt.x;
                p["y"] = pt.y;
                item["polygon"].push_back(p);
            }
            // 自动空间归属（SpatialIndexer 维护，持久化用于启动时立即可用）
            item["related_nav_point_names"] = a.related_nav_point_names;
            item["related_junction_ids"]    = a.related_junction_ids;
            j["areas"].push_back(item);
        }
        std::ofstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(logger_, "Failed to open for writing: %s", file_path.c_str());
            return false;
        }
        file << j.dump(2);
        RCLCPP_INFO(logger_, "Saved %zu areas to %s", in.size(), file_path.c_str());
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Exception saving areas: %s", e.what());
        return false;
    }
}

// ──────────────────────────────────────────────────────────────────────
// 地图文件夹
// ──────────────────────────────────────────────────────────────────────

bool FileManager::loadMapFolder(const std::string& base_path,
                                const std::string& folder_name,
                                std::string& points_file,
                                std::string& routes_file,
                                std::string& areas_file) {
    std::lock_guard<std::mutex> lock(file_mutex_);
    std::string folder_path = base_path + "/" + folder_name;
    if (!std::filesystem::exists(folder_path)) {
        RCLCPP_ERROR(logger_, "Map folder does not exist: %s", folder_path.c_str());
        return false;
    }
    points_file = folder_path + "/points.json";
    routes_file = folder_path + "/routes.json";
    areas_file  = folder_path + "/areas.json";
    RCLCPP_INFO(logger_, "Map folder: %s", folder_path.c_str());
    return true;
}

// ──────────────────────────────────────────────────────────────────────
// 统一资产文件（map_assets.json）
// ──────────────────────────────────────────────────────────────────────

bool FileManager::loadAllAssetsFromFile(const std::string& file_path,
                                        std::unordered_map<std::string, NavPoint>& points,
                                        std::unordered_map<std::string, Route>& routes,
                                        std::unordered_map<std::string, Area>& areas,
                                        std::vector<RouteJunction>& junctions,
                                        std::vector<RouteEdge>& edges,
                                        std::unordered_map<std::string, Room>& rooms) {
    std::lock_guard<std::mutex> lock(file_mutex_);
    points.clear();
    routes.clear();
    areas.clear();
    junctions.clear();
    edges.clear();
    rooms.clear();

    if (!std::filesystem::exists(file_path)) {
        RCLCPP_WARN(logger_, "Assets file not found (will be created on first save): %s",
                    file_path.c_str());
        return true;  // 新地图没有资产文件是正常情况
    }

    try {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(logger_, "Failed to open assets file: %s", file_path.c_str());
            return false;
        }
        json j;
        file >> j;

        // ── points ──────────────────────────────────────────────
        if (j.contains("points") && j["points"].is_array()) {
            for (const auto& item : j["points"]) {
                NavPoint pt;
                pt.name      = item["name"].get<std::string>();
                pt.type      = item["type"].get<std::string>();
                pt.tolerance = item["tolerance"].get<double>();
                pt.x         = item["pose"]["x"].get<double>();
                pt.y         = item["pose"]["y"].get<double>();
                pt.yaw       = item["pose"]["yaw"].get<double>();
                points[pt.name] = pt;
            }
        }

        // ── routes ──────────────────────────────────────────────
        if (j.contains("routes") && j["routes"].is_array()) {
            for (const auto& item : j["routes"]) {
                Route r;
                r.name        = item["name"].get<std::string>();
                r.point_names = item["points"].get<std::vector<std::string>>();
                routes[r.name] = r;
            }
        }

        // ── areas ───────────────────────────────────────────────
        if (j.contains("areas") && j["areas"].is_array()) {
            for (const auto& item : j["areas"]) {
                Area a;
                a.name             = item["name"].get<std::string>();
                a.type             = AreaType::fromString(item.value("type", std::string{}));
                a.max_speed        = item.value("max_speed", 0.0f);
                a.stop_distance    = item.value("stop_distance", 0.0f);
                if (item.contains("manual_linked_point_names") && item["manual_linked_point_names"].is_array()) {
                    a.manual_linked_point_names = item["manual_linked_point_names"].get<std::vector<std::string>>();
                }
                a.charger_type     = item.value("charger_type", std::string{});
                a.attribute        = item.value("attribute", std::string{});
                if (item.contains("polygon") && item["polygon"].is_array()) {
                    for (const auto& pt : item["polygon"]) {
                        geometry_msgs::msg::Point p;
                        p.x = pt["x"].get<double>();
                        p.y = pt["y"].get<double>();
                        p.z = 0.0;
                        a.polygon.push_back(p);
                    }
                }
                // 自动空间归属（持久化）
                if (item.contains("related_nav_point_names") && item["related_nav_point_names"].is_array()) {
                    a.related_nav_point_names = item["related_nav_point_names"].get<std::vector<std::string>>();
                }
                if (item.contains("related_junction_ids") && item["related_junction_ids"].is_array()) {
                    a.related_junction_ids = item["related_junction_ids"].get<std::vector<std::string>>();
                }
                areas[a.name] = a;
            }
        }

        RCLCPP_INFO(logger_, "Loaded assets from %s: %zu points, %zu routes, %zu areas",
                    file_path.c_str(), points.size(), routes.size(), areas.size());

        // ── route_graph ──────────────────────────────────────────
        if (j.contains("route_graph")) {
            const auto& rg = j["route_graph"];
            if (rg.contains("junctions") && rg["junctions"].is_array()) {
                for (const auto& item : rg["junctions"]) {
                    RouteJunction jct;
                    jct.id  = item["id"].get<std::string>();
                    jct.x   = item.value("x", 0.0f);
                    jct.y   = item.value("y", 0.0f);
                    jct.yaw = item.value("yaw", 0.0f);
                    junctions.push_back(jct);
                }
            }
            if (rg.contains("edges") && rg["edges"].is_array()) {
                for (const auto& item : rg["edges"]) {
                    RouteEdge e;
                    e.id              = item["id"].get<std::string>();
                    e.from.id         = item["from"]["id"].get<std::string>();
                    e.from.type       = item["from"]["type"].get<std::string>();
                    e.to.id           = item["to"]["id"].get<std::string>();
                    e.to.type         = item["to"]["type"].get<std::string>();
                    e.edge_type       = item.value("edge_type", std::string{"LINE"});
                    e.bidirectional   = item.value("bidirectional", false);
                    e.max_speed       = item.value("max_speed", 0.0f);
                    e.xy_tolerance    = item.value("xy_tolerance", 0.3f);
                    e.th_tolerance    = item.value("th_tolerance", 0.5f);
                    e.lane_width      = item.value("lane_width", 0.0f);
                    e.obs_policy      = item.value("obs_policy", std::string{"STOP"});
                    if (item.contains("control_points") && item["control_points"].is_array()) {
                        for (const auto& cp : item["control_points"]) {
                            e.control_points.emplace_back(
                                cp[0].get<float>(), cp[1].get<float>());
                        }
                    }
                    e.backward = item.value("backward", false);
                    if (item.contains("hold_direction") && item["hold_direction"].is_number()) {
                        e.hold_direction = item["hold_direction"].get<float>();
                    }
                    if (item.contains("padding") && item["padding"].is_number()) {
                        e.padding = item["padding"].get<float>();
                    }
                    e.mode = static_cast<uint8_t>(item.value("mode", 0));
                    edges.push_back(e);
                }
            }
        }

        RCLCPP_INFO(logger_, "Loaded route_graph: %zu junctions, %zu edges",
                    junctions.size(), edges.size());

        // ── dsg.layers.ROOMS（v2.0 新增；v1 文件缺失视为空，向后兼容）────
        if (j.contains("dsg") && j["dsg"].is_object() &&
            j["dsg"].contains("layers") && j["dsg"]["layers"].is_object() &&
            j["dsg"]["layers"].contains("ROOMS") &&
            j["dsg"]["layers"]["ROOMS"].is_array()) {
            for (const auto& item : j["dsg"]["layers"]["ROOMS"]) {
                Room r;
                r.id              = item.value("id", std::string{});
                r.source          = item.value("source", std::string{"manual"});
                r.created_at_ns   = item.value("created_at_ns", int64_t{0});
                r.updated_at_ns   = item.value("updated_at_ns", int64_t{0});
                r.name            = item.value("name", std::string{});
                r.type            = RoomType::fromString(item.value("type", std::string{}));
                r.semantic_label  = item.value("semantic_label", std::string{});
                r.display_label   = item.value("display_label", std::string{});
                if (item.contains("manual_anchor_point_names") && item["manual_anchor_point_names"].is_array()) {
                    r.manual_anchor_point_names = item["manual_anchor_point_names"].get<std::vector<std::string>>();
                }
                r.attribute         = item.value("attribute", std::string{});
                r.external_dsg_node_id = item.value("external_dsg_node_id", std::string{});

                if (item.contains("pose") && item["pose"].is_object()) {
                    const auto& p = item["pose"];
                    if (p.contains("position")) {
                        r.pose.position.x = p["position"].value("x", 0.0);
                        r.pose.position.y = p["position"].value("y", 0.0);
                        r.pose.position.z = p["position"].value("z", 0.0);
                    }
                    if (p.contains("orientation")) {
                        r.pose.orientation.x = p["orientation"].value("x", 0.0);
                        r.pose.orientation.y = p["orientation"].value("y", 0.0);
                        r.pose.orientation.z = p["orientation"].value("z", 0.0);
                        r.pose.orientation.w = p["orientation"].value("w", 1.0);
                    } else {
                        r.pose.orientation.w = 1.0;
                    }
                } else {
                    r.pose.orientation.w = 1.0;
                }

                if (item.contains("polygon") && item["polygon"].is_array()) {
                    for (const auto& pt : item["polygon"]) {
                        geometry_msgs::msg::Point pp;
                        pp.x = pt.value("x", 0.0);
                        pp.y = pt.value("y", 0.0);
                        pp.z = 0.0;
                        r.polygon.push_back(pp);
                    }
                }

                if (item.contains("bounding_box") && item["bounding_box"].is_array() &&
                    item["bounding_box"].size() == 6) {
                    for (size_t i = 0; i < 6; ++i) {
                        r.bounding_box[i] = item["bounding_box"][i].get<double>();
                    }
                }

                // 自动空间归属（持久化）
                if (item.contains("related_nav_point_names") && item["related_nav_point_names"].is_array()) {
                    r.related_nav_point_names = item["related_nav_point_names"].get<std::vector<std::string>>();
                }
                if (item.contains("related_junction_ids") && item["related_junction_ids"].is_array()) {
                    r.related_junction_ids = item["related_junction_ids"].get<std::vector<std::string>>();
                }

                if (!r.name.empty()) {
                    rooms[r.name] = r;
                }
            }
            RCLCPP_INFO(logger_, "Loaded dsg.layers.ROOMS: %zu rooms", rooms.size());
        }

        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Exception loading assets file: %s", e.what());
        return false;
    }
}

json FileManager::buildAssetsJson(const std::unordered_map<std::string, NavPoint>& points,
                                  const std::unordered_map<std::string, Route>& routes,
                                  const std::unordered_map<std::string, Area>& areas,
                                  const std::vector<RouteJunction>& junctions,
                                  const std::vector<RouteEdge>& edges,
                                  const std::unordered_map<std::string, Room>& rooms) {
    json j;

    // ── points ──────────────────────────────────────────────
    j["points"] = json::array();
    for (const auto& [name, pt] : points) {
        json item;
        item["name"]        = pt.name;
        item["type"]        = pt.type;
        item["tolerance"]   = pt.tolerance;
        item["pose"]["x"]   = pt.x;
        item["pose"]["y"]   = pt.y;
        item["pose"]["yaw"] = pt.yaw;
        j["points"].push_back(item);
    }

    // ── routes ──────────────────────────────────────────────
    j["routes"] = json::array();
    for (const auto& [name, r] : routes) {
        json item;
        item["name"]   = r.name;
        item["points"] = r.point_names;
        j["routes"].push_back(item);
    }

    // ── areas ───────────────────────────────────────────────
    j["areas"] = json::array();
    for (const auto& [name, a] : areas) {
        json item;
        item["name"]               = a.name;
        item["type"]               = AreaType::toString(a.type);
        item["max_speed"]          = a.max_speed;
        item["stop_distance"]      = a.stop_distance;
        item["manual_linked_point_names"] = a.manual_linked_point_names;
        item["charger_type"]       = a.charger_type;
        item["attribute"]          = a.attribute;
        item["polygon"]            = json::array();
        for (const auto& pt : a.polygon) {
            json p;
            p["x"] = pt.x;
            p["y"] = pt.y;
            item["polygon"].push_back(p);
        }
        // 自动空间归属（SpatialIndexer 维护）
        item["related_nav_point_names"] = a.related_nav_point_names;
        item["related_junction_ids"]    = a.related_junction_ids;
        j["areas"].push_back(item);
    }

    // ── route_graph ──────────────────────────────────────────
    json rg;
    rg["junctions"] = json::array();
    for (const auto& jct : junctions) {
        json item;
        item["id"]  = jct.id;
        item["x"]   = jct.x;
        item["y"]   = jct.y;
        item["yaw"] = jct.yaw;
        rg["junctions"].push_back(item);
    }
    rg["edges"] = json::array();
    for (const auto& e : edges) {
        json item;
        item["id"]            = e.id;
        item["from"]["id"]    = e.from.id;
        item["from"]["type"]  = e.from.type;
        item["to"]["id"]      = e.to.id;
        item["to"]["type"]    = e.to.type;
        item["edge_type"]     = e.edge_type;
        item["bidirectional"] = e.bidirectional;
        item["max_speed"]     = e.max_speed;
        item["xy_tolerance"]  = e.xy_tolerance;
        item["th_tolerance"]  = e.th_tolerance;
        item["lane_width"]    = e.lane_width;
        item["obs_policy"]    = e.obs_policy;
        item["control_points"] = json::array();
        for (const auto& cp : e.control_points) {
            item["control_points"].push_back({cp.first, cp.second});
        }
        if (e.hold_direction) {
            item["hold_direction"] = *e.hold_direction;
        } else {
            item["hold_direction"] = nullptr;
        }
        item["backward"] = e.backward;
        if (e.padding) {
            item["padding"] = *e.padding;
        } else {
            item["padding"] = nullptr;
        }
        item["mode"] = e.mode;
        rg["edges"].push_back(item);
    }
    j["route_graph"] = rg;

    // ── dsg.layers.ROOMS（v2.0 新增）────────────────────────────
    json dsg;
    dsg["metadata"] = {
        {"version", "1.0"},
        {"name", "dexe_dsg"},
        {"schema_compatible_with", "spark_dsg/1.2"}
    };
    json layers;
    json rooms_arr = json::array();
    for (const auto& [name, r] : rooms) {
        json item;
        item["id"]                   = r.id;
        item["source"]               = r.source;
        item["created_at_ns"]        = r.created_at_ns;
        item["updated_at_ns"]        = r.updated_at_ns;
        item["name"]                 = r.name;
        item["type"]                 = RoomType::toString(r.type);
        item["semantic_label"]       = r.semantic_label;
        item["display_label"]        = r.display_label;
        item["manual_anchor_point_names"]   = r.manual_anchor_point_names;
        // 行为约束（内嵌约束子集，方案 D）
        item["behavior"] = json{
            {"max_speed_m_s",         r.behavior.max_speed_m_s},
            {"restricted",            r.behavior.restricted},
            {"detour_enabled",        r.behavior.detour_enabled},
            {"stop_distance_m",       r.behavior.stop_distance_m},
            {"depth_min_height_m",    r.behavior.depth_min_height_m},
            {"additional_area_names", r.behavior.additional_area_names}
        };
        item["attribute"]            = r.attribute;
        item["external_dsg_node_id"] = r.external_dsg_node_id;

        json pose;
        pose["position"]    = { {"x", r.pose.position.x},
                                {"y", r.pose.position.y},
                                {"z", r.pose.position.z} };
        pose["orientation"] = { {"x", r.pose.orientation.x},
                                {"y", r.pose.orientation.y},
                                {"z", r.pose.orientation.z},
                                {"w", r.pose.orientation.w} };
        item["pose"] = pose;

        json polygon = json::array();
        for (const auto& pt : r.polygon) {
            polygon.push_back({ {"x", pt.x}, {"y", pt.y} });
        }
        item["polygon"] = polygon;

        item["bounding_box"] = { r.bounding_box[0], r.bounding_box[1], r.bounding_box[2],
                                 r.bounding_box[3], r.bounding_box[4], r.bounding_box[5] };

        // 自动空间归属（SpatialIndexer 维护）
        item["related_nav_point_names"] = r.related_nav_point_names;
        item["related_junction_ids"]    = r.related_junction_ids;

        rooms_arr.push_back(item);
    }
    layers["ROOMS"]     = rooms_arr;
    layers["PLACES"]    = json::array();  // Phase 2 填
    layers["FLOORS"]    = json::array();  // Phase N
    layers["OBJECTS"]   = json::array();
    layers["BUILDINGS"] = json::array();
    dsg["layers"] = layers;
    dsg["edges"]  = json::array();        // Phase 2 填
    j["dsg"] = dsg;

    j["schema_version"]          = "2.0";
    j["schema_compatible_with"]  = "spark_dsg/1.2";
    return j;
}

std::string LocalTimeIso8601() {
    const std::time_t now = std::time(nullptr);
    struct tm tm_buf {};
    localtime_r(&now, &tm_buf);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &tm_buf);
    return buf;
}

std::time_t AssetsLastEditedAt(const std::string& assets_file) {
    std::error_code ec;
    if (!std::filesystem::exists(assets_file, ec)) return 0;
    try {
        std::ifstream in(assets_file);
        const json j = json::parse(in);
        if (j.contains("last_edited_at") && j["last_edited_at"].is_string()) {
            struct tm tm_buf {};
            const std::string s = j["last_edited_at"].get<std::string>();
            if (::strptime(s.c_str(), "%Y-%m-%dT%H:%M:%S", &tm_buf) != nullptr) {
                tm_buf.tm_isdst = -1;
                const std::time_t t = std::mktime(&tm_buf);
                if (t > 0) return t;
            }
        }
    } catch (const std::exception&) {
        // 读不出就退回文件时间
    }
    // 旧文件没有该键：退回文件修改时间
    struct stat st {};
    return ::stat(assets_file.c_str(), &st) == 0 ? st.st_mtime : 0;
}

bool FileManager::saveAllAssetsToFile(const std::string& file_path,
                                      const std::unordered_map<std::string, NavPoint>& points,
                                      const std::unordered_map<std::string, Route>& routes,
                                      const std::unordered_map<std::string, Area>& areas,
                                      const std::vector<RouteJunction>& junctions,
                                      const std::vector<RouteEdge>& edges,
                                      const std::unordered_map<std::string, Room>& rooms,
                                      const std::string& last_edited_at) {
    std::lock_guard<std::mutex> lock(file_mutex_);
    try {
        json j = buildAssetsJson(points, routes, areas, junctions, edges, rooms);
        j["last_edited_at"] = last_edited_at.empty() ? LocalTimeIso8601() : last_edited_at;
        // 首次写一张没有资产的图时 assets/ 目录还不存在
        std::error_code dir_ec;
        std::filesystem::create_directories(std::filesystem::path(file_path).parent_path(), dir_ec);

        // 原子写（DEC-004 资产引擎原子安全）：先写 <file>.tmp，再同卷 rename 覆盖（POSIX 原子），
        // 避免崩溃中途留下半写/损坏的 map_assets.json。
        const std::string tmp_path = file_path + ".tmp";
        {
            std::ofstream file(tmp_path, std::ios::trunc);
            if (!file.is_open()) {
                RCLCPP_ERROR(logger_, "Failed to open for writing: %s", tmp_path.c_str());
                return false;
            }
            file << j.dump(2);
            file.flush();
            if (!file.good()) {
                RCLCPP_ERROR(logger_, "Failed to write assets to %s", tmp_path.c_str());
                return false;
            }
        }  // ofstream 析构 → 关闭
        std::error_code ec;
        std::filesystem::rename(tmp_path, file_path, ec);
        if (ec) {
            RCLCPP_ERROR(logger_, "Failed to atomically rename %s -> %s: %s",
                         tmp_path.c_str(), file_path.c_str(), ec.message().c_str());
            std::filesystem::remove(tmp_path, ec);
            return false;
        }
        RCLCPP_INFO(logger_,
                    "Saved assets to %s: %zu points, %zu routes, %zu areas, %zu junctions, %zu edges, %zu rooms",
                    file_path.c_str(), points.size(), routes.size(), areas.size(),
                    junctions.size(), edges.size(), rooms.size());
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Exception saving assets file: %s", e.what());
        return false;
    }
}

}  // namespace map_asset_manager
