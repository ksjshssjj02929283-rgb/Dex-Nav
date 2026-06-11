#include "map_asset_manager/file_manager.hpp"

#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace map_asset_manager {

FileManager::FileManager(rclcpp::Logger logger) : logger_(logger) {}

// ──────────────────────────────────────────────────────────────────────
// 导航点
// ──────────────────────────────────────────────────────────────────────

bool FileManager::loadPointsFromFile(const std::string& file_path, std::unordered_map<std::string, NavPoint>& out) {
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
            pt.name = item["name"].get<std::string>();
            pt.type = item["type"].get<std::string>();
            pt.tolerance = item["tolerance"].get<double>();
            pt.x = item["pose"]["x"].get<double>();
            pt.y = item["pose"]["y"].get<double>();
            pt.yaw = item["pose"]["yaw"].get<double>();
            out[pt.name] = pt;
        }

        RCLCPP_INFO(logger_, "Loaded %zu points from %s", out.size(), file_path.c_str());
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Exception loading points: %s", e.what());
        return false;
    }
}

bool FileManager::savePointsToFile(const std::string& file_path, const std::unordered_map<std::string, NavPoint>& in) {
    std::lock_guard<std::mutex> lock(file_mutex_);
    try {
        json j;
        j["points"] = json::array();
        for (const auto& [name, pt] : in) {
            json item;
            item["name"] = pt.name;
            item["type"] = pt.type;
            item["tolerance"] = pt.tolerance;
            item["pose"]["x"] = pt.x;
            item["pose"]["y"] = pt.y;
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

bool FileManager::loadRoutesFromFile(const std::string& file_path, std::unordered_map<std::string, Route>& out) {
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
            r.name = item["name"].get<std::string>();
            r.point_names = item["points"].get<std::vector<std::string>>();
            out[r.name] = r;
        }

        RCLCPP_INFO(logger_, "Loaded %zu routes from %s", out.size(), file_path.c_str());
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Exception loading routes: %s", e.what());
        return false;
    }
}

bool FileManager::saveRoutesToFile(const std::string& file_path, const std::unordered_map<std::string, Route>& in) {
    std::lock_guard<std::mutex> lock(file_mutex_);
    try {
        json j;
        j["routes"] = json::array();
        for (const auto& [name, r] : in) {
            json item;
            item["name"] = r.name;
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

bool FileManager::loadAreasFromFile(const std::string& file_path, std::unordered_map<std::string, Area>& out) {
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
            a.name = item["name"].get<std::string>();
            a.type = AreaType::fromString(item.value("type", std::string{}));
            a.max_speed = item.value("max_speed", 0.0f);
            a.stop_distance = item.value("stop_distance", 0.0f);
            a.linked_point_name = item.value("linked_point_name", std::string{});
            a.charger_type = item.value("charger_type", std::string{});
            a.depth_min_height = item.value("depth_min_height", 0.1f);
            a.attribute = item.value("attribute", std::string{});
            if (item.contains("polygon") && item["polygon"].is_array()) {
                for (const auto& pt : item["polygon"]) {
                    geometry_msgs::msg::Point p;
                    p.x = pt["x"].get<double>();
                    p.y = pt["y"].get<double>();
                    p.z = 0.0;
                    a.polygon.push_back(p);
                }
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

bool FileManager::saveAreasToFile(const std::string& file_path, const std::unordered_map<std::string, Area>& in) {
    std::lock_guard<std::mutex> lock(file_mutex_);
    try {
        json j;
        j["areas"] = json::array();
        for (const auto& [name, a] : in) {
            json item;
            item["name"] = a.name;
            item["type"] = AreaType::toString(a.type);
            item["max_speed"] = a.max_speed;
            item["stop_distance"] = a.stop_distance;
            item["linked_point_name"] = a.linked_point_name;
            item["charger_type"] = a.charger_type;
            item["attribute"] = a.attribute;
            item["polygon"] = json::array();
            for (const auto& pt : a.polygon) {
                json p;
                p["x"] = pt.x;
                p["y"] = pt.y;
                item["polygon"].push_back(p);
            }
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
    areas_file = folder_path + "/areas.json";
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
                                        std::vector<RouteEdge>& edges) {
    std::lock_guard<std::mutex> lock(file_mutex_);
    points.clear();
    routes.clear();
    areas.clear();
    junctions.clear();
    edges.clear();

    if (!std::filesystem::exists(file_path)) {
        RCLCPP_WARN(logger_, "Assets file not found (will be created on first save): %s", file_path.c_str());
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
                pt.name = item["name"].get<std::string>();
                pt.type = item["type"].get<std::string>();
                pt.tolerance = item["tolerance"].get<double>();
                pt.x = item["pose"]["x"].get<double>();
                pt.y = item["pose"]["y"].get<double>();
                pt.yaw = item["pose"]["yaw"].get<double>();
                points[pt.name] = pt;
            }
        }

        // ── routes ──────────────────────────────────────────────
        if (j.contains("routes") && j["routes"].is_array()) {
            for (const auto& item : j["routes"]) {
                Route r;
                r.name = item["name"].get<std::string>();
                r.point_names = item["points"].get<std::vector<std::string>>();
                routes[r.name] = r;
            }
        }

        // ── areas ───────────────────────────────────────────────
        if (j.contains("areas") && j["areas"].is_array()) {
            for (const auto& item : j["areas"]) {
                Area a;
                a.name = item["name"].get<std::string>();
                a.type = AreaType::fromString(item.value("type", std::string{}));
                a.max_speed = item.value("max_speed", 0.0f);
                a.stop_distance = item.value("stop_distance", 0.0f);
                a.linked_point_name = item.value("linked_point_name", std::string{});
                a.charger_type = item.value("charger_type", std::string{});
                a.attribute = item.value("attribute", std::string{});
                if (item.contains("polygon") && item["polygon"].is_array()) {
                    for (const auto& pt : item["polygon"]) {
                        geometry_msgs::msg::Point p;
                        p.x = pt["x"].get<double>();
                        p.y = pt["y"].get<double>();
                        p.z = 0.0;
                        a.polygon.push_back(p);
                    }
                }
                areas[a.name] = a;
            }
        }

        RCLCPP_INFO(logger_, "Loaded assets from %s: %zu points, %zu routes, %zu areas", file_path.c_str(),
                    points.size(), routes.size(), areas.size());

        // ── route_graph ──────────────────────────────────────────
        if (j.contains("route_graph")) {
            const auto& rg = j["route_graph"];
            if (rg.contains("junctions") && rg["junctions"].is_array()) {
                for (const auto& item : rg["junctions"]) {
                    RouteJunction jct;
                    jct.id = item["id"].get<std::string>();
                    jct.x = item.value("x", 0.0f);
                    jct.y = item.value("y", 0.0f);
                    jct.yaw = item.value("yaw", 0.0f);
                    junctions.push_back(jct);
                }
            }
            if (rg.contains("edges") && rg["edges"].is_array()) {
                for (const auto& item : rg["edges"]) {
                    RouteEdge e;
                    e.id = item["id"].get<std::string>();
                    e.from.id = item["from"]["id"].get<std::string>();
                    e.from.type = item["from"]["type"].get<std::string>();
                    e.to.id = item["to"]["id"].get<std::string>();
                    e.to.type = item["to"]["type"].get<std::string>();
                    e.edge_type = item.value("edge_type", std::string{"LINE"});
                    e.bidirectional = item.value("bidirectional", false);
                    e.max_speed = item.value("max_speed", 0.0f);
                    e.xy_tolerance = item.value("xy_tolerance", 0.3f);
                    e.th_tolerance = item.value("th_tolerance", 0.5f);
                    e.lane_width = item.value("lane_width", 0.0f);
                    e.obs_policy = item.value("obs_policy", std::string{"STOP"});
                    if (item.contains("control_points") && item["control_points"].is_array()) {
                        for (const auto& cp : item["control_points"]) {
                            e.control_points.emplace_back(cp[0].get<float>(), cp[1].get<float>());
                        }
                    }
                    edges.push_back(e);
                }
            }
        }

        RCLCPP_INFO(logger_, "Loaded route_graph: %zu junctions, %zu edges", junctions.size(), edges.size());
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Exception loading assets file: %s", e.what());
        return false;
    }
}

bool FileManager::saveAllAssetsToFile(const std::string& file_path,
                                      const std::unordered_map<std::string, NavPoint>& points,
                                      const std::unordered_map<std::string, Route>& routes,
                                      const std::unordered_map<std::string, Area>& areas,
                                      const std::vector<RouteJunction>& junctions,
                                      const std::vector<RouteEdge>& edges) {
    std::lock_guard<std::mutex> lock(file_mutex_);
    try {
        json j;

        // ── points ──────────────────────────────────────────────
        j["points"] = json::array();
        for (const auto& [name, pt] : points) {
            json item;
            item["name"] = pt.name;
            item["type"] = pt.type;
            item["tolerance"] = pt.tolerance;
            item["pose"]["x"] = pt.x;
            item["pose"]["y"] = pt.y;
            item["pose"]["yaw"] = pt.yaw;
            j["points"].push_back(item);
        }

        // ── routes ──────────────────────────────────────────────
        j["routes"] = json::array();
        for (const auto& [name, r] : routes) {
            json item;
            item["name"] = r.name;
            item["points"] = r.point_names;
            j["routes"].push_back(item);
        }

        // ── areas ───────────────────────────────────────────────
        j["areas"] = json::array();
        for (const auto& [name, a] : areas) {
            json item;
            item["name"] = a.name;
            item["type"] = AreaType::toString(a.type);
            item["max_speed"] = a.max_speed;
            item["stop_distance"] = a.stop_distance;
            item["linked_point_name"] = a.linked_point_name;
            item["charger_type"] = a.charger_type;
            item["attribute"] = a.attribute;
            item["polygon"] = json::array();
            for (const auto& pt : a.polygon) {
                json p;
                p["x"] = pt.x;
                p["y"] = pt.y;
                item["polygon"].push_back(p);
            }
            j["areas"].push_back(item);
        }

        std::ofstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(logger_, "Failed to open for writing: %s", file_path.c_str());
            return false;
        }

        // ── route_graph ──────────────────────────────────────────
        json rg;
        rg["junctions"] = json::array();
        for (const auto& jct : junctions) {
            json item;
            item["id"] = jct.id;
            item["x"] = jct.x;
            item["y"] = jct.y;
            item["yaw"] = jct.yaw;
            rg["junctions"].push_back(item);
        }
        rg["edges"] = json::array();
        for (const auto& e : edges) {
            json item;
            item["id"] = e.id;
            item["from"]["id"] = e.from.id;
            item["from"]["type"] = e.from.type;
            item["to"]["id"] = e.to.id;
            item["to"]["type"] = e.to.type;
            item["edge_type"] = e.edge_type;
            item["bidirectional"] = e.bidirectional;
            item["max_speed"] = e.max_speed;
            item["xy_tolerance"] = e.xy_tolerance;
            item["th_tolerance"] = e.th_tolerance;
            item["lane_width"] = e.lane_width;
            item["obs_policy"] = e.obs_policy;
            item["control_points"] = json::array();
            for (const auto& cp : e.control_points) {
                item["control_points"].push_back({cp.first, cp.second});
            }
            rg["edges"].push_back(item);
        }
        j["route_graph"] = rg;

        file << j.dump(2);
        RCLCPP_INFO(logger_, "Saved assets to %s: %zu points, %zu routes, %zu areas, %zu junctions, %zu edges",
                    file_path.c_str(), points.size(), routes.size(), areas.size(), junctions.size(), edges.size());
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Exception saving assets file: %s", e.what());
        return false;
    }
}

}  // namespace map_asset_manager
