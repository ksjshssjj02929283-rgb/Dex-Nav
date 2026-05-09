#include "nav_point_manager/file_manager.hpp"

#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace nav_point_manager {

FileManager::FileManager(rclcpp::Logger logger) : logger_(logger) {}

bool FileManager::loadPointsFromFile(const std::string& file_path,
                                     std::unordered_map<std::string, NavPoint>& points_map) {
    std::lock_guard<std::mutex> lock(file_mutex_);

    try {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(logger_, "Failed to open points file: %s", file_path.c_str());
            return false;
        }

        json j;
        file >> j;
        file.close();

        points_map.clear();

        if (!j.contains("points") || !j["points"].is_array()) {
            RCLCPP_ERROR(logger_, "Invalid JSON format: missing 'points' array");
            return false;
        }

        for (const auto& point_json : j["points"]) {
            NavPoint point;
            point.name = point_json["name"].get<std::string>();
            point.type = point_json["type"].get<std::string>();
            point.tolerance = point_json["tolerance"].get<double>();
            point.x = point_json["pose"]["x"].get<double>();
            point.y = point_json["pose"]["y"].get<double>();
            point.yaw = point_json["pose"]["yaw"].get<double>();

            points_map[point.name] = point;
        }

        RCLCPP_INFO(logger_, "Loaded %zu points from %s", points_map.size(), file_path.c_str());
        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Exception loading points: %s", e.what());
        return false;
    }
}

bool FileManager::savePointsToFile(const std::string& file_path,
                                   const std::unordered_map<std::string, NavPoint>& points_map) {
    std::lock_guard<std::mutex> lock(file_mutex_);

    try {
        json j;
        j["points"] = json::array();

        for (const auto& [name, point] : points_map) {
            json point_json;
            point_json["name"] = point.name;
            point_json["type"] = point.type;
            point_json["tolerance"] = point.tolerance;
            point_json["pose"]["x"] = point.x;
            point_json["pose"]["y"] = point.y;
            point_json["pose"]["yaw"] = point.yaw;

            j["points"].push_back(point_json);
        }

        std::ofstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(logger_, "Failed to open file for writing: %s", file_path.c_str());
            return false;
        }

        file << j.dump(2);
        file.close();

        RCLCPP_INFO(logger_, "Saved %zu points to %s", points_map.size(), file_path.c_str());
        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Exception saving points: %s", e.what());
        return false;
    }
}

bool FileManager::loadRoutesFromFile(const std::string& file_path, std::unordered_map<std::string, Route>& routes_map) {
    std::lock_guard<std::mutex> lock(file_mutex_);

    try {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_WARN(logger_, "Routes file not found: %s (this is optional)", file_path.c_str());
            return true;  // Routes are optional
        }

        json j;
        file >> j;
        file.close();

        routes_map.clear();

        if (!j.contains("routes") || !j["routes"].is_array()) {
            RCLCPP_WARN(logger_, "Invalid JSON format: missing 'routes' array");
            return true;
        }

        for (const auto& route_json : j["routes"]) {
            Route route;
            route.name = route_json["name"].get<std::string>();
            route.point_names = route_json["points"].get<std::vector<std::string>>();

            routes_map[route.name] = route;
        }

        RCLCPP_INFO(logger_, "Loaded %zu routes from %s", routes_map.size(), file_path.c_str());
        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Exception loading routes: %s", e.what());
        return false;
    }
}

bool FileManager::saveRoutesToFile(const std::string& file_path,
                                   const std::unordered_map<std::string, Route>& routes_map) {
    std::lock_guard<std::mutex> lock(file_mutex_);

    try {
        json j;
        j["routes"] = json::array();

        for (const auto& [name, route] : routes_map) {
            json route_json;
            route_json["name"] = route.name;
            route_json["points"] = route.point_names;

            j["routes"].push_back(route_json);
        }

        std::ofstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(logger_, "Failed to open file for writing: %s", file_path.c_str());
            return false;
        }

        file << j.dump(2);
        file.close();

        RCLCPP_INFO(logger_, "Saved %zu routes to %s", routes_map.size(), file_path.c_str());
        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Exception saving routes: %s", e.what());
        return false;
    }
}

}  // namespace nav_point_manager

#include <filesystem>

namespace nav_point_manager {

bool FileManager::loadMapFolder(const std::string& base_path,
                                const std::string& folder_name,
                                std::string& points_file,
                                std::string& routes_file) {
    std::lock_guard<std::mutex> lock(file_mutex_);

    std::string folder_path = base_path + "/" + folder_name;
    points_file = folder_path + "/points.json";
    routes_file = folder_path + "/routes.json";

    if (!std::filesystem::exists(folder_path)) {
        RCLCPP_ERROR(logger_, "Map folder does not exist: %s", folder_path.c_str());
        return false;
    }

    RCLCPP_INFO(logger_, "Loading map folder: %s", folder_path.c_str());
    RCLCPP_INFO(logger_, "  Points file: %s", points_file.c_str());
    RCLCPP_INFO(logger_, "  Routes file: %s", routes_file.c_str());

    return true;
}

}  // namespace nav_point_manager
