#include "nav_point_manager/point_database.hpp"

namespace nav_point_manager {

PointDatabase::PointDatabase(rclcpp::Logger logger) : logger_(logger) {}

int PointDatabase::addPoint(const NavPoint& point) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    auto it = points_map_.find(point.name);
    if (it != points_map_.end()) {
        // 覆盖已存在的点位
        points_map_[point.name] = point;
        RCLCPP_INFO(logger_, "Overwritten point: %s", point.name.c_str());
        return 2;  // 覆盖
    }

    // 新增点位
    points_map_[point.name] = point;
    RCLCPP_INFO(logger_, "Added point: %s", point.name.c_str());
    return 1;  // 新增
}

bool PointDatabase::removePoint(const std::string& name) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    auto it = points_map_.find(name);
    if (it == points_map_.end()) {
        RCLCPP_WARN(logger_, "Point '%s' not found", name.c_str());
        return false;
    }

    points_map_.erase(it);
    RCLCPP_INFO(logger_, "Removed point: %s", name.c_str());
    return true;
}

bool PointDatabase::updatePoint(const NavPoint& point) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    auto it = points_map_.find(point.name);
    if (it == points_map_.end()) {
        RCLCPP_WARN(logger_, "Point '%s' not found for update", point.name.c_str());
        return false;
    }

    points_map_[point.name] = point;
    RCLCPP_INFO(logger_, "Updated point: %s", point.name.c_str());
    return true;
}

std::optional<NavPoint> PointDatabase::getPoint(const std::string& name) const {
    std::lock_guard<std::mutex> lock(data_mutex_);

    auto it = points_map_.find(name);
    if (it == points_map_.end()) {
        return std::nullopt;
    }

    return it->second;
}

std::vector<NavPoint> PointDatabase::getAllPoints() const {
    std::lock_guard<std::mutex> lock(data_mutex_);

    std::vector<NavPoint> points;
    points.reserve(points_map_.size());

    for (const auto& [name, point] : points_map_) {
        points.push_back(point);
    }

    return points;
}

bool PointDatabase::hasPoint(const std::string& name) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return points_map_.find(name) != points_map_.end();
}

bool PointDatabase::addRoute(const Route& route) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (routes_map_.find(route.name) != routes_map_.end()) {
        RCLCPP_WARN(logger_, "Route '%s' already exists", route.name.c_str());
        return false;
    }

    routes_map_[route.name] = route;
    RCLCPP_INFO(logger_, "Added route: %s", route.name.c_str());
    return true;
}

bool PointDatabase::removeRoute(const std::string& name) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    auto it = routes_map_.find(name);
    if (it == routes_map_.end()) {
        RCLCPP_WARN(logger_, "Route '%s' not found", name.c_str());
        return false;
    }

    routes_map_.erase(it);
    RCLCPP_INFO(logger_, "Removed route: %s", name.c_str());
    return true;
}

std::optional<Route> PointDatabase::getRoute(const std::string& name) const {
    std::lock_guard<std::mutex> lock(data_mutex_);

    auto it = routes_map_.find(name);
    if (it == routes_map_.end()) {
        return std::nullopt;
    }

    return it->second;
}

std::vector<Route> PointDatabase::getAllRoutes() const {
    std::lock_guard<std::mutex> lock(data_mutex_);

    std::vector<Route> routes;
    routes.reserve(routes_map_.size());

    for (const auto& [name, route] : routes_map_) {
        routes.push_back(route);
    }

    return routes;
}

bool PointDatabase::hasRoute(const std::string& name) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return routes_map_.find(name) != routes_map_.end();
}

void PointDatabase::setPoints(const std::unordered_map<std::string, NavPoint>& points) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    points_map_ = points;
}

void PointDatabase::setRoutes(const std::unordered_map<std::string, Route>& routes) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    routes_map_ = routes;
}

std::unordered_map<std::string, NavPoint> PointDatabase::getPointsMap() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return points_map_;
}

std::unordered_map<std::string, Route> PointDatabase::getRoutesMap() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return routes_map_;
}

void PointDatabase::clear() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    points_map_.clear();
    routes_map_.clear();
    RCLCPP_INFO(logger_, "Database cleared");
}

}  // namespace nav_point_manager
