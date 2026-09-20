#include "map_asset_manager/route_manager/route_store.hpp"

#include <algorithm>

namespace map_asset_manager {

RouteStore::RouteStore(rclcpp::Logger logger) : logger_(logger) {}

bool RouteStore::addRoute(const Route& route) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (routes_map_.count(route.name)) {
        RCLCPP_WARN(logger_, "Route '%s' already exists", route.name.c_str());
        return false;
    }
    routes_map_[route.name] = route;
    RCLCPP_INFO(logger_, "Added route: %s", route.name.c_str());
    return true;
}

bool RouteStore::removeRoute(const std::string& name) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = routes_map_.find(name);
    if (it == routes_map_.end()) {
        RCLCPP_WARN(logger_, "Route '%s' not found", name.c_str());
        return false;
    }
    routes_map_.erase(it);
    RCLCPP_INFO(logger_, "Removed route: %s", name.c_str());
    return true;
}

bool RouteStore::updateRoute(const Route& route) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!routes_map_.count(route.name)) {
        RCLCPP_WARN(logger_, "Route '%s' not found for update", route.name.c_str());
        return false;
    }
    routes_map_[route.name] = route;
    RCLCPP_INFO(logger_, "Updated route: %s", route.name.c_str());
    return true;
}

std::optional<Route> RouteStore::getRoute(const std::string& name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = routes_map_.find(name);
    if (it == routes_map_.end()) return std::nullopt;
    return it->second;
}

std::vector<Route> RouteStore::getAllRoutes() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<Route> result;
    result.reserve(routes_map_.size());
    for (const auto& [name, r] : routes_map_) {
        result.push_back(r);
    }
    return result;
}

bool RouteStore::hasRoute(const std::string& name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return routes_map_.count(name) > 0;
}

void RouteStore::setRoutes(const std::unordered_map<std::string, Route>& routes) {
    std::lock_guard<std::mutex> lock(mutex_);
    routes_map_ = routes;
}

std::unordered_map<std::string, Route> RouteStore::getRoutesMap() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return routes_map_;
}

std::vector<std::string> RouteStore::onPointRemoved(const std::string& point_name) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> modified;
    for (auto& [name, route] : routes_map_) {
        auto& pts = route.point_names;
        size_t before = pts.size();
        pts.erase(std::remove(pts.begin(), pts.end(), point_name), pts.end());
        if (pts.size() != before) {
            modified.push_back(name);
            RCLCPP_INFO(logger_, "Route '%s': removed deleted point '%s'", name.c_str(), point_name.c_str());
        }
    }
    return modified;
}

std::vector<std::string> RouteStore::onPointRenamed(const std::string& old_name, const std::string& new_name) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> modified;
    for (auto& [name, route] : routes_map_) {
        bool changed = false;
        for (auto& pt : route.point_names) {
            if (pt == old_name) {
                pt = new_name;
                changed = true;
            }
        }
        if (changed) {
            modified.push_back(name);
            RCLCPP_INFO(logger_, "Route '%s': renamed point '%s' -> '%s'",
                        name.c_str(), old_name.c_str(), new_name.c_str());
        }
    }
    return modified;
}

void RouteStore::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    routes_map_.clear();
}

}  // namespace map_asset_manager
