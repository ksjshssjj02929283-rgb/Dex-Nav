#include "map_asset_manager/point_manager/point_store.hpp"

namespace map_asset_manager {

PointStore::PointStore(rclcpp::Logger logger) : logger_(logger) {}

int PointStore::addPoint(const NavPoint& point) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = points_map_.find(point.name);
    if (it != points_map_.end()) {
        points_map_[point.name] = point;
        RCLCPP_INFO(logger_, "Overwritten point: %s", point.name.c_str());
        return 2;
    }
    points_map_[point.name] = point;
    RCLCPP_INFO(logger_, "Added point: %s", point.name.c_str());
    return 1;
}

bool PointStore::removePoint(const std::string& name) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = points_map_.find(name);
    if (it == points_map_.end()) {
        RCLCPP_WARN(logger_, "Point '%s' not found", name.c_str());
        return false;
    }
    points_map_.erase(it);
    RCLCPP_INFO(logger_, "Removed point: %s", name.c_str());
    return true;
}

bool PointStore::updatePoint(const NavPoint& point) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = points_map_.find(point.name);
    if (it == points_map_.end()) {
        RCLCPP_WARN(logger_, "Point '%s' not found for update", point.name.c_str());
        return false;
    }
    points_map_[point.name] = point;
    RCLCPP_INFO(logger_, "Updated point: %s", point.name.c_str());
    return true;
}

std::optional<NavPoint> PointStore::getPoint(const std::string& name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = points_map_.find(name);
    if (it == points_map_.end()) return std::nullopt;
    return it->second;
}

std::vector<NavPoint> PointStore::getAllPoints() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<NavPoint> result;
    result.reserve(points_map_.size());
    for (const auto& [name, pt] : points_map_) {
        result.push_back(pt);
    }
    return result;
}

bool PointStore::hasPoint(const std::string& name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return points_map_.count(name) > 0;
}

void PointStore::setPoints(const std::unordered_map<std::string, NavPoint>& points) {
    std::lock_guard<std::mutex> lock(mutex_);
    points_map_ = points;
}

std::unordered_map<std::string, NavPoint> PointStore::getPointsMap() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return points_map_;
}

void PointStore::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    points_map_.clear();
}

}  // namespace map_asset_manager
