#include "map_asset_manager/area_manager/area_store.hpp"

namespace map_asset_manager {

AreaStore::AreaStore(rclcpp::Logger logger) : logger_(logger) {}

bool AreaStore::addArea(const Area& area) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (areas_map_.count(area.name)) {
        RCLCPP_WARN(logger_, "Area '%s' already exists", area.name.c_str());
        return false;
    }
    areas_map_[area.name] = area;
    RCLCPP_INFO(logger_, "Added area: %s", area.name.c_str());
    return true;
}

bool AreaStore::removeArea(const std::string& name) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = areas_map_.find(name);
    if (it == areas_map_.end()) {
        RCLCPP_WARN(logger_, "Area '%s' not found", name.c_str());
        return false;
    }
    areas_map_.erase(it);
    RCLCPP_INFO(logger_, "Removed area: %s", name.c_str());
    return true;
}

bool AreaStore::updateArea(const Area& area) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!areas_map_.count(area.name)) {
        RCLCPP_WARN(logger_, "Area '%s' not found for update", area.name.c_str());
        return false;
    }
    areas_map_[area.name] = area;
    RCLCPP_INFO(logger_, "Updated area: %s", area.name.c_str());
    return true;
}

std::optional<Area> AreaStore::getArea(const std::string& name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = areas_map_.find(name);
    if (it == areas_map_.end()) return std::nullopt;
    return it->second;
}

std::vector<Area> AreaStore::getAllAreas() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<Area> result;
    result.reserve(areas_map_.size());
    for (const auto& [name, a] : areas_map_) {
        result.push_back(a);
    }
    return result;
}

bool AreaStore::hasArea(const std::string& name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return areas_map_.count(name) > 0;
}

void AreaStore::setAreas(const std::unordered_map<std::string, Area>& areas) {
    std::lock_guard<std::mutex> lock(mutex_);
    areas_map_ = areas;
}

std::unordered_map<std::string, Area> AreaStore::getAreasMap() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return areas_map_;
}

void AreaStore::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    areas_map_.clear();
}

}  // namespace map_asset_manager
