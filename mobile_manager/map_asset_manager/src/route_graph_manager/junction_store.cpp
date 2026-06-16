#include "map_asset_manager/route_graph_manager/junction_store.hpp"

namespace map_asset_manager {

JunctionStore::JunctionStore(rclcpp::Logger logger) : logger_(logger) {}

bool JunctionStore::add(const RouteJunction& jct) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (map_.count(jct.id)) {
        RCLCPP_WARN(logger_, "JunctionStore: id '%s' already exists", jct.id.c_str());
        return false;
    }
    map_[jct.id] = jct;
    return true;
}

bool JunctionStore::remove(const std::string& id) {
    std::lock_guard<std::mutex> lock(mutex_);
    return map_.erase(id) > 0;
}

bool JunctionStore::update(const RouteJunction& jct) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!map_.count(jct.id)) return false;
    map_[jct.id] = jct;
    return true;
}

std::optional<RouteJunction> JunctionStore::get(const std::string& id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = map_.find(id);
    if (it == map_.end()) return std::nullopt;
    return it->second;
}

std::vector<RouteJunction> JunctionStore::getAll() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<RouteJunction> result;
    result.reserve(map_.size());
    for (const auto& [id, jct] : map_) {
        result.push_back(jct);
    }
    return result;
}

bool JunctionStore::has(const std::string& id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return map_.count(id) > 0;
}

void JunctionStore::set(const std::vector<RouteJunction>& junctions) {
    std::lock_guard<std::mutex> lock(mutex_);
    map_.clear();
    for (const auto& jct : junctions) {
        map_[jct.id] = jct;
    }
}

std::vector<RouteJunction> JunctionStore::getList() const { return getAll(); }

void JunctionStore::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    map_.clear();
}

}  // namespace map_asset_manager
