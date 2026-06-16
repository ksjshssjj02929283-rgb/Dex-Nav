#include "map_asset_manager/route_graph_manager/route_edge_store.hpp"

namespace map_asset_manager {

RouteEdgeStore::RouteEdgeStore(rclcpp::Logger logger) : logger_(logger) {}

bool RouteEdgeStore::add(const RouteEdge& edge) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (map_.count(edge.id)) {
        RCLCPP_WARN(logger_, "RouteEdgeStore: id '%s' already exists", edge.id.c_str());
        return false;
    }
    map_[edge.id] = edge;
    return true;
}

bool RouteEdgeStore::remove(const std::string& id) {
    std::lock_guard<std::mutex> lock(mutex_);
    return map_.erase(id) > 0;
}

bool RouteEdgeStore::update(const RouteEdge& edge) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!map_.count(edge.id)) return false;
    map_[edge.id] = edge;
    return true;
}

std::optional<RouteEdge> RouteEdgeStore::get(const std::string& id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = map_.find(id);
    if (it == map_.end()) return std::nullopt;
    return it->second;
}

std::vector<RouteEdge> RouteEdgeStore::getAll() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<RouteEdge> result;
    result.reserve(map_.size());
    for (const auto& [id, e] : map_) {
        result.push_back(e);
    }
    return result;
}

bool RouteEdgeStore::has(const std::string& id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return map_.count(id) > 0;
}

void RouteEdgeStore::set(const std::vector<RouteEdge>& edges) {
    std::lock_guard<std::mutex> lock(mutex_);
    map_.clear();
    for (const auto& e : edges) {
        map_[e.id] = e;
    }
}

std::vector<std::string> RouteEdgeStore::removeEdgesContainingNav(const std::string& nav_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> removed;
    for (auto it = map_.begin(); it != map_.end();) {
        const auto& e = it->second;
        bool refs = (e.from.type == "nav" && e.from.id == nav_id) || (e.to.type == "nav" && e.to.id == nav_id);
        if (refs) {
            removed.push_back(it->first);
            it = map_.erase(it);
        } else {
            ++it;
        }
    }
    return removed;
}

std::vector<std::string> RouteEdgeStore::removeEdgesContainingJunction(const std::string& jct_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> removed;
    for (auto it = map_.begin(); it != map_.end();) {
        const auto& e = it->second;
        bool refs =
            (e.from.type == "junction" && e.from.id == jct_id) || (e.to.type == "junction" && e.to.id == jct_id);
        if (refs) {
            removed.push_back(it->first);
            it = map_.erase(it);
        } else {
            ++it;
        }
    }
    return removed;
}

void RouteEdgeStore::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    map_.clear();
}

}  // namespace map_asset_manager
