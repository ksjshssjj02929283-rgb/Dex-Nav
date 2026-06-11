#ifndef MAP_ASSET_MANAGER__ROUTE_EDGE_STORE_HPP_
#define MAP_ASSET_MANAGER__ROUTE_EDGE_STORE_HPP_

#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "map_asset_manager/types.hpp"

namespace map_asset_manager {

// 路网边内存数据库
class RouteEdgeStore {
public:
    explicit RouteEdgeStore(rclcpp::Logger logger);

    bool add(const RouteEdge& edge);
    bool remove(const std::string& id);
    bool update(const RouteEdge& edge);
    std::optional<RouteEdge> get(const std::string& id) const;
    std::vector<RouteEdge> getAll() const;
    bool has(const std::string& id) const;

    void set(const std::vector<RouteEdge>& edges);

    // 级联删除：删除所有引用指定 nav 点的边，返回被删除的边 ID 列表
    std::vector<std::string> removeEdgesContainingNav(const std::string& nav_id);

    // 级联删除：删除所有引用指定 junction 的边，返回被删除的边 ID 列表
    std::vector<std::string> removeEdgesContainingJunction(const std::string& jct_id);

    void clear();

private:
    mutable std::mutex mutex_;
    std::unordered_map<std::string, RouteEdge> map_;
    rclcpp::Logger logger_;
};

}  // namespace map_asset_manager

#endif  // MAP_ASSET_MANAGER__ROUTE_EDGE_STORE_HPP_
