#ifndef MAP_ASSET_MANAGER__JUNCTION_STORE_HPP_
#define MAP_ASSET_MANAGER__JUNCTION_STORE_HPP_

#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "map_asset_manager/types.hpp"

namespace map_asset_manager {

// Junction 内存数据库（路网拓扑中间节点，RouteGraphManager 私有）
class JunctionStore {
public:
    explicit JunctionStore(rclcpp::Logger logger);

    bool add(const RouteJunction& jct);
    bool remove(const std::string& id);
    bool update(const RouteJunction& jct);
    std::optional<RouteJunction> get(const std::string& id) const;
    std::vector<RouteJunction> getAll() const;
    bool has(const std::string& id) const;

    void set(const std::vector<RouteJunction>& junctions);
    std::vector<RouteJunction> getList() const;

    void clear();

private:
    mutable std::mutex mutex_;
    std::unordered_map<std::string, RouteJunction> map_;
    rclcpp::Logger logger_;
};

}  // namespace map_asset_manager

#endif  // MAP_ASSET_MANAGER__JUNCTION_STORE_HPP_
