#ifndef MAP_ASSET_MANAGER__ROUTE_STORE_HPP_
#define MAP_ASSET_MANAGER__ROUTE_STORE_HPP_

#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "map_asset_manager/types.hpp"

namespace map_asset_manager {

// 路径内存数据库（只管 Route，不含导航点坐标）
// Route 通过 point_names 字符串列表引用导航点，坐标在执行时由 RouteManager 解析
class RouteStore {
public:
    explicit RouteStore(rclcpp::Logger logger);

    bool addRoute(const Route& route);
    bool removeRoute(const std::string& name);
    bool updateRoute(const Route& route);
    std::optional<Route> getRoute(const std::string& name) const;
    std::vector<Route> getAllRoutes() const;
    bool hasRoute(const std::string& name) const;

    // 批量操作（热重载时使用）
    void setRoutes(const std::unordered_map<std::string, Route>& routes);
    std::unordered_map<std::string, Route> getRoutesMap() const;

    // ──────────────────────────────────────────────────────────────
    // EventBus 响应接口：由 MapAssetManagerNode 通过订阅事件调用
    // ──────────────────────────────────────────────────────────────

    // 导航点被删除时：从所有路径的 point_names 中移除该点
    // 返回被修改的路径名列表（用于调用方持久化）
    std::vector<std::string> onPointRemoved(const std::string& point_name);

    // 导航点被重命名时：将所有路径中的 old_name 替换为 new_name
    // 返回被修改的路径名列表
    std::vector<std::string> onPointRenamed(const std::string& old_name, const std::string& new_name);

    void clear();

private:
    mutable std::mutex mutex_;
    std::unordered_map<std::string, Route> routes_map_;
    rclcpp::Logger logger_;
};

}  // namespace map_asset_manager

#endif  // MAP_ASSET_MANAGER__ROUTE_STORE_HPP_
