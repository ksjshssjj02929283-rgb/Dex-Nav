#ifndef MAP_ASSET_MANAGER__ROUTE_MANAGER_HPP_
#define MAP_ASSET_MANAGER__ROUTE_MANAGER_HPP_

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <visualization_msgs/msg/marker_array.hpp>

#include <nav_interfaces/msg/asset_snapshot.hpp>
#include "nav_interfaces/msg/route.hpp"
#include "nav_interfaces/srv/get_route.hpp"

#include "map_asset_manager/data_event_bus.hpp"
#include "map_asset_manager/point_manager/point_store.hpp"
#include "map_asset_manager/route_manager/route_store.hpp"
#include "map_asset_manager/types.hpp"

namespace map_asset_manager {

// 路线子管理器：注册 Route 相关的 Service（导航 action 已归 navigation 门面，DEC-041）
class RouteManager {
public:

    // save_fn 由 MapAssetManagerNode 提供，保存全局统一资产文件
    RouteManager(rclcpp::Node* node,
                    std::shared_ptr<RouteStore> route_store,
                    std::shared_ptr<PointStore> point_store,
                    std::shared_ptr<DataEventBus> event_bus,
                    std::function<void()> save_fn);

    // 发布所有路线的持久化 marker（set_route 后调用）
    void publishAllRouteMarkers();

    // EventBus 响应接口（由 MapAssetManagerNode 通过订阅事件调用）
    void onPointRemoved(const std::string& point_name);
    void onPointRenamed(const std::string& old_name, const std::string& new_name);

private:
    // Service 回调
    void getRouteCallback(const std::shared_ptr<nav_interfaces::srv::GetRoute::Request> req,
                          std::shared_ptr<nav_interfaces::srv::GetRoute::Response> res);
    // 辅助
    void saveToFile();

    rclcpp::Node* node_;
    std::shared_ptr<RouteStore> route_store_;
    std::shared_ptr<PointStore> point_store_;     // 仅在执行导航时查坐标
    std::shared_ptr<DataEventBus> event_bus_;
    std::function<void()> save_fn_;

    // Services
    rclcpp::Service<nav_interfaces::srv::GetRoute>::SharedPtr get_route_srv_;

    // Publisher
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr route_marker_pub_;
};

}  // namespace map_asset_manager

#endif  // MAP_ASSET_MANAGER__ROUTE_MANAGER_HPP_
