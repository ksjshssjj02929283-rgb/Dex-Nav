#ifndef MAP_ASSET_MANAGER__NAV_POINT_MANAGER_HPP_
#define MAP_ASSET_MANAGER__NAV_POINT_MANAGER_HPP_

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <visualization_msgs/msg/marker_array.hpp>

#include <nav_interfaces/msg/asset_snapshot.hpp>
#include "nav_interfaces/srv/get_all_nav_points.hpp"

#include "map_asset_manager/data_event_bus.hpp"
#include "map_asset_manager/point_manager/point_store.hpp"
#include "map_asset_manager/types.hpp"

namespace map_asset_manager {

// 导航点子管理器：发布点位可视化（读写统一走 get_assets / set_assets，导航由 navigation 门面负责）
class NavPointManager {
public:

    // save_fn 由 MapAssetManagerNode 提供，保存全局统一资产文件
    NavPointManager(rclcpp::Node* node,
                       std::shared_ptr<PointStore> point_store,
                       std::shared_ptr<DataEventBus> event_bus,
                       std::function<void()> save_fn);

    // 供主节点访问：发布点位可视化 Marker
    void publishMarkers();

private:
    // 兼容 dexe_robot_client 的导航客户端：只读全部点位（新调用方用 get_assets）
    void getAllPointsCallback(const std::shared_ptr<nav_interfaces::srv::GetAllNavPoints::Request> req,
                              std::shared_ptr<nav_interfaces::srv::GetAllNavPoints::Response> res);

    // 辅助
    visualization_msgs::msg::Marker createPointMarker(const NavPoint& point, int id);
    std_msgs::msg::ColorRGBA getColorForType(const std::string& type);

    rclcpp::Node* node_;
    std::shared_ptr<PointStore> point_store_;
    std::shared_ptr<DataEventBus> event_bus_;
    std::function<void()> save_fn_;

    rclcpp::Service<nav_interfaces::srv::GetAllNavPoints>::SharedPtr get_all_points_srv_;

    // Publisher
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    int prev_marker_count_{0};
};

}  // namespace map_asset_manager

#endif  // MAP_ASSET_MANAGER__NAV_POINT_MANAGER_HPP_
