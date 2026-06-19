#ifndef MAP_ASSET_MANAGER__AREA_MANAGER_HPP_
#define MAP_ASSET_MANAGER__AREA_MANAGER_HPP_

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <nav_interfaces/msg/area_info.hpp>
#include <nav_interfaces/msg/area_info_array.hpp>
#include <nav_interfaces/srv/add_area_info.hpp>
#include <nav_interfaces/srv/get_all_areas.hpp>
#include <nav_interfaces/srv/remove_area_info.hpp>
#include <nav_interfaces/srv/update_area_info.hpp>

#include "map_asset_manager/area_manager/area_store.hpp"
#include "map_asset_manager/data_event_bus.hpp"
#include "map_asset_manager/types.hpp"

namespace map_asset_manager {

class AreaManager {
public:
    AreaManager(rclcpp::Node* node,
                std::shared_ptr<AreaStore> area_store,
                std::shared_ptr<DataEventBus> event_bus,
                std::function<void()> save_fn);

    void publishMarkers();
    void publishAreas();

private:
    void addAreaCallback(
        const std::shared_ptr<nav_interfaces::srv::AddAreaInfo::Request> req,
        std::shared_ptr<nav_interfaces::srv::AddAreaInfo::Response> res);

    void removeAreaCallback(
        const std::shared_ptr<nav_interfaces::srv::RemoveAreaInfo::Request> req,
        std::shared_ptr<nav_interfaces::srv::RemoveAreaInfo::Response> res);

    void updateAreaCallback(
        const std::shared_ptr<nav_interfaces::srv::UpdateAreaInfo::Request> req,
        std::shared_ptr<nav_interfaces::srv::UpdateAreaInfo::Response> res);

    void getAllAreasCallback(
        const std::shared_ptr<nav_interfaces::srv::GetAllAreas::Request> req,
        std::shared_ptr<nav_interfaces::srv::GetAllAreas::Response> res);

    static Area fromMsg(const nav_interfaces::msg::AreaInfo& msg);
    static nav_interfaces::msg::AreaInfo toMsg(const Area& area);

    struct Color { float r, g, b; };
    static Color typeColor(uint32_t type);

    rclcpp::Node* node_;
    std::shared_ptr<AreaStore> area_store_;
    std::shared_ptr<DataEventBus> event_bus_;
    std::function<void()> save_fn_;

    rclcpp::Service<nav_interfaces::srv::AddAreaInfo>::SharedPtr    add_area_srv_;
    rclcpp::Service<nav_interfaces::srv::RemoveAreaInfo>::SharedPtr remove_area_srv_;
    rclcpp::Service<nav_interfaces::srv::UpdateAreaInfo>::SharedPtr update_area_srv_;
    rclcpp::Service<nav_interfaces::srv::GetAllAreas>::SharedPtr get_all_areas_srv_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<nav_interfaces::msg::AreaInfoArray>::SharedPtr   areas_pub_;

    int prev_marker_count_{0};
};

}  // namespace map_asset_manager

#endif  // MAP_ASSET_MANAGER__AREA_MANAGER_HPP_
