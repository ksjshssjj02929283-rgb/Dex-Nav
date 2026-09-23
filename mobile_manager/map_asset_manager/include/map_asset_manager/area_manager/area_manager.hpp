#ifndef MAP_ASSET_MANAGER__AREA_MANAGER_HPP_
#define MAP_ASSET_MANAGER__AREA_MANAGER_HPP_

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <nav_interfaces/msg/area_info.hpp>
#include <nav_interfaces/msg/asset_snapshot.hpp>

#include "map_asset_manager/area_manager/area_store.hpp"
#include "map_asset_manager/data_event_bus.hpp"
#include "map_asset_manager/types.hpp"

namespace map_asset_manager {

class AreaManager {
public:
    static Area fromMsg(const nav_interfaces::msg::AreaInfo& msg);
    static nav_interfaces::msg::AreaInfo toMsg(const Area& area);
    AreaManager(rclcpp::Node* node,
                std::shared_ptr<AreaStore> area_store,
                std::shared_ptr<DataEventBus> event_bus,
                std::function<void()> save_fn);

    void publishMarkers();

private:


    struct Color { float r, g, b; };
    static Color typeColor(uint32_t type);

    rclcpp::Node* node_;
    std::shared_ptr<AreaStore> area_store_;
    std::shared_ptr<DataEventBus> event_bus_;
    std::function<void()> save_fn_;


    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    int prev_marker_count_{0};
};

}  // namespace map_asset_manager

#endif  // MAP_ASSET_MANAGER__AREA_MANAGER_HPP_
