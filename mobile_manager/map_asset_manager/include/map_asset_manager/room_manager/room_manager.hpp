#ifndef MAP_ASSET_MANAGER__ROOM_MANAGER__ROOM_MANAGER_HPP_
#define MAP_ASSET_MANAGER__ROOM_MANAGER__ROOM_MANAGER_HPP_

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <nav_interfaces/msg/room_info.hpp>
#include <nav_interfaces/msg/asset_snapshot.hpp>

#include "map_asset_manager/data_event_bus.hpp"
#include "map_asset_manager/room_manager/room_store.hpp"
#include "map_asset_manager/types.hpp"

namespace map_asset_manager {

class RoomManager {
public:
    static Room fromMsg(const nav_interfaces::msg::RoomInfo& msg);
    static nav_interfaces::msg::RoomInfo toMsg(const Room& room);
    RoomManager(rclcpp::Node* node,
                std::shared_ptr<RoomStore> room_store,
                std::shared_ptr<DataEventBus> event_bus,
                std::function<void()> save_fn);

    void publishMarkers();

    // 在 fromMsg 之外提供的工具方法（FileManager 反序列化 / 内部 import 可复用）
    static Room enrich(Room room, const std::string& source, const rclcpp::Time& now);

private:


    static std::string sanitizeIdSegment(const std::string& s);
    static std::string buildId(const std::string& name, const std::string& source);

    struct Color { float r, g, b; };
    static Color typeColor(uint32_t type);

    rclcpp::Node* node_;
    std::shared_ptr<RoomStore> room_store_;
    std::shared_ptr<DataEventBus> event_bus_;
    std::function<void()> save_fn_;


    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    int prev_marker_count_{0};
};

}  // namespace map_asset_manager

#endif  // MAP_ASSET_MANAGER__ROOM_MANAGER__ROOM_MANAGER_HPP_
