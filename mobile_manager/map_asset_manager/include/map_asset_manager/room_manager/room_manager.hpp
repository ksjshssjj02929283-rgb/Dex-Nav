#ifndef MAP_ASSET_MANAGER__ROOM_MANAGER__ROOM_MANAGER_HPP_
#define MAP_ASSET_MANAGER__ROOM_MANAGER__ROOM_MANAGER_HPP_

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <nav_interfaces/msg/room_info.hpp>
#include <nav_interfaces/msg/room_info_array.hpp>
#include <nav_interfaces/srv/add_room.hpp>
#include <nav_interfaces/srv/get_all_rooms.hpp>
#include <nav_interfaces/srv/get_room_at_pose.hpp>
#include <nav_interfaces/srv/get_room_by_name.hpp>
#include <nav_interfaces/srv/remove_room.hpp>
#include <nav_interfaces/srv/update_room.hpp>

#include "map_asset_manager/data_event_bus.hpp"
#include "map_asset_manager/room_manager/room_store.hpp"
#include "map_asset_manager/types.hpp"

namespace map_asset_manager {

class RoomManager {
public:
    RoomManager(rclcpp::Node* node,
                std::shared_ptr<RoomStore> room_store,
                std::shared_ptr<DataEventBus> event_bus,
                std::function<void()> save_fn);

    void publishMarkers();
    void publishRooms();

    // 在 fromMsg 之外提供的工具方法（FileManager 反序列化 / 内部 import 可复用）
    static Room enrich(Room room, const std::string& source, const rclcpp::Time& now);

private:
    void addRoomCallback(
        const std::shared_ptr<nav_interfaces::srv::AddRoom::Request> req,
        std::shared_ptr<nav_interfaces::srv::AddRoom::Response> res);

    void removeRoomCallback(
        const std::shared_ptr<nav_interfaces::srv::RemoveRoom::Request> req,
        std::shared_ptr<nav_interfaces::srv::RemoveRoom::Response> res);

    void updateRoomCallback(
        const std::shared_ptr<nav_interfaces::srv::UpdateRoom::Request> req,
        std::shared_ptr<nav_interfaces::srv::UpdateRoom::Response> res);

    void getAllRoomsCallback(
        const std::shared_ptr<nav_interfaces::srv::GetAllRooms::Request> req,
        std::shared_ptr<nav_interfaces::srv::GetAllRooms::Response> res);

    void getRoomByNameCallback(
        const std::shared_ptr<nav_interfaces::srv::GetRoomByName::Request> req,
        std::shared_ptr<nav_interfaces::srv::GetRoomByName::Response> res);

    void getRoomAtPoseCallback(
        const std::shared_ptr<nav_interfaces::srv::GetRoomAtPose::Request> req,
        std::shared_ptr<nav_interfaces::srv::GetRoomAtPose::Response> res);

    static Room fromMsg(const nav_interfaces::msg::RoomInfo& msg);
    static nav_interfaces::msg::RoomInfo toMsg(const Room& room);

    static std::string sanitizeIdSegment(const std::string& s);
    static std::string buildId(const std::string& name, const std::string& source);

    struct Color { float r, g, b; };
    static Color typeColor(uint32_t type);

    rclcpp::Node* node_;
    std::shared_ptr<RoomStore> room_store_;
    std::shared_ptr<DataEventBus> event_bus_;
    std::function<void()> save_fn_;

    rclcpp::Service<nav_interfaces::srv::AddRoom>::SharedPtr        add_room_srv_;
    rclcpp::Service<nav_interfaces::srv::RemoveRoom>::SharedPtr     remove_room_srv_;
    rclcpp::Service<nav_interfaces::srv::UpdateRoom>::SharedPtr     update_room_srv_;
    rclcpp::Service<nav_interfaces::srv::GetAllRooms>::SharedPtr    get_all_rooms_srv_;
    rclcpp::Service<nav_interfaces::srv::GetRoomByName>::SharedPtr  get_room_by_name_srv_;
    rclcpp::Service<nav_interfaces::srv::GetRoomAtPose>::SharedPtr  get_room_at_pose_srv_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<nav_interfaces::msg::RoomInfoArray>::SharedPtr   rooms_pub_;

    int prev_marker_count_{0};
};

}  // namespace map_asset_manager

#endif  // MAP_ASSET_MANAGER__ROOM_MANAGER__ROOM_MANAGER_HPP_
