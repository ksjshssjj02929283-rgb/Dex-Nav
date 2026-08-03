#ifndef MAP_ASSET_MANAGER__ROOM_MANAGER__ROOM_STORE_HPP_
#define MAP_ASSET_MANAGER__ROOM_MANAGER__ROOM_STORE_HPP_

#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "map_asset_manager/types.hpp"

namespace map_asset_manager {

// 房间内存数据库（按 name 主键存储；id 字段为 DSG 节点标识，可重复构造）
class RoomStore {
public:
    explicit RoomStore(rclcpp::Logger logger);

    bool addRoom(const Room& room);
    bool removeRoom(const std::string& name);
    bool updateRoom(const Room& room);
    std::optional<Room> getRoom(const std::string& name) const;
    std::vector<Room> getAllRooms() const;
    bool hasRoom(const std::string& name) const;

    // 批量操作
    void setRooms(const std::unordered_map<std::string, Room>& rooms);
    std::unordered_map<std::string, Room> getRoomsMap() const;

    void clear();

private:
    mutable std::mutex mutex_;
    std::unordered_map<std::string, Room> rooms_map_;
    rclcpp::Logger logger_;
};

}  // namespace map_asset_manager

#endif  // MAP_ASSET_MANAGER__ROOM_MANAGER__ROOM_STORE_HPP_
