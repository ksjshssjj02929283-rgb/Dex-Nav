#include "map_asset_manager/room_manager/room_store.hpp"

namespace map_asset_manager {

RoomStore::RoomStore(rclcpp::Logger logger) : logger_(logger) {}

bool RoomStore::addRoom(const Room& room) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (rooms_map_.count(room.name)) {
        RCLCPP_WARN(logger_, "Room '%s' already exists", room.name.c_str());
        return false;
    }
    rooms_map_[room.name] = room;
    RCLCPP_INFO(logger_, "Added room: %s (id=%s)", room.name.c_str(), room.id.c_str());
    return true;
}

bool RoomStore::removeRoom(const std::string& name) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = rooms_map_.find(name);
    if (it == rooms_map_.end()) {
        RCLCPP_WARN(logger_, "Room '%s' not found", name.c_str());
        return false;
    }
    rooms_map_.erase(it);
    RCLCPP_INFO(logger_, "Removed room: %s", name.c_str());
    return true;
}

bool RoomStore::updateRoom(const Room& room) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!rooms_map_.count(room.name)) {
        RCLCPP_WARN(logger_, "Room '%s' not found for update", room.name.c_str());
        return false;
    }
    rooms_map_[room.name] = room;
    RCLCPP_INFO(logger_, "Updated room: %s", room.name.c_str());
    return true;
}

std::optional<Room> RoomStore::getRoom(const std::string& name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = rooms_map_.find(name);
    if (it == rooms_map_.end()) return std::nullopt;
    return it->second;
}

std::vector<Room> RoomStore::getAllRooms() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<Room> result;
    result.reserve(rooms_map_.size());
    for (const auto& [name, r] : rooms_map_) {
        result.push_back(r);
    }
    return result;
}

bool RoomStore::hasRoom(const std::string& name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return rooms_map_.count(name) > 0;
}

void RoomStore::setRooms(const std::unordered_map<std::string, Room>& rooms) {
    std::lock_guard<std::mutex> lock(mutex_);
    rooms_map_ = rooms;
}

std::unordered_map<std::string, Room> RoomStore::getRoomsMap() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return rooms_map_;
}

void RoomStore::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    rooms_map_.clear();
}

}  // namespace map_asset_manager
