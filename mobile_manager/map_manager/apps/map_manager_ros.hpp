#ifndef MAP_MANAGER_ROS_HPP_
#define MAP_MANAGER_ROS_HPP_

#include <iostream>
#include <memory>
#include <nav_interfaces/srv/delete_map.hpp>
#include <nav_interfaces/srv/get_map_info.hpp>
#include <nav_interfaces/srv/get_map_list.hpp>
#include <nav_interfaces/srv/get_string.hpp>
#include <nav_interfaces/srv/load_map.hpp>
#include <nav_interfaces/srv/rename_map.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <vector>

#include "map_manager/map_manager.hpp"
#include "map_manager/map_manager_config.hpp"

namespace map_manager {

class MapManagerROS : public rclcpp::Node {
public:
    MapManagerROS();
    ~MapManagerROS() = default;

private:
    bool ActivateCurrentMap();

    void LoadDefaultMap();

    void PublishMapAndNotify();

    bool SetActivateSignalToNode(const std::string& node_name, const std::string& service_name);

    void GetMapListCallback(const std::shared_ptr<nav_interfaces::srv::GetMapList::Request> req,
                            std::shared_ptr<nav_interfaces::srv::GetMapList::Response> res);

    void LoadMapCallback(const std::shared_ptr<nav_interfaces::srv::LoadMap::Request> req,
                         std::shared_ptr<nav_interfaces::srv::LoadMap::Response> res);

    void RemoveMapCallback(const std::shared_ptr<nav_interfaces::srv::DeleteMap::Request> req,
                           std::shared_ptr<nav_interfaces::srv::DeleteMap::Response> res);

    void RenameMapCallback(const std::shared_ptr<nav_interfaces::srv::RenameMap::Request> req,
                           std::shared_ptr<nav_interfaces::srv::RenameMap::Response> res);

    void GetCurrentMapInfoCallback(const std::shared_ptr<nav_interfaces::srv::GetMapInfo::Request> req,
                                   std::shared_ptr<nav_interfaces::srv::GetMapInfo::Response> res);

    void GetMapFolderCallback(const std::shared_ptr<nav_interfaces::srv::GetString::Request> req,
                              std::shared_ptr<nav_interfaces::srv::GetString::Response> res);

    void GetMapNameCallback(const std::shared_ptr<nav_interfaces::srv::GetString::Request> req,
                            std::shared_ptr<nav_interfaces::srv::GetString::Response> res);

    void GetCurrentMapDataCallback(const std::shared_ptr<nav_msgs::srv::GetMap::Request> req,
                                   std::shared_ptr<nav_msgs::srv::GetMap::Response> res);

private:
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr map_metadata_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr map_folder_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr map_name_pub_;

    rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr get_map_data_srv_;
    rclcpp::Service<nav_interfaces::srv::GetMapInfo>::SharedPtr get_map_info_srv_;
    rclcpp::Service<nav_interfaces::srv::GetString>::SharedPtr get_map_folder_srv_;
    rclcpp::Service<nav_interfaces::srv::GetString>::SharedPtr get_map_name_srv_;
    rclcpp::Service<nav_interfaces::srv::GetMapList>::SharedPtr get_all_maps_srv_;
    rclcpp::Service<nav_interfaces::srv::LoadMap>::SharedPtr load_map_srv_;
    rclcpp::Service<nav_interfaces::srv::DeleteMap>::SharedPtr remove_map_srv_;
    rclcpp::Service<nav_interfaces::srv::RenameMap>::SharedPtr rename_map_srv_;

    std::unique_ptr<MapManager> map_manager_;
    std::unique_ptr<MapManagerConfig> config_;

    // Callback group for services (separate from default)
    rclcpp::CallbackGroup::SharedPtr callback_group_services_;
};

}  // namespace map_manager

#endif  // MAP_MANAGER_ROS_HPP_
