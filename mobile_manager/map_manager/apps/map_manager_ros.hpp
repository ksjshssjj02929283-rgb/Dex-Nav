#ifndef MAP_MANAGER_ROS_HPP_
#define MAP_MANAGER_ROS_HPP_

#include <iostream>
#include <memory>
#include <nav_interfaces/msg/map_state.hpp>
#include <nav_interfaces/srv/get_maps.hpp>
#include <nav_interfaces/srv/map_manager_command.hpp>
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
    void LoadDefaultMap();

    void PublishMapAndNotify();

    void GetMapsCallback(const std::shared_ptr<nav_interfaces::srv::GetMaps::Request> req,
                         std::shared_ptr<nav_interfaces::srv::GetMaps::Response> res);
    void MapManagerCommandCallback(const std::shared_ptr<nav_interfaces::srv::MapManagerCommand::Request> req,
                            std::shared_ptr<nav_interfaces::srv::MapManagerCommand::Response> res);
    void GetCurrentMapDataCallback(const std::shared_ptr<nav_msgs::srv::GetMap::Request> req,
                                   std::shared_ptr<nav_msgs::srv::GetMap::Response> res);

private:
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr map_metadata_pub_;
    // 地图状态：folder+name+epoch 原子下发，消费方按 epoch 比对重载
    rclcpp::Publisher<nav_interfaces::msg::MapState>::SharedPtr map_state_pub_;

    rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr get_map_data_srv_;
    rclcpp::Service<nav_interfaces::srv::GetMaps>::SharedPtr get_maps_srv_;
    rclcpp::Service<nav_interfaces::srv::MapManagerCommand>::SharedPtr map_command_srv_;

    std::unique_ptr<MapManager> map_manager_;
    std::unique_ptr<MapManagerConfig> config_;

    // Callback group for services (separate from default)
    rclcpp::CallbackGroup::SharedPtr callback_group_services_;
};

}  // namespace map_manager

#endif  // MAP_MANAGER_ROS_HPP_
