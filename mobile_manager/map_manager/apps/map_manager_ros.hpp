#ifndef MAP_MANAGER_ROS_HPP_
#define MAP_MANAGER_ROS_HPP_

#include <iostream>
#include <memory>
#include <nav_interfaces/msg/map_state.hpp>
#include <nav_interfaces/msg/trashed_map_info.hpp>
#include <nav_interfaces/srv/get_maps.hpp>
#include <nav_interfaces/srv/get_trashed_maps.hpp>
#include <nav_interfaces/srv/map_manager_command.hpp>
#include <nav_interfaces/srv/map_package.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <mutex>
#include <string>
#include <vector>

#include "map_manager/map_manager.hpp"
#include "map_manager/map_manager_config.hpp"
#include "map_manager/map_package.hpp"

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
    // 地图回收站查询：列出可恢复的归档条目。不可见的回收站等于慢一点的删除。
    void GetTrashedMapsCallback(const std::shared_ptr<nav_interfaces::srv::GetTrashedMaps::Request> req,
                                std::shared_ptr<nav_interfaces::srv::GetTrashedMaps::Response> res);

    void MapManagerCommandCallback(const std::shared_ptr<nav_interfaces::srv::MapManagerCommand::Request> req,
                            std::shared_ptr<nav_interfaces::srv::MapManagerCommand::Response> res);
    void GetCurrentMapDataCallback(const std::shared_ptr<nav_msgs::srv::GetMap::Request> req,
                                   std::shared_ptr<nav_msgs::srv::GetMap::Response> res);
    // 地图导入导出（zip 单图包）。打包解包在独立回调组里跑，不挡加载、改名；
    // 只有导入最后的落盘一步与 map_command 共用 map_dir_mutex_ 串行。
    void MapPackageCallback(const std::shared_ptr<nav_interfaces::srv::MapPackage::Request> req,
                            std::shared_ptr<nav_interfaces::srv::MapPackage::Response> res);

private:
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr map_metadata_pub_;
    // 地图状态：folder+name+epoch 原子下发，消费方按 epoch 比对重载
    rclcpp::Publisher<nav_interfaces::msg::MapState>::SharedPtr map_state_pub_;

    rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr get_map_data_srv_;
    rclcpp::Service<nav_interfaces::srv::GetMaps>::SharedPtr get_maps_srv_;
    rclcpp::Service<nav_interfaces::srv::MapManagerCommand>::SharedPtr map_command_srv_;
    rclcpp::Service<nav_interfaces::srv::GetTrashedMaps>::SharedPtr get_trashed_maps_srv_;
    rclcpp::Service<nav_interfaces::srv::MapPackage>::SharedPtr map_package_srv_;

    std::unique_ptr<MapManager> map_manager_;
    std::unique_ptr<MapManagerConfig> config_;

    // Callback group for services (separate from default)
    rclcpp::CallbackGroup::SharedPtr callback_group_services_;
    rclcpp::CallbackGroup::SharedPtr callback_group_package_;
    // 改地图目录（加载 / 删除 / 改名 / 恢复 / 移入回收站 / 导入落盘）互斥；
    // map_command 整个回调持有它，导入只在落盘那一步持有
    std::mutex map_dir_mutex_;
    map_package::Options package_options_;
    rclcpp::TimerBase::SharedPtr single_instance_timer_;
};

}  // namespace map_manager

#endif  // MAP_MANAGER_ROS_HPP_
