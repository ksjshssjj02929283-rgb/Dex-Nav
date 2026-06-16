#ifndef MAP_MANAGER_MAP_MANAGER_HPP_
#define MAP_MANAGER_MAP_MANAGER_HPP_

#include <iostream>
#include <memory>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <string>
#include <vector>

#include "map_manager/image_loader.hpp"

namespace map_manager {

struct MapInfo {
    std::string name;
    std::string path;
    double resolution;
    double origin_x;
    double origin_y;
    double origin_yaw;
    double width;
    double height;
};

class MapManager {
public:
    explicit MapManager(const std::string& default_map_folder);
    ~MapManager() = default;

    bool UpdateMapList();
    bool LoadMap(const std::string& map_name);
    bool RemoveMap(const std::string& map_name);
    bool RenameMap(const std::string& old_name, const std::string& new_name, std::string& error_msg);

    bool loadMapFromYaml(std::string path_to_yaml);
    bool loadMapFromValues(std::string map_file_name,
                           double resolution,
                           int negate,
                           double occ_th,
                           double free_th,
                           double origin[3],
                           MapMode mode);

    std::vector<std::string> GetMapList() const { return map_list_; }
    std::shared_ptr<nav_msgs::msg::MapMetaData> GetMapMetaData() const { return meta_data_message_; }
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> GetMap() const { return current_map_; }
    const std::string GetMapName() const { return current_map_name_; }
    const std::string GetMapFolder() const { return map_folder_; }
    const bool HasMapChanged() const { return has_map_changed_; }

    void ResetFlags() { has_map_changed_ = false; }

private:
    std::string map_folder_ = "";
    std::vector<std::string> map_list_;

    std::string current_map_name_ = "";

    std::shared_ptr<nav_msgs::msg::MapMetaData> meta_data_message_;
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> current_map_;

    bool has_map_changed_ = false;
};

}  // namespace map_manager

#endif  // MAP_MANAGER_MAP_MANAGER_HPP_
