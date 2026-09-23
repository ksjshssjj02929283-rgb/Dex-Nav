#ifndef MAP_MANAGER_MAP_MANAGER_HPP_
#define MAP_MANAGER_MAP_MANAGER_HPP_

#include <cstdint>
#include <iostream>
#include <memory>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <string>
#include <vector>
#include <ctime>

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

// get_maps 返回的目录条目（只读 yaml，不加载地图）
struct MapCatalogEntry {
    std::string name;
    std::string map_path;           // <map_folder>/<name>/slam
    double total_area_m2 = 0.0;
    double effective_area_m2 = 0.0;
    std::time_t updated_at = 0;     // map.yaml mtime
    std::string thumbnail;          // <map_path>/map.png
};

class MapManager {
public:
    explicit MapManager(const std::string& default_map_folder);
    ~MapManager() = default;

    bool UpdateMapList();
    bool LoadMap(const std::string& map_name);
    bool RemoveMap(const std::string& map_name);
    bool RenameMap(const std::string& old_name, const std::string& new_name, std::string& error_msg);
    // 带错误码版本（供 ROS 层 MapManagerCommand 使用）。返回 0 成功，否则为 MapState 内的 14xx 常量。
    int32_t LoadMapEx(const std::string& map_name, std::string& message);
    int32_t RemoveMapEx(const std::string& map_name, std::string& message);
    int32_t RenameMapEx(const std::string& old_name, const std::string& new_name, std::string& message);
    // 扫描 map_folder 下各地图的 slam/map.yaml，生成目录条目（不加载地图）
    std::vector<MapCatalogEntry> ScanMapCatalog();

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

    // 地图加载 epoch：每次成功 LoadMap（含同名重载/回退）单调 +1。
    // 消费方比对 epoch 判断是否需要重载，不依赖名字变化。
    uint64_t GetEpoch() const { return map_epoch_; }

    void ResetFlags() { has_map_changed_ = false; }

private:
    // 探测旧扁平格式地图并迁移到 slam/assets 分层（DEC-007 新格式）；LoadMap 前置调用，幂等。
    bool MigrateLegacyMapLayout(const std::string& map_name);

    std::string map_folder_ = "";
    std::vector<std::string> map_list_;

    std::string current_map_name_ = "";

    std::shared_ptr<nav_msgs::msg::MapMetaData> meta_data_message_;
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> current_map_;

    bool has_map_changed_ = false;

    // 单调递增的地图加载版本号；0 = 尚未加载任何地图。不持久化。
    uint64_t map_epoch_ = 0;
};

}  // namespace map_manager

#endif  // MAP_MANAGER_MAP_MANAGER_HPP_
