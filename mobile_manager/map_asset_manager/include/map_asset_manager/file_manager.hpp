#ifndef MAP_ASSET_MANAGER__FILE_MANAGER_HPP_
#define MAP_ASSET_MANAGER__FILE_MANAGER_HPP_

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>

#include "map_asset_manager/types.hpp"

namespace map_asset_manager {

// 底层 JSON 文件读写工具类
// 各类数据（点位/路径/区域）的序列化/反序列化都在此实现
class FileManager {
public:
    explicit FileManager(rclcpp::Logger logger);

    // ── 导航点 ──────────────────────────────────────────────────
    bool loadPointsFromFile(const std::string& file_path, std::unordered_map<std::string, NavPoint>& out);
    bool savePointsToFile(const std::string& file_path, const std::unordered_map<std::string, NavPoint>& in);

    // ── 路径 ────────────────────────────────────────────────────
    bool loadRoutesFromFile(const std::string& file_path, std::unordered_map<std::string, Route>& out);
    bool saveRoutesToFile(const std::string& file_path, const std::unordered_map<std::string, Route>& in);

    // ── 区域 ────────────────────────────────────────────────────
    bool loadAreasFromFile(const std::string& file_path, std::unordered_map<std::string, Area>& out);
    bool saveAreasToFile(const std::string& file_path, const std::unordered_map<std::string, Area>& in);

    // ── 统一资产文件（map_assets.json，v2.0 含 DSG dsg.layers.ROOMS 段）─────
    // 一个文件包含 points / routes / areas / route_graph / dsg.layers.ROOMS 五个部分
    // 旧版（v1.x）文件中 rooms 段缺失时视为空，向后兼容
    bool loadAllAssetsFromFile(const std::string& file_path,
                               std::unordered_map<std::string, NavPoint>& points,
                               std::unordered_map<std::string, Route>& routes,
                               std::unordered_map<std::string, Area>& areas,
                               std::vector<RouteJunction>& junctions,
                               std::vector<RouteEdge>& edges,
                               std::unordered_map<std::string, Room>& rooms);
    bool saveAllAssetsToFile(const std::string& file_path,
                             const std::unordered_map<std::string, NavPoint>& points,
                             const std::unordered_map<std::string, Route>& routes,
                             const std::unordered_map<std::string, Area>& areas,
                             const std::vector<RouteJunction>& junctions,
                             const std::vector<RouteEdge>& edges,
                             const std::unordered_map<std::string, Room>& rooms);

    // ── 地图文件夹加载（旧接口，保留兼容）──────────────────────────
    bool loadMapFolder(const std::string& base_path,
                       const std::string& folder_name,
                       std::string& points_file,
                       std::string& routes_file,
                       std::string& areas_file);

private:
    rclcpp::Logger logger_;
    std::mutex file_mutex_;
};

}  // namespace map_asset_manager

#endif  // MAP_ASSET_MANAGER__FILE_MANAGER_HPP_
