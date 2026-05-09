#ifndef NAV_POINT_MANAGER__FILE_MANAGER_HPP_
#define NAV_POINT_MANAGER__FILE_MANAGER_HPP_

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>

#include "nav_point_manager/types.hpp"

namespace nav_point_manager {

class FileManager {
public:
    explicit FileManager(rclcpp::Logger logger);
    ~FileManager() = default;

    // 加载点位文件
    bool loadPointsFromFile(const std::string& file_path, std::unordered_map<std::string, NavPoint>& points_map);

    // 保存点位到文件
    bool savePointsToFile(const std::string& file_path, const std::unordered_map<std::string, NavPoint>& points_map);

    // 加载路网文件
    bool loadRoutesFromFile(const std::string& file_path, std::unordered_map<std::string, Route>& routes_map);

    // 保存路网到文件
    bool saveRoutesToFile(const std::string& file_path, const std::unordered_map<std::string, Route>& routes_map);

    // 加载指定文件夹下的点位和路网
    // base_path: /home/dexforce/.dexe/data/maps
    // folder_name: 文件夹名称（如 map_2_standard）
    // 返回实际的points和routes文件路径
    bool loadMapFolder(const std::string& base_path,
                       const std::string& folder_name,
                       std::string& points_file,
                       std::string& routes_file);

private:
    rclcpp::Logger logger_;
    std::mutex file_mutex_;
};

}  // namespace nav_point_manager

#endif  // NAV_POINT_MANAGER__FILE_MANAGER_HPP_
