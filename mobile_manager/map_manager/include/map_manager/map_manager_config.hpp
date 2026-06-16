#ifndef MAP_MANAGER_CONFIG_HPP_
#define MAP_MANAGER_CONFIG_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>

// Forward declaration for YAML
namespace YAML {
class Node;
}

namespace map_manager {

/**
 * @brief Map Manager 配置类
 * 负责从 YAML 文件加载和管理 Map Manager 的所有配置参数
 */
class MapManagerConfig {
public:
    /**
     * @brief 构造函数
     * @param logger ROS2 logger 用于日志输出
     */
    explicit MapManagerConfig(rclcpp::Logger logger);

    /**
     * @brief 从默认安装路径加载配置文件
     * @param package_name 包名，用于定位安装路径
     * @return 是否成功加载
     */
    bool LoadFromDefaultPath(const std::string& package_name);

    /**
     * @brief 从指定路径加载配置文件
     * @param config_file 配置文件路径
     * @return 是否成功加载
     */
    bool LoadFromFile(const std::string& config_file);

    /**
     * @brief 保存 default_map_name 到配置文件
     * @param map_name 要保存的地图名称
     * @return 是否成功保存
     */
    bool SaveDefaultMapName(const std::string& map_name);

    // Getters
    const std::string& GetMapFolder() const { return map_folder_; }
    const std::string& GetDefaultMapName() const { return default_map_name_; }
    bool IsUseDefaultMap() const { return use_default_map_; }
    const std::string& GetNavPointManagerService() const { return nav_point_manager_service_; }
    const std::string& GetMissionManagerService() const { return mission_manager_service_; }
    const std::string& GetMoveBaseService() const { return move_base_service_; }
    const std::string& GetLocalizationService() const { return localization_service_; }

private:
    /**
     * @brief 从 YAML 节点解析配置
     * @param config YAML 节点
     */
    void ParseConfig(const YAML::Node& config);

    /**
     * @brief 设置默认值
     */
    void SetDefaults();

    /**
     * @brief 扩展路径中的 ~ 为当前用户的 home 目录
     * @param path 可能包含 ~ 的路径
     * @return 扩展后的完整路径
     */
    std::string ExpandHomePath(const std::string& path);

    rclcpp::Logger logger_;

    // 配置参数
    std::string map_folder_;
    std::string default_map_name_;
    bool use_default_map_;
    std::string nav_point_manager_service_;
    std::string mission_manager_service_;
    std::string move_base_service_;
    std::string localization_service_;

    // 配置文件路径（用于保存）
    std::string config_file_path_;
};

}  // namespace map_manager

#endif  // MAP_MANAGER_CONFIG_HPP_
