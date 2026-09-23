#include "map_manager/map_manager_config.hpp"

#include <pwd.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdlib>
#include <fstream>

namespace map_manager {

MapManagerConfig::MapManagerConfig(rclcpp::Logger logger) : logger_(logger) { SetDefaults(); }

bool MapManagerConfig::LoadFromDefaultPath(const std::string& package_name) {
    try {
        std::string pkg_share_dir = ament_index_cpp::get_package_share_directory(package_name);
        std::string config_file = pkg_share_dir + "/config/config.yaml";

        RCLCPP_INFO(logger_, "Loading config from default path: %s", config_file.c_str());

        return LoadFromFile(config_file);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Failed to get package share directory: %s", e.what());
        return false;
    }
}

bool MapManagerConfig::LoadFromFile(const std::string& config_file) {
    try {
        // 检查文件是否存在
        std::ifstream file(config_file);
        if (!file.good()) {
            RCLCPP_ERROR(logger_, "Config file does not exist: %s", config_file.c_str());
            return false;
        }
        file.close();

        RCLCPP_INFO(logger_, "Loading config from: %s", config_file.c_str());

        // 保存配置文件路径，用于后续保存
        config_file_path_ = config_file;

        YAML::Node config = YAML::LoadFile(config_file);
        ParseConfig(config);

        RCLCPP_INFO(logger_, "Successfully loaded configuration");
        return true;

    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(logger_, "Failed to load config file: %s", e.what());
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Unexpected error loading config: %s", e.what());
        return false;
    }
}

void MapManagerConfig::ParseConfig(const YAML::Node& config) {
    // 读取地图文件夹路径
    if (config["map_folder"]) {
        std::string raw_path = config["map_folder"].as<std::string>();
        map_folder_ = ExpandHomePath(raw_path);
        RCLCPP_INFO(logger_, "Config: map_folder = %s (expanded from: %s)", map_folder_.c_str(), raw_path.c_str());
    } else {
        RCLCPP_WARN(logger_, "map_folder not found in config file, using default");
    }

    // 读取默认地图名称
    if (config["default_map_name"]) {
        default_map_name_ = config["default_map_name"].as<std::string>();
        RCLCPP_INFO(logger_, "Config: default_map_name = %s", default_map_name_.c_str());
    } else {
        RCLCPP_WARN(logger_, "default_map_name not found in config file, using default");
    }

    // 读取是否使用默认地图
    if (config["use_default_map"]) {
        use_default_map_ = config["use_default_map"].as<bool>();
        RCLCPP_INFO(logger_, "Config: use_default_map = %s", use_default_map_ ? "true" : "false");
    } else {
        RCLCPP_WARN(logger_, "use_default_map not found in config file, using default (false)");
    }

    // 读取服务名称
    if (config["services"]) {
        auto services = config["services"];

        if (services["nav_point_manager"]) {
            nav_point_manager_service_ = services["nav_point_manager"].as<std::string>();
            RCLCPP_INFO(logger_, "Config: nav_point_manager_service = %s", nav_point_manager_service_.c_str());
        } else {
            RCLCPP_WARN(logger_, "nav_point_manager service not in config, using default");
        }

        if (services["mission_manager"]) {
            mission_manager_service_ = services["mission_manager"].as<std::string>();
            RCLCPP_INFO(logger_, "Config: mission_manager_service = %s", mission_manager_service_.c_str());
        } else {
            RCLCPP_WARN(logger_, "mission_manager service not in config, using default");
        }

        if (services["move_base"]) {
            move_base_service_ = services["move_base"].as<std::string>();
            RCLCPP_INFO(logger_, "Config: move_base_service = %s", move_base_service_.c_str());
        } else {
            RCLCPP_WARN(logger_, "move_base service not in config, using default");
        }

        if (services["localization"]) {
            localization_service_ = services["localization"].as<std::string>();
            RCLCPP_INFO(logger_, "Config: localization_service = %s", localization_service_.c_str());
        } else {
            RCLCPP_WARN(logger_, "localization service not in config, using default");
        }
    } else {
        RCLCPP_WARN(logger_, "services section not found in config file, using defaults");
    }
}

void MapManagerConfig::SetDefaults() {
    map_folder_ = "";
    default_map_name_ = "";
    use_default_map_ = false;
    nav_point_manager_service_ = "/nav_point_manager_node/set_map_changed";
    mission_manager_service_ = "/mission_manager/set_map_changed";
    move_base_service_ = "/move_base/set_map_changed";
    localization_service_ = "/localization/set_map_changed";
}

std::string MapManagerConfig::ExpandHomePath(const std::string& path) {
    if (path.empty() || path[0] != '~') {
        return path;
    }

    std::string home_dir;

    // 首先尝试从环境变量获取 HOME
    const char* home_env = std::getenv("HOME");
    if (home_env != nullptr) {
        home_dir = home_env;
    } else {
        // 如果环境变量不存在，使用 getpwuid 获取
        struct passwd* pw = getpwuid(getuid());
        if (pw != nullptr && pw->pw_dir != nullptr) {
            home_dir = pw->pw_dir;
        } else {
            RCLCPP_WARN(logger_, "Cannot determine home directory, using path as-is: %s", path.c_str());
            return path;
        }
    }

    // 替换 ~ 为 home 目录
    if (path.length() == 1) {
        // 只有 ~
        return home_dir;
    } else if (path[1] == '/') {
        // ~/path/to/something
        return home_dir + path.substr(1);
    } else {
        // ~username 形式不支持，返回原始路径
        RCLCPP_WARN(logger_, "~username syntax not supported, using path as-is: %s", path.c_str());
        return path;
    }
}

bool MapManagerConfig::SaveDefaultMapName(const std::string& map_name) {
    if (config_file_path_.empty()) {
        RCLCPP_ERROR(logger_, "Config file path is empty, cannot save");
        return false;
    }

    try {
        // 加载现有配置
        YAML::Node config = YAML::LoadFile(config_file_path_);

        // 更新 default_map_name
        config["default_map_name"] = map_name;

        // 同时更新内存中的值
        default_map_name_ = map_name;

        // 写入文件
        std::ofstream fout(config_file_path_);
        if (!fout.is_open()) {
            RCLCPP_ERROR(logger_, "Failed to open config file for writing: %s", config_file_path_.c_str());
            return false;
        }

        fout << config;
        fout.close();

        RCLCPP_INFO(logger_, "Successfully saved default_map_name '%s' to config file", map_name.c_str());
        return true;

    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(logger_, "Failed to save config file: %s", e.what());
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Unexpected error saving config: %s", e.what());
        return false;
    }
}

}  // namespace map_manager
