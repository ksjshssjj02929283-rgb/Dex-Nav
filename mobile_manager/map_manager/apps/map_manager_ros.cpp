#include "map_manager_ros.hpp"

#include <tf2_ros/buffer.h>

#include <chrono>
#include <filesystem>
#include <std_msgs/msg/string.hpp>
#include <thread>

using namespace std::chrono_literals;

namespace map_manager {

MapManagerROS::MapManagerROS() : rclcpp::Node("map_manager") {
    // 声明 ROS 参数
    this->declare_parameter<std::string>("map_folder", "");
    this->declare_parameter<std::string>("map_name", "");

    // 创建并加载配置
    config_ = std::make_unique<MapManagerConfig>(this->get_logger());

    // 从默认安装路径加载配置文件
    if (!config_->LoadFromDefaultPath("map_manager")) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load config, using defaults");
    }

    // 检查 map folder 是否存在
    std::string map_folder = config_->GetMapFolder();
    if (map_folder.empty()) {
        RCLCPP_FATAL(this->get_logger(), "Map folder is empty");
        throw std::runtime_error("Map folder is empty");
    }

    if (!std::filesystem::exists(map_folder)) {
        RCLCPP_FATAL(this->get_logger(), "Map folder does not exist: %s", map_folder.c_str());
        std::filesystem::create_directories(map_folder);
    }

    // 设置 map_folder 参数（启动时从配置设置一次）
    this->set_parameter(rclcpp::Parameter("map_folder", map_folder));

    // 使用配置初始化 MapManager
    map_manager_ = std::make_unique<MapManager>(config_->GetMapFolder());

    RCLCPP_INFO(this->get_logger(), "Map folder: %s", config_->GetMapFolder().c_str());
    RCLCPP_INFO(this->get_logger(), "Default map name: %s", config_->GetDefaultMapName().c_str());
    RCLCPP_INFO(this->get_logger(), "Use default map: %s", config_->IsUseDefaultMap() ? "true" : "false");

    // 创建专门的 callback group 用于服务，避免死锁
    callback_group_services_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // 发布器（latched 风格在ROS2使用可靠队列即可）
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1).transient_local().reliable());
    map_metadata_pub_ =
        this->create_publisher<nav_msgs::msg::MapMetaData>("map_info", rclcpp::QoS(1).transient_local().reliable());
    // 地图状态原子下发（folder+name+epoch）：消费方订阅它按 epoch 比对重载
    map_state_pub_ = this->create_publisher<nav_interfaces::msg::MapState>(
        "~/map_state", rclcpp::QoS(1).transient_local().reliable());

    // 所有服务使用专门的 callback group，避免死锁
    get_map_data_srv_ = this->create_service<nav_msgs::srv::GetMap>(
        "~/get_current_map_data",
        std::bind(&MapManagerROS::GetCurrentMapDataCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_services_);

    get_map_info_srv_ = this->create_service<nav_interfaces::srv::GetMapInfo>(
        "~/get_current_map_info",
        std::bind(&MapManagerROS::GetCurrentMapInfoCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_services_);

    get_map_folder_srv_ = this->create_service<nav_interfaces::srv::GetString>(
        "~/get_map_folder",
        std::bind(&MapManagerROS::GetMapFolderCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_services_);

    get_map_name_srv_ = this->create_service<nav_interfaces::srv::GetString>(
        "~/get_map_name",
        std::bind(&MapManagerROS::GetMapNameCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_services_);

    get_all_maps_srv_ = this->create_service<nav_interfaces::srv::GetMapList>(
        "~/get_map_list",
        std::bind(&MapManagerROS::GetMapListCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_services_);

    load_map_srv_ = this->create_service<nav_interfaces::srv::LoadMap>(
        "~/load_map",
        std::bind(&MapManagerROS::LoadMapCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_services_);

    remove_map_srv_ = this->create_service<nav_interfaces::srv::DeleteMap>(
        "~/remove_map",
        std::bind(&MapManagerROS::RemoveMapCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_services_);

    rename_map_srv_ = this->create_service<nav_interfaces::srv::RenameMap>(
        "~/rename_map",
        std::bind(&MapManagerROS::RenameMapCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_services_);

    if (config_->IsUseDefaultMap()) {
        LoadDefaultMap();
    }
}

void MapManagerROS::GetMapListCallback(const std::shared_ptr<nav_interfaces::srv::GetMapList::Request> req,
                                       std::shared_ptr<nav_interfaces::srv::GetMapList::Response> res) {
    (void)req;
    map_manager_->UpdateMapList();
    res->map_list = map_manager_->GetMapList();
    res->success = true;
}

void MapManagerROS::LoadMapCallback(const std::shared_ptr<nav_interfaces::srv::LoadMap::Request> req,
                                    std::shared_ptr<nav_interfaces::srv::LoadMap::Response> res) {
    bool load_success = map_manager_->LoadMap(req->map_name);
    if (!load_success) {
        res->success = false;
        return;
    }

    // 更新 map_name 参数
    this->set_parameter(rclcpp::Parameter("map_name", map_manager_->GetMapName()));
    RCLCPP_INFO(this->get_logger(), "Updated map_name parameter to: %s", map_manager_->GetMapName().c_str());

    PublishMapAndNotify();

    // 如果启用了 use_default_map，则将当前加载的地图保存回配置文件
    if (config_->IsUseDefaultMap()) {
        RCLCPP_INFO(this->get_logger(), "use_default_map is enabled, saving current map name to config");
        config_->SaveDefaultMapName(map_manager_->GetMapName());
    }

    // 地图加载本身成功，即返回 true
    // 节点激活失败不影响地图加载结果
    res->success = true;
}

void MapManagerROS::LoadDefaultMap() {
    const std::string default_map = config_->GetDefaultMapName();

    if (default_map.empty()) {
        RCLCPP_ERROR(this->get_logger(), "default_map_name is empty, no map will be loaded");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Loading default map: %s", default_map.c_str());

    bool load_success = map_manager_->LoadMap(default_map);
    if (!load_success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load default map: %s", default_map.c_str());
        return;
    } else {
        // 更新 map_name 参数
        this->set_parameter(rclcpp::Parameter("map_name", map_manager_->GetMapName()));
        RCLCPP_INFO(this->get_logger(), "Updated map_name parameter to: %s", map_manager_->GetMapName().c_str());

        // 如果启用了 use_default_map，则将当前加载的地图保存回配置文件
        if (config_->IsUseDefaultMap()) {
            RCLCPP_INFO(this->get_logger(), "use_default_map is enabled, saving current map name to config");
            config_->SaveDefaultMapName(map_manager_->GetMapName());
        }
    }

    PublishMapAndNotify();
}

void MapManagerROS::PublishMapAndNotify() {
    auto map_ptr = map_manager_->GetMap();
    auto metadata_ptr = map_manager_->GetMapMetaData();
    if (map_ptr && metadata_ptr) {
        // 设置时间戳
        map_ptr->header.stamp = this->get_clock()->now();
        map_metadata_pub_->publish(*metadata_ptr);
        map_pub_->publish(*map_ptr);
    }

    // 原子下发 MapState（folder+name+epoch+stamp）：消费方按 epoch 比对重载，同名重载/回退也能感知
    nav_interfaces::msg::MapState map_state_msg;
    map_state_msg.header.stamp = this->get_clock()->now();
    map_state_msg.map_folder = map_manager_->GetMapFolder();
    map_state_msg.map_name = map_manager_->GetMapName();
    map_state_msg.epoch = map_manager_->GetEpoch();
    map_state_pub_->publish(map_state_msg);

    this->set_parameter(rclcpp::Parameter("map_name", map_manager_->GetMapName()));

    // 自动生效走 latched map_state（epoch）：消费方自动重载，无需主动 push。
    map_manager_->ResetFlags();
}

void MapManagerROS::RemoveMapCallback(const std::shared_ptr<nav_interfaces::srv::DeleteMap::Request> req,
                                      std::shared_ptr<nav_interfaces::srv::DeleteMap::Response> res) {
    res->success = map_manager_->RemoveMap(req->map_name);
}

void MapManagerROS::RenameMapCallback(const std::shared_ptr<nav_interfaces::srv::RenameMap::Request> req,
                                      std::shared_ptr<nav_interfaces::srv::RenameMap::Response> res) {
    const bool is_current_map = (req->old_name == map_manager_->GetMapName());
    const std::string default_map_before = config_->GetDefaultMapName();

    std::string error_msg;
    const bool ok = map_manager_->RenameMap(req->old_name, req->new_name, error_msg);
    if (!ok) {
        res->success = false;
        res->message = error_msg;
        RCLCPP_ERROR(this->get_logger(), "Rename map failed: %s", error_msg.c_str());
        return;
    }

    // 当前加载的地图被改名 → epoch+1 + 重发 map_state（消费方据新名/路径重载）
    if (is_current_map) {
        PublishMapAndNotify();
    }

    // 默认地图的名字被改 → 同步配置文件
    if (config_->IsUseDefaultMap() && default_map_before == req->old_name) {
        RCLCPP_INFO(this->get_logger(), "Default map renamed, updating config: %s -> %s", req->old_name.c_str(),
                    req->new_name.c_str());
        config_->SaveDefaultMapName(req->new_name);
    }

    res->success = true;
    res->message = "Rename success";
}

void MapManagerROS::GetCurrentMapInfoCallback(const std::shared_ptr<nav_interfaces::srv::GetMapInfo::Request> req,
                                              std::shared_ptr<nav_interfaces::srv::GetMapInfo::Response> res) {
    (void)req;
    res->map_name = map_manager_->GetMapName();
    if (res->map_name.empty()) {
        res->success = false;
        res->error_msg = "No map loaded";
        RCLCPP_ERROR(this->get_logger(), "No map loaded");
        return;
    }
    res->map_folder = map_manager_->GetMapFolder();
    if (res->map_folder.empty()) {
        res->success = false;
        res->error_msg = "Map folder not set";
        RCLCPP_ERROR(this->get_logger(), "Map folder not set");
        return;
    }
    auto metadata_ptr = map_manager_->GetMapMetaData();
    if (!metadata_ptr) {
        res->success = false;
        res->error_msg = "load map failed";
        RCLCPP_ERROR(this->get_logger(), "load map failed");
        return;
    }
    res->map_info = *metadata_ptr;
    res->success = true;
}

void MapManagerROS::GetCurrentMapDataCallback(const std::shared_ptr<nav_msgs::srv::GetMap::Request> req,
                                              std::shared_ptr<nav_msgs::srv::GetMap::Response> res) {
    (void)req;
    const auto map_ptr = map_manager_->GetMap();
    if (!map_ptr) {
        RCLCPP_ERROR(this->get_logger(), "No map loaded");
        return;
    }
    auto map_copy = *map_ptr;
    map_copy.header.stamp = this->get_clock()->now();
    res->map = map_copy;
}

void MapManagerROS::GetMapFolderCallback(const std::shared_ptr<nav_interfaces::srv::GetString::Request> req,
                                         std::shared_ptr<nav_interfaces::srv::GetString::Response> res) {
    (void)req;
    res->data = map_manager_->GetMapFolder();
    res->success = true;
}

void MapManagerROS::GetMapNameCallback(const std::shared_ptr<nav_interfaces::srv::GetString::Request> req,
                                       std::shared_ptr<nav_interfaces::srv::GetString::Response> res) {
    (void)req;
    res->data = map_manager_->GetMapName();
    res->success = true;
}

}  // namespace map_manager
