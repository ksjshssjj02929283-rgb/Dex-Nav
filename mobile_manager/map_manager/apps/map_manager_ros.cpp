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
    // 地图回收站：建图 overwrite 时旧地图目录整体 rename 到
    // <map_folder>/<map_trash_dirname>/<name>_<ts>/，不再硬删。
    // 保留 50 份为主判据，总容量上限为副判据（兜底极端大图）。
    this->declare_parameter<bool>("map_trash_enable", true);
    this->declare_parameter<std::string>("map_trash_dirname", ".map_trash");
    this->declare_parameter<int>("map_trash_max_entries", 50);
    this->declare_parameter<double>("map_trash_max_total_gib", 10.0);

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
    {
        MapManager::MapTrashOptions trash_opts;
        trash_opts.enable = this->get_parameter("map_trash_enable").as_bool();
        trash_opts.dirname = this->get_parameter("map_trash_dirname").as_string();
        trash_opts.max_entries = this->get_parameter("map_trash_max_entries").as_int();
        trash_opts.max_total_gib = this->get_parameter("map_trash_max_total_gib").as_double();
        map_manager_->SetMapTrashOptions(trash_opts);
        RCLCPP_INFO(this->get_logger(),
                    "Map trash: %s, dir=%s, max_entries=%d, max_total=%.1f GiB",
                    trash_opts.enable ? "enabled" : "disabled",
                    trash_opts.dirname.c_str(), trash_opts.max_entries,
                    trash_opts.max_total_gib);
        // 构造函数里已跑过一次 UpdateMapList（彼时 options 还是默认值），
        // 这里按实际配置补一次扫描与轮换。
        map_manager_->UpdateMapList();
    }

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

    get_maps_srv_ = this->create_service<nav_interfaces::srv::GetMaps>(
        "~/get_maps",
        std::bind(&MapManagerROS::GetMapsCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_services_);

    map_command_srv_ = this->create_service<nav_interfaces::srv::MapManagerCommand>(
        "~/map_command",
        std::bind(&MapManagerROS::MapManagerCommandCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_services_);
    get_trashed_maps_srv_ = this->create_service<nav_interfaces::srv::GetTrashedMaps>(
        "~/get_trashed_maps",
        std::bind(&MapManagerROS::GetTrashedMapsCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_services_);

    if (config_->IsUseDefaultMap()) {
        LoadDefaultMap();
    }
    // 未加载任何地图（含无默认图）时也发一次 map_state：latched 语义要求晚订阅者总能拿到 map_folder。
    // epoch 仍为 0，消费方据此知道「尚未加载」。
    if (map_manager_->GetEpoch() == 0) {
        PublishMapAndNotify();
    }
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

void MapManagerROS::GetMapsCallback(const std::shared_ptr<nav_interfaces::srv::GetMaps::Request> req,
                                    std::shared_ptr<nav_interfaces::srv::GetMaps::Response> res) {
    (void)req;
    using MS = nav_interfaces::msg::MapState;
    // map_folder 来自配置，与是否加载地图无关，无条件返回（此前的 get_current_map_info 先判加载态再填 folder，
    // 导致未加载时拿不到 folder，这也是 get_map_folder 一直删不掉的原因）。
    res->map_folder = map_manager_->GetMapFolder();
    res->current_map_name = map_manager_->GetMapName();   // 未加载为空
    if (res->map_folder.empty()) {
        res->success = false;
        res->error_code = MS::E_MAP_FOLDER_MISSING;
        res->message = "map_folder is not configured";
        return;
    }
    for (const auto& e : map_manager_->ScanMapCatalog()) {
        nav_interfaces::msg::MapInfo m;
        m.name = e.name;
        m.map_path = e.map_path;
        m.total_area_m2 = static_cast<float>(e.total_area_m2);
        m.effective_area_m2 = static_cast<float>(e.effective_area_m2);
        m.updated_at.sec = static_cast<int32_t>(e.updated_at);
        m.updated_at.nanosec = 0;
        m.created_at = e.created_at;
        m.data_version = e.data_version;
        m.thumbnail = e.thumbnail;
        res->maps.push_back(std::move(m));
    }
    res->success = true;
    res->error_code = MS::E_NONE;
    res->message = std::to_string(res->maps.size()) + " maps";
}

void MapManagerROS::GetTrashedMapsCallback(
    const std::shared_ptr<nav_interfaces::srv::GetTrashedMaps::Request> req,
    std::shared_ptr<nav_interfaces::srv::GetTrashedMaps::Response> res) {
    (void)req;
    using MS = nav_interfaces::msg::MapState;
    if (map_manager_->GetMapFolder().empty()) {
        res->success = false;
        res->error_code = MS::E_MAP_FOLDER_MISSING;
        res->message = "map_folder is not configured";
        return;
    }
    const auto& opts = map_manager_->GetMapTrashOptions();
    if (!opts.enable) {
        res->success = false;
        res->error_code = MS::E_MAP_TRASH_DISABLED;
        res->message = "map trash is disabled (map_trash_enable=false)";
        return;
    }
    res->trash_folder = map_manager_->GetMapTrashDir();
    for (const auto& e : map_manager_->ScanTrashCatalog()) {
        nav_interfaces::msg::TrashedMapInfo t;
        t.entry_name = e.entry_name;
        t.original_name = e.original_name;
        t.archived_at.sec = static_cast<int32_t>(e.archived_at);
        t.archived_at.nanosec = 0;
        // 元数据与正常地图同口径（归档保留完整 slam/map.yaml）
        t.map_info.name = e.map_info.name;
        t.map_info.map_path = e.map_info.map_path;
        t.map_info.total_area_m2 = static_cast<float>(e.map_info.total_area_m2);
        t.map_info.effective_area_m2 = static_cast<float>(e.map_info.effective_area_m2);
        t.map_info.updated_at.sec = static_cast<int32_t>(e.map_info.updated_at);
        t.map_info.updated_at.nanosec = 0;
        t.map_info.created_at = e.map_info.created_at;
        t.map_info.data_version = e.map_info.data_version;
        t.map_info.thumbnail = e.map_info.thumbnail;
        res->entries.push_back(std::move(t));
    }
    res->success = true;
    res->error_code = MS::E_NONE;
    res->message = std::to_string(res->entries.size()) + " trashed maps (kept up to " +
                   std::to_string(opts.max_entries) + ")";
}

void MapManagerROS::MapManagerCommandCallback(const std::shared_ptr<nav_interfaces::srv::MapManagerCommand::Request> req,
                                       std::shared_ptr<nav_interfaces::srv::MapManagerCommand::Response> res) {
    using MC = nav_interfaces::srv::MapManagerCommand;
    using MS = nav_interfaces::msg::MapState;
    std::string message;
    int32_t code = MS::E_NONE;

    switch (req->op) {
        case MC::Request::OP_LOAD: {
            code = map_manager_->LoadMapEx(req->map_name, message);
            if (code == MS::E_NONE) {
                this->set_parameter(rclcpp::Parameter("map_name", map_manager_->GetMapName()));
                PublishMapAndNotify();
                if (config_->IsUseDefaultMap()) {
                    config_->SaveDefaultMapName(map_manager_->GetMapName());
                }
            }
            break;
        }
        case MC::Request::OP_REMOVE: {
            code = map_manager_->RemoveMapEx(req->map_name, message);
            break;
        }
        case MC::Request::OP_RENAME: {
            const bool is_current = (req->map_name == map_manager_->GetMapName());
            const std::string default_before = config_->GetDefaultMapName();
            code = map_manager_->RenameMapEx(req->map_name, req->new_name, message);
            if (code == MS::E_NONE) {
                // 重命名当前图：epoch 递增并重新下发 map_state，下游据此重载（契约已声明）
                if (is_current) PublishMapAndNotify();
                if (config_->IsUseDefaultMap() && default_before == req->map_name) {
                    config_->SaveDefaultMapName(req->new_name);
                }
            }
            break;
        }
        case MC::Request::OP_RESTORE: {
            // 从回收站取回：整目录 rename 回 map_folder。护栏在核心层
            // （绝不覆盖 / 不动当前图 / move 非 copy），此处只做副作用处理。
            std::string restored;
            code = map_manager_->RestoreMapEx(req->map_name, req->new_name, message, restored);
            if (code == MS::E_NONE) {
                // 恢复出的是一张「新出现的地图」，不改变当前加载图，
                // 因此不 PublishMapAndNotify、不动 default_map —— 要用得再显式 OP_LOAD。
                RCLCPP_WARN(this->get_logger(), "Map restored from trash: %s -> %s",
                            req->map_name.c_str(), restored.c_str());
            }
            break;
        }
        case MC::Request::OP_TRASH: {
            // 整目录移入回收站（可恢复的删除）。护栏在核心层：不动当前图、
            // 回收站未启用时拒绝而不是退化为硬删。
            std::string entry;
            code = map_manager_->TrashMapEx(req->map_name, message, entry);
            break;
        }
        default:
            code = MS::E_MAP_OP_INVALID;
            message = "invalid op " + std::to_string(req->op) +
                      "; expected 1 (LOAD), 2 (REMOVE), 3 (RENAME), 4 (RESTORE) or 5 (TRASH)";
            break;
    }

    res->success = (code == MS::E_NONE);
    res->error_code = code;
    res->message = message;
    if (!res->success) {
        RCLCPP_ERROR(this->get_logger(), "map_command op=%u map='%s' failed (%d): %s",
                     req->op, req->map_name.c_str(), code, message.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "map_command op=%u: %s", req->op, message.c_str());
    }
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

}  // namespace map_manager
