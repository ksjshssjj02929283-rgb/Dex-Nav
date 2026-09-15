#include <algorithm>
#include "map_manager/map_manager.hpp"

#include <glog/logging.h>

#include <boost/filesystem.hpp>

#include "map_manager/image_loader.hpp"
#include "yaml-cpp/yaml.h"

namespace map_manager {

namespace fs = boost::filesystem;

MapManager::MapManager(const std::string& default_map_folder) : map_folder_(default_map_folder) { UpdateMapList(); }

bool MapManager::UpdateMapList() {
    if (!fs::exists(map_folder_) || !fs::is_directory(map_folder_)) {
        LOG(ERROR) << "Directory does not exist or is not a directory: " << map_folder_;
        return false;
    }

    map_list_.clear();
    for (const auto& entry : fs::directory_iterator(map_folder_)) {
        if (!fs::is_directory(entry)) continue;
        const std::string name = entry.path().filename().string();
        if (!IsLoadableMapDir(name)) {
            // 建图被中断留下的空 slam/、只有 pbstream 的目录等：不是地图，不进目录列表
            LOG(INFO) << "Skipping non-loadable map directory (no slam/map.yaml or map.yaml): " << name;
            continue;
        }
        map_list_.push_back(name);
    }
    std::sort(map_list_.begin(), map_list_.end());

    return true;
}

bool MapManager::IsLoadableMapDir(const std::string& map_name) const {
    const fs::path dir = fs::path(map_folder_) / map_name;
    boost::system::error_code ec;
    if (!fs::is_directory(dir, ec) || ec) return false;
    return fs::exists(dir / "slam" / "map.yaml", ec) || fs::exists(dir / "map.yaml", ec);
}

bool MapManager::LoadMap(const std::string& map_name) {
    LOG(WARNING) << "Loading map: " << map_name;
    std::string map_yaml_path = map_folder_ + "/" + map_name + "/slam/map.yaml";  // 底图在 slam/（DEC-007）

    if (map_folder_.empty()) {
        LOG(ERROR) << "Invalid map folder.";
        return false;
    }
    if (map_name.empty()) {
        LOG(ERROR) << "Invalid map name.";
        return false;
    }
    if (current_map_name_ == map_name) {
        LOG(INFO) << "Current map is already loaded: " << current_map_name_;
        LOG(WARNING) << "Reloading current map.";
        has_map_changed_ = true;
        ++map_epoch_;  // 同名重载/回退也递增 epoch，消费方据此重载（根治同名漏检）
        return true;
    }

    // 启动加载前置：探测旧扁平格式地图并迁移到 slam/assets 分层（DEC-007 新格式），
    // 迁移完成后才读 slam/map.yaml 正常加载（已是新格式或非地图目录则幂等跳过）。
    if (!MigrateLegacyMapLayout(map_name)) {
        LOG(ERROR) << "Legacy map migration failed for: " << map_name;
        return false;
    }

    bool load_success = loadMapFromYaml(map_yaml_path);
    if (!load_success) {
        LOG(ERROR) << "Failed to load map: " << map_folder_ + "/" + map_name;
        return false;
    }
    current_map_name_ = map_name;
    has_map_changed_ = true;
    ++map_epoch_;  // 切换到不同地图，epoch 递增

    LOG(INFO) << "Load Map success. Current map: " << current_map_name_;
    return true;
}

// 探测旧扁平格式（map.yaml/map.pgm/*.pbstream/map_assets.json 都在 <map>/ 根），迁移到
// slam/assets 分层（DEC-007）：底图文件 → slam/，map_assets.json → assets/。原地 mv（同盘原子）。
// 幂等：已是新格式（slam/map.yaml 存在）或非地图目录（根无 map.yaml）直接跳过。
bool MapManager::MigrateLegacyMapLayout(const std::string& map_name) {
    const fs::path map_dir = fs::path(map_folder_) / map_name;
    const fs::path slam_dir = map_dir / "slam";
    const fs::path assets_dir = map_dir / "assets";

    // 已是新格式 → 跳过
    if (fs::exists(slam_dir / "map.yaml")) {
        return true;
    }
    // 根无 map.yaml → 非旧格式地图目录（空/损坏/已迁移中断），交给后续加载报错
    if (!fs::exists(map_dir / "map.yaml")) {
        return true;
    }

    LOG(WARNING) << "Detected legacy (flat) map layout for '" << map_name
                 << "', migrating to slam/assets layout...";
    try {
        fs::create_directories(slam_dir);
        fs::create_directories(assets_dir);

        // 先收集根目录文件列表（避免迭代中 rename 导致迭代器失效），跳过子目录。
        std::vector<fs::path> files;
        for (fs::directory_iterator it(map_dir), end; it != end; ++it) {
            if (!fs::is_directory(it->status())) files.push_back(it->path());
        }
        // 按类型分流：map_assets.json → assets/，其余底图文件 → slam/。
        for (const auto& src : files) {
            const std::string fname = src.filename().string();
            const fs::path dst =
                (fname == "map_assets.json") ? (assets_dir / fname) : (slam_dir / fname);
            fs::rename(src, dst);
            LOG(INFO) << "  migrated " << fname << " -> " << dst.string();
        }
        LOG(WARNING) << "Legacy map '" << map_name << "' migrated to slam/assets layout.";
        return true;
    } catch (const fs::filesystem_error& e) {
        LOG(ERROR) << "Failed to migrate legacy map '" << map_name << "': " << e.what();
        return false;
    }
}

bool MapManager::RemoveMap(const std::string& map_name) {
    UpdateMapList();
    if (map_list_.empty()) {
        LOG(ERROR) << "Map list is empty.";
        return false;
    }
    if (std::find(map_list_.begin(), map_list_.end(), map_name) == map_list_.end()) {
        LOG(ERROR) << "Map not found: " << map_name;
        return false;
    }
    if (current_map_name_ == map_name) {
        LOG(ERROR) << "Cannot remove current map: " << current_map_name_;
        return false;
    }

    std::string directoryToDelete = map_folder_ + "/" + map_name;
    if (fs::exists(directoryToDelete) && fs::is_directory(directoryToDelete)) {
        auto remove_all_ref = fs::remove_all(directoryToDelete);
        (void)remove_all_ref;
    } else {
        LOG(ERROR) << "Directory does not exist or is not a directory: " << directoryToDelete;
        return false;
    }
    UpdateMapList();
    return true;
}

bool MapManager::RenameMap(const std::string& old_name, const std::string& new_name, std::string& error_msg) {
    if (new_name.empty()) {
        error_msg = "new_name is empty";
        LOG(ERROR) << error_msg;
        return false;
    }
    if (old_name.empty()) {
        error_msg = "old_name is empty";
        LOG(ERROR) << error_msg;
        return false;
    }
    if (old_name == new_name) {
        LOG(INFO) << "old_name == new_name (" << old_name << "), no-op";
        return true;
    }

    const std::string old_dir = map_folder_ + "/" + old_name;
    const std::string new_dir = map_folder_ + "/" + new_name;

    if (!fs::exists(old_dir) || !fs::is_directory(old_dir)) {
        error_msg = "Source map does not exist: " + old_dir;
        LOG(ERROR) << error_msg;
        return false;
    }
    if (fs::exists(new_dir)) {
        error_msg = "Target map already exists: " + new_dir;
        LOG(ERROR) << error_msg;
        return false;
    }

    boost::system::error_code ec;
    fs::rename(old_dir, new_dir, ec);
    if (ec) {
        error_msg = "Failed to rename: " + ec.message();
        LOG(ERROR) << error_msg;
        return false;
    }

    if (current_map_name_ == old_name) {
        current_map_name_ = new_name;
        has_map_changed_ = true;
        ++map_epoch_;  // 当前图改名 → epoch+1（消费方据新名/路径重载）
        LOG(INFO) << "Current map renamed: " << old_name << " -> " << new_name;
    }

    UpdateMapList();
    LOG(INFO) << "Rename Map success: " << old_name << " -> " << new_name;
    return true;
}

bool MapManager::loadMapFromYaml(std::string path_to_yaml) {
    std::string mapfname;
    MapMode mode;
    double res;
    int negate;
    double occ_th;
    double free_th;
    double origin[3];
    std::ifstream fin(path_to_yaml.c_str());
    if (fin.fail()) {
        LOG(ERROR) << "Could not open " << path_to_yaml;
        return false;
    }
    YAML::Node doc = YAML::Load(fin);

    try {
        res = doc["resolution"].as<double>();
    } catch (...) {
        LOG(ERROR) << "resolution invalid";
        return false;
    }
    try {
        negate = doc["negate"].as<int>();
    } catch (...) {
        LOG(ERROR) << "negate invalid";
        return false;
    }
    try {
        occ_th = doc["occupied_thresh"].as<double>();
    } catch (...) {
        LOG(ERROR) << "occupied_thresh invalid";
        return false;
    }
    try {
        free_th = doc["free_thresh"].as<double>();
    } catch (...) {
        LOG(ERROR) << "free_thresh invalid";
        return false;
    }
    try {
        std::string modeS = doc["mode"].as<std::string>("trinary");
        if (modeS == "trinary")
            mode = MapMode::TRINARY;
        else if (modeS == "scale")
            mode = MapMode::SCALE;
        else if (modeS == "raw")
            mode = MapMode::RAW;
        else {
            LOG(ERROR) << "Invalid mode tag : " << modeS;
            return false;
        }
    } catch (...) {
        mode = MapMode::TRINARY;
    }
    try {
        origin[0] = doc["origin"][0].as<double>();
        origin[1] = doc["origin"][1].as<double>();
        origin[2] = doc["origin"][2].as<double>();
    } catch (...) {
        LOG(ERROR) << "origin invalid";
        return false;
    }
    try {
        mapfname = doc["image"].as<std::string>();
        if (mapfname.empty()) {
            LOG(ERROR) << "image empty";
            return false;
        }
        boost::filesystem::path mapfpath(mapfname);
        if (!mapfpath.is_absolute()) {
            boost::filesystem::path dir(path_to_yaml);
            dir = dir.parent_path();
            mapfpath = dir / mapfpath;
            mapfname = mapfpath.string();
        }
    } catch (...) {
        LOG(ERROR) << "image invalid";
        return false;
    }

    return loadMapFromValues(mapfname, res, negate, occ_th, free_th, origin, mode);
}

bool MapManager::loadMapFromValues(std::string map_file_name,
                                   double resolution,
                                   int negate,
                                   double occ_th,
                                   double free_th,
                                   double origin[3],
                                   MapMode mode) {
    LOG(INFO) << "Loading map from image : " << map_file_name << ".";
    try {
        current_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        loadMapFromFile(current_map_.get(), map_file_name.c_str(), resolution, negate, occ_th, free_th, origin, mode);
    } catch (std::runtime_error& e) {
        LOG(ERROR) << e.what();
        return false;
    }

    current_map_->header.frame_id = "map";
    // 时间戳由ROS层发布前设置，core层不依赖rclcpp

    meta_data_message_ = std::make_shared<nav_msgs::msg::MapMetaData>();
    *meta_data_message_ = current_map_->info;
    return true;
}


// ── 带错误码版本 + 目录扫描（MapManagerCommand / GetMaps 使用）──────────────────
// 错误码与 nav_interfaces/msg/MapState.msg 的 14xx 常量保持一致；这里用字面量避免核心层依赖 ROS msg。
namespace {
constexpr int32_t kErrNone = 0;
constexpr int32_t kErrNameInvalid = 1402;
constexpr int32_t kErrNotFound = 1403;
constexpr int32_t kErrInUse = 1404;
constexpr int32_t kErrNameExists = 1405;
constexpr int32_t kErrFolderMissing = 1406;
constexpr int32_t kErrLoadFailed = 1407;
constexpr int32_t kErrIoFailed = 1408;
}  // namespace

int32_t MapManager::LoadMapEx(const std::string& map_name, std::string& message) {
    if (map_folder_.empty()) { message = "map_folder is not configured"; return kErrFolderMissing; }
    if (map_name.empty())    { message = "map_name is empty"; return kErrNameInvalid; }
    UpdateMapList();
    if (std::find(map_list_.begin(), map_list_.end(), map_name) == map_list_.end()) {
        message = "map not found: " + map_name; return kErrNotFound;
    }
    if (!LoadMap(map_name)) { message = "failed to load map files: " + map_name; return kErrLoadFailed; }
    message = "map loaded: " + map_name;
    return kErrNone;
}

int32_t MapManager::RemoveMapEx(const std::string& map_name, std::string& message) {
    if (map_folder_.empty()) { message = "map_folder is not configured"; return kErrFolderMissing; }
    if (map_name.empty())    { message = "map_name is empty"; return kErrNameInvalid; }
    // 按目录存在性判断而不是按可加载列表：建图中断留下的残留目录也允许用 REMOVE 清掉
    const std::string dir = map_folder_ + "/" + map_name;
    boost::system::error_code ec;
    if (!fs::is_directory(dir, ec) || ec) {
        message = "map not found: " + map_name; return kErrNotFound;
    }
    if (current_map_name_ == map_name) {
        message = "cannot remove the currently loaded map: " + map_name; return kErrInUse;
    }
    fs::remove_all(dir, ec);
    if (ec) { message = "remove failed: " + ec.message(); return kErrIoFailed; }
    UpdateMapList();
    message = "map removed: " + map_name;
    return kErrNone;
}

int32_t MapManager::RenameMapEx(const std::string& old_name, const std::string& new_name, std::string& message) {
    if (map_folder_.empty()) { message = "map_folder is not configured"; return kErrFolderMissing; }
    if (old_name.empty() || new_name.empty()) { message = "map_name or new_name is empty"; return kErrNameInvalid; }
    boost::system::error_code ec;
    if (!fs::is_directory(map_folder_ + "/" + old_name, ec) || ec) {
        message = "map not found: " + old_name; return kErrNotFound;
    }
    if (fs::exists(map_folder_ + "/" + new_name, ec)) {
        message = "target name already exists: " + new_name; return kErrNameExists;
    }
    std::string err;
    if (!RenameMap(old_name, new_name, err)) { message = err; return kErrIoFailed; }
    message = "map renamed: " + old_name + " -> " + new_name;
    return kErrNone;
}

std::vector<MapCatalogEntry> MapManager::ScanMapCatalog() {
    std::vector<MapCatalogEntry> out;
    UpdateMapList();
    for (const auto& name : map_list_) {
        MapCatalogEntry e;
        e.name = name;
        e.map_path = map_folder_ + "/" + name + "/slam";
        e.thumbnail = e.map_path + "/map.png";
        const std::string yaml_path = e.map_path + "/map.yaml";
        boost::system::error_code ec;
        if (fs::exists(yaml_path, ec) && !ec) {
            e.updated_at = fs::last_write_time(yaml_path, ec);
            if (ec) e.updated_at = 0;
            try {
                YAML::Node y = YAML::LoadFile(yaml_path);
                if (y["total_area_m2"])     e.total_area_m2     = y["total_area_m2"].as<double>();
                if (y["effective_area_m2"]) e.effective_area_m2 = y["effective_area_m2"].as<double>();
                if (y["created_at"])        e.created_at        = y["created_at"].as<std::string>();
            } catch (const std::exception& ex) {
                LOG(WARNING) << "ScanMapCatalog: cannot parse " << yaml_path << ": " << ex.what();
            }
        }
        out.push_back(std::move(e));
    }
    return out;
}

}  // namespace map_manager
