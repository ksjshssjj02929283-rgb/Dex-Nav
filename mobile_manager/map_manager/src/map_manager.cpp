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
        if (fs::is_directory(entry)) {
            map_list_.push_back(entry.path().filename().string());
        }
    }

    return true;
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

}  // namespace map_manager
