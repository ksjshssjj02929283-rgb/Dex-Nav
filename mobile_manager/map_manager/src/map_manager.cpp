#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <mutex>
#include <sstream>
#include <optional>
#include <unordered_map>

#include <sys/stat.h>
#include "map_manager/map_manager.hpp"
#include "map_manager/map_package.hpp"
#include "map_manager/map_yaml_doc.hpp"

#include <opencv2/imgcodecs.hpp>

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
        // 隐藏目录（.map_trash / .git / lost+found 之类）一律不是地图。
        // 放在 IsLoadableMapDir 之前：回收站内层结构比地图深一层，本来也过不了
        // 那道检查，但那样每次扫描都会打一条 INFO 噪音，且依赖「深一层」这个
        // 隐式性质——回收站结构一改就漏。显式跳过更稳。
        if (!name.empty() && name[0] == '.') continue;
        if (!IsLoadableMapDir(name)) {
            // 建图被中断留下的空 slam/、只有 pbstream 的目录等：不是地图，不进目录列表
            LOG(INFO) << "Skipping non-loadable map directory (no slam/map.yaml or map.yaml): " << name;
            continue;
        }
        map_list_.push_back(name);
    }
    std::sort(map_list_.begin(), map_list_.end());

    // 顺带轮换回收站：扫描目录本就要遍历 map_folder，这里做不额外增加 IO 时机；
    // 清理失败不影响地图列表，故忽略返回值。
    PruneMapTrash();

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
        LOG(WARNING) << "Reloading current map from disk: " << current_map_name_;
        // 同名重载必须重读文件：底图编辑、回滚、对齐、替换、重设原点改的都是磁盘上的文件，
        // 只递增 epoch 的话 /map 发布的仍是内存里的旧栅格。重读失败保留原来的栅格。
        const auto previous_map = current_map_;
        const auto previous_meta = meta_data_message_;
        if (!loadMapFromYaml(map_yaml_path)) {
            current_map_ = previous_map;
            meta_data_message_ = previous_meta;
            LOG(ERROR) << "Failed to reload current map, keeping the previous grid: " << map_name;
            return false;
        }
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
// 状态码与 nav_interfaces/msg/MapState.msg 的常量保持一致；这里用字面量避免核心层依赖 ROS msg。
// 整机状态码体系：模块 22 dexe_mobile_nav + 子模块 02 map_manager + 状态序号 2 位 + 等级末位。
// 改这里必须同步改 MapState.msg，两处是同一份真值的两个副本。
namespace {
constexpr int32_t kErrNone = 0;
constexpr int32_t kErrNameInvalid = 2202020;    // 等级 0
constexpr int32_t kErrNotFound = 2202030;       // 等级 0
constexpr int32_t kErrInUse = 2202040;          // 等级 0
constexpr int32_t kErrNameExists = 2202050;     // 等级 0
constexpr int32_t kErrFolderMissing = 2202062;  // 等级 2
constexpr int32_t kErrLoadFailed = 2202072;     // 等级 2
constexpr int32_t kErrIoFailed = 2202082;       // 等级 2
constexpr int32_t kErrTrashDisabled = 2202090;  // 等级 0
constexpr int32_t kErrEditInvalid = 2202100;    // 等级 0
constexpr int32_t kErrGridRevisionMismatch = 2202110;  // 等级 0
constexpr int32_t kErrVersionNotFound = 2202120;       // 等级 0
// 底图版本序列（与定位侧保存、SLAM 离线工具同一约定）：<slam>/backups/backup.<YYYYMMDD-HHMMSS>[-N]/
//   必备三件（回滚要求齐全）+ 存在即一并保存的附带文件 + .source（来源）
constexpr const char* kTripleFiles[] = {"map.pbstream", "map.png", "map.yaml"};
constexpr const char* kVersionExtraFiles[] = {"map.pcd", "map_highlight.pcd", "map_line_map.json", "map_origin.png"};
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
    // 硬删按名字拼路径：. / .. / 含 / / 隐藏目录一律拒绝，否则 remove_all 会删到地图目录之外
    if (map_name == "." || map_name == ".." || map_name.find('/') != std::string::npos || map_name[0] == '.') {
        message = "invalid map name: " + map_name; return kErrNameInvalid;
    }
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
    // 新名字按统一规则校验（UTF-8、字节上限、禁用字符），与建图、导入同一套
    std::string name_error;
    if (!map_package::ValidateNewMapName(new_name, &name_error)) {
        message = "invalid new_name: " + name_error; return kErrNameInvalid;
    }
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

int32_t MapManager::SetDisplayYawEx(const std::string& map_name, const double display_yaw,
                                    std::string& message) {
    if (map_folder_.empty()) { message = "map_folder is not configured"; return kErrFolderMissing; }
    if (map_name.empty())    { message = "map_name is empty"; return kErrNameInvalid; }
    if (!std::isfinite(display_yaw)) { message = "display_yaw must be a finite number"; return kErrEditInvalid; }
    if (!IsLoadableMapDir(map_name)) { message = "map not found: " + map_name; return kErrNotFound; }
    // 旧扁平布局先迁成 slam/ 分层（幂等），否则 slam/map.yaml 不存在
    if (!MigrateLegacyMapLayout(map_name)) {
        message = "failed to migrate legacy map layout: " + map_name; return kErrIoFailed;
    }
    const std::string slam_dir = map_folder_ + "/" + map_name + "/slam";
    const std::string yaml_path = slam_dir + "/map.yaml";
    boost::system::error_code ec;
    if (!fs::exists(yaml_path, ec) || ec) {  // 只改已有文件，绝不凭空造一份 map.yaml
        message = "map.yaml not found: " + yaml_path; return kErrNotFound;
    }
    const double yaw = NormalizeAngle(display_yaw);

    MapYamlDirLock lock(slam_dir);
    MapYamlDoc yaml = MapYamlDoc::Load(yaml_path);
    if (yaml.Get("display_yaw") == FormatAngle(yaw)) {  // 值没变不算编辑，不写盘也不推进 last_edited_at
        message = "display_yaw unchanged: " + map_name + " = " + FormatAngle(yaw) + " rad";
        return kErrNone;
    }
    yaml.Set("display_yaw", FormatAngle(yaw));
    yaml.Set("last_edited_at", LocalTimeIso8601());
    std::string error;
    if (!WriteFileAtomically(yaml_path, yaml.ToString(), &error)) {
        message = error; return kErrIoFailed;
    }
    message = "display_yaw set: " + map_name + " -> " + FormatAngle(yaw) + " rad";
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
        FillEntryFromYaml(e.map_path, &e);
        out.push_back(std::move(e));
    }
    return out;
}

// ── 地图回收站（map trash）实现 ──────────────────────────────────────────
// 约定：<map_folder>/<dirname>/<map_name>_<YYYYMMDD_HHMMSS>/
// 建图侧 ResetMapDirectory 在 overwrite 时把旧地图目录整体 rename 进来，
// 本类负责「不当地图列出」+「按上限轮换清理」两件事。
// 之所以由 map_manager 提供而不是建图侧自管：map_folder 的生命周期本就归它。

std::string MapManager::GetMapTrashDir() const {
    if (map_folder_.empty() || trash_options_.dirname.empty()) return "";
    return (fs::path(map_folder_) / trash_options_.dirname).string();
}

std::vector<std::string> MapManager::ListTrashedMaps() const {
    std::vector<std::string> names;
    const std::string trash = GetMapTrashDir();
    if (trash.empty()) return names;
    boost::system::error_code ec;
    if (!fs::is_directory(trash, ec) || ec) return names;

    // 按目录 mtime 倒序（最新在前）。条目名自带时间戳，但以 mtime 为准更稳：
    // 手工放进来的目录未必遵守命名约定。
    std::vector<std::pair<std::time_t, std::string>> entries;
    for (fs::directory_iterator it(trash, ec), end; !ec && it != end; it.increment(ec)) {
        if (!fs::is_directory(it->path(), ec) || ec) continue;
        boost::system::error_code t_ec;
        const std::time_t mt = fs::last_write_time(it->path(), t_ec);
        entries.emplace_back(t_ec ? 0 : mt, it->path().filename().string());
    }
    std::sort(entries.begin(), entries.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; });
    names.reserve(entries.size());
    for (auto& e : entries) names.push_back(std::move(e.second));
    return names;
}

namespace {
// 递归统计目录字节数；出错的条目跳过（宁可少算也不要因权限问题抛异常）。
std::uintmax_t DirectorySizeBytes(const fs::path& dir) {
    std::uintmax_t total = 0;
    boost::system::error_code ec;
    for (fs::recursive_directory_iterator it(dir, ec), end; !ec && it != end;
         it.increment(ec)) {
        boost::system::error_code f_ec;
        if (fs::is_regular_file(it->path(), f_ec) && !f_ec) {
            const std::uintmax_t sz = fs::file_size(it->path(), f_ec);
            if (!f_ec) total += sz;
        }
    }
    return total;
}
}  // namespace

int MapManager::PruneMapTrash() {
    if (!trash_options_.enable) return 0;
    const std::string trash = GetMapTrashDir();
    if (trash.empty()) return 0;
    boost::system::error_code ec;
    if (!fs::is_directory(trash, ec) || ec) return 0;

    // 最新在前；删除从列表尾部（最旧）开始。
    std::vector<std::string> names = ListTrashedMaps();
    if (names.empty()) return 0;

    int removed = 0;
    const auto drop_oldest = [&](const char* reason) {
        if (names.empty()) return false;
        const std::string victim = names.back();
        names.pop_back();
        boost::system::error_code r_ec;
        const std::uintmax_t n = fs::remove_all(fs::path(trash) / victim, r_ec);
        if (r_ec) {
            LOG(WARNING) << "PruneMapTrash: failed to remove " << victim << ": "
                         << r_ec.message();
            return false;
        }
        ++removed;
        LOG(INFO) << "PruneMapTrash: removed '" << victim << "' (" << reason
                  << ", " << n << " entries)";
        return true;
    };

    // 判据 1（主）：条目数上限。
    while (trash_options_.max_entries > 0 &&
           static_cast<int>(names.size()) > trash_options_.max_entries) {
        if (!drop_oldest("over max_entries")) break;
    }

    // 判据 2（副）：总容量上限。兜底「50 份里混进 500MB 级大图」的情形。
    // 每删一份重算一次总量：目录大小差异很大，按单份均值估算会不准。
    if (trash_options_.max_total_gib > 0.0) {
        const std::uintmax_t limit = static_cast<std::uintmax_t>(
            trash_options_.max_total_gib * 1024.0 * 1024.0 * 1024.0);
        while (!names.empty()) {
            std::uintmax_t total = 0;
            for (const auto& n : names) {
                total += DirectorySizeBytes(fs::path(trash) / n);
            }
            if (total <= limit) break;
            // 只剩一份还超限：保留它（删光反而丢掉唯一的回退点），只告警。
            if (names.size() == 1) {
                LOG(WARNING) << "PruneMapTrash: single entry '" << names.front()
                             << "' alone exceeds max_total_gib="
                             << trash_options_.max_total_gib << "; keeping it";
                break;
            }
            if (!drop_oldest("over max_total_gib")) break;
        }
    }
    return removed;
}

// ── 回收站：查询与恢复 ──────────────────────────────────────────────────

namespace {

std::time_t MtimeOr0(const std::string& path) {
    boost::system::error_code ec;
    if (!fs::exists(path, ec) || ec) return 0;
    const std::time_t t = fs::last_write_time(path, ec);
    return ec ? 0 : t;
}

}  // namespace

namespace {

// "20261009-122600" / "20261009-122600-1" → 本地时间；失败返回 0。
std::time_t ParseVersionTs(const std::string& ts) {
    struct tm tm_buf {};
    if (ts.size() < 15 || ::strptime(ts.substr(0, 15).c_str(), "%Y%m%d-%H%M%S", &tm_buf) == nullptr) return 0;
    tm_buf.tm_isdst = -1;
    return std::mktime(&tm_buf);
}

std::string VersionSource(const fs::path& dir) {
    std::ifstream in((dir / ".source").string());
    std::string s;
    if (in && std::getline(in, s) && !s.empty()) return s;
    if (fs::exists(dir / ".complete")) return "localization_save";  // 定位侧保存的版本带 .complete
    return "unknown";
}

bool TripleComplete(const fs::path& dir) {
    for (const char* f : kTripleFiles) {
        if (!fs::is_regular_file(dir / f)) return false;
    }
    return true;
}

// backups/ 下全部 backup.* 目录名（不含前缀），字典序 = 时间序（旧在前）。
std::vector<std::string> ListBackupTs(const fs::path& slam_dir) {
    std::vector<std::string> out;
    boost::system::error_code ec;
    for (fs::directory_iterator it(slam_dir / "backups", ec), end; !ec && it != end; it.increment(ec)) {
        const std::string name = it->path().filename().string();
        if (name.rfind("backup.", 0) == 0 && fs::is_directory(it->path())) out.push_back(name.substr(7));
    }
    std::sort(out.begin(), out.end());
    return out;
}

// 一张地图（slam 目录）的全部底图版本，新的在前。
std::vector<MapVersionEntry> ScanVersions(const fs::path& slam_dir) {
    std::vector<MapVersionEntry> out;
    const std::vector<std::string> all = ListBackupTs(slam_dir);
    for (auto it = all.rbegin(); it != all.rend(); ++it) {
        const fs::path dir = slam_dir / "backups" / ("backup." + *it);
        MapVersionEntry v;
        v.version_ts = *it;
        v.saved_at = ParseVersionTs(*it);
        if (v.saved_at == 0) {
            boost::system::error_code ec;
            v.saved_at = fs::last_write_time(dir, ec);
        }
        v.source = VersionSource(dir);
        v.complete = TripleComplete(dir);
        out.push_back(std::move(v));
    }
    return out;
}

}  // namespace

void MapManager::FillEntryFromYaml(const std::string& map_path, MapCatalogEntry* entry) {
    const std::string yaml_path = map_path + "/map.yaml";
    boost::system::error_code ec;
    if (!fs::exists(yaml_path, ec) || ec) return;
    entry->updated_at = fs::last_write_time(yaml_path, ec);
    if (ec) entry->updated_at = 0;
    try {
        YAML::Node y = YAML::LoadFile(yaml_path);
        if (y["total_area_m2"])     entry->total_area_m2     = y["total_area_m2"].as<double>();
        if (y["effective_area_m2"]) entry->effective_area_m2 = y["effective_area_m2"].as<double>();
        if (y["created_at"])        entry->created_at        = y["created_at"].as<std::string>();
        if (y["data_version"]) {
            entry->data_version = y["data_version"].as<std::string>();
        }
        if (y["display_yaw"])       entry->display_yaw       = y["display_yaw"].as<double>();
    } catch (const std::exception& ex) {
        LOG(WARNING) << "FillEntryFromYaml: cannot parse " << yaml_path << ": " << ex.what();
    }
    const MapYamlDoc doc = MapYamlDoc::Load(yaml_path);
    entry->grid_revision = GridRevision(map_path, doc);
    // 底图最后编辑时间只认 map.yaml 的 last_edited_at（编辑操作写入，加载 / 读取不动它）；
    // 资产的编辑时间由 map_asset_manager 自己管，这里不看。
    // 旧图没有该键时退回 map.png / map.yaml 的 mtime。
    entry->last_edited_at = ParseLocalIso8601(doc.Get("last_edited_at").value_or(""));
    if (entry->last_edited_at == 0) {
        entry->last_edited_at = std::max(MtimeOr0(map_path + "/map.png"), MtimeOr0(yaml_path));
    }
    entry->versions = ScanVersions(map_path);
}

namespace {

// "[x, y, yaw]" → 三个数；格式不对返回 false。
bool ParseOrigin(const std::string& text, double* x, double* y, double* yaw) {
    std::string s = text;
    std::replace(s.begin(), s.end(), '[', ' ');
    std::replace(s.begin(), s.end(), ']', ' ');
    std::replace(s.begin(), s.end(), ',', ' ');
    std::istringstream in(s);
    return static_cast<bool>(in >> *x >> *y >> *yaw);
}

// 与 SLAM 侧写面积的格式一致（6 位有效数字）。
std::string FormatArea(double v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%g", v);
    return buf;
}

std::string LocalBackupTimestamp() {
    const std::time_t now = std::time(nullptr);
    struct tm tm_buf;
    ::localtime_r(&now, &tm_buf);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d-%H%M%S", &tm_buf);
    return buf;
}

// 把三件套备份成 <slam>/backups/backup.<ts>[-N]/（与定位侧保存、rollback_map 同一约定），
// 并写 .source 标明版本来源。先写到隐藏的临时目录再 rename，避免留下不完整的版本被回滚选中。
bool BackupSlamTriple(const fs::path& slam_dir, const std::string& source, std::string* out_ts,
                      std::string* error) {
    const fs::path backups = slam_dir / "backups";
    boost::system::error_code ec;
    fs::create_directories(backups, ec);
    if (ec) { *error = "create " + backups.string() + " failed: " + ec.message(); return false; }
    const std::string base_ts = LocalBackupTimestamp();
    std::string ts = base_ts;
    for (int n = 1; fs::exists(backups / ("backup." + ts)); ++n) ts = base_ts + "-" + std::to_string(n);

    const fs::path staging = backups / (".backup." + ts + ".tmp");
    fs::remove_all(staging, ec);
    fs::create_directories(staging, ec);
    if (ec) { *error = "create " + staging.string() + " failed: " + ec.message(); return false; }
    for (const char* f : kTripleFiles) {
        fs::copy_file(slam_dir / f, staging / f, fs::copy_options::overwrite_existing, ec);
        if (ec) {
            *error = std::string("backup ") + f + " failed: " + ec.message();
            fs::remove_all(staging, ec);
            return false;
        }
    }
    for (const char* f : kVersionExtraFiles) {  // 附带文件：存在才存
        if (!fs::exists(slam_dir / f)) continue;
        fs::copy_file(slam_dir / f, staging / f, fs::copy_options::overwrite_existing, ec);
        if (ec) {
            *error = std::string("backup ") + f + " failed: " + ec.message();
            fs::remove_all(staging, ec);
            return false;
        }
    }
    std::ofstream(( staging / ".source").string()) << source << "\n";
    fs::rename(staging, backups / ("backup." + ts), ec);
    if (ec) {
        *error = "finalize backup failed: " + ec.message();
        fs::remove_all(staging, ec);
        return false;
    }
    *out_ts = ts;
    return true;
}

// 只保留最新的 keep 份 backup.*（名字即时间，字典序 = 时间序），protect 指定的版本不删。删除失败只告警。
void PruneSlamBackups(const fs::path& slam_dir, int keep, const std::string& protect = "") {
    const fs::path backups = slam_dir / "backups";
    boost::system::error_code ec;
    std::vector<fs::path> dirs;
    for (fs::directory_iterator it(backups, ec), end; !ec && it != end; it.increment(ec)) {
        const std::string name = it->path().filename().string();
        if (name.rfind("backup.", 0) == 0 && fs::is_directory(it->path())) dirs.push_back(it->path());
    }
    std::sort(dirs.begin(), dirs.end());
    for (size_t i = 0; i + keep < dirs.size(); ++i) {
        if (!protect.empty() && dirs[i].filename().string() == "backup." + protect) continue;
        fs::remove_all(dirs[i], ec);
        if (ec) LOG(WARNING) << "prune backup failed: " << dirs[i].string() << ": " << ec.message();
    }
}

}  // namespace

int32_t MapManager::EditGridEx(const std::string& map_name, const std::vector<GridStrokeSpec>& strokes,
                               const std::string& expected_revision, GridEditResult* result,
                               std::string& message) {
    *result = GridEditResult{};
    if (map_folder_.empty()) { message = "map_folder is not configured"; return kErrFolderMissing; }
    if (map_name.empty())    { message = "map_name is empty"; return kErrNameInvalid; }
    const std::string invalid = ValidateStrokes(strokes);
    if (!invalid.empty()) { message = invalid; return kErrEditInvalid; }
    if (!IsLoadableMapDir(map_name)) { message = "map not found: " + map_name; return kErrNotFound; }
    if (!MigrateLegacyMapLayout(map_name)) {
        message = "failed to migrate legacy map layout: " + map_name; return kErrIoFailed;
    }
    const fs::path slam_dir = fs::path(map_folder_) / map_name / "slam";
    for (const char* f : {"map.yaml", "map.png", "map.pbstream"}) {
        if (!fs::exists(slam_dir / f)) {  // 三件缺一就无法产生可回退的版本
            message = std::string(f) + " not found in " + slam_dir.string(); return kErrLoadFailed;
        }
    }
    const std::string yaml_path = (slam_dir / "map.yaml").string();
    const std::string png_path = (slam_dir / "map.png").string();

    MapOpLock op_lock(slam_dir.string());  // 与对齐 / 替换 / 重设原点互斥
    if (!op_lock.acquired()) {
        message = "map is being modified by another operation (align / replace / reset origin); retry later";
        return kErrIoFailed;
    }
    MapYamlDirLock lock(slam_dir.string());
    MapYamlDoc yaml = MapYamlDoc::Load(yaml_path);

    // 1. 乐观锁：编辑期间底图被别人改过（编辑、回滚、替换、对齐）就拒绝，不做自动合并
    const std::string current_revision = GridRevision(slam_dir.string(), yaml);
    if (!expected_revision.empty() && expected_revision != current_revision) {
        message = "grid revision mismatch (expected " + expected_revision + ", current " +
                  current_revision + "); reload the map and edit again";
        return kErrGridRevisionMismatch;
    }

    // 2. 栅格几何与阈值取自这张图自己的 map.yaml
    GridGeometry geo;
    GridThresholds th;
    double origin_yaw = 0.0;
    geo.resolution = yaml.GetDouble("resolution").value_or(0.0);
    th.occupied_thresh = yaml.GetDouble("occupied_thresh").value_or(-1.0);
    th.free_thresh = yaml.GetDouble("free_thresh").value_or(-1.0);
    if (!(geo.resolution > 0.0) ||
        !ParseOrigin(yaml.Get("origin").value_or(""), &geo.origin_x, &geo.origin_y, &origin_yaw)) {
        message = "map.yaml has invalid resolution or origin"; return kErrLoadFailed;
    }
    if (std::abs(origin_yaw) > 1e-9 || yaml.GetDouble("negate").value_or(0.0) != 0.0) {
        message = "map.yaml with rotated origin or negate != 0 is not supported for editing";
        return kErrLoadFailed;
    }
    if (!ValidThresholds(th)) { message = "map.yaml has invalid occupied/free thresholds"; return kErrLoadFailed; }

    cv::Mat gray = cv::imread(png_path, cv::IMREAD_UNCHANGED);
    if (gray.empty() || gray.type() != CV_8UC1) {
        message = "map.png is missing or not an 8-bit grayscale image"; return kErrLoadFailed;
    }
    geo.width = gray.cols;
    geo.height = gray.rows;

    // 3. 栅格化；没有像素变化就不备份、不写盘
    result->changed_pixels = ApplyStrokes(&gray, geo, th, strokes);
    result->requires_reload = false;
    if (result->changed_pixels == 0) {
        result->revision = current_revision;
        message = "no pixel changed: " + map_name;
        return kErrNone;
    }

    // 4. 先备份三件套成一个可回退的版本，再写盘
    std::string error;
    if (!BackupSlamTriple(slam_dir, "grid_edit", &result->backup_ts, &error)) { message = error; return kErrIoFailed; }
    PruneSlamBackups(slam_dir, map_backup_keep_count_);

    std::vector<uchar> png_bytes;
    if (!cv::imencode(".png", gray, png_bytes)) { message = "encode map.png failed"; return kErrIoFailed; }
    if (!WriteFileAtomically(png_path, std::string(png_bytes.begin(), png_bytes.end()), &error)) {
        message = error; return kErrIoFailed;
    }

    // 5. 底图像素变了 ⇒ 面积必须重算（口径同建图侧：自由像素 > (int)(255·(1−free_thresh))）；其余键原样保留
    const int free_pixel_thresh = static_cast<int>(255.0 * (1.0 - th.free_thresh));
    const double cell_area = geo.resolution * geo.resolution;
    yaml.Set("total_area_m2", FormatArea(static_cast<double>(geo.width) * geo.height * cell_area));
    yaml.Set("effective_area_m2", FormatArea(cv::countNonZero(gray > free_pixel_thresh) * cell_area));
    yaml.Set("last_edited_at", LocalTimeIso8601());
    if (!WriteFileAtomically(yaml_path, yaml.ToString(), &error)) {
        message = error + " (map.png already written; roll back with backup " + result->backup_ts + ")";
        return kErrIoFailed;
    }

    result->revision = GridRevision(slam_dir.string(), yaml);
    result->requires_reload = (map_name == current_map_name_);
    message = "grid edited: " + map_name + ", " + std::to_string(result->changed_pixels) +
              " pixels changed, backup " + result->backup_ts +
              (result->requires_reload ? "; reload the map to apply" : "");
    return kErrNone;
}


int32_t MapManager::RollbackMapEx(const std::string& map_name, const std::string& version_ts,
                                  GridEditResult* result, std::string& message) {
    *result = GridEditResult{};
    if (map_folder_.empty()) { message = "map_folder is not configured"; return kErrFolderMissing; }
    if (map_name.empty())    { message = "map_name is empty"; return kErrNameInvalid; }
    if (version_ts.find('/') != std::string::npos || version_ts == "." || version_ts == "..") {
        message = "invalid version_ts: " + version_ts; return kErrVersionNotFound;
    }
    if (!IsLoadableMapDir(map_name)) { message = "map not found: " + map_name; return kErrNotFound; }
    if (!MigrateLegacyMapLayout(map_name)) {
        message = "failed to migrate legacy map layout: " + map_name; return kErrIoFailed;
    }
    const fs::path slam_dir = fs::path(map_folder_) / map_name / "slam";
    MapOpLock op_lock(slam_dir.string());  // 与对齐 / 替换 / 重设原点互斥
    if (!op_lock.acquired()) {
        message = "map is being modified by another operation (align / replace / reset origin); retry later";
        return kErrIoFailed;
    }
    MapYamlDirLock lock(slam_dir.string());

    // 1. 确定目标版本（在快照之前定，否则「最新」会变成刚存的快照）
    std::string target = version_ts;
    if (target.empty()) {
        const auto all = ListBackupTs(slam_dir);
        if (all.empty()) { message = "no version to roll back to: " + map_name; return kErrVersionNotFound; }
        target = all.back();
    }
    const fs::path target_dir = slam_dir / "backups" / ("backup." + target);
    if (!fs::is_directory(target_dir) || !TripleComplete(target_dir)) {
        message = "version not found or incomplete: " + target; return kErrVersionNotFound;
    }

    // 2. 先把当前状态存成一个版本，回滚到它即可撤销；当前三件不齐（地图已损坏）就跳过快照
    std::string error;
    if (TripleComplete(slam_dir)) {
        if (!BackupSlamTriple(slam_dir, "rollback", &result->backup_ts, &error)) { message = error; return kErrIoFailed; }
        PruneSlamBackups(slam_dir, map_backup_keep_count_, target);
    } else {
        LOG(WARNING) << "current base map of '" << map_name << "' is incomplete; rollback without snapshot";
    }

    // 3. 逐个原子恢复：必备三件 + 版本里存在的附带文件（点云、线特征等与底图同一坐标系，必须一起回去）
    std::vector<std::string> files(std::begin(kTripleFiles), std::end(kTripleFiles));
    for (const char* f : kVersionExtraFiles) {
        if (fs::exists(target_dir / f)) files.emplace_back(f);
    }
    for (const auto& f : files) {
        const fs::path tmp = slam_dir / (f + ".rbtmp");
        boost::system::error_code ec;
        fs::copy_file(target_dir / f, tmp, fs::copy_options::overwrite_existing, ec);
        if (!ec) fs::rename(tmp, slam_dir / f, ec);
        if (ec) {
            fs::remove(tmp, ec);
            message = "restore " + f + " failed: " + ec.message() +
                      (result->backup_ts.empty() ? "" : " (state before rollback saved as " + result->backup_ts + ")");
            return kErrIoFailed;
        }
    }

    // 4. 回滚本身也是一次底图编辑：恢复出来的是旧版本的 last_edited_at，改成现在
    const std::string yaml_path = (slam_dir / "map.yaml").string();
    MapYamlDoc yaml = MapYamlDoc::Load(yaml_path);
    yaml.Set("last_edited_at", LocalTimeIso8601());
    if (!WriteFileAtomically(yaml_path, yaml.ToString(), &error)) {
        LOG(WARNING) << "rollback of '" << map_name << "' restored, but stamping last_edited_at failed: " << error;
    }

    result->revision = GridRevision(slam_dir.string(), yaml);
    result->requires_reload = (map_name == current_map_name_);
    message = "base map of " + map_name + " rolled back to " + target +
              (result->backup_ts.empty() ? "" : "; previous state saved as " + result->backup_ts) +
              (result->requires_reload ? "; reload the map to apply" : "");
    return kErrNone;
}

std::string MapManager::ParseOriginalName(const std::string& entry_name) {
    // 归档目录名形如 <name>_<YYYYMMDD>_<HHMMSS> 或 <name>_<YYYYMMDD>_<HHMMSS>_<N>。
    // 从尾部剥：先吃掉可选的纯数字去重后缀，再要求 <6 位> 与 <8 位> 两段数字。
    // 时间戳段长度固定，所以 "office_2" 这类名字本身带数字的不会被误伤。
    const auto all_digits = [](const std::string& s, size_t n) {
        return s.size() == n && s.find_first_not_of("0123456789") == std::string::npos;
    };
    const auto split_tail = [](const std::string& s, std::string* head, std::string* tail) {
        const size_t pos = s.find_last_of('_');
        if (pos == std::string::npos || pos == 0) return false;
        *head = s.substr(0, pos);
        *tail = s.substr(pos + 1);
        return true;
    };

    std::string cur = entry_name, head, tail;
    // 可选去重后缀 _<N>（长度不定的纯数字，且不能是 6 位——那会和 HHMMSS 混淆）
    if (split_tail(cur, &head, &tail) && !tail.empty() && tail.size() != 6 && tail.size() != 8 &&
        tail.find_first_not_of("0123456789") == std::string::npos) {
        cur = head;
    }
    // <HHMMSS>
    if (!split_tail(cur, &head, &tail) || !all_digits(tail, 6)) return entry_name;
    cur = head;
    // <YYYYMMDD>
    if (!split_tail(cur, &head, &tail) || !all_digits(tail, 8)) return entry_name;
    return head;
}

namespace {
// 从 <YYYYMMDD>_<HHMMSS> 解析成 time_t；失败返回 0 由调用方回退到目录 mtime。
std::time_t ParseTrashStamp(const std::string& entry_name) {
    const size_t p2 = entry_name.find_last_of('_');
    if (p2 == std::string::npos || p2 == 0) return 0;
    std::string hms = entry_name.substr(p2 + 1);
    std::string rest = entry_name.substr(0, p2);
    // 若尾部是去重后缀，再往前取一段
    if (hms.size() != 6) {
        const size_t p3 = rest.find_last_of('_');
        if (p3 == std::string::npos) return 0;
        hms = rest.substr(p3 + 1);
        rest = rest.substr(0, p3);
        if (hms.size() != 6) return 0;
    }
    const size_t p1 = rest.find_last_of('_');
    if (p1 == std::string::npos) return 0;
    const std::string ymd = rest.substr(p1 + 1);
    if (ymd.size() != 8) return 0;
    std::tm tm_buf{};
    if (strptime((ymd + hms).c_str(), "%Y%m%d%H%M%S", &tm_buf) == nullptr) return 0;
    tm_buf.tm_isdst = -1;  // 让 mktime 自行判定夏令时
    return std::mktime(&tm_buf);
}
}  // namespace

std::vector<TrashedMapEntry> MapManager::ScanTrashCatalog() const {
    std::vector<TrashedMapEntry> out;
    const std::string trash = GetMapTrashDir();
    if (trash.empty() || !trash_options_.enable) return out;
    boost::system::error_code ec;
    if (!fs::is_directory(trash, ec) || ec) return out;

    // ListTrashedMaps 已按 mtime 倒序（最新在前），沿用其顺序。
    for (const auto& name : ListTrashedMaps()) {
        TrashedMapEntry e;
        e.entry_name = name;
        e.original_name = ParseOriginalName(name);
        e.archived_at = ParseTrashStamp(name);
        if (e.archived_at == 0) {
            boost::system::error_code t_ec;
            const std::time_t mt = fs::last_write_time(fs::path(trash) / name, t_ec);
            e.archived_at = t_ec ? 0 : mt;
        }
        // 归档保留完整目录结构，元数据与正常地图同口径
        e.map_info.name = name;
        e.map_info.map_path = (fs::path(trash) / name / "slam").string();
        e.map_info.thumbnail = e.map_info.map_path + "/map.png";
        FillEntryFromYaml(e.map_info.map_path, &e.map_info);
        out.push_back(std::move(e));
    }
    return out;
}

int32_t MapManager::RestoreMapEx(const std::string& entry_name, const std::string& new_name,
                                 std::string& message, std::string& out_restored_name) {
    out_restored_name.clear();
    if (map_folder_.empty()) { message = "map_folder is not configured"; return kErrFolderMissing; }
    if (!trash_options_.enable) {
        message = "map trash is disabled (map_trash_enable=false); nothing to restore";
        return kErrTrashDisabled;
    }
    if (entry_name.empty()) { message = "entry_name is empty"; return kErrNameInvalid; }
    // 条目名只能是回收站里的一层目录名，不能借 ../ 挪走回收站外的目录
    if (entry_name == "." || entry_name == ".." || entry_name.find('/') != std::string::npos) {
        message = "invalid trashed entry name: " + entry_name; return kErrNameInvalid;
    }

    const std::string trash = GetMapTrashDir();
    boost::system::error_code ec;
    const fs::path src = fs::path(trash) / entry_name;
    if (trash.empty() || !fs::is_directory(src, ec) || ec) {
        message = "trashed entry not found: " + entry_name;
        return kErrNotFound;
    }

    // 目标名：显式给定优先，否则用解析出的原名
    const std::string target = new_name.empty() ? ParseOriginalName(entry_name) : new_name;
    if (target.empty() || target.find('/') != std::string::npos || target == "." || target == "..") {
        message = "invalid restore target name: " + target;
        return kErrNameInvalid;
    }
    // 显式给的新名字按统一规则校验；用原名恢复时不套新规则（旧图的名字可能早于规则）
    std::string name_error;
    if (!new_name.empty() && !map_package::ValidateNewMapName(new_name, &name_error)) {
        message = "invalid new_name: " + name_error;
        return kErrNameInvalid;
    }

    // 护栏①：绝不覆盖。恢复一张图不该毁掉另一张现存的图。
    if (fs::exists(fs::path(map_folder_) / target, ec)) {
        message = "target name already exists: " + target +
                  "; pass new_name to restore under a different name";
        return kErrNameExists;
    }
    // 护栏②：不在定位运行时把当前图换掉（与 RemoveMapEx 一致）。
    // 走到这里 target 目录必不存在，所以只有「当前图已被删/改名但仍加载着」
    // 的边角情形会命中；仍然拦，保持护栏语义完整。
    if (!current_map_name_.empty() && target == current_map_name_) {
        message = "target is the currently loaded map: " + target;
        return kErrInUse;
    }

    // 护栏③：move 不是 copy —— 它本就是 move 进来的，同挂载点 rename 原子。
    fs::rename(src, fs::path(map_folder_) / target, ec);
    if (ec) {
        message = "failed to restore " + entry_name + " -> " + target + ": " + ec.message();
        return kErrIoFailed;
    }

    UpdateMapList();
    out_restored_name = target;
    message = "map restored: " + entry_name + " -> " + target;
    LOG(WARNING) << "RestoreMapEx: " << message;
    return kErrNone;
}

int32_t MapManager::TrashMapEx(const std::string& map_name, std::string& message,
                               std::string& out_entry_name) {
    out_entry_name.clear();
    if (map_folder_.empty()) { message = "map_folder is not configured"; return kErrFolderMissing; }
    if (!trash_options_.enable) {
        // 不退化为硬删：调用方要的是可恢复的删除，硬删请显式用 RemoveMapEx。
        message = "map trash is disabled (map_trash_enable=false); map not moved";
        return kErrTrashDisabled;
    }
    if (map_name.empty() || map_name.find('/') != std::string::npos ||
        map_name == "." || map_name == ".." || map_name[0] == '.') {
        message = "invalid map name: " + map_name;
        return kErrNameInvalid;
    }
    boost::system::error_code ec;
    const fs::path src = fs::path(map_folder_) / map_name;
    if (!fs::is_directory(src, ec) || ec) {
        message = "map not found: " + map_name;
        return kErrNotFound;
    }
    if (current_map_name_ == map_name) {
        message = "cannot trash the currently loaded map: " + map_name;
        return kErrInUse;
    }

    // 命名与建图侧 ResetMapDirectory 一致：<name>_<YYYYMMDD_HHMMSS>，同秒追加 _<N>；
    // move 不是 copy：同挂载点 rename 原子，与 RestoreMapEx 互为逆操作。与导入覆盖共用同一实现。
    std::string entry;
    std::string error;
    if (!map_package::MoveMapToTrash(map_folder_, trash_options_.dirname, map_name, &entry, &error)) {
        message = error;
        return kErrIoFailed;
    }
    out_entry_name = entry;
    PruneMapTrash();
    UpdateMapList();
    message = "map trashed: " + map_name + " -> " + entry;
    LOG(WARNING) << "TrashMapEx: " << message;
    return kErrNone;
}

}  // namespace map_manager
