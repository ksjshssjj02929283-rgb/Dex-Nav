#include <algorithm>
#include <ctime>
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
    } catch (const std::exception& ex) {
        LOG(WARNING) << "FillEntryFromYaml: cannot parse " << yaml_path << ": " << ex.what();
    }
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

    const fs::path trash = GetMapTrashDir();
    fs::create_directories(trash, ec);
    if (ec) {
        message = "failed to create trash dir " + trash.string() + ": " + ec.message();
        return kErrIoFailed;
    }
    // 命名与建图侧 ResetMapDirectory 一致：<name>_<YYYYMMDD_HHMMSS>，同秒追加 _<N>。
    const std::time_t now = std::time(nullptr);
    std::tm tm_buf{};
    localtime_r(&now, &tm_buf);
    char ts[32];
    std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &tm_buf);
    std::string entry = map_name + "_" + ts;
    for (int n = 2; fs::exists(trash / entry, ec); ++n) {
        entry = map_name + "_" + ts + "_" + std::to_string(n);
    }

    // move 不是 copy：同挂载点 rename 原子，与 RestoreMapEx 互为逆操作。
    fs::rename(src, trash / entry, ec);
    if (ec) {
        message = "failed to trash " + map_name + ": " + ec.message();
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
