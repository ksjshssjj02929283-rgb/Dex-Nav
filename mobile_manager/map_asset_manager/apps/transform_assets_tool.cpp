// transform_assets_tool — 离线资产坐标变换工具（不依赖任何节点）
//
// 地图原点改变（重设原点 / 对齐旋转）后，把这张图的资产换算到新坐标系。
// 与 map_asset_manager 的 transform_assets 服务是同一实现（TransformAssetsFile）。
//
// 用法：
//   ros2 run map_asset_manager transform_assets_tool --map_name <图名> --x <m> --y <m> --yaw <rad>
//   ros2 run map_asset_manager transform_assets_tool --map_dir <地图目录> --x 0 --y 0 --yaw 0.1
//   可选 --maps_root <地图根目录>（默认 ~/.dexe/data/maps，仅配合 --map_name）
//
// (x, y, yaw) 是新原点在旧坐标系下的位姿，与 reset_map_origin_tool 的 --new_x/--new_y/--new_yaw 同义；
// 对齐是绕原点纯旋转，传 x=y=0。
//
// 改动前先存资产版本 <assets>/backups/map_assets.<ts>.json；资产文件不存在视为成功（无资产可转）。
// 资产在线编辑由 map_asset_manager 持有内存副本：改的是当前加载的地图时，调用方须随后重新加载地图
// （map_command OP_LOAD），节点按 epoch 变化从文件重载。
//
// 输出：最后一行 [TRANSFORM_ASSETS_RESULT] {json}；退出码 0 成功 / 1 失败 / 2 参数错误。

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

#include <nlohmann/json.hpp>
#include <rclcpp/logging.hpp>

#include "map_asset_manager/asset_transform.hpp"
#include "map_asset_manager/file_manager.hpp"

namespace fs = std::filesystem;
using map_asset_manager::AssetFileTransformResult;
using map_asset_manager::OriginChange;

namespace {

void PrintUsage() {
    std::cerr << "Usage: transform_assets_tool (--map_name NAME [--maps_root DIR] | --map_dir DIR)\n"
                 "                             --x X --y Y --yaw YAW_RAD\n";
}

void PrintResult(bool ok, const std::string& reason, const std::string& message,
                 const AssetFileTransformResult* r) {
    nlohmann::json j;
    j["ok"] = ok;
    j["reason"] = reason;
    j["message"] = message;
    if (r) {
        j["assets_file"] = r->assets_file;
        j["no_assets"] = r->no_assets;
        j["version_ts"] = r->version_ts;
        j["points"] = r->stats.points;
        j["areas"] = r->stats.areas;
        j["junctions"] = r->stats.junctions;
        j["edges"] = r->stats.edges;
        j["rooms"] = r->stats.rooms;
    }
    std::cout << "[TRANSFORM_ASSETS_RESULT] " << j.dump() << std::endl;
}

bool ParseDouble(const std::string& s, double& out) {
    try {
        size_t pos = 0;
        out = std::stod(s, &pos);
        return pos == s.size() && std::isfinite(out);
    } catch (const std::exception&) {
        return false;
    }
}

}  // namespace

int main(int argc, char** argv) {
    std::string map_name, maps_root, map_dir;
    double x = 0.0, y = 0.0, yaw = 0.0;
    bool has_x = false, has_y = false, has_yaw = false;

    for (int i = 1; i < argc; ++i) {
        std::string key = argv[i];
        std::string val;
        const auto eq = key.find('=');
        if (eq != std::string::npos) {
            val = key.substr(eq + 1);
            key = key.substr(0, eq);
        } else if (key == "-h" || key == "--help") {
            PrintUsage();
            return 0;
        } else if (i + 1 < argc) {
            val = argv[++i];
        } else {
            PrintUsage();
            PrintResult(false, "bad_args", "missing value for " + key, nullptr);
            return 2;
        }

        bool ok = true;
        if (key == "--map_name") map_name = val;
        else if (key == "--maps_root") maps_root = val;
        else if (key == "--map_dir") map_dir = val;
        else if (key == "--x") ok = has_x = ParseDouble(val, x);
        else if (key == "--y") ok = has_y = ParseDouble(val, y);
        else if (key == "--yaw") ok = has_yaw = ParseDouble(val, yaw);
        else ok = false;
        if (!ok) {
            PrintUsage();
            PrintResult(false, "bad_args", "invalid argument " + key + " " + val, nullptr);
            return 2;
        }
    }

    if (!has_x || !has_y || !has_yaw || (map_name.empty() == map_dir.empty())) {
        PrintUsage();
        PrintResult(false, "bad_args", "need --x --y --yaw and exactly one of --map_name / --map_dir",
                    nullptr);
        return 2;
    }

    if (map_dir.empty()) {
        if (map_name.find('/') != std::string::npos || map_name == "." || map_name == "..") {
            PrintResult(false, "bad_args", "map_name must be a plain name: " + map_name, nullptr);
            return 2;
        }
        if (maps_root.empty()) {
            const char* home = std::getenv("HOME");
            if (home == nullptr) {
                PrintResult(false, "bad_args", "HOME not set, pass --maps_root", nullptr);
                return 2;
            }
            maps_root = (fs::path(home) / ".dexe" / "data" / "maps").string();
        }
        map_dir = (fs::path(maps_root) / map_name).string();
    }

    std::error_code ec;
    if (!fs::is_directory(map_dir, ec)) {
        PrintResult(false, "map_not_found", "map directory not found: " + map_dir, nullptr);
        return 1;
    }

    const auto logger = rclcpp::get_logger("transform_assets_tool");
    map_asset_manager::FileManager file_manager(logger);
    const std::string assets_file = map_asset_manager::ResolveAssetsFile(map_dir);
    const AssetFileTransformResult r =
        map_asset_manager::TransformAssetsFile(assets_file, OriginChange{x, y, yaw}, file_manager, logger);

    PrintResult(r.ok, r.ok ? "" : "transform_failed", r.message, &r);
    return r.ok ? 0 : 1;
}
