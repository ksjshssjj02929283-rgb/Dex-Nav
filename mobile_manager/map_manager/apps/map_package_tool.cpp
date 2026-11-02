// 地图导入导出应急命令行工具：map_manager 崩溃或没启动时使用；服务可用时请走 ~/map_package。
// 与服务共用 map_package_core（同一份包格式、白名单、校验、锁），不链接 ROS，不需要 source 环境。
//
//   map_package_tool list   --map-folder <dir>
//   map_package_tool export --map-folder <dir> --map <name> [--full] [--out-dir <dir>]
//   map_package_tool import --map-folder <dir> --file <zip> [--name <新名>]
//                           [--on-conflict rename|reject|overwrite] [--force]
//
// stdout 只输出一行 JSON，字段与 MapPackage 响应同名（另加 op；list 输出 maps 数组），日志写 stderr。
// 退出码：0 成功；1 业务失败（JSON 里有 error_code / message，错误码与服务相同）；2 参数错误。
#include <glog/logging.h>

#include <iostream>
#include <map>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "map_manager/map_package.hpp"

namespace mp = map_manager::map_package;
using ordered_json = nlohmann::ordered_json;

namespace {

const char* kUsage =
    "usage:\n"
    "  map_package_tool list   --map-folder <dir>\n"
    "  map_package_tool export --map-folder <dir> --map <name> [--full] [--out-dir <dir>]\n"
    "  map_package_tool import --map-folder <dir> --file <zip> [--name <new name>]\n"
    "                          [--on-conflict rename|reject|overwrite] [--force]\n"
    "Prints one JSON line on stdout. Exit code: 0 ok, 1 failed (see error_code), 2 usage error.\n"
    "Emergency use only: when map_manager is running, call the ~/map_package service instead.\n";

void Print(const ordered_json& j) {
    // 名字已按 UTF-8 校验过；兜底用 replace，输出永远是合法 JSON
    std::cout << j.dump(-1, ' ', false, nlohmann::json::error_handler_t::replace) << std::endl;
}

int UsageError(const std::string& op, const std::string& message) {
    ordered_json j;
    j["op"] = op;
    j["success"] = false;
    j["error_code"] = mp::kErrOpInvalid;
    j["message"] = message;
    Print(j);
    std::cerr << message << "\n" << kUsage;
    return 2;
}

int PrintResult(const std::string& op, const mp::Result& r) {
    ordered_json j;
    j["op"] = op;
    j["success"] = r.ok();
    j["error_code"] = r.error_code;
    j["message"] = r.message;
    j["map_name"] = r.map_name;
    j["file_path"] = r.file_path;
    j["size_bytes"] = r.size_bytes;
    j["sha256"] = r.sha256;
    j["trashed_entry"] = r.trashed_entry;
    Print(j);
    return r.ok() ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
    FLAGS_logtostderr = true;
    google::InitGoogleLogging(argv[0]);

    if (argc < 2 || std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help") {
        std::cerr << kUsage;
        return argc < 2 ? 2 : 0;
    }
    const std::string op = argv[1];
    std::map<std::string, std::string> opts;
    std::vector<std::string> flags;
    for (int i = 2; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--full" || a == "--force") {
            flags.push_back(a);
        } else if (a.rfind("--", 0) == 0 && i + 1 < argc) {
            opts[a] = argv[++i];
        } else {
            return UsageError(op, "unexpected argument: " + a);
        }
    }
    auto has_flag = [&flags](const char* f) {
        for (const auto& x : flags) if (x == f) return true;
        return false;
    };
    auto opt = [&opts](const char* key) {
        const auto it = opts.find(key);
        return it == opts.end() ? std::string() : it->second;
    };

    mp::Options options;
    options.map_folder = opt("--map-folder");
    if (options.map_folder.empty()) return UsageError(op, "--map-folder is required");

    if (op == "list") {
        ordered_json j;
        j["op"] = "list";
        j["success"] = true;
        j["error_code"] = 0;
        j["message"] = "";
        ordered_json maps = ordered_json::array();
        for (const auto& m : mp::ListMaps(options.map_folder)) {
            maps.push_back({{"name", m.name}, {"created_at", m.created_at}, {"last_edited_at", m.last_edited_at}});
        }
        j["maps"] = maps;
        Print(j);
        return 0;
    }

    if (op == "export") {
        const std::string name = opt("--map");
        if (name.empty()) return UsageError(op, "--map is required");
        const mp::Content content = has_flag("--full") ? mp::Content::kFull : mp::Content::kStandard;
        return PrintResult(op, mp::ExportMap(options, name, content, opt("--out-dir")));
    }

    if (op == "import") {
        mp::ImportRequest req;
        req.file_path = opt("--file");
        if (req.file_path.empty()) return UsageError(op, "--file is required");
        req.new_name = opt("--name");
        const std::string conflict = opt("--on-conflict").empty() ? "rename" : opt("--on-conflict");
        if (conflict == "rename") req.on_conflict = mp::OnConflict::kRename;
        else if (conflict == "reject") req.on_conflict = mp::OnConflict::kReject;
        else if (conflict == "overwrite") req.on_conflict = mp::OnConflict::kOverwrite;
        else return UsageError(op, "--on-conflict must be rename, reject or overwrite");
        // 工具不知道当前加载的是哪张图，覆盖必须显式确认
        if (req.on_conflict == mp::OnConflict::kOverwrite && !has_flag("--force")) {
            return UsageError(op, "--on-conflict overwrite needs --force: this tool cannot tell which map is "
                                  "currently loaded");
        }
        req.require_imports_dir = false;
        return PrintResult(op, mp::ImportMap(options, req));
    }

    return UsageError(op, "unknown command: " + op);
}
