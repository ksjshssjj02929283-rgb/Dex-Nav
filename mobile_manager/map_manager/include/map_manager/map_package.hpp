#ifndef MAP_MANAGER_MAP_PACKAGE_HPP_
#define MAP_MANAGER_MAP_PACKAGE_HPP_

// 地图导入导出（zip 单图包）的核心实现，不依赖 ROS。
// map_manager 节点的 ~/map_package 服务与应急命令行工具 map_package_tool 共用本实现，
// 包格式、文件白名单、校验规则、锁只有这一份。
//
// 包内结构：
//   manifest.json                 清单：格式版本、来源、地图元数据、每个文件的大小与 sha256
//   <map_name>/slam/...           map.pbstream / map.png / map.yaml / .complete（完整包另带附带文件与 backups/）
//   <map_name>/assets/...         map_assets.json（完整包另带 backups/）

#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

namespace map_manager {
namespace map_package {

// 错误码与 MapState 的 22-02 段一致
constexpr int32_t kOk = 0;
constexpr int32_t kErrOpInvalid = 2202010;
constexpr int32_t kErrNameInvalid = 2202020;
constexpr int32_t kErrNotFound = 2202030;
constexpr int32_t kErrInUse = 2202040;
constexpr int32_t kErrNameExists = 2202050;
constexpr int32_t kErrFolderMissing = 2202062;
constexpr int32_t kErrLoadFailed = 2202072;
constexpr int32_t kErrIoFailed = 2202082;
constexpr int32_t kErrTrashDisabled = 2202090;
constexpr int32_t kErrNoSpace = 2202132;
constexpr int32_t kErrImportFileInvalid = 2202140;
constexpr int32_t kErrImportIncompatible = 2202150;

constexpr const char* kFormatName = "dexe_map_package";
constexpr int kFormatMajor = 1;
constexpr int kFormatMinor = 0;
constexpr const char* kExportsDirname = ".exports";
constexpr const char* kImportsDirname = ".imports";

// 地图名字节上限：ext4 单个文件名上限 255 字节，地图名之后还要拼回收站条目
// 「_YYYYMMDD_HHMMSS_N」、导出包「_YYYYMMDD_HHMMSS_N.zip.part」等后缀，留出余量。
// 中文一个字 3 字节，约 66 个汉字。
constexpr size_t kMaxMapNameBytes = 200;

enum class Content { kStandard = 0, kFull = 1 };
enum class OnConflict { kRename = 0, kReject = 1, kOverwrite = 2 };

struct Options {
    std::string map_folder;
    bool trash_enable = true;
    std::string trash_dirname = ".map_trash";
    int retention_min = 60;                            // .exports / .imports 里的文件保留时长
    uint64_t max_package_bytes = 2048ULL << 20;        // 导入文件大小上限
    uint64_t max_unpacked_bytes = 4096ULL << 20;       // 解压后总大小上限（防压缩炸弹）
    uint64_t max_entries = 20000;                      // 包内条目数上限
    uint64_t min_free_bytes = 200ULL << 20;            // 写盘之后至少还要剩下的空间
};

struct Result {
    int32_t error_code = kOk;
    std::string message;
    std::string map_name;       // 导出：被导出的图；导入：实际落盘的名字
    std::string file_path;      // 导出：生成的包
    uint64_t size_bytes = 0;    // 导出：包的大小
    std::string sha256;         // 导出：整个 zip 的 sha256
    std::string trashed_entry;  // 导入 + 覆盖：旧图进回收站的条目名
    bool ok() const { return error_code == kOk; }
};

struct MapListItem {
    std::string name;
    std::string created_at;
    std::string last_edited_at;
};

// 新地图名的校验（导入落盘名、改名、恢复名）：单层目录名、不以 '.' 开头、合法 UTF-8、
// 不含控制字符、不超过 kMaxMapNameBytes 字节。通过返回 true；否则 error 写原因。
bool ValidateNewMapName(const std::string& name, std::string* error);

// 与 get_maps 同一判据：名字不以 '.' 开头，且有 slam/map.yaml（或旧扁平布局的 map.yaml）。
bool IsLoadableMapDirectory(const std::string& map_folder, const std::string& name);

// 列出 map_folder 下的地图（按名字排序），created_at / last_edited_at 取自 map.yaml。
std::vector<MapListItem> ListMaps(const std::string& map_folder);

// 把一张地图打包。out_dir 为空时写到 <map_folder>/.exports/。
Result ExportMap(const Options& options, const std::string& map_name, Content content,
                 const std::string& out_dir = "");

struct ImportRequest {
    std::string file_path;
    std::string new_name;                  // 空 = 包里的原名
    OnConflict on_conflict = OnConflict::kRename;
    bool require_imports_dir = true;       // 服务：只接受 <map_folder>/.imports/ 下的文件；命令行工具不限
    // 落盘一步与其它改地图目录的操作串行（节点内传 map_command 用的同一把锁；命令行工具传空）
    std::mutex* commit_mutex = nullptr;
    // 判断某个名字是不是当前已加载的地图（持 commit_mutex 时调用）；为空表示无法判断（命令行工具）
    std::function<bool(const std::string&)> is_current_map;
};

Result ImportMap(const Options& options, const ImportRequest& request);

// 把一张地图整个目录移进回收站：<map_folder>/<trash_dirname>/<name>_<YYYYMMDD_HHMMSS>[_<N>]/。
// 与建图 overwrite 时的归档、map_command OP_TRASH 同一命名。成功返回 true，条目名写入 entry。
bool MoveMapToTrash(const std::string& map_folder, const std::string& trash_dirname,
                    const std::string& name, std::string* entry, std::string* error);

// 清理 .exports / .imports 里超过保留期的文件与暂存目录。返回删除的条目数。
int CleanupExpired(const Options& options);

// 文件 sha256（小写十六进制）；读失败返回空串。
std::string FileSha256(const std::string& path);

}  // namespace map_package
}  // namespace map_manager

#endif  // MAP_MANAGER_MAP_PACKAGE_HPP_
