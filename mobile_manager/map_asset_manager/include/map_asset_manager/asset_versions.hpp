#pragma once

// 资产版本序列：<assets>/backups/map_assets.<YYYYMMDD-HHMMSS>[-N].json，来源标记在同名 .source 文件。
// 与底图版本同构（时间戳命名、字典序即时间序、临时文件改名落盘、保留最新 50 份）；
// 写入方：显式保存、坐标变换前（离线工具）、回滚前快照。读取 / 恢复方只有 map_asset_manager。

#include <cstdint>
#include <string>
#include <vector>

#include <rclcpp/logger.hpp>

namespace map_asset_manager {

constexpr int kAssetVersionKeepCount = 50;

// 来源取值，与 MapVersion.source 对应
namespace AssetVersionSource {
constexpr const char* SAVE = "save";            // 显式保存（save_assets / set_assets save=true）
constexpr const char* TRANSFORM = "transform";  // 坐标变换前（对齐 / 重设原点）
constexpr const char* ROLLBACK = "rollback";    // 回滚前的自动快照
}  // namespace AssetVersionSource

struct AssetVersionEntry {
    std::string version_ts;
    std::string source;     // 缺 .source 文件时为 unknown
    int64_t saved_at_sec{0};
    std::string path;
};

// 把当前资产文件存成一份版本并轮换；返回版本号，文件不存在或失败返回空。
// protect 非空时该版本不参与本次轮换（回滚时保护目标版本）
std::string CreateAssetVersion(const std::string& assets_file, const rclcpp::Logger& logger,
                               const std::string& source = AssetVersionSource::SAVE,
                               int keep = kAssetVersionKeepCount, const std::string& protect = "");

// 列出版本，新的在前
std::vector<AssetVersionEntry> ListAssetVersions(const std::string& assets_file);

// 版本文件路径；版本号非法（含路径分隔符等）返回空
std::string AssetVersionFile(const std::string& assets_file, const std::string& version_ts);

// 资产文件读改写的进程间排它锁（<assets>/.map_assets.lock，flock）：资产节点改非当前图 / 回滚、
// 离线变换工具都先拿它，读到的内容在写回前不会被另一方改掉。析构释放
class AssetFileLock {
public:
    explicit AssetFileLock(const std::string& assets_file);
    ~AssetFileLock();
    AssetFileLock(const AssetFileLock&) = delete;
    AssetFileLock& operator=(const AssetFileLock&) = delete;
    bool locked() const { return fd_ >= 0; }

private:
    int fd_{-1};
};

}  // namespace map_asset_manager
