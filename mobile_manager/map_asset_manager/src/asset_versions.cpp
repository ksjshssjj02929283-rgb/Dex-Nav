#include "map_asset_manager/asset_versions.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <rclcpp/logging.hpp>

namespace fs = std::filesystem;

namespace map_asset_manager {

namespace {

constexpr const char* kPrefix = "map_assets.";
constexpr const char* kSuffix = ".json";

fs::path BackupsDir(const std::string& assets_file) { return fs::path(assets_file).parent_path() / "backups"; }

std::string LocalTimestamp() {
    const std::time_t t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm_buf;
    localtime_r(&t, &tm_buf);
    char ts[32];
    std::strftime(ts, sizeof(ts), "%Y%m%d-%H%M%S", &tm_buf);
    return ts;
}

bool ValidVersionTs(const std::string& ts) {
    if (ts.empty() || ts.size() > 40) return false;
    return std::all_of(ts.begin(), ts.end(), [](char c) { return std::isdigit(static_cast<unsigned char>(c)) || c == '-'; });
}

// 版本号取文件名中段：map_assets.<ts>.json
bool ParseVersionName(const std::string& name, std::string& ts) {
    const std::string pre(kPrefix), suf(kSuffix);
    if (name.size() <= pre.size() + suf.size()) return false;
    if (name.compare(0, pre.size(), pre) != 0) return false;
    if (name.compare(name.size() - suf.size(), suf.size(), suf) != 0) return false;
    ts = name.substr(pre.size(), name.size() - pre.size() - suf.size());
    return ValidVersionTs(ts);
}

std::string ReadSource(const fs::path& source_file) {
    std::ifstream in(source_file);
    std::string s;
    if (!in || !std::getline(in, s)) return "unknown";
    while (!s.empty() && (s.back() == '\r' || s.back() == ' ')) s.pop_back();
    return s.empty() ? "unknown" : s;
}

int64_t ParseTsSeconds(const std::string& ts, const fs::path& fallback) {
    std::tm tm_buf{};
    if (ts.size() >= 15 && strptime(ts.substr(0, 15).c_str(), "%Y%m%d-%H%M%S", &tm_buf) != nullptr) {
        tm_buf.tm_isdst = -1;
        return static_cast<int64_t>(mktime(&tm_buf));
    }
    std::error_code ec;
    const auto ft = fs::last_write_time(fallback, ec);
    if (ec) return 0;
    const auto sys = std::chrono::time_point_cast<std::chrono::seconds>(
        ft - fs::file_time_type::clock::now() + std::chrono::system_clock::now());
    return sys.time_since_epoch().count();
}

}  // namespace

std::string AssetVersionFile(const std::string& assets_file, const std::string& version_ts) {
    if (!ValidVersionTs(version_ts)) return "";
    return (BackupsDir(assets_file) / (std::string(kPrefix) + version_ts + kSuffix)).string();
}

std::string CreateAssetVersion(const std::string& assets_file, const rclcpp::Logger& logger,
                               const std::string& source, int keep, const std::string& protect) {
    std::error_code ec;
    if (!fs::exists(assets_file, ec)) return "";
    const fs::path dir = BackupsDir(assets_file);
    fs::create_directories(dir, ec);

    // 同一秒内第二个版本追加 -N，不覆盖已有版本
    const std::string base_ts = LocalTimestamp();
    std::string ts = base_ts;
    for (int n = 1; fs::exists(dir / (kPrefix + ts + kSuffix), ec); ++n) ts = base_ts + "-" + std::to_string(n);

    const fs::path dst = dir / (kPrefix + ts + kSuffix);
    const fs::path tmp = dir / ("." + std::string(kPrefix) + ts + ".json.tmp");
    fs::copy_file(assets_file, tmp, fs::copy_options::overwrite_existing, ec);
    if (!ec) {
        std::ofstream(dir / (kPrefix + ts + ".source")) << source << "\n";
        fs::rename(tmp, dst, ec);
    }
    if (ec) {
        RCLCPP_WARN(logger, "asset version backup failed: %s", ec.message().c_str());
        fs::remove(tmp, ec);
        fs::remove(dir / (kPrefix + ts + ".source"), ec);
        return "";
    }

    // 轮换：保留最新 keep 份，protect 指定的版本不删
    std::vector<std::string> all;
    for (const auto& e : fs::directory_iterator(dir, ec)) {
        std::string v;
        if (ParseVersionName(e.path().filename().string(), v)) all.push_back(v);
    }
    std::sort(all.begin(), all.end());
    const size_t limit = static_cast<size_t>(std::max(keep, 1));
    for (size_t i = 0; i + limit < all.size(); ++i) {
        if (all[i] == protect) continue;
        fs::remove(dir / (kPrefix + all[i] + kSuffix), ec);
        fs::remove(dir / (kPrefix + all[i] + ".source"), ec);
    }
    return ts;
}

std::vector<AssetVersionEntry> ListAssetVersions(const std::string& assets_file) {
    std::vector<AssetVersionEntry> out;
    std::error_code ec;
    const fs::path dir = BackupsDir(assets_file);
    if (!fs::is_directory(dir, ec)) return out;
    for (const auto& e : fs::directory_iterator(dir, ec)) {
        std::string ts;
        if (!e.is_regular_file(ec) || !ParseVersionName(e.path().filename().string(), ts)) continue;
        AssetVersionEntry v;
        v.version_ts = ts;
        v.path = e.path().string();
        v.source = ReadSource(dir / (kPrefix + ts + ".source"));
        v.saved_at_sec = ParseTsSeconds(ts, e.path());
        out.push_back(std::move(v));
    }
    std::sort(out.begin(), out.end(),
              [](const AssetVersionEntry& a, const AssetVersionEntry& b) { return a.version_ts > b.version_ts; });
    return out;
}

AssetFileLock::AssetFileLock(const std::string& assets_file) {
    std::error_code ec;
    const fs::path dir = fs::path(assets_file).parent_path();
    fs::create_directories(dir, ec);
    fd_ = ::open((dir / ".map_assets.lock").c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0644);
    if (fd_ >= 0 && ::flock(fd_, LOCK_EX) != 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

AssetFileLock::~AssetFileLock() {
    if (fd_ >= 0) {
        ::flock(fd_, LOCK_UN);
        ::close(fd_);
    }
}

}  // namespace map_asset_manager
