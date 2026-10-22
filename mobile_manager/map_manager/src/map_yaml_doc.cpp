#include "map_manager/map_yaml_doc.hpp"

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iterator>
#include <mutex>
#include <optional>
#include <unordered_map>

#include <sys/stat.h>

#include <glog/logging.h>

namespace map_manager {

MapYamlDoc MapYamlDoc::Load(const std::string& path) {
    MapYamlDoc doc;
    if (path.empty()) return doc;
    std::ifstream in(path);
    for (std::string text; std::getline(in, text);) {
        const std::string key = KeyOf(text);
        if (!key.empty() && doc.index_.count(key) > 0) continue;
        if (!key.empty()) doc.index_[key] = doc.lines_.size();
        doc.lines_.push_back({key, text, false});
    }
    return doc;
}

std::optional<std::string> MapYamlDoc::Get(const std::string& key) const {
    const auto it = index_.find(key);
    if (it == index_.end()) return std::nullopt;
    const std::string& text = lines_[it->second].text;
    const size_t b = text.find_first_not_of(" \t", text.find(':') + 1);
    const size_t e = text.find_last_not_of(" \t\r");
    if (b == std::string::npos || e < b) return std::string();
    return text.substr(b, e - b + 1);
}

std::optional<double> MapYamlDoc::GetDouble(const std::string& key) const {
    const auto value = Get(key);
    if (!value.has_value()) return std::nullopt;
    char* end = nullptr;
    const double v = std::strtod(value->c_str(), &end);
    if (end == value->c_str() || !std::isfinite(v)) return std::nullopt;
    return v;
}

void MapYamlDoc::Set(const std::string& key, const std::string& value) {
    const std::string text = key + ": " + value;
    const auto it = index_.find(key);
    if (it != index_.end()) {
        lines_[it->second].text = text;
        return;
    }
    index_[key] = lines_.size();
    lines_.push_back({key, text, false});
}

void MapYamlDoc::Erase(const std::string& key) {
    const auto it = index_.find(key);
    if (it == index_.end()) return;
    lines_[it->second].erased = true;
    index_.erase(it);
}

std::string MapYamlDoc::ToString() const {
    std::string out;
    for (const auto& line : lines_) {
        if (!line.erased) out += line.text + "\n";
    }
    return out;
}

// 顶层键行 = 行首非空白、非 '#'，且冒号前有键名。
std::string MapYamlDoc::KeyOf(const std::string& text) {
    if (text.empty() || text[0] == ' ' || text[0] == '\t' || text[0] == '#') return "";
    const size_t colon = text.find(':');
    if (colon == std::string::npos || colon == 0) return "";
    const size_t e = text.find_last_not_of(" \t", colon - 1);
    return e == std::string::npos ? "" : text.substr(0, e + 1);
}

MapYamlDirLock::MapYamlDirLock(const std::string& dir) {
    const std::string path = dir + "/.map.yaml.lock";
    fd_ = ::open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0644);
    if (fd_ < 0 || ::flock(fd_, LOCK_EX) != 0) {
        LOG(WARNING) << "map.yaml lock unavailable (" << path << "): " << std::strerror(errno);
    }
}

MapYamlDirLock::~MapYamlDirLock() {
    if (fd_ >= 0) {
        ::flock(fd_, LOCK_UN);
        ::close(fd_);
    }
}

MapOpLock::MapOpLock(const std::string& slam_dir) {
    const std::string path = slam_dir + "/.map_op.lock";
    const int fd = ::open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0644);
    if (fd < 0) {
        LOG(WARNING) << "map operation lock unavailable (" << path << "): " << std::strerror(errno);
        return;
    }
    if (::flock(fd, LOCK_EX | LOCK_NB) != 0) {
        ::close(fd);
        return;
    }
    fd_ = fd;
}

MapOpLock::~MapOpLock() {
    if (fd_ >= 0) {
        ::flock(fd_, LOCK_UN);
        ::close(fd_);
    }
}

bool WriteFileAtomically(const std::string& path, const std::string& content, std::string* error) {
    const std::string tmp = path + ".tmp";
    FILE* f = std::fopen(tmp.c_str(), "w");
    if (f == nullptr) {
        if (error) *error = "open " + tmp + " failed: " + std::strerror(errno);
        return false;
    }
    const bool ok = std::fwrite(content.data(), 1, content.size(), f) == content.size() &&
                    std::fflush(f) == 0 && ::fsync(::fileno(f)) == 0;
    std::fclose(f);
    if (!ok || std::rename(tmp.c_str(), path.c_str()) != 0) {
        if (error) *error = "write " + path + " failed: " + std::strerror(errno);
        std::remove(tmp.c_str());
        return false;
    }
    return true;
}

double NormalizeAngle(const double rad) {
    double r = std::remainder(rad, 2.0 * M_PI);  // [-π, π]
    if (r <= -M_PI) r += 2.0 * M_PI;              // 归到 (-π, π]
    return r;
}

std::string FormatAngle(const double rad) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.10g", rad);
    return buf;
}

std::string LocalTimeIso8601() {
    const std::time_t now = std::time(nullptr);
    struct tm tm_buf {};
    localtime_r(&now, &tm_buf);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &tm_buf);
    return buf;
}

std::time_t ParseLocalIso8601(const std::string& text) {
    std::string s = text;
    if (s.size() >= 2 && (s.front() == '"' || s.front() == '\'') && s.back() == s.front()) {
        s = s.substr(1, s.size() - 2);
    }
    struct tm tm_buf {};
    const char* end = ::strptime(s.c_str(), "%Y-%m-%dT%H:%M:%S", &tm_buf);
    if (end == nullptr) return 0;
    tm_buf.tm_isdst = -1;
    const std::time_t t = std::mktime(&tm_buf);
    return t < 0 ? 0 : t;
}

namespace {

// FNV-1a 64：跨进程、跨版本稳定（std::hash 不保证），够做乐观锁指纹。
uint64_t Fnv1a(const std::string& data, uint64_t h = 1469598103934665603ULL) {
    for (const unsigned char c : data) {
        h ^= c;
        h *= 1099511628211ULL;
    }
    return h;
}

}  // namespace

// 底图指纹 = map.png 文件内容 + resolution + origin 文本。
// 只认 png 字节而不解码像素：map.png 全由我方写出，同内容同字节；回滚恢复的也是原字节。
// get_maps 会对每张图算一次，按 (inode, 大小, 纳秒 mtime) 缓存，文件不变就不重读。
// 带上 inode：原子 rename 写入每次都换 inode，同一秒内改两次、大小又恰好相同也不会命中旧缓存。
std::string GridRevision(const std::string& map_path, const MapYamlDoc& yaml) {
    const std::string png = map_path + "/map.png";
    struct stat st {};
    if (::stat(png.c_str(), &st) != 0) return "";

    struct Cached { ino_t ino; off_t size; timespec mtime; uint64_t hash; };
    static std::mutex mutex;
    static std::unordered_map<std::string, Cached> cache;
    std::optional<uint64_t> png_hash;
    {
        std::lock_guard<std::mutex> lock(mutex);
        const auto it = cache.find(png);
        if (it != cache.end() && it->second.ino == st.st_ino && it->second.size == st.st_size &&
            it->second.mtime.tv_sec == st.st_mtim.tv_sec && it->second.mtime.tv_nsec == st.st_mtim.tv_nsec) {
            png_hash = it->second.hash;
        }
    }
    if (!png_hash.has_value()) {
        std::ifstream in(png, std::ios::binary);
        const std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        png_hash = Fnv1a(bytes);
        std::lock_guard<std::mutex> lock(mutex);
        cache[png] = {st.st_ino, st.st_size, st.st_mtim, *png_hash};
    }
    const uint64_t h = Fnv1a(yaml.Get("resolution").value_or("") + "|" + yaml.Get("origin").value_or(""),
                             *png_hash);
    char buf[24];
    std::snprintf(buf, sizeof(buf), "%016llx", static_cast<unsigned long long>(h));
    return buf;
}

}  // namespace map_manager
