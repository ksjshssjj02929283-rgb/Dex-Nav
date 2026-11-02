#include "map_manager/map_package.hpp"

#include <archive.h>
#include <archive_entry.h>
#include <fcntl.h>
#include <locale.h>
#include <openssl/evp.h>
#include <stdio.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <map>
#include <nlohmann/json.hpp>
#include <set>
#include <sstream>
#include <thread>

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include "map_manager/map_yaml_doc.hpp"

#ifndef MAP_MANAGER_VERSION
#define MAP_MANAGER_VERSION "unknown"
#endif

namespace map_manager {
namespace map_package {

namespace fs = std::filesystem;
using ordered_json = nlohmann::ordered_json;

namespace {

constexpr const char* kTripleFiles[] = {"map.pbstream", "map.png", "map.yaml"};
// 完整包才带的 slam/ 附带文件（与底图版本里的附带文件同一份清单）
constexpr const char* kSlamExtraFiles[] = {"map.pcd", "map_highlight.pcd", "map_line_map.json", "map_origin.png"};
// 定位侧保存的瞬态：存在即说明正在保存这张图
constexpr const char* kSwapTransients[] = {"slam.staging", "slam.swap_backup", "slam.rollback_marker"};
constexpr size_t kIoChunk = 64 * 1024;
constexpr int kLockRetries = 20;           // 拿锁最多重试 20 次 × 100 ms
constexpr int kMaxCopySuffix = 999;

// libarchive 按当前线程的 locale 转换条目路径。进程跑在 C locale（systemd 默认不设 LANG）时，
// 中文路径转换失败，libarchive 3.6 写 zip 头会解引用空指针崩溃。读写 zip 期间把本线程切到 UTF-8，
// 只影响本线程，结束时还原。
class ScopedUtf8Locale {
public:
    ScopedUtf8Locale() {
        for (const char* name : {"C.UTF-8", "C.utf8", "en_US.UTF-8"}) {
            loc_ = ::newlocale(LC_CTYPE_MASK, name, static_cast<locale_t>(0));
            if (loc_ != static_cast<locale_t>(0)) break;
        }
        if (loc_ != static_cast<locale_t>(0)) prev_ = ::uselocale(loc_);
        else LOG(WARNING) << "no UTF-8 locale available; non-ASCII map names may fail to pack";
    }
    ~ScopedUtf8Locale() {
        if (loc_ != static_cast<locale_t>(0)) {
            ::uselocale(prev_);
            ::freelocale(loc_);
        }
    }
    ScopedUtf8Locale(const ScopedUtf8Locale&) = delete;
    ScopedUtf8Locale& operator=(const ScopedUtf8Locale&) = delete;

private:
    locale_t loc_ = static_cast<locale_t>(0);
    locale_t prev_ = static_cast<locale_t>(0);
};

Result Fail(int32_t code, const std::string& message, Result r = Result()) {
    r.error_code = code;
    r.message = message;
    return r;
}

// 暂存目录名：时间 + 进程号 + 进程内序号。同一秒、同一进程里的两次操作（节点内并发、测试多线程）也不撞名
std::string StagingName(const std::string& stamp) {
    static std::atomic<uint64_t> seq{0};
    return ".staging_" + stamp + "_" + std::to_string(::getpid()) + "_" + std::to_string(seq.fetch_add(1));
}

std::string NowStamp() {
    const std::time_t now = std::time(nullptr);
    std::tm tm_buf{};
    localtime_r(&now, &tm_buf);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm_buf);
    return buf;
}

bool IsValidUtf8(const std::string& s) {
    size_t i = 0;
    const size_t n = s.size();
    while (i < n) {
        const unsigned char c = static_cast<unsigned char>(s[i]);
        if (c < 0x80) { ++i; continue; }
        int len = 0;
        uint32_t cp = 0;
        if ((c & 0xE0) == 0xC0) { len = 2; cp = c & 0x1F; }
        else if ((c & 0xF0) == 0xE0) { len = 3; cp = c & 0x0F; }
        else if ((c & 0xF8) == 0xF0) { len = 4; cp = c & 0x07; }
        else return false;
        if (i + len > n) return false;
        for (int k = 1; k < len; ++k) {
            const unsigned char cc = static_cast<unsigned char>(s[i + k]);
            if ((cc & 0xC0) != 0x80) return false;
            cp = (cp << 6) | (cc & 0x3F);
        }
        // 拒绝超长编码、代理区、超出 Unicode 范围
        if ((len == 2 && cp < 0x80) || (len == 3 && cp < 0x800) || (len == 4 && cp < 0x10000)) return false;
        if (cp >= 0xD800 && cp <= 0xDFFF) return false;
        if (cp > 0x10FFFF) return false;
        i += len;
    }
    return true;
}

bool HasControlChar(const std::string& s) {
    for (const unsigned char c : s) {
        if (c < 0x20 || c == 0x7F) return true;
    }
    return false;
}

// 单层目录名（已有地图的名字只做这一项检查，不套用新名字的长度 / 字符规则）
bool IsSingleComponent(const std::string& name) {
    return !name.empty() && name.find('/') == std::string::npos && name.find('\0') == std::string::npos &&
           name != "." && name != "..";
}

// 包内相对路径是否安全：非空、相对、无反斜杠、无空段、无 . / ..、合法 UTF-8、无控制字符
bool IsSafeRelativePath(std::string path) {
    if (path.empty() || path[0] == '/' || path.find('\\') != std::string::npos) return false;
    if (!IsValidUtf8(path) || HasControlChar(path)) return false;
    if (path.back() == '/') path.pop_back();
    if (path.empty()) return false;
    std::stringstream ss(path);
    for (std::string part; std::getline(ss, part, '/');) {
        if (part.empty() || part == "." || part == "..") return false;
    }
    return true;
}

uint64_t FreeBytes(const fs::path& dir) {
    struct statvfs st {};
    if (::statvfs(dir.c_str(), &st) != 0) return UINT64_MAX;  // 查不到就不拦
    return static_cast<uint64_t>(st.f_bavail) * st.f_frsize;
}

struct FileSig {
    ino_t ino = 0;
    off_t size = -1;
    int64_t mtime_ns = 0;
    bool operator==(const FileSig& o) const { return ino == o.ino && size == o.size && mtime_ns == o.mtime_ns; }
};

std::vector<FileSig> TripleSigs(const fs::path& slam_dir) {
    std::vector<FileSig> out;
    for (const char* f : kTripleFiles) {
        struct stat st {};
        FileSig sig;
        if (::stat((slam_dir / f).c_str(), &st) == 0) {
            sig.ino = st.st_ino;
            sig.size = st.st_size;
            sig.mtime_ns = static_cast<int64_t>(st.st_mtim.tv_sec) * 1000000000LL + st.st_mtim.tv_nsec;
        }
        out.push_back(sig);
    }
    return out;
}

// 资产文件锁（与 map_asset_manager 同一把：assets/.map_assets.lock）。非阻塞重试，拿不到视为忙。
class AssetDirLock {
public:
    explicit AssetDirLock(const fs::path& assets_dir) {
        fd_ = ::open((assets_dir / ".map_assets.lock").c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0644);
        if (fd_ < 0) return;
        for (int i = 0; i < kLockRetries; ++i) {
            if (::flock(fd_, LOCK_EX | LOCK_NB) == 0) { locked_ = true; return; }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    ~AssetDirLock() {
        if (fd_ >= 0) {
            if (locked_) ::flock(fd_, LOCK_UN);
            ::close(fd_);
        }
    }
    bool acquired() const { return locked_; }

private:
    int fd_ = -1;
    bool locked_ = false;
};

bool SkipName(const std::string& name) {
    auto ends_with = [&name](const char* suffix) {
        const size_t n = std::strlen(suffix);
        return name.size() >= n && name.compare(name.size() - n, n, suffix) == 0;
    };
    return ends_with(".tmp") || ends_with(".lock") || ends_with(".part");
}

// 按白名单收集要打包的文件（相对 map_dir 的路径）
std::vector<std::string> CollectFiles(const fs::path& map_dir, Content content) {
    std::vector<std::string> out;
    std::error_code ec;
    const fs::path slam = map_dir / "slam";
    for (const char* f : kTripleFiles) {
        if (fs::is_regular_file(slam / f, ec)) out.push_back(std::string("slam/") + f);
    }
    if (fs::is_regular_file(slam / ".complete", ec)) out.push_back("slam/.complete");
    if (fs::is_regular_file(map_dir / "assets" / "map_assets.json", ec)) out.push_back("assets/map_assets.json");
    if (content != Content::kFull) return out;

    for (const char* f : kSlamExtraFiles) {
        if (fs::is_regular_file(slam / f, ec)) out.push_back(std::string("slam/") + f);
    }
    // 底图版本：backup.<ts>/ 下的常规文件；写入中的 .backup.<ts>.tmp/ 与轮换中的 tombstone.<i>/ 不带
    std::vector<std::string> backups;
    for (fs::directory_iterator it(slam / "backups", ec), end; !ec && it != end; it.increment(ec)) {
        const std::string name = it->path().filename().string();
        if (name.rfind("backup.", 0) == 0 && it->is_directory(ec)) backups.push_back(name);
    }
    std::sort(backups.begin(), backups.end());
    for (const auto& b : backups) {
        std::vector<std::string> files;
        for (fs::directory_iterator it(slam / "backups" / b, ec), end; !ec && it != end; it.increment(ec)) {
            const std::string name = it->path().filename().string();
            if (it->is_regular_file(ec) && !SkipName(name)) files.push_back(name);
        }
        std::sort(files.begin(), files.end());
        for (const auto& f : files) out.push_back("slam/backups/" + b + "/" + f);
    }
    // 资产版本：map_assets.<ts>.json / .source
    std::vector<std::string> asset_versions;
    for (fs::directory_iterator it(map_dir / "assets" / "backups", ec), end; !ec && it != end; it.increment(ec)) {
        const std::string name = it->path().filename().string();
        if (it->is_regular_file(ec) && name.rfind("map_assets.", 0) == 0 && !SkipName(name)) {
            asset_versions.push_back(name);
        }
    }
    std::sort(asset_versions.begin(), asset_versions.end());
    for (const auto& f : asset_versions) out.push_back("assets/backups/" + f);
    return out;
}

bool PngSize(const fs::path& png, uint32_t* w, uint32_t* h) {
    std::ifstream in(png, std::ios::binary);
    unsigned char buf[24];
    if (!in.read(reinterpret_cast<char*>(buf), sizeof(buf))) return false;
    static const unsigned char kSig[8] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
    if (std::memcmp(buf, kSig, 8) != 0 || std::memcmp(buf + 12, "IHDR", 4) != 0) return false;
    *w = (uint32_t(buf[16]) << 24) | (uint32_t(buf[17]) << 16) | (uint32_t(buf[18]) << 8) | buf[19];
    *h = (uint32_t(buf[20]) << 24) | (uint32_t(buf[21]) << 16) | (uint32_t(buf[22]) << 8) | buf[23];
    return true;
}

std::string Unquote(std::string s) {
    if (s.size() >= 2 && (s.front() == '"' || s.front() == '\'') && s.back() == s.front()) {
        s = s.substr(1, s.size() - 2);
    }
    return s;
}

std::string HostName() {
    char buf[256] = {0};
    if (::gethostname(buf, sizeof(buf) - 1) != 0) return "";
    return buf;
}

std::string Env(const char* key) {
    const char* v = std::getenv(key);
    return v ? v : "";
}

// "1.2" → (1, 2)；格式不对返回 false
bool ParseMajorMinor(const std::string& v, int* major, int* minor) {
    int a = 0, b = 0;
    char dot = 0;
    std::istringstream ss(v);
    if (!(ss >> a)) return false;
    *major = a;
    *minor = 0;
    if (ss >> dot) {
        if (dot != '.' || !(ss >> b)) return false;
        *minor = b;
    }
    return true;
}

// map.yaml 的 data_version 是否在本版本能读的范围内：空 / legacy / 不高于 0.3.x
bool DataVersionSupported(const std::string& raw) {
    const std::string v = Unquote(raw);
    if (v.empty() || v == "legacy") return true;
    int major = 0, minor = 0;
    if (!ParseMajorMinor(v.substr(0, v.find('.', v.find('.') + 1)), &major, &minor)) return false;
    return major == 0 && minor <= 3;
}

bool CopyRegularFile(const fs::path& src, const fs::path& dst, std::string* error) {
    std::error_code ec;
    fs::create_directories(dst.parent_path(), ec);
    fs::copy_file(src, dst, fs::copy_options::overwrite_existing, ec);
    if (ec) {
        *error = "copy " + src.string() + " failed: " + ec.message();
        return false;
    }
    struct stat st {};
    if (::stat(src.c_str(), &st) == 0) {
        // 保留原文件时间，压缩包里看到的就是地图文件真实的修改时间
        fs::last_write_time(dst, fs::last_write_time(src, ec), ec);
    }
    return true;
}

bool IsStoredUncompressed(const std::string& rel) {
    auto ends_with = [&rel](const char* suffix) {
        const size_t n = std::strlen(suffix);
        return rel.size() >= n && rel.compare(rel.size() - n, n, suffix) == 0;
    };
    // png / pbstream 本身已压缩，再 deflate 只费时间
    return ends_with(".png") || ends_with(".pbstream");
}

bool AddEntry(struct archive* a, const std::string& path_in_zip, const std::string& data_from_file,
              const std::string* data_inline, std::time_t mtime, std::string* error) {
    uint64_t size = 0;
    std::ifstream in;
    if (data_inline != nullptr) {
        size = data_inline->size();
    } else {
        in.open(data_from_file, std::ios::binary);
        if (!in) {
            *error = "open " + data_from_file + " failed";
            return false;
        }
        std::error_code ec;
        size = fs::file_size(data_from_file, ec);
        if (ec) {
            *error = "stat " + data_from_file + " failed: " + ec.message();
            return false;
        }
    }
    if (IsStoredUncompressed(path_in_zip)) archive_write_zip_set_compression_store(a);
    else archive_write_zip_set_compression_deflate(a);

    struct archive_entry* entry = archive_entry_new();
    archive_entry_set_pathname_utf8(entry, path_in_zip.c_str());
    if (archive_entry_pathname(entry) == nullptr) {
        // 路径无法按当前 locale 编码时 libarchive 会崩溃，先拦下来
        *error = "cannot encode path in package (no UTF-8 locale?): " + path_in_zip;
        archive_entry_free(entry);
        return false;
    }
    archive_entry_set_filetype(entry, AE_IFREG);
    archive_entry_set_perm(entry, 0644);
    archive_entry_set_size(entry, static_cast<la_int64_t>(size));
    archive_entry_set_mtime(entry, mtime, 0);
    if (archive_write_header(a, entry) != ARCHIVE_OK) {
        *error = std::string("write zip header failed: ") + archive_error_string(a);
        archive_entry_free(entry);
        return false;
    }
    archive_entry_free(entry);
    if (data_inline != nullptr) {
        if (!data_inline->empty() &&
            archive_write_data(a, data_inline->data(), data_inline->size()) < 0) {
            *error = std::string("write zip data failed: ") + archive_error_string(a);
            return false;
        }
        return true;
    }
    std::vector<char> buf(kIoChunk);
    while (in) {
        in.read(buf.data(), static_cast<std::streamsize>(buf.size()));
        const std::streamsize n = in.gcount();
        if (n > 0 && archive_write_data(a, buf.data(), static_cast<size_t>(n)) < 0) {
            *error = std::string("write zip data failed: ") + archive_error_string(a);
            return false;
        }
    }
    return true;
}

struct ZipEntryInfo {
    std::string path;
    bool is_dir = false;
    bool is_regular = false;
    uint64_t size = 0;
};

const char* EntryPath(struct archive_entry* entry) {
    const char* p = archive_entry_pathname_utf8(entry);
    return p ? p : archive_entry_pathname(entry);
}

// 只读条目目录（不解压）。open_failed = 打不开或不是 zip。
bool ListZip(const std::string& path, uint64_t max_entries, std::vector<ZipEntryInfo>* entries,
             bool* open_failed, std::string* error) {
    *open_failed = false;
    struct archive* a = archive_read_new();
    archive_read_support_format_zip_seekable(a);
    if (archive_read_open_filename(a, path.c_str(), kIoChunk) != ARCHIVE_OK) {
        *open_failed = true;
        *error = std::string("cannot open as zip: ") + archive_error_string(a);
        archive_read_free(a);
        return false;
    }
    bool ok = true;
    struct archive_entry* entry = nullptr;
    int rc = ARCHIVE_OK;
    while ((rc = archive_read_next_header(a, &entry)) == ARCHIVE_OK || rc == ARCHIVE_WARN) {
        if (entries->size() >= max_entries) {
            *error = "too many entries in package (limit " + std::to_string(max_entries) + ")";
            ok = false;
            break;
        }
        ZipEntryInfo info;
        const char* p = EntryPath(entry);
        info.path = p ? p : "";
        const auto type = archive_entry_filetype(entry);
        info.is_dir = (type == AE_IFDIR);
        info.is_regular = (type == AE_IFREG) && archive_entry_hardlink(entry) == nullptr &&
                          archive_entry_symlink(entry) == nullptr;
        info.size = archive_entry_size_is_set(entry) ? static_cast<uint64_t>(archive_entry_size(entry)) : 0;
        entries->push_back(info);
        archive_read_data_skip(a);
    }
    if (ok && rc != ARCHIVE_EOF) {
        // 第一个条目都读不出来，说明根本不是 zip
        *open_failed = entries->empty();
        *error = std::string("corrupt zip: ") + archive_error_string(a);
        ok = false;
    }
    archive_read_free(a);
    return ok;
}

// 解压全部常规文件到 dest_root（路径已在 ListZip 阶段校验过，这里再校验一次）。
// 按实际解压出的字节计数，超过 max_bytes 立即停止（声明大小可以造假）。
bool ExtractZip(const std::string& path, const fs::path& dest_root, uint64_t max_bytes, std::string* error) {
    struct archive* a = archive_read_new();
    archive_read_support_format_zip_seekable(a);
    if (archive_read_open_filename(a, path.c_str(), kIoChunk) != ARCHIVE_OK) {
        *error = std::string("cannot open as zip: ") + archive_error_string(a);
        archive_read_free(a);
        return false;
    }
    bool ok = true;
    uint64_t total = 0;
    std::vector<char> buf(kIoChunk);
    struct archive_entry* entry = nullptr;
    int rc = ARCHIVE_OK;
    while (ok && ((rc = archive_read_next_header(a, &entry)) == ARCHIVE_OK || rc == ARCHIVE_WARN)) {
        const char* p = EntryPath(entry);
        const std::string rel = p ? p : "";
        if (archive_entry_filetype(entry) == AE_IFDIR) { archive_read_data_skip(a); continue; }
        if (archive_entry_filetype(entry) != AE_IFREG || !IsSafeRelativePath(rel)) {
            *error = "unsafe entry in package: " + rel;
            ok = false;
            break;
        }
        const fs::path dst = dest_root / rel;
        std::error_code ec;
        fs::create_directories(dst.parent_path(), ec);
        std::ofstream out(dst, std::ios::binary | std::ios::trunc);
        if (!out) {
            *error = "cannot write " + dst.string();
            ok = false;
            break;
        }
        for (;;) {
            const la_ssize_t n = archive_read_data(a, buf.data(), buf.size());
            if (n == 0) break;
            if (n < 0) {
                *error = std::string("corrupt zip data in ") + rel + ": " + archive_error_string(a);
                ok = false;
                break;
            }
            total += static_cast<uint64_t>(n);
            if (total > max_bytes) {
                *error = "unpacked size exceeds limit (" + std::to_string(max_bytes >> 20) + " MiB)";
                ok = false;
                break;
            }
            out.write(buf.data(), n);
        }
        if (ok && !out) {
            *error = "write failed: " + dst.string();
            ok = false;
        }
    }
    if (ok && rc != ARCHIVE_EOF) {
        *error = std::string("corrupt zip: ") + archive_error_string(a);
        ok = false;
    }
    archive_read_free(a);
    return ok;
}

// renameat2(RENAME_NOREPLACE)：目标已存在就失败（errno = EEXIST），绝不覆盖。
// 文件系统不支持时退回「先查再改名」并告警。
int RenameNoReplace(const fs::path& src, const fs::path& dst) {
    if (::renameat2(AT_FDCWD, src.c_str(), AT_FDCWD, dst.c_str(), RENAME_NOREPLACE) == 0) return 0;
    if (errno != EINVAL && errno != ENOSYS) return errno;
    LOG(WARNING) << "renameat2(RENAME_NOREPLACE) unsupported here; falling back to check-then-rename: " << dst;
    std::error_code ec;
    if (fs::exists(dst, ec)) return EEXIST;
    return ::rename(src.c_str(), dst.c_str()) == 0 ? 0 : errno;
}

std::string CopyName(const std::string& base, int n) {
    return n == 0 ? base : base + "_copy" + std::to_string(n);
}

}  // namespace

std::string FileSha256(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return "";
    EVP_MD_CTX* ctx = EVP_MD_CTX_new();
    if (ctx == nullptr) return "";
    std::string out;
    if (EVP_DigestInit_ex(ctx, EVP_sha256(), nullptr) == 1) {
        std::vector<char> buf(kIoChunk);
        bool ok = true;
        while (in) {
            in.read(buf.data(), static_cast<std::streamsize>(buf.size()));
            const std::streamsize n = in.gcount();
            if (n > 0 && EVP_DigestUpdate(ctx, buf.data(), static_cast<size_t>(n)) != 1) { ok = false; break; }
        }
        unsigned char md[EVP_MAX_MD_SIZE];
        unsigned int len = 0;
        if (ok && EVP_DigestFinal_ex(ctx, md, &len) == 1) {
            static const char* kHex = "0123456789abcdef";
            out.reserve(len * 2);
            for (unsigned int i = 0; i < len; ++i) {
                out.push_back(kHex[md[i] >> 4]);
                out.push_back(kHex[md[i] & 0x0F]);
            }
        }
    }
    EVP_MD_CTX_free(ctx);
    return out;
}

bool ValidateNewMapName(const std::string& name, std::string* error) {
    auto fail = [error](const std::string& m) {
        if (error) *error = m;
        return false;
    };
    if (name.empty()) return fail("map name is empty");
    if (!IsValidUtf8(name)) return fail("map name is not valid UTF-8");
    if (name.size() > kMaxMapNameBytes) {
        return fail("map name is too long: " + std::to_string(name.size()) + " bytes (limit " +
                    std::to_string(kMaxMapNameBytes) + " bytes, about 66 Chinese characters)");
    }
    if (name == "." || name == "..") return fail("map name must not be '.' or '..'");
    if (name[0] == '.' || name[0] == '-') return fail("map name must not start with '.' or '-'");
    // 逐个码点检查：控制字符、不可见格式字符、路径与 Windows 文件名不允许的字符、单引号
    std::vector<uint32_t> cps;
    for (size_t i = 0; i < name.size();) {
        const unsigned char c = static_cast<unsigned char>(name[i]);
        const int len = c < 0x80 ? 1 : (c & 0xE0) == 0xC0 ? 2 : (c & 0xF0) == 0xE0 ? 3 : 4;
        uint32_t cp = len == 1 ? c : len == 2 ? (c & 0x1F) : len == 3 ? (c & 0x0F) : (c & 0x07);
        for (int k = 1; k < len; ++k) cp = (cp << 6) | (static_cast<unsigned char>(name[i + k]) & 0x3F);
        cps.push_back(cp);
        i += len;
    }
    for (const uint32_t cp : cps) {
        if (cp < 0x20 || cp == 0x7F || (cp >= 0x80 && cp <= 0x9F)) return fail("map name contains control characters");
        if ((cp >= 0x200B && cp <= 0x200F) || (cp >= 0x202A && cp <= 0x202E) || (cp >= 0x2060 && cp <= 0x2064) ||
            cp == 0x2028 || cp == 0x2029 || cp == 0xFEFF) {
            return fail("map name contains invisible characters (zero-width / direction marks)");
        }
        // / 是路径分隔；\\ : * ? " < > | 在 Windows 上不能做文件名（导出包在 Windows 解压）；
        // ' $ ` 会破坏脚本里的引号或被 shell 展开
        if (cp < 0x80 && std::strchr("/\\:*?\"<>|'$`", static_cast<int>(cp)) != nullptr) {
            return fail("map name must not contain any of / \\ : * ? \" < > | ' $ `");
        }
    }
    auto is_space = [](uint32_t cp) { return cp == 0x20 || cp == 0xA0 || cp == 0x3000; };
    if (is_space(cps.front()) || is_space(cps.back())) {
        return fail("map name must not start or end with a space (including full-width space)");
    }
    if (error) error->clear();
    return true;
}

bool IsLoadableMapDirectory(const std::string& map_folder, const std::string& name) {
    if (!IsSingleComponent(name) || name[0] == '.') return false;
    const fs::path dir = fs::path(map_folder) / name;
    std::error_code ec;
    if (!fs::is_directory(dir, ec)) return false;
    return fs::exists(dir / "slam" / "map.yaml", ec) || fs::exists(dir / "map.yaml", ec);
}

std::vector<MapListItem> ListMaps(const std::string& map_folder) {
    std::vector<MapListItem> out;
    std::error_code ec;
    for (fs::directory_iterator it(map_folder, ec), end; !ec && it != end; it.increment(ec)) {
        const std::string name = it->path().filename().string();
        if (!IsLoadableMapDirectory(map_folder, name)) continue;
        fs::path yaml = it->path() / "slam" / "map.yaml";
        if (!fs::exists(yaml, ec)) yaml = it->path() / "map.yaml";
        const MapYamlDoc doc = MapYamlDoc::Load(yaml.string());
        MapListItem item;
        item.name = name;
        item.created_at = Unquote(doc.Get("created_at").value_or(""));
        item.last_edited_at = Unquote(doc.Get("last_edited_at").value_or(""));
        out.push_back(std::move(item));
    }
    std::sort(out.begin(), out.end(), [](const MapListItem& a, const MapListItem& b) { return a.name < b.name; });
    return out;
}

bool MoveMapToTrash(const std::string& map_folder, const std::string& trash_dirname, const std::string& name,
                    std::string* entry, std::string* error) {
    std::error_code ec;
    const fs::path trash = fs::path(map_folder) / trash_dirname;
    fs::create_directories(trash, ec);
    if (ec) {
        *error = "failed to create trash dir " + trash.string() + ": " + ec.message();
        return false;
    }
    // 命名与建图侧 ResetMapDirectory、map_command OP_TRASH 一致：<name>_<YYYYMMDD_HHMMSS>，同秒追加 _<N>
    const std::string ts = NowStamp();
    for (int n = 1; n < 1000; ++n) {
        const std::string candidate = n == 1 ? name + "_" + ts : name + "_" + ts + "_" + std::to_string(n);
        const int rc = RenameNoReplace(fs::path(map_folder) / name, trash / candidate);
        if (rc == 0) {
            *entry = candidate;
            return true;
        }
        if (rc != EEXIST && rc != ENOTEMPTY) {
            *error = "failed to move " + name + " to trash: " + std::strerror(rc);
            return false;
        }
    }
    *error = "failed to find a free trash entry name for " + name;
    return false;
}

int CleanupExpired(const Options& options) {
    if (options.map_folder.empty() || options.retention_min < 0) return 0;
    const auto now = fs::file_time_type::clock::now();
    const auto keep = std::chrono::minutes(options.retention_min);
    int removed = 0;
    for (const char* sub : {kExportsDirname, kImportsDirname}) {
        const fs::path dir = fs::path(options.map_folder) / sub;
        std::error_code ec;
        std::vector<fs::path> victims;
        for (fs::directory_iterator it(dir, ec), end; !ec && it != end; it.increment(ec)) {
            const std::string name = it->path().filename().string();
            const bool staging = it->is_directory(ec) && name.rfind(".staging_", 0) == 0;
            if (!staging && !it->is_regular_file(ec)) continue;
            const auto mtime = it->last_write_time(ec);
            if (ec) { ec.clear(); continue; }
            if (now - mtime > keep) victims.push_back(it->path());
        }
        for (const auto& v : victims) {
            std::error_code rm_ec;
            fs::remove_all(v, rm_ec);
            if (!rm_ec) ++removed;
        }
    }
    return removed;
}

Result ExportMap(const Options& options, const std::string& map_name, Content content, const std::string& out_dir_in) {
    Result r;
    r.map_name = map_name;
    if (options.map_folder.empty()) return Fail(kErrFolderMissing, "map_folder is not configured", r);
    if (!IsSingleComponent(map_name) || map_name[0] == '.') return Fail(kErrNameInvalid, "invalid map name: " + map_name, r);
    if (!IsValidUtf8(map_name)) return Fail(kErrNameInvalid, "map name is not valid UTF-8; rename it before exporting", r);

    const fs::path map_dir = fs::path(options.map_folder) / map_name;
    const fs::path slam_dir = map_dir / "slam";
    std::error_code ec;
    if (!fs::is_directory(map_dir, ec)) return Fail(kErrNotFound, "map not found: " + map_name, r);
    if (!fs::exists(slam_dir / "map.yaml", ec)) {
        if (fs::exists(map_dir / "map.yaml", ec)) {
            return Fail(kErrLoadFailed, "legacy flat map layout; load the map once to migrate it before exporting", r);
        }
        return Fail(kErrNotFound, "map not found (no slam/map.yaml): " + map_name, r);
    }
    for (const char* t : kSwapTransients) {
        if (fs::exists(map_dir / t, ec)) {
            return Fail(kErrIoFailed, "map is being saved by localization; retry later", r);
        }
    }
    for (const char* f : kTripleFiles) {
        if (!fs::is_regular_file(slam_dir / f, ec)) {
            return Fail(kErrLoadFailed, std::string("map is incomplete, missing slam/") + f, r);
        }
    }

    const fs::path out_dir = out_dir_in.empty() ? fs::path(options.map_folder) / kExportsDirname : fs::path(out_dir_in);
    fs::create_directories(out_dir, ec);
    if (ec) return Fail(kErrIoFailed, "cannot create output dir " + out_dir.string() + ": " + ec.message(), r);
    CleanupExpired(options);

    uint64_t total = 0;
    for (const auto& rel : CollectFiles(map_dir, content)) {
        total += fs::file_size(map_dir / rel, ec);
        ec.clear();
    }
    if (FreeBytes(out_dir) < total * 2 + options.min_free_bytes) {
        return Fail(kErrNoSpace, "not enough disk space to export " + map_name + " (needs about " +
                                     std::to_string((total * 2 + options.min_free_bytes) >> 20) + " MiB)", r);
    }

    // 锁内复制快照：持锁时间只等于复制时间。复制前后比对三件的 (inode, 大小, mtime)，
    // 兜住不经过地图操作锁的写入方（定位侧保存）；变了就重做一次，再变就返回忙。
    const std::string stamp = NowStamp();
    const fs::path staging = out_dir / StagingName(stamp);
    std::vector<std::string> files;
    bool consistent = false;
    for (int attempt = 0; attempt < 2 && !consistent; ++attempt) {
        std::unique_ptr<MapOpLock> op_lock;
        for (int i = 0; i < kLockRetries; ++i) {
            op_lock = std::make_unique<MapOpLock>(slam_dir.string());
            if (op_lock->acquired()) break;
            op_lock.reset();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (!op_lock) {
            return Fail(kErrIoFailed, "map is being modified by another operation (edit / rollback / align / "
                                      "replace / reset origin); retry later", r);
        }
        std::unique_ptr<AssetDirLock> asset_lock;
        if (fs::is_directory(map_dir / "assets", ec)) {
            asset_lock = std::make_unique<AssetDirLock>(map_dir / "assets");
            if (!asset_lock->acquired()) return Fail(kErrIoFailed, "map assets are being saved; retry later", r);
        }
        const auto before = TripleSigs(slam_dir);
        files = CollectFiles(map_dir, content);
        fs::remove_all(staging, ec);
        std::string error;
        for (const auto& rel : files) {
            if (!CopyRegularFile(map_dir / rel, staging / map_name / rel, &error)) {
                fs::remove_all(staging, ec);
                return Fail(kErrIoFailed, error, r);
            }
        }
        consistent = (TripleSigs(slam_dir) == before);
    }
    if (!consistent) {
        fs::remove_all(staging, ec);
        return Fail(kErrIoFailed, "map files changed while exporting (localization may be saving); retry later", r);
    }

    // 清单
    const fs::path staged_map = staging / map_name;
    const MapYamlDoc yaml = MapYamlDoc::Load((staged_map / "slam" / "map.yaml").string());
    ordered_json manifest;
    manifest["format"] = kFormatName;
    manifest["format_version"] = std::to_string(kFormatMajor) + "." + std::to_string(kFormatMinor);
    manifest["content"] = content == Content::kFull ? "full" : "standard";
    manifest["exported_at"] = LocalTimeIso8601();
    manifest["source"] = {{"hostname", HostName()},
                          {"robot_type", Env("DEXE_ROBOT_TYPE")},
                          {"robot_version", Env("DEXE_ROBOT_VERSION")},
                          {"map_manager_version", MAP_MANAGER_VERSION}};
    ordered_json m;
    m["name"] = map_name;
    m["data_version"] = Unquote(yaml.Get("data_version").value_or(""));
    // 指纹按原地图路径算（与 get_maps 同一缓存），快照前后一致已在上面核过
    m["grid_revision"] = GridRevision(slam_dir.string(), MapYamlDoc::Load((slam_dir / "map.yaml").string()));
    m["created_at"] = Unquote(yaml.Get("created_at").value_or(""));
    m["last_edited_at"] = Unquote(yaml.Get("last_edited_at").value_or(""));
    m["resolution"] = yaml.GetDouble("resolution").value_or(0.0);
    uint32_t w = 0, h = 0;
    if (PngSize(staged_map / "slam" / "map.png", &w, &h)) m["size_px"] = {w, h};
    const fs::path assets_json = staged_map / "assets" / "map_assets.json";
    m["has_assets"] = fs::exists(assets_json, ec);
    if (fs::exists(assets_json, ec)) {
        try {
            std::ifstream in(assets_json);
            const auto j = nlohmann::json::parse(in);
            if (j.contains("schema_version") && j["schema_version"].is_string()) {
                m["assets_schema_version"] = j["schema_version"].get<std::string>();
            }
            if (j.contains("last_edited_at") && j["last_edited_at"].is_string()) {
                m["assets_last_edited_at"] = j["last_edited_at"].get<std::string>();
            }
        } catch (const std::exception& e) {
            LOG(WARNING) << "export " << map_name << ": cannot parse map_assets.json: " << e.what();
        }
    }
    manifest["map"] = m;
    ordered_json file_list = ordered_json::array();
    for (const auto& rel : files) {
        const fs::path p = staged_map / rel;
        const std::string sha = FileSha256(p.string());
        if (sha.empty()) {
            fs::remove_all(staging, ec);
            return Fail(kErrIoFailed, "cannot read " + p.string(), r);
        }
        file_list.push_back({{"path", map_name + "/" + rel}, {"size", fs::file_size(p, ec)}, {"sha256", sha}});
    }
    manifest["files"] = file_list;
    const std::string manifest_text = manifest.dump(2) + "\n";

    // 写 zip：先写 .zip.part，写完改名，调用方看到的 .zip 一定完整
    fs::path zip_path;
    for (int n = 1; n < 1000; ++n) {
        const std::string base = map_name + "_" + stamp + (n == 1 ? "" : "_" + std::to_string(n)) + ".zip";
        if (!fs::exists(out_dir / base, ec) && !fs::exists(out_dir / (base + ".part"), ec)) {
            zip_path = out_dir / base;
            break;
        }
    }
    if (zip_path.empty()) {
        fs::remove_all(staging, ec);
        return Fail(kErrIoFailed, "cannot find a free package file name in " + out_dir.string(), r);
    }
    const fs::path part = zip_path.string() + ".part";
    ScopedUtf8Locale utf8_locale;
    struct archive* a = archive_write_new();
    archive_write_set_format_zip(a);
    // 条目名按 UTF-8 写并打上 UTF-8 标志位，中文图名在 Windows 上解压不乱码
    archive_write_set_options(a, "zip:hdrcharset=UTF-8");
    std::string error;
    bool ok = archive_write_open_filename(a, part.c_str()) == ARCHIVE_OK;
    if (!ok) error = std::string("cannot create ") + part.string() + ": " + archive_error_string(a);
    const std::time_t now = std::time(nullptr);
    if (ok) ok = AddEntry(a, "manifest.json", "", &manifest_text, now, &error);
    for (size_t i = 0; ok && i < files.size(); ++i) {
        const fs::path p = staged_map / files[i];
        const auto mt = fs::last_write_time(p, ec);
        const std::time_t mtime = ec ? now : std::chrono::system_clock::to_time_t(
            std::chrono::time_point_cast<std::chrono::system_clock::duration>(
                mt - fs::file_time_type::clock::now() + std::chrono::system_clock::now()));
        ok = AddEntry(a, map_name + "/" + files[i], p.string(), nullptr, mtime, &error);
    }
    if (archive_write_close(a) != ARCHIVE_OK && ok) {
        error = std::string("finish zip failed: ") + archive_error_string(a);
        ok = false;
    }
    archive_write_free(a);
    fs::remove_all(staging, ec);
    if (!ok) {
        fs::remove(part, ec);
        return Fail(kErrIoFailed, error, r);
    }
    fs::rename(part, zip_path, ec);
    if (ec) {
        fs::remove(part, ec);
        return Fail(kErrIoFailed, "rename package failed: " + ec.message(), r);
    }
    r.file_path = zip_path.string();
    r.size_bytes = fs::file_size(zip_path, ec);
    r.sha256 = FileSha256(r.file_path);
    r.message = "map exported: " + map_name + " -> " + r.file_path;
    return r;
}

Result ImportMap(const Options& options, const ImportRequest& req) {
    Result r;
    if (options.map_folder.empty()) return Fail(kErrFolderMissing, "map_folder is not configured", r);
    std::error_code ec;
    const fs::path map_folder = fs::path(options.map_folder);
    const fs::path imports_dir = map_folder / kImportsDirname;
    fs::create_directories(imports_dir, ec);
    CleanupExpired(options);

    // 1. 文件本身
    if (req.file_path.empty()) return Fail(kErrImportFileInvalid, "file_path is empty", r);
    const fs::path file = fs::weakly_canonical(fs::path(req.file_path), ec);
    if (ec || !fs::exists(file, ec)) return Fail(kErrImportFileInvalid, "file not found: " + req.file_path, r);
    if (!fs::is_regular_file(file, ec)) return Fail(kErrImportFileInvalid, "not a regular file: " + req.file_path, r);
    bool in_imports_dir = false;
    {
        const std::string root = fs::weakly_canonical(imports_dir, ec).string() + "/";
        in_imports_dir = file.string().rfind(root, 0) == 0;
    }
    if (req.require_imports_dir && !in_imports_dir) {
        return Fail(kErrImportFileInvalid, "file must be under " + imports_dir.string() + "/: " + req.file_path, r);
    }
    const uint64_t file_size = fs::file_size(file, ec);
    if (file_size > options.max_package_bytes) {
        return Fail(kErrImportFileInvalid, "file is too large (limit " + std::to_string(options.max_package_bytes >> 20) +
                                               " MiB): " + req.file_path, r);
    }

    // 2. 只读条目目录：路径安全、类型、单一顶层目录、条目数与声明大小
    ScopedUtf8Locale utf8_locale;
    std::vector<ZipEntryInfo> entries;
    bool open_failed = false;
    std::string error;
    if (!ListZip(file.string(), options.max_entries, &entries, &open_failed, &error)) {
        return Fail(open_failed ? kErrImportFileInvalid : kErrImportIncompatible, error, r);
    }
    // 先看有没有清单：没有的多半是把几个包又压了一层，直接给出处理办法
    const bool has_manifest = std::any_of(entries.begin(), entries.end(), [](const ZipEntryInfo& e) {
        return e.path == "manifest.json" && e.is_regular;
    });
    if (!has_manifest) {
        return Fail(kErrImportIncompatible, "no manifest.json in package; if this file bundles several packages, "
                                            "unzip it and import them one by one", r);
    }
    std::string top;
    uint64_t declared = 0;
    for (const auto& e : entries) {
        if (!IsSafeRelativePath(e.path)) return Fail(kErrImportIncompatible, "unsafe path in package: " + e.path, r);
        if (!e.is_dir && !e.is_regular) {
            return Fail(kErrImportIncompatible, "package contains a link or special file: " + e.path, r);
        }
        if (e.path == "manifest.json") continue;
        const std::string first = e.path.substr(0, e.path.find('/'));
        if (first == e.path && e.is_regular) {
            return Fail(kErrImportIncompatible, "unexpected top-level file in package: " + e.path, r);
        }
        if (top.empty()) top = first;
        if (first != top) {
            return Fail(kErrImportIncompatible, "package contains more than one map directory (" + top + ", " + first +
                                                    "); export one map per package", r);
        }
        declared += e.size;
    }
    if (top.empty()) return Fail(kErrImportIncompatible, "package contains no map directory", r);
    if (declared > options.max_unpacked_bytes) {
        return Fail(kErrImportIncompatible, "unpacked size exceeds limit (" +
                                                std::to_string(options.max_unpacked_bytes >> 20) + " MiB)", r);
    }

    // 3. 空间
    if (FreeBytes(imports_dir) < declared + options.min_free_bytes) {
        return Fail(kErrNoSpace, "not enough disk space to import (needs about " +
                                     std::to_string((declared + options.min_free_bytes) >> 20) + " MiB)", r);
    }

    // 4. 解压到暂存目录（与地图目录同一块盘，最后一步 rename 才是原子的）
    const fs::path staging = imports_dir / StagingName(NowStamp());
    fs::remove_all(staging, ec);
    fs::create_directories(staging, ec);
    struct StagingGuard {
        fs::path dir;
        ~StagingGuard() {
            std::error_code e;
            fs::remove_all(dir, e);
        }
    } guard{staging};
    if (!ExtractZip(file.string(), staging, options.max_unpacked_bytes, &error)) {
        return Fail(kErrImportIncompatible, error, r);
    }

    // 5. 清单与内容校验
    ordered_json manifest;
    try {
        std::ifstream in(staging / "manifest.json");
        manifest = ordered_json::parse(in);
    } catch (const std::exception& e) {
        return Fail(kErrImportIncompatible, std::string("manifest.json is not valid JSON: ") + e.what(), r);
    }
    if (!manifest.is_object() || manifest.value("format", "") != kFormatName) {
        return Fail(kErrImportIncompatible, "not a map package (manifest format is not " + std::string(kFormatName) + ")", r);
    }
    int major = 0, minor = 0;
    if (!ParseMajorMinor(manifest.value("format_version", ""), &major, &minor) || major != kFormatMajor) {
        return Fail(kErrImportIncompatible, "unsupported package format_version: " + manifest.value("format_version", "") +
                                                " (supported " + std::to_string(kFormatMajor) + ".x)", r);
    }
    if (!manifest.contains("map") || !manifest["map"].is_object() ||
        manifest["map"].value("name", "") != top) {
        return Fail(kErrImportIncompatible, "manifest map.name does not match the map directory in package", r);
    }
    if (!manifest.contains("files") || !manifest["files"].is_array()) {
        return Fail(kErrImportIncompatible, "manifest has no file list", r);
    }
    std::set<std::string> listed;
    for (const auto& f : manifest["files"]) {
        const std::string path = f.value("path", "");
        if (!IsSafeRelativePath(path) || path.rfind(top + "/", 0) != 0) {
            return Fail(kErrImportIncompatible, "manifest lists an invalid path: " + path, r);
        }
        const fs::path p = staging / path;
        if (!fs::is_regular_file(p, ec)) return Fail(kErrImportIncompatible, "file listed in manifest is missing: " + path, r);
        if (f.contains("size") && f["size"].is_number_unsigned() && f["size"].get<uint64_t>() != fs::file_size(p, ec)) {
            return Fail(kErrImportIncompatible, "size mismatch: " + path, r);
        }
        if (FileSha256(p.string()) != f.value("sha256", "")) {
            return Fail(kErrImportIncompatible, "sha256 mismatch (package corrupted or modified): " + path, r);
        }
        listed.insert(path);
    }
    for (fs::recursive_directory_iterator it(staging / top, ec), end; !ec && it != end; it.increment(ec)) {
        if (!it->is_regular_file(ec)) continue;
        const std::string rel = top + "/" + fs::relative(it->path(), staging / top, ec).string();
        if (listed.count(rel) == 0) return Fail(kErrImportIncompatible, "file not listed in manifest: " + rel, r);
    }
    const fs::path staged_map = staging / top;
    for (const char* f : kTripleFiles) {
        if (!fs::is_regular_file(staged_map / "slam" / f, ec)) {
            return Fail(kErrImportIncompatible, std::string("package is missing slam/") + f, r);
        }
    }
    try {
        const YAML::Node y = YAML::LoadFile((staged_map / "slam" / "map.yaml").string());
        if (!y.IsMap() || !y["image"] || Unquote(y["image"].as<std::string>()) != "map.png" || !y["resolution"] ||
            !y["origin"]) {
            return Fail(kErrImportIncompatible, "map.yaml is invalid (image must be map.png, resolution and origin required)", r);
        }
    } catch (const std::exception& e) {
        return Fail(kErrImportIncompatible, std::string("map.yaml cannot be parsed: ") + e.what(), r);
    }
    const std::string data_version =
        Unquote(MapYamlDoc::Load((staged_map / "slam" / "map.yaml").string()).Get("data_version").value_or(""));
    if (!DataVersionSupported(data_version)) {
        return Fail(kErrImportIncompatible, "map data_version " + data_version + " is newer than this software supports (0.3.x)", r);
    }
    const fs::path assets_json = staged_map / "assets" / "map_assets.json";
    if (fs::exists(assets_json, ec)) {
        try {
            std::ifstream in(assets_json);
            const auto j = nlohmann::json::parse(in);
            if (!j.is_object()) return Fail(kErrImportIncompatible, "map_assets.json is not a JSON object", r);
            int a_major = 0, a_minor = 0;
            if (j.contains("schema_version") && j["schema_version"].is_string() &&
                ParseMajorMinor(j["schema_version"].get<std::string>(), &a_major, &a_minor) && a_major > 2) {
                return Fail(kErrImportIncompatible, "map_assets.json schema_version " +
                                                        j["schema_version"].get<std::string>() + " is not supported", r);
            }
        } catch (const std::exception& e) {
            return Fail(kErrImportIncompatible, std::string("map_assets.json cannot be parsed: ") + e.what(), r);
        }
    }

    // 6. 定名字
    const std::string target = req.new_name.empty() ? top : req.new_name;
    if (!ValidateNewMapName(target, &error)) {
        r.map_name = target;
        return Fail(kErrNameInvalid, error + (req.new_name.empty() ? " (name taken from package; pass a new name)" : ""), r);
    }

    // 7. 落盘：与改地图目录的其它操作串行；renameat2 不覆盖，被抢先占用时按策略继续
    std::unique_lock<std::mutex> lock;
    if (req.commit_mutex) lock = std::unique_lock<std::mutex>(*req.commit_mutex);
    for (int n = 0; n <= kMaxCopySuffix; ++n) {
        const std::string candidate = CopyName(target, n);
        const fs::path dst = map_folder / candidate;
        std::string trashed;
        if (fs::exists(dst, ec)) {
            if (req.on_conflict == OnConflict::kRename) continue;
            if (req.on_conflict == OnConflict::kReject) {
                return Fail(kErrNameExists, "a map named " + candidate + " already exists", r);
            }
            if (req.is_current_map && req.is_current_map(candidate)) {
                return Fail(kErrInUse, "cannot overwrite the currently loaded map: " + candidate, r);
            }
            if (!IsLoadableMapDirectory(options.map_folder, candidate)) {
                return Fail(kErrNameExists, candidate + " exists but is not a complete map (mapping in progress?); "
                                                        "not overwritten", r);
            }
            if (!options.trash_enable) {
                return Fail(kErrTrashDisabled, "map trash is disabled; cannot overwrite " + candidate, r);
            }
            if (!MoveMapToTrash(options.map_folder, options.trash_dirname, candidate, &trashed, &error)) {
                return Fail(kErrIoFailed, error, r);
            }
        }
        const int rc = RenameNoReplace(staged_map, dst);
        if (rc == 0) {
            r.map_name = candidate;
            r.trashed_entry = trashed;
            break;
        }
        if (!trashed.empty()) {
            // 旧图已进回收站但新图没放进去：原样移回
            ::rename((map_folder / options.trash_dirname / trashed).c_str(), dst.c_str());
        }
        if ((rc == EEXIST || rc == ENOTEMPTY) && req.on_conflict == OnConflict::kRename) continue;
        if (rc == EEXIST || rc == ENOTEMPTY) {
            return Fail(kErrNameExists, "a map named " + candidate + " appeared while importing; not overwritten", r);
        }
        return Fail(kErrIoFailed, "failed to place imported map: " + std::string(std::strerror(rc)), r);
    }
    if (r.map_name.empty()) return Fail(kErrNameExists, "no free name for " + target + " (tried _copy1 .. _copy999)", r);
    if (lock.owns_lock()) lock.unlock();

    // 8. 成功：删掉上传的文件（只删导入目录里的，命令行工具传入的任意路径不动）
    if (in_imports_dir) fs::remove(file, ec);
    r.message = "map imported: " + r.map_name + (r.trashed_entry.empty() ? "" : " (previous map moved to trash: " +
                                                                                    r.trashed_entry + ")");
    return r;
}

}  // namespace map_package
}  // namespace map_manager
