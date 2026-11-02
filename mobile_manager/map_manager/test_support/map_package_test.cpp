// 地图导入导出（map_package_core）测试，不起 ROS 节点。
//
// 运行：ros2 run map_manager map_package_test   # 全通过则退出码 0
//
// 覆盖：
//   N1 新地图名规则：中文、字节上限、禁用字符、首尾空白（含全角空格）、不可见字符、非法 UTF-8
//   X1 标准包：文件名、无 .part 残留、zip sha256、条目清单、manifest 字段与底图指纹
//   X2 完整包：带附带文件与两类版本；不带锁文件、.tmp、tombstone、export/
//   X3 导出护栏：不存在、旧扁平布局、定位侧保存中、地图操作锁被占用、空间不足
//   I1 往返：导入到另一个 map_folder，逐文件 sha256 一致、指纹一致、上传文件被删
//   I2 中文名往返：zip 内条目名为 UTF-8，导入后目录名正确
//   I3 重名：RENAME 连续三次 _copy1/2/3；REJECT；OVERWRITE（非当前 / 当前 / 回收站关闭 / 目标不完整）
//   I4 损坏与不安全的包：改一个字节、../ 路径、绝对路径、符号链接、无清单（套了一层）、
//      主版本 2、缺 pbstream、清单外文件、两个顶层目录、数据版本过新
//   I5 文件不可用：不在 .imports、不存在、不是 zip
//   I6 并发：两个线程不经互斥同时导入同一个包（RENAME），renameat2 保证不互相覆盖
//   C1 过期清理；回收站同秒命名
//
// 测试自建临时目录、自清理，不依赖 ~/.dexe/data/maps。
#include <archive.h>
#include <archive_entry.h>
#include <glog/logging.h>
#include <locale.h>
#include <unistd.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "map_manager/map_package.hpp"
#include "map_manager/map_yaml_doc.hpp"

namespace fs = std::filesystem;
namespace mp = map_manager::map_package;

static int g_pass = 0;
static int g_fail = 0;
#define EXPECT(cond, msg)                                                        \
    do {                                                                         \
        if (cond) { ++g_pass; }                                                  \
        else { ++g_fail; std::cout << "  ❌ " << msg << " (line " << __LINE__ << ")\n"; } \
    } while (0)

namespace {

// 1×1 灰度 PNG（只用到签名与 IHDR 宽高）
const unsigned char kPng[] = {0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A, 0x00, 0x00, 0x00, 0x0D, 0x49, 0x48,
                              0x44, 0x52, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x08, 0x00, 0x00, 0x00,
                              0x00, 0x3A, 0x7E, 0x9B, 0x55, 0x00, 0x00, 0x00, 0x0A, 0x49, 0x44, 0x41, 0x54, 0x08,
                              0xD7, 0x63, 0xF8, 0x0F, 0x00, 0x01, 0x01, 0x01, 0x00, 0x1B, 0xB6, 0xEE, 0x56, 0x00,
                              0x00, 0x00, 0x00, 0x49, 0x45, 0x4E, 0x44, 0xAE, 0x42, 0x60, 0x82};

void WriteText(const fs::path& p, const std::string& s) {
    fs::create_directories(p.parent_path());
    std::ofstream(p, std::ios::binary) << s;
}

std::string ReadAll(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

std::string PngBytes() { return std::string(reinterpret_cast<const char*>(kPng), sizeof(kPng)); }

// 造一张新布局地图；extras = 完整包才带的附带文件与版本
void MakeMap(const fs::path& folder, const std::string& name, bool with_assets = true, bool extras = false,
             const std::string& data_version = "0.3.0") {
    const fs::path d = folder / name;
    WriteText(d / "slam" / "map.png", PngBytes());
    WriteText(d / "slam" / "map.yaml", "image: map.png\nresolution: 0.05\norigin: [-1.0, -2.0, 0.0]\nnegate: 0\n"
                                       "occupied_thresh: 0.65\nfree_thresh: 0.196\ncreated_at: 2026-10-01T10:00:00\n"
                                       "last_edited_at: 2026-10-02T11:00:00\ndata_version: " + data_version + "\n");
    WriteText(d / "slam" / "map.pbstream", std::string(4096, '\x5a') + name);
    WriteText(d / "slam" / ".complete", "version=1\n");
    WriteText(d / "slam" / ".map_op.lock", "");
    WriteText(d / "slam" / ".map.yaml.lock", "");
    if (with_assets) {
        WriteText(d / "assets" / "map_assets.json",
                  "{\n  \"schema_version\": \"2.0\",\n  \"last_edited_at\": \"2026-10-03T12:00:00\",\n  \"points\": []\n}\n");
        WriteText(d / "assets" / ".map_assets.lock", "");
    }
    if (extras) {
        WriteText(d / "slam" / "map_origin.png", PngBytes());
        WriteText(d / "slam" / "map.pcd", "pcd");
        WriteText(d / "slam" / "backups" / "backup.20261001-100000" / "map.png", PngBytes());
        WriteText(d / "slam" / "backups" / "backup.20261001-100000" / "map.yaml", "image: map.png\n");
        WriteText(d / "slam" / "backups" / "backup.20261001-100000" / "map.pbstream", "old");
        WriteText(d / "slam" / "backups" / "backup.20261001-100000" / ".source", "grid_edit\n");
        WriteText(d / "slam" / "backups" / ".backup.20261002-100000.tmp" / "map.png", "partial");
        WriteText(d / "slam" / "backups" / "tombstone.1" / "map.png", "dying");
        WriteText(d / "assets" / "backups" / "map_assets.20261001-100000.json", "{}");
        WriteText(d / "assets" / "backups" / "map_assets.20261001-100000.source", "set_assets\n");
        WriteText(d / "assets" / "backups" / "map_assets.20261002-100000.json.tmp", "partial");
        WriteText(d / "export" / "cloud.pcd", "derived");
    }
}

// 测试进程保持 C locale（与 systemd 下的节点一致），只在测试自己读写 zip 时切到 UTF-8
struct TestUtf8Locale {
    locale_t loc = ::newlocale(LC_CTYPE_MASK, "C.UTF-8", static_cast<locale_t>(0));
    locale_t prev = loc ? ::uselocale(loc) : static_cast<locale_t>(0);
    ~TestUtf8Locale() {
        if (loc) {
            ::uselocale(prev);
            ::freelocale(loc);
        }
    }
};

struct ZipItem {
    std::string path;
    std::string data;
    bool symlink = false;
};

// 用 libarchive 造任意内容的 zip（构造损坏 / 恶意包）
void WriteZip(const fs::path& zip, const std::vector<ZipItem>& items) {
    TestUtf8Locale utf8;
    struct archive* a = archive_write_new();
    archive_write_set_format_zip(a);
    archive_write_set_options(a, "zip:hdrcharset=UTF-8");
    archive_write_open_filename(a, zip.c_str());
    for (const auto& it : items) {
        struct archive_entry* e = archive_entry_new();
        archive_entry_set_pathname_utf8(e, it.path.c_str());
        if (it.symlink) {
            archive_entry_set_filetype(e, AE_IFLNK);
            archive_entry_set_symlink(e, it.data.c_str());
            archive_entry_set_size(e, 0);
        } else {
            archive_entry_set_filetype(e, AE_IFREG);
            archive_entry_set_size(e, static_cast<la_int64_t>(it.data.size()));
        }
        archive_entry_set_perm(e, 0644);
        archive_write_header(a, e);
        if (!it.symlink && !it.data.empty()) archive_write_data(a, it.data.data(), it.data.size());
        archive_entry_free(e);
    }
    archive_write_close(a);
    archive_write_free(a);
}

// 读出 zip 全部条目（路径 → 内容）
std::vector<ZipItem> ReadZip(const fs::path& zip) {
    TestUtf8Locale utf8;
    std::vector<ZipItem> out;
    struct archive* a = archive_read_new();
    archive_read_support_format_zip_seekable(a);
    if (archive_read_open_filename(a, zip.c_str(), 65536) != ARCHIVE_OK) {
        archive_read_free(a);
        return out;
    }
    struct archive_entry* e = nullptr;
    while (archive_read_next_header(a, &e) == ARCHIVE_OK) {
        ZipItem it;
        const char* p = archive_entry_pathname_utf8(e);
        it.path = p ? p : "";
        char buf[8192];
        la_ssize_t n = 0;
        while ((n = archive_read_data(a, buf, sizeof(buf))) > 0) it.data.append(buf, n);
        out.push_back(it);
    }
    archive_read_free(a);
    return out;
}

std::set<std::string> Paths(const std::vector<ZipItem>& items) {
    std::set<std::string> s;
    for (const auto& i : items) s.insert(i.path);
    return s;
}

mp::Options Opts(const fs::path& folder) {
    mp::Options o;
    o.map_folder = folder.string();
    o.min_free_bytes = 0;
    return o;
}

// 把包放进 <folder>/.imports/ 并导入
mp::Result ImportInto(const fs::path& folder, const fs::path& zip, mp::OnConflict c = mp::OnConflict::kRename,
                      const std::string& new_name = "", std::function<bool(const std::string&)> is_current = nullptr,
                      mp::Options* custom = nullptr) {
    const fs::path dst = folder / mp::kImportsDirname / zip.filename();
    fs::create_directories(dst.parent_path());
    fs::copy_file(zip, dst, fs::copy_options::overwrite_existing);
    mp::ImportRequest req;
    req.file_path = dst.string();
    req.new_name = new_name;
    req.on_conflict = c;
    req.is_current_map = std::move(is_current);
    return mp::ImportMap(custom ? *custom : Opts(folder), req);
}

bool NoStagingLeft(const fs::path& folder) {
    std::error_code ec;
    for (const char* sub : {mp::kImportsDirname, mp::kExportsDirname}) {
        for (fs::directory_iterator it(folder / sub, ec), end; !ec && it != end; it.increment(ec)) {
            if (it->path().filename().string().rfind(".staging_", 0) == 0) return false;
        }
    }
    return true;
}

// 用 zip 内容改造一个包：修改 / 删除 / 添加条目
fs::path Rezip(const fs::path& src, const fs::path& dst, const std::function<void(std::vector<ZipItem>*)>& edit) {
    auto items = ReadZip(src);
    edit(&items);
    WriteZip(dst, items);
    return dst;
}

}  // namespace

int main(int, char** argv) {
    FLAGS_logtostderr = true;
    FLAGS_minloglevel = 2;  // 只看 ERROR，测试输出保持干净
    google::InitGoogleLogging(argv[0]);

    const fs::path root = fs::temp_directory_path() / ("map_package_test_" + std::to_string(::getpid()));
    fs::remove_all(root);
    const fs::path A = root / "robot_a";
    const fs::path B = root / "robot_b";
    fs::create_directories(A);
    fs::create_directories(B);

    // ── N1 新地图名规则 ─────────────────────────────────────────────
    std::cout << "N1 new map name rule\n";
    {
        std::string err;
        EXPECT(mp::ValidateNewMapName("一楼大厅", &err), "Chinese name accepted");
        EXPECT(mp::ValidateNewMapName("Floor 1 东区", &err), "interior ASCII space accepted");
        std::string han66;
        for (int i = 0; i < 66; ++i) han66 += "图";
        EXPECT(han66.size() == 198 && mp::ValidateNewMapName(han66, &err), "66 Chinese chars (198 bytes) accepted");
        EXPECT(!mp::ValidateNewMapName(han66 + "图", &err), "67 Chinese chars (201 bytes) rejected");
        for (const std::string bad : {"", ".", "..", ".hidden", "-opt", "a/b", "a\\b", "a:b", "a*b", "a?b", "a\"b",
                                      "a<b", "a>b", "a|b", "a'b", "a$b", "a`b", " lead", "trail ", "全角\xE3\x80\x80",
                                      "\xE3\x80\x80全角", "零\xE2\x80\x8B宽", "bad\xFF", "line\nbreak", "tab\tx",
                                      "\xC0\xAF"}) {
            EXPECT(!mp::ValidateNewMapName(bad, &err), "rejected: [" + bad + "]");
        }
    }

    // ── X1 标准包 ───────────────────────────────────────────────────
    std::cout << "X1 standard export\n";
    MakeMap(A, "office", true, true);
    mp::Result ex = mp::ExportMap(Opts(A), "office", mp::Content::kStandard);
    EXPECT(ex.ok(), "export office: " + ex.message);
    const fs::path zip_office = ex.file_path;
    {
        const std::string fname = zip_office.filename().string();
        EXPECT(fname.rfind("office_", 0) == 0 && fname.size() == std::string("office_20261016_120000.zip").size(),
               "package file name <map>_<YYYYMMDD_HHMMSS>.zip: " + fname);
        EXPECT(zip_office.parent_path() == A / mp::kExportsDirname, "written to .exports/");
        EXPECT(!fs::exists(zip_office.string() + ".part"), "no .part left");
        EXPECT(ex.sha256 == mp::FileSha256(zip_office.string()) && ex.sha256.size() == 64, "zip sha256 in result");
        EXPECT(ex.size_bytes == fs::file_size(zip_office), "size_bytes");
        const auto items = ReadZip(zip_office);
        const std::set<std::string> expected = {"manifest.json", "office/slam/map.pbstream", "office/slam/map.png",
                                                "office/slam/map.yaml", "office/slam/.complete",
                                                "office/assets/map_assets.json"};
        EXPECT(Paths(items) == expected, "standard package entries");
        nlohmann::json m;
        for (const auto& i : items) if (i.path == "manifest.json") m = nlohmann::json::parse(i.data);
        EXPECT(m.value("format", "") == "dexe_map_package" && m.value("format_version", "") == "1.0", "format");
        EXPECT(m["map"].value("name", "") == "office" && m.value("content", "") == "standard", "map.name / content");
        const auto yaml = map_manager::MapYamlDoc::Load((A / "office" / "slam" / "map.yaml").string());
        EXPECT(m["map"].value("grid_revision", "") == map_manager::GridRevision((A / "office" / "slam").string(), yaml),
               "manifest grid_revision equals get_maps fingerprint");
        EXPECT(m["map"]["size_px"] == nlohmann::json::array({1, 1}), "size_px from PNG header");
        EXPECT(m["map"].value("data_version", "") == "0.3.0" && m["map"].value("has_assets", false), "data_version / has_assets");
        EXPECT(m["map"].value("assets_schema_version", "") == "2.0", "assets_schema_version");
        EXPECT(m["files"].size() == 5, "manifest lists 5 files");
    }

    // ── X2 完整包 ───────────────────────────────────────────────────
    std::cout << "X2 full export\n";
    {
        mp::Result full = mp::ExportMap(Opts(A), "office", mp::Content::kFull);
        EXPECT(full.ok(), "export full: " + full.message);
        EXPECT(full.file_path != zip_office.string(), "same second export gets a distinct file name");
        const auto p = Paths(ReadZip(full.file_path));
        EXPECT(p.count("office/slam/map_origin.png") && p.count("office/slam/map.pcd"), "extras included");
        EXPECT(p.count("office/slam/backups/backup.20261001-100000/map.pbstream") &&
                   p.count("office/slam/backups/backup.20261001-100000/.source"), "grid version included");
        EXPECT(p.count("office/assets/backups/map_assets.20261001-100000.json") &&
                   p.count("office/assets/backups/map_assets.20261001-100000.source"), "asset version included");
        bool leaked = false;
        for (const auto& x : p) {
            if (x.find(".lock") != std::string::npos || x.find(".tmp") != std::string::npos ||
                x.find("tombstone") != std::string::npos || x.find("/export/") != std::string::npos) leaked = true;
        }
        EXPECT(!leaked, "no lock / tmp / tombstone / export files in package");
    }

    // ── X3 导出护栏 ─────────────────────────────────────────────────
    std::cout << "X3 export guards\n";
    {
        EXPECT(mp::ExportMap(Opts(A), "nope", mp::Content::kStandard).error_code == mp::kErrNotFound, "missing map");
        WriteText(A / "flat" / "map.yaml", "image: map.png\n");
        EXPECT(mp::ExportMap(Opts(A), "flat", mp::Content::kStandard).error_code == mp::kErrLoadFailed, "legacy layout");
        MakeMap(A, "saving");
        fs::create_directories(A / "saving" / "slam.staging");
        EXPECT(mp::ExportMap(Opts(A), "saving", mp::Content::kStandard).error_code == mp::kErrIoFailed,
               "localization save in progress -> busy");
        {
            map_manager::MapOpLock hold((A / "office" / "slam").string());
            EXPECT(hold.acquired(), "test holds map op lock");
            const auto t0 = std::chrono::steady_clock::now();
            EXPECT(mp::ExportMap(Opts(A), "office", mp::Content::kStandard).error_code == mp::kErrIoFailed,
                   "map op lock held -> busy");
            EXPECT(std::chrono::steady_clock::now() - t0 < std::chrono::seconds(5), "busy returns within 5 s");
        }
        mp::Options tight = Opts(A);
        tight.min_free_bytes = UINT64_MAX / 4;
        const size_t before = std::distance(fs::directory_iterator(A / mp::kExportsDirname), fs::directory_iterator());
        EXPECT(mp::ExportMap(tight, "office", mp::Content::kStandard).error_code == mp::kErrNoSpace, "no space");
        const size_t after = std::distance(fs::directory_iterator(A / mp::kExportsDirname), fs::directory_iterator());
        EXPECT(before == after, "no file written when out of space");
        EXPECT(NoStagingLeft(A), "no export staging left");
    }

    // ── I1 往返 ─────────────────────────────────────────────────────
    std::cout << "I1 round trip\n";
    {
        mp::Result im = ImportInto(B, zip_office);
        EXPECT(im.ok() && im.map_name == "office", "import office: " + im.message);
        for (const char* rel : {"slam/map.pbstream", "slam/map.png", "slam/map.yaml", "slam/.complete",
                                "assets/map_assets.json"}) {
            EXPECT(mp::FileSha256((A / "office" / rel).string()) == mp::FileSha256((B / "office" / rel).string()),
                   std::string("byte-identical: ") + rel);
        }
        const auto ya = map_manager::MapYamlDoc::Load((A / "office" / "slam" / "map.yaml").string());
        const auto yb = map_manager::MapYamlDoc::Load((B / "office" / "slam" / "map.yaml").string());
        EXPECT(map_manager::GridRevision((A / "office" / "slam").string(), ya) ==
                   map_manager::GridRevision((B / "office" / "slam").string(), yb), "grid_revision unchanged");
        EXPECT(!fs::exists(B / mp::kImportsDirname / zip_office.filename()), "uploaded file removed on success");
        EXPECT(!fs::exists(B / "office" / "slam" / "backups"), "standard package carries no versions");
        EXPECT(mp::IsLoadableMapDirectory(B.string(), "office"), "imported map is listed");
        EXPECT(NoStagingLeft(B), "no import staging left");
    }

    // ── I2 中文名往返 ───────────────────────────────────────────────
    std::cout << "I2 Chinese name\n";
    {
        MakeMap(A, "一楼大厅", false);
        mp::Result e = mp::ExportMap(Opts(A), "一楼大厅", mp::Content::kStandard);
        EXPECT(e.ok(), "export Chinese map: " + e.message);
        EXPECT(fs::path(e.file_path).filename().string().rfind("一楼大厅_", 0) == 0, "package file name keeps Chinese");
        const auto p = Paths(ReadZip(e.file_path));
        EXPECT(p.count("一楼大厅/slam/map.png") == 1, "zip entry names are UTF-8");
        EXPECT(p.count("一楼大厅/assets/map_assets.json") == 0, "map without assets exports without assets/");
        mp::Result i = ImportInto(B, e.file_path);
        EXPECT(i.ok() && i.map_name == "一楼大厅", "import Chinese map: " + i.message);
        EXPECT(fs::is_regular_file(B / "一楼大厅" / "slam" / "map.pbstream"), "Chinese map dir created");
        mp::Result renamed = ImportInto(B, e.file_path, mp::OnConflict::kReject, "二楼走廊");
        EXPECT(renamed.ok() && renamed.map_name == "二楼走廊", "import under a new Chinese name");
    }

    // ── I3 重名 ─────────────────────────────────────────────────────
    std::cout << "I3 name conflicts\n";
    {
        for (int k = 1; k <= 3; ++k) {
            mp::Result r = ImportInto(B, zip_office);
            EXPECT(r.ok() && r.map_name == "office_copy" + std::to_string(k), "RENAME -> office_copy" + std::to_string(k));
        }
        mp::Result rej = ImportInto(B, zip_office, mp::OnConflict::kReject);
        EXPECT(rej.error_code == mp::kErrNameExists, "REJECT -> 2202050");
        EXPECT(fs::exists(B / mp::kImportsDirname / zip_office.filename()), "uploaded file kept on failure");

        WriteText(B / "office" / "slam" / "marker.txt", "old map");
        mp::Result cur = ImportInto(B, zip_office, mp::OnConflict::kOverwrite, "",
                                    [](const std::string& n) { return n == "office"; });
        EXPECT(cur.error_code == mp::kErrInUse && fs::exists(B / "office" / "slam" / "marker.txt"),
               "OVERWRITE current map -> 2202040, untouched");
        mp::Options no_trash = Opts(B);
        no_trash.trash_enable = false;
        mp::Result dis = ImportInto(B, zip_office, mp::OnConflict::kOverwrite, "", nullptr, &no_trash);
        EXPECT(dis.error_code == mp::kErrTrashDisabled, "OVERWRITE with trash disabled -> 2202090");
        mp::Result ow = ImportInto(B, zip_office, mp::OnConflict::kOverwrite);
        EXPECT(ow.ok() && ow.map_name == "office" && !ow.trashed_entry.empty(), "OVERWRITE -> trashed: " + ow.message);
        EXPECT(fs::exists(B / ".map_trash" / ow.trashed_entry / "slam" / "marker.txt"), "old map is in trash");
        EXPECT(!fs::exists(B / "office" / "slam" / "marker.txt"), "new map in place");
        fs::create_directories(B / "half" / "slam");
        mp::Result half = ImportInto(B, zip_office, mp::OnConflict::kOverwrite, "half");
        EXPECT(half.error_code == mp::kErrNameExists && fs::is_directory(B / "half" / "slam"),
               "OVERWRITE onto an incomplete dir (mapping?) -> 2202050, untouched");
        mp::Result badname = ImportInto(B, zip_office, mp::OnConflict::kRename, "a:b");
        EXPECT(badname.error_code == mp::kErrNameInvalid, "invalid new name -> 2202020");
    }

    // ── I4 损坏与不安全的包 ─────────────────────────────────────────
    std::cout << "I4 corrupted / unsafe packages\n";
    {
        const fs::path work = root / "work";
        fs::create_directories(work);
        auto expect_incompatible = [&](const fs::path& zip, const std::string& what, const std::string& hint = "") {
            const auto before = std::distance(fs::directory_iterator(B), fs::directory_iterator());
            mp::Result r = ImportInto(B, zip, mp::OnConflict::kRename, "probe");
            const auto after = std::distance(fs::directory_iterator(B), fs::directory_iterator());
            EXPECT(r.error_code == mp::kErrImportIncompatible, what + " -> 2202150 (got " +
                                                                   std::to_string(r.error_code) + ": " + r.message + ")");
            EXPECT(before == after && !fs::exists(B / "probe"), what + ": no map created");
            EXPECT(NoStagingLeft(B), what + ": no staging left");
            if (!hint.empty()) EXPECT(r.message.find(hint) != std::string::npos, what + ": message hints '" + hint + "'");
        };
        expect_incompatible(Rezip(zip_office, work / "flip.zip", [](std::vector<ZipItem>* v) {
            for (auto& i : *v) if (i.path == "office/slam/map.pbstream") i.data[10] ^= 0x01;
        }), "one byte modified", "sha256");
        expect_incompatible(Rezip(zip_office, work / "slip.zip", [](std::vector<ZipItem>* v) {
            v->push_back({"office/../../evil.txt", "x"});
        }), "../ entry");
        EXPECT(!fs::exists(root / "evil.txt") && !fs::exists(B / "evil.txt"), "nothing written outside map_folder");
        expect_incompatible(Rezip(zip_office, work / "abs.zip", [](std::vector<ZipItem>* v) {
            v->push_back({"/tmp/map_package_test_abs.txt", "x"});
        }), "absolute entry");
        EXPECT(!fs::exists("/tmp/map_package_test_abs.txt"), "absolute path not written");
        expect_incompatible(Rezip(zip_office, work / "link.zip", [](std::vector<ZipItem>* v) {
            v->push_back({"office/slam/link", "/etc/passwd", true});
        }), "symlink entry");
        WriteZip(work / "nested.zip", {{"office_a.zip", ReadAll(zip_office)}, {"office_b.zip", ReadAll(zip_office)}});
        expect_incompatible(work / "nested.zip", "bundle of packages (no manifest)", "unzip");
        expect_incompatible(Rezip(zip_office, work / "v2.zip", [](std::vector<ZipItem>* v) {
            for (auto& i : *v) {
                if (i.path != "manifest.json") continue;
                auto m = nlohmann::json::parse(i.data);
                m["format_version"] = "2.0";
                i.data = m.dump();
            }
        }), "format_version 2.0");
        expect_incompatible(Rezip(zip_office, work / "nopb.zip", [](std::vector<ZipItem>* v) {
            v->erase(std::remove_if(v->begin(), v->end(), [](const ZipItem& i) { return i.path == "office/slam/map.pbstream"; }),
                     v->end());
            for (auto& i : *v) {
                if (i.path != "manifest.json") continue;
                auto m = nlohmann::json::parse(i.data);
                nlohmann::json files = nlohmann::json::array();
                for (const auto& f : m["files"]) if (f["path"] != "office/slam/map.pbstream") files.push_back(f);
                m["files"] = files;
                i.data = m.dump();
            }
        }), "missing map.pbstream");
        expect_incompatible(Rezip(zip_office, work / "extra.zip", [](std::vector<ZipItem>* v) {
            v->push_back({"office/slam/unlisted.bin", "x"});
        }), "file not in manifest");
        expect_incompatible(Rezip(zip_office, work / "two.zip", [](std::vector<ZipItem>* v) {
            v->push_back({"other/slam/map.png", "x"});
        }), "two top-level dirs");
        MakeMap(A, "future", true, false, "0.4.0");
        const fs::path future = mp::ExportMap(Opts(A), "future", mp::Content::kStandard).file_path;
        expect_incompatible(future, "data_version 0.4.0");
        MakeMap(A, "old", true, false, "legacy");
        mp::Result legacy = ImportInto(B, mp::ExportMap(Opts(A), "old", mp::Content::kStandard).file_path);
        EXPECT(legacy.ok(), "data_version legacy accepted: " + legacy.message);
    }

    // ── I5 文件不可用 ───────────────────────────────────────────────
    std::cout << "I5 unusable files\n";
    {
        mp::ImportRequest req;
        req.file_path = zip_office.string();  // 在 A/.exports，不在 B/.imports
        EXPECT(mp::ImportMap(Opts(B), req).error_code == mp::kErrImportFileInvalid, "outside .imports -> 2202140");
        req.file_path = (B / mp::kImportsDirname / "missing.zip").string();
        EXPECT(mp::ImportMap(Opts(B), req).error_code == mp::kErrImportFileInvalid, "missing -> 2202140");
        WriteText(B / mp::kImportsDirname / "text.zip", "this is not a zip");
        req.file_path = (B / mp::kImportsDirname / "text.zip").string();
        EXPECT(mp::ImportMap(Opts(B), req).error_code == mp::kErrImportFileInvalid, "not a zip -> 2202140");
        mp::ImportRequest any;
        any.file_path = zip_office.string();
        any.new_name = "from_cli";
        any.require_imports_dir = false;  // 命令行工具：任意路径
        EXPECT(mp::ImportMap(Opts(B), any).ok() && fs::exists(zip_office), "CLI mode: any path, source file kept");
    }

    // ── I6 并发导入 ─────────────────────────────────────────────────
    std::cout << "I6 concurrent imports\n";
    {
        const fs::path C = root / "robot_c";
        fs::create_directories(C / mp::kImportsDirname);
        fs::copy_file(zip_office, C / mp::kImportsDirname / "p1.zip");
        fs::copy_file(zip_office, C / mp::kImportsDirname / "p2.zip");
        mp::Result r1, r2;
        auto run = [&C](const std::string& f, mp::Result* out) {
            mp::ImportRequest q;
            q.file_path = (C / mp::kImportsDirname / f).string();
            *out = mp::ImportMap(Opts(C), q);  // 不传 commit_mutex，模拟节点与命令行工具同时导入
        };
        std::thread t1(run, "p1.zip", &r1), t2(run, "p2.zip", &r2);
        t1.join();
        t2.join();
        EXPECT(r1.ok() && r2.ok() && r1.map_name != r2.map_name, "both imported under distinct names: " +
                                                                     r1.map_name + " / " + r2.map_name);
    }

    // ── C1 过期清理、回收站命名 ─────────────────────────────────────
    std::cout << "C1 cleanup / trash naming\n";
    {
        const fs::path old_file = A / mp::kExportsDirname / "old.zip";
        WriteText(old_file, "old");
        fs::last_write_time(old_file, fs::file_time_type::clock::now() - std::chrono::hours(2));
        const fs::path old_staging = A / mp::kExportsDirname / ".staging_old";
        fs::create_directories(old_staging);
        fs::last_write_time(old_staging, fs::file_time_type::clock::now() - std::chrono::hours(2));
        const int n = mp::CleanupExpired(Opts(A));
        EXPECT(n >= 2 && !fs::exists(old_file) && !fs::exists(old_staging), "expired file and staging removed");
        EXPECT(fs::exists(zip_office), "fresh package kept");

        MakeMap(A, "t1");
        std::string e1, e2, err;
        EXPECT(mp::MoveMapToTrash(A.string(), ".map_trash", "t1", &e1, &err), "trash t1: " + err);
        MakeMap(A, "t1");
        EXPECT(mp::MoveMapToTrash(A.string(), ".map_trash", "t1", &e2, &err), "trash t1 again: " + err);
        EXPECT(e1 != e2 && (e2 == e1 + "_2" || e2.rfind("t1_", 0) == 0), "same-second entry gets _2: " + e1 + " / " + e2);
    }

    // ── ListMaps ────────────────────────────────────────────────────
    {
        const auto maps = mp::ListMaps(B.string());
        bool has_cn = false, has_dot = false;
        for (const auto& m : maps) {
            if (m.name == "一楼大厅") has_cn = m.created_at == "2026-10-01T10:00:00";
            if (!m.name.empty() && m.name[0] == '.') has_dot = true;
        }
        EXPECT(has_cn && !has_dot, "ListMaps: Chinese map with created_at, no hidden dirs");
    }

    fs::remove_all(root);
    std::cout << "\nmap_package_test: " << g_pass << " passed, " << g_fail << " failed\n";
    return g_fail == 0 ? 0 : 1;
}
