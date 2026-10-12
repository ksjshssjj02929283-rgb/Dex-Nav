// map_manager 地图编辑核心层测试（不起 ROS 节点）。
//
// 运行：ros2 run map_manager map_manager_edit_test   # 全通过则退出码 0
//
// 覆盖：
//   E1 MapYamlDoc：注释 / 未知键 / 行尾注释原样保留，原位改、追加、删除、重复键去重、非数值读取
//   E2 SetDisplayYawEx：未加载的图也能设；只动 display_yaw 与 last_edited_at；归一化到 (-π, π]；无键时追加；
//      值不变不写盘
//   E3 SetDisplayYawEx 参数与存在性护栏：空名 / 不存在 / NaN / Inf
//   E4 落盘副作用：无 .tmp 残留，锁文件落在 slam/
//   E5 ScanMapCatalog：display_yaw 读回；grid_revision 稳定、改底图即变、改显示朝向不变；
//      last_edited_at 只认 map.yaml 的 last_edited_at 键（无键退回 png / yaml mtime），资产文件不参与
//   G1 三值判定与「未知」灰度（free_thresh=0.2 的图上不能写 205）
//   G2 栅格化固定用例：单点、胶囊、擦除只动障碍、墙体不重写障碍、笔画顺序、越界裁剪、矩形未知
//      （前后端一致性基准：前端用同一组笔画逐格比对）
//   G3 笔画参数校验
//   G4 EditGridEx：指纹不符拒绝且不动文件、三件套备份、像素 / 面积 / 其余 yaml 行、新指纹、无变化不备份、缺 pbstream
//   G5 版本轮换与同秒命名
//   G6 编辑当前图 → requires_reload；同名重新加载从磁盘重读（失败保留原栅格）
//   V1 （回滚推进 last_edited_at）版本列表随 ScanMapCatalog 返回（顺序 / 来源 / 三件不齐）、回滚、回滚可撤销、空版本号 = 最新、各类拒绝
//   V2 回滚当前图 → requires_reload；轮换时目标版本受保护
//   V3 附带文件（点云 / 线特征）随版本保存、随回滚恢复
//
// 测试自建临时目录、自清理，不依赖 ~/.dexe/data/maps。
#include <glog/logging.h>
#include <unistd.h>

#include <boost/filesystem.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "map_manager/map_manager.hpp"
#include "map_manager/map_yaml_doc.hpp"

#include <opencv2/imgcodecs.hpp>

namespace fs = boost::filesystem;
using map_manager::MapManager;
using map_manager::MapYamlDoc;

static int g_pass = 0;
static int g_fail = 0;
#define EXPECT(cond, msg)                                                        \
    do {                                                                         \
        if (cond) { ++g_pass; }                                                  \
        else { ++g_fail; std::cout << "  ❌ " << msg << " (line " << __LINE__ << ")\n"; } \
    } while (0)

static void WriteText(const fs::path& p, const std::string& s) {
    fs::create_directories(p.parent_path());
    std::ofstream(p.string()) << s;
}

static std::string ReadText(const fs::path& p) {
    std::ifstream in(p.string());
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

static std::vector<std::string> Lines(const std::string& s) {
    std::vector<std::string> out;
    std::stringstream ss(s);
    for (std::string l; std::getline(ss, l);) out.push_back(l);
    return out;
}

// "<prefix><本地时间 ISO 8601>" 且与现在相差不超过 5 秒。
static bool StampIsNow(const std::string& line, const std::string& prefix) {
    if (line.rfind(prefix, 0) != 0) return false;
    const std::time_t t = map_manager::ParseLocalIso8601(line.substr(prefix.size()));
    return t > 0 && std::abs(static_cast<double>(std::time(nullptr) - t)) <= 5.0;
}

// 原子替换写 png：与实际编辑写盘一样走 rename，内容不同但大小相同。
static void ReplacePng(const fs::path& png, const std::string& bytes) {
    WriteText(png.string() + ".tmp", bytes);
    fs::rename(png.string() + ".tmp", png);
}

static const char* kYaml =
    "# written by mapping node\n"
    "image: map.png\n"
    "resolution: 0.05\n"
    "origin: [-1.5, 2.25, 0.0]\n"
    "negate: 0\n"
    "total_area_m2: 12.5  # trailing comment\n"
    "effective_area_m2: 3.5\n"
    "\n"
    "created_at: 2026-01-02T03:04:05\n"
    "vendor_note: keep me\n"
    "data_version: 0.3.0\n";

void TestE1_Doc(const fs::path& root) {
    std::cout << "▶ E1 MapYamlDoc\n";
    const fs::path p = root / "e1.yaml";
    WriteText(p, std::string(kYaml) + "resolution: 9\nbad: abc\n");
    MapYamlDoc d = MapYamlDoc::Load(p.string());
    EXPECT(d.Get("resolution").value_or("") == "0.05", "duplicate key keeps first");
    EXPECT(d.Get("total_area_m2").value_or("") == "12.5  # trailing comment", "raw value incl. comment");
    EXPECT(!d.GetDouble("bad").has_value(), "non-number GetDouble = nullopt");
    EXPECT(!d.Get("missing").has_value(), "missing key");
    d.Set("resolution", "0.1");
    d.Set("new_key", "1");
    d.Erase("vendor_note");
    const auto ls = Lines(d.ToString());
    EXPECT(ls[0] == "# written by mapping node", "comment kept");
    EXPECT(ls[2] == "resolution: 0.1", "in-place set: " + ls[2]);
    EXPECT(ls[7].empty(), "blank line kept");
    EXPECT(ls.back() == "new_key: 1", "append at end: " + ls.back());
    EXPECT(d.ToString().find("vendor_note") == std::string::npos, "erased");
    EXPECT(d.ToString().find("resolution: 9") == std::string::npos, "duplicate dropped");
}

void TestE2_SetDisplayYaw(const fs::path& maps) {
    std::cout << "▶ E2 SetDisplayYawEx\n";
    WriteText(maps / "m1" / "slam" / "map.yaml", kYaml);
    WriteText(maps / "m1" / "slam" / "map.png", "PNG-A");
    MapManager mm(maps.string());
    std::string msg;
    EXPECT(mm.SetDisplayYawEx("m1", 4.0, msg) == 0, "set on non-loaded map: " + msg);
    const auto after = Lines(ReadText(maps / "m1" / "slam" / "map.yaml"));
    const auto before = Lines(kYaml);
    EXPECT(after.size() == before.size() + 2, "display_yaw + last_edited_at added");
    for (size_t i = 0; i < before.size() && i < after.size(); ++i) {
        EXPECT(after[i] == before[i], "line " + std::to_string(i) + " untouched");
    }
    const std::string yaw_line = after.size() > before.size() ? after[before.size()] : "";
    const double yaw = std::stod(yaw_line.substr(std::string("display_yaw: ").size()));
    EXPECT(std::abs(yaw - (4.0 - 2.0 * M_PI)) < 1e-9, "normalized: " + yaw_line);
    EXPECT(StampIsNow(after.back(), "last_edited_at: "), "last_edited_at stamped: " + after.back());

    EXPECT(mm.SetDisplayYawEx("m1", M_PI, msg) == 0, "set π: " + msg);
    const auto again = Lines(ReadText(maps / "m1" / "slam" / "map.yaml"));
    EXPECT(again.size() == after.size(), "second set overrides in place");
    const std::string again_yaw = again.size() > before.size() ? again[before.size()] : "";
    EXPECT(std::abs(std::stod(again_yaw.substr(13)) - M_PI) < 1e-9, "π stays π: " + again_yaw);

    // 值不变：不算编辑，文件逐字节不动
    const std::string snapshot = ReadText(maps / "m1" / "slam" / "map.yaml");
    EXPECT(mm.SetDisplayYawEx("m1", M_PI, msg) == 0, "same yaw: " + msg);
    EXPECT(ReadText(maps / "m1" / "slam" / "map.yaml") == snapshot, "same yaw must not rewrite map.yaml");
}

void TestE3_Guards(const fs::path& maps) {
    std::cout << "▶ E3 guards\n";
    MapManager mm(maps.string());
    std::string msg;
    EXPECT(mm.SetDisplayYawEx("", 0.1, msg) == 2202020, "empty name");
    EXPECT(mm.SetDisplayYawEx("nope", 0.1, msg) == 2202030, "not found");
    EXPECT(mm.SetDisplayYawEx("m1", std::nan(""), msg) == 2202100, "NaN");
    EXPECT(mm.SetDisplayYawEx("m1", INFINITY, msg) == 2202100, "Inf");
    WriteText(maps / "half" / "slam" / "map.pbstream", "x");  // 建图中断残留：没有 map.yaml
    EXPECT(mm.SetDisplayYawEx("half", 0.1, msg) == 2202030, "no map.yaml = not found");
    EXPECT(!fs::exists(maps / "half" / "slam" / "map.yaml"), "must not create map.yaml");
}

void TestE4_SideEffects(const fs::path& maps) {
    std::cout << "▶ E4 side effects\n";
    EXPECT(!fs::exists(maps / "m1" / "slam" / "map.yaml.tmp"), "no tmp left");
    EXPECT(fs::exists(maps / "m1" / "slam" / ".map.yaml.lock"), "lock file in slam/");
}

void TestE5_Catalog(const fs::path& maps) {
    std::cout << "▶ E5 ScanMapCatalog\n";
    MapManager mm(maps.string());
    auto find = [&](const std::string& name) {
        for (const auto& e : mm.ScanMapCatalog()) if (e.name == name) return e;
        return map_manager::MapCatalogEntry{};
    };
    const auto e1 = find("m1");
    EXPECT(std::abs(e1.display_yaw - M_PI) < 1e-6, "display_yaw read back");
    EXPECT(e1.grid_revision.size() == 16, "revision is 16 hex: " + e1.grid_revision);
    EXPECT(find("m1").grid_revision == e1.grid_revision, "revision stable");

    std::string msg;
    mm.SetDisplayYawEx("m1", 0.3, msg);
    EXPECT(find("m1").grid_revision == e1.grid_revision, "display_yaw does not change revision");

    ReplacePng(maps / "m1" / "slam" / "map.png", "PNG-B");  // 同大小、同一秒内改
    const auto e2 = find("m1");
    EXPECT(e2.grid_revision != e1.grid_revision, "png change -> new revision");

    // last_edited_at 读 yaml 键；改资产文件、改 png 文件时间都不影响它
    std::string yaml_text = ReadText(maps / "m1" / "slam" / "map.yaml");
    {
        map_manager::MapYamlDoc doc = map_manager::MapYamlDoc::Load((maps / "m1" / "slam" / "map.yaml").string());
        doc.Set("last_edited_at", "2026-01-02T03:04:05");
        WriteText(maps / "m1" / "slam" / "map.yaml", doc.ToString());
    }
    sleep(1);
    WriteText(maps / "m1" / "assets" / "map_assets.json", "{}");
    const auto e3 = find("m1");
    EXPECT(e3.last_edited_at == map_manager::ParseLocalIso8601("2026-01-02T03:04:05") && e3.last_edited_at > 0,
           "last_edited_at from yaml key, assets ignored");
    // 旧图无该键：退回 png / yaml mtime 较大值，同样不看资产
    WriteText(maps / "m1" / "slam" / "map.yaml", kYaml);
    sleep(1);
    WriteText(maps / "m1" / "assets" / "map_assets.json", "{\"x\":1}");
    const auto e4 = find("m1");
    const std::time_t yaml_mtime = fs::last_write_time(maps / "m1" / "slam" / "map.yaml");
    EXPECT(e4.last_edited_at == yaml_mtime, "legacy fallback = yaml/png mtime, not assets");
    WriteText(maps / "m1" / "slam" / "map.yaml", yaml_text);
}

// ── 底图笔画编辑 ─────────────────────────────────────────────────────────

using map_manager::GridGeometry;
using map_manager::GridStrokeSpec;
using map_manager::GridThresholds;

static GridStrokeSpec Stroke(uint8_t tool, uint8_t shape, std::vector<std::pair<double, double>> pts,
                             double w) {
    GridStrokeSpec s;
    s.tool = tool;
    s.shape = shape;
    s.points = std::move(pts);
    s.width_m = w;
    return s;
}

// G1 三值判定与「未知」灰度
void TestG1_Classify() {
    std::cout << "▶ G1 classify / unknown gray\n";
    const GridThresholds def{0.65, 0.196};
    const GridThresholds robot{0.6, 0.2};   // 本体现有地图的阈值
    EXPECT(map_manager::UnknownGray(def) == 205, "default thresholds use 205");
    EXPECT(map_manager::ClassifyPixel(205, robot) == map_manager::CellClass::kFree, "205 is free when free_thresh=0.2");
    const int g = map_manager::UnknownGray(robot);
    EXPECT(g == 153, "robot thresholds use mid of [102, 204]: " + std::to_string(g));
    EXPECT(map_manager::ClassifyPixel(static_cast<uint8_t>(g), robot) == map_manager::CellClass::kUnknown,
           "unknown gray classifies as unknown");
    EXPECT(!map_manager::ValidThresholds({0.2, 0.6}), "free >= occupied is invalid");
}

// G2 栅格化固定用例（前后端一致性基准）：10×10，分辨率 0.1 m，原点 (0, 0)
void TestG2_Rasterize() {
    std::cout << "▶ G2 rasterize fixtures\n";
    const GridGeometry geo{0.1, 0.0, 0.0, 10, 10};
    const GridThresholds th{0.65, 0.196};
    using map_manager::kShapePolyline;
    using map_manager::kShapeRect;
    using map_manager::kToolErase;
    using map_manager::kToolUnknown;
    using map_manager::kToolWall;

    // 单点 + 极细笔：只覆盖所在一格；y=0.25 → 行 (10−1)−2 = 7，x=0.25 → 列 2
    cv::Mat m(10, 10, CV_8UC1, cv::Scalar(150));
    EXPECT(map_manager::ApplyStrokes(&m, geo, th, {Stroke(kToolWall, kShapePolyline, {{0.25, 0.25}}, 0.01)}) == 1,
           "dot covers exactly one cell");
    EXPECT(m.at<uint8_t>(7, 2) == 0, "row/col convention (origin at bottom-left)");

    // 水平线段，直径 0.1 → 半径 0.1（含半格余量）：中心距线 ≤ 0.1 的 3 行 × 10 列
    m.setTo(150);
    EXPECT(map_manager::ApplyStrokes(&m, geo, th, {Stroke(kToolWall, kShapePolyline, {{0.05, 0.55}, {0.95, 0.55}}, 0.1)}) == 30,
           "capsule covers 3 rows x 10 cols");

    // 擦除只动障碍：0 → 255；未知 150 与中间灰度的自由 230 都不变
    m.setTo(150);
    m.at<uint8_t>(5, 5) = 0;
    m.at<uint8_t>(5, 6) = 230;
    const int erased = map_manager::ApplyStrokes(&m, geo, th, {Stroke(kToolErase, kShapePolyline, {{0.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}, {1.0, 0.0}}, 3.0)});
    EXPECT(erased == 1 && m.at<uint8_t>(5, 5) == 255, "erase turns only obstacle into free");
    EXPECT(m.at<uint8_t>(0, 0) == 150 && m.at<uint8_t>(5, 6) == 230, "unknown and mid-gray free untouched");

    // 墙体不重写已是障碍的像素（灰度 30 保持 30）
    m.setTo(150);
    m.at<uint8_t>(2, 2) = 30;
    map_manager::ApplyStrokes(&m, geo, th, {Stroke(kToolWall, kShapePolyline, {{0.25, 0.75}}, 0.01)});
    EXPECT(m.at<uint8_t>(2, 2) == 30, "obstacle pixel keeps its gray under wall");

    // 顺序有意义：先墙后擦 = 自由；先擦后墙 = 障碍
    m.setTo(150);
    map_manager::ApplyStrokes(&m, geo, th, {Stroke(kToolWall, kShapePolyline, {{0.45, 0.45}}, 0.01),
                                            Stroke(kToolErase, kShapePolyline, {{0.45, 0.45}}, 0.01)});
    EXPECT(m.at<uint8_t>(5, 4) == 255, "wall then erase = free");
    m.setTo(150);
    map_manager::ApplyStrokes(&m, geo, th, {Stroke(kToolErase, kShapePolyline, {{0.45, 0.45}}, 0.01),
                                            Stroke(kToolWall, kShapePolyline, {{0.45, 0.45}}, 0.01)});
    EXPECT(m.at<uint8_t>(5, 4) == 0, "erase then wall = obstacle");

    // 越界：整笔在外 0 变化；跨边界的只画界内部分
    m.setTo(150);
    EXPECT(map_manager::ApplyStrokes(&m, geo, th, {Stroke(kToolWall, kShapePolyline, {{-5, -5}, {-4, -4}}, 0.1)}) == 0,
           "stroke fully outside changes nothing");
    EXPECT(map_manager::ApplyStrokes(&m, geo, th, {Stroke(kToolWall, kShapePolyline, {{-1.0, 0.05}, {0.15, 0.05}}, 0.01)}) == 2,
           "stroke crossing border is clipped");

    // 矩形未知：中心落在矩形内的格 → 205
    m.setTo(255);
    EXPECT(map_manager::ApplyStrokes(&m, geo, th, {Stroke(kToolUnknown, kShapeRect, {{0.0, 0.0}, {0.3, 0.2}}, 0.0)}) == 6,
           "rect covers 3 cols x 2 rows");
    EXPECT(m.at<uint8_t>(9, 0) == 205, "unknown gray written");
}

// G3 参数校验
void TestG3_Validate() {
    std::cout << "▶ G3 validate strokes\n";
    using map_manager::ValidateStrokes;
    EXPECT(!ValidateStrokes({}).empty(), "empty strokes");
    EXPECT(!ValidateStrokes({Stroke(9, 1, {{0, 0}}, 0.1)}).empty(), "bad tool");
    EXPECT(!ValidateStrokes({Stroke(1, 9, {{0, 0}}, 0.1)}).empty(), "bad shape");
    EXPECT(!ValidateStrokes({Stroke(1, 1, {}, 0.1)}).empty(), "no points");
    EXPECT(!ValidateStrokes({Stroke(1, 1, {{std::nan(""), 0}}, 0.1)}).empty(), "NaN point");
    EXPECT(!ValidateStrokes({Stroke(1, 1, {{0, 0}}, 0.0)}).empty(), "zero width");
    EXPECT(!ValidateStrokes({Stroke(3, 2, {{0, 0}}, 0.0)}).empty(), "rect with 1 point");
    EXPECT(ValidateStrokes({Stroke(3, 2, {{0, 0}, {1, 1}}, 0.0)}).empty(), "rect ignores width");
}

static const char* kEditYaml =
    "image: map.png\n"
    "resolution: 0.1\n"
    "origin: [-1, -1, 0.0]\n"
    "negate: 0\n"
    "occupied_thresh: 0.65\n"
    "free_thresh: 0.196\n"
    "total_area_m2: 4\n"
    "effective_area_m2: 1\n"
    "created_at: 2026-01-02T03:04:05\n"
    "display_yaw: 0.5\n"
    "vendor_note: keep me\n";

static void MakeEditMap(const fs::path& maps, const std::string& name) {
    const fs::path slam = maps / name / "slam";
    fs::create_directories(slam);
    cv::Mat img(20, 20, CV_8UC1, cv::Scalar(255));
    img(cv::Rect(0, 0, 20, 5)).setTo(205);   // 上面 5 行未知
    img.at<uint8_t>(10, 10) = 0;              // 一个噪点
    cv::imwrite((slam / "map.png").string(), img);
    WriteText(slam / "map.yaml", kEditYaml);
    WriteText(slam / "map.pbstream", "PBSTREAM");
}

static int CountBackups(const fs::path& slam) {
    int n = 0;
    if (!fs::exists(slam / "backups")) return 0;
    for (fs::directory_iterator it(slam / "backups"), end; it != end; ++it) {
        if (it->path().filename().string().rfind("backup.", 0) == 0) ++n;
    }
    return n;
}

// G4 EditGridEx 端到端（未加载的图）
void TestG4_EditGrid(const fs::path& maps) {
    std::cout << "▶ G4 EditGridEx\n";
    MakeEditMap(maps, "g4");
    const fs::path slam = maps / "g4" / "slam";
    MapManager mm(maps.string());
    std::string rev;
    for (const auto& e : mm.ScanMapCatalog()) if (e.name == "g4") rev = e.grid_revision;
    const std::string png_before = ReadText(slam / "map.png");
    using map_manager::kShapePolyline;
    using map_manager::kToolErase;
    using map_manager::kToolWall;
    // 墙：从 (-0.95,-0.95) 到 (0.95,-0.95)，即图像最下面一行
    const std::vector<GridStrokeSpec> wall = {Stroke(kToolWall, kShapePolyline, {{-0.95, -0.95}, {0.95, -0.95}}, 0.01)};
    MapManager::GridEditResult r;
    std::string msg;

    EXPECT(mm.EditGridEx("g4", wall, "0000000000000000", &r, msg) == 2202110, "wrong revision rejected: " + msg);
    EXPECT(ReadText(slam / "map.png") == png_before && CountBackups(slam) == 0, "rejected edit touches nothing");

    EXPECT(mm.EditGridEx("g4", wall, rev, &r, msg) == 0, "edit ok: " + msg);
    EXPECT(r.changed_pixels == 20, "bottom row painted: " + std::to_string(r.changed_pixels));
    EXPECT(!r.backup_ts.empty() && !r.requires_reload, "backup made, not current map");
    const fs::path bdir = slam / "backups" / ("backup." + r.backup_ts);
    EXPECT(ReadText(bdir / "map.png") == png_before, "backup holds pre-edit png");
    EXPECT(ReadText(bdir / "map.yaml") == kEditYaml && ReadText(bdir / "map.pbstream") == "PBSTREAM",
           "backup holds pre-edit yaml + pbstream");
    const cv::Mat after = cv::imread((slam / "map.png").string(), cv::IMREAD_UNCHANGED);
    EXPECT(after.cols == 20 && after.rows == 20 && after.type() == CV_8UC1, "size and type unchanged");
    EXPECT(after.at<uint8_t>(19, 0) == 0 && after.at<uint8_t>(19, 19) == 0, "bottom row is obstacle");
    EXPECT(after.at<uint8_t>(10, 10) == 0 && after.at<uint8_t>(0, 0) == 205, "other pixels untouched");

    const auto ls = Lines(ReadText(slam / "map.yaml"));
    const auto before = Lines(kEditYaml);
    EXPECT(ls.size() == before.size() + 1, "only last_edited_at added");
    for (size_t i = 0; i < ls.size() && i < before.size(); ++i) {
        if (before[i].rfind("total_area_m2", 0) == 0 || before[i].rfind("effective_area_m2", 0) == 0) continue;
        EXPECT(ls[i] == before[i], "yaml line kept: " + before[i]);
    }
    EXPECT(StampIsNow(ls.back(), "last_edited_at: "), "edit stamps last_edited_at: " + ls.back());
    // 自由像素：20×20 − 上 5 行未知（100）− 噪点 1 − 最下一行 20 = 279 → 2.79 m²
    EXPECT(ls[6] == "total_area_m2: 4" && ls[7] == "effective_area_m2: 2.79", "area recomputed: " + ls[7]);
    std::string rev2;
    for (const auto& e : mm.ScanMapCatalog()) if (e.name == "g4") rev2 = e.grid_revision;
    EXPECT(r.revision == rev2 && rev2 != rev, "new revision returned and visible in catalog");

    // 擦除只扫未知区：无像素变化 → 成功但不备份
    const int backups = CountBackups(slam);
    EXPECT(mm.EditGridEx("g4", {Stroke(kToolErase, kShapePolyline, {{-0.5, 0.75}}, 0.3)}, rev2, &r, msg) == 0,
           "no-op edit ok: " + msg);
    EXPECT(r.changed_pixels == 0 && r.backup_ts.empty() && CountBackups(slam) == backups, "no-op: no backup");

    // 参数非法 / 缺 pbstream
    EXPECT(mm.EditGridEx("g4", {}, "", &r, msg) == 2202100, "empty strokes");
    EXPECT(mm.EditGridEx("nope", wall, "", &r, msg) == 2202030, "map not found");
    fs::remove(slam / "map.pbstream");
    EXPECT(mm.EditGridEx("g4", wall, "", &r, msg) == 2202072, "missing pbstream: " + msg);
}

// G5 版本轮换：保留 2 份，同一秒内连续编辑得到不同名字
void TestG5_Prune(const fs::path& maps) {
    std::cout << "▶ G5 backup rotation\n";
    MakeEditMap(maps, "g5");
    const fs::path slam = maps / "g5" / "slam";
    MapManager mm(maps.string());
    mm.SetMapBackupKeepCount(2);
    MapManager::GridEditResult r;
    std::string msg;
    std::vector<std::string> ts;
    for (int i = 0; i < 3; ++i) {
        const double y = -0.95 + 0.1 * i;  // 每次画不同的行，保证有像素变化
        mm.EditGridEx("g5", {Stroke(map_manager::kToolWall, map_manager::kShapePolyline, {{-0.95, y}, {0.95, y}}, 0.01)},
                      "", &r, msg);
        ts.push_back(r.backup_ts);
    }
    EXPECT(ts[0] != ts[1] && ts[1] != ts[2], "unique backup names");
    EXPECT(CountBackups(slam) == 2, "keeps 2 newest");
    EXPECT(!fs::exists(slam / "backups" / ("backup." + ts[0])) && fs::exists(slam / "backups" / ("backup." + ts[2])),
           "oldest pruned");
}

// G6 编辑当前已加载地图 → requires_reload
void TestG6_Current(const fs::path& maps) {
    std::cout << "▶ G6 current map requires reload\n";
    MakeEditMap(maps, "g6");
    MapManager mm(maps.string());
    if (!mm.LoadMap("g6")) { std::cout << "  (skip: LoadMap unavailable in this env)\n"; return; }
    MapManager::GridEditResult r;
    std::string msg;
    // 点放在格中心（0.05, 0.05）：落在四格公共角点上的极细笔一格都不覆盖，这是规则本身的结果
    const auto before = mm.GetMap();
    const std::vector<int8_t> grid_before = before ? before->data : std::vector<int8_t>{};
    EXPECT(mm.EditGridEx("g6", {Stroke(map_manager::kToolWall, map_manager::kShapePolyline, {{0.05, 0.05}}, 0.01)},
                         "", &r, msg) == 0 && r.requires_reload, "current map edit requires reload: " + msg);
    // 同名重新加载必须从磁盘重读：内存栅格变成编辑后的内容，epoch 递增
    const uint64_t epoch_before = mm.GetEpoch();
    EXPECT(mm.LoadMap("g6"), "reload current map");
    const auto after = mm.GetMap();
    EXPECT(after && after->data != grid_before, "reload re-reads the edited map.png");
    EXPECT(mm.GetEpoch() == epoch_before + 1, "reload bumps epoch");
    // 重读失败（yaml 坏掉）保留原栅格，返回失败
    const fs::path yaml = maps / "g6" / "slam" / "map.yaml";
    const std::string yaml_text = ReadText(yaml);
    WriteText(yaml, "image: missing.png\nresolution: 0.1\norigin: [0, 0, 0]\n");
    EXPECT(!mm.LoadMap("g6") && mm.GetMap() == after, "failed reload keeps the previous grid");
    WriteText(yaml, yaml_text);
}

// ── 底图版本列表与回滚 ─────────────────────────────────────────────────

// V1~V3 列版本、回滚、撤销、拒绝
void TestV1_VersionsAndRollback(const fs::path& maps) {
    std::cout << "▶ V1 versions / rollback / undo\n";
    MakeEditMap(maps, "v1");
    const fs::path slam = maps / "v1" / "slam";
    MapManager mm(maps.string());
    // 一份「定位侧保存」的旧版本（带 .complete）和一份三件不齐的版本
    for (const std::string ts : {"20200101-000000", "20200102-000000"}) {
        const fs::path b = slam / "backups" / ("backup." + ts);
        fs::create_directories(b);
        fs::copy_file(slam / "map.yaml", b / "map.yaml");
        fs::copy_file(slam / "map.png", b / "map.png");
        if (ts == "20200101-000000") {
            fs::copy_file(slam / "map.pbstream", b / "map.pbstream");
            WriteText(b / ".complete", "timestamp=x\n");
        }
    }
    const std::string png_orig = ReadText(slam / "map.png");
    std::string rev_orig;
    for (const auto& e : mm.ScanMapCatalog()) if (e.name == "v1") rev_orig = e.grid_revision;

    MapManager::GridEditResult r;
    std::string msg;
    mm.EditGridEx("v1", {Stroke(map_manager::kToolWall, map_manager::kShapePolyline, {{-0.95, -0.95}, {0.95, -0.95}}, 0.01)},
                  "", &r, msg);
    const std::string edit_ts = r.backup_ts;
    const std::string png_edited = ReadText(slam / "map.png");

    // 版本随目录扫描（get_maps）一起返回
    auto versions_of = [&](const std::string& name) {
        for (const auto& e : mm.ScanMapCatalog()) if (e.name == name) return e.versions;
        return std::vector<map_manager::MapVersionEntry>{};
    };
    auto vs = versions_of("v1");
    EXPECT(vs.size() == 3, "3 versions listed in catalog");
    EXPECT(vs.size() == 3 && vs[0].version_ts == edit_ts && vs[0].source == "grid_edit" && vs[0].complete,
           "newest first, grid_edit");
    EXPECT(vs.size() == 3 && ReadText(slam / "backups" / ("backup." + edit_ts) / "map.png") == png_orig,
           "grid_edit version holds pre-edit content");
    EXPECT(vs.size() == 3 && vs[1].version_ts == "20200102-000000" && !vs[1].complete && vs[1].source == "unknown",
           "incomplete flagged");
    EXPECT(vs.size() == 3 && vs[2].source == "localization_save" && vs[2].complete, ".complete -> localization_save");
    EXPECT(vs.size() == 3 && vs[2].saved_at > 0, "saved_at parsed from version_ts");

    // 回滚到编辑前；回滚前的状态（编辑后）自动存成 rollback 版本
    EXPECT(mm.RollbackMapEx("v1", edit_ts, &r, msg) == 0, "rollback ok: " + msg);
    EXPECT(ReadText(slam / "map.png") == png_orig, "png restored to pre-edit");
    EXPECT(!r.backup_ts.empty() && !r.requires_reload && r.revision == rev_orig, "snapshot + revision");
    {
        const auto yl = Lines(ReadText(slam / "map.yaml"));
        EXPECT(!yl.empty() && StampIsNow(yl.back(), "last_edited_at: "), "rollback stamps last_edited_at");
    }
    vs = versions_of("v1");
    EXPECT(!vs.empty() && vs[0].version_ts == r.backup_ts && vs[0].source == "rollback", "snapshot listed as rollback");
    // 撤销：回滚到快照
    EXPECT(mm.RollbackMapEx("v1", r.backup_ts, &r, msg) == 0 && ReadText(slam / "map.png") == png_edited,
           "undo rollback restores edited png");
    // 空版本号 = 最新一份：此刻最新的是撤销那一步存下的快照（编辑前的状态）
    EXPECT(mm.RollbackMapEx("v1", "", &r, msg) == 0 && ReadText(slam / "map.png") == png_orig,
           "empty version_ts = latest: " + msg);

    // 拒绝
    EXPECT(mm.RollbackMapEx("v1", "20990101-000000", &r, msg) == 2202120, "missing version");
    EXPECT(mm.RollbackMapEx("v1", "20200102-000000", &r, msg) == 2202120, "incomplete version");
    EXPECT(mm.RollbackMapEx("v1", "../x", &r, msg) == 2202120, "path-like version_ts");
    EXPECT(mm.RollbackMapEx("nope", "20200101-000000", &r, msg) == 2202030, "missing map");
}

// V2 当前图回滚 → requires_reload；轮换时目标版本受保护
void TestV2_CurrentAndProtect(const fs::path& maps) {
    std::cout << "▶ V2 current map / protect target from pruning\n";
    MakeEditMap(maps, "v2");
    const fs::path slam = maps / "v2" / "slam";
    MapManager mm(maps.string());
    mm.SetMapBackupKeepCount(2);
    MapManager::GridEditResult r;
    std::string msg;
    std::vector<std::string> ts;
    for (int i = 0; i < 2; ++i) {
        const double y = -0.95 + 0.1 * i;
        mm.EditGridEx("v2", {Stroke(map_manager::kToolWall, map_manager::kShapePolyline, {{-0.95, y}, {0.95, y}}, 0.01)},
                      "", &r, msg);
        ts.push_back(r.backup_ts);
    }
    // 保留 2 份时回滚到最旧那份：快照 + 轮换不能把目标删掉
    EXPECT(mm.RollbackMapEx("v2", ts[0], &r, msg) == 0, "rollback to oldest ok: " + msg);
    EXPECT(fs::exists(slam / "backups" / ("backup." + ts[0])), "target version survived pruning");
    if (mm.LoadMap("v2")) {
        EXPECT(mm.RollbackMapEx("v2", ts[1], &r, msg) == 0 && r.requires_reload, "current map -> requires_reload");
    }
}

// V3 附带文件：存在时随版本保存，回滚时一并恢复（点云 / 线特征与底图同一坐标系）
void TestV3_ExtraFiles(const fs::path& maps) {
    std::cout << "▶ V3 extra files in versions\n";
    MakeEditMap(maps, "v3");
    const fs::path slam = maps / "v3" / "slam";
    WriteText(slam / "map.pcd", "PCD-OLD");
    WriteText(slam / "map_line_map.json", "LINES-OLD");
    MapManager mm(maps.string());
    MapManager::GridEditResult r;
    std::string msg;
    mm.EditGridEx("v3", {Stroke(map_manager::kToolWall, map_manager::kShapePolyline, {{0.05, 0.05}}, 0.01)}, "", &r, msg);
    const fs::path vdir = slam / "backups" / ("backup." + r.backup_ts);
    EXPECT(ReadText(vdir / "map.pcd") == "PCD-OLD" && ReadText(vdir / "map_line_map.json") == "LINES-OLD",
           "extras saved into version");
    EXPECT(!fs::exists(vdir / "map_highlight.pcd"), "absent extras not created");
    WriteText(slam / "map.pcd", "PCD-NEW");
    WriteText(slam / "map_line_map.json", "LINES-NEW");
    EXPECT(mm.RollbackMapEx("v3", r.backup_ts, &r, msg) == 0, "rollback ok: " + msg);
    EXPECT(ReadText(slam / "map.pcd") == "PCD-OLD" && ReadText(slam / "map_line_map.json") == "LINES-OLD",
           "extras restored with the triple");
}

int main(int argc, char** argv) {
    (void)argc;
    google::InitGoogleLogging(argv[0]);
    FLAGS_minloglevel = 1;
    const fs::path root = fs::temp_directory_path() / ("map_manager_edit_test_" + std::to_string(::getpid()));
    const fs::path maps = root / "maps";
    fs::create_directories(maps);

    TestE1_Doc(root);
    TestE2_SetDisplayYaw(maps);
    TestE3_Guards(maps);
    TestE4_SideEffects(maps);
    TestE5_Catalog(maps);
    TestG1_Classify();
    TestG2_Rasterize();
    TestG3_Validate();
    TestG4_EditGrid(maps);
    TestG5_Prune(maps);
    TestG6_Current(maps);
    TestV1_VersionsAndRollback(maps);
    TestV2_CurrentAndProtect(maps);
    TestV3_ExtraFiles(maps);

    fs::remove_all(root);
    std::cout << "\n== " << g_pass << " checks passed, " << g_fail << " failed ==\n";
    return g_fail == 0 ? 0 : 1;
}
