// map_manager 回收站恢复功能测试（核心层，不起 ROS 节点）。
//
// 覆盖：ParseOriginalName 的边角情形、回收站不污染地图列表、ScanTrashCatalog
// 的元数据读取、RestoreMapEx 的三条护栏（条目不存在 / 目标名已存在绝不覆盖 /
// 回收站禁用）、成功恢复的 move 语义与资产一并回来、改名恢复。
//
// 本包现有测试都是 test_support/ 下的 Python 服务级脚本，没有 CMake 测试基建，
// 故本文件不接进 CMakeLists，手动构建运行：
//
//   source /opt/ros/humble/setup.bash
//   # 先按包的常规方式构建出 libmap_manager_core.a，然后在该 build 目录下：
//   g++ -std=c++17 -o restore_test \
//       ../<pkg>/test_support/map_manager_restore_test.cpp \
//       -I ../<pkg>/include -I /opt/ros/humble/include \
//       $(for d in /opt/ros/humble/include/*/; do echo -I$d; done) \
//       libmap_manager_core.a -lboost_filesystem -lboost_system -lglog -lyaml-cpp \
//       $(sdl2-config --libs) -lSDL2_image -L/opt/ros/humble/lib -lrclcpp -lrcutils
//   ./restore_test    # 全通过则退出码 0
//
// 测试自建临时目录、自清理，不依赖 ~/.dexe/data/maps。
#include <glog/logging.h>
#include <boost/filesystem.hpp>
#include <iostream>
#include "map_manager/map_manager.hpp"

namespace fs = boost::filesystem;
using map_manager::MapManager;

static int g_fail = 0;
#define CHECK_EQ_STR(actual, expected, what)                                        \
  do { const std::string a__ = (actual), e__ = (expected);                          \
       if (a__ != e__) { std::cout << "  ❌ " << what << ": got '" << a__           \
                                   << "' want '" << e__ << "'\n"; ++g_fail; }       \
       else std::cout << "  ✅ " << what << " = " << a__ << "\n"; } while (0)
#define CHECK_CODE(actual, expected, what)                                          \
  do { const int a__ = (actual), e__ = (expected);                                  \
       if (a__ != e__) { std::cout << "  ❌ " << what << ": got " << a__            \
                                   << " want " << e__ << "\n"; ++g_fail; }          \
       else std::cout << "  ✅ " << what << " -> " << a__ << "\n"; } while (0)

static void WriteMap(const fs::path& slam_dir, const std::string& tag) {
  fs::create_directories(slam_dir);
  std::ofstream(( slam_dir / "map.yaml").string())
      << "resolution: 0.05\ntotal_area_m2: 12.5\neffective_area_m2: 9.5\n"
      << "created_at: 2026-09-20T10:00:00\n# " << tag << "\n";
  std::ofstream((slam_dir / "map.png").string()) << "png";
}

int main() {
  google::InitGoogleLogging("restore_test");
  FLAGS_minloglevel = 2;  // 只看测试输出

  std::cout << "── ParseOriginalName（含边角）──\n";
  CHECK_EQ_STR(MapManager::ParseOriginalName("office_20260922_143012"), "office", "常规");
  CHECK_EQ_STR(MapManager::ParseOriginalName("office_2_20260922_143012"), "office_2",
               "名字自带数字后缀");
  CHECK_EQ_STR(MapManager::ParseOriginalName("office_20260922_143012_2"), "office",
               "同秒去重后缀");
  CHECK_EQ_STR(MapManager::ParseOriginalName("my_map_v3_20260101_000000"), "my_map_v3",
               "多下划线名字");
  CHECK_EQ_STR(MapManager::ParseOriginalName("plain_name"), "plain_name", "非归档名原样返回");
  CHECK_EQ_STR(MapManager::ParseOriginalName("office_2026_143012"), "office_2026_143012",
               "日期段位数不对 -> 原样返回");

  const fs::path root = fs::temp_directory_path() / fs::unique_path("mm_restore_%%%%");
  fs::create_directories(root);
  const fs::path trash = root / ".map_trash";
  WriteMap(trash / "office_20260922_143012" / "slam", "trashed-office");
  fs::create_directories(trash / "office_20260922_143012" / "assets");
  std::ofstream((trash / "office_20260922_143012" / "assets" / "map_assets.json").string()) << "{}";
  WriteMap(trash / "lab_20260921_090000" / "slam", "trashed-lab");
  WriteMap(root / "existing" / "slam", "live-existing");

  MapManager mm(root.string());
  MapManager::MapTrashOptions opts;  // enable=true, dirname=".map_trash"
  mm.SetMapTrashOptions(opts);
  mm.UpdateMapList();

  std::cout << "\n── 回收站不污染地图列表 ──\n";
  const auto maps = mm.GetMapList();
  bool has_trash_dir = false;
  for (const auto& n : maps) if (n == ".map_trash") has_trash_dir = true;
  CHECK_CODE(has_trash_dir ? 1 : 0, 0, ".map_trash 不在地图列表");
  CHECK_CODE(static_cast<int>(maps.size()), 1, "地图列表只剩 existing");

  std::cout << "\n── ScanTrashCatalog（元数据同口径）──\n";
  const auto cat = mm.ScanTrashCatalog();
  CHECK_CODE(static_cast<int>(cat.size()), 2, "回收站条目数");
  if (!cat.empty()) {
    CHECK_EQ_STR(cat[0].entry_name, "office_20260922_143012", "倒序首条 entry_name");
    CHECK_EQ_STR(cat[0].original_name, "office", "首条 original_name");
    CHECK_CODE(cat[0].map_info.total_area_m2 > 12.0 ? 1 : 0, 1, "归档条目仍能读出面积");
    CHECK_EQ_STR(cat[0].map_info.created_at, "2026-09-20T10:00:00", "归档条目 created_at");
    CHECK_CODE(cat[0].archived_at > 0 ? 1 : 0, 1, "archived_at 解析成功");
  }

  std::cout << "\n── 护栏 ──\n";
  std::string msg, restored;
  CHECK_CODE(mm.RestoreMapEx("nope_20260101_000000", "", msg, restored), 2202030, "条目不存在");
  CHECK_CODE(mm.RestoreMapEx("office_20260922_143012", "existing", msg, restored), 2202050,
             "目标名已存在 -> 绝不覆盖");
  CHECK_CODE(fs::exists(root / "existing" / "slam" / "map.yaml") ? 1 : 0, 1,
             "被拒后现存地图完好无损");
  MapManager::MapTrashOptions off; off.enable = false;
  mm.SetMapTrashOptions(off);
  CHECK_CODE(mm.RestoreMapEx("office_20260922_143012", "", msg, restored), 2202090, "回收站禁用");
  mm.SetMapTrashOptions(opts);

  std::cout << "\n── 成功恢复（move 语义 + 资产一起回来）──\n";
  CHECK_CODE(mm.RestoreMapEx("office_20260922_143012", "", msg, restored), 0, "恢复成功");
  CHECK_EQ_STR(restored, "office", "恢复成原名");
  CHECK_CODE(fs::exists(root / "office" / "slam" / "map.yaml") ? 1 : 0, 1, "底图已就位");
  CHECK_CODE(fs::exists(root / "office" / "assets" / "map_assets.json") ? 1 : 0, 1,
             "资产一起恢复");
  CHECK_CODE(fs::exists(trash / "office_20260922_143012") ? 1 : 0, 0,
             "move 非 copy：回收站条目已移走");
  CHECK_CODE(static_cast<int>(mm.GetMapList().size()), 2, "地图列表已刷新");

  std::cout << "\n── 改名恢复 ──\n";
  CHECK_CODE(mm.RestoreMapEx("lab_20260921_090000", "lab_backup", msg, restored), 0, "改名恢复");
  CHECK_EQ_STR(restored, "lab_backup", "恢复成指定名");
  CHECK_CODE(fs::exists(root / "lab_backup" / "slam" / "map.yaml") ? 1 : 0, 1, "新名目录就位");

  fs::remove_all(root);
  std::cout << "\n" << (g_fail == 0 ? "✅ 全部通过" : "❌ 失败 " + std::to_string(g_fail) + " 项") << "\n";
  return g_fail == 0 ? 0 : 1;
}
