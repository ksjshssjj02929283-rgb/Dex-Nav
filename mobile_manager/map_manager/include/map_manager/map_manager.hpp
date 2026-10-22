#ifndef MAP_MANAGER_MAP_MANAGER_HPP_
#define MAP_MANAGER_MAP_MANAGER_HPP_

#include <cstdint>
#include <iostream>
#include <memory>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <string>
#include <vector>
#include <ctime>

#include "map_manager/grid_edit.hpp"
#include "map_manager/image_loader.hpp"

namespace map_manager {

struct MapInfo {
    std::string name;
    std::string path;
    double resolution;
    double origin_x;
    double origin_y;
    double origin_yaw;
    double width;
    double height;
};

// 一个底图版本（slam/backups/backup.<ts>/）。只取读目录即可得到的信息，保证列表查询轻量。
struct MapVersionEntry {
    std::string version_ts;
    std::time_t saved_at = 0;
    std::string source;          // localization_save / grid_edit / rollback / unknown
    bool complete = false;       // 三件套齐全
};

// get_maps 返回的目录条目（只读 yaml，不加载地图）
struct MapCatalogEntry {
    std::string name;
    std::string map_path;           // <map_folder>/<name>/slam
    double total_area_m2 = 0.0;
    double effective_area_m2 = 0.0;
    std::time_t updated_at = 0;     // map.yaml mtime
    std::string created_at;         // map.yaml created_at（本地时间 ISO 8601，建图完成时刻）；无此键为空
    // map.yaml data_version：同目录 map.pbstream 的数据版本，"0.3.0" 等 /
    // "legacy"（0.3.0 之前的旧版）；无此键为空，表示未知（yaml 由旧版本程序写出）
    std::string data_version;
    std::string thumbnail;          // <map_path>/map.png
    // 底图内容指纹：map.png 文件内容 + map.yaml 的 resolution / origin。
    // 供 OP_EDIT_GRID 乐观锁，编辑、回滚、替换、对齐之后都会变；display_yaw 与面积不参与。
    std::string grid_revision;
    double display_yaw = 0.0;       // map.yaml display_yaw（弧度）；无此键为 0
    // 底图最后编辑时间：map.yaml last_edited_at（底图编辑 / 回滚 / 对齐 / 替换 / 重设原点 / 定位侧保存时写入）；
    // 旧图无此键时退回 map.png、map.yaml mtime 的较大值。不含资产（资产时间见 get_assets）
    std::time_t last_edited_at = 0;
    std::vector<MapVersionEntry> versions;  // 底图版本，新的在前
};

// get_trashed_maps 返回的回收站条目。
// 归档是整目录 rename，<entry>/slam/map.yaml 结构完整保留，因此面积 /
// created_at / 缩略图与正常地图同口径可读 —— 直接复用 MapCatalogEntry。
struct TrashedMapEntry {
    std::string entry_name;      // 回收站内目录名，如 office_20260922_143012
    std::string original_name;   // 解析出的原地图名，如 office
    std::time_t archived_at = 0; // 归档时刻（解析自 entry_name 时间戳段，失败回退目录 mtime）
    MapCatalogEntry map_info;    // name = entry_name；map_path 指向回收站内路径
};

class MapManager {
public:
    explicit MapManager(const std::string& default_map_folder);
    ~MapManager() = default;

    bool UpdateMapList();
    bool LoadMap(const std::string& map_name);
    bool RemoveMap(const std::string& map_name);
    bool RenameMap(const std::string& old_name, const std::string& new_name, std::string& error_msg);
    // 带错误码版本（供 ROS 层 MapManagerCommand 使用）。返回 0 成功，否则为 MapState 内的状态码常量（22-02 段）。
    int32_t LoadMapEx(const std::string& map_name, std::string& message);
    int32_t RemoveMapEx(const std::string& map_name, std::string& message);
    int32_t RenameMapEx(const std::string& old_name, const std::string& new_name, std::string& message);
    // 设置任意一张地图的显示朝向（不要求已加载）：只改 map.yaml 的 display_yaw 一行，
    // 归一化到 (-π, π]，其余行逐字节保留；与 SLAM 侧写 map.yaml 用同一把目录锁。
    // 只是显示元数据，坐标、底图、定位都不变，因此不重新下发地图。
    int32_t SetDisplayYawEx(const std::string& map_name, double display_yaw, std::string& message);

    // 底图笔画编辑（不要求已加载）。依次：
    //   ① 校验笔画 → ② expected_revision 非空时与当前底图指纹比对 → ③ 栅格化（像素规则见 grid_edit.hpp）
    //   ④ 有像素变化才落盘：先把 map.pbstream / map.png / map.yaml 三件备份为
    //      slam/backups/backup.<YYYYMMDD-HHMMSS>/（rollback_map 按此回退，超出保留份数的最旧版本删除），
    //      再原子写 map.png、合并写 map.yaml 的两个面积键（其余键原样保留）
    // 图像宽高、origin、map.pbstream 都不变，定位不受影响。没有像素变化时返回成功但不备份不写盘。
    struct GridEditResult {
        std::string revision;      // 编辑后的底图指纹
        std::string backup_ts;     // 本次产生的版本；无像素变化时为空
        bool requires_reload = false;  // 当前图改了文件但导航尚未用上（回滚、或编辑后重读失败）
        bool applied = false;          // 当前图：已重读底图，随 /map 重发对导航生效，epoch 不变
        int changed_pixels = 0;
    };
    int32_t EditGridEx(const std::string& map_name, const std::vector<GridStrokeSpec>& strokes,
                       const std::string& expected_revision, GridEditResult* result, std::string& message);

    // 把任意一张地图的底图三件套恢复到某个版本（version_ts 空 = 最新）。恢复前先把当前三件套
    // 存成新版本（source = rollback）写入 result->backup_ts，回滚到它即可撤销；目标版本不参与这次轮换。
    // 回滚当前地图只恢复文件、requires_reload = true，是否重新下发由调用方（command_board）决定。
    int32_t RollbackMapEx(const std::string& map_name, const std::string& version_ts,
                          GridEditResult* result, std::string& message);

    // 底图版本保留份数（与 SLAM 侧 map_backup_keep_count 同义，默认 50）。
    void SetMapBackupKeepCount(int n) { map_backup_keep_count_ = n < 0 ? 0 : n; }
    // 扫描 map_folder 下各地图的 slam/map.yaml，生成目录条目（不加载地图）
    std::vector<MapCatalogEntry> ScanMapCatalog();

    bool loadMapFromYaml(std::string path_to_yaml);
    bool loadMapFromValues(std::string map_file_name,
                           double resolution,
                           int negate,
                           double occ_th,
                           double free_th,
                           double origin[3],
                           MapMode mode);

    std::vector<std::string> GetMapList() const { return map_list_; }
    std::shared_ptr<nav_msgs::msg::MapMetaData> GetMapMetaData() const { return meta_data_message_; }
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> GetMap() const { return current_map_; }
    const std::string GetMapName() const { return current_map_name_; }
    const std::string GetMapFolder() const { return map_folder_; }
    const bool HasMapChanged() const { return has_map_changed_; }

    // 重读当前地图的底图（map.yaml / map.png）替换内存栅格，不递增 epoch。
    // 只用于底图像素变化而 map.pbstream 不变的情况：定位不需要重载，导航随 /map 重发即可用上新底图。
    // 重读失败保留原栅格并返回 false。
    bool RefreshCurrentGrid(std::string* error);

    // 导航当前使用的底图指纹（加载或重读底图时更新）；与 get_maps 的文件指纹不同 = 有未生效的修改
    std::string GetAppliedGridRevision() const { return applied_grid_revision_; }

    // 地图加载 epoch：每次成功 LoadMap（含同名重载/回退）单调 +1。
    // 消费方比对 epoch 判断是否需要重载，不依赖名字变化。
    uint64_t GetEpoch() const { return map_epoch_; }

    void ResetFlags() { has_map_changed_ = false; }

    // ── 地图回收站（map trash）─────────────────────────────────────────
    // map_folder 的生命周期归本类管，回收站是其中一项设施：
    //   - 目录约定 <map_folder>/<trash_dirname>/<map_name>_<YYYYMMDD_HHMMSS>/
    //   - 建图侧 overwrite 重建地图目录时，把旧目录整体 rename 进来（不再硬删）
    //   - 本类负责不把它当地图列出、并按上限轮换清理
    // 回收站必须位于 map_folder 内部：同挂载点下 rename 才是原子且瞬时的，
    // 放到别的挂载点会退化成 copy + delete，既慢又非原子。
    struct MapTrashOptions {
        bool enable = true;
        std::string dirname = ".map_trash";
        int max_entries = 50;              // 主判据：保留份数上限
        double max_total_gib = 10.0;       // 副判据：总容量上限，兜底极端大图
    };
    void SetMapTrashOptions(const MapTrashOptions& options) { trash_options_ = options; }
    const MapTrashOptions& GetMapTrashOptions() const { return trash_options_; }

    // 回收站绝对路径（<map_folder>/<dirname>）。map_folder 为空时返回空串。
    std::string GetMapTrashDir() const;
    // 按 max_entries / max_total_gib 两条判据轮换清理，从旧到新删。
    // 返回实际删除的条目数；回收站不存在或未启用时返回 0。
    int PruneMapTrash();
    // 回收站内的归档条目（名称按时间倒序，最新在前），供排查与后续 restore 用。
    std::vector<std::string> ListTrashedMaps() const;
    // 回收站条目 + 元数据（供 get_trashed_maps 返回），同样按归档时刻倒序。
    std::vector<TrashedMapEntry> ScanTrashCatalog() const;
    // 从回收站取回一张地图：整目录 rename 回 <map_folder>/<目标名>。
    // 目标名 = new_name，留空则用从 entry_name 解析出的 original_name。
    //
    // 三条护栏（缺一不可，顺序即检查顺序）：
    //   ① 目标名已存在        → kErrNameExists，**绝不覆盖**（恢复不能毁掉现存数据）
    //   ② 目标名 = 当前加载图  → kErrInUse（不在定位运行时换图，与 RemoveMapEx 一致）
    //   ③ move 不是 copy      —— 它本就是 move 进去的，同挂载点 rename 原子；
    //      copy 会双倍占盘且留下「恢复完还剩一份」的歧义
    // 成功时把实际恢复成的名字写入 out_restored_name，并刷新地图列表。
    int32_t RestoreMapEx(const std::string& entry_name, const std::string& new_name,
                         std::string& message, std::string& out_restored_name);
    // 把一张地图整目录移入回收站：<trash>/<map_name>_<YYYYMMDD_HHMMSS>[_<N>]/，
    // 与建图 overwrite 时的归档同一命名，可用 RestoreMapEx 取回。
    // 护栏：当前加载图 → kErrInUse；回收站未启用 → kErrTrashDisabled（不退化为硬删）。
    // 成功时把条目名写入 out_entry_name，随后按上限轮换并刷新地图列表。
    int32_t TrashMapEx(const std::string& map_name, std::string& message,
                       std::string& out_entry_name);
    // 从回收站目录名解析原地图名：剥掉结尾的 _<YYYYMMDD>_<HHMMSS>（及可选的
    // 同秒去重后缀 _<N>）。格式不匹配时原样返回。
    // 放在服务端做，不让每个调用方各自实现一遍解析规则。
    static std::string ParseOriginalName(const std::string& entry_name);

private:
    // 探测旧扁平格式地图并迁移到 slam/assets 分层（DEC-007 新格式）；LoadMap 前置调用，幂等。
    bool MigrateLegacyMapLayout(const std::string& map_name);
    // 从 <map_path>/map.yaml 读取面积 / created_at / mtime 填进条目。
    // ScanMapCatalog 与 ScanTrashCatalog 共用 —— 两者只是扫描根不同。
    static void FillEntryFromYaml(const std::string& map_path, MapCatalogEntry* entry);
    // 目录是否为可加载地图：新布局 slam/map.yaml 或旧扁平布局 map.yaml（可迁移）。
    // 只有 slam/ 空目录（建图被中断的残留）或只有 pbstream 的目录都不算，get_maps 不列出，load 报 2202030（不存在）而不是 2202072（读取失败）。
    bool IsLoadableMapDir(const std::string& map_name) const;

    std::string map_folder_ = "";
    std::vector<std::string> map_list_;

    std::string current_map_name_ = "";

    std::shared_ptr<nav_msgs::msg::MapMetaData> meta_data_message_;
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> current_map_;

    bool has_map_changed_ = false;

    std::string applied_grid_revision_;
    void UpdateAppliedGridRevision();

    // 单调递增的地图加载版本号；0 = 尚未加载任何地图。不持久化。
    uint64_t map_epoch_ = 0;

    MapTrashOptions trash_options_;
    int map_backup_keep_count_ = 50;
};

}  // namespace map_manager

#endif  // MAP_MANAGER_MAP_MANAGER_HPP_
