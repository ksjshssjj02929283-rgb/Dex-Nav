#ifndef MAP_MANAGER_MAP_YAML_DOC_HPP_
#define MAP_MANAGER_MAP_YAML_DOC_HPP_

#include <map>
#include <ctime>
#include <optional>
#include <string>
#include <vector>

namespace map_manager {

// map.yaml 的内存镜像：整份载入，按键读改，整份写出。
// 与 SLAM 侧（cartographer_ros ros_map.cc）的同名实现规则一致，两边写同一份文件：
//   - 内部按行保存，没改过的行（含注释、空行、行尾注释）写回时逐字节不变
//   - 只处理扁平的单行 "key: value"；其它行当作无键行原样保留
//   - 重复键只认第一次出现
// 不用 yaml-cpp 输出：它会丢注释、改浮点格式和 origin 的行内列表写法。
class MapYamlDoc {
public:
    // 路径为空或文件不存在 = 空文档。
    static MapYamlDoc Load(const std::string& path);

    bool Has(const std::string& key) const { return index_.count(key) > 0; }
    std::optional<std::string> Get(const std::string& key) const;
    std::optional<double> GetDouble(const std::string& key) const;
    // 已有：原位改值；没有：追加到末尾。
    void Set(const std::string& key, const std::string& value);
    void Erase(const std::string& key);
    std::string ToString() const;

private:
    struct Line {
        std::string key;  // 空 = 注释 / 空行 / 续行
        std::string text;
        bool erased = false;
    };
    static std::string KeyOf(const std::string& text);

    std::vector<Line> lines_;
    std::map<std::string, size_t> index_;  // 键 → lines_ 下标（已删除的不在内）
};

// 目录级排它锁：写同一份 map.yaml 的所有进程（SLAM 节点、离线工具、map_manager）
// 都对 <dir>/.map.yaml.lock 加 flock，读改写之间不会被别人插进来。
// 拿不到锁不阻止写盘，只告警（与 SLAM 侧一致）。
class MapYamlDirLock {
public:
    explicit MapYamlDirLock(const std::string& dir);
    ~MapYamlDirLock();
    MapYamlDirLock(const MapYamlDirLock&) = delete;
    MapYamlDirLock& operator=(const MapYamlDirLock&) = delete;

private:
    int fd_ = -1;
};

// 地图级排它锁（与 SLAM 离线工具同一把：<slam_dir>/.map_op.lock）：底图编辑 / 回滚与对齐 / 替换 / 重设原点
// 互斥，同一张地图同一时刻只有一个改动操作。非阻塞：已被占用时 acquired() 为 false。
class MapOpLock {
public:
    explicit MapOpLock(const std::string& slam_dir);
    ~MapOpLock();
    MapOpLock(const MapOpLock&) = delete;
    MapOpLock& operator=(const MapOpLock&) = delete;
    bool acquired() const { return fd_ >= 0; }

private:
    int fd_ = -1;
};

// 写 <path>.tmp → fsync → rename。失败时删临时文件，原文件不受影响。
bool WriteFileAtomically(const std::string& path, const std::string& content, std::string* error);

// 归一化到 (-π, π]。
double NormalizeAngle(double rad);

// 角度会被反复读回再叠加，给足 10 位有效数字。
std::string FormatAngle(double rad);

// map.yaml 时间键（created_at / last_edited_at）的格式：机器人本地时间 ISO 8601、无时区后缀，
// 如 2026-10-10T20:51:03，与 SLAM 侧写法一致。
std::string LocalTimeIso8601();
// 解析上面的格式（允许带引号）；失败返回 0。
std::time_t ParseLocalIso8601(const std::string& text);

// 底图内容指纹：FNV-1a 64(map.png 文件字节) 再叠加 map.yaml 的 resolution / origin 原文，16 位十六进制。
// map_path 为 slam 目录；map.png 不存在时返回空串。get_maps、底图编辑乐观锁、导出清单共用这一个实现。
std::string GridRevision(const std::string& map_path, const MapYamlDoc& yaml);

}  // namespace map_manager

#endif  // MAP_MANAGER_MAP_YAML_DOC_HPP_
