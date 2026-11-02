#ifndef MAP_ASSET_RVIZ_PLUGIN__ASSET_TARGET_HPP_
#define MAP_ASSET_RVIZ_PLUGIN__ASSET_TARGET_HPP_

// 资产编辑目标：MapAssetManagerPanel 选定的地图，面板各页签与地图上的资产工具共用。
// 空 = 当前已加载地图（资产节点的内存工作集）；非空 = 指定地图（资产节点按图寻址读写文件）。
// 所有 get_assets / set_assets / save_assets 请求都按它填 map_name。

#include <QString>

#include <mutex>
#include <string>

#include <atomic>
#include <functional>
#include <memory>

#include <nav_interfaces/srv/get_assets.hpp>
#include <nav_interfaces/srv/set_assets.hpp>

namespace map_asset_rviz_plugin {

class AssetTarget {
public:
    static std::string mapName() {
        auto& s = state();
        std::lock_guard<std::mutex> lock(s.mutex);
        return s.map_name;
    }
    static void setMapName(const std::string& name) {
        auto& s = state();
        std::lock_guard<std::mutex> lock(s.mutex);
        s.map_name = name;
    }
    static bool isCurrent() { return mapName().empty(); }

    // 数据代数：编辑目标切换或目标图资产变化时由面板递增。地图工具缓存了端点 / 节点列表，
    // 发现代数变了就在下一次鼠标事件时重新加载，避免拿着旧图或旧数据去连边、命名
    static uint64_t generation() { return state().generation.load(); }
    static void bumpGeneration() { ++state().generation; }

    // 工具执行结果转给面板状态栏显示（工具自己的提示只在 RViz 窗口底部，容易看漏）
    static void setReporter(std::function<void(const QString&, bool)> fn) {
        auto& s = state();
        std::lock_guard<std::mutex> lock(s.mutex);
        s.reporter = std::move(fn);
    }
    static void report(const QString& text, bool ok) {
        std::function<void(const QString&, bool)> fn;
        {
            auto& s = state();
            std::lock_guard<std::mutex> lock(s.mutex);
            fn = s.reporter;
        }
        if (fn) fn(text, ok);
    }
    // 给状态栏用的地图名
    static std::string label() {
        const std::string n = mapName();
        return n.empty() ? std::string("当前地图") : n;
    }

private:
    struct State {
        std::mutex mutex;
        std::string map_name;
        std::atomic<uint64_t> generation{0};
        std::function<void(const QString&, bool)> reporter;
    };
    static State& state() {
        static State s;
        return s;
    }
};

// 请求按编辑目标填 map_name（GetAssets / SetAssets / SaveAssets 都有该字段）
template <typename RequestPtr>
inline void applyTarget(const RequestPtr& req) {
    req->map_name = AssetTarget::mapName();
}

// 按编辑目标读指定类型资产的 get_assets 请求
inline std::shared_ptr<nav_interfaces::srv::GetAssets::Request> targetAssetsRequest(uint8_t kinds) {
    auto req = std::make_shared<nav_interfaces::srv::GetAssets::Request>();
    req->kinds = kinds;
    applyTarget(req);
    return req;
}

// 失败提示：message + 逐项原因（几何校验不通过时 rejected[] 给出 point:P1:in_obstacle 之类）
template <typename Response>
inline QString assetMessage(const Response& r) {
    return QString::fromStdString(r.message);
}
inline QString assetMessage(const nav_interfaces::srv::SetAssets::Response& r) {
    QString s = QString::fromStdString(r.message);
    for (size_t i = 0; i < r.rejected.size() && i < 5; ++i) s += "\n  " + QString::fromStdString(r.rejected[i]);
    if (r.rejected.size() > 5) s += QString("\n  ... %1 more").arg(r.rejected.size() - 5);
    return s;
}

// 工具里每次写入的结果都报给面板：成功给 message，失败给 message + 逐项原因
template <typename Response>
inline void reportResult(const Response& r, const QString& what) {
    AssetTarget::report(QString("[%1] %2：%3").arg(QString::fromStdString(AssetTarget::label()), what,
                                                  r.success ? QString("成功") : assetMessage(r)),
                        r.success);
}

}  // namespace map_asset_rviz_plugin

#endif  // MAP_ASSET_RVIZ_PLUGIN__ASSET_TARGET_HPP_
