#ifndef MAP_ASSET_MANAGER__DATA_EVENT_BUS_HPP_
#define MAP_ASSET_MANAGER__DATA_EVENT_BUS_HPP_

#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace map_asset_manager {

enum class AssetEventType {
    POINT_ADDED,
    POINT_UPDATED,   // 含重命名：old_name 非空时为重命名事件
    POINT_REMOVED,
    ROUTE_ADDED,
    ROUTE_UPDATED,
    ROUTE_REMOVED,
    AREA_ADDED,
    AREA_UPDATED,
    AREA_REMOVED,
    ROOM_ADDED,
    ROOM_UPDATED,
    ROOM_REMOVED,
    JUNCTION_ADDED,    // RouteJunction（路网节点）
    JUNCTION_UPDATED,
    JUNCTION_REMOVED,
    DATA_RELOADED,   // 整体热重载
};

struct AssetEvent {
    AssetEventType type;
    std::string name;      // 操作后的名称
    std::string old_name;  // 重命名时的旧名称（否则为空）
};

// 同进程内轻量事件总线（同步、顺序触发）
// 发布方不感知订阅方，订阅方通过回调响应联动更新
class DataEventBus {
public:
    using Handler = std::function<void(const AssetEvent&)>;

    void subscribe(AssetEventType type, Handler handler);
    void publish(const AssetEvent& event);

private:
    // 使用 int key 避免 enum class hash 问题
    std::unordered_map<int, std::vector<Handler>> handlers_;
    std::mutex mutex_;
};

}  // namespace map_asset_manager

#endif  // MAP_ASSET_MANAGER__DATA_EVENT_BUS_HPP_
