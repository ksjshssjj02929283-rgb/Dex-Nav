#include "map_asset_manager/data_event_bus.hpp"

namespace map_asset_manager {

void DataEventBus::subscribe(AssetEventType type, Handler handler) {
    std::lock_guard<std::mutex> lock(mutex_);
    handlers_[static_cast<int>(type)].push_back(std::move(handler));
}

void DataEventBus::publish(const AssetEvent& event) {
    // 取一份快照，避免回调内部订阅时死锁
    std::vector<Handler> snapshot;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = handlers_.find(static_cast<int>(event.type));
        if (it != handlers_.end()) {
            snapshot = it->second;
        }
    }
    for (auto& handler : snapshot) {
        handler(event);
    }
}

}  // namespace map_asset_manager
