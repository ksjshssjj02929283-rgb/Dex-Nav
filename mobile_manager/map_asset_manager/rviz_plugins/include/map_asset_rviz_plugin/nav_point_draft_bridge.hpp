#ifndef MAP_ASSET_RVIZ_PLUGIN__NAV_POINT_DRAFT_BRIDGE_HPP_
#define MAP_ASSET_RVIZ_PLUGIN__NAV_POINT_DRAFT_BRIDGE_HPP_

#include <QObject>

namespace map_asset_rviz_plugin {

// 工具→面板通信桥：AddNavPointTool 完成拖拽后通过此单例
// 向 NavPointTab 传递草稿位置/朝向，供用户编辑后确认
class NavPointDraftBridge : public QObject {
    Q_OBJECT

public:
    static NavPointDraftBridge* instance();

    // 由 AddNavPointTool 在 leftUp 时调用
    void notifyDraft(double x, double y, double yaw_rad);

Q_SIGNALS:
    void draftReady(double x, double y, double yaw_rad);

private:
    NavPointDraftBridge() = default;
};

}  // namespace map_asset_rviz_plugin

#endif  // MAP_ASSET_RVIZ_PLUGIN__NAV_POINT_DRAFT_BRIDGE_HPP_
