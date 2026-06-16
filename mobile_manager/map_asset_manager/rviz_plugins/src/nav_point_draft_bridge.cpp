#include "map_asset_rviz_plugin/nav_point_draft_bridge.hpp"

namespace map_asset_rviz_plugin {

NavPointDraftBridge* NavPointDraftBridge::instance() {
    static NavPointDraftBridge inst;
    return &inst;
}

void NavPointDraftBridge::notifyDraft(double x, double y, double yaw_rad) { Q_EMIT draftReady(x, y, yaw_rad); }

}  // namespace map_asset_rviz_plugin
