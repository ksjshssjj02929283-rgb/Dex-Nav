#include "map_manager_rviz_plugin/map_edit_tool.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/viewport_mouse_event.hpp>

#include "map_manager_rviz_plugin/grid_edit_draft.hpp"

namespace map_manager_rviz_plugin {

MapEditTool::MapEditTool() {
    shortcut_key_ = 'e';
}

void MapEditTool::onInitialize() {
    projection_finder_ = std::make_shared<rviz_rendering::ViewportProjectionFinder>();
    setName("Map Edit");
}

void MapEditTool::activate() {
    if (!GridEditDraft::instance().editing) {
        setStatus("请先在 Map Manager 面板「底图编辑」页点「开始编辑」");
    } else {
        setStatus("按住左键拖动绘制，松开结束一笔；右键退出");
    }
}

void MapEditTool::deactivate() {
    GridEditDraft::instance().endStroke();
}

int MapEditTool::processMouseEvent(rviz_common::ViewportMouseEvent& event) {
    auto& draft = GridEditDraft::instance();
    if (event.rightDown()) {
        draft.endStroke();
        return Render | Finished;
    }
    if (!draft.editing) return 0;

    const auto hit = projection_finder_->getViewportPointProjectionOnXYPlane(
        event.panel->getRenderWindow(), event.x, event.y);
    if (!hit.first) return 0;
    const double x = hit.second.x, y = hit.second.y;

    if (event.leftDown()) {
        draft.beginStroke(x, y);
        return Render;
    }
    if (event.leftUp()) {
        draft.moveStroke(x, y, event.shift());
        draft.endStroke();
        setStatus(QString("待保存 %1 笔").arg(draft.strokes().size()));
        return Render;
    }
    if (event.type == QEvent::MouseMove && event.left() && draft.hasActive()) {
        draft.moveStroke(x, y, event.shift());
        return Render;
    }
    return 0;
}

}  // namespace map_manager_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(map_manager_rviz_plugin::MapEditTool, rviz_common::Tool)
