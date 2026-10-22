#ifndef MAP_MANAGER_RVIZ_PLUGIN__MAP_EDIT_TOOL_HPP_
#define MAP_MANAGER_RVIZ_PLUGIN__MAP_EDIT_TOOL_HPP_

#include <memory>

#include <rviz_common/tool.hpp>
#include <rviz_rendering/viewport_projection_finder.hpp>

namespace map_manager_rviz_plugin {

// 底图编辑画笔：按住左键拖动画一笔（自由 / 直线 / 矩形由面板设定，直线按住 Shift 吸附 45°），
// 松开即结束这一笔；右键退出工具。笔画只进草稿，由 Map Manager 面板统一保存。
class MapEditTool : public rviz_common::Tool {
    Q_OBJECT

public:
    MapEditTool();
    ~MapEditTool() override = default;

    void onInitialize() override;
    void activate() override;
    void deactivate() override;
    int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;

private:
    std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;
};

}  // namespace map_manager_rviz_plugin

#endif  // MAP_MANAGER_RVIZ_PLUGIN__MAP_EDIT_TOOL_HPP_
