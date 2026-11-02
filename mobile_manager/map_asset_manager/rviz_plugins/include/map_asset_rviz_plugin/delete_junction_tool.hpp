#pragma once

#include <memory>
#include <string>
#include <vector>

#include <OgreManualObject.h>
#include <OgreSceneNode.h>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>

#include "map_asset_rviz_plugin/route_graph_tools_common.hpp"

namespace map_asset_rviz_plugin {

/// 删除路网 Junction 工具
/// activate() 加载 Junction 列表；鼠标悬停高亮最近的 Junction；
/// 左键点击 → 确认 → 调用 remove_route_junction 服务；右键退出
class DeleteJunctionTool : public rviz_common::Tool {
    Q_OBJECT

public:
    DeleteJunctionTool();
    ~DeleteJunctionTool() override;

    void onInitialize() override;
    void activate() override;
    void deactivate() override;
    int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;
    void load(const rviz_common::Config& config) override;
    void save(rviz_common::Config config) const override;

private:
    rclcpp::Node::SharedPtr node_;
    std::string service_prefix_{"/map_asset_manager"};
    uint64_t loaded_gen_{0};

    std::vector<EndpointInfo> junctions_;   ///< 当前已加载的 Junction 列表
    const EndpointInfo*       hovered_{nullptr};  ///< 当前高亮的 Junction

    Ogre::ManualObject* preview_obj_{nullptr};
    Ogre::SceneNode*    preview_node_{nullptr};

    void reloadJunctions();
    void updateHighlight(double mx, double my);
    void clearPreview();
    void doDelete(const EndpointInfo& ep);
};

}  // namespace map_asset_rviz_plugin
