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

/// 直线路网边工具（快捷键 L）— 对标 line_tool 交互：
///   左键选起点（snap）→ selecting_=true，鼠标移动实时预览连线
///   右键完成：snap 到最近端点为终点 → 弹出配置框 → 提交
///   右键（未选起点时）→ 退出工具
class AddEdgeTool : public rviz_common::Tool {
    Q_OBJECT

public:
    AddEdgeTool();
    ~AddEdgeTool() override;

    void onInitialize() override;
    void activate() override;
    void deactivate() override;
    int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;
    void load(const rviz_common::Config& config) override;
    void save(rviz_common::Config config) const override;

private:
    void updatePreview(double mx, double my);
    void clearPreview();
    void submitEdge(const EndpointInfo& from, const EndpointInfo& to,
                    const EdgeConfig& cfg);

    rclcpp::Node::SharedPtr      node_;
    std::string                  service_prefix_{"/map_asset_manager"};
    std::vector<EndpointInfo>    endpoints_;
    bool                         loading_{false};  ///< 正在异步加载端点列表
    double                       last_mx_{0.0};    ///< 最后鼠标位置，加载完成后刷新预览用
    double                       last_my_{0.0};

    bool         selecting_{false};
    EndpointInfo from_ep_;

    Ogre::ManualObject* preview_obj_{nullptr};
    Ogre::SceneNode*    preview_node_{nullptr};
};

}  // namespace map_asset_rviz_plugin

