#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <OgreManualObject.h>
#include <OgreSceneNode.h>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>

#include "map_asset_rviz_plugin/route_graph_tools_common.hpp"

namespace map_asset_rviz_plugin {

/// 贝塞尔曲线路网边工具（快捷键 B）— 对标 curve_line_tool 交互：
///   左键选起点（snap）→ selecting_=true
///   后续左键添加自由控制点（不 snap）
///   右键完成：snap 当前鼠标到最近端点 → 弹出配置框 → 提交
///   右键（未选起点时）→ 退出工具
class AddCurveEdgeTool : public rviz_common::Tool {
    Q_OBJECT

public:
    AddCurveEdgeTool();
    ~AddCurveEdgeTool() override;

    void onInitialize() override;
    void activate() override;
    void deactivate() override;
    int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;
    void load(const rviz_common::Config& config) override;
    void save(rviz_common::Config config) const override;

private:
    void updatePreview(double mx, double my);
    void clearPreview();
    void reloadEndpoints();  // 按编辑目标异步加载端点（点位 + 路网节点），记下当时的数据代数
    void submitEdge(const EdgeConfig& cfg);

    rclcpp::Node::SharedPtr              node_;
    std::string                          service_prefix_{"/map_asset_manager"};
    std::vector<EndpointInfo>            endpoints_;
    bool                                 loading_{false};  ///< 正在异步加载端点列表
    uint64_t loaded_gen_{0};
    double                               last_mx_{0.0};
    double                               last_my_{0.0};

    bool         selecting_{false};
    EndpointInfo from_ep_;
    EndpointInfo to_ep_;   ///< snap 到的终点（右键完成时确定）
    std::vector<std::pair<float, float>> ctrl_pts_;  ///< 中间自由控制点

    Ogre::ManualObject* preview_obj_{nullptr};
    Ogre::SceneNode*    preview_node_{nullptr};
};

}  // namespace map_asset_rviz_plugin
