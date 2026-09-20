#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <OgreManualObject.h>
#include <OgreSceneNode.h>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>

#include "map_asset_rviz_plugin/route_graph_tools_common.hpp"

namespace map_asset_rviz_plugin {

/// 路网删除工具（快捷键 D）— 对标 del_tool 交互：
///   左键点击：高亮最近 Junction 或 Edge（同 del_tool 的 is_stop=false）
///   Del 键：确认删除高亮项（同 del_tool 的 is_stop=true）
///   右键：退出工具
///
/// 可删除：
///   - Junction（snap 到节点圆心，半径 kSnapRadius）
///   - RouteEdge（snap 到线段，distance-to-segment < kSnapRadius）
class RouteGraphDeleteTool : public rviz_common::Tool {
    Q_OBJECT

public:
    RouteGraphDeleteTool();
    ~RouteGraphDeleteTool() override;

    void onInitialize() override;
    void activate() override;
    void deactivate() override;
    int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;
    int processKeyEvent(QKeyEvent* event, rviz_common::RenderPanel* panel) override;
    void load(const rviz_common::Config& config) override;
    void save(rviz_common::Config config) const override;

private:
    struct EdgeInfo {
        std::string id;
        float x0{0.0f}, y0{0.0f};  ///< from_ep 坐标
        float x1{0.0f}, y1{0.0f};  ///< to_ep 坐标
    };

    enum class SelType { NONE, JUNCTION, EDGE };

    void reloadGraph();
    void refreshPreview();  ///< 加载完成后立即绘制所有元素（无高亮）
    void updateHighlight(double mx, double my);
    void clearPreview();
    void doDelete();
    float distToSegment(float px, float py,
                        float x0, float y0, float x1, float y1) const;

    rclcpp::Node::SharedPtr      node_;
    std::string                  service_prefix_{"/map_asset_manager"};

    std::mutex                   data_mutex_;  ///< 保护 junctions_/edges_ 的多线程访问
    std::vector<EndpointInfo>    junctions_;
    std::vector<EdgeInfo>        edges_;
    // 端点位置映射（id → x,y）用于边 snap
    std::vector<EndpointInfo>    all_eps_;

    SelType      sel_type_{SelType::NONE};
    std::string  sel_id_;        ///< 当前选中项 ID
    float        sel_x_{0.0f};
    float        sel_y_{0.0f};
    double       last_mx_{0.0};  ///< 最后一次鼠标 3D 坐标（用于 reload 后刷新预览）
    double       last_my_{0.0};

    Ogre::ManualObject* preview_obj_{nullptr};
    Ogre::SceneNode*    preview_node_{nullptr};
};

}  // namespace map_asset_rviz_plugin
