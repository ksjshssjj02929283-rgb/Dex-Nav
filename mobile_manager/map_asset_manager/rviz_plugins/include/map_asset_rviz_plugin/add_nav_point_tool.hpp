#ifndef MAP_ASSET_RVIZ_PLUGIN__ADD_NAV_POINT_TOOL_HPP_
#define MAP_ASSET_RVIZ_PLUGIN__ADD_NAV_POINT_TOOL_HPP_

#include <memory>
#include <string>

#include <OgreManualObject.h>
#include <OgreSceneNode.h>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>

namespace map_asset_rviz_plugin {

// 拖拽式添加导航点工具（仿 Nav2Goal）：
// 左键按下 = 确定位置，拖动 = 确定朝向（箭头预览），
// 左键释放 = 通过 NavPointDraftBridge 发送草稿到 NavPointTab 填入编辑框。
class AddNavPointTool : public rviz_common::Tool {
    Q_OBJECT

public:
    AddNavPointTool();
    ~AddNavPointTool() override;

    void onInitialize() override;
    void activate() override;
    void deactivate() override;

    int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;

    void load(const rviz_common::Config& config) override;
    void save(rviz_common::Config config) const override;

private:
    void updateArrowPreview(double x0, double y0, double x1, double y1);
    void clearArrowPreview();

    rclcpp::Node::SharedPtr node_;
    std::string service_name_{"/map_asset_manager/add_point"};

    // 拖拽状态
    bool   dragging_{false};
    double start_x_{0.0};
    double start_y_{0.0};

    // Ogre 箭头预览
    Ogre::ManualObject* arrow_obj_{nullptr};
    Ogre::SceneNode*    arrow_node_{nullptr};
};

}  // namespace map_asset_rviz_plugin

#endif  // MAP_ASSET_RVIZ_PLUGIN__ADD_NAV_POINT_TOOL_HPP_
