#pragma once

#include <memory>
#include <string>

#include <OgreManualObject.h>
#include <OgreSceneNode.h>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>

namespace map_asset_rviz_plugin {

/// 路网 Junction 放置工具
/// 左键点击地图放置 Junction（无需输入朝向），右键/Esc 退出
/// 工具激活后保持活动，可连续放置多个 Junction
class AddJunctionTool : public rviz_common::Tool {
    Q_OBJECT

public:
    AddJunctionTool();
    ~AddJunctionTool() override;

    void onInitialize() override;
    void activate() override;
    void deactivate() override;
    int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;
    void load(const rviz_common::Config& config) override;
    void save(rviz_common::Config config) const override;

private:
    rclcpp::Node::SharedPtr node_;
    std::string service_prefix_{"/map_asset_manager"};
    int junction_counter_{0};  ///< auto-ID 计数，用于建议默认名

    Ogre::ManualObject* preview_obj_{nullptr};
    Ogre::SceneNode*    preview_node_{nullptr};

    void updatePreview(double x, double y);
    void clearPreview();
};

}  // namespace map_asset_rviz_plugin
