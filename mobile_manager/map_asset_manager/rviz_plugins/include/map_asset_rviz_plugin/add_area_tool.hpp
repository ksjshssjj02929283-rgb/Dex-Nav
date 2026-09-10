#ifndef MAP_ASSET_RVIZ_PLUGIN__ADD_AREA_TOOL_HPP_
#define MAP_ASSET_RVIZ_PLUGIN__ADD_AREA_TOOL_HPP_

#include <memory>
#include <string>
#include <vector>

#include <OgreManualObject.h>
#include <OgreSceneNode.h>

#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_interfaces/srv/set_area.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>

namespace map_asset_rviz_plugin {

// 创建区域流程：
//   1. 激活工具时弹窗：输入名称、选择类型及类型属性
//   2. 左键单击：添加顶点（预览线随类型变色）
//   3. 右键单击：完成绘制，调用 add_area 服务
class AddAreaTool : public rviz_common::Tool {
    Q_OBJECT

public:
    AddAreaTool();
    ~AddAreaTool() override;

    void onInitialize() override;
    void activate() override;
    void deactivate() override;

    int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;

    void load(const rviz_common::Config& config) override;
    void save(rviz_common::Config config) const override;

private:
    struct Color { float r, g, b; };

    bool showPreDialog();          // 激活时弹窗，返回 false 表示用户取消
    void finishPolygon();          // 右键触发，调用服务
    void resetPolygon();
    void updatePreviewLine();
    static Color typeColor(uint32_t type);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<nav_interfaces::srv::SetArea>::SharedPtr client_;
    std::string service_name_{"/map_asset_manager/set_area"};

    // 当前区域的预设属性（来自预弹窗）
    std::string area_name_;
    uint32_t    area_type_{1};
    float       area_max_speed_{0.5f};
    float       area_stop_distance_{0.5f};
    std::string area_linked_point_;
    std::string area_charger_type_;
    float       area_depth_min_height_{0.1f};

    Color preview_color_{1.0f, 0.0f, 0.0f};

    std::vector<geometry_msgs::msg::Pose2D> vertices_;

    // Ogre 预览线
    Ogre::ManualObject* preview_line_{nullptr};
    Ogre::SceneNode*    preview_node_{nullptr};

    bool cancelled_{false};
};

}  // namespace map_asset_rviz_plugin

#endif  // MAP_ASSET_RVIZ_PLUGIN__ADD_AREA_TOOL_HPP_
