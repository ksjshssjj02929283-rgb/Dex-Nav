#ifndef MAP_ASSET_RVIZ_PLUGIN__ADD_ROOM_TOOL_HPP_
#define MAP_ASSET_RVIZ_PLUGIN__ADD_ROOM_TOOL_HPP_

#include <memory>
#include <string>
#include <vector>

#include <OgreManualObject.h>
#include <OgreSceneNode.h>

#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_interfaces/srv/add_room.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>

namespace map_asset_rviz_plugin {

// 创建房间流程（默认就是多边形）：
//   1. 激活工具时弹窗：输入名称、选择类型、可选填语义标签 / 关联 NavPoint（多个，逗号分隔）
//   2. 左键单击：添加顶点（预览线随类型变色）
//   3. 右键单击：完成绘制，调用 add_room 服务
class AddRoomTool : public rviz_common::Tool {
    Q_OBJECT

public:
    AddRoomTool();
    ~AddRoomTool() override;

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
    rclcpp::Client<nav_interfaces::srv::AddRoom>::SharedPtr client_;
    std::string service_name_{"/map_asset_manager/add_room"};

    // 当前房间的预设属性（来自预弹窗）
    std::string room_name_;
    uint32_t    room_type_{1};      // 默认 MEETING_ROOM
    std::string room_semantic_label_;
    std::string room_display_label_;
    std::vector<std::string> room_anchor_point_names_;

    // 区域属性（RoomBehavior，方案 D 内嵌约束子集；0 / false = 不启用）
    float       behavior_max_speed_m_s_{0.0f};
    bool        behavior_restricted_{false};
    bool        behavior_detour_enabled_{false};
    float       behavior_stop_distance_m_{0.0f};
    float       behavior_depth_min_height_m_{0.0f};
    std::vector<std::string> behavior_additional_area_names_;

    Color preview_color_{0.2f, 0.55f, 0.95f};  // 默认蓝（MEETING_ROOM 配色）

    std::vector<geometry_msgs::msg::Pose2D> vertices_;

    // Ogre 预览线
    Ogre::ManualObject* preview_line_{nullptr};
    Ogre::SceneNode*    preview_node_{nullptr};

    bool cancelled_{false};
};

}  // namespace map_asset_rviz_plugin

#endif  // MAP_ASSET_RVIZ_PLUGIN__ADD_ROOM_TOOL_HPP_
