#ifndef MAP_ASSET_RVIZ_PLUGIN__DELETE_EDGE_TOOL_HPP_
#define MAP_ASSET_RVIZ_PLUGIN__DELETE_EDGE_TOOL_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>
#include <nav_interfaces/srv/get_route_graph.hpp>
#include <nav_interfaces/srv/set_assets.hpp>

namespace map_asset_rviz_plugin {

// 删除路网边工具：点击地图弹出列表选择要删除的边
class DeleteEdgeTool : public rviz_common::Tool {
    Q_OBJECT

public:
    DeleteEdgeTool();
    ~DeleteEdgeTool() override = default;

    void onInitialize() override;
    void activate() override;
    void deactivate() override;
    int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;
    void load(const rviz_common::Config& config) override;
    void save(rviz_common::Config config) const override;

private:
    rclcpp::Node::SharedPtr node_;
    std::string service_prefix_{"/map_asset_manager"};

    rclcpp::Client<nav_interfaces::srv::GetRouteGraph>::SharedPtr   get_graph_client_;
    rclcpp::Client<nav_interfaces::srv::SetAssets>::SharedPtr remove_edge_client_;
};

}  // namespace map_asset_rviz_plugin

#endif  // MAP_ASSET_RVIZ_PLUGIN__DELETE_EDGE_TOOL_HPP_
