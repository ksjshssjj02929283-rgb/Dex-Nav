#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>
#include <string>

namespace map_asset_rviz_plugin {

/// 路网开始编辑工具（对标 start/LoadWay）
/// activate() 立即调用 ~/reload_data 服务，从文件重新加载路网（丢弃内存中的未保存修改）
class RouteGraphStartTool : public rviz_common::Tool {
    Q_OBJECT
public:
    RouteGraphStartTool();
    void onInitialize() override;
    void activate() override;
    void deactivate() override {}
    int processMouseEvent(rviz_common::ViewportMouseEvent&) override { return 0; }
    void load(const rviz_common::Config& c) override { Tool::load(c); }
    void save(rviz_common::Config c) const override { Tool::save(c); }

private:
    rclcpp::Node::SharedPtr node_;
    std::string service_prefix_{"/map_asset_manager"};
};

/// 路网保存工具（对标 save/SaveWay）
/// activate() 立即调用 ~/save_assets 服务，将内存中的路网持久化到文件
class RouteGraphSaveTool : public rviz_common::Tool {
    Q_OBJECT
public:
    RouteGraphSaveTool();
    void onInitialize() override;
    void activate() override;
    void deactivate() override {}
    int processMouseEvent(rviz_common::ViewportMouseEvent&) override { return 0; }
    void load(const rviz_common::Config& c) override { Tool::load(c); }
    void save(rviz_common::Config c) const override { Tool::save(c); }

private:
    rclcpp::Node::SharedPtr node_;
    std::string service_prefix_{"/map_asset_manager"};
};

}  // namespace map_asset_rviz_plugin
