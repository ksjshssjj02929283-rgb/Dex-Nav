// RouteGraphStartTool / RouteGraphSaveTool
// 对标 start.cpp / save.cpp：activate() 立即调用服务，无鼠标交互
// RouteGraphStartTool → s_route_graph_editing = true（开始修改）
// RouteGraphSaveTool  → s_route_graph_editing = false（保存并锁定）

#include "map_asset_rviz_plugin/route_graph_start_save_tool.hpp"
#include "map_asset_rviz_plugin/route_graph_tools_common.hpp"  // s_route_graph_editing

#include <std_srvs/srv/trigger.hpp>
#include <nav_interfaces/srv/save_assets.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/display_context.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace map_asset_rviz_plugin {

// ============================================================
// RouteGraphStartTool
// ============================================================

RouteGraphStartTool::RouteGraphStartTool() {
    shortcut_key_ = 'r';   // [R] Reload / start
}

void RouteGraphStartTool::onInitialize() {
    node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
}

void RouteGraphStartTool::activate() {
    // 先设置编辑标志，再从文件重载数据（对标 create_waypoint::NewWaypointCallBack）
    s_route_graph_editing = true;
    setStatus("正在加载路网数据，进入编辑模式…");

    auto cli = node_->create_client<std_srvs::srv::Trigger>(
        service_prefix_ + "/reload_data");
    if (!cli->wait_for_service(std::chrono::milliseconds(1000))) {
        s_route_graph_editing = false;
        setStatus("reload_data 服务不可用（map_asset_manager 是否在运行？）");
        return;
    }
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    cli->async_send_request(req,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture fut) {
            try {
                auto resp = fut.get();
                if (resp->success) {
                    setStatus(QString("✓ 路网已加载，进入编辑模式（使用完毕请保存）"));
                } else {
                    s_route_graph_editing = false;
                    setStatus(QString("加载失败：%1")
                                  .arg(QString::fromStdString(resp->message)));
                }
            } catch (const std::exception& ex) {
                s_route_graph_editing = false;
                setStatus(QString("异常：%1").arg(ex.what()));
            }
        });
}

// ============================================================
// RouteGraphSaveTool
// ============================================================

RouteGraphSaveTool::RouteGraphSaveTool() {
    shortcut_key_ = 's';   // [S] Save
}

void RouteGraphSaveTool::onInitialize() {
    node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
}

void RouteGraphSaveTool::activate() {
    if (!s_route_graph_editing) {
        setStatus("当前不在编辑模式，无需保存（请先点击「开始修改路网」）");
        return;
    }
    // 先关闭编辑标志，再保存（对标 create_waypoint::SaveWaypointCallBack）
    s_route_graph_editing = false;
    setStatus("正在保存路网到文件…");

    auto cli = node_->create_client<nav_interfaces::srv::SaveAssets>(
        service_prefix_ + "/save_assets");
    if (!cli->wait_for_service(std::chrono::milliseconds(1000))) {
        setStatus("save_assets 服务不可用（map_asset_manager 是否在运行？）");
        return;
    }
    auto req = std::make_shared<nav_interfaces::srv::SaveAssets::Request>();
    cli->async_send_request(req,
        [this](rclcpp::Client<nav_interfaces::srv::SaveAssets>::SharedFuture fut) {
            try {
                auto resp = fut.get();
                setStatus(resp->success
                    ? QString("✓ 路网已保存，退出编辑模式")
                    : QString("保存失败：%1")
                          .arg(QString::fromStdString(resp->message)));
            } catch (const std::exception& ex) {
                setStatus(QString("异常：%1").arg(ex.what()));
            }
        });
}

}  // namespace map_asset_rviz_plugin

PLUGINLIB_EXPORT_CLASS(map_asset_rviz_plugin::RouteGraphStartTool, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(map_asset_rviz_plugin::RouteGraphSaveTool,  rviz_common::Tool)
