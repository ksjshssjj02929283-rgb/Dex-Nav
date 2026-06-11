#include "navigation_adapter/navigation_adapter_node.hpp"

namespace navigation_adapter {

// ──────────────────────────────────────────────────────────────────────
// 构造与初始化
// ──────────────────────────────────────────────────────────────────────

NavigationAdapterNode::NavigationAdapterNode() : rclcpp::Node("navigation_adapter_node") {
    initialize();
    RCLCPP_INFO(get_logger(), "NavigationAdapterNode initialized");
}

void NavigationAdapterNode::initialize() {
    // 两个模块共享同一个 ROS node，对外只暴露 /navigation_adapter_node。
    area_filter_adapter_ = std::make_shared<AreaFilterAdapter>(this);
    asset_navigation_adapter_ = std::make_shared<AssetNavigationAdapter>(this);
}

}  // namespace navigation_adapter
