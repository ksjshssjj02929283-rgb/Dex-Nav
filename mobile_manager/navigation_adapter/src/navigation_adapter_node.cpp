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
    // 区域 → costmap filter 掩膜已移到 map_asset_manager（资产节点自己持有区域与底图）
    asset_navigation_adapter_ = std::make_shared<AssetNavigationAdapter>(this);
}

}  // namespace navigation_adapter
