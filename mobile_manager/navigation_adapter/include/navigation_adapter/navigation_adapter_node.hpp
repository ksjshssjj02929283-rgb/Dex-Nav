#ifndef NAVIGATION_ADAPTER__NAVIGATION_ADAPTER_NODE_HPP_
#define NAVIGATION_ADAPTER__NAVIGATION_ADAPTER_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "navigation_adapter/area_filter_adapter.hpp"
#include "navigation_adapter/asset_navigation_adapter.hpp"

namespace navigation_adapter {

class NavigationAdapterNode : public rclcpp::Node {
public:
    NavigationAdapterNode();

private:
    void initialize();

    // ── 子模块 ───────────────────────────────────────────────────
    std::shared_ptr<AreaFilterAdapter> area_filter_adapter_;
    std::shared_ptr<AssetNavigationAdapter> asset_navigation_adapter_;
};

}  // namespace navigation_adapter

#endif  // NAVIGATION_ADAPTER__NAVIGATION_ADAPTER_NODE_HPP_
