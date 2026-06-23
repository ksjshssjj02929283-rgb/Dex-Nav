// command_board / MapVersionManager
//
// 地图统一版本管理（Model C：DEC-006/007/008/009）。移植自 nav2_command_board 同名模块，
// 因 nav2_command_board 尚未测试，先在旧 command_board（在用）落地这套逻辑。
//
//   - ~/backup_unified（Trigger）：触发 localization.save_map + map_asset_manager.save_assets
//       → 各自产引擎本地 ring 版本 → 写 <map>/versions/<unified_ts>.json manifest（只记两版本号，不拷数据）。
//       提交点 = manifest 写成功；任一 save 失败则不写。
//   - ~/rollback_map(map_version_ts, assets_version_ts)：回退唯一入口（引擎不提供 rollback，DEC-009）。
//       各自可独立指定 = 任意组合；皆空 = 各取对应 ring 最新。直接从
//       <map>/slam/backups/backup.<map_ts>/ + <map>/assets/backups/map_assets.<assets_ts>.json
//       原子恢复（tmp+rename）到当前 → 触发 map_manager 重下发让运行节点重载。

#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_interfaces/srv/save_map.hpp>
#include <nav_interfaces/srv/save_assets.hpp>
#include <nav_interfaces/srv/load_map.hpp>
#include <nav_interfaces/srv/rollback_map.hpp>

namespace command_board {

class MapVersionManager {
public:
    explicit MapVersionManager(rclcpp::Node* node);

private:
    void OnMapFolder(const std_msgs::msg::String::SharedPtr msg);
    void OnMapName(const std_msgs::msg::String::SharedPtr msg);
    std::string MapDir() const;  // <map_folder>/<map_name>；任一为空返回 ""

    void OnBackupUnified(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void OnRollbackMap(const std::shared_ptr<nav_interfaces::srv::RollbackMap::Request> req,
                       std::shared_ptr<nav_interfaces::srv::RollbackMap::Response> res);

    // temp node + 独立 executor，避免 service 回调内调 service 死锁
    bool CallSaveMap(std::string& out_ts, std::string& out_msg);
    bool CallSaveAssets(std::string& out_ts, std::string& out_msg);
    bool TriggerRedeploy(std::string& out_msg);  // map_manager LoadMap(当前图) 重下发

    rclcpp::Node* node_;
    std::string map_folder_;
    std::string map_name_;

    rclcpp::CallbackGroup::SharedPtr cbg_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_folder_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_name_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr backup_unified_srv_;
    rclcpp::Service<nav_interfaces::srv::RollbackMap>::SharedPtr rollback_map_srv_;

    double service_timeout_sec_ = 30.0;
};

}  // namespace command_board
