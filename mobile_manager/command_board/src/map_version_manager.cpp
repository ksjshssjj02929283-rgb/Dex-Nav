#include "map_version_manager.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <vector>

namespace fs = std::filesystem;

namespace command_board {

namespace {

// 取 ring 目录中匹配 <prefix>*<suffix> 的最新条目，返回中间 ts（字典序=时间序）。空=无。
std::string LatestRing(const fs::path& dir, const std::string& prefix, const std::string& suffix) {
    std::error_code ec;
    if (!fs::exists(dir, ec)) return "";
    std::vector<std::string> names;
    for (const auto& e : fs::directory_iterator(dir, ec)) {
        const std::string n = e.path().filename().string();
        const bool pre = n.rfind(prefix, 0) == 0;
        const bool suf = suffix.empty() ||
            (n.size() >= suffix.size() &&
             n.compare(n.size() - suffix.size(), suffix.size(), suffix) == 0);
        if (pre && suf && n.size() > prefix.size() + suffix.size()) names.push_back(n);
    }
    if (names.empty()) return "";
    std::sort(names.begin(), names.end());
    const std::string& best = names.back();
    return best.substr(prefix.size(), best.size() - prefix.size() - suffix.size());
}

template <typename SrvT>
typename SrvT::Response::SharedPtr CallService(
    const std::string& name, typename SrvT::Request::SharedPtr req,
    double timeout_s, std::string& err) {
    auto tmp = std::make_shared<rclcpp::Node>(
        "cb_mvm_tmp_client",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(false));
    auto cli = tmp->create_client<SrvT>(name);
    if (!cli->wait_for_service(std::chrono::seconds(3))) {
        err = "service not available: " + name;
        return nullptr;
    }
    auto fut = cli->async_send_request(req);
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(tmp);
    if (exec.spin_until_future_complete(
            fut, std::chrono::duration_cast<std::chrono::milliseconds>(
                     std::chrono::duration<double>(timeout_s)))
        != rclcpp::FutureReturnCode::SUCCESS) {
        err = "timeout/failed calling " + name;
        return nullptr;
    }
    return fut.get();
}

}  // namespace

MapVersionManager::MapVersionManager(rclcpp::Node* node) : node_(node) {
    cbg_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    auto qos = rclcpp::QoS(1).transient_local().reliable();
    map_state_sub_ = node_->create_subscription<nav_interfaces::msg::MapState>(
        "map_manager/map_state", qos,
        std::bind(&MapVersionManager::OnMapState, this, std::placeholders::_1));

    backup_unified_srv_ = node_->create_service<std_srvs::srv::Trigger>(
        "~/backup_unified",
        std::bind(&MapVersionManager::OnBackupUnified, this,
                  std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, cbg_);
    rollback_map_srv_ = node_->create_service<nav_interfaces::srv::RollbackMap>(
        "~/rollback_map",
        std::bind(&MapVersionManager::OnRollbackMap, this,
                  std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, cbg_);

    RCLCPP_INFO(node_->get_logger(),
                "MapVersionManager ready: ~/backup_unified, ~/rollback_map "
                "(any map; base map restored via map_manager map_command OP_ROLLBACK).");
}

void MapVersionManager::OnMapState(const nav_interfaces::msg::MapState::SharedPtr msg) {
    // folder+name 经 map_state 原子到达；本管理器只需当前路径（MapDir）
    if (!msg->map_folder.empty()) map_folder_ = msg->map_folder;
    if (!msg->map_name.empty()) map_name_ = msg->map_name;
}

std::string MapVersionManager::MapDir() const {
    if (map_folder_.empty() || map_name_.empty()) return "";
    return map_folder_ + "/" + map_name_;
}

bool MapVersionManager::CallSaveMap(std::string& out_ts, std::string& out_msg) {
    auto req = std::make_shared<nav_interfaces::srv::SaveMap::Request>();
    req->map_name = "";
    auto resp = CallService<nav_interfaces::srv::SaveMap>(
        "/localization/save_map", req, service_timeout_sec_, out_msg);
    if (!resp) return false;
    if (!resp->success) { out_msg = resp->message; return false; }
    out_ts = resp->backup_ts;
    return true;
}

bool MapVersionManager::CallSaveAssets(std::string& out_ts, std::string& out_msg) {
    auto req = std::make_shared<nav_interfaces::srv::SaveAssets::Request>();
    req->map_name = "";
    auto resp = CallService<nav_interfaces::srv::SaveAssets>(
        "/map_asset_manager/save_assets", req, service_timeout_sec_, out_msg);
    if (!resp) return false;
    if (!resp->success) { out_msg = resp->message; return false; }
    out_ts = resp->backup_ts;
    return true;
}

bool MapVersionManager::TriggerRedeploy(std::string& out_msg) {
    auto req = std::make_shared<nav_interfaces::srv::MapManagerCommand::Request>();
    req->op = nav_interfaces::srv::MapManagerCommand::Request::OP_LOAD;
    req->map_name = map_name_;
    auto resp = CallService<nav_interfaces::srv::MapManagerCommand>(
        "map_manager/map_command", req, service_timeout_sec_, out_msg);
    if (!resp) return false;
    if (!resp->success) { out_msg = "map_command(LOAD) failed: " + resp->message; return false; }
    return true;
}

void MapVersionManager::OnBackupUnified(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    const std::string mapdir = MapDir();
    if (mapdir.empty()) {
        res->success = false;
        res->message = "map not ready (waiting for map_manager map_folder/map_name)";
        return;
    }
    std::string map_ts, assets_ts, msg;
    if (!CallSaveMap(map_ts, msg)) {
        res->success = false; res->message = "save_map failed: " + msg; return;
    }
    if (!CallSaveAssets(assets_ts, msg)) {
        res->success = false; res->message = "save_assets failed: " + msg; return;
    }
    res->success = true;
    res->message = "backed up (map=" + map_ts + ", assets=" + assets_ts + ")";
    RCLCPP_INFO(node_->get_logger(), "[MapVersionManager] backup_unified: %s", res->message.c_str());
}

void MapVersionManager::OnRollbackMap(
    const std::shared_ptr<nav_interfaces::srv::RollbackMap::Request> req,
    std::shared_ptr<nav_interfaces::srv::RollbackMap::Response> res) {
    // 回滚任意一张地图的底图 + 资产（编排层）：
    //   - 底图委托 map_manager map_command OP_ROLLBACK（文件层，回滚前自动快照）
    //   - 资产委托 map_asset_manager rollback_assets（同样先存回滚前版本；当前地图由它自己重载内存）
    //   - 目标是当前地图时再让 map_manager 重新下发，定位 / 资产 / 导航按 epoch 重载；非当前地图只动文件
    // 版本号：空 = 该 ring 最新；"none" = 该部分不回退。
    const std::string target = req->map_name.empty() ? map_name_ : req->map_name;
    if (target.find('/') != std::string::npos || target == "." || target == "..") {
        res->success = false;
        res->message = "invalid map_name: " + target;
        return;
    }
    const bool is_current = (target == map_name_);
    const std::string mapdir = (map_folder_.empty() || target.empty()) ? std::string() : map_folder_ + "/" + target;
    if (mapdir.empty()) {
        res->success = false;
        res->message = "map not ready (waiting for map_manager map_folder/map_name)";
        return;
    }
    {
        std::error_code ec;
        if (!fs::is_directory(fs::path(mapdir) / "slam", ec)) {
            res->success = false;
            res->message = "map not found: " + target;
            return;
        }
    }
    const fs::path slam_b = fs::path(mapdir) / "slam" / "backups";
    const fs::path asset_b = fs::path(mapdir) / "assets" / "backups";

    const bool skip_map = (req->map_version_ts == "none");
    const bool skip_assets = (req->assets_version_ts == "none");
    // 「空 = 最新」在这里先解析成具体版本：底图回滚会先存一份快照，之后「最新」就变了
    std::string map_ts = skip_map ? std::string() : req->map_version_ts;
    std::string assets_ts = skip_assets ? std::string() : req->assets_version_ts;
    if (!skip_map && map_ts.empty())       map_ts = LatestRing(slam_b, "backup.", "");
    if (!skip_assets && assets_ts.empty()) assets_ts = LatestRing(asset_b, "map_assets.", ".json");
    if (map_ts.empty() && assets_ts.empty()) {
        res->success = false;
        res->message = "nothing to roll back (no version found, or both parts are \"none\")";
        return;
    }

    // 资产版本先确认存在，避免底图已回滚、资产却找不到版本的半截状态
    if (!assets_ts.empty()) {
        const fs::path assets_src = asset_b / ("map_assets." + assets_ts + ".json");
        std::error_code ec;
        if (assets_ts.find('/') != std::string::npos || !fs::exists(assets_src, ec)) {
            res->success = false; res->message = "assets version not found: " + assets_src.string(); return;
        }
    }

    std::string map_note;
    if (!map_ts.empty()) {
        auto mreq = std::make_shared<nav_interfaces::srv::MapManagerCommand::Request>();
        mreq->op = nav_interfaces::srv::MapManagerCommand::Request::OP_ROLLBACK;
        mreq->map_name = target;
        mreq->version_ts = map_ts;
        std::string err;
        auto mres = CallService<nav_interfaces::srv::MapManagerCommand>(
            "map_manager/map_command", mreq, service_timeout_sec_, err);
        if (!mres) { res->success = false; res->message = "restore map: " + err; return; }
        if (!mres->success) {
            res->success = false;
            res->message = "restore map failed (" + std::to_string(mres->error_code) + "): " + mres->message;
            return;
        }
        if (!mres->backup_ts.empty()) map_note = ", previous base map saved as " + mres->backup_ts;
    }
    std::string assets_note;
    if (!assets_ts.empty()) {
        auto areq = std::make_shared<nav_interfaces::srv::RollbackAssets::Request>();
        areq->map_name = target;
        areq->version_ts = assets_ts;
        std::string err;
        auto ares = CallService<nav_interfaces::srv::RollbackAssets>(
            "/map_asset_manager/rollback_assets", areq, service_timeout_sec_, err);
        if (!ares || !ares->success) {
            res->success = false;
            res->message = "restore assets failed: " + (ares ? ares->message : err) + map_note;
            return;
        }
        if (!ares->backup_ts.empty()) assets_note = ", previous assets saved as " + ares->backup_ts;
    }

    // 资产节点回滚时已自行重载当前地图的资产；只有底图变了才需要整图重新下发
    std::string redeploy_note = is_current ? "assets reloaded by map_asset_manager" : "files restored only (not the current map)";
    if (is_current && !map_ts.empty()) {
        std::string redeploy_msg;
        const bool re_ok = TriggerRedeploy(redeploy_msg);
        redeploy_note = std::string("redeploy ") + (re_ok ? "ok" : ("FAILED: " + redeploy_msg));
    }
    res->success = true;
    res->map_version_ts = map_ts;
    res->assets_version_ts = assets_ts;
    res->message = "rolled back " + target + " (map=" + (map_ts.empty() ? std::string("-") : map_ts) +
                   ", assets=" + (assets_ts.empty() ? std::string("-") : assets_ts) + ")" + map_note + assets_note +
                   "; " + redeploy_note;
    RCLCPP_WARN(node_->get_logger(), "[MapVersionManager] rollback_map: %s", res->message.c_str());
}

}  // namespace command_board
