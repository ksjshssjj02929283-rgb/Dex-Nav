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

bool AtomicCopy(const fs::path& src, const fs::path& dst, std::string& err) {
    std::error_code ec;
    if (!fs::exists(src, ec)) { err = "missing source " + src.string(); return false; }
    const fs::path tmp = dst.string() + ".rbtmp";
    fs::copy_file(src, tmp, fs::copy_options::overwrite_existing, ec);
    if (ec) { err = "copy " + src.string() + ": " + ec.message(); return false; }
    fs::rename(tmp, dst, ec);
    if (ec) { fs::remove(tmp, ec); err = "rename " + dst.string() + ": " + ec.message(); return false; }
    return true;
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
                "MapVersionManager ready (Model C/DEC-009): ~/backup_unified, ~/rollback_map. "
                "回退唯一入口；引擎不提供 rollback。");
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
    const std::string mapdir = MapDir();
    if (mapdir.empty()) {
        res->success = false;
        res->message = "map not ready (waiting for map_manager map_folder/map_name)";
        return;
    }
    const fs::path slam_b = fs::path(mapdir) / "slam" / "backups";
    const fs::path asset_b = fs::path(mapdir) / "assets" / "backups";

    std::string map_ts = req->map_version_ts;
    std::string assets_ts = req->assets_version_ts;
    if (map_ts.empty())    map_ts = LatestRing(slam_b, "backup.", "");
    if (assets_ts.empty()) assets_ts = LatestRing(asset_b, "map_assets.", ".json");
    if (map_ts.empty() && assets_ts.empty()) {
        res->success = false;
        res->message = "no version found in either ring (slam/backups, assets/backups)";
        return;
    }

    std::string err;
    if (!map_ts.empty()) {
        const fs::path bdir = slam_b / ("backup." + map_ts);
        std::error_code ec;
        if (!fs::exists(bdir, ec)) {
            res->success = false; res->message = "map version not found: " + bdir.string(); return;
        }
        const fs::path slam = fs::path(mapdir) / "slam";
        for (const char* f : {"map.pbstream", "map.png", "map.yaml"}) {
            if (!AtomicCopy(bdir / f, slam / f, err)) {
                res->success = false; res->message = "restore map: " + err; return;
            }
        }
        if (fs::exists(bdir / ".complete", ec)) {
            AtomicCopy(bdir / ".complete", slam / ".complete", err);  // sentinel 可选
        }
    }
    if (!assets_ts.empty()) {
        const fs::path src = asset_b / ("map_assets." + assets_ts + ".json");
        std::error_code ec;
        if (!fs::exists(src, ec)) {
            res->success = false; res->message = "assets version not found: " + src.string(); return;
        }
        const fs::path dst = fs::path(mapdir) / "assets" / "map_assets.json";
        if (!AtomicCopy(src, dst, err)) {
            res->success = false; res->message = "restore assets: " + err; return;
        }
    }
    std::string redeploy_msg;
    const bool re_ok = TriggerRedeploy(redeploy_msg);

    res->success = true;
    res->map_version_ts = map_ts;
    res->assets_version_ts = assets_ts;
    res->message = "rolled back (map=" + (map_ts.empty() ? std::string("-") : map_ts) +
                   ", assets=" + (assets_ts.empty() ? std::string("-") : assets_ts) +
                   "); redeploy " + (re_ok ? "ok" : ("FAILED: " + redeploy_msg));
    RCLCPP_WARN(node_->get_logger(), "[MapVersionManager] rollback_map: %s", res->message.c_str());
}

}  // namespace command_board
