// ----------------------------------------------------------------------------
//
// Copyright (c) 2021--2026 DexForce Technology Co., Ltd.
//
// All Rights Reserved.
//
//    Authors: Li Yao <liyao2@dexforce.top>
// ----------------------------------------------------------------------------

#include "command_board_ros.hpp"

#include "nav_interfaces/msg/nav_system_status.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <array>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <ctime>
#include <filesystem>
#include <stdexcept>
#include <algorithm>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <vector>

extern char** environ;

namespace command_board {

using namespace std::chrono_literals;

CommandBoardManagerROS::CommandBoardManagerROS() : rclcpp::Node("command_board_manager") {
    localization_sm_ = std::make_unique<Localization_SM>([this]() { this->HandleLocalizationSuccess(); },
                                                         [this]() { this->HandleLocalizationFailed(); });

    reset_origin_sample_frames_ =
        this->declare_parameter("reset_origin_sample_frames", reset_origin_sample_frames_);
    reset_origin_sample_timeout_sec_ =
        this->declare_parameter("reset_origin_sample_timeout_sec", reset_origin_sample_timeout_sec_);
    service_call_timeout_sec_ =
        this->declare_parameter("service_call_timeout_sec", service_call_timeout_sec_);
    service_wait_timeout_sec_ =
        this->declare_parameter("service_wait_timeout_sec", service_wait_timeout_sec_);
    robot_status_timeout_sec_ = 
        this->declare_parameter("robot_status_timeout_sec", robot_status_timeout_sec_);
    
    react_to_estop_  = this->declare_parameter("react_to_estop", react_to_estop_);
    react_to_bumper_ = this->declare_parameter("react_to_bumper", react_to_bumper_);
    cancel_schedule_planner_ =
        this->declare_parameter("cancel_schedule_planner", cancel_schedule_planner_);
    schedule_planner_cancel_service_ =
        this->declare_parameter("schedule_planner_cancel_service", schedule_planner_cancel_service_);

    // ── 导航后端选择（唯一真值源，仅启动期读取）──────────────────
    navigation_backend_ = this->declare_parameter("navigation_backend", navigation_backend_);
    path_nav_autostart_ = this->declare_parameter("path_navigation.autostart", path_nav_autostart_);
    path_nav_manager_service_ =
        this->declare_parameter("path_navigation.lifecycle_manager_service", path_nav_manager_service_);
    path_nav_startup_timeout_sec_ =
        this->declare_parameter("path_navigation.startup_timeout_sec", path_nav_startup_timeout_sec_);
    path_nav_require_map_ready_ =
        this->declare_parameter("path_navigation.require_map_ready", path_nav_require_map_ready_);
    nav2_autostart_ = this->declare_parameter("nav2.autostart", nav2_autostart_);
    nav2_manager_service_ =
        this->declare_parameter("nav2.lifecycle_manager_service", nav2_manager_service_);
    nav2_startup_timeout_sec_ =
        this->declare_parameter("nav2.startup_timeout_sec", nav2_startup_timeout_sec_);
    if (navigation_backend_ != "nav2" && navigation_backend_ != "path_navigation") {
        RCLCPP_ERROR(get_logger(),
                     "Unknown navigation_backend '%s' (expected \"nav2\" or \"path_navigation\"), "
                     "falling back to nav2. The path_navigation servers will stay unconfigured.",
                     navigation_backend_.c_str());
        navigation_backend_ = "nav2";
    }
    react_to_robot_status_ = this->declare_parameter("react_to_robot_status", react_to_robot_status_);

    // 单一状态源：/diagnostics_agg、/localization/state 由 StatusMonitor 唯一订阅聚合，
    // 本节点不再重复订阅，改在 OnRecoverTimer 轮询 GetSnapshot 读取。
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    navigation_action_client_ =
        rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "/navigate_to_pose");

    // ── TF buffer（用于位姿采样）─────────────────────────────────
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ── 服务客户端 ────────────────────────────────────────────────
    schedule_planner_cancel_cli_ =
        this->create_client<std_srvs::srv::Trigger>(schedule_planner_cancel_service_);
    stop_localization_cli_ = this->create_client<std_srvs::srv::Trigger>("/localization/stop_localization");
    start_localization_cli_ = this->create_client<std_srvs::srv::Trigger>("/localization/start_localization");
    map_command_cli_ = this->create_client<nav_interfaces::srv::MapManagerCommand>("map_manager/map_command");

    // ── 订阅当前地图状态（map_state，folder+name+epoch 原子下发）──────────
    auto map_state_qos = rclcpp::QoS(1).transient_local().reliable();
    map_state_sub_ = this->create_subscription<nav_interfaces::msg::MapState>(
        "map_manager/map_state", map_state_qos,
        [this](const nav_interfaces::msg::MapState::SharedPtr msg) {
            current_map_name_ = msg->map_name;
            current_map_folder_ = msg->map_folder;  // 对齐编排解析地图目录要用
        });

    // ── 对外服务 ──────────────────────────────────────────────────
    reset_origin_xyyaw_srv_ = this->create_service<nav_interfaces::srv::ResetMapOriginByXYYaw>(
        "~/reset_map_origin_by_xyyaw",
        std::bind(&CommandBoardManagerROS::ResetMapOriginByXYYawCallback, this, std::placeholders::_1,
                  std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_);

    reset_origin_robot_pose_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "~/reset_map_origin_by_robot_pose",
        std::bind(&CommandBoardManagerROS::ResetMapOriginByRobotPoseCallback, this, std::placeholders::_1,
                  std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_);

    align_min_yaw_deg_ = this->declare_parameter("align_min_yaw_deg", align_min_yaw_deg_);
    align_map_srv_ = this->create_service<nav_interfaces::srv::AlignMap>(
        "~/align_map",
        std::bind(&CommandBoardManagerROS::AlignMapCallback, this, std::placeholders::_1,
                  std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_);

    replace_map_srv_ = this->create_service<nav_interfaces::srv::ReplaceMap>(
        "~/replace_map",
        std::bind(&CommandBoardManagerROS::ReplaceMapCallback, this, std::placeholders::_1,
                  std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_);

    // 地图版本备份 + 回退单入口：~/backup_unified、~/rollback_map
    map_version_manager_ = std::make_unique<MapVersionManager>(this);

    // 状态聚合 + 条件自动保存（StatusMonitor 提供快照，AutoMapSaver 日级触发 + ~/save_map 手动）
    status_monitor_ = std::make_unique<StatusMonitor>(this);
    auto_map_saver_ = std::make_unique<AutoMapSaver>(this, status_monitor_.get());

    // ── 传感器监测初始化 ──────────────────────────────────────────
    // 参数不合法（缺 topic/type、超时非法、check_period 过大、名称重复）→ 启动失败。
    {
        SensorMonitor::Config sensor_config;
        std::string sensor_err;
        if (!LoadSensorMonitorConfig(this, sensor_config, sensor_err)) {
            RCLCPP_ERROR(get_logger(), "Invalid sensor_monitor parameters: %s", sensor_err.c_str());
            throw std::runtime_error("invalid sensor_monitor parameters: " + sensor_err);
        }
        SensorMonitor::Hooks hooks;
        hooks.cancel_navigation = [this]() {
            if (navigation_action_client_) {
                navigation_action_client_->async_cancel_all_goals();
            }
        };
        hooks.is_estop_active = [this]() { return status_monitor_->IsEstopActive(); };
        hooks.publish_now = [this]() { status_monitor_->PublishNow(); };
        hooks.set_nav_fault =
            [this](const int32_t code, const std::string& detail,
                   const std::vector<nav_interfaces::msg::SensorState>& sensors) {
                status_monitor_->SetNavFault(code, detail, sensors);
            };
        sensor_monitor_ = std::make_unique<SensorMonitor>(sensor_config, std::move(hooks));
        sensor_monitor_callback_group_ =
            this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        sensor_monitor_->AttachToNode(this, sensor_monitor_callback_group_);
    }

    // ── 急停丢位恢复初始化 ────────────────────────────────────────
    enable_loss_recovery_ = this->declare_parameter("enable_loss_recovery", enable_loss_recovery_);
    recover_settle_sec_   = this->declare_parameter("recover_settle_sec", recover_settle_sec_);
    reloc_wait_sec_       = this->declare_parameter("reloc_wait_sec", reloc_wait_sec_);
    // 默认值取自契约 LocalizationState 常量（唯一真值源），现场仍可用参数 override。
    // 不在此处另写字面量，避免与契约形成双真值源。
    score_good_           = this->declare_parameter(
        "reloc_score_good",
        static_cast<double>(nav_interfaces::msg::LocalizationState::SCORE_THRESHOLD_GOOD));
    score_bad_            = this->declare_parameter(
        "reloc_score_bad",
        static_cast<double>(nav_interfaces::msg::LocalizationState::SCORE_THRESHOLD_BAD));
    pose_cache_min_score_ = this->declare_parameter("pose_cache_min_score", pose_cache_min_score_);
    velocity_still_threshold_ =
        this->declare_parameter("still_velocity_threshold", velocity_still_threshold_);
    relocalization_cli_ = this->create_client<nav_interfaces::srv::Relocalize>(
        "/localization/relocalization");
    alarm_pub_ = this->create_publisher<std_msgs::msg::String>(
        "~/safety_alarm", rclcpp::QoS(1).transient_local());
    // OnRecoverTimer 始终运行（驱动定位状态机 + 急停响应 + 丢位恢复）；enable_loss_recovery_
    // 仅控制其中的停定位 + 重定位部分。
    recover_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&CommandBoardManagerROS::OnRecoverTimer, this), callback_group_);

    // 后端编排：只碰**被选中**那套的生命周期管理器，另一套原样留在它自己的状态
    // （两套同时常驻时，未选中那套的 launch 必须用 autostart:=false 起，
    //  否则它自己的管理器会把自己拉到 active，本节点管不着——这是唯一需要现场配对的地方）。
    if (navigation_backend_ == "path_navigation" && path_nav_autostart_) {
        backend_manager_service_ = path_nav_manager_service_;
        backend_startup_timeout_sec_ = path_nav_startup_timeout_sec_;
        backend_require_map_ready_ = path_nav_require_map_ready_;
    } else if (navigation_backend_ == "nav2" && nav2_autostart_) {
        backend_manager_service_ = nav2_manager_service_;
        backend_startup_timeout_sec_ = nav2_startup_timeout_sec_;
        backend_require_map_ready_ = false;  // nav2 自己有地图服务器，不卡在 MapState 上
    }

    // 两套管理器的客户端都建：运行时切换要同时 PAUSE 旧的、STARTUP 新的。
    // 放在 Reentrant 组里——切换服务会同步等这些应答，同组必须能并行处理，否则自锁。
    switch_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    path_nav_manager_cli_ = this->create_client<nav2_msgs::srv::ManageLifecycleNodes>(
        path_nav_manager_service_, rmw_qos_profile_services_default, switch_group_);
    nav2_manager_cli_ = this->create_client<nav2_msgs::srv::ManageLifecycleNodes>(
        nav2_manager_service_, rmw_qos_profile_services_default, switch_group_);

    backend_state_pub_ = this->create_publisher<std_msgs::msg::String>(
        "~/navigation_backend", rclcpp::QoS(1).transient_local());
    PublishBackendState();

    // navigation_backend 是开机默认值，运行中改它不生效。不拦的话现场会以为 param set
    // 生效了、实际速度源没换，比报错危险得多
    backend_param_cb_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto& p : params) {
                if (p.get_name() == "navigation_backend" &&
                    p.as_string() != navigation_backend_) {
                    result.successful = false;
                    result.reason =
                        "navigation_backend is the boot-time default only; change it in "
                        "command_board_params.yaml and restart the auto service.";
                }
            }
            return result;
        });

    if (!backend_manager_service_.empty()) {
        backend_startup_begin_ = this->now();
        backend_startup_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&CommandBoardManagerROS::OnBackendStartupTimer, this), callback_group_);
        RCLCPP_INFO(get_logger(),
                    "navigation_backend=%s: waiting for %s to bring that backend up (timeout %.0fs)",
                    navigation_backend_.c_str(), backend_manager_service_.c_str(),
                    backend_startup_timeout_sec_);
    } else {
        RCLCPP_INFO(get_logger(),
                    "navigation_backend=%s, autostart disabled for it - not touching any "
                    "lifecycle manager (the selected backend is expected to start itself, "
                    "the other one must be launched with autostart:=false)",
                    navigation_backend_.c_str());
    }

    RCLCPP_INFO(get_logger(),
                "CommandBoardManagerROS initialized. "
                "Services: ~/reset_map_origin_by_xyyaw, ~/reset_map_origin_by_robot_pose, "
                "~/align_map, ~/replace_map, ~/backup_unified, ~/rollback_map");

    // 同名实例保护：同一个服务名被多个进程同时提供时，一次请求会被每个实例各执行一遍
    // （例如重复 ros2 run 留在后台的旧进程，会同时改写同一张地图）。启动后查一次 ROS 图，
    // 已有同名节点就退出。只在启动时查：先起来的实例不受影响，退出的总是后来者。
    single_instance_timer_ = this->create_wall_timer(std::chrono::seconds(2), [this]() {
        single_instance_timer_->cancel();
        const std::string me = this->get_fully_qualified_name();
        const auto names = this->get_node_names();
        const auto count = std::count(names.begin(), names.end(), me);
        if (count > 1) {
            RCLCPP_FATAL(this->get_logger(),
                         "Another instance of node '%s' is already running (%ld found); exiting so that "
                         "requests are not executed twice. Stop the old instance first.",
                         me.c_str(), static_cast<long>(count));
            rclcpp::shutdown();
        }
    });
}

// ──────────────────────────────────────────────────────────────────────
// 外部服务入口
// ──────────────────────────────────────────────────────────────────────

// 入口①：按 xyyaw 重置任意地图
//   - 传入 map_name（必填），不限于当前使用中的地图
//   - 只做：① subprocess 修改 pbstream/png/yaml
//            ② subprocess transform_assets_tool 修改地图资产文件
//   - 非当前地图：不停/启定位，不重载地图；当前地图：转入 DoResetCurrentMapOrigin 完整流程
void CommandBoardManagerROS::ResetMapOriginByXYYawCallback(
    const std::shared_ptr<nav_interfaces::srv::ResetMapOriginByXYYaw::Request> req,
    std::shared_ptr<nav_interfaces::srv::ResetMapOriginByXYYaw::Response> res) {
    if (req->map_name.empty()) {
        res->success = false;
        res->message = "map_name must not be empty for reset_map_origin_by_xyyaw";
        RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
        return;
    }

    RCLCPP_INFO(get_logger(), "[XYYaw] reset map='%s' x=%.3f y=%.3f yaw=%.4f", req->map_name.c_str(), req->x, req->y,
                req->yaw);

    // 当前地图：资产节点与定位都持有旧坐标的内存副本，只改文件会被后续自动保存覆盖，
    // 走完整流程（停定位 → 两个工具 → 重新加载地图 → 启定位）
    if (req->map_name == current_map_name_) {
        RCLCPP_INFO(get_logger(), "[XYYaw] '%s' is the current map, using the full reset flow",
                    req->map_name.c_str());
        std::string msg;
        res->success = DoResetCurrentMapOrigin(req->x, req->y, req->yaw, msg);
        res->message = msg;
        res->applied_x = req->x;
        res->applied_y = req->y;
        res->applied_yaw = req->yaw;
        return;
    }

    // ── 底图 + 资产两个离线工具串行，任一失败回滚底图 ─────────────
    std::string msg;
    const bool ok = ResetOriginFiles(req->map_name, req->x, req->y, req->yaw, msg);
    res->success = ok;
    res->message = msg;
    res->applied_x = req->x;
    res->applied_y = req->y;
    res->applied_yaw = req->yaw;

    if (ok) {
        RCLCPP_INFO(get_logger(), "[XYYaw] Done: %s", res->message.c_str());
    } else {
        RCLCPP_ERROR(get_logger(), "[XYYaw] %s", res->message.c_str());
    }
}

// 重设原点的文件部分：reset_map_origin_tool（底图）→ transform_assets_tool（资产），串行。
// 底图工具改前会存一份 reset_origin 版本；任一步失败都回滚到该版本，底图与资产始终一致。
bool CommandBoardManagerROS::ResetOriginFiles(const std::string& map_name, double x, double y, double yaw,
                                              std::string& out_msg) {
    const std::set<std::string> backups_before = ListSlamBackupDirs(map_name);
    std::string tool_log;
    if (!RunResetMapOriginTool(map_name, x, y, yaw, tool_log)) {
        std::string rb_msg;
        const int rb = RollbackSlamAfterToolFailure(map_name, backups_before, "", rb_msg);
        out_msg = "reset_map_origin_tool failed: " + tool_log + "; " +
                  (rb < 0 ? "ROLLBACK FAILED (" + rb_msg + "), manual recovery required" : rb_msg);
        return false;
    }
    const AssetsToolResult assets = RunTransformAssetsTool(map_name, x, y, yaw);
    if (!assets.ok) {
        std::string rb_msg;
        const int rb = RollbackSlamAfterToolFailure(map_name, backups_before, "", rb_msg);
        out_msg = "transform_assets failed: " + assets.Summary() + "; " +
                  (rb < 0 ? "base map ROLLBACK FAILED (" + rb_msg + "), base map and assets are INCONSISTENT"
                          : "base map rolled back: " + rb_msg);
        return false;
    }
    out_msg = "OK: " + assets.Summary();
    return true;
}

// 入口②：按当前机器人位姿重置（仅当前地图，完整流程）
void CommandBoardManagerROS::ResetMapOriginByRobotPoseCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                                               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    RCLCPP_INFO(get_logger(), "[RobotPose] sampling TF map→base_link ...");
    double x, y, yaw;
    if (!SampleRobotPose(reset_origin_sample_frames_, reset_origin_sample_timeout_sec_, x, y, yaw)) {
        res->success = false;
        res->message = "Failed to sample robot pose from TF (map→base_link)";
        RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
        return;
    }
    RCLCPP_INFO(get_logger(), "[RobotPose] sampled (%d-frame mean): x=%.3f y=%.3f yaw=%.4f",
                reset_origin_sample_frames_, x, y, yaw);
    std::string msg;
    res->success = DoResetCurrentMapOrigin(x, y, yaw, msg);
    res->message = msg;
}

// ──────────────────────────────────────────────────────────────────────
// 核心编排：完整重置流程（当前地图，RobotPose 入口专用）
// 步骤：停导航 → 停定位 → 底图工具 → 资产工具（失败回滚底图）→ 重载地图 → 启定位
// ──────────────────────────────────────────────────────────────────────

bool CommandBoardManagerROS::DoResetCurrentMapOrigin(double x, double y, double yaw, std::string& out_msg) {
    if (current_map_name_.empty()) {
        out_msg = "current_map_name is empty, no map loaded";
        RCLCPP_ERROR(get_logger(), "%s", out_msg.c_str());
        return false;
    }

    // ── 步骤 ①：停止导航 ─────────────────────────────────────────
    RCLCPP_INFO(get_logger(), "[Reset] Step 1: cancel navigation");
    navigation_action_client_->async_cancel_all_goals();
    // 地图原点马上要被挪，路网任务更不能留着：它跟的是旧原点下的一串点
    CancelSchedulePlannerRoute("map origin reset");
    std::this_thread::sleep_for(300ms);

    // ── 步骤 ②：停止定位 ─────────────────────────────────────────
    RCLCPP_INFO(get_logger(), "[Reset] Step 2: stop localization");
    if (stop_localization_cli_->service_is_ready()) {
        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto fut = stop_localization_cli_->async_send_request(req);
        if (fut.wait_for(std::chrono::duration<double>(service_call_timeout_sec_)) != std::future_status::ready) {
            out_msg = "Timeout waiting for stop_localization";
            RCLCPP_ERROR(get_logger(), "%s", out_msg.c_str());
            return false;
        }
        if (!fut.get()->success) {
            RCLCPP_WARN(get_logger(), "stop_localization returned false, continuing anyway");
        }
    } else {
        RCLCPP_WARN(get_logger(), "stop_localization service not ready, skipping");
    }

    // ── 步骤 ③④：底图工具 → 资产工具（串行，任一失败回滚底图）────
    RCLCPP_INFO(get_logger(), "[Reset] Steps 3+4: reset_map_origin_tool -> transform_assets_tool");
    std::string files_msg;
    const bool files_ok = ResetOriginFiles(current_map_name_, x, y, yaw, files_msg);
    if (files_ok) {
        RCLCPP_INFO(get_logger(), "[Reset] files updated: %s", files_msg.c_str());
    } else {
        RCLCPP_ERROR(get_logger(), "[Reset] %s", files_msg.c_str());
        // 失败也继续重载与启定位，保持系统可用（底图已回滚到重设前）
    }

    // ── 步骤 ⑤：重载地图（触发 map_manager 发布新 png/yaml + 通知各节点）
    RCLCPP_INFO(get_logger(), "[Reset] Step 5: reload map '%s'", current_map_name_.c_str());
    if (map_command_cli_->service_is_ready()) {
        auto req = std::make_shared<nav_interfaces::srv::MapManagerCommand::Request>();
        req->op = nav_interfaces::srv::MapManagerCommand::Request::OP_LOAD;
        req->map_name = current_map_name_;
        auto fut = map_command_cli_->async_send_request(req);
        if (fut.wait_for(std::chrono::duration<double>(service_call_timeout_sec_)) != std::future_status::ready) {
            RCLCPP_WARN(get_logger(), "[Reset] Timeout waiting for map_command(LOAD), continuing");
        } else if (!fut.get()->success) {
            RCLCPP_WARN(get_logger(), "[Reset] map_command(LOAD) returned false");
        } else {
            RCLCPP_INFO(get_logger(), "[Reset] map_command(LOAD) OK");
        }
    } else {
        RCLCPP_WARN(get_logger(), "[Reset] map_command service not ready, skipping");
    }

    // ── 步骤 ⑥：重启定位（自动触发 relocalization）────────────────
    RCLCPP_INFO(get_logger(), "[Reset] Step 6: start localization");
    if (start_localization_cli_->service_is_ready()) {
        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto fut = start_localization_cli_->async_send_request(req);
        if (fut.wait_for(std::chrono::duration<double>(service_call_timeout_sec_)) != std::future_status::ready) {
            RCLCPP_WARN(get_logger(), "[Reset] Timeout waiting for start_localization");
        } else if (!fut.get()->success) {
            RCLCPP_WARN(get_logger(), "[Reset] start_localization returned false");
        }
    } else {
        RCLCPP_WARN(get_logger(), "[Reset] start_localization service not ready, skipping");
    }

    if (!files_ok) {
        out_msg = "Reset map origin failed: " + files_msg;
        return false;
    }

    out_msg = "Reset map origin success. map=" + current_map_name_ + " x=" + std::to_string(x) +
              " y=" + std::to_string(y) + " yaw=" + std::to_string(yaw);
    RCLCPP_INFO(get_logger(), "[Reset] Done. %s", out_msg.c_str());
    return true;
}

// ──────────────────────────────────────────────────────────────────────
// 采样机器人位姿（TF map→base_link，n_frames 帧均值）
// ──────────────────────────────────────────────────────────────────────

bool CommandBoardManagerROS::SampleRobotPose(
    int n_frames, double timeout_sec, double& out_x, double& out_y, double& out_yaw) {
    const auto deadline = this->now() + rclcpp::Duration::from_seconds(timeout_sec);
    double sum_x = 0.0, sum_y = 0.0, sum_sin = 0.0, sum_cos = 0.0;
    int collected = 0;

    while (collected < n_frames && this->now() < deadline) {
        try {
            auto tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            sum_x += tf.transform.translation.x;
            sum_y += tf.transform.translation.y;
            tf2::Quaternion q;
            tf2::fromMsg(tf.transform.rotation, q);
            const double yaw = tf2::getYaw(q);
            sum_cos += std::cos(yaw);
            sum_sin += std::sin(yaw);
            ++collected;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "SampleRobotPose: TF not available: %s", ex.what());
        }
        std::this_thread::sleep_for(100ms);
    }

    if (collected < n_frames / 2) {
        RCLCPP_ERROR(get_logger(), "SampleRobotPose: only %d/%d frames collected", collected, n_frames);
        return false;
    }

    out_x = sum_x / collected;
    out_y = sum_y / collected;
    out_yaw = std::atan2(sum_sin / collected, sum_cos / collected);
    return true;
}

namespace {

// 以参数列表启动子进程（posix_spawnp，不经 shell），stdout 与 stderr 合并读回。
// 地图名、路径原样作为单个参数传入：名字里的引号、$、反引号、空格都不会被 shell 解释或拆开。
// 返回退出码；启动失败返回 -1，被信号终止返回 128 + 信号号。
int RunProcessCapture(const std::vector<std::string>& args, std::string* output) {
    int fds[2];
    if (::pipe2(fds, O_CLOEXEC) != 0) {
        *output = std::string("pipe failed: ") + std::strerror(errno);
        return -1;
    }
    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_adddup2(&actions, fds[1], STDOUT_FILENO);
    posix_spawn_file_actions_adddup2(&actions, fds[1], STDERR_FILENO);
    std::vector<char*> argv;
    for (const auto& a : args) argv.push_back(const_cast<char*>(a.c_str()));
    argv.push_back(nullptr);
    pid_t pid = 0;
    const int rc = posix_spawnp(&pid, argv[0], &actions, nullptr, argv.data(), environ);
    posix_spawn_file_actions_destroy(&actions);
    ::close(fds[1]);
    if (rc != 0) {
        ::close(fds[0]);
        *output = "spawn " + args[0] + " failed: " + std::strerror(rc);
        return -1;
    }
    char buf[4096];
    for (;;) {
        const ssize_t n = ::read(fds[0], buf, sizeof(buf));
        if (n > 0) { output->append(buf, static_cast<size_t>(n)); continue; }
        if (n < 0 && errno == EINTR) continue;
        break;
    }
    ::close(fds[0]);
    int status = 0;
    while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {}
    if (WIFEXITED(status)) return WEXITSTATUS(status);
    if (WIFSIGNALED(status)) return 128 + WTERMSIG(status);
    return -1;
}

std::string FormatFixed(double v, int digits) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.*f", digits, v);
    return buf;
}

// 只用于日志显示：参数里有空白或引号时加单引号
std::string CommandLineForLog(const std::vector<std::string>& args) {
    std::string out;
    for (const auto& a : args) {
        if (!out.empty()) out += ' ';
        if (a.find_first_of(" \t'\"$`\\") == std::string::npos && !a.empty()) {
            out += a;
        } else {
            out += '\'';
            for (const char c : a) out += (c == '\'' ? std::string("'\\''") : std::string(1, c));
            out += '\'';
        }
    }
    return out;
}

}  // namespace

// ──────────────────────────────────────────────────────────────────────
// 子进程调用 reset_map_origin_tool
// ──────────────────────────────────────────────────────────────────────

bool CommandBoardManagerROS::RunResetMapOriginTool(
    const std::string& map_name, double x, double y, double yaw, std::string& out_log) {
    // 依赖 source install/setup.bash 后 PATH 含 ros2
    const std::vector<std::string> args = {
        "ros2", "run", "cartographer_ros", "reset_map_origin_tool",
        "--map_name", map_name,
        "--new_x", FormatFixed(x, 6), "--new_y", FormatFixed(y, 6), "--new_yaw", FormatFixed(yaw, 6)};
    RCLCPP_INFO(get_logger(), "RunResetMapOriginTool: %s", CommandLineForLog(args).c_str());

    const int rc = RunProcessCapture(args, &out_log);
    if (rc != 0) {
        RCLCPP_ERROR(get_logger(), "reset_map_origin_tool exited with code %d\n%s", rc, out_log.c_str());
        return false;
    }
    return true;
}

// ──────────────────────────────────────────────────────────────────────
// 原有事件处理
// ──────────────────────────────────────────────────────────────────────

// 单一状态源：所有上游由 StatusMonitor 订阅聚合，本节点 200ms 轮询 GetSnapshot
// 驱动定位状态机 + 急停响应 + 丢位恢复（不再重复订阅 /diagnostics_agg、/localization/state）。
void CommandBoardManagerROS::OnRecoverTimer() {
    if (!status_monitor_) return;
    const auto snap = status_monitor_->GetSnapshot();
    const auto now = this->now();

    // 1. 更新定位 score + 缓存高 score 位姿。
    if (snap.localization_valid) {
        latest_score_ = snap.localization.score;
        SnapshotGoodPose();
    }

    // 2. 驱动定位状态机（localization.status 变化时）。
    if (snap.localization_valid && snap.localization.status != last_localization_status_) {
        last_localization_status_ = snap.localization.status;
        if (snap.localization.status == nav_interfaces::msg::LocalizationState::LOC_SUCCESS) {
            localization_sm_->LocalizationSuccess();
        } else {
            localization_sm_->LocalizationFailed();
        }
    }

    // 3. 急停/碰撞翻转/掉op检测（从 StatusMonitor 聚合的安全状态读）。
    const bool robot_status_stale =
        snap.robot_status_valid &&
        (now - snap.robot_status_stamp).seconds() >
            robot_status_timeout_sec_;

    const bool robot_not_ready =
        snap.robot_status_valid &&
        snap.robot_statusword != "READY" &&
        snap.robot_statusword != "ENABLED";

    const bool robot_status_fault =
        react_to_robot_status_ &&
        (robot_not_ready || robot_status_stale);
    const bool active = (react_to_estop_ && snap.is_estop_active) ||
                        (react_to_bumper_ && (snap.bumper_front_active || snap.bumper_rear_active)) ||
                        robot_status_fault;
    if (active && !safety_active_) {
        navigation_action_client_->async_cancel_all_goals();
        // 路网任务跑在 NavigateThroughGraphPoints 上，上面那刀砍不到它
        CancelSchedulePlannerRoute("safety active");
        localization_sm_->LocalizationFailed();
        RCLCPP_WARN(this->get_logger(),
                    "Safety active (estop=%d bumper=%d) — cancel navigation%s",
                    snap.is_estop_active,
                    (snap.bumper_front_active || snap.bumper_rear_active),
                    enable_loss_recovery_ ? " + stop localization (recovery armed)" : "");
        if (enable_loss_recovery_) {
            StopLocalization();
            recover_state_ = RecoverState::kSafeStop;
        }
    } else if (!active && safety_active_) {
        RCLCPP_INFO(this->get_logger(), "Safety released — awaiting settle for relocalization.");
        settle_start_ = now;
    }
    safety_active_ = active;

    // 4. 丢位恢复状态机。
    if (!enable_loss_recovery_) { PushRecoverStatus(); return; }
    switch (recover_state_) {
        case RecoverState::kNormal:
            break;
        case RecoverState::kSafeStop:
            if (safety_active_) break;
            if (!RobotIsStill()) { settle_start_ = now; break; }
            if ((now - settle_start_).seconds() < recover_settle_sec_) break;
            StartLocalization();
            if (last_known_pose_valid_) {
                Relocalize(last_known_pose_);
                recover_state_ = RecoverState::kRelocating;
            } else {
                RCLCPP_WARN(this->get_logger(),
                            "No cached pose — escalating to global relocalization.");
                GlobalRelocalize();
                recover_state_ = RecoverState::kGlobalReloc;
            }
            reloc_start_ = now;
            break;
        case RecoverState::kRelocating:
            if ((now - reloc_start_).seconds() < reloc_wait_sec_) break;
            if (latest_score_ >= score_good_) {
                RCLCPP_INFO(this->get_logger(),
                            "Relocalized OK from last pose (score=%.2f).", latest_score_);
                recover_state_ = RecoverState::kNormal;
            } else if (latest_score_ < score_bad_) {
                RCLCPP_WARN(this->get_logger(),
                            "Local relocalization weak (score=%.2f) — global fallback.",
                            latest_score_);
                GlobalRelocalize();
                reloc_start_ = now;
                recover_state_ = RecoverState::kGlobalReloc;
            } else {
                reloc_start_ = now;  // 中间档，再等一轮
            }
            break;
        case RecoverState::kGlobalReloc:
            if ((now - reloc_start_).seconds() < reloc_wait_sec_ * 2.0) break;
            if (latest_score_ >= score_good_) {
                RCLCPP_INFO(this->get_logger(),
                            "Global relocalization OK (score=%.2f).", latest_score_);
                recover_state_ = RecoverState::kNormal;
            } else {
                RaiseAlarm("Global relocalization failed — manual RViz 2D Pose Estimate needed");
                recover_state_ = RecoverState::kAlarm;
            }
            break;
        case RecoverState::kAlarm:
            if (latest_score_ >= score_good_) {
                RCLCPP_INFO(this->get_logger(),
                            "Recovered from alarm (manual relocalization, score=%.2f).",
                            latest_score_);
                recover_state_ = RecoverState::kNormal;
            }
            break;
    }
    PushRecoverStatus();
}

void CommandBoardManagerROS::SnapshotGoodPose() {
    if (recover_state_ != RecoverState::kNormal) return;
    if (!status_monitor_) return;
    if (latest_score_ < pose_cache_min_score_) return;
    const auto snap = status_monitor_->GetSnapshot();
    if (snap.pose_valid) {
        last_known_pose_ = snap.pose;
        last_known_pose_valid_ = true;
    }
}

bool CommandBoardManagerROS::RobotIsStill() const {
    if (!status_monitor_) return true;
    const auto snap = status_monitor_->GetSnapshot();
    if (!snap.velocity_valid) return true;
    const double v = std::hypot(snap.velocity.linear.x, snap.velocity.linear.y);
    const double w = std::abs(snap.velocity.angular.z);
    return v < velocity_still_threshold_ && w < velocity_still_threshold_;
}


rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr
CommandBoardManagerROS::ManagerFor(const std::string& backend) const {
    if (backend == "path_navigation") return path_nav_manager_cli_;
    if (backend == "nav2") return nav2_manager_cli_;
    return nullptr;
}

void CommandBoardManagerROS::PublishBackendState() {
    if (!backend_state_pub_) return;
    std_msgs::msg::String msg;
    msg.data = navigation_backend_;
    backend_state_pub_->publish(msg);
}

bool CommandBoardManagerROS::CallBackendManager(
    const std::string& backend, uint8_t command, std::string& err) {
    const auto cli = ManagerFor(backend);
    if (!cli) {
        err = "unknown backend '" + backend + "'";
        return false;
    }
    if (!cli->service_is_ready()) {
        err = "lifecycle manager of '" + backend + "' is not available "
              "(is its launch running?)";
        return false;
    }

    auto req = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
    req->command = command;
    auto future = cli->async_send_request(req);
    // 同步等：本函数只在 switch_group_（Reentrant）里跑，应答由组内另一个线程投递，
    // 不会自锁。超时不当成功——宁可报失败让上层重试，也不能在状态不明时继续往下走
    if (future.wait_for(std::chrono::duration<double>(backend_switch_timeout_sec_)) !=
        std::future_status::ready) {
        err = "timeout (" + std::to_string(static_cast<int>(backend_switch_timeout_sec_)) +
              "s) waiting for the lifecycle manager of '" + backend + "'";
        return false;
    }
    if (!future.get()->success) {
        err = "lifecycle manager of '" + backend + "' returned failure "
              "(check which node refused the transition in its log)";
        return false;
    }
    return true;
}

bool CommandBoardManagerROS::SwitchNavigationBackend(
    const std::string& target, bool force, std::string& msg) {
    const std::string from = navigation_backend_;
    // 只有"确实已经 active"才走短路。开机 STARTUP 失败/超时时 backend_active_ 仍是 false，
    // 这时再调本服务要真去把它拉起来，而不是回一句"已经是这个后端了"把人挡回去
    if (target == from && !force && backend_active_.load()) {
        msg = "already on '" + target + "' and active (pass force=true to redo the transition)";
        return true;
    }

    RCLCPP_WARN(get_logger(), "Switching navigation backend: %s -> %s%s",
                from.c_str(), target.c_str(), force ? " (forced)" : "");

    // ① 先把在跑的导航停掉再动生命周期：直接 PAUSE 会让正在执行的任务以"节点被拔掉"的
    //    方式死掉，上层拿不到明确终态。这两刀分别打两套后端，都不在时是无副作用的空调用
    navigation_action_client_->async_cancel_all_goals();
    CancelSchedulePlannerRoute("navigation backend switch");

    // ② PAUSE 旧后端（deactivate 但保留 configured，切回来时快）。
    //    旧后端的管理器不在（比如那套 launch 压根没起）时不算失败——本来就没东西在输出速度
    std::string err;
    if (!from.empty() && ManagerFor(from) && ManagerFor(from)->service_is_ready()) {
        if (!CallBackendManager(from, nav2_msgs::srv::ManageLifecycleNodes::Request::PAUSE, err)) {
            msg = "failed to pause the current backend '" + from + "': " + err +
                  "; nothing was switched";
            RCLCPP_ERROR(get_logger(), "%s", msg.c_str());
            return false;
        }
        backend_active_.store(false);
    } else {
        RCLCPP_WARN(get_logger(),
                    "Current backend '%s' has no reachable lifecycle manager, "
                    "skipping its PAUSE (it cannot be producing velocity anyway)",
                    from.c_str());
    }

    // ③ STARTUP 新后端
    if (!CallBackendManager(target, nav2_msgs::srv::ManageLifecycleNodes::Request::STARTUP, err)) {
        // 回滚：新的起不来就把旧的拉回来，否则两套都是停的 = 机器人彻底不能导航
        std::string rollback_err;
        const bool rolled_back = CallBackendManager(
            from, nav2_msgs::srv::ManageLifecycleNodes::Request::STARTUP, rollback_err);
        backend_active_.store(rolled_back);
        msg = "failed to start '" + target + "': " + err +
              (rolled_back ? "; rolled back to '" + from + "'"
                           : "; rollback to '" + from + "' ALSO FAILED: " + rollback_err +
                             " -- no navigation backend is active");
        RCLCPP_ERROR(get_logger(), "%s", msg.c_str());
        return false;
    }

    navigation_backend_ = target;
    backend_active_.store(true);
    PublishBackendState();
    msg = "switched to '" + target + "'";
    RCLCPP_WARN(get_logger(), "Navigation backend is now '%s'", target.c_str());
    return true;
}

void CommandBoardManagerROS::OnBackendStartupTimer() {
    const double waited = (this->now() - backend_startup_begin_).seconds();
    const bool expired = waited > backend_startup_timeout_sec_;

    // 上一发 STARTUP 还没回来就不要再发：管理器的 STARTUP 会一直阻塞到它名下的节点出现
    // （nav2_util::LifecycleServiceClient 的构造函数就在死等 get_state 服务），
    // 所以"很久没回应答"是正常现象，不是丢了
    if (backend_startup_inflight_.load()) {
        return;
    }

    const auto manager = ManagerFor(navigation_backend_);
    if (!manager || !manager->service_is_ready()) {
        if (expired) {
            RCLCPP_ERROR(get_logger(),
                         "navigation_backend=%s but %s never appeared within %.0fs. "
                         "That backend stays unconfigured (no navigation available). "
                         "Check that its launch is actually running, then restart the "
                         "auto service.",
                         navigation_backend_.c_str(), backend_manager_service_.c_str(),
                         backend_startup_timeout_sec_);
            backend_startup_timer_->cancel();
        }
        return;
    }

    // 路网任务要靠 map_asset_manager 把点名解析成坐标，没地图时激活了也跑不了任务。
    // 默认不等：有的机型压根不发 MapState，等下去等于永远不激活。
    if (backend_require_map_ready_ && current_map_name_.empty()) {
        if (expired) {
            RCLCPP_ERROR(get_logger(),
                         "%s startup gave up: no map after %.0fs "
                         "(require_map_ready=true). Servers stay unconfigured.",
                         navigation_backend_.c_str(), backend_startup_timeout_sec_);
            backend_startup_timer_->cancel();
        }
        return;
    }

    if (expired) {
        // 服务在、但 STARTUP 一直失败（典型：那套里有节点拒绝转换）。到点就别再刷了
        RCLCPP_ERROR(get_logger(),
                     "%s backend could not be started within %.0fs. No navigation available; "
                     "restart the auto service once the cause is fixed.",
                     navigation_backend_.c_str(), backend_startup_timeout_sec_);
        backend_startup_timer_->cancel();
        return;
    }

    backend_startup_inflight_.store(true);
    auto req = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
    req->command = nav2_msgs::srv::ManageLifecycleNodes::Request::STARTUP;
    manager->async_send_request(
        req,
        [this](rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedFuture future) {
            const auto res = future.get();
            backend_startup_inflight_.store(false);
            if (res->success) {
                backend_active_.store(true);
                PublishBackendState();
                backend_startup_timer_->cancel();
                RCLCPP_INFO(get_logger(), "%s backend is up (STARTUP acknowledged by %s)",
                            navigation_backend_.c_str(), backend_manager_service_.c_str());
            } else {
                // 失败不收摊：定时器继续跑，在 startup_timeout_sec 预算内每秒重试一次。
                // 常见原因是那套的某个节点还没起全，等一会儿就好了
                RCLCPP_WARN(get_logger(),
                            "STARTUP of the %s backend returned failure (%s); will retry. "
                            "If it keeps failing, check which node refused the transition "
                            "in that manager's log.",
                            navigation_backend_.c_str(), backend_manager_service_.c_str());
            }
        });
    RCLCPP_INFO(get_logger(), "Requested STARTUP of the %s backend after %.1fs wait",
                navigation_backend_.c_str(), waited);
}

void CommandBoardManagerROS::CancelSchedulePlannerRoute(const std::string& why) {
    if (!cancel_schedule_planner_ || !schedule_planner_cancel_cli_) return;

    // 服务不在（schedule_planner 没起、或本来就不跑路网任务）是常态，不是错误；
    // 但也不能默默跳过——真有任务在跑时这就是"没取消掉"，必须留痕
    if (!schedule_planner_cancel_cli_->service_is_ready()) {
        RCLCPP_WARN(get_logger(),
                    "Route cancel skipped (%s): %s is not available; "
                    "if a graph route task is running it will keep going",
                    why.c_str(), schedule_planner_cancel_service_.c_str());
        return;
    }

    // 带回调的单发：不阻塞调用点，同时把应答消费掉（丢弃 future 会让应答一直挂在客户端上）
    schedule_planner_cancel_cli_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>(),
        [this, why](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            const auto res = future.get();
            RCLCPP_WARN(get_logger(), "Route cancel (%s) -> success=%d, %s",
                        why.c_str(), res->success, res->message.c_str());
        });
    RCLCPP_WARN(get_logger(), "Cancelling schedule_planner route task (%s)", why.c_str());
}

void CommandBoardManagerROS::StopLocalization() {
    if (stop_localization_cli_->service_is_ready()) {
        stop_localization_cli_->async_send_request(
            std::make_shared<std_srvs::srv::Trigger::Request>());
    }
}

void CommandBoardManagerROS::StartLocalization() {
    if (start_localization_cli_->service_is_ready()) {
        start_localization_cli_->async_send_request(
            std::make_shared<std_srvs::srv::Trigger::Request>());
    }
}

void CommandBoardManagerROS::Relocalize(const geometry_msgs::msg::Pose2D& pose) {
    if (!relocalization_cli_->service_is_ready()) {
        RCLCPP_WARN(this->get_logger(), "relocalization service not ready.");
        return;
    }
    auto req = std::make_shared<nav_interfaces::srv::Relocalize::Request>();
    req->mode = nav_interfaces::srv::Relocalize::Request::MODE_LOCAL;
    req->pose.header.frame_id = "map";
    req->pose.header.stamp = this->now();
    req->pose.pose.pose.position.x = pose.x;
    req->pose.pose.pose.position.y = pose.y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pose.theta);
    req->pose.pose.pose.orientation = tf2::toMsg(q);
    req->pose.pose.covariance[0]  = 0.25;
    req->pose.pose.covariance[7]  = 0.25;
    req->pose.pose.covariance[35] = 0.07;
    relocalization_cli_->async_send_request(req);
    RCLCPP_INFO(this->get_logger(),
                "Local relocalize from last pose (%.2f, %.2f, %.3f).",
                pose.x, pose.y, pose.theta);
}

void CommandBoardManagerROS::GlobalRelocalize() {
    if (relocalization_cli_->service_is_ready()) {
        auto req = std::make_shared<nav_interfaces::srv::Relocalize::Request>();
        req->mode = nav_interfaces::srv::Relocalize::Request::MODE_GLOBAL;
        relocalization_cli_->async_send_request(req);
        RCLCPP_INFO(this->get_logger(), "Triggered global relocalization.");
    } else {
        RCLCPP_WARN(this->get_logger(), "relocalization service not ready.");
    }
}

void CommandBoardManagerROS::RaiseAlarm(const std::string& reason) {
    last_alarm_ = reason;
    std_msgs::msg::String m;
    m.data = reason;
    if (alarm_pub_) alarm_pub_->publish(m);
    RCLCPP_ERROR(this->get_logger(), "[SafetyReactor ALARM] %s", reason.c_str());
}

const char* CommandBoardManagerROS::RecoverStateToString(RecoverState s) {
    switch (s) {
        case RecoverState::kNormal:      return "NORMAL";
        case RecoverState::kSafeStop:    return "SAFE_STOP";
        case RecoverState::kRelocating:  return "RELOCATING";
        case RecoverState::kGlobalReloc: return "GLOBAL_RELOC";
        case RecoverState::kAlarm:       return "ALARM";
    }
    return "NORMAL";
}

void CommandBoardManagerROS::PushRecoverStatus() {
    if (!status_monitor_) return;
    status_monitor_->SetRecoverStatus(
        RecoverStateToString(recover_state_),
        recover_state_ == RecoverState::kAlarm ? last_alarm_ : std::string());
}

// 实现接口方法
void CommandBoardManagerROS::HandleLocalizationFailed() {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Handling localization failure - stopping robot");
    this->StopRobotMovement();
}

void CommandBoardManagerROS::HandleLocalizationSuccess() {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Handling localization success");
    // 可以在这里添加其他处理逻辑
}

void CommandBoardManagerROS::StopRobotMovement() {
    // 创建并调用停止运动的服务客户端
    navigation_action_client_->async_cancel_all_goals();
}

// ══════════════════════════════════════════════════════════════════════
// 地图对齐编排
// 与 reset_map_origin 同一条链路，差别只在角度由工具自动估出。
// 两处刻意偏离既有实现：串行代替并行、失败必须回滚（见 hpp 注释）。
// ══════════════════════════════════════════════════════════════════════

namespace {

namespace cbfs = std::filesystem;

// 极简 JSON 取值器。只用于解析**本编排自己调起的** align_map_tool 的
// --emit_json 输出：单行、扁平、键已知，不需要通用解析器。
// 找不到键时返回 false，调用方保留默认值。
bool JsonFindRaw(const std::string& json, const std::string& key,
                 std::string& out_raw) {
    const std::string pattern = "\"" + key + "\"";
    const size_t k = json.find(pattern);
    if (k == std::string::npos) return false;
    size_t c = json.find(':', k + pattern.size());
    if (c == std::string::npos) return false;
    ++c;
    while (c < json.size() && (json[c] == ' ' || json[c] == '\t')) ++c;
    size_t e = c;
    if (e < json.size() && json[e] == '"') {  // 字符串值
        ++e;
        const size_t start = e;
        while (e < json.size() && json[e] != '"') ++e;
        out_raw = json.substr(start, e - start);
        return true;
    }
    while (e < json.size() && json[e] != ',' && json[e] != '}') ++e;
    out_raw = json.substr(c, e - c);
    // 去尾空白
    while (!out_raw.empty() && (out_raw.back() == ' ' || out_raw.back() == '\n' ||
                                out_raw.back() == '\r' || out_raw.back() == '\t')) {
        out_raw.pop_back();
    }
    return !out_raw.empty();
}

bool JsonGetDouble(const std::string& json, const std::string& key, double* out) {
    std::string raw;
    if (!JsonFindRaw(json, key, raw)) return false;
    try {
        *out = std::stod(raw);
    } catch (const std::exception&) {
        return false;
    }
    return true;
}

bool JsonGetInt(const std::string& json, const std::string& key, int* out) {
    double d = 0.0;
    if (!JsonGetDouble(json, key, &d)) return false;
    *out = static_cast<int>(d);
    return true;
}

bool JsonGetBool(const std::string& json, const std::string& key, bool* out) {
    std::string raw;
    if (!JsonFindRaw(json, key, raw)) return false;
    *out = (raw == "true" || raw == "1");
    return true;
}

bool JsonGetString(const std::string& json, const std::string& key,
                   std::string* out) {
    return JsonFindRaw(json, key, *out);
}

std::string MakeStamp() {
    const std::time_t now = std::time(nullptr);
    std::tm tm_buf{};
    localtime_r(&now, &tm_buf);
    char ts[32];
    std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &tm_buf);
    return std::string(ts);
}

}  // namespace

std::string CommandBoardManagerROS::ResolveMapDir(const std::string& map_name) const {
    if (map_name.empty()) return "";
    if (!current_map_folder_.empty()) {
        return (cbfs::path(current_map_folder_) / map_name).string();
    }
    const char* home = std::getenv("HOME");
    if (home == nullptr) return "";
    return (cbfs::path(home) / ".dexe" / "data" / "maps" / map_name).string();
}

CommandBoardManagerROS::AlignToolResult CommandBoardManagerROS::RunAlignMapTool(
    const std::string& map_name, const bool dry_run) {
    AlignToolResult out;
    const std::string map_dir = ResolveMapDir(map_name);
    if (map_dir.empty()) {
        out.reason = "map_dir_unresolved";
        out.log = "cannot resolve map directory for '" + map_name + "'";
        return out;
    }

    std::vector<std::string> args = {"ros2", "run", "cartographer_ros", "align_map_tool", "--map_dir", map_dir};
    if (dry_run) args.push_back("--dry_run");
    // --i_know_assets_will_break：本编排随后会调 transform_assets_tool 把资产一起转，
    // 所以这里是「正确路径」，工具那道确认门不该拦我们。裸跑 tool 仍会被拦。
    args.push_back("--i_know_assets_will_break");
    RCLCPP_INFO(get_logger(), "RunAlignMapTool: %s", CommandLineForLog(args).c_str());

    const int rc = RunProcessCapture(args, &out.log);
    if (rc < 0) {
        out.reason = "spawn_failed";
        return out;
    }
    out.process_ok = (rc == 0);

    // 从捕获的 stdout 里取**最后一条** [ALIGN_RESULT] 标记行。
    // 取最后一条而不是第一条：glog 的 alsologtostderr 与工具日志混在同一个流里，
    // 而工具在失败路径会先打结果行再 LOG(FATAL)，正常路径也只打一条——
    // 取最后一条对两种情况都正确，且不受前面任何日志文本影响。
    // 无论退出码如何都要解析：估角失败时工具非零退出，但结果行里带着 reason
    // （no_line_features / no_reliable_orthogonal_structure），那正是要回给调用方的。
    constexpr const char* kMarker = "[ALIGN_RESULT]";
    const size_t marker_pos = out.log.rfind(kMarker);
    if (marker_pos != std::string::npos) {
        const size_t line_end = out.log.find('\n', marker_pos);
        const std::string json = out.log.substr(
            marker_pos, line_end == std::string::npos ? std::string::npos
                                                      : line_end - marker_pos);
        JsonGetBool(json, "ok", &out.ok);
        JsonGetString(json, "reason", &out.reason);
        JsonGetDouble(json, "yaw_rad", &out.yaw_rad);
        JsonGetDouble(json, "yaw_deg", &out.yaw_deg);
        JsonGetDouble(json, "orthogonality_error_deg", &out.orthogonality_error_deg);
        JsonGetDouble(json, "rms_axis_residual_deg", &out.rms_axis_residual_deg);
        JsonGetInt(json, "candidate_lines", &out.candidate_lines);
        JsonGetBool(json, "applied", &out.applied);
        JsonGetDouble(json, "residual_yaw_deg", &out.residual_yaw_deg);
    } else {
        out.reason = "no_result_line";
        RCLCPP_ERROR(get_logger(),
                     "align_map_tool printed no %s line; tool version mismatch?",
                     kMarker);
    }

    if (!out.process_ok) {
        RCLCPP_ERROR(get_logger(), "align_map_tool exited with code %d (reason=%s)",
                     rc, out.reason.c_str());
    }
    return out;
}

CommandBoardManagerROS::AssetsToolResult CommandBoardManagerROS::RunTransformAssetsTool(
    const std::string& map_name, double x, double y, double yaw) {
    AssetsToolResult out;
    const std::string map_dir = ResolveMapDir(map_name);
    if (map_dir.empty()) {
        out.reason = "map_dir_unresolved";
        out.message = "cannot resolve map directory for '" + map_name + "'";
        return out;
    }

    const std::vector<std::string> args = {
        "ros2", "run", "map_asset_manager", "transform_assets_tool", "--map_dir", map_dir,
        "--x", FormatFixed(x, 9), "--y", FormatFixed(y, 9), "--yaw", FormatFixed(yaw, 9)};
    RCLCPP_INFO(get_logger(), "RunTransformAssetsTool: %s", CommandLineForLog(args).c_str());

    const int rc = RunProcessCapture(args, &out.log);
    if (rc < 0) {
        out.reason = "spawn_failed";
        out.message = out.log;
        return out;
    }

    // 取最后一条结果行；进程非零退出时结果行里的 reason / message 就是要回给调用方的原因
    constexpr const char* kMarker = "[TRANSFORM_ASSETS_RESULT]";
    const size_t marker_pos = out.log.rfind(kMarker);
    if (marker_pos == std::string::npos) {
        out.reason = "no_result_line";
        out.message = "transform_assets_tool exited with code " + std::to_string(rc) +
                      " and printed no result line";
        RCLCPP_ERROR(get_logger(), "%s\n%s", out.message.c_str(), out.log.c_str());
        return out;
    }
    const size_t line_end = out.log.find('\n', marker_pos);
    const std::string json = out.log.substr(
        marker_pos, line_end == std::string::npos ? std::string::npos : line_end - marker_pos);
    JsonGetBool(json, "ok", &out.ok);
    JsonGetBool(json, "no_assets", &out.no_assets);
    JsonGetString(json, "reason", &out.reason);
    JsonGetString(json, "message", &out.message);
    JsonGetString(json, "version_ts", &out.version_ts);
    JsonGetInt(json, "points", &out.points);
    JsonGetInt(json, "areas", &out.areas);
    JsonGetInt(json, "junctions", &out.junctions);
    JsonGetInt(json, "edges", &out.edges);
    JsonGetInt(json, "rooms", &out.rooms);
    if (rc != 0) out.ok = false;

    if (out.ok) {
        RCLCPP_INFO(get_logger(), "transform_assets_tool OK: %s", out.message.c_str());
    } else {
        RCLCPP_ERROR(get_logger(), "transform_assets_tool failed (code %d): %s\n%s", rc,
                     out.Summary().c_str(), out.log.c_str());
    }
    return out;
}

bool CommandBoardManagerROS::RestoreSlamBackup(const std::string& map_name,
                                               std::string& out_msg) {
    // 底图版本序列与 map_manager / 离线工具 / 定位侧保存统一：backups/backup.<ts>/，名字即时间。
    // 取最新一份（刚执行的工具在改动前存下的那份）。
    const auto names = ListSlamBackupDirs(map_name);
    if (names.empty()) {
        out_msg = "slam/backups has no backup.* version; cannot roll back base map";
        return false;
    }
    const cbfs::path dir = cbfs::path(ResolveMapDir(map_name)) / "slam" / "backups" / *names.rbegin();
    return RestoreSlamFromBackupDir(map_name, dir.string(), out_msg);
}

bool CommandBoardManagerROS::RestoreSlamFromBackupDir(const std::string& map_name,
                                                      const std::string& backup_dir,
                                                      std::string& out_msg) {
    // 文件恢复统一交给 map_manager（map_command OP_ROLLBACK）：锁、校验、恢复前快照只有一份实现。
    const std::string name = cbfs::path(backup_dir).filename().string();
    if (name.rfind("backup.", 0) != 0) {
        out_msg = "not a map version directory: " + backup_dir;
        return false;
    }
    return RollbackBaseMapVersion(map_name, name.substr(std::string("backup.").size()), out_msg);
}

bool CommandBoardManagerROS::RollbackBaseMapVersion(const std::string& map_name,
                                                    const std::string& version_ts,
                                                    std::string& out_msg) {
    if (!map_command_cli_->service_is_ready() &&
        !map_command_cli_->wait_for_service(std::chrono::seconds(2))) {
        out_msg = "map_manager map_command not available";
        return false;
    }
    auto mc = std::make_shared<nav_interfaces::srv::MapManagerCommand::Request>();
    mc->op = nav_interfaces::srv::MapManagerCommand::Request::OP_ROLLBACK;
    mc->map_name = map_name;
    mc->version_ts = version_ts;
    auto fut = map_command_cli_->async_send_request(mc);
    if (fut.wait_for(std::chrono::duration<double>(service_call_timeout_sec_)) != std::future_status::ready) {
        out_msg = "map_manager rollback timed out";
        return false;
    }
    const auto r = fut.get();
    out_msg = "base map restored to version " + version_ts + ": " + r->message;
    if (r->success) RCLCPP_WARN(get_logger(), "%s", out_msg.c_str());
    return r->success;
}

std::set<std::string> CommandBoardManagerROS::ListSlamBackupDirs(
    const std::string& map_name) const {
    std::set<std::string> out;
    const cbfs::path backups = cbfs::path(ResolveMapDir(map_name)) / "slam" / "backups";
    std::error_code ec;
    if (!cbfs::is_directory(backups, ec) || ec) return out;
    for (const auto& e : cbfs::directory_iterator(backups, ec)) {
        if (ec) break;
        if (!e.is_directory()) continue;
        const std::string name = e.path().filename().string();
        if (name.rfind("backup.", 0) == 0) out.insert(name);
    }
    return out;
}

int CommandBoardManagerROS::RollbackSlamAfterToolFailure(
    const std::string& map_name, const std::set<std::string>& backups_before,
    const std::string& reported_backup_dir, std::string& out_msg) {
    std::string backup_dir = reported_backup_dir;
    std::error_code ec;
    if (backup_dir.empty() || !cbfs::is_directory(backup_dir, ec) || ec) {
        backup_dir.clear();
        // 工具可能异常退出、没打结果行：以执行前后版本序列的差集为准，取其中最新的（名字即时间）。
        const cbfs::path root = cbfs::path(ResolveMapDir(map_name)) / "slam" / "backups";
        for (const auto& name : ListSlamBackupDirs(map_name)) {
            if (!backups_before.count(name)) backup_dir = (root / name).string();
        }
    }
    if (backup_dir.empty()) {
        // 工具先存版本、后写文件；没有新版本 = 失败发生在存版本之前，底图未被改动。
        out_msg = "no backup was created, base map was not modified";
        return 0;
    }
    return RestoreSlamFromBackupDir(map_name, backup_dir, out_msg) ? 1 : -1;
}

void CommandBoardManagerROS::AlignMapCallback(
    const std::shared_ptr<nav_interfaces::srv::AlignMap::Request> req,
    std::shared_ptr<nav_interfaces::srv::AlignMap::Response> res) {
    using NS = nav_interfaces::msg::NavSystemStatus;

    res->success = false;
    res->error_code = NS::E_CB_NONE;
    res->applied = false;

    // ── 互斥：对齐与重置原点都改地图文件，不允许并发 ──────────────
    bool expected = false;
    if (!map_transform_busy_.compare_exchange_strong(expected, true)) {
        res->error_code = NS::E_CB_ALIGN_BUSY;
        res->message = "another map transform task is running";
        return;
    }
    struct BusyGuard {
        std::atomic<bool>* flag;
        ~BusyGuard() { flag->store(false); }
    } guard{&map_transform_busy_};

    const std::string map_name = req->map_name.empty() ? current_map_name_ : req->map_name;
    if (map_name.empty()) {
        res->error_code = NS::E_CB_ALIGN_MAP_NOT_FOUND;
        res->message = "map_name is empty and no map is currently loaded";
        return;
    }
    if (req->min_yaw_deg < 0.0) {
        res->error_code = NS::E_CB_ALIGN_ARG_INVALID;
        res->message = "min_yaw_deg must not be negative";
        return;
    }
    const std::string map_dir = ResolveMapDir(map_name);
    std::error_code fs_ec;
    if (map_dir.empty() || !cbfs::is_directory(map_dir, fs_ec) || fs_ec) {
        res->error_code = NS::E_CB_ALIGN_MAP_NOT_FOUND;
        res->message = "map directory not found: " + map_dir;
        return;
    }
    const double min_yaw_deg =
        req->min_yaw_deg > 0.0 ? req->min_yaw_deg : align_min_yaw_deg_;
    const bool is_current = (map_name == current_map_name_);

    RCLCPP_INFO(get_logger(),
                "[Align] map='%s' (current=%s) dry_run=%d min_yaw=%.2f deg",
                map_name.c_str(), is_current ? "yes" : "no",
                static_cast<int>(req->dry_run), min_yaw_deg);

    // ── 步骤 ①：dry_run 估角（零文件改动）────────────────────────
    const AlignToolResult est = RunAlignMapTool(map_name, /*dry_run=*/true);
    res->estimated_yaw_deg = est.yaw_deg;
    res->orthogonality_error_deg = est.orthogonality_error_deg;
    res->rms_axis_residual_deg = est.rms_axis_residual_deg;
    res->residual_yaw_deg = est.yaw_deg;
    if (!est.usable()) {
        res->error_code = NS::E_CB_ALIGN_ESTIMATE_FAILED;
        res->message = "estimate failed (" +
                       (est.reason.empty() ? std::string("unknown") : est.reason) +
                       "); no file was modified";
        RCLCPP_ERROR(get_logger(), "[Align] %s", res->message.c_str());
        return;
    }
    RCLCPP_INFO(get_logger(),
                "[Align] estimated yaw=%.4f deg (delta=%.4f, rms=%.4f, lines=%d)",
                est.yaw_deg, est.orthogonality_error_deg,
                est.rms_axis_residual_deg, est.candidate_lines);

    // ── 步骤 ②：阈值校验 / dry_run 出口 ─────────────────────────
    if (req->dry_run) {
        res->success = true;
        res->message = "dry run: estimated only, no file modified";
        return;
    }
    if (std::abs(est.yaw_deg) < min_yaw_deg) {
        res->success = true;
        res->message = "already aligned (|yaw|=" + std::to_string(std::abs(est.yaw_deg)) +
                       " deg < " + std::to_string(min_yaw_deg) +
                       " deg); no file modified";
        RCLCPP_INFO(get_logger(), "[Align] %s", res->message.c_str());
        return;
    }

    // ── 步骤 ③：当前地图才需要停导航 / 停定位 ────────────────────
    if (is_current) {
        RCLCPP_INFO(get_logger(), "[Align] Step 3: cancel navigation + stop localization");
        navigation_action_client_->async_cancel_all_goals();
        std::this_thread::sleep_for(300ms);
        if (stop_localization_cli_->service_is_ready()) {
            auto stop_req = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto fut = stop_localization_cli_->async_send_request(stop_req);
            if (fut.wait_for(std::chrono::duration<double>(service_call_timeout_sec_)) !=
                std::future_status::ready) {
                res->error_code = NS::E_CB_LOC_STOP_TIMEOUT;
                res->message = "timeout waiting for stop_localization; no file modified";
                RCLCPP_ERROR(get_logger(), "[Align] %s", res->message.c_str());
                return;
            }
        } else {
            RCLCPP_WARN(get_logger(), "[Align] stop_localization not ready, skipping");
        }
    }

    // 从这里开始会改文件；任何失败都要走回滚。
    const auto restart_localization = [&]() {
        if (!is_current) return;
        if (start_localization_cli_->service_is_ready()) {
            auto r = std::make_shared<std_srvs::srv::Trigger::Request>();
            start_localization_cli_->async_send_request(r);
        }
    };

    // ── 步骤 ⑤：align_map_tool 正式执行（串行，不与 assets 并行）──
    //    并行收益近零：assets 改几百个坐标是毫秒级，tool 重建栅格+点云是秒级。
    const std::set<std::string> backups_before = ListSlamBackupDirs(map_name);
    const AlignToolResult applied = RunAlignMapTool(map_name, /*dry_run=*/false);
    if (!applied.usable()) {
        // 工具可能在写文件途中失败：按本次新建的备份回滚底图，资产此时尚未改动。
        std::string rb_msg;
        const int rb = RollbackSlamAfterToolFailure(map_name, backups_before, "", rb_msg);
        const std::string reason =
            applied.reason.empty() ? std::string("unknown") : applied.reason;
        if (rb < 0) {
            res->error_code = NS::E_CB_ALIGN_ROLLBACK_FAILED;
            res->message = "align_map_tool failed (" + reason + ") AND rollback failed (" +
                           rb_msg + "); manual recovery required";
            RCLCPP_FATAL(get_logger(), "[Align] %s", res->message.c_str());
        } else {
            res->error_code = NS::E_CB_ALIGN_TOOL_FAILED;
            res->message = "align_map_tool failed (" + reason + "); " + rb_msg;
        }
        RCLCPP_ERROR(get_logger(), "[Align] %s\n%s", res->message.c_str(),
                     applied.log.c_str());
        restart_localization();
        return;
    }
    res->residual_yaw_deg = applied.residual_yaw_deg;
    RCLCPP_INFO(get_logger(), "[Align] tool applied, residual=%.4f deg",
                applied.residual_yaw_deg);

    // ── 步骤 ⑥：transform_assets_tool（同一角度，纯旋转 ⇒ x=y=0）──
    if (!req->skip_assets) {
        const AssetsToolResult assets = RunTransformAssetsTool(map_name, 0.0, 0.0, est.yaw_rad);
        const bool assets_ok = assets.ok;
        const std::string assets_msg = assets.Summary();
        res->transformed_points = assets.points;
        res->transformed_areas = assets.areas;
        res->transformed_junctions = assets.junctions;

        // ── 步骤 ⑦：资产失败 → 回滚底图，保证两者始终一致 ─────────
        //    资产工具原子写回，失败时资产文件未动，只需回滚底图
        if (!assets_ok) {
            std::string rb_map_msg;
            const bool rb_map = RestoreSlamBackup(map_name, rb_map_msg);
            if (rb_map) {
                res->error_code = NS::E_CB_ALIGN_ASSETS_FAILED;
                res->message = "transform_assets failed (" + assets_msg +
                               "); rolled back: " + rb_map_msg;
                RCLCPP_ERROR(get_logger(), "[Align] %s", res->message.c_str());
            } else {
                res->error_code = NS::E_CB_ALIGN_ROLLBACK_FAILED;
                res->message = "transform_assets failed (" + assets_msg +
                               ") AND rollback failed [map: " + rb_map_msg +
                               "]. Base map and assets may be INCONSISTENT; "
                               "manual recovery required.";
                RCLCPP_FATAL(get_logger(), "[Align] %s", res->message.c_str());
            }
            restart_localization();
            return;
        }
        RCLCPP_INFO(get_logger(), "[Align] transform_assets OK: %s", assets_msg.c_str());
    } else {
        RCLCPP_WARN(get_logger(),
                    "[Align] skip_assets=true: base map rotated but assets left in "
                    "the OLD frame. They are now INCONSISTENT.");
    }

    // ── 步骤 ⑧：当前地图 → 重载 + 启定位 ────────────────────────
    res->applied = true;
    res->success = true;
    res->message = "aligned by " + std::to_string(est.yaw_deg) +
                   " deg, residual " + std::to_string(applied.residual_yaw_deg) +
                   " deg";
    if (is_current) {
        if (map_command_cli_->service_is_ready()) {
            auto mc = std::make_shared<nav_interfaces::srv::MapManagerCommand::Request>();
            mc->op = nav_interfaces::srv::MapManagerCommand::Request::OP_LOAD;
            mc->map_name = map_name;
            auto fut = map_command_cli_->async_send_request(mc);
            if (fut.wait_for(std::chrono::duration<double>(service_call_timeout_sec_)) !=
                std::future_status::ready) {
                // 数据已一致，只是没重载成功：降级为一级故障，不算失败。
                res->error_code = NS::E_CB_MAP_RELOAD_TIMEOUT;
                res->message += " (map reload timed out; reload manually)";
                RCLCPP_WARN(get_logger(), "[Align] map reload timed out");
            }
        }
        restart_localization();
    }
    RCLCPP_INFO(get_logger(), "[Align] done: %s", res->message.c_str());
}

// ══════════════════════════════════════════════════════════════════════
// 地图替换编排
// 与对齐同一套串行编排与回滚；坐标系沿用 target，资产不动（见 hpp 注释）。
// ══════════════════════════════════════════════════════════════════════

CommandBoardManagerROS::ReplaceToolResult CommandBoardManagerROS::RunReplaceMapTool(
    const std::string& target_map, const std::string& source_map, const bool dry_run,
    const bool manual, const double x, const double y, const double yaw_rad) {
    ReplaceToolResult out;
    const std::string tgt_dir = ResolveMapDir(target_map);
    const std::string src_dir = ResolveMapDir(source_map);
    if (tgt_dir.empty() || src_dir.empty()) {
        out.reason = "map_dir_unresolved";
        out.log = "cannot resolve map directory";
        return out;
    }

    std::vector<std::string> args = {"ros2", "run", "cartographer_ros", "replace_map_tool",
                                     "--map_target", tgt_dir + "/slam", "--map_source", src_dir + "/slam"};
    if (manual) {
        for (const auto& a : {std::string("--transform_mode"), std::string("manual"),
                              std::string("--src_in_tgt_x"), FormatFixed(x, 10),
                              std::string("--src_in_tgt_y"), FormatFixed(y, 10),
                              std::string("--src_in_tgt_yaw"), FormatFixed(yaw_rad, 10)}) {
            args.push_back(a);
        }
    } else {
        args.push_back("--transform_mode");
        args.push_back("auto");
    }
    if (dry_run) args.push_back("--dry_run");
    RCLCPP_INFO(get_logger(), "RunReplaceMapTool: %s", CommandLineForLog(args).c_str());

    const int rc = RunProcessCapture(args, &out.log);
    if (rc < 0) {
        out.reason = "spawn_failed";
        return out;
    }
    out.process_ok = (rc == 0);

    // 取最后一条 [REPLACE_RESULT]：工具在失败路径会先打结果行再 LOG(FATAL)，
    // 与对齐同一约定（见 RunAlignMapTool）。
    constexpr const char* kMarker = "[REPLACE_RESULT]";
    const size_t marker_pos = out.log.rfind(kMarker);
    if (marker_pos != std::string::npos) {
        const size_t line_end = out.log.find('\n', marker_pos);
        const std::string json = out.log.substr(
            marker_pos, line_end == std::string::npos ? std::string::npos
                                                      : line_end - marker_pos);
        JsonGetBool(json, "ok", &out.ok);
        JsonGetString(json, "reason", &out.reason);
        JsonGetDouble(json, "x", &out.x);
        JsonGetDouble(json, "y", &out.y);
        JsonGetDouble(json, "yaw_rad", &out.yaw_rad);
        JsonGetDouble(json, "yaw_deg", &out.yaw_deg);
        JsonGetInt(json, "inliers", &out.inliers);
        JsonGetBool(json, "converged", &out.converged);
        JsonGetBool(json, "degraded", &out.degraded);
        JsonGetString(json, "stop_reason", &out.stop_reason);
        JsonGetDouble(json, "inlier_rms_xy_m", &out.inlier_rms_xy_m);
        JsonGetDouble(json, "inlier_rms_yaw_deg", &out.inlier_rms_yaw_deg);
        JsonGetDouble(json, "src_known_area_m2", &out.src_known_area_m2);
        JsonGetDouble(json, "tgt_known_area_m2", &out.tgt_known_area_m2);
        JsonGetBool(json, "coverage_warning", &out.coverage_warning);
        JsonGetDouble(json, "total_area_m2", &out.total_area_m2);
        JsonGetDouble(json, "effective_area_m2", &out.effective_area_m2);
        JsonGetString(json, "data_version", &out.data_version);
        JsonGetBool(json, "align_evaluated", &out.align_evaluated);
        JsonGetDouble(json, "align_residual_yaw_deg", &out.align_residual_yaw_deg);
        JsonGetString(json, "backup_dir", &out.backup_dir);
        JsonGetBool(json, "applied", &out.applied);
    } else {
        out.reason = "no_result_line";
        RCLCPP_ERROR(get_logger(),
                     "replace_map_tool printed no %s line; tool version mismatch?", kMarker);
    }
    if (!out.process_ok) {
        RCLCPP_ERROR(get_logger(), "replace_map_tool exited with code %d (reason=%s)", rc,
                     out.reason.c_str());
    }
    return out;
}

void CommandBoardManagerROS::ReplaceMapCallback(
    const std::shared_ptr<nav_interfaces::srv::ReplaceMap::Request> req,
    std::shared_ptr<nav_interfaces::srv::ReplaceMap::Response> res) {
    using NS = nav_interfaces::msg::NavSystemStatus;

    res->success = false;
    res->error_code = NS::E_CB_NONE;
    res->applied = false;

    // ── 互斥：与对齐 / 重置原点共用一把锁 ─────────────────────────
    bool expected = false;
    if (!map_transform_busy_.compare_exchange_strong(expected, true)) {
        res->error_code = NS::E_CB_ALIGN_BUSY;
        res->message = "another map transform task is running";
        return;
    }
    struct BusyGuard {
        std::atomic<bool>* flag;
        ~BusyGuard() { flag->store(false); }
    } guard{&map_transform_busy_};

    // ── 入参校验（零文件改动）────────────────────────────────────
    const std::string target =
        req->target_map_name.empty() ? current_map_name_ : req->target_map_name;
    const std::string source = req->source_map_name;
    if (target.empty()) {
        res->error_code = NS::E_CB_REPLACE_MAP_NOT_FOUND;
        res->message = "target_map_name is empty and no map is currently loaded";
        return;
    }
    if (source.empty()) {
        res->error_code = NS::E_CB_REPLACE_ARG_INVALID;
        res->message = "source_map_name is required";
        return;
    }
    if (source == target) {
        res->error_code = NS::E_CB_REPLACE_ARG_INVALID;
        res->message = "source_map_name must differ from target_map_name";
        return;
    }
    for (const auto& name : {target, source}) {
        const std::string pb = ResolveMapDir(name) + "/slam/map.pbstream";
        std::error_code fs_ec;
        if (ResolveMapDir(name).empty() || !cbfs::is_regular_file(pb, fs_ec) || fs_ec) {
            res->error_code = NS::E_CB_REPLACE_MAP_NOT_FOUND;
            res->message = "map not found (no slam/map.pbstream): " + name;
            return;
        }
    }
    const bool target_is_current = (target == current_map_name_);
    const bool source_is_current = (source == current_map_name_);
    const bool manual = req->use_manual_transform;
    RCLCPP_INFO(get_logger(),
                "[Replace] target='%s' (current=%s) source='%s' dry_run=%d manual=%d",
                target.c_str(), target_is_current ? "yes" : "no", source.c_str(),
                static_cast<int>(req->dry_run), static_cast<int>(manual));

    // ── 步骤 ①：dry_run 求变换（零文件改动）──────────────────────
    const ReplaceToolResult est = RunReplaceMapTool(target, source, /*dry_run=*/true, manual,
                                                    req->manual_x, req->manual_y,
                                                    req->manual_yaw);
    res->x = est.x;
    res->y = est.y;
    res->yaw_deg = est.yaw_deg;
    res->inliers = est.inliers;
    res->converged = est.converged;
    res->degraded = est.degraded;
    res->stop_reason = est.stop_reason;
    res->inlier_rms_xy_m = est.inlier_rms_xy_m;
    res->inlier_rms_yaw_deg = est.inlier_rms_yaw_deg;
    res->src_known_area_m2 = est.src_known_area_m2;
    res->tgt_known_area_m2 = est.tgt_known_area_m2;
    res->coverage_warning = est.coverage_warning;
    res->data_version = est.data_version;
    if (!est.usable()) {
        res->error_code = NS::E_CB_REPLACE_MATCH_FAILED;
        res->message = "transform estimation failed (" +
                       (est.reason.empty() ? std::string("unknown") : est.reason) +
                       "); no file was modified";
        RCLCPP_ERROR(get_logger(), "[Replace] %s", res->message.c_str());
        return;
    }
    RCLCPP_INFO(get_logger(),
                "[Replace] T_src_in_tgt=(%.4f, %.4f, %.4f deg) inliers=%d converged=%d "
                "degraded=%d rms=(%.4f m, %.4f deg) coverage_warning=%d",
                est.x, est.y, est.yaw_deg, est.inliers, static_cast<int>(est.converged),
                static_cast<int>(est.degraded), est.inlier_rms_xy_m, est.inlier_rms_yaw_deg,
                static_cast<int>(est.coverage_warning));

    // ── 步骤 ②：质量校验 / dry_run 出口 ─────────────────────────
    std::string notes;
    if (est.coverage_warning) {
        notes += "; WARNING: source covers much less area than target, the site may not "
                 "have been fully re-walked";
    }
    if (!manual && est.degraded && !req->accept_degraded) {
        res->error_code = NS::E_CB_REPLACE_MATCH_DEGRADED;
        res->message = "match did not converge (stop_reason=" + est.stop_reason +
                       "); set accept_degraded=true to apply anyway; no file was modified" +
                       notes;
        RCLCPP_WARN(get_logger(), "[Replace] %s", res->message.c_str());
        return;
    }
    if (req->dry_run) {
        res->success = true;
        res->message = "dry run: transform estimated only, no file modified" + notes;
        return;
    }

    // ── 步骤 ③：target 为当前地图才需要停导航 / 停定位 ──────────
    if (target_is_current) {
        RCLCPP_INFO(get_logger(), "[Replace] Step 3: cancel navigation + stop localization");
        navigation_action_client_->async_cancel_all_goals();
        std::this_thread::sleep_for(300ms);
        if (stop_localization_cli_->service_is_ready()) {
            auto stop_req = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto fut = stop_localization_cli_->async_send_request(stop_req);
            if (fut.wait_for(std::chrono::duration<double>(service_call_timeout_sec_)) !=
                std::future_status::ready) {
                res->error_code = NS::E_CB_LOC_STOP_TIMEOUT;
                res->message = "timeout waiting for stop_localization; no file modified";
                RCLCPP_ERROR(get_logger(), "[Replace] %s", res->message.c_str());
                return;
            }
        } else {
            RCLCPP_WARN(get_logger(), "[Replace] stop_localization not ready, skipping");
        }
    }
    const auto restart_localization = [&]() {
        if (!target_is_current) return;
        if (start_localization_cli_->service_is_ready()) {
            auto r = std::make_shared<std_srvs::srv::Trigger::Request>();
            start_localization_cli_->async_send_request(r);
        }
    };

    // ── 步骤 ④：正式执行。把 dry_run 的变换以 manual 模式传入，执行的就是
    //    调用方看到并认可的那个变换，不再重新匹配。工具先把 target 底图
    //    备份到 slam/backups/backup_<ts>/ 再写入。
    const std::set<std::string> backups_before = ListSlamBackupDirs(target);
    const ReplaceToolResult applied =
        RunReplaceMapTool(target, source, /*dry_run=*/false, /*manual=*/true, est.x, est.y,
                          est.yaw_rad);

    // ── 步骤 ⑤：失败 → 用本次创建的备份回滚底图 ─────────────────
    //    不能只看结果行里的 backup_dir：工具异常退出时可能没有结果行，
    //    但文件已经写了一半。以执行前后 backups/ 的差集兜底。
    if (!applied.usable()) {
        const std::string reason =
            applied.reason.empty() ? std::string("unknown") : applied.reason;
        std::string rb_msg;
        const int rb =
            RollbackSlamAfterToolFailure(target, backups_before, applied.backup_dir, rb_msg);
        if (rb < 0) {
            res->error_code = NS::E_CB_REPLACE_ROLLBACK_FAILED;
            res->message = "replace_map_tool failed (" + reason + ") AND rollback failed (" +
                           rb_msg + "). Target base map may be incomplete; restore it "
                           "manually from slam/backups/";
            RCLCPP_FATAL(get_logger(), "[Replace] %s", res->message.c_str());
        } else {
            res->error_code = NS::E_CB_REPLACE_TOOL_FAILED;
            res->message = "replace_map_tool failed (" + reason + "); " +
                           (rb == 1 ? "rolled back: " : "") + rb_msg;
        }
        RCLCPP_ERROR(get_logger(), "[Replace] %s\n%s", res->message.c_str(),
                     applied.log.c_str());
        restart_localization();
        return;
    }

    res->applied = true;
    res->success = true;
    res->total_area_m2 = applied.total_area_m2;
    res->effective_area_m2 = applied.effective_area_m2;
    if (!applied.data_version.empty()) res->data_version = applied.data_version;
    res->align_evaluated = applied.align_evaluated;
    res->align_residual_yaw_deg = applied.align_residual_yaw_deg;
    // 替换不触发对齐：残余角大说明 target 坐标系本身没对齐，交由人决定是否单独 ~/align_map。
    res->align_warning =
        applied.align_evaluated && std::abs(applied.align_residual_yaw_deg) > align_min_yaw_deg_;
    if (res->align_warning) {
        notes += "; NOTE: ALIGN-01 residual " + std::to_string(applied.align_residual_yaw_deg) +
                 " deg exceeds " + std::to_string(align_min_yaw_deg_) +
                 " deg, the target frame is not aligned; call ~/align_map if needed";
    }
    res->message = "replaced '" + target + "' with content of '" + source + "'";
    RCLCPP_INFO(get_logger(), "[Replace] tool applied: effective area %.3f m2, data_version %s",
                applied.effective_area_m2, res->data_version.c_str());

    // ── 步骤 ⑥：target 为当前地图 → 重载 + 启定位 ───────────────
    if (target_is_current) {
        if (map_command_cli_->service_is_ready()) {
            auto mc = std::make_shared<nav_interfaces::srv::MapManagerCommand::Request>();
            mc->op = nav_interfaces::srv::MapManagerCommand::Request::OP_LOAD;
            mc->map_name = target;
            auto fut = map_command_cli_->async_send_request(mc);
            if (fut.wait_for(std::chrono::duration<double>(service_call_timeout_sec_)) !=
                std::future_status::ready) {
                res->error_code = NS::E_CB_MAP_RELOAD_TIMEOUT;
                notes += "; map reload timed out, reload manually";
                RCLCPP_WARN(get_logger(), "[Replace] map reload timed out");
            }
        }
        restart_localization();
    }

    // ── 步骤 ⑦：source 移入回收站 ───────────────────────────────
    if (req->keep_source) {
        notes += "; source kept";
    } else if (source_is_current) {
        notes += "; source not trashed because it is the currently loaded map";
    } else {
        std::string trash_msg;
        bool trashed = false;
        if (map_command_cli_->service_is_ready()) {
            auto mc = std::make_shared<nav_interfaces::srv::MapManagerCommand::Request>();
            mc->op = nav_interfaces::srv::MapManagerCommand::Request::OP_TRASH;
            mc->map_name = source;
            auto fut = map_command_cli_->async_send_request(mc);
            if (fut.wait_for(std::chrono::duration<double>(service_call_timeout_sec_)) ==
                std::future_status::ready) {
                const auto r = fut.get();
                trashed = r->success;
                trash_msg = r->message;
            } else {
                trash_msg = "timeout waiting for map_command";
            }
        } else {
            trash_msg = "map_command service not available";
        }
        if (trashed) {
            // map_manager 的成功消息形如 "map trashed: <name> -> <entry>"
            const size_t arrow = trash_msg.rfind("-> ");
            res->source_trashed_to =
                arrow == std::string::npos ? std::string() : trash_msg.substr(arrow + 3);
            notes += "; source moved to trash as '" + res->source_trashed_to + "'";
        } else {
            // 替换本身已成功，只把 source 的去向降级为一级故障。
            if (res->error_code == NS::E_CB_NONE) {
                res->error_code = NS::E_CB_REPLACE_SOURCE_TRASH_FAILED;
            }
            notes += "; failed to move source to trash (" + trash_msg + ")";
            RCLCPP_WARN(get_logger(), "[Replace] trash source failed: %s", trash_msg.c_str());
        }
    }
    res->message += notes;
    RCLCPP_INFO(get_logger(), "[Replace] done: %s", res->message.c_str());
}

}  // namespace command_board