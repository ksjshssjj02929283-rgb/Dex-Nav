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
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

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
    react_to_estop_  = this->declare_parameter("react_to_estop", react_to_estop_);
    react_to_bumper_ = this->declare_parameter("react_to_bumper", react_to_bumper_);

    // 单一状态源：/diagnostics_agg、/localization/state 由 StatusMonitor 唯一订阅聚合，
    // 本节点不再重复订阅，改在 OnRecoverTimer 轮询 GetSnapshot 读取。
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    navigation_action_client_ =
        rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "/navigate_to_pose");

    // ── TF buffer（用于位姿采样）─────────────────────────────────
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ── 服务客户端 ────────────────────────────────────────────────
    stop_localization_cli_ = this->create_client<std_srvs::srv::Trigger>("/localization/stop_localization");
    start_localization_cli_ = this->create_client<std_srvs::srv::Trigger>("/localization/start_localization");
    map_command_cli_ = this->create_client<nav_interfaces::srv::MapManagerCommand>("map_manager/map_command");
    transform_assets_cli_ =
        this->create_client<nav_interfaces::srv::TransformAssets>("/map_asset_manager/transform_assets");

    // ── 订阅当前地图状态（map_state，folder+name+epoch 原子下发）──────────
    auto map_state_qos = rclcpp::QoS(1).transient_local().reliable();
    map_state_sub_ = this->create_subscription<nav_interfaces::msg::MapState>(
        "map_manager/map_state", map_state_qos,
        [this](const nav_interfaces::msg::MapState::SharedPtr msg) { current_map_name_ = msg->map_name; });

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

    // 地图版本备份 + 回退单入口：~/backup_unified、~/rollback_map
    map_version_manager_ = std::make_unique<MapVersionManager>(this);

    // 状态聚合 + 条件自动保存（StatusMonitor 提供快照，AutoMapSaver 日级触发 + ~/save_map 手动）
    status_monitor_ = std::make_unique<StatusMonitor>(this);
    auto_map_saver_ = std::make_unique<AutoMapSaver>(this, status_monitor_.get());

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

    RCLCPP_INFO(get_logger(),
                "CommandBoardManagerROS initialized. "
                "Services: ~/reset_map_origin_by_xyyaw, ~/reset_map_origin_by_robot_pose, "
                "~/backup_unified, ~/rollback_map");
}

// ──────────────────────────────────────────────────────────────────────
// 外部服务入口
// ──────────────────────────────────────────────────────────────────────

// 入口①：按 xyyaw 重置任意地图
//   - 传入 map_name（必填），不限于当前使用中的地图
//   - 只做：① subprocess 修改 pbstream/png/yaml
//            ② transform_assets 修改地图资产文件
//   - 不停/启定位，不重载地图
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

    // ── 步骤①：subprocess 修改 pbstream/png/yaml ─────────────────
    std::string tool_log;
    if (!RunResetMapOriginTool(req->map_name, req->x, req->y, req->yaw, tool_log)) {
        res->success = false;
        res->message = "reset_map_origin_tool failed: " + tool_log;
        RCLCPP_ERROR(get_logger(), "[XYYaw] %s", res->message.c_str());
        return;
    }
    RCLCPP_INFO(get_logger(), "[XYYaw] reset_map_origin_tool OK");

    // ── 步骤②：transform_assets 修改地图资产文件 ─────────────────
    // 需在子线程中发送请求，避免单线程执行器死锁
    bool assets_ok = false;
    std::string assets_msg;
    std::thread t([&]() {
        if (!transform_assets_cli_->wait_for_service(std::chrono::duration<double>(service_wait_timeout_sec_))) {
            assets_msg = "transform_assets service not available";
            return;
        }
        auto ta_req = std::make_shared<nav_interfaces::srv::TransformAssets::Request>();
        ta_req->map_name = req->map_name;
        ta_req->x = req->x;
        ta_req->y = req->y;
        ta_req->yaw = req->yaw;
        auto fut = transform_assets_cli_->async_send_request(ta_req);
        if (fut.wait_for(std::chrono::duration<double>(service_call_timeout_sec_)) != std::future_status::ready) {
            assets_msg = "Timeout waiting for transform_assets";
            return;
        }
        auto resp = fut.get();
        assets_ok = resp->success;
        assets_msg = resp->message;
    });
    t.join();

    res->success = assets_ok;
    res->message = assets_ok ? "OK: " + assets_msg : "transform_assets failed: " + assets_msg;
    res->applied_x = req->x;
    res->applied_y = req->y;
    res->applied_yaw = req->yaw;

    if (assets_ok) {
        RCLCPP_INFO(get_logger(), "[XYYaw] Done: %s", res->message.c_str());
    } else {
        RCLCPP_ERROR(get_logger(), "[XYYaw] %s", res->message.c_str());
    }
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
// 步骤：停导航 → 停定位 → 并行(tool + transform_assets) → 重载地图 → 启定位
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

    // ── 步骤 ③④：并行执行 ────────────────────────────────────────
    //   ③ subprocess: reset_map_origin_tool（修改 pbstream/png/yaml）
    //   ④ transform_assets（修改内存坐标 + 持久化）
    RCLCPP_INFO(get_logger(), "[Reset] Steps 3+4 (parallel): tool + transform_assets");

    std::string tool_log;
    bool tool_ok = false;
    bool assets_ok = false;
    std::string assets_msg;

    // 线程 A：reset_map_origin_tool
    std::thread tool_thread([&]() { tool_ok = RunResetMapOriginTool(current_map_name_, x, y, yaw, tool_log); });

    // 线程 B：transform_assets 服务调用
    std::thread assets_thread([&]() {
        if (!transform_assets_cli_->service_is_ready()) {
            // 等最多 3s
            if (!transform_assets_cli_->wait_for_service(std::chrono::duration<double>(service_wait_timeout_sec_))) {
                assets_msg = "transform_assets service not available";
                assets_ok = false;
                return;
            }
        }
        auto req = std::make_shared<nav_interfaces::srv::TransformAssets::Request>();
        req->map_name = current_map_name_;
        req->x = x;
        req->y = y;
        req->yaw = yaw;
        auto fut = transform_assets_cli_->async_send_request(req);
        if (fut.wait_for(std::chrono::duration<double>(service_call_timeout_sec_)) != std::future_status::ready) {
            assets_msg = "Timeout waiting for transform_assets";
            assets_ok = false;
            return;
        }
        auto resp = fut.get();
        assets_ok = resp->success;
        assets_msg = resp->message;
    });

    tool_thread.join();
    assets_thread.join();

    if (!tool_ok) {
        RCLCPP_ERROR(get_logger(), "[Reset] reset_map_origin_tool failed: %s", tool_log.c_str());
        // 工具失败仍继续（assets 已变换），记录警告后重启定位
    } else {
        RCLCPP_INFO(get_logger(), "[Reset] reset_map_origin_tool OK");
    }
    if (!assets_ok) {
        out_msg = "transform_assets failed: " + assets_msg;
        RCLCPP_ERROR(get_logger(), "[Reset] %s", out_msg.c_str());
        // assets 变换失败是硬错误，但仍需重启定位
        // 此处不提前 return，继续走重载地图流程以保持系统可用
    } else {
        RCLCPP_INFO(get_logger(), "[Reset] transform_assets OK: %s", assets_msg.c_str());
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

    if (!tool_ok || !assets_ok) {
        out_msg = "Partial failure — tool_ok=" + std::to_string(tool_ok) + " assets_ok=" + std::to_string(assets_ok) +
                  " tool_log=[" + tool_log + "] assets=[" + assets_msg + "]";
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

// ──────────────────────────────────────────────────────────────────────
// 子进程调用 reset_map_origin_tool
// ──────────────────────────────────────────────────────────────────────

bool CommandBoardManagerROS::RunResetMapOriginTool(
    const std::string& map_name, double x, double y, double yaw, std::string& out_log) {
    // 构造命令（依赖 source install/setup.bash 后 PATH 含工具）
    char cmd[1024];
    std::snprintf(cmd, sizeof(cmd),
                  "ros2 run cartographer_ros reset_map_origin_tool"
                  " --map_name \"%s\""
                  " --new_x %.6f --new_y %.6f --new_yaw %.6f"
                  " 2>&1",
                  map_name.c_str(), x, y, yaw);

    RCLCPP_INFO(get_logger(), "RunResetMapOriginTool: %s", cmd);

    FILE* pipe = popen(cmd, "r");
    if (!pipe) {
        out_log = "popen failed";
        return false;
    }
    char buf[256];
    while (fgets(buf, sizeof(buf), pipe)) {
        out_log += buf;
    }
    const int rc = pclose(pipe);
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

    // 3. 急停/碰撞翻转检测（从 StatusMonitor 聚合的安全状态读）。
    const bool active = (react_to_estop_ && snap.is_estop_active) ||
                        (react_to_bumper_ && (snap.bumper_front_active || snap.bumper_rear_active));
    if (active && !safety_active_) {
        navigation_action_client_->async_cancel_all_goals();
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

}  // namespace command_board