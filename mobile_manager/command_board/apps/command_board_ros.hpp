
// ----------------------------------------------------------------------------
//
// Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
//
// All Rights Reserved.
//
//    Authors: Li Yao <liyao2@dexforce.top>
//
// ----------------------------------------------------------------------------

#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <iostream>
#include <atomic>
#include <memory>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <vector>

#include "localization_sm.h"
#include "nav_interfaces/msg/localization_state.hpp"
#include "nav_interfaces/msg/map_state.hpp"
#include <nav_interfaces/srv/map_manager_command.hpp>
#include "nav_interfaces/srv/relocalize.hpp"
#include "nav_interfaces/srv/reset_map_origin_by_xy_yaw.hpp"
#include "nav_interfaces/srv/transform_assets.hpp"
#include "map_version_manager.hpp"
#include "status_monitor.hpp"
#include "sensor_monitor.hpp"
#include "auto_map_saver.hpp"

namespace command_board {

class CommandBoardManagerROS : public rclcpp::Node {
public:
    CommandBoardManagerROS();
    ~CommandBoardManagerROS() = default;

    void StopRobotMovement();

    void HandleLocalizationFailed();

    void HandleLocalizationSuccess();

private:
    // ── 重置地图原点编排 ────────────────────────────────────────────
    // 入口①：按指定 x/y/yaw 重置任意地图（不停定位，不重载地图）
    void ResetMapOriginByXYYawCallback(const std::shared_ptr<nav_interfaces::srv::ResetMapOriginByXYYaw::Request> req,
                                       std::shared_ptr<nav_interfaces::srv::ResetMapOriginByXYYaw::Response> res);

    // 入口②：采样当前机器人位姿，只操作当前加载地图，完整流程（停定位→工具→重载→启定位）
    void ResetMapOriginByRobotPoseCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                           std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    // 内部：完整重置流程（仅用于当前地图 + RobotPose 入口）
    bool DoResetCurrentMapOrigin(double x, double y, double yaw, std::string& out_msg);

    /// 当前版本不对外提供运行时切换入口（SetNavigationBackend 服务未开放），本函数暂无调用者，保留备用。
    /// 执行一次后端切换：取消导航 → PAUSE 旧后端 → STARTUP 新后端，失败回滚。
    /// 同步执行（调用方拿到应答时新后端已 active），故只能跑在 switch_group_ 这个
    /// Reentrant 回调组里——阻塞等管理器应答期间，应答本身也要有线程去处理
    bool SwitchNavigationBackend(const std::string& target, bool force, std::string& msg);

    /// 给某个后端的生命周期管理器发一条命令并等应答。
    /// \param command ManageLifecycleNodes 的 STARTUP/PAUSE/...；err 带失败原因
    bool CallBackendManager(
        const std::string& backend, uint8_t command, std::string& err);

    /// 取某个后端的管理器客户端；未知后端返回 nullptr
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr ManagerFor(
        const std::string& backend) const;

    /// 把当前生效的后端发到 ~/navigation_backend（latched），给 UI/上层看
    void PublishBackendState();

    /// 启动期后端编排：navigation_backend=path_navigation 且 autostart 时，等依赖就绪后
    /// 调 lifecycle_manager_path_navigation 的 STARTUP 把两个 Server 拉到 active。
    ///
    /// 做成 1Hz 轮询的状态机而不是构造期阻塞等待：本节点起得比导航栈早是常态，
    /// 构造函数里 wait_for_service 会把整个 launch 卡住（且参数回调、定时器全还没起）。
    void OnBackendStartupTimer();

    /// 取消 schedule_planner 正在执行的路网任务（NavigateThroughGraphPoints）。
    ///
    /// 为什么需要单独一刀：navigation_action_client_ 只认 /navigate_to_pose，而路网任务
    /// 跑在另一套 action 上，不取消的话急停后 goal 仍是 active——车停了、任务没结束，
    /// 要等 schedule_planner 的 segment_timeout（默认 300 s）才超时收尾；期间松开急停
    /// 且定位恢复的话，path_follower 的 route_ 还在，会直接接着原路径继续走。
    ///
    /// 异步单发、不等应答：调用点在 200 ms 定时器和重置流程里，都不能阻塞。
    void CancelSchedulePlannerRoute(const std::string& why);

    // 内部：采样 TF map→base_link，返回 N 帧均值（循环等待最多 timeout_sec）
    bool SampleRobotPose(int n_frames, double timeout_sec, double& out_x, double& out_y, double& out_yaw);

    // 内部：调用子进程 reset_map_origin_tool
    bool RunResetMapOriginTool(const std::string& map_name, double x, double y, double yaw, std::string& out_log);

    // Tunable parameters (declared in ctor).
    int    reset_origin_sample_frames_{11};
    double reset_origin_sample_timeout_sec_{5.0};
    double service_call_timeout_sec_{15.0};
    double service_wait_timeout_sec_{3.0};
    bool   react_to_estop_{true};
    bool   react_to_bumper_{true};
    /// 急停/碰撞（以及重置地图原点）时是否一并取消 schedule_planner 的路网任务。
    /// 取消走它的 force_cancel 服务而不是 action cancel：该服务无论有没有 active goal
    /// 都会兜底停车，且本节点不必 depend nav_interfaces 去拿 action 类型
    bool   cancel_schedule_planner_{true};
    std::string schedule_planner_cancel_service_{"/schedule_planner/force_cancel"};

    // ── 导航后端选择（唯一真值源，仅启动期读取）─────────────────────
    // "nav2" = 原 Nav2 bringup + /navigate_to_pose；
    // "path_navigation" = schedule_planner + path_follower 那套。
    // 刻意不再提供第二个 path_navigation.enabled：两个开关会拼出互相矛盾的组合。
    // 运行中 ros2 param set 不切后端——切换要处理任务取消、零速、速度源互斥与失败回滚，
    // 那是一个独立的 Action/Service 状态机，不是改个参数的事。
    std::string navigation_backend_{"nav2"};

    bool   path_nav_autostart_{true};
    std::string path_nav_manager_service_{"/lifecycle_manager_path_navigation/manage_nodes"};
    double path_nav_startup_timeout_sec_{60.0};
    /// true = 等到 MapState 有地图名再 STARTUP（路网任务离了地图没意义）；
    /// false = 管理器一就绪就 STARTUP（默认，避免不发 MapState 的机型永远起不来）
    bool   path_nav_require_map_ready_{false};

    // Nav2 后端。默认 false：nav2_bringup 的 autostart 现在还是 True，它自己会把整套拉起来，
    // 本节点再发一次 STARTUP 只是多余噪声。只有把 w1_nav2_actual.launch.py 起成
    // autostart:=false（两套后端同时常驻的模式）时，才需要把这项设成 true。
    bool   nav2_autostart_{false};
    std::string nav2_manager_service_{"/lifecycle_manager_navigation/manage_nodes"};
    double nav2_startup_timeout_sec_{60.0};

    /// 运行时切换的单次超时（秒）：PAUSE 与 STARTUP 各按它等
    double backend_switch_timeout_sec_{20.0};

    // ── 启动期编排用（按 navigation_backend 收敛）──
    /// 被选中后端的管理器服务名；空串 = 本次不做编排（未选中/autostart=false）
    std::string backend_manager_service_;
    double backend_startup_timeout_sec_{60.0};
    bool   backend_require_map_ready_{false};
    rclcpp::Time backend_startup_begin_;
    rclcpp::TimerBase::SharedPtr backend_startup_timer_;

    // 两套管理器的客户端都建好：切换要同时操作"旧的"和"新的"
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr path_nav_manager_cli_;
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr nav2_manager_cli_;
    /// 切换服务与两个管理器客户端共用的 Reentrant 组（见 SwitchNavigationBackend 注释）
    rclcpp::CallbackGroup::SharedPtr switch_group_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr backend_state_pub_;
    /// 被选中的后端**当前确实是 active**（STARTUP 成功过且之后没切走）。
    /// 启动 STARTUP 失败/超时时仍是 false —— 这样之后再调切换服务会真去重试，
    /// 而不是被"已经是这个后端了"挡回去
    std::atomic<bool> backend_active_{false};
    /// 启动期 STARTUP 请求在途标记：定时器每秒一跳，不能在上一发没回来时又发一发
    std::atomic<bool> backend_startup_inflight_{false};
    /// 参数回调句柄：用来拒绝运行时改 navigation_backend（改了也不生效，必须明确告诉人）
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr backend_param_cb_;

    rclcpp::CallbackGroup::SharedPtr callback_group_;

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;

    // 定位控制 & 地图管理服务客户端
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr schedule_planner_cancel_cli_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_localization_cli_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_localization_cli_;
    rclcpp::Client<nav_interfaces::srv::MapManagerCommand>::SharedPtr map_command_cli_;
    rclcpp::Client<nav_interfaces::srv::TransformAssets>::SharedPtr transform_assets_cli_;

    // 对外暴露的重置原点服务
    rclcpp::Service<nav_interfaces::srv::ResetMapOriginByXYYaw>::SharedPtr reset_origin_xyyaw_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_origin_robot_pose_srv_;

    // TF buffer（用于采样机器人位姿）
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 当前地图名称（从 map_manager/map_state 话题获取）
    std::string current_map_name_;
    rclcpp::Subscription<nav_interfaces::msg::MapState>::SharedPtr map_state_sub_;

    std::unique_ptr<Localization_SM> localization_sm_;

    // 地图版本备份 + 回退单入口（移植自 nav2_command_board）
    std::unique_ptr<MapVersionManager> map_version_manager_;

    // 状态聚合 + 条件自动保存地图（移植自 nav2_command_board）
    std::unique_ptr<StatusMonitor> status_monitor_;
    std::unique_ptr<AutoMapSaver>  auto_map_saver_;

    // ── 传感器监测（CE：导航必要传感器失效 → 取消导航 + 故障码）──────
    // 独立回调组 + 独立定时器：不与 OnRecoverTimer / 重置原点服务共用的
    // 互斥回调组串行，避免地图操作（含 sleep 与最长约 15 s 等待）期间失明。
    rclcpp::CallbackGroup::SharedPtr sensor_monitor_callback_group_;
    std::unique_ptr<SensorMonitor>   sensor_monitor_;

    // ── 急停丢位恢复（estop 断里程计会丢位，激光雷达照常工作）──────────
    enum class RecoverState { kNormal, kSafeStop, kRelocating, kGlobalReloc, kAlarm };
    void OnRecoverTimer();
    void SnapshotGoodPose();
    bool RobotIsStill() const;
    void StopLocalization();
    void StartLocalization();
    void Relocalize(const geometry_msgs::msg::Pose2D& pose);
    void GlobalRelocalize();
    void RaiseAlarm(const std::string& reason);
    void PushRecoverStatus();
    static const char* RecoverStateToString(RecoverState s);

    bool   enable_loss_recovery_{true};
    double recover_settle_sec_{1.0};
    double reloc_wait_sec_{2.0};
    double score_good_{0.8};
    double score_bad_{0.5};
    double pose_cache_min_score_{0.6};
    double velocity_still_threshold_{0.05};
    RecoverState recover_state_{RecoverState::kNormal};
    bool         safety_active_{false};
    geometry_msgs::msg::Pose2D last_known_pose_{};
    bool         last_known_pose_valid_{false};
    rclcpp::Time settle_start_;
    rclcpp::Time reloc_start_;
    float        latest_score_{0.0f};
    int          last_localization_status_{-999};
    std::string  last_alarm_;
    // 重定位统一入口（mode 区分全局/局部）
    rclcpp::Client<nav_interfaces::srv::Relocalize>::SharedPtr relocalization_cli_;
    rclcpp::TimerBase::SharedPtr recover_timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alarm_pub_;
};

}  // namespace command_board
