
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
#include <memory>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <vector>

#include "localization_sm.h"
#include "nav_interfaces/msg/localization_state.hpp"
#include "nav_interfaces/srv/load_map.hpp"
#include "nav_interfaces/srv/reset_map_origin_by_xy_yaw.hpp"
#include "nav_interfaces/srv/set_pose.hpp"
#include "nav_interfaces/srv/transform_assets.hpp"
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace command_board {

class CommandBoardManagerROS : public rclcpp::Node {
public:
    CommandBoardManagerROS();
    ~CommandBoardManagerROS() = default;

    void OnLocalizationState(const nav_interfaces::msg::LocalizationState::SharedPtr msg);

    void OnDiagnosticsAgg(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);

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

    // 内部：采样 TF map→base_link，返回 N 帧均值（循环等待最多 timeout_sec）
    bool SampleRobotPose(int n_frames, double timeout_sec, double& out_x, double& out_y, double& out_yaw);

    // 内部：调用子进程 reset_map_origin_tool
    bool RunResetMapOriginTool(const std::string& map_name, double x, double y, double yaw, std::string& out_log);

    rclcpp::CallbackGroup::SharedPtr callback_group_;

    rclcpp::Subscription<nav_interfaces::msg::LocalizationState>::SharedPtr localization_state_sub_;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_agg_sub_;

    // localization_node 以 "~/state" 发布，节点名 localization_node → /localization_node/state
    std::string localization_state_topic_{"/localization_node/state"};

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;

    // 定位控制 & 地图管理服务客户端
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_localization_cli_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_localization_cli_;
    rclcpp::Client<nav_interfaces::srv::LoadMap>::SharedPtr load_map_cli_;
    rclcpp::Client<nav_interfaces::srv::TransformAssets>::SharedPtr transform_assets_cli_;

    // 对外暴露的重置原点服务
    rclcpp::Service<nav_interfaces::srv::ResetMapOriginByXYYaw>::SharedPtr reset_origin_xyyaw_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_origin_robot_pose_srv_;

    // TF buffer（用于采样机器人位姿）
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 当前地图名称（从 map_manager/map_name 话题获取）
    std::string current_map_name_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_name_sub_;

    std::unique_ptr<Localization_SM> localization_sm_;

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
    static const char* RecoverStateToString(RecoverState s);

    // 恢复功能参数（可配置）
    bool   enable_loss_recovery_{true};
    double recover_settle_sec_{1.0};
    double reloc_wait_sec_{2.0};
    double score_good_{0.8};
    double score_bad_{0.5};
    double pose_cache_min_score_{0.6};
    double velocity_still_threshold_{0.05};

    // 恢复状态变量
    RecoverState recover_state_{RecoverState::kNormal};
    bool         safety_active_{false};
    geometry_msgs::msg::Pose2D last_known_pose_{};
    bool         last_known_pose_valid_{false};
    rclcpp::Time settle_start_;
    rclcpp::Time reloc_start_;
    float        latest_score_{0.0f};
    std::string  last_alarm_;

    // 恢复功能服务客户端和发布器
    rclcpp::Client<nav_interfaces::srv::SetPose>::SharedPtr relocalization_cli_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr global_reloc_cli_;
    rclcpp::TimerBase::SharedPtr recover_timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alarm_pub_;

    // 用于速度检测的订阅（实测里程计速度，不用 /cmd_vel 指令速度：
    // 急停时指令恒为零，但机器人可能仍在滑行）
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    geometry_msgs::msg::Twist latest_velocity_{};
    bool velocity_valid_{false};
    std::string odom_topic_{"/robot_localization/odometry/filtered"};
};

}  // namespace command_board
