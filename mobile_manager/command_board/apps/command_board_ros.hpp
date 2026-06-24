
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
#include "nav_interfaces/srv/transform_assets.hpp"
#include "map_version_manager.hpp"
#include "status_monitor.hpp"
#include "auto_map_saver.hpp"

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

    // Tunable parameters (declared in ctor; see params/command_board_params.yaml).
    int    reset_origin_sample_frames_{11};
    double reset_origin_sample_timeout_sec_{5.0};
    double service_call_timeout_sec_{15.0};
    double service_wait_timeout_sec_{3.0};
    bool   react_to_estop_{true};
    bool   react_to_bumper_{true};

    rclcpp::CallbackGroup::SharedPtr callback_group_;

    rclcpp::Subscription<nav_interfaces::msg::LocalizationState>::SharedPtr localization_state_sub_;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_agg_sub_;

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

    // 地图统一版本备份 + 回退单入口（Model C / DEC-009，移植自 nav2_command_board）
    std::unique_ptr<MapVersionManager> map_version_manager_;

    // 状态聚合 + 条件自动保存地图（移植自 nav2_command_board）
    std::unique_ptr<StatusMonitor> status_monitor_;
    std::unique_ptr<AutoMapSaver>  auto_map_saver_;
};

}  // namespace command_board
