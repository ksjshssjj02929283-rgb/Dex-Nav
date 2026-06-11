#ifndef MISSION_MANAGER_ROS_HPP_
#define MISSION_MANAGER_ROS_HPP_

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#include "mission_manager/mission_manager.hpp"
#include "mission_manager/robot_status.hpp"
#include "nav_interfaces/msg/battery_info.hpp"
#include "nav_interfaces/msg/nav_point.hpp"
#include "nav_interfaces/msg/task.hpp"
#include "nav_interfaces/srv/add_task.hpp"
#include "nav_interfaces/srv/add_task_group.hpp"
#include "nav_interfaces/srv/get_all_nav_points.hpp"
#include "nav_interfaces/srv/get_current_task_info.hpp"
#include "nav_interfaces/srv/get_status.hpp"
#include "nav_interfaces/srv/nav_to_point.hpp"
#include "nav_interfaces/srv/set_command_board.hpp"

namespace pnc {

class MissionManagerROS : public rclcpp::Node {
public:
    MissionManagerROS();
    ~MissionManagerROS() = default;

private:
    void Initialize();

    std::shared_ptr<TaskInterface> CreateTaskFromMsg(nav_interfaces::msg::Task& task);

    void UpdateRobotPose();

    // 里程计
    void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // 电池信息
    void BatteryInfoCallback(const nav_interfaces::msg::BatteryInfo::SharedPtr msg);

    // 紧急停止
    void EmergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg);

    // 导航模式（自动、手动）
    void ControlModeCallback(const std_msgs::msg::String::SharedPtr msg);

    // 导航结果回调（来自 nav_adapter）
    void NavigationResultCallback(const std_msgs::msg::Bool::SharedPtr msg);

    // 到达目标点
    void GoalReachedCallback(const std_msgs::msg::Bool::SharedPtr msg);

    // 对接是否成功
    void DockingSuccessCallback(const std_msgs::msg::Bool::SharedPtr msg);

    // 向manager中加入task
    void AddTaskCallback(const std::shared_ptr<nav_interfaces::srv::AddTask::Request> req,
                         std::shared_ptr<nav_interfaces::srv::AddTask::Response> res);

    void AddChargeTaskCallback(const std::shared_ptr<nav_interfaces::srv::AddTask::Request> req,
                               std::shared_ptr<nav_interfaces::srv::AddTask::Response> res);

    void AddTaskGroupCallback(const std::shared_ptr<nav_interfaces::srv::AddTaskGroup::Request> req,
                              std::shared_ptr<nav_interfaces::srv::AddTaskGroup::Response> res);

    void GetCurrentTaskInfoCallback(const std::shared_ptr<nav_interfaces::srv::GetCurrentTaskInfo::Request> req,
                                    std::shared_ptr<nav_interfaces::srv::GetCurrentTaskInfo::Response> res);

    void PauseTaskCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    void ResumeTaskCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    void CancelTaskCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    void CancelTaskGroupCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    void ClearAllTaskCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    void GetMissionStatusCallback(const std::shared_ptr<nav_interfaces::srv::GetStatus::Request> req,
                                  std::shared_ptr<nav_interfaces::srv::GetStatus::Response> res);

    void SetNavigationSuccessCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                      std::shared_ptr<std_srvs::srv::SetBool::Response> res);

    void SetDockingSuccessCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                   std::shared_ptr<std_srvs::srv::SetBool::Response> res);

    void SetCmdBoardService(const std::shared_ptr<nav_interfaces::srv::SetCommandBoard::Request> req,
                            std::shared_ptr<nav_interfaces::srv::SetCommandBoard::Response> res);

    void Task2Msg(const TaskInterface& task, nav_interfaces::msg::Task& msg) const;
    void TaskGroup2Msg(const TaskGroup& task_group, nav_interfaces::msg::TaskGroup& msg) const;

    void Run();

    void PublishMissionStatus();

    bool GetAllNavPointsFromClient();

    bool RequestStartNavigation();
    bool RequestStartDocking();
    bool RequestStartPerception();

    bool RequestCancelNavigation();
    bool RequestCancelDocking();
    bool RequestCancelPerception();

    bool MaybeSetMissionSuccess();

    void SetMapChangedCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> res);

private:
    // Callback groups for multi-threaded execution
    rclcpp::CallbackGroup::SharedPtr callback_group_services_;
    rclcpp::CallbackGroup::SharedPtr callback_group_clients_;
    rclcpp::CallbackGroup::SharedPtr callback_group_services_navs_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Client<nav_interfaces::srv::GetAllNavPoints>::SharedPtr get_all_nav_points_client_;  // 请求所有导航点
    rclcpp::Client<nav_interfaces::srv::NavToPoint>::SharedPtr
        start_navigation_client_;                                               // 请求move base 导航到指定点位
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_navigation_client_;  //  请求move base 停止导航
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_docking_client_;    // 请求开始对接

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr set_mission_success_client_;  // 向其它节点中设置任务是否成功

    // rclcpp::Publisher<nav_interfaces::msg::DockingBaseActionGoal>::SharedPtr action_goal_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr manager_status_pub_;
    rclcpp::Publisher<nav_interfaces::msg::Task>::SharedPtr current_task_info_pub_;

    rclcpp::Service<nav_interfaces::srv::AddTask>::SharedPtr add_task_server_;             // 添加任务
    rclcpp::Service<nav_interfaces::srv::AddTask>::SharedPtr add_charge_task_server_;      // 添加任务
    rclcpp::Service<nav_interfaces::srv::AddTaskGroup>::SharedPtr add_task_group_server_;  // 添加任务组
    rclcpp::Service<nav_interfaces::srv::GetCurrentTaskInfo>::SharedPtr get_current_task_info_server_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_task_server_;          // 暂停任务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_task_server_;         // 恢复任务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_task_server_;         // 取消任务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_task_group_server_;   // 取消任务组
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_all_tasks_server_;     // 取消所有任务
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr navigation_success_server_;  // 取消所有任务
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr docking_success_server_;     // 取消所有任务

    rclcpp::Service<nav_interfaces::srv::GetStatus>::SharedPtr get_mission_status_server_;  // 获取任务状态

    rclcpp::Service<nav_interfaces::srv::SetCommandBoard>::SharedPtr set_cmd_board_server_;  // 设置cmdboard状态
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_map_changed_server_;              // 地图是否更新

    rclcpp::TimerBase::SharedPtr run_timer_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_interfaces::msg::BatteryInfo>::SharedPtr battery_info_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_mode_sub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr navigation_result_sub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_reached_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr docking_success_sub_;

    std::unordered_map<std::string, nav_interfaces::msg::NavPoint> nav_point_msg_umap_;

    std::unique_ptr<MissionManager> mission_manager_;
    std::shared_ptr<RobotStatus> robot_status_;

    std::string default_speed_gear_;
    std::string default_control_mode_;
    std::string default_navigation_mode_;

    bool is_cancel_called_ = false;
    bool is_requesting_navigation_ = false;  // 防止重复请求导航

    bool is_map_set_ = false;
    // rclcpp::TimerBase::SharedPtr visualize_timer_;
};

}  // namespace pnc

#endif  // MISSION_MANAGER_ROS_HPP_
