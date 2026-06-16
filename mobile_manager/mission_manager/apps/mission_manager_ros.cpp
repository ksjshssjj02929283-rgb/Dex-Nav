#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "nav_interfaces/msg/nav_point.hpp"
#include "nav_interfaces/srv/get_all_nav_points.hpp"
// #include "nav_interfaces/msg/docking_base_action_goal.hpp"

#include "mission_manager/mission_manager_param.hpp"
#include "mission_manager/task_group.hpp"
#include "mission_manager/tasks/task_auto_charging.hpp"
#include "mission_manager/tasks/task_docking_pile.hpp"
#include "mission_manager/tasks/task_exit_pile.hpp"
#include "mission_manager/tasks/task_navigation.hpp"
#include "mission_manager_ros.hpp"

namespace pnc {

namespace {

NavPointType NavPointTypeFromMsgType(const std::string& type) {
    if (type == "FREE_GOAL") {
        return NavPointType::FREE_GOAL;
    }
    if (type == "STATION") {
        return NavPointType::STATION;
    }
    if (type == "CHARGING") {
        return NavPointType::CHARGING;
    }
    if (type == "PARKING") {
        return NavPointType::PARKING;
    }
    if (type == "PICKUP") {
        return NavPointType::PICKUP;
    }

    return NavPointType::UNKNOWN;
}

}  // namespace

MissionManagerROS::MissionManagerROS() : Node("mission_manager_node") { Initialize(); }

void MissionManagerROS::Initialize() {
    // 创建 callback groups
    // 使用 Reentrant，允许在同一组内并发执行回调（需要 MultiThreadedExecutor）
    callback_group_services_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    callback_group_clients_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    callback_group_services_navs_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    robot_status_ = std::make_shared<RobotStatus>();

    MissionManagerParam param;
    this->declare_parameter<double>("mission_manager.auto_charge_threshold", 30.0);
    this->declare_parameter<double>("mission_manager.force_charge_threshold", 20.0);
    this->declare_parameter<bool>("mission_manager.respawn_task", false);

    param.auto_charge_threshold = this->get_parameter("mission_manager.auto_charge_threshold").as_double();
    param.force_charge_threshold = this->get_parameter("mission_manager.force_charge_threshold").as_double();
    param.respawn_task = this->get_parameter("mission_manager.respawn_task").as_bool();

    mission_manager_ = std::make_unique<MissionManager>(param);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 1, std::bind(&MissionManagerROS::OdomCallback, this, std::placeholders::_1));
    battery_info_sub_ = this->create_subscription<nav_interfaces::msg::BatteryInfo>(
        "/battery_info", 1, std::bind(&MissionManagerROS::BatteryInfoCallback, this, std::placeholders::_1));
    estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/emergency_stop", 1, std::bind(&MissionManagerROS::EmergencyStopCallback, this, std::placeholders::_1));
    control_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/nav_mode", 1, std::bind(&MissionManagerROS::ControlModeCallback, this, std::placeholders::_1));

    // action_goal_pub_ = this->create_publisher<nav_interfaces::msg::DockingBaseActionGoal>(
    //   "/docking_base/goal", 1);
    manager_status_pub_ = this->create_publisher<std_msgs::msg::String>("~/manager_status", 1);
    current_task_info_pub_ = this->create_publisher<nav_interfaces::msg::Task>("~/current_task_info", 1);

    // 所有服务使用独立的 callback group，避免被 50ms 高频 timer 阻塞
    // 原代码（ROS1 风格，会被阻塞）：
    // add_task_server_ = this->create_service<nav_interfaces::srv::AddTask>(
    //   "~/add_task", std::bind(&MissionManagerROS::AddTaskCallback, this, std::placeholders::_1,
    //   std::placeholders::_2));
    add_task_server_ = this->create_service<nav_interfaces::srv::AddTask>(
        "~/add_task",
        std::bind(&MissionManagerROS::AddTaskCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_services_);

    // add_charge_task_server_ = this->create_service<nav_interfaces::srv::AddTask>(
    //   "~/add_charge_task", std::bind(&MissionManagerROS::AddChargeTaskCallback, this, std::placeholders::_1,
    //   std::placeholders::_2));
    add_charge_task_server_ = this->create_service<nav_interfaces::srv::AddTask>(
        "~/add_charge_task",
        std::bind(&MissionManagerROS::AddChargeTaskCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_services_);

    // add_task_group_server_ = this->create_service<nav_interfaces::srv::AddTaskGroup>(
    //   "~/add_task_group", std::bind(&MissionManagerROS::AddTaskGroupCallback, this, std::placeholders::_1,
    //   std::placeholders::_2));
    add_task_group_server_ = this->create_service<nav_interfaces::srv::AddTaskGroup>(
        "~/add_task_group",
        std::bind(&MissionManagerROS::AddTaskGroupCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_services_);

    // get_current_task_info_server_ = this->create_service<nav_interfaces::srv::GetCurrentTaskInfo>(
    //   "~/get_current_task_info", std::bind(&MissionManagerROS::GetCurrentTaskInfoCallback, this,
    //   std::placeholders::_1, std::placeholders::_2));
    get_current_task_info_server_ = this->create_service<nav_interfaces::srv::GetCurrentTaskInfo>(
        "~/get_current_task_info",
        std::bind(&MissionManagerROS::GetCurrentTaskInfoCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_services_);

    // pause_task_server_ = this->create_service<std_srvs::srv::Trigger>(
    //   "~/pause_task", std::bind(&MissionManagerROS::PauseTaskCallback, this, std::placeholders::_1,
    //   std::placeholders::_2));
    pause_task_server_ = this->create_service<std_srvs::srv::Trigger>(
        "~/pause_task",
        std::bind(&MissionManagerROS::PauseTaskCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_services_);

    // resume_task_server_ = this->create_service<std_srvs::srv::Trigger>(
    //   "~/resume_task", std::bind(&MissionManagerROS::ResumeTaskCallback, this, std::placeholders::_1,
    //   std::placeholders::_2));
    resume_task_server_ = this->create_service<std_srvs::srv::Trigger>(
        "~/resume_task",
        std::bind(&MissionManagerROS::ResumeTaskCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_services_);

    // cancel_task_server_ = this->create_service<std_srvs::srv::Trigger>(
    //   "~/cancel_task", std::bind(&MissionManagerROS::CancelTaskCallback, this, std::placeholders::_1,
    //   std::placeholders::_2));
    cancel_task_server_ = this->create_service<std_srvs::srv::Trigger>(
        "~/cancel_task",
        std::bind(&MissionManagerROS::CancelTaskCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_services_);

    // cancel_task_group_server_ = this->create_service<std_srvs::srv::Trigger>(
    //   "~/cancel_task_group", std::bind(&MissionManagerROS::CancelTaskGroupCallback, this, std::placeholders::_1,
    //   std::placeholders::_2));
    cancel_task_group_server_ = this->create_service<std_srvs::srv::Trigger>(
        "~/cancel_task_group",
        std::bind(&MissionManagerROS::CancelTaskGroupCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_services_);

    // clear_all_tasks_server_ = this->create_service<std_srvs::srv::Trigger>(
    //   "~/clear_all_tasks", std::bind(&MissionManagerROS::ClearAllTaskCallback, this, std::placeholders::_1,
    //   std::placeholders::_2));
    clear_all_tasks_server_ = this->create_service<std_srvs::srv::Trigger>(
        "~/clear_all_tasks",
        std::bind(&MissionManagerROS::ClearAllTaskCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_services_);

    // set_cmd_board_server_ = this->create_service<nav_interfaces::srv::SetCommandBoard>(
    //   "~/set_command_board", std::bind(&MissionManagerROS::SetCmdBoardService, this, std::placeholders::_1,
    //   std::placeholders::_2));
    set_cmd_board_server_ = this->create_service<nav_interfaces::srv::SetCommandBoard>(
        "~/set_command_board",
        std::bind(&MissionManagerROS::SetCmdBoardService, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_services_);

    // set_map_changed 服务使用独立的 callback group，这样可以在回调中安全地等待其他服务
    set_map_changed_server_ = this->create_service<std_srvs::srv::Trigger>(
        "~/set_map_changed",
        std::bind(&MissionManagerROS::SetMapChangedCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_services_);

    // navigation_success_server_ = this->create_service<std_srvs::srv::SetBool>(
    //   "~/set_navigation_success", std::bind(&MissionManagerROS::SetNavigationSuccessCallback, this,
    //   std::placeholders::_1, std::placeholders::_2));
    navigation_success_server_ = this->create_service<std_srvs::srv::SetBool>(
        "~/set_navigation_success",
        std::bind(&MissionManagerROS::SetNavigationSuccessCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_services_navs_);

    // docking_success_server_ = this->create_service<std_srvs::srv::SetBool>(
    //   "~/set_docking_success", std::bind(&MissionManagerROS::SetDockingSuccessCallback, this, std::placeholders::_1,
    //   std::placeholders::_2));
    docking_success_server_ = this->create_service<std_srvs::srv::SetBool>(
        "~/set_docking_success",
        std::bind(&MissionManagerROS::SetDockingSuccessCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_services_);

    // get_mission_status_server_ = this->create_service<nav_interfaces::srv::GetStatus>(
    //   "~/get_mission_status", std::bind(&MissionManagerROS::GetMissionStatusCallback, this, std::placeholders::_1,
    //   std::placeholders::_2));
    get_mission_status_server_ = this->create_service<nav_interfaces::srv::GetStatus>(
        "~/get_mission_status",
        std::bind(&MissionManagerROS::GetMissionStatusCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_services_);

    // 使用 nav_adapter 替代直接调用 move_base
    start_navigation_client_ = this->create_client<nav_interfaces::srv::NavToPoint>(
        "/nav_adapter_node/start_navigation", rmw_qos_profile_services_default, callback_group_clients_);

    // get_all_nav_points 客户端使用独立的 callback group
    get_all_nav_points_client_ = this->create_client<nav_interfaces::srv::GetAllNavPoints>(
        "/nav_point_manager_node/get_all_nav_points", rmw_qos_profile_services_default, callback_group_clients_);

    stop_navigation_client_ = this->create_client<std_srvs::srv::Trigger>("/nav_adapter_node/cancel_navigation");

    run_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&MissionManagerROS::Run, this));
}

void MissionManagerROS::Run() {
    if (robot_status_ == nullptr) {
        return;
    }
    UpdateRobotPose();
    mission_manager_->UpdateRobotStatus(robot_status_);
    mission_manager_->Run();
    PublishMissionStatus();

    RequestStartNavigation();
    // RequestStartPerception();
    // RequestStartDocking();

    RequestCancelNavigation();
    // RequestCancelDocking();
    // RequestCancelPerception();

    MaybeSetMissionSuccess();
}

void MissionManagerROS::PublishMissionStatus() {
    auto status = mission_manager_->GetManagerStatus();
    std_msgs::msg::String status_msg;
    switch (status) {
        case ManagerStatus::EXECUTING:
            status_msg.data = "executing";
            break;
        case ManagerStatus::FAILED:
            status_msg.data = "failed";
            break;
        case ManagerStatus::IDLE:
            status_msg.data = "idle";
            break;
        case ManagerStatus::PAUSING:
            status_msg.data = "pausing";
            break;
        case ManagerStatus::WAITING:
            status_msg.data = "waiting";
            break;
        case ManagerStatus::FINISHED:
            status_msg.data = "finished";
            break;
        default:
            status_msg.data = "unknown";
            break;
    }
    manager_status_pub_->publish(status_msg);

    auto curr_task = mission_manager_->GetCurrTask();
    if (curr_task != nullptr) {
        nav_interfaces::msg::Task curr_task_msg;
        Task2Msg(*curr_task, curr_task_msg);
        current_task_info_pub_->publish(curr_task_msg);
    }
}

bool MissionManagerROS::RequestStartNavigation() {
    // 如果已经在导航中，不重复请求
    if (mission_manager_->is_navigating()) {
        return true;
    }

    if (!mission_manager_->IsNeedNavigation()) {
        return false;
    }

    // 获取当前的导航点，并发布出
    const auto curr_goal = mission_manager_->GetCurrentGoal();
    if (curr_goal == nullptr) {
        RCLCPP_ERROR(this->get_logger(), "[MissionManagerROS] RequestStartNavigation: curr_goal is nullptr");
        return false;
    }
    if (!nav_point_msg_umap_.count(curr_goal->name)) {
        RCLCPP_ERROR(this->get_logger(), "[MissionManagerROS] RequestStartNavigation: curr_goal is not found");
        return false;
    }

    auto nav_point_msg = nav_point_msg_umap_[curr_goal->name];

    nav_point_msg.header.frame_id = "map";
    nav_point_msg.header.stamp = this->now();

    auto start_navigation_srv = std::make_shared<nav_interfaces::srv::NavToPoint::Request>();
    start_navigation_srv->nav_point = nav_point_msg;

    // 检查服务是否存在
    RCLCPP_INFO(this->get_logger(), "Waiting for navigation service...");
    if (!start_navigation_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(),
                     "start_navigation_client_ not exists - service /nav_adapter_node/start_navigation unavailable");
        RCLCPP_ERROR(this->get_logger(), "Make sure nav_adapter_node is running!");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Service available, sending navigation request...");
    auto result_future = start_navigation_client_->async_send_request(start_navigation_srv);

    // 等待响应（最多等待 1 秒，避免阻塞 executor 太久）
    RCLCPP_INFO(this->get_logger(), "Waiting for navigation response...");
    auto status = result_future.wait_for(std::chrono::milliseconds(1000));

    if (status == std::future_status::ready) {
        auto result = result_future.get();
        if (result && result->success) {
            // 立即设置导航状态（与 ROS1 代码一致）
            mission_manager_->SetNavigating();
            RCLCPP_INFO(this->get_logger(), "start_navigation_client_ call success");
            RCLCPP_INFO(this->get_logger(), "[MissionManagerROS] RequestStartNavigation: goal: %s",
                        nav_point_msg.name.c_str());
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Navigation service returned failure");
            return false;
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "start_navigation_client_ call failed (timeout after 1 second)");
        return false;
    }
}

bool MissionManagerROS::RequestCancelNavigation() {
    if (!mission_manager_->IsNeedCancelNavigation()) {
        return false;
    }

    auto stop_navigation_srv = std::make_shared<std_srvs::srv::Trigger::Request>();

    if (!stop_navigation_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(), "stop_navigation_client_ not exists");
        return false;
    }

    auto result_future = stop_navigation_client_->async_send_request(stop_navigation_srv);
    auto status = result_future.wait_for(std::chrono::seconds(1));
    if (status == std::future_status::ready) {
        RCLCPP_INFO(this->get_logger(), "stop_navigation_client_ call success");
        mission_manager_->SetNavigationSuccess(false);
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "stop_navigation_client_ call failed (timeout)");
        return false;
    }

    return false;
}

// bool MissionManagerROS::RequestStartDocking()
// {
//   if (!mission_manager_->IsNeedDocking()) {
//     return false;
//   }

//   auto docking_info_ = mission_manager_->GetDockingInfo();
//   if (docking_info_ == nullptr) {
//     RCLCPP_ERROR(this->get_logger(), "[MissionManagerROS] RequestStartDocking: docking_info_ is nullptr");
//     return false;
//   }

//   // 然后启动对接程序
//   nav_interfaces::msg::DockingBaseActionGoal action_goal;
//   if (docking_info_->GetTaskType() == DockingInfo::TaskType::DOCKING_PILE) {
//     action_goal.goal.target.task_type = nav_interfaces::msg::DockingInfo::DOCKING_PILE;
//     action_goal.goal.target.model_type = nav_interfaces::msg::DockingInfo::CHARGE_STATION;
//   } else if (docking_info_->GetTaskType() == DockingInfo::TaskType::EXIT_PILE) {
//     action_goal.goal.target.task_type = nav_interfaces::msg::DockingInfo::EXIT_PILE;
//     action_goal.goal.target.model_type = nav_interfaces::msg::DockingInfo::POINT;
//   } else {
//     RCLCPP_ERROR(this->get_logger(), "[MissionManagerROS] RequestStartDocking: Unknown docking task type");
//     return false;
//   }
//   action_goal.goal.target.model_name = docking_info_->GetModelName();

//   // 把model pose转换到odom坐标系下
//   geometry_msgs::msg::PoseStamped model_in_base;
//   model_in_base.header.stamp = this->now();
//   model_in_base.header.frame_id = "base_link";
//   model_in_base.pose.position.x = docking_info_->GetModelPose().x();
//   model_in_base.pose.position.y = docking_info_->GetModelPose().y();

//   tf2::Quaternion q;
//   q.setRPY(0, 0, docking_info_->GetModelPose().z());
//   model_in_base.pose.orientation = tf2::toMsg(q);

//   geometry_msgs::msg::PoseStamped model_in_odom;
//   try {
//     if (tf_buffer_->canTransform("odom", "base_link", tf2::TimePointZero)) {
//       model_in_odom = tf_buffer_->transform(model_in_base, "odom", tf2::durationFromSec(10.5));
//     } else {
//       RCLCPP_ERROR(this->get_logger(), "[MissionManagerROS] RequestStartDocking: can not transform model pose from
//       base_link to odom");
//     }
//   } catch (const tf2::TransformException& ex) {
//     RCLCPP_ERROR(this->get_logger(), "[MissionManagerROS] RequestStartDocking: %s", ex.what());
//   }

//   action_goal.goal.target.pose = model_in_odom;

//   // 再向DockingBase Action Goal中发布goal
//   // action_goal_pub_->publish(action_goal);

//   mission_manager_->SetDocking();
//   return true;
// }

bool MissionManagerROS::RequestCancelDocking() {
    if (!mission_manager_->IsNeedCancelDocking()) {
        return false;
    }

    auto stop_docking_client = this->create_client<std_srvs::srv::Trigger>("/docking_base/cancel_docking_task");
    if (!stop_docking_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(),
                     "[MissionManagerROS] RequestCancelDocking: stop_docking_client is not available");
        return false;
    }

    auto stop_docking_srv = std::make_shared<std_srvs::srv::Trigger::Request>();

    auto result_future = stop_docking_client->async_send_request(stop_docking_srv);
    auto status = result_future.wait_for(std::chrono::seconds(1));
    if (status == std::future_status::ready) {
        RCLCPP_INFO(this->get_logger(), "[MissionManagerROS] RequestCancelDocking: stop_docking_client call success");
        mission_manager_->SetDockingSuccess(false);
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "[MissionManagerROS] RequestCancelDocking: stop_docking_client call failed");
        return false;
    }

    return false;
}

// bool MissionManagerROS::RequestStartPerception()
// {
//   // 首先启动检测程序
//   if (!mission_manager_->IsNeedPerception()) {
//     return false;
//   }

//   auto perception_client = this->create_client<std_srvs::srv::Empty>("/perception_2d_node/activate_perception");
//   if (!perception_client->wait_for_service(std::chrono::seconds(1))) {
//     RCLCPP_ERROR(this->get_logger(), "[MissionManagerROS] RequestStartDocking: perception_client is not available");
//     return false;
//   }

//   auto start_perception_srv = std::make_shared<std_srvs::srv::Empty::Request>();

//   auto result_future = perception_client->async_send_request(start_perception_srv);
//   auto status = result_future.wait_for(std::chrono::seconds(1));
//   if (status == std::future_status::ready) {
//     RCLCPP_INFO(this->get_logger(), "[MissionManagerROS] RequestStartDocking: perception_client call success");
//     mission_manager_->SetPerceptionFinished();
//     return true;
//   } else {
//     RCLCPP_ERROR(this->get_logger(), "[MissionManagerROS] RequestStartDocking: perception_client call failed
//     (timeout)"); return false;
//   }
//   return false;
// }

bool MissionManagerROS::RequestCancelPerception() {
    if (!mission_manager_->IsNeedCancelPerception()) {
        return false;
    }

    auto stop_perception_client =
        this->create_client<std_srvs::srv::Empty>("/perception_2d_node/deactivate_perception");

    if (!stop_perception_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(),
                     "[MissionManagerROS] RequestCancelPerception: stop_perception_client is not available");
        return false;
    }

    bool success = false;
    auto stop_perception_srv = std::make_shared<std_srvs::srv::Empty::Request>();

    auto result_future = stop_perception_client->async_send_request(stop_perception_srv);
    auto status = result_future.wait_for(std::chrono::seconds(1));
    if (status == std::future_status::ready) {
        mission_manager_->SetPerceptionFinished();
        RCLCPP_INFO(this->get_logger(),
                    "[MissionManagerROS] RequestCancelPerception: stop_perception_client call success");
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(),
                     "[MissionManagerROS] RequestCancelPerception: stop_perception_client call failed (timeout)");
        return false;
    }

    return false;
}

bool MissionManagerROS::MaybeSetMissionSuccess() {
    auto manager_status = mission_manager_->GetManagerStatus();

    // 只有在状态为FINISHED或者FAILED的情况下才进行设置
    if (!((manager_status == ManagerStatus::FINISHED || manager_status == ManagerStatus::FAILED) ||
          is_cancel_called_ == true)) {
        return true;
    }
    // TODO: 预留接口
    // return true;
    is_cancel_called_ = false;

    set_mission_success_client_ =
        this->create_client<std_srvs::srv::SetBool>("/middleware_set_mission/set_mission_success");

    if (!set_mission_success_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(), "set_mission_success_client_ is not available");
        return false;
    }

    auto set_mission_success_srv = std::make_shared<std_srvs::srv::SetBool::Request>();
    set_mission_success_srv->data = (manager_status == ManagerStatus::FINISHED);

    auto result_future = set_mission_success_client_->async_send_request(set_mission_success_srv);
    auto status = result_future.wait_for(std::chrono::seconds(1));
    if (status == std::future_status::ready) {
        RCLCPP_INFO(this->get_logger(), "set_mission_success_client_ call success");
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "set_mission_success_client_ call failed (timeout)");
        return false;
    }
}

// 向manager中加入task group
void MissionManagerROS::AddTaskGroupCallback(const std::shared_ptr<nav_interfaces::srv::AddTaskGroup::Request> req,
                                             std::shared_ptr<nav_interfaces::srv::AddTaskGroup::Response> res) {
    if (!is_map_set_) {
        RCLCPP_ERROR(this->get_logger(), "Map not set!");
        res->success = false;
        return;
    }

    // 每次添加任务之前，都要更新一下nav_points
    if (!GetAllNavPointsFromClient()) {
        RCLCPP_ERROR(this->get_logger(), "Get all nav points failed!");
        res->success = false;
        return;
    }
    std::shared_ptr<TaskGroup> task_group = std::make_shared<TaskGroup>(req->task_group.name);
    for (auto& task_msg : req->task_group.tasks) {
        std::shared_ptr<TaskInterface> task;
        task = CreateTaskFromMsg(task_msg);

        if (task == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Create task failed!");
            res->success = false;
            return;
        }
        task_group->AddTask(task);
    }

    task_group->SetMaxExecuteCount(req->task_group.max_execute_count);
    task_group->SetInverval(req->task_group.interval);
    if (!mission_manager_->AddTaskGroup(task_group)) {
        RCLCPP_ERROR(this->get_logger(), "Add task group failed!");
        res->success = false;
        return;
    }
    RCLCPP_INFO(this->get_logger(), "add task group %s", task_group->name().c_str());

    res->success = true;
}

// 向manager中加入task
void MissionManagerROS::AddTaskCallback(const std::shared_ptr<nav_interfaces::srv::AddTask::Request> req,
                                        std::shared_ptr<nav_interfaces::srv::AddTask::Response> res) {
    if (!is_map_set_) {
        RCLCPP_ERROR(this->get_logger(), "Map not set!");
        res->success = false;
        return;
    }

    // 每次添加任务之前，都要更新一下nav_points
    if (!GetAllNavPointsFromClient()) {
        RCLCPP_ERROR(this->get_logger(), "Get all nav points failed!");
        res->success = false;
        return;
    }

    std::shared_ptr<TaskInterface> task;
    task = CreateTaskFromMsg(req->task);
    if (task == nullptr) {
        RCLCPP_ERROR(this->get_logger(), "Create task failed!");
        res->success = false;
        return;
    }

    std::shared_ptr<TaskGroup> task_group = std::make_shared<TaskGroup>(task->name());
    task_group->AddTask(task);

    if (!mission_manager_->AddTaskGroup(task_group)) {
        RCLCPP_ERROR(this->get_logger(), "Add task failed!");
        res->success = false;
        return;
    }

    RCLCPP_INFO(this->get_logger(), "add task %s", task_group->name().c_str());

    res->success = true;
}

void MissionManagerROS::AddChargeTaskCallback(const std::shared_ptr<nav_interfaces::srv::AddTask::Request> req,
                                              std::shared_ptr<nav_interfaces::srv::AddTask::Response> res) {
    if (req->task.type != nav_interfaces::msg::Task::DOCKING_PILE) {
        RCLCPP_ERROR(this->get_logger(), "task type is not CHARGING");
        res->success = false;
        return;
    }

    mission_manager_->CancelCurrentTask();
    mission_manager_->ClearAllTasks();

    AddTaskCallback(req, res);
}

void MissionManagerROS::GetCurrentTaskInfoCallback(
    const std::shared_ptr<nav_interfaces::srv::GetCurrentTaskInfo::Request> req,
    std::shared_ptr<nav_interfaces::srv::GetCurrentTaskInfo::Response> res) {
    auto curr_task = mission_manager_->GetCurrTask();
    if (curr_task == nullptr) {
        RCLCPP_WARN(this->get_logger(), "No task in mission_manager.");
        res->success = true;
        return;
    }
    nav_interfaces::msg::Task curr_task_msg;
    Task2Msg(*curr_task, curr_task_msg);

    auto task_list = mission_manager_->GetTaskList();
    std::vector<nav_interfaces::msg::Task> task_list_msg;
    for (const auto& task : task_list) {
        nav_interfaces::msg::Task task_msg;
        Task2Msg(*task, task_msg);
        task_list_msg.push_back(task_msg);
    }

    auto curr_task_group = mission_manager_->GetCurrTaskGroup();
    nav_interfaces::msg::TaskGroup curr_group_msg;
    TaskGroup2Msg(*curr_task_group, curr_group_msg);

    res->current_task = std::move(curr_task_msg);
    res->current_task_list = std::move(task_list_msg);
    res->current_task_group = std::move(curr_group_msg);
    res->execution_left = mission_manager_->GetCurrGroupExecutionLeft();

    res->success = true;
}

void MissionManagerROS::Task2Msg(const TaskInterface& task, nav_interfaces::msg::Task& msg) const {
    msg.name = task.name();
    if (task.task_type() == TaskType::NAVIGATION) {
        msg.type = nav_interfaces::msg::Task::NAVIGATION;
    } else if (task.task_type() == TaskType::DOCKING_PILE) {
        msg.type = nav_interfaces::msg::Task::DOCKING_PILE;
    } else if (task.task_type() == TaskType::EXIT_PILE) {
        msg.type = nav_interfaces::msg::Task::EXIT_PILE;
    } else if (task.task_type() == TaskType::AUTO_CHARGING) {
        msg.type = nav_interfaces::msg::Task::AUTO_CHARGING;
    } else {
        msg.type = nav_interfaces::msg::Task::UNKNOWN;
    }

    for (const auto& nav_point : task.GetPassingBy()) {
        msg.station_names.push_back(nav_point->name);
    }

    if (task.priority() == TaskPriority::NORMAL) {
        msg.priority = nav_interfaces::msg::Task::NORMAL;
    } else if (task.priority() == TaskPriority::HIGH) {
        msg.priority = nav_interfaces::msg::Task::HIGH;
    } else if (task.priority() == TaskPriority::EMERGENCY) {
        msg.priority = nav_interfaces::msg::Task::EMERGENCY;
    } else {
        msg.priority = nav_interfaces::msg::Task::NORMAL;
    }

    msg.waiting_time = task.waiting_time();
}

void MissionManagerROS::TaskGroup2Msg(const TaskGroup& task_group, nav_interfaces::msg::TaskGroup& msg) const {
    msg.name = task_group.name();
    msg.max_execute_count = task_group.max_execute_count();
    msg.interval = task_group.interval();
    for (auto task : task_group.GetTaskList()) {
        nav_interfaces::msg::Task task_msg;
        Task2Msg(*task, task_msg);
        msg.tasks.push_back(task_msg);
    }
}

void MissionManagerROS::PauseTaskCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                          std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    res->success = mission_manager_->PauseCurrentTask();
}

void MissionManagerROS::ResumeTaskCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                           std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    res->success = mission_manager_->ResumeCurrentTask();
}

void MissionManagerROS::CancelTaskCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                           std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    res->success = mission_manager_->CancelCurrentTask();
    is_cancel_called_ = true;
    MaybeSetMissionSuccess();
}

void MissionManagerROS::CancelTaskGroupCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                                std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    res->success = mission_manager_->CancelCurrentTaskGroup();
    is_cancel_called_ = true;
    MaybeSetMissionSuccess();
}

void MissionManagerROS::ClearAllTaskCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    res->success = mission_manager_->ClearAllTasks();
}

void MissionManagerROS::GetMissionStatusCallback(const std::shared_ptr<nav_interfaces::srv::GetStatus::Request> req,
                                                 std::shared_ptr<nav_interfaces::srv::GetStatus::Response> res) {
    auto status = mission_manager_->GetManagerStatus();
    auto curr_task = mission_manager_->GetCurrTask();
    res->name = curr_task ? curr_task->name() : "";
    switch (status) {
        case ManagerStatus::EXECUTING:
            res->status = "executing";
            break;
        case ManagerStatus::FAILED:
            res->status = "failed";
            break;
        case ManagerStatus::IDLE:
            res->status = "idle";
            break;
        case ManagerStatus::PAUSING:
            res->status = "pausing";
            break;
        case ManagerStatus::WAITING:
            res->status = "waiting";
            break;
        case ManagerStatus::FINISHED:
            res->status = "finished";
            break;
        default:
            res->status = "unknown";
            break;
    }
    res->success = true;
}

void MissionManagerROS::SetNavigationSuccessCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                                     std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
    RCLCPP_INFO(
        this->get_logger(), "===SetNavigationSuccessCallback enter %ld ms",
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
            .count());

    RCLCPP_INFO(this->get_logger(), "Received navigation result: %s", req->data ? "success" : "failed");

    mission_manager_->SetNavigationSuccess(req->data);

    res->success = true;
    RCLCPP_INFO(this->get_logger(), "===SetNavigationSuccessCallback exit");
}

void MissionManagerROS::SetDockingSuccessCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                                  std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
    mission_manager_->SetDockingSuccess(req->data);

    res->success = true;
}

void MissionManagerROS::SetMapChangedCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                              std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    is_map_set_ = true;

    // 使用 MultiThreadedExecutor + CallbackGroup 后，可以在服务回调中同步等待其他服务
    // 因为服务回调和客户端响应在不同的 callback group 中，可以并发执行
    if (!GetAllNavPointsFromClient()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get all nav points from client.");
        res->success = false;
        res->message = "Failed to load navigation points";
        return;
    }

    RCLCPP_INFO(this->get_logger(), "[MissionManager] Map changed and nav points loaded successfully.");

    res->success = true;
    res->message = "Map activated and nav points loaded";
}

void MissionManagerROS::SetCmdBoardService(const std::shared_ptr<nav_interfaces::srv::SetCommandBoard::Request> req,
                                           std::shared_ptr<nav_interfaces::srv::SetCommandBoard::Response> res) {
    if (req->cmd_type != "control_mode") {
        RCLCPP_WARN(this->get_logger(), "cmd_type is not control_mode");
        res->success = true;
        return;
    }

    RCLCPP_WARN(this->get_logger(), "[MissionManager] Set %s to %s", req->cmd_type.c_str(), req->cmd_value.c_str());

    if (req->cmd_value == "auto") {
        robot_status_->auto_mode = true;
    } else if (req->cmd_value == "manual") {
        robot_status_->auto_mode = false;
    }
    res->success = true;
}

void MissionManagerROS::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (robot_status_ == nullptr) {
        return;
    }
    // robot_status_->pose = Eigen::Vector3d(msg->pose.pose.position.x,
    //                                       msg->pose.pose.position.y,
    //                                       tf2::getYaw(msg->pose.pose.orientation));
    robot_status_->velocity =
        Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.angular.z);
}

void MissionManagerROS::BatteryInfoCallback(const nav_interfaces::msg::BatteryInfo::SharedPtr msg) {
    if (robot_status_ == nullptr) {
        return;
    }
    robot_status_->soc = static_cast<double>(msg->soc);
    robot_status_->is_charging = msg->charge_state;
}

void MissionManagerROS::EmergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (robot_status_ == nullptr) {
        return;
    }
    robot_status_->emergency_stop = msg->data;
}

void MissionManagerROS::ControlModeCallback(const std_msgs::msg::String::SharedPtr msg) {
    if (robot_status_ == nullptr) {
        return;
    }
    if (msg->data == "AUTO") {
        robot_status_->auto_mode = true;
    } else if (msg->data == "MANUAL") {
        robot_status_->auto_mode = false;
    }
    // 如果从自动模式切换到手动模式，则暂停导航任务
    // 但是从手动模式切换到自动模式，需要手动调用resume接口
}

void MissionManagerROS::UpdateRobotPose() {
    std::string error_msg;
    if (!tf_buffer_->canTransform("map", "base_link", tf2::TimePointZero, tf2::durationFromSec(0.1), &error_msg)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "can not find transform from map to base_link. Please localize the robot first.");
        return;
    }

    try {
        geometry_msgs::msg::TransformStamped map_base;
        map_base = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero, tf2::durationFromSec(0.1));

        tf2::Quaternion q(map_base.transform.rotation.x, map_base.transform.rotation.y, map_base.transform.rotation.z,
                          map_base.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        robot_status_->pose = Eigen::Vector3d(map_base.transform.translation.x, map_base.transform.translation.y, yaw);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Could not transform map to base_link: %s",
                             ex.what());
    }
}

std::shared_ptr<TaskInterface> MissionManagerROS::CreateTaskFromMsg(nav_interfaces::msg::Task& task_msg) {
    std::shared_ptr<TaskInterface> task;
    std::string task_name = task_msg.name;
    if (task_msg.type == nav_interfaces::msg::Task::AUTO_CHARGING) {
        task = std::make_shared<TaskAutoCharging>();
        if (task_name.empty()) {
            task_name = "auto_charging_" + task_msg.station_names.back();
        }
        task->Init(task_name);
    } else if (task_msg.type == nav_interfaces::msg::Task::NAVIGATION) {
        task = std::make_shared<TaskNavigation>();
        if (task_name.empty()) {
            task_name = "navigation_" + task_msg.station_names.back();
        }
        task->Init(task_name);
    } else if (task_msg.type == nav_interfaces::msg::Task::DOCKING_PILE) {
        task = std::make_shared<TaskDockingPile>();
        if (task_name.empty()) {
            task_name = "docking_pile_" + task_msg.station_names.back();
        }
        task->Init(task_name);
    } else if (task_msg.type == nav_interfaces::msg::Task::EXIT_PILE) {
        task = std::make_shared<TaskExitPile>();
        if (task_name.empty()) {
            task_name = "exit_pile_" + task_msg.station_names.back();
        }
        task->Init(task_name);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Task type is not supported");
        return nullptr;
    }

    if (nav_point_msg_umap_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "No nav_point is received");
        return nullptr;
    }

    std::vector<std::shared_ptr<NavPoint>> passing_by_point;
    for (const auto& station_name : task_msg.station_names) {
        if (nav_point_msg_umap_.count(station_name)) {
            auto point_msg = nav_point_msg_umap_[station_name];
            NavPoint point;
            point.pose.x = point_msg.x;
            point.pose.y = point_msg.y;
            point.pose.theta = point_msg.theta;
            point.name = point_msg.name;
            point.id = point_msg.id;
            point.point_type = NavPointTypeFromMsgType(point_msg.type);
            passing_by_point.push_back(std::make_shared<NavPoint>(point));
        } else {
            RCLCPP_ERROR(this->get_logger(), "station %s not found", station_name.c_str());
            return nullptr;
        }
    }

    if (passing_by_point.empty()) {
        RCLCPP_ERROR(this->get_logger(), "passing by points is empty");
        return nullptr;
    }

    task->SetPassingBy(passing_by_point);
    task->SetWaitingTime(task_msg.waiting_time);
    return task;
}

bool MissionManagerROS::GetAllNavPointsFromClient() {
    nav_point_msg_umap_.clear();

    auto srv = std::make_shared<nav_interfaces::srv::GetAllNavPoints::Request>();

    // 等待服务 2s（应该很快，因为 nav_point_manager 已经通过话题加载了导航点）
    if (!get_all_nav_points_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "GetAllNavPoints service not available");
        return false;
    }

    // 发送异步请求
    auto result_future = get_all_nav_points_client_->async_send_request(srv);

    //-- add pzc 原来的spin_until_future_complete会导致executor冲突，所以使用wait_for等待响应
    //-- 不确定会不会有其他问题，需要进一步测试

    // 使用wait_for等待响应（避免spin_until_future_complete导致的executor冲突）
    // 减少超时时间到3秒，因为 nav_point_manager 应该已经加载完成
    auto status = result_future.wait_for(std::chrono::seconds(3));
    if (status != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "GetAllNavPoints service call timed out");
        return false;
    }

    auto result = result_future.get();
    if (!result) {
        RCLCPP_ERROR(this->get_logger(), "GetAllNavPoints service call failed");
        return false;
    }

    RCLCPP_WARN(this->get_logger(), "Get %ld nav points for manager", result->points.size());

    std::string point_list_str;
    for (const auto& nav_point_msg : result->points) {
        point_list_str += nav_point_msg.name + " ";
    }

    RCLCPP_INFO(this->get_logger(), "Get nav point list: %s \n", point_list_str.c_str());

    std::vector<std::shared_ptr<NavPoint>> nav_points;
    for (const auto& nav_point_msg : result->points) {
        nav_point_msg_umap_[nav_point_msg.name] = nav_point_msg;

        NavPoint nav_point;
        nav_point.name = nav_point_msg.name;
        nav_point.id = nav_point_msg.id;
        nav_point.pose.x = nav_point_msg.x;
        nav_point.pose.y = nav_point_msg.y;
        nav_point.pose.theta = nav_point_msg.theta;
        nav_point.point_type = NavPointTypeFromMsgType(nav_point_msg.type);
        nav_points.push_back(std::make_shared<NavPoint>(nav_point));
    }

    assert(mission_manager_);
    mission_manager_->SetNavPoints(nav_points);

    return true;
}

}  // namespace pnc
