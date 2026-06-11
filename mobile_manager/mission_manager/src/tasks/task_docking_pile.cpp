#include "mission_manager/tasks/task_docking_pile.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <memory>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

#include "glog/logging.h"
#include "mission_manager/nav_util_adapter.hpp"

namespace pnc {

TaskDockingPile::TaskDockingPile() {
    curr_task_stage_ = DockingPileTaskStage::INIT;
    task_type_ = TaskType::DOCKING_PILE;
    docking_info_ = std::make_shared<DockingInfo>();
}

bool TaskDockingPile::Init(const std::string& task_name) {
    name_ = task_name;
    return true;
}

TaskStatus TaskDockingPile::Execute() {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (mission_info_ == nullptr) {
        LOG(ERROR) << "mission info is not set, charging task failed!" << std::endl;
        return TaskStatus::FAILED;
    }

    if (is_paussing_) {
        // 如果任务被终止，则充电任务从头开始执行
        return TaskStatus::PAUSING;
    }

    // 首先，导航到充电点
    if (curr_task_stage_ == DockingPileTaskStage::INIT) {
        charge_station_ = passing_by_.back();
        assert(charge_station_ != nullptr);

        bool need_navigation = false;
        if (!IsNearStation()) {
            // 如果不在充电点附近，则直接任务失败
            // curr_task_stage_ = DockingPileTaskStage::FAILED;
            // return TaskStatus::FAILED;
            // 或者进行导航
            need_navigation = true;
            std::cout << "is not near station, start navigation to station" << std::endl;
        } else {
            if (!IsAtStation()) {
                need_navigation = true;
                std::cout << "is not at station, start navigation to station" << std::endl;
            }
        }

        if (need_navigation) {
            curr_goal_ = charge_station_;
            mission_info_->SetCurrentGoal(curr_goal_);
            mission_info_->SetNeedNavigation();
            curr_task_stage_ = DockingPileTaskStage::NAV_TO_STATION;
        } else {
            // 开始执行上桩操作
            // 发布上桩指令到move base
            docking_info_->SetTaskType(DockingInfo::TaskType::DOCKING_PILE);
            docking_info_->SetModelType(DockingInfo::ModelType::CHARGE_STATION);
            docking_info_->SetModelName("ChargeStation");
            Eigen::Vector3d model_pose;
            model_pose << 0.6, 0.0, 0.0;
            docking_info_->SetModelPose(model_pose);

            mission_info_->SetNeedDocking();
            mission_info_->SetNeedPerception();

            curr_task_stage_ = DockingPileTaskStage::DOCKING;
        }
        return TaskStatus::EXECUTING;
    }

    // 对于任务失败后，重启
    if (curr_task_stage_ == DockingPileTaskStage::NAV_TO_STATION) {
        if (mission_info_->is_navigating()) {
            curr_task_stage_ = DockingPileTaskStage::NAV_TO_STATION;
            return TaskStatus::EXECUTING;
        }

        if (mission_info_->is_navigation_finished()) {
            if (mission_info_->is_navigation_success()) {
                curr_task_stage_ = DockingPileTaskStage::INIT;
            } else {
                LOG(ERROR) << "navigation failed, docking pile task failed!" << std::endl;
                curr_task_stage_ = DockingPileTaskStage::FAILED;
                return TaskStatus::FAILED;
            }
        }
    }

    if (curr_task_stage_ == DockingPileTaskStage::DOCKING) {
        // 等待上桩完成
        if (mission_info_->is_docking_finished()) {
            if (mission_info_->is_docking_success() && mission_info_->GetRobotStatus()->is_charging) {
                // mission_info_->SetNeedCancelPerception();
                curr_task_stage_ = DockingPileTaskStage::FINISHED;
            } else {
                LOG(ERROR) << "docking failed, docking pile task failed!" << std::endl;
                curr_task_stage_ = DockingPileTaskStage::FAILED;
                return TaskStatus::FAILED;
            }
        }
        return TaskStatus::EXECUTING;
    }

    if (curr_task_stage_ == DockingPileTaskStage::FINISHED) {
        return TaskStatus::FINISHED;
    }

    return TaskStatus::FAILED;
}

bool TaskDockingPile::IsAtStation() {
    auto robot_status = mission_info_->GetRobotStatus();
    geometry_msgs::msg::PoseStamped robot_pose;
    robot_pose.pose.position.x = robot_status->pose.x();
    robot_pose.pose.position.y = robot_status->pose.y();

    tf2::Quaternion q_robot;
    q_robot.setRPY(0, 0, robot_status->pose.z());
    robot_pose.pose.orientation = tf2::toMsg(q_robot);

    geometry_msgs::msg::PoseStamped station_pose;
    station_pose.pose.position.x = charge_station_->pose.x;
    station_pose.pose.position.y = charge_station_->pose.y;

    tf2::Quaternion q_station;
    q_station.setRPY(0, 0, charge_station_->pose.theta);
    station_pose.pose.orientation = tf2::toMsg(q_station);

    constexpr double dist_th = 0.1;
    constexpr double yaw_th = 0.17 * 5;

    return nav_util::is_same_pose(robot_pose, station_pose, dist_th, yaw_th);
}

bool TaskDockingPile::IsNearStation() {
    if (charge_station_ == nullptr) {
        return false;
    }
    auto robot_status = mission_info_->GetRobotStatus();

    geometry_msgs::msg::PoseStamped robot_pose;
    robot_pose.pose.position.x = robot_status->pose.x();
    robot_pose.pose.position.y = robot_status->pose.y();

    tf2::Quaternion q_robot;
    q_robot.setRPY(0, 0, robot_status->pose.z());
    robot_pose.pose.orientation = tf2::toMsg(q_robot);

    geometry_msgs::msg::PoseStamped station_pose;
    station_pose.pose.position.x = charge_station_->pose.x;
    station_pose.pose.position.y = charge_station_->pose.y;

    tf2::Quaternion q_station;
    q_station.setRPY(0, 0, charge_station_->pose.theta);
    station_pose.pose.orientation = tf2::toMsg(q_station);

    return nav_util::euclidean_distance(robot_pose, station_pose) < 1.0;
}

TaskStatus TaskDockingPile::Pause() {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (curr_task_stage_ == DockingPileTaskStage::FINISHED) {
        return TaskStatus::PAUSING;
    }
    mission_info_->ResetStatus();
    curr_task_stage_ = DockingPileTaskStage::INIT;
    // 如果到达了最后一个点，在对接的时候进行取消任务，启动的时候再回到最后一个任务点
    if (curr_goal_idx_ >= passing_by_.size()) {
        curr_goal_idx_ -= 1;
    }

    is_paussing_ = true;
    return TaskStatus::PAUSING;
}

TaskStatus TaskDockingPile::Resume() {
    if (!is_paussing_) {
        LOG(INFO) << "TaskNavigation::Resume() called but not in pausing state";
        return TaskStatus::EXECUTING;
    }
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (curr_task_stage_ == DockingPileTaskStage::FINISHED) {
        return TaskStatus::FINISHED;
    }
    mission_info_->ResetStatus();
    curr_task_stage_ = DockingPileTaskStage::INIT;
    is_paussing_ = false;

    // 这里不改变curr_goal_idx_，还是从断点前的目标点开始执行
    return TaskStatus::EXECUTING;
}

TaskStatus TaskDockingPile::Restart() {
    ResetTask();
    return TaskStatus::EXECUTING;
}

void TaskDockingPile::ResetTask() {
    std::lock_guard<std::mutex> lock(status_mutex_);
    is_paussing_ = false;
    curr_task_stage_ = DockingPileTaskStage::INIT;
    mission_info_->ResetStatus();
    curr_goal_.reset();
    curr_goal_idx_ = 0;
    task_progress_ = 0.0;
}

}  // namespace pnc
