#include "mission_manager/tasks/task_navigation.hpp"

#include <memory>
#include <thread>
#include <vector>

#include "glog/logging.h"

namespace pnc {

TaskNavigation::TaskNavigation() {
    curr_task_stage_ = NavigationTaskStage::INIT;
    task_type_ = TaskType::NAVIGATION;
    docking_info_ = std::make_shared<DockingInfo>();
}

bool TaskNavigation::Init(const std::string& task_name) {
    name_ = task_name;
    curr_goal_idx_ = 0;
    return true;
}

TaskStatus TaskNavigation::Execute() {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (mission_info_ == nullptr) {
        LOG(ERROR) << "mission info is not set, navigation task failed!" << std::endl;
        return TaskStatus::FAILED;
    }

    if (mission_info_->is_navigation_finished()) {
        if (!mission_info_->is_navigation_success()) {
            LOG(ERROR) << "navigation failed, navigation task failed!" << std::endl;
            curr_task_stage_ = NavigationTaskStage::FAILED;
            return TaskStatus::FAILED;
        }
    }

    if (is_paussing_) {
        // 如果任务被终止，则充电任务从头开始执行
        return Pause();
    }

    if (passing_by_.empty()) {
        LOG(ERROR) << "passing by is empty, navigation task failed!" << std::endl;
        return TaskStatus::FAILED;
    }

    if (curr_task_stage_ == NavigationTaskStage::INIT) {
        // 首先，判断是否是充电状态，导航到最近的充电点
        if (mission_info_->GetRobotStatus()->is_charging) {
            docking_info_->SetTaskType(DockingInfo::TaskType::EXIT_PILE);
            docking_info_->SetModelType(DockingInfo::ModelType::POINT);
            docking_info_->SetModelName("exit_point");
            Eigen::Vector3d point_pose;
            point_pose << 0.4, 0.0, 0.0;
            docking_info_->SetModelPose(point_pose);
            mission_info_->SetNeedDocking();

            curr_task_stage_ = NavigationTaskStage::EXIT_CHARGE_STATION;
        } else {
            if (curr_goal_idx_ < passing_by_.size()) {
                curr_goal_ = passing_by_[curr_goal_idx_];

                mission_info_->SetCurrentGoal(curr_goal_);
                mission_info_->SetNeedNavigation();
                // mission_info_->SetNeedCancelPerception();
            }
            // 下发完点位后，直接转到GOAL_SENDING中，防止重复下发
            curr_task_stage_ = NavigationTaskStage::GOAL_SENDING;
        }

        return TaskStatus::EXECUTING;
    }

    if (curr_task_stage_ == NavigationTaskStage::EXIT_CHARGE_STATION) {
        if (mission_info_->is_docking_finished()) {
            if (mission_info_->is_docking_success()) {
                curr_task_stage_ = NavigationTaskStage::INIT;
                return TaskStatus::FINISHED;
            } else {
                LOG(ERROR) << "docking failed, navigation task failed!" << std::endl;
                curr_task_stage_ = NavigationTaskStage::FAILED;
                return TaskStatus::FAILED;
            }
        }
        return TaskStatus::EXECUTING;
    }

    if (curr_task_stage_ == NavigationTaskStage::GOAL_SENDING) {
        if (mission_info_->is_navigating()) {
            std::cout << "navigation started, goal sending" << std::endl;
            curr_task_stage_ = NavigationTaskStage::NAVIGATING;
        }
        return TaskStatus::EXECUTING;
    }

    if (curr_task_stage_ == NavigationTaskStage::NAVIGATING) {
        // todo: 增加条件，判断机器人是否到达点位，goal reached 且 距离小于0.1m
        if (mission_info_->is_navigation_finished()) {
            if (mission_info_->is_navigation_success()) {
                curr_goal_idx_ += 1;  // 指向下一个点
                if (curr_goal_idx_ < passing_by_.size()) {
                    // 该点不是最后一个点，则继续导航
                    curr_task_stage_ = NavigationTaskStage::INIT;
                } else {  // 大于等于的情况都结束导航
                    curr_task_stage_ = NavigationTaskStage::GOAL_REACHED;
                }
                return TaskStatus::EXECUTING;
            } else {
                LOG(ERROR) << "navigation failed, navigation task failed!" << std::endl;
                curr_task_stage_ = NavigationTaskStage::FAILED;
                return TaskStatus::FAILED;
            }
        }
        return TaskStatus::EXECUTING;
    }

    if (curr_task_stage_ == NavigationTaskStage::GOAL_REACHED) {
        // 等待一段时间，任务完成
        curr_task_stage_ = NavigationTaskStage::FINISHED;
        return TaskStatus::EXECUTING;
    }

    if (curr_task_stage_ == NavigationTaskStage::FINISHED) {
        return TaskStatus::FINISHED;
    }
    return TaskStatus::FAILED;
}

TaskStatus TaskNavigation::Pause() {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (curr_task_stage_ == NavigationTaskStage::FINISHED) {
        return TaskStatus::PAUSING;
    }
    is_paussing_ = true;
    mission_info_->ResetStatus();
    curr_task_stage_ = NavigationTaskStage::INIT;
    return TaskStatus::PAUSING;
}

TaskStatus TaskNavigation::Resume() {
    if (!is_paussing_) {
        LOG(INFO) << "TaskNavigation::Resume() called but not in pausing state";
        return TaskStatus::EXECUTING;
    }
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (curr_task_stage_ == NavigationTaskStage::FINISHED) {
        return TaskStatus::FINISHED;
    }
    mission_info_->ResetStatus();
    curr_task_stage_ = NavigationTaskStage::INIT;
    is_paussing_ = false;
    // 这里不改变curr_goal_idx_，还是从断点前的目标点开始执行
    return TaskStatus::EXECUTING;
}

TaskStatus TaskNavigation::Restart() {
    ResetTask();
    return TaskStatus::EXECUTING;
}

void TaskNavigation::ResetTask() {
    std::lock_guard<std::mutex> lock(status_mutex_);
    is_paussing_ = false;
    curr_task_stage_ = NavigationTaskStage::INIT;
    mission_info_->ResetStatus();
    curr_goal_.reset();
    curr_goal_idx_ = 0;
    task_progress_ = 0.0;
}

}  // namespace pnc
