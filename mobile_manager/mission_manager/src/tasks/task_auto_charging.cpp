#include "mission_manager/tasks/task_auto_charging.hpp"

#include <memory>
#include <vector>

#include "glog/logging.h"
#include "mission_manager/nav_util_adapter.hpp"

namespace pnc {

TaskAutoCharging::TaskAutoCharging() {
    curr_task_stage_ = AutoChargingTaskStage::INIT;
    task_type_ = TaskType::AUTO_CHARGING;
    docking_info_ = std::make_shared<DockingInfo>();
}

bool TaskAutoCharging::Init(const std::string& task_name) {
    name_ = task_name;
    curr_goal_idx_ = 0;
    return true;
}

TaskStatus TaskAutoCharging::Execute() {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (mission_info_ == nullptr) {
        LOG(ERROR) << "mission info is not set, charging task failed!" << std::endl;
        return TaskStatus::FAILED;
    }

    if (is_paussing_) {
        // 如果任务被终止，则充电任务从头开始执行
        return TaskStatus::PAUSING;
    }

    if (passing_by_.empty()) {
        LOG(ERROR) << "passing by is empty, charging task failed!" << std::endl;
        return TaskStatus::FAILED;
    }

    if (passing_by_.back()->point_type != NavPointType::CHARGING) {
        LOG(ERROR) << "goal is not charging point, charging task failed!" << std::endl;
        return TaskStatus::FAILED;
    }

    if (mission_info_->GetRobotStatus()->is_charging) {
        curr_task_stage_ = AutoChargingTaskStage::FINISHED;
        return TaskStatus::FINISHED;
    }

    // 首先，导航到充电点
    if (curr_task_stage_ == AutoChargingTaskStage::INIT) {
        // 先导航到充电点
        // todo: 发布充电点
        // 按照顺序发布每一充电点
        // std::cout << "curr_goal_idx_: " << curr_goal_idx_ << std::endl;
        // std::cout << "passing_by_.size(): " << passing_by_.size() << std::endl;
        if (curr_goal_idx_ < passing_by_.size()) {
            curr_goal_ = passing_by_[curr_goal_idx_];

            mission_info_->SetCurrentGoal(curr_goal_);
            mission_info_->SetNeedNavigation();
        }
        // 下发完点位后，直接转到GOAL_SENDING中，防止重复下发
        curr_task_stage_ = AutoChargingTaskStage::GOAL_SENDING;
        return TaskStatus::EXECUTING;
    }

    if (curr_task_stage_ == AutoChargingTaskStage::GOAL_SENDING) {
        if (mission_info_->is_navigating()) {
            curr_task_stage_ = AutoChargingTaskStage::NAVIGATING;
        }

        return TaskStatus::EXECUTING;
    }

    if (curr_task_stage_ == AutoChargingTaskStage::NAVIGATING) {
        // todo: 增加条件，判断机器人是否到达点位，goal reached 且 距离小于0.1m
        if (mission_info_->is_navigation_finished()) {
            if (mission_info_->is_navigation_success()) {
                curr_goal_idx_ += 1;  // 指向下一个点
                if (curr_goal_idx_ < passing_by_.size()) {
                    // 该点不是最后一个点，则继续导航
                    curr_task_stage_ = AutoChargingTaskStage::INIT;
                } else {  // 大于等于的情况都结束导航
                    curr_task_stage_ = AutoChargingTaskStage::CHARGING_POINT_REACHED;
                }
                return TaskStatus::EXECUTING;
            } else {
                LOG(ERROR) << "navigation failed, charging task failed!" << std::endl;
                curr_task_stage_ = AutoChargingTaskStage::FAILED;
                return TaskStatus::FAILED;
            }
        }
    }

    if (curr_task_stage_ == AutoChargingTaskStage::CHARGING_POINT_REACHED) {
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

        curr_task_stage_ = AutoChargingTaskStage::DOCKING;
        return TaskStatus::EXECUTING;
    }

    if (curr_task_stage_ == AutoChargingTaskStage::DOCKING) {
        // 等待上桩完成
        if (mission_info_->is_docking_finished()) {
            if (mission_info_->is_docking_success() && mission_info_->GetRobotStatus()->is_charging) {
                // mission_info_->SetNeedCancelPerception();
                curr_task_stage_ = AutoChargingTaskStage::FINISHED;
                return TaskStatus::FINISHED;
            } else {
                LOG(ERROR) << "docking failed, charging task failed!" << std::endl;
                curr_task_stage_ = AutoChargingTaskStage::FAILED;
                return TaskStatus::FAILED;
            }
        }
        return TaskStatus::EXECUTING;
    }

    if (curr_task_stage_ == AutoChargingTaskStage::FINISHED) {
        return TaskStatus::FINISHED;
    }

    return TaskStatus::FAILED;
}

TaskStatus TaskAutoCharging::Pause() {
    std::lock_guard<std::mutex> lock(status_mutex_);
    // 如果在等待过程中暂停任务，则 TaskStatus 状态为 FINISHED，此时不需要充值任务
    if (curr_task_stage_ == AutoChargingTaskStage::FINISHED) {
        return TaskStatus::PAUSING;
    }
    mission_info_->ResetStatus();
    curr_task_stage_ = AutoChargingTaskStage::INIT;
    // 如果到达了最后一个点，在对接的时候进行取消任务，启动的时候再回到最后一个任务点
    if (curr_goal_idx_ >= passing_by_.size()) {
        curr_goal_idx_ -= 1;
    }

    is_paussing_ = true;
    return TaskStatus::PAUSING;
}

TaskStatus TaskAutoCharging::Resume() {
    if (!is_paussing_) {
        LOG(INFO) << "TaskNavigation::Resume() called but not in pausing state";
        return TaskStatus::EXECUTING;
    }
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (curr_task_stage_ == AutoChargingTaskStage::FINISHED) {
        return TaskStatus::FINISHED;
    }
    mission_info_->ResetStatus();
    curr_task_stage_ = AutoChargingTaskStage::INIT;
    is_paussing_ = false;

    // 这里不改变curr_goal_idx_，还是从断点前的目标点开始执行
    return TaskStatus::EXECUTING;
}

TaskStatus TaskAutoCharging::Restart() {
    ResetTask();
    return TaskStatus::EXECUTING;
}

void TaskAutoCharging::ResetTask() {
    std::lock_guard<std::mutex> lock(status_mutex_);
    is_paussing_ = false;
    curr_task_stage_ = AutoChargingTaskStage::INIT;
    mission_info_->ResetStatus();
    curr_goal_.reset();
    curr_goal_idx_ = 0;
    task_progress_ = 0.0;
}

}  // namespace pnc
