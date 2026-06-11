#include "mission_manager/tasks/task_exit_pile.hpp"

#include <memory>
#include <thread>
#include <vector>

#include "glog/logging.h"

namespace pnc {

TaskExitPile::TaskExitPile() {
    curr_task_stage_ = ExitPileTaskStage::INIT;
    task_type_ = TaskType::EXIT_PILE;
    docking_info_ = std::make_shared<DockingInfo>();
}

bool TaskExitPile::Init(const std::string& task_name) {
    name_ = task_name;
    return true;
}

TaskStatus TaskExitPile::Execute() {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (mission_info_ == nullptr) {
        LOG(ERROR) << "mission info is not set, navigation task failed!" << std::endl;
        return TaskStatus::FAILED;
    }

    if (is_paussing_) {
        // 如果任务被终止，则充电任务从头开始执行
        return Pause();
    }
    if (curr_task_stage_ == ExitPileTaskStage::INIT) {
        // 首先，判断是否是充电状态，导航到最近的充电点
        if (mission_info_->GetRobotStatus()->is_charging) {
            docking_info_->SetTaskType(DockingInfo::TaskType::EXIT_PILE);
            docking_info_->SetModelType(DockingInfo::ModelType::POINT);
            docking_info_->SetModelName("exit_point");
            Eigen::Vector3d point_pose;
            point_pose << 0.4, 0.0, 0.0;
            docking_info_->SetModelPose(point_pose);
            mission_info_->SetNeedDocking();
            // mission_info_->SetNeedCancelPerception();

            curr_task_stage_ = ExitPileTaskStage::EXIT_CHARGE_STATION;
        } else {
            // 如果没有充电，则直接返回成功
            curr_task_stage_ = ExitPileTaskStage::FINISHED;
        }
        return TaskStatus::EXECUTING;
    }

    if (curr_task_stage_ == ExitPileTaskStage::EXIT_CHARGE_STATION) {
        if (mission_info_->is_docking_finished()) {
            if (mission_info_->is_docking_success()) {
                curr_task_stage_ = ExitPileTaskStage::FINISHED;
            } else {
                LOG(ERROR) << "docking failed, docking exit pile task failed!" << std::endl;
                curr_task_stage_ = ExitPileTaskStage::FAILED;
                return TaskStatus::FAILED;
            }
        }
        return TaskStatus::EXECUTING;
    }

    if (curr_task_stage_ == ExitPileTaskStage::FINISHED) {
        return TaskStatus::FINISHED;
    }

    return TaskStatus::FAILED;
}

TaskStatus TaskExitPile::Pause() {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (curr_task_stage_ == ExitPileTaskStage::FINISHED) {
        return TaskStatus::PAUSING;
    }
    is_paussing_ = true;
    curr_task_stage_ = ExitPileTaskStage::INIT;
    mission_info_->ResetStatus();
    return TaskStatus::PAUSING;
}

TaskStatus TaskExitPile::Resume() {
    if (!is_paussing_) {
        LOG(INFO) << "TaskExitPile::Resume() called but not in pausing state";
        return TaskStatus::EXECUTING;
    }
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (curr_task_stage_ == ExitPileTaskStage::FINISHED) {
        return TaskStatus::FINISHED;
    }
    curr_task_stage_ = ExitPileTaskStage::INIT;
    mission_info_->ResetStatus();
    is_paussing_ = false;
    return TaskStatus::EXECUTING;
}

TaskStatus TaskExitPile::Restart() {
    ResetTask();
    return TaskStatus::EXECUTING;
}

void TaskExitPile::ResetTask() {
    std::lock_guard<std::mutex> lock(status_mutex_);
    is_paussing_ = false;
    curr_task_stage_ = ExitPileTaskStage::INIT;
    mission_info_->ResetStatus();
    task_progress_ = 0.0;
}

}  // namespace pnc
