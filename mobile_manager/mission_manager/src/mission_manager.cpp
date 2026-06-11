#include <glog/logging.h>

#include <fstream>
// #include "missing_manager/mission_manager.hpp"

#include "mission_manager/mission_manager.hpp"
#include "mission_manager/tasks/task_auto_charging.hpp"

namespace pnc {

MissionManager::MissionManager(const MissionManagerParam& param) : param_(param) {
    mission_info_ = std::make_shared<MissionInfo>();
}

void MissionManager::UpdateRobotStatus(const std::shared_ptr<RobotStatus>& status) {
    assert(mission_info_);
    mission_info_->UpdateRobotStatus(status);

    // LOG(INFO) << *mission_info_->GetRobotStatus() << std::endl;
}

void MissionManager::Run() {
    std::lock_guard<std::recursive_mutex> s_lock(status_mutex_);
    MaybeFinishCurrentTask();

    MaybeLoadTaskFromGroup();
    MaybeLoadCurrentTask();
    MaybePauseCurrentTask();

    // MaybeAddAutoChargingTask();
    if (valid_task_deque_.empty() && task_group_deque_.empty() && current_task_ == nullptr) {
        if (manager_status_ != ManagerStatus::FAILED) {
            manager_status_ = ManagerStatus::IDLE;
        }
        return;
    }

    if (manager_status_ == ManagerStatus::PAUSING) {
        // 如果是暂停状态，则不执行任务
        return;
    }

    if (current_task_ == nullptr) {
        return;
    }

    TaskStatus task_status = current_task_->Execute();
    MaybeRespawnCurrentTask(task_status);
    UpdateManagerStatus(task_status);
    MaybeWaiting();
    // std::cout << "task_status: " << task_status << std::endl;
    // std::cout << "manager_status_: " << manager_status_ << std::endl;
    // if (current_task_) {
    //   std::cout << "current_task_: " << current_task_->name() << std::endl;
    // }
    // std::cout << "valid_task_deque_.size(): " << valid_task_deque_.size() << std::endl;
    // std::cout << "task_group_deque_.size(): " << task_group_deque_.size() << std::endl;
}

void MissionManager::UpdateManagerStatus(const TaskStatus& task_status) {
    std::lock_guard<std::recursive_mutex> s_lock(status_mutex_);
    switch (task_status) {
        case TaskStatus::EXECUTING:
            manager_status_ = ManagerStatus::EXECUTING;
            break;
        case TaskStatus::PAUSING:
            manager_status_ = ManagerStatus::PAUSING;
            break;
        case TaskStatus::FINISHED: {
            if (manager_status_ == ManagerStatus::PAUSING) {
                // 任务完成状态，且manager为暂停，说明在waiting中发生了暂停
                // 暂停恢复后，直接完成当前任务
                manager_status_ = ManagerStatus::FINISHED;
            } else {
                manager_status_ = ManagerStatus::WAITING;
            }
            break;
        }
        case TaskStatus::CANCELED:
            manager_status_ = ManagerStatus::FAILED;
            break;
        case TaskStatus::FAILED: {
            // 如果当前任务失败，则从队列中删除，如果重新执行该任务需要重新下发
            // 完成任务，从队列中删除
            manager_status_ = ManagerStatus::FAILED;
            CancelCurrentTask();
        } break;
    }
}

void MissionManager::MaybeRespawnCurrentTask(TaskStatus& task_status) {
    assert(current_task_);
    if (!param_.respawn_task) {
        return;
    }

    if (task_status != TaskStatus::FAILED) {
        return;
    }
    std::lock_guard<std::recursive_mutex> s_lock(status_mutex_);
    std::lock_guard<std::recursive_mutex> t_lock(task_mutex_);

    LOG(WARNING) << "Current task: " << current_task_->name() << " is FAILED, respawn current task" << std::endl;
    task_status = current_task_->Restart();
    LOG(INFO) << "Task status after respawn: " << task_status << std::endl;
}

void MissionManager::MaybeLoadTaskFromGroup() {
    // 当任务队列为空的时候，才需要加载任务队列
    if (!valid_task_deque_.empty()) {
        return;
    }

    if (task_group_deque_.empty()) {
        return;
    }

    if (task_group_deque_.front()->IsFinished()) {
        return;
    }

    if (!task_group_deque_.front()->CanLoadNextCycle()) {
        return;
    }

    if (task_group_deque_.front()->GetTaskList().empty()) {
        return;
    }

    // 将task group中的task加入到任务队列中
    const auto& task_list = task_group_deque_.front()->GetTaskList();

    for (const auto& task : task_list) {
        valid_task_deque_.push_back(task);
    }
    LOG(INFO) << "Load task group: " << task_group_deque_.front()->name() << std::endl;
    LOG(INFO) << "Task count: " << task_group_deque_.front()->GetTaskList().size() << std::endl;
}

void MissionManager::MaybeLoadCurrentTask() {
    if (current_task_ != nullptr) {
        return;
    }

    if (valid_task_deque_.empty()) {
        return;
    }

    current_task_ = valid_task_deque_.front();
    current_task_->SetMissionInfo(mission_info_);
    current_task_->ResetTask();
    mission_info_->SetNeedCancelPerception();
    LOG(INFO) << "Load task: " << current_task_->name() << std::endl;
    LOG(INFO) << "Task in this group remain: " << valid_task_deque_.size() << std::endl;
}

// TODO: 需要编写充电时，暂停其他所有任务的逻辑；等充电大于60%以后，继续执行任务

void MissionManager::MaybeAddAutoChargingTask() {
    if (IsNeedAutoCharging()) {
        AddAutoChargingTask();
    }
}

bool MissionManager::IsNeedAutoCharging() {
    if (mission_info_->GetRobotStatus()->emergency_stop) {
        return false;
    }

    if (mission_info_->GetRobotStatus()->is_charging) {
        return false;
    }

    // if (task_group_deque_.empty() && valid_task_deque_.empty()) {
    //   return false;
    // }

    if (mission_info_->GetRobotStatus()->soc < param_.auto_charge_threshold) {
        if (manager_status_ == ManagerStatus::IDLE) {
            return true;
        }
    }

    if (mission_info_->GetRobotStatus()->soc < param_.force_charge_threshold) {
        return true;
    }

    return false;
}

void MissionManager::AddAutoChargingTask() {
    std::shared_ptr<NavPoint> nearest_charge_station = GetNearestNavPoint(mission_info_->GetRobotStatus()->pose, true);

    auto task = std::make_shared<TaskAutoCharging>();
    std::string task_name = "auto_charging_" + nearest_charge_station->name;
    task->Init(task_name);
    task->SetPassingBy({nearest_charge_station});
    this->AddTasks({task});
}

void MissionManager::MaybePauseCurrentTask() {
    if (mission_info_->GetRobotStatus()->auto_mode && current_task_ == nullptr) {
        manager_status_ = ManagerStatus::IDLE;
        return;
    }

    if (manager_status_ == ManagerStatus::PAUSING) {
        return;
    }
    // 手动模式下，暂停当前任务
    if (!mission_info_->GetRobotStatus()->auto_mode) {
        LOG(WARNING) << "Manual mode, pause current task" << std::endl;
        PauseCurrentTask();
    }

    // 急停状态下，暂停当前任务
    if (mission_info_->GetRobotStatus()->emergency_stop) {
        LOG(WARNING) << "Emergency stop active, pause current task" << std::endl;
        PauseCurrentTask();
    }
}

std::shared_ptr<NavPoint> MissionManager::GetNearestNavPoint(const Eigen::Vector3d& pose, bool is_charging_point) {
    std::shared_ptr<NavPoint> nearest_point;
    // 找到最近的充电桩 find available charging station
    float min_dist = 1000000.0;
    for (const auto& pair : nav_point_umap_) {
        const auto nav_point = pair.second;
        float dist = 1000000.0;
        if (is_charging_point) {
            if (nav_point->point_type == NavPointType::CHARGING) {
                dist = std::sqrt(std::pow(nav_point->pose.x - pose.x(), 2) + std::pow(nav_point->pose.y - pose.y(), 2));
            }
        } else {
            dist = std::sqrt(std::pow(nav_point->pose.x - pose.x(), 2) + std::pow(nav_point->pose.y - pose.y(), 2));
        }
        if (dist < min_dist) {
            min_dist = dist;
            nearest_point = nav_point;
        }
    }
    return nearest_point;
}

bool MissionManager::AddTaskGroup(const std::shared_ptr<TaskGroup>& task_group) {
    if (task_group == nullptr) {
        return false;
    }
    if (task_group->max_execute_count() == 0) {
        LOG(ERROR) << "Task group max_execute_count must be greater than 0";
        return false;
    }

    for (const auto& group : task_group_deque_) {
        if (group->name() == task_group->name()) {
            LOG(WARNING) << "Task group with name: " << task_group->name() << " already exists";
            return false;
        }
    }

    task_group_deque_.push_back(task_group);
    return true;
}

bool MissionManager::AddTasks(const std::vector<std::shared_ptr<TaskInterface>>& task_in) {
    std::lock_guard<std::recursive_mutex> t_lock(task_mutex_);
    for (const auto& task : task_in) {
        // note: task要和 mission_info_ 进行绑定
        valid_task_deque_.push_back(task);
    }
    return true;
}

const std::vector<std::shared_ptr<TaskInterface>> MissionManager::GetTaskList() {
    std::lock_guard<std::recursive_mutex> t_lock(task_mutex_);
    std::vector<std::shared_ptr<TaskInterface>> res;

    std::copy(valid_task_deque_.begin(), valid_task_deque_.end(), std::back_inserter(res));

    return res;
}

bool MissionManager::PauseCurrentTask() {
    if (manager_status_ == ManagerStatus::PAUSING) {
        return true;
    }
    manager_status_ = ManagerStatus::PAUSING;

    if (valid_task_deque_.empty()) {
        LOG(INFO) << "No task in MissionManager." << std::endl;
        return true;
    }

    if (current_task_ == nullptr) {
        LOG(INFO) << "No current task." << std::endl;
        return true;
    }

    TaskStatus task_status = current_task_->Pause();
    UpdateManagerStatus(task_status);

    mission_info_->SetNeedCancelNavigation();
    mission_info_->SetNeedCancelDocking();
    mission_info_->SetNeedCancelPerception();

    return manager_status_ == ManagerStatus::PAUSING;
}

bool MissionManager::ResumeCurrentTask() {
    std::lock_guard<std::recursive_mutex> t_lock(task_mutex_);

    if (valid_task_deque_.empty()) {
        LOG(INFO) << "No task in MissionManager." << std::endl;
        return true;
    }

    current_task_ = valid_task_deque_.front();
    TaskStatus task_status = current_task_->Resume();
    UpdateManagerStatus(task_status);
    // Task的状态
    return (manager_status_ == ManagerStatus::EXECUTING || manager_status_ == ManagerStatus::FINISHED);
}

bool MissionManager::CancelCurrentTask() {
    std::lock_guard<std::recursive_mutex> s_lock(status_mutex_);
    std::lock_guard<std::recursive_mutex> t_lock(task_mutex_);

    if (valid_task_deque_.empty()) {
        LOG(INFO) << "No task in MissionManager." << std::endl;
        return true;
    }

    if (current_task_ == nullptr) {
        LOG(INFO) << "No current task." << std::endl;
        return true;
    }

    if (current_task_->name() == valid_task_deque_.front()->name()) {
        LOG(WARNING) << "Cancel current task : " << current_task_->name() << std::endl;
        valid_task_deque_.pop_front();
        task_group_deque_.front()->IncreaseExecuteCount();
        if (task_group_deque_.front()->IsFinished()) {
            task_group_deque_.pop_front();
        }
    } else {
        LOG(ERROR) << "Current task : " << current_task_->name() << " is not the front task in queue." << std::endl;
        manager_status_ = ManagerStatus::FAILED;
        return false;
    }

    // TaskStatus task_status = current_task_->Cancel();
    // UpdateManagerStatus(task_status);

    current_task_.reset();

    mission_info_->SetNeedCancelNavigation();
    mission_info_->SetNeedCancelDocking();
    mission_info_->SetNeedCancelPerception();
    return true;
}

bool MissionManager::CancelCurrentTaskGroup() {
    std::lock_guard<std::recursive_mutex> s_lock(status_mutex_);
    std::lock_guard<std::recursive_mutex> t_lock(task_mutex_);

    if (!CancelCurrentTask()) {
        LOG(INFO) << "Cancel current task failed." << std::endl;
        return false;
    }
    if (task_group_deque_.empty()) {
        LOG(INFO) << "No task group in MissionManager." << std::endl;
        return true;
    }
    LOG(WARNING) << "Cancel current task group : " << task_group_deque_.front()->name() << std::endl;

    task_group_deque_.pop_front();
    valid_task_deque_.clear();

    manager_status_ = ManagerStatus::IDLE;

    return true;
}

void MissionManager::MaybeWaiting() {
    if (manager_status_ != ManagerStatus::WAITING) {
        waiting_tic_.reset();
        return;
    }

    if (current_task_ == nullptr) {
        return;
    }

    if (waiting_tic_ == nullptr) {
        waiting_tic_ = std::make_shared<nav_util::TicToc>();
        // waiting的时候就把对应的任务取消掉（或者在任务完成的时候取消掉都可以，关系不大）
        mission_info_->SetNeedCancelDocking();
        mission_info_->SetNeedCancelPerception();
    }

    double time_count = waiting_tic_->toc() * 1e-3;
    if (time_count > current_task_->waiting_time()) {
        LOG(INFO) << "Waiting for " << time_count << " seconds." << std::endl;
        waiting_tic_.reset();
        manager_status_ = ManagerStatus::FINISHED;
    }
}

void MissionManager::MaybeFinishCurrentTask() {
    if (manager_status_ != ManagerStatus::FINISHED) {
        return;
    }

    if (valid_task_deque_.empty()) {
        return;
    }

    if (task_group_deque_.empty()) {
        return;
    }

    if (current_task_ == nullptr) {
        return;
    }

    LOG(INFO) << "Current task : " << current_task_->name() << " finished." << std::endl;

    if (current_task_->name() == valid_task_deque_.front()->name()) {
        valid_task_deque_.pop_front();
    } else {
        LOG(ERROR) << "Current task : " << current_task_->name() << " is not the front task in queue." << std::endl;
        manager_status_ = ManagerStatus::FAILED;
        return;
    }

    current_task_.reset();

    // 当前任务队列中的任务执行完毕
    if (valid_task_deque_.empty()) {
        task_group_deque_.front()->IncreaseExecuteCount();
        if (!task_group_deque_.front()->IsFinished()) {
            task_group_deque_.front()->StartWaiting();  // 等待下次任务
        }
    }

    if (task_group_deque_.front()->IsFinished()) {
        task_group_deque_.pop_front();
    }

    // note: 如果这里设置了取消任务，则就会出现冲突问题，导致nav_monitor挂掉
    // mission_info_->SetNeedCancelNavigation();
    // mission_info_->SetNeedCancelDocking();
    // mission_info_->SetNeedCancelPerception();
}

// 按照名称删除特定的任务
bool MissionManager::RemoveTask(const std::string& task_name) {
    std::lock_guard<std::recursive_mutex> s_lock(status_mutex_);
    std::lock_guard<std::recursive_mutex> t_lock(task_mutex_);

    return true;
}

bool MissionManager::ClearAllTasks() {
    std::lock_guard<std::recursive_mutex> s_lock(status_mutex_);
    std::lock_guard<std::recursive_mutex> t_lock(task_mutex_);
    if (!CancelCurrentTask()) {
        return false;
    }

    current_task_.reset();
    valid_task_deque_.clear();
    task_group_deque_.clear();

    manager_status_ = ManagerStatus::IDLE;
    mission_info_->SetNeedCancelNavigation();
    mission_info_->SetNeedCancelDocking();
    mission_info_->SetNeedCancelPerception();
    LOG(INFO) << "All tasks cleared." << std::endl;
    return true;
}

bool MissionManager::SetNavPoints(const std::vector<std::shared_ptr<NavPoint>>& nav_points) {
    for (const auto& nav_point : nav_points) {
        nav_point_umap_[nav_point->name] = nav_point;
    }
    return true;
}

const std::shared_ptr<NavPoint> MissionManager::GetCurrentGoal() const {
    if (current_task_ == nullptr) {
        return nullptr;
    }
    return current_task_->GetCurrentGoal();
}

const std::shared_ptr<DockingInfo> MissionManager::GetDockingInfo() const {
    if (current_task_ == nullptr) {
        return nullptr;
    }
    return current_task_->GetDockingInfo();
}

const uint32_t MissionManager::GetCurrGroupExecutionLeft() {
    const uint32_t& execute_count = task_group_deque_.front()->execute_count();
    return task_group_deque_.front()->max_execute_count() - execute_count;
}

const ManagerStatus& MissionManager::GetManagerStatus() {
    std::lock_guard<std::recursive_mutex> s_lock(status_mutex_);
    return manager_status_;
}

}  // namespace pnc
