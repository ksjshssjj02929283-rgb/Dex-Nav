#ifndef MISSION_MANAGER__MISSION_INFO_HPP_
#define MISSION_MANAGER__MISSION_INFO_HPP_

#include <iostream>
#include <memory>
#include <vector>

#include "mission_manager/nav_point.hpp"
#include "mission_manager/robot_status.hpp"

namespace pnc {

// 任务信息（用于交互）
class MissionInfo {
public:
    MissionInfo() : robot_status_(std::make_shared<RobotStatus>()) {}

    ~MissionInfo() = default;

    void UpdateRobotStatus(const std::shared_ptr<RobotStatus>& status) { robot_status_ = status; }

    const std::shared_ptr<RobotStatus>& GetRobotStatus() const { return robot_status_; }

    void SetCurrentGoal(const std::shared_ptr<NavPoint>& goal) { current_goal_ = goal; }

    const std::shared_ptr<NavPoint>& GetCurrentGoal() const { return current_goal_; }

    bool is_need_navigation() const { return is_need_navigation_; }

    bool is_need_docking() const { return is_need_docking_; }

    bool is_navigating() const { return is_navigating_; }

    bool is_docking() const { return is_docking_; }

    bool is_need_cancel_navigation() const { return is_need_cancel_navigation_; }

    bool is_need_cancel_docking() const { return is_need_cancel_docking_; }

    bool is_docking_finished() const { return is_docking_finished_; }

    bool is_docking_success() const { return is_docking_success_; }

    bool is_navigation_finished() const { return is_navigation_finished_; }

    bool is_navigation_success() const { return is_navigation_success_; }

    bool is_need_perception() const { return is_need_perception_; }

    bool is_need_cancel_perception() const { return is_need_cancel_perception_; }

    void SetNeedNavigation() {
        is_need_navigation_ = true;
        is_need_cancel_navigation_ = false;
        // navigation和docking是互斥的
        is_need_docking_ = false;
    }

    void SetNeedCancelNavigation() {
        is_need_cancel_navigation_ = true;
        is_need_navigation_ = false;
    }

    void SetNeedDocking() {
        is_need_docking_ = true;
        is_need_cancel_docking_ = false;
        // navigation和docking是互斥的
        is_need_navigation_ = false;
    }

    void SetNeedCancelDocking() {
        is_need_cancel_docking_ = true;
        is_need_docking_ = false;
    }

    void SetNeedPerception() {
        is_need_perception_ = true;
        is_need_cancel_perception_ = false;
    }

    void SetNeedCancelPerception() {
        is_need_cancel_perception_ = true;
        is_need_perception_ = false;
    }

    void SetNavigating() {
        ResetStatus();
        is_navigating_ = true;
    }

    void SetDocking() {
        ResetStatus();
        is_docking_ = true;
    }

    void SetNavigationSuccess(const bool is_success) {
        ResetStatus();
        is_navigation_finished_ = true;
        is_navigation_success_ = is_success;
    }

    void SetDockingSuccess(const bool is_success) {
        ResetStatus();
        is_docking_finished_ = true;
        is_docking_success_ = is_success;
    }

    void SetPerceptionFinished() {
        is_need_perception_ = false;
        is_need_cancel_perception_ = false;
    }

    // ResetStatus不重置perception的状态
    void ResetStatus() {
        is_need_docking_ = false;
        is_need_navigation_ = false;

        is_need_cancel_navigation_ = false;
        is_need_cancel_docking_ = false;

        is_navigating_ = false;
        is_docking_ = false;

        is_navigation_finished_ = false;
        is_docking_finished_ = false;

        is_navigation_success_ = false;
        is_docking_success_ = false;
    }

private:
    std::shared_ptr<NavPoint> current_goal_;
    std::shared_ptr<RobotStatus> robot_status_;

    bool is_need_docking_ = false;
    bool is_need_navigation_ = false;

    bool is_navigating_ = false;  // 正在导航
    bool is_docking_ = false;     // 正在对接
    bool is_need_perception_ = false;

    bool is_need_cancel_navigation_ = false;
    bool is_need_cancel_docking_ = false;
    bool is_need_cancel_perception_ = false;

    bool is_navigation_finished_ = false;  // 导航完成
    bool is_docking_finished_ = false;     // 对接完成

    bool is_navigation_success_ = false;  // 导航是否成功
    bool is_docking_success_ = false;     // 对接是否成功

    bool is_task_success_ = false;
};

}  // namespace pnc

#endif  // MISSION_MANAGER__MISSION_INFO_HPP_