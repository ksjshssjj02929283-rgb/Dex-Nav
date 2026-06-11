#ifndef MISSION_MANAGER__DOCKING_INFO_HPP_
#define MISSION_MANAGER__DOCKING_INFO_HPP_

#include <Eigen/Core>
#include <string>

namespace pnc {

class DockingInfo {
public:
    DockingInfo() = default;
    ~DockingInfo() = default;

    enum class TaskType {
        UNKOWN = 0,
        DOCKING_PILE = 1,  // 上桩充电
        EXIT_PILE = 2,     // 退出充电桩
        PICK_UP = 3        // 货架对接
    };

    enum class ModelType {
        UNKOWN = 0,
        POINT = 1,           // 到达指定点位
        CHARGE_STATION = 2,  // 与充电桩对接
        SHELF = 3            // 与货架对接
    };

    void SetTaskType(TaskType task_type) { task_type_ = task_type; }

    TaskType GetTaskType() const { return task_type_; }

    void SetModelType(ModelType task_type) { model_type_ = task_type; }

    ModelType GetModelType() const { return model_type_; }

    void SetModelName(std::string model_name) { model_name_ = model_name; }

    std::string GetModelName() const { return model_name_; }

    void SetModelPose(Eigen::Vector3d model_pose) { pose_ = model_pose; }

    Eigen::Vector3d GetModelPose() const { return pose_; }

private:
    TaskType task_type_;
    std::string model_name_;  // 对接结构的名称
    ModelType model_type_;    // 任务的类型
    Eigen::Vector3d pose_;    // 对接初始的位姿
};

}  // namespace pnc

#endif  // MISSION_MANAGER__DOCKING_INFO_HPP_
