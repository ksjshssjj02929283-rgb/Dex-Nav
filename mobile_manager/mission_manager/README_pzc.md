# Mission Manager 代码分析文档

## 概述

Mission Manager 是一个基于 ROS 的任务管理系统，用于管理机器人的各种任务执行，包括导航、对接、充电等操作。它提供了完整的任务调度、状态管理和执行控制功能，是机器人自主导航系统的核心组件。

## 主要功能

### 1. 任务管理
- **任务队列管理**: 支持任务组和单个任务的队列管理
- **任务调度**: 按优先级和顺序执行任务
- **任务状态跟踪**: 实时监控任务执行状态
- **任务控制**: 支持暂停、恢复、取消任务操作

### 2. 多种任务类型支持
- **导航任务 (NAVIGATION)**: 机器人点到点导航
- **对接任务 (DOCKING_PILE)**: 与充电桩对接
- **出桩任务 (EXIT_PILE)**: 从充电桩退出
- **自动充电任务 (AUTO_CHARGING)**: 自动充电流程

### 3. 智能充电管理
- **自动充电**: 根据电量阈值自动触发充电任务
- **强制充电**: 低电量时强制执行充电
- **充电状态监控**: 实时监控充电状态

### 4. 状态管理
- **机器人状态**: 位姿、速度、电量、急停状态等
- **任务状态**: 执行中、暂停、完成、失败等
- **系统状态**: 空闲、执行中、暂停、等待等

## 核心类结构

### 1. MissionManager (任务管理器主类)

**文件位置**: `src/mission_manager.cpp`, `include/mission_manager/mission_manager.hpp`

**主要功能**:
- 管理整个任务执行系统
- 协调各种任务类型的执行
- 处理任务状态转换

**关键方法**:
```cpp
// 构造函数，初始化任务管理器
MissionManager(const MissionManagerParam& param);

// 主运行循环
void Run();

// 更新机器人状态
void UpdateRobotStatus(const std::shared_ptr<RobotStatus>& status);

// 添加任务
bool AddTasks(const std::vector<std::shared_ptr<TaskInterface>>& task_in);
bool AddTaskGroup(const std::shared_ptr<TaskGroup>& task_group);

// 任务控制
bool PauseCurrentTask();
bool ResumeCurrentTask();
bool CancelCurrentTask();
bool CancelCurrentTaskGroup();
bool ClearAllTasks();

// 状态查询
const ManagerStatus& GetManagerStatus();
const std::shared_ptr<TaskInterface> GetCurrTask() const;
const std::shared_ptr<NavPoint> GetCurrentGoal() const;
const std::shared_ptr<DockingInfo> GetDockingInfo() const;
```

### 2. MissionManagerROS (ROS接口类)

**文件位置**: `apps/mission_manager_ros.cpp`, `apps/mission_manager_ros.hpp`

**主要功能**:
- 提供ROS服务接口
- 处理ROS话题订阅和发布
- 与外部系统通信

**关键方法**:
```cpp
// 构造函数，初始化ROS接口
MissionManagerROS();

// 主运行循环
void Run(const ros::TimerEvent& event);

// 服务回调函数
bool AddTaskCallback(dcm_srvs::AddTask::Request& req, dcm_srvs::AddTask::Response& res);
bool AddTaskGroupCallback(dcm_srvs::AddTaskGroup::Request& req, dcm_srvs::AddTaskGroup::Response& res);
bool PauseTaskCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
bool ResumeTaskCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
bool CancelTaskCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

// 导航控制
bool RequestStartNavigation();
bool RequestCancelNavigation();

// 对接控制
bool RequestStartDocking();
bool RequestCancelDocking();

// 感知控制
bool RequestStartPerception();
bool RequestCancelPerception();
```

### 3. TaskInterface (任务接口基类)

**文件位置**: `include/mission_manager/task_interface.hpp`

**主要功能**:
- 定义任务的标准接口
- 提供任务状态管理
- 支持任务生命周期控制

**关键方法**:
```cpp
// 任务生命周期控制
virtual bool Init(const std::string & task_name) = 0;
virtual TaskStatus Execute() = 0;
virtual TaskStatus Pause() = 0;
virtual TaskStatus Resume() = 0;
virtual TaskStatus Restart() = 0;
virtual void ResetTask() = 0;
virtual TaskStatus Cancel();

// 任务配置
void SetPassingBy(const std::vector<std::shared_ptr<NavPoint>>& passing_by);
void SetWaitingTime(const double waiting_time);
void SetPriority(const TaskPriority priority);

// 状态查询
const std::string name() const;
const TaskType task_type() const;
const TaskStatus task_status() const;
const double progress() const;
const TaskPriority priority() const;
const double waiting_time() const;
```

### 4. TaskGroup (任务组类)

**文件位置**: `include/mission_manager/task_group.hpp`

**主要功能**:
- 管理一组相关任务
- 支持任务组循环执行
- 控制任务组执行间隔

**关键方法**:
```cpp
// 任务组管理
void AddTask(std::shared_ptr<TaskInterface> task);
void SetMaxExecuteCount(const uint32_t count);
void SetInverval(const double interval);

// 执行控制
void StartWaiting();
bool CanLoadNextCycle();
bool IsFinished() const;
void IncreaseExecuteCount();

// 状态查询
const std::string name() const;
const std::vector<std::shared_ptr<TaskInterface>>& GetTaskList() const;
const uint32_t max_execute_count() const;
const uint32_t execute_count() const;
const double interval() const;
```

### 5. 具体任务实现类

#### TaskNavigation (导航任务)
**文件位置**: `src/tasks/task_navigation.cpp`, `include/mission_manager/tasks/task_navigation.hpp`

**主要功能**:
- 执行点到点导航任务
- 支持多路径点导航
- 处理导航失败和重试

#### TaskAutoCharging (自动充电任务)
**文件位置**: `src/tasks/task_auto_charging.cpp`, `include/mission_manager/tasks/task_auto_charging.hpp`

**主要功能**:
- 自动充电流程管理
- 导航到充电桩
- 执行对接和充电操作

#### TaskDockingPile (对接任务)
**文件位置**: `src/tasks/task_docking_pile.cpp`, `include/mission_manager/tasks/task_docking_pile.hpp`

**主要功能**:
- 与充电桩对接
- 处理对接失败情况
- 监控对接状态

#### TaskExitPile (出桩任务)
**文件位置**: `src/tasks/task_exit_pile.cpp`, `include/mission_manager/tasks/task_exit_pile.hpp`

**主要功能**:
- 从充电桩退出
- 安全退出流程
- 退出状态监控

### 6. MissionInfo (任务信息类)

**文件位置**: `include/mission_manager/mission_info.hpp`

**主要功能**:
- 管理任务执行过程中的状态信息
- 协调导航、对接、感知等模块
- 提供状态查询接口

**关键方法**:
```cpp
// 状态设置
void SetNeedNavigation();
void SetNeedDocking();
void SetNeedPerception();
void SetNeedCancelNavigation();
void SetNeedCancelDocking();
void SetNeedCancelPerception();

// 状态查询
bool is_need_navigation() const;
bool is_need_docking() const;
bool is_need_perception() const;
bool is_navigating() const;
bool is_docking() const;
bool is_navigation_finished() const;
bool is_docking_finished() const;
bool is_navigation_success() const;
bool is_docking_success() const;

// 状态重置
void ResetStatus();
```

## 数据结构

### 1. 任务状态枚举

```cpp
// 任务状态
enum class TaskStatus {
    EXECUTING = 1,  // 执行中
    PAUSING = 2,    // 暂停中
    FINISHED = 3,   // 已完成
    CANCELED = 4,   // 已取消
    FAILED = 5      // 失败
};

// 任务类型
enum class TaskType {
    UNKNOWN = -1,
    NAVIGATION = 0,      // 导航任务
    DOCKING_PILE = 1,    // 对接任务
    EXIT_PILE = 2,       // 出桩任务
    AUTO_CHARGING = 3    // 自动充电任务
};

// 任务优先级
enum class TaskPriority {
    NORMAL = 0,      // 普通优先级
    HIGH = 1,        // 高优先级
    EMERGENCY = 2    // 紧急优先级
};

// 管理器状态
enum class ManagerStatus {
    FAILED = -1,     // 失败
    IDLE = 0,        // 空闲
    EXECUTING = 1,   // 执行中
    PAUSING = 2,     // 暂停中
    WAITING = 3,     // 等待中
    FINISHED = 4     // 已完成
};
```

### 2. 机器人状态结构

```cpp
typedef struct {
    Eigen::Vector3d pose;        // 位姿 (x, y, theta)
    Eigen::Vector3d velocity;    // 速度 (vx, vy, vtheta)
    double soc;                  // 电量百分比
    bool emergency_stop;         // 急停状态
    bool auto_mode;              // 自动模式
    bool is_charging;            // 充电状态
} RobotStatus;
```

### 3. 导航点结构

```cpp
// 导航点类型
enum NavPointType {
    UNKNOWN = -1,    // 未知类型
    FREE_GOAL = 0,   // 自由目标点
    STATION = 1,     // 站点
    CHARGING = 2,    // 充电点
    PARKING = 3,     // 停车点
    PICKUP = 4       // 取货点
};

// 位姿结构
typedef struct {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
} PointPose;

// 导航点结构
typedef struct {
    std::string name;                    // 目标点名称
    int id;                             // 目标点ID
    NavPointType point_type;            // 目标点类型
    PointPose pose;                     // 位姿
} NavPoint;
```

### 4. 对接信息结构

```cpp
class DockingInfo {
public:
    // 任务类型
    enum class TaskType {
        UNKOWN = 0,
        DOCKING_PILE = 1,   // 上桩充电
        EXIT_PILE = 2,      // 退出充电桩
        PICK_UP = 3         // 货架对接
    };

    // 模型类型
    enum class ModelType {
        UNKOWN = 0,
        POINT = 1,                // 到达指定点位
        CHARGE_STATION = 2,       // 与充电桩对接
        SHELF = 3                 // 与货架对接
    };

private:
    TaskType task_type_;
    std::string model_name_;      // 对接结构名称
    ModelType model_type_;        // 任务类型
    Eigen::Vector3d pose_;        // 对接初始位姿
};
```

## 核心算法

### 1. 任务调度算法

任务管理器使用基于优先级的队列调度算法：

```cpp
void MissionManager::Run() {
    std::lock_guard<std::recursive_mutex> s_lock(status_mutex_);
    
    // 1. 检查当前任务是否完成
    MaybeFinishCurrentTask();
    
    // 2. 从任务组加载任务到队列
    MaybeLoadTaskFromGroup();
    
    // 3. 加载当前任务
    MaybeLoadCurrentTask();
    
    // 4. 检查是否需要暂停任务
    MaybePauseCurrentTask();
    
    // 5. 执行当前任务
    if (current_task_ != nullptr) {
        TaskStatus task_status = current_task_->Execute();
        MaybeRespawnCurrentTask(task_status);
        UpdateManagerStatus(task_status);
        MaybeWaiting();
    }
}
```

### 2. 自动充电判断算法

```cpp
bool MissionManager::IsNeedAutoCharging() {
    // 检查急停状态
    if (mission_info_->GetRobotStatus()->emergency_stop) {
        return false;
    }
    
    // 检查是否正在充电
    if (mission_info_->GetRobotStatus()->is_charging) {
        return false;
    }
    
    // 检查电量阈值
    if (mission_info_->GetRobotStatus()->soc < param_.auto_charge_threshold) {
        if (manager_status_ == ManagerStatus::IDLE) {
            return true;
        }
    }
    
    // 强制充电阈值
    if (mission_info_->GetRobotStatus()->soc < param_.force_charge_threshold) {
        return true;
    }
    
    return false;
}
```

### 3. 任务状态转换算法

```cpp
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
            manager_status_ = ManagerStatus::FAILED;
            CancelCurrentTask();
            break;
        }
    }
}
```

### 4. 最近导航点查找算法

```cpp
std::shared_ptr<NavPoint> MissionManager::GetNearestNavPoint(
    const Eigen::Vector3d& pose, bool is_charging_point) {
    
    std::shared_ptr<NavPoint> nearest_point;
    float min_dist = 1000000.0;
    
    for (const auto& pair : nav_point_umap_) {
        const auto nav_point = pair.second;
        float dist = 1000000.0;
        
        if (is_charging_point) {
            if (nav_point->point_type == NavPointType::CHARGING) {
                dist = std::sqrt(std::pow(nav_point->pose.x - pose.x(), 2) +
                                std::pow(nav_point->pose.y - pose.y(), 2));
            }
        } else {
            dist = std::sqrt(std::pow(nav_point->pose.x - pose.x(), 2) +
                            std::pow(nav_point->pose.y - pose.y(), 2));
        }
        
        if (dist < min_dist) {
            min_dist = dist;
            nearest_point = nav_point;
        }
    }
    
    return nearest_point;
}
```

## 配置参数

### 1. 主要参数 (mission_manager_params.yaml)

```yaml
# 自动充电阈值 (百分比)
auto_charge_threshold: 30

# 强制充电阈值 (百分比)
force_charge_threshold: 20

# 是否启用任务重启
respawn_task: false
```

### 2. 参数说明

- **auto_charge_threshold**: 自动充电触发阈值，当电量低于此值时，在空闲状态下自动触发充电任务
- **force_charge_threshold**: 强制充电阈值，当电量低于此值时，无论当前状态如何都会强制执行充电任务
- **respawn_task**: 任务重启开关，当任务失败时是否自动重启任务

## ROS接口

### 1. 服务接口

```cpp
// 任务管理服务
add_task                    // 添加单个任务
add_task_group             // 添加任务组
add_charge_task            // 添加充电任务
get_current_task_info      // 获取当前任务信息
pause_task                 // 暂停任务
resume_task                // 恢复任务
cancel_task                // 取消任务
cancel_task_group          // 取消任务组
clear_all_tasks            // 清空所有任务

// 状态管理服务
get_mission_status         // 获取任务状态
set_navigation_success     // 设置导航成功状态
set_docking_success        // 设置对接成功状态
set_map_changed            // 设置地图变化
set_command_board          // 设置命令板

// 导航点管理服务
get_all_nav_points         // 获取所有导航点
```

### 2. 话题接口

```cpp
// 订阅话题
/odom                      // 里程计信息
/battery_info              // 电池信息
/emergency_stop            // 急停状态
/nav_mode                  // 导航模式

// 发布话题
/manager_status            // 管理器状态
/current_task_info         // 当前任务信息
/docking_base/goal         // 对接目标
```

### 3. 服务客户端

```cpp
// 导航服务
/move_base/start_navigation    // 开始导航
/move_base/cancel_navigation   // 取消导航

// 导航点服务
/nav_point_manager/get_all_nav_points  // 获取所有导航点

// 对接服务
/docking_base/cancel_docking_task  // 取消对接任务

// 感知服务
/perception_2d_node/activate_perception    // 激活感知
/perception_2d_node/deactivate_perception  // 停用感知
```

## 工作流程

### 1. 系统初始化流程
1. 创建MissionManagerROS实例
2. 初始化ROS节点和参数
3. 创建MissionManager实例
4. 设置TF变换监听器
5. 订阅相关话题
6. 注册服务接口
7. 启动定时器

### 2. 任务执行流程
1. 接收任务请求 (通过服务接口)
2. 创建任务对象
3. 将任务添加到队列
4. 从队列中取出任务执行
5. 监控任务执行状态
6. 处理任务完成/失败
7. 清理任务资源

### 3. 自动充电流程
1. 监控机器人电量
2. 判断是否需要充电
3. 查找最近充电桩
4. 创建自动充电任务
5. 导航到充电桩
6. 执行对接操作
7. 监控充电状态
8. 充电完成后继续原任务

### 4. 任务状态转换流程
```
IDLE -> EXECUTING -> WAITING -> FINISHED
  |         |           |
  |         |           +-> PAUSING
  |         |
  |         +-> FAILED
  |
  +-> PAUSING -> EXECUTING
```

## 扩展开发

### 1. 添加新的任务类型
1. 继承TaskInterface类
2. 实现所有纯虚函数
3. 在CreateTaskFromMsg中添加创建逻辑
4. 在TaskType枚举中添加新类型

### 2. 添加新的状态管理
1. 在MissionInfo中添加新状态
2. 实现状态设置和查询方法
3. 在任务执行中使用新状态

### 3. 添加新的ROS接口
1. 在MissionManagerROS中添加服务回调
2. 实现相应的业务逻辑
3. 更新服务注册

## 性能特点

### 1. 实时性
- 主循环运行频率: 20Hz (50ms周期)
- 任务状态更新: 实时
- 机器人状态监控: 实时

### 2. 可靠性
- 任务失败自动重试
- 状态异常自动恢复
- 急停状态立即响应

### 3. 扩展性
- 模块化设计
- 接口标准化
- 配置参数化

## 依赖关系

### ROS包依赖
- `roscpp`: ROS C++客户端库
- `rospy`: ROS Python客户端库
- `geometry_msgs`: 几何消息
- `nav_msgs`: 导航消息
- `std_srvs`: 标准服务
- `tf2_ros`: TF2 ROS接口
- `dcm_msgs`: 自定义消息
- `dcm_srvs`: 自定义服务
- `visualization_msgs`: 可视化消息
- `nav_util`: 导航工具库

### 第三方库依赖
- `glog`: Google日志库
- `Eigen3`: 线性代数库

### 系统要求
- ROS Noetic
- C++11标准
- CMake 3.0+

## 使用示例

### 1. 启动节点
```bash
roslaunch mission_manager mission_manager.launch
```

### 2. 添加导航任务
```bash
rosservice call /mission_manager/add_task "task:
  name: 'navigation_task_1'
  type: 0
  station_names: ['point_A', 'point_B']
  priority: 0
  waiting_time: 0.0"
```

### 3. 添加充电任务
```bash
rosservice call /mission_manager/add_charge_task "task:
  name: 'charging_task_1'
  type: 1
  station_names: ['charging_station_1']
  priority: 2
  waiting_time: 0.0"
```

### 4. 查询任务状态
```bash
rosservice call /mission_manager/get_mission_status "{}"
```

### 5. 暂停/恢复任务
```bash
# 暂停任务
rosservice call /mission_manager/pause_task "{}"

# 恢复任务
rosservice call /mission_manager/resume_task "{}"
```

## 注意事项

1. **地图设置**: 使用前必须设置地图，否则任务添加会失败
2. **导航点管理**: 任务执行前会自动获取最新的导航点信息
3. **状态同步**: 机器人状态更新频率影响任务执行响应速度
4. **电量监控**: 自动充电功能依赖电池信息话题
5. **急停处理**: 急停状态下所有任务会被暂停
6. **模式切换**: 手动模式下任务会被暂停，需要手动恢复

## 调试建议

1. **日志查看**: 使用glog查看详细执行日志
2. **状态监控**: 订阅manager_status话题监控系统状态
3. **任务信息**: 订阅current_task_info话题查看当前任务
4. **参数调整**: 根据实际需求调整充电阈值等参数
5. **服务测试**: 使用rosservice测试各个服务接口

## 总结

Mission Manager 是一个功能完善的任务管理系统，通过模块化设计和标准化接口，为机器人提供了可靠的任务执行能力。其支持多种任务类型、智能充电管理、完整的状态控制，是机器人自主导航系统的重要组成部分。系统具有良好的扩展性和可维护性，能够满足各种复杂的任务执行需求。

