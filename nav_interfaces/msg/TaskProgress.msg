# TaskProgress — mission_manager 任务执行进度推送（DEC-018 2026-05-21）
#
# mission_manager 在 `~/task_progress` topic 上 publish 此消息，
# 客户端订阅后按 task_group_id 过滤即可跟踪"我下发的任务"进度。
#
# 发布时机：
#   QUEUED     — AddTaskGroup/AddTask 成功入队
#   EXECUTING  — 任务开始执行或推进到下一个 task
#   WAITING    — 任务组循环间隔等待中
#   SUCCESS    — 任务组全部完成
#   FAILED     — 任务失败
#   CANCELLED  — 任务被取消（cancel_task / cancel_task_group / clear_all_tasks）

string task_group_id          # 客户端生成的 UUID（空则服务端自动生成；用于多客户端隔离）
string task_group_name        # TaskGroup 名（人类可读）
string current_task_name      # 当前正在执行的 Task 名（空 = 任务组完成/未开始）
uint32 current_task_index     # 当前任务在 TaskGroup 中的索引（0-based）
uint32 total_tasks            # TaskGroup 中总任务数
string current_station        # 当前导航的目标 station_name（第一个 goal 点）
uint32 current_execute_count  # 当前是第几轮循环（1-based；max_execute_count 中）

# 状态枚举
uint32 state
uint32 QUEUED = 0
uint32 EXECUTING = 1
uint32 WAITING = 2
uint32 SUCCESS = 3
uint32 FAILED = 4
uint32 CANCELLED = 5

string message                # 状态描述（如失败原因 / 进度详情）
builtin_interfaces/Time stamp

# ─── 细分任务进度（DEC-018 扩展 2026-05-21）─────────────────────────
# 由 BtDispatcher 从底层 Action 的 feedback 中提取，统一映射到此通用结构
# -1 / 0 = 该字段不可用（如 BtExecutor 任务无 distance 概念）
float32 distance_remaining            # 当前子任务剩余距离（米，仅导航任务有效）
float32 estimated_time_remaining_sec  # 当前子任务预计剩余时间（秒）
int32 number_of_recoveries            # 当前子任务恢复行为执行次数（Nav2 stuck 重试等）
int32 number_of_poses_remaining       # 多点途经导航剩余目标点数（navigate_through_poses 有效）
string sub_current_node               # 当前活跃 BT 节点名（BtExecutor 任务有效）
float32 sub_elapsed_sec               # 子任务已执行秒数
