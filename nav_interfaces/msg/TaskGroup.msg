string name                   # 任务组的名称
string task_group_id          # ⭐ DEC-018：客户端生成的 UUID（空则服务端自动生成）；用于跟踪 ~/task_progress topic
nav_interfaces/Task[] tasks   # 任务组的任务列表
uint32 max_execute_count      # 任务组执行的次数
float32 interval              # 任务组之间的时间间隔
uint8 ON_FAILURE_CANCEL=0     # 任务失败策略：取消整个任务组，继续下一个
uint8 ON_FAILURE_PAUSE=1      # 任务失败策略：暂停等待确认（可 resume_task 重试或 cancel_task_group 放弃）
uint8 on_failure_policy       # 任务失败策略（默认 0=CANCEL）