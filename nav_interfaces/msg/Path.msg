std_msgs/Header header

uint64 task_id

nav_interfaces/PathPose[] path_poses
# 楼层
int8 floor

# 机器人移动到终点时，姿态保持方式
uint8 adjust_mode

# 辅助判断是否到达路径终点的方式
# GoalCheckerAssistance goal_checker_assistance

# 导航方式
uint8 navigation_type   # 0 : 自主导航  1 : 路网导航或路径点跟踪

# 路径终点属性
uint8 goal_type      # 0 : 正常任务点  1 : 等待点
