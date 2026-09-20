# collision_monitor 障碍检测实时状态：每个处理周期发布一次（非仅状态切换时）。
std_msgs/Header header

uint8 status
uint8 FREE = 0
uint8 SLOWDOWN = 1
uint8 STOP = 2

string zone_name        # 触发该状态的多边形/区域名，FREE 时为空
float32 slowdown_ratio  # SLOWDOWN 时相对当前速度的限速比例(0~1)，其余状态为 0
