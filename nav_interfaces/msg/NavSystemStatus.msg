# 导航系统聚合状态（command_board 对外发布，~1Hz）
# 各字段来源见注释；后续可扩展导航/任务进度等。
std_msgs/Header header

# ── 电池（解析自 /diagnostics_agg 的 PowerBoard 段）──
uint16  battery_percentage   # 电量百分比 0-100
bool    battery_charging     # 是否充电
float32 battery_voltage      # 电压 V
float32 battery_current      # 电流 A（负=放电）

# ── 安全状态（解析自 /diagnostics_agg 的 PowerBoard Safety 段）──
bool estop_active            # 急停触发
bool bumper_front_active     # 前防撞条触发
bool bumper_rear_active      # 后防撞条触发

# ── 定位状态（来自 /localization_node/state）──
nav_interfaces/LocalizationState localization

# ── 当前加载地图（来自 map_manager/map_name）──
string current_map_name

# ── 任务状态（来自 mission_manager/status）──
string task_status

# ── 位姿 + 速度（来自 /feedback/odom）──
geometry_msgs/Pose  pose       # odom 系位姿
geometry_msgs/Twist velocity   # 机体速度

# ── 急停丢位恢复状态（来自 SafetyReactor）──
string recover_state    # NORMAL / SAFE_STOP / RELOCATING / GLOBAL_RELOC / ALARM
string safety_alarm     # 告警信息（ALARM 时非空，否则空）
