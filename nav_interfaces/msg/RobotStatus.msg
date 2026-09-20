std_msgs/Header header

string robot_type

float32 x
float32 y
float32 z
float32 roll
float32 pitch
float32 yaw

float32 linear_velocity_x
float32 linear_velocity_y
float32 linear_velocity_z
float32 angular_velocity_x
float32 angular_velocity_y
float32 angular_velocity_z

float32 max_linear_speed  # 最大的线速度
float32 max_angular_speed  # 最大的线速度

string speed_gear      # 速度档位 0：低速，1：中速，2：高速
string control_mode     # "auto"  "manual"
string navigation_mode  # "free_goal"（自由导航）  "topo_map"（路网导航）
string avoid_obstacle_mode   # "enabled" "disabled"

string navigation_status  # 导航状态  idle|running|failed
string recharge_status    # 回桩充电状态 unknown|running|finished|no_pile|failed|leave_pile
bool is_charging          # 是否正在充电，0表示否，1表示是
uint32 es_stop_status     # 0表示没有停止， 1表示硬件急停， 2表示软件急停

# speed_gear
uint32 LOW_SPEED = 0
uint32 MID_SPEED = 1
uint32 HIGH_SPEED = 2

# control_mode
uint32 AUTO = 0
uint32 MANUAL = 1

# navigation_mode
uint32 FREE_GOAL = 0
uint32 TOPO_MAP = 1
