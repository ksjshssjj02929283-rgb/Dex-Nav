std_msgs/Header header
geometry_msgs/Pose pose

uint8 curve_control_point_type  # 曲线的控制点类型

uint8 curve_type                # 曲线类型

uint8 move_mode                 # 机器人的移动方式

uint8 detour_mode               # 0: 自由绕障  1: 高精度跟踪

uint8 localization_mode         # 定位模式

uint8 carpet_area               # 0: 非地毯区  1: 地毯区

uint8 elevator_area             # 0: 非电梯区  1: 电梯区

float64 speed_limit             # 最大限速

float64 slope                   # 坡度

float64 curvature               # 曲率

# 最小限速
float64 min_speed_limit

# 左右避障距离
float64 avoid_obstacle_width
# 前后避障距离
float64 avoid_obstacle_length
# 左右距离障碍物多远进行减速
float64 decelerate_area_width


uint8 MIN_SPEED_VALID = 1   # 最小限速参数有效
uint8 MAX_SPEED_VALID  = 2   # 最大限速参数有效
uint8 AVOID_OBSTACLE_WIDTH_VALID  = 3   # 左右避障距离参数有效
uint8 AVOID_OBSTACLE_LENGTH_VALID  = 4   # 前后避障距离参数有效
uint8 DECELERATE_AREA_WIDTH_VALID  = 5   # 左右距离障碍物多远进行减速 参数有效

uint8[] valid_limit_list



