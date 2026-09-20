# 一段已解析的几何路径（纯坐标，无地图引用）。
# 由 map_asset_manager 解析 nav_point_ids 后产出，供 schedule_planner 离散成 1cm 轨迹跟踪。
#
# 双向边的“方向”由解析方处理：control_points 一律按行进方向序给出，
# schedule_planner 不感知正/反向，只按 control_points 顺序离散。

uint8 type
uint8 LINE   = 0   # 直线
uint8 BEZIER = 1   # 贝塞尔曲线（一般 Bernstein，阶数 = control_points.size()-1）

# 控制点序列（按行进方向序）：
#   LINE   : [起点, 终点]（2 个）
#   BEZIER : [P0, P1, ..., Pn]（n+1 个）
geometry_msgs/Point[] control_points

# 跟踪方式（高级属性，默认 0=按 schedule_planner 默认逻辑自动选择；
# map_asset_manager 的 rviz 新建路径工具不暴露该字段）。
uint8 mode
uint8 MODE_DIFF = 1                 # 差速
uint8 MODE_OMNI_TRANSLATION = 2     # 全向平移（依赖 hold_direction，为 NaN 时 schedule_planner 会降级为 MODE_DIFF）
uint8 MODE_OMNI_TRANS_ROTATION = 3  # 全向一边平移一边旋转

# 段末目标朝向 rad（path_follower 只采信末点朝向，中间点朝向不使用）。
float64 target_yaw

# 倒车模式：差速车车头反向、vx<0（对应 path_follower enable_reverse_）。
bool backward

# 段限速 m/s（<=0 表示不限）。
float32 max_speed

# 到点容差（<=0 表示用 path_follower 默认）。
float32 xy_tolerance
float32 th_tolerance

# 平移朝向 rad：全向车以平移方式移动时保持的朝向；NaN=差速方式移动。
float32 hold_direction

# 车体四周膨胀大小 m（停障距离检查）；NaN=系统默认。
float32 padding
