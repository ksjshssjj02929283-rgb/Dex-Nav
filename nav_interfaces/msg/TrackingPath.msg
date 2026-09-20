# path_follower 的跟踪路径：离散几何 + 段级跟踪参数。
# 由 schedule_planner 把一段 PathSegment 离散成 1cm 轨迹后填充发布。
#
# 与 nav_msgs/Path 的区别：携带跟踪方式/倒车/横移朝向/限速/膨胀/目标朝向，
# 使段级参数成为轨迹的固有属性——下游无需旁路 topic、也无需参数反查。
#
# NaN 约定（float32）：hold_direction/max_speed/padding/target_yaw 取 NaN 表示
#   "该参数不约束/无效"，path_follower 回退到自身默认行为。
#   target_yaw 的 NaN 另有专义：到点不做最后阶段旋转（见下）。
#
# poses 朝向：每点的**解析切向角**（行进方向），由发布方填入 orientation 的 yaw 分量。
#   path_follower 据此直接求曲率 kappa = dtheta/ds；未填（全为单位四元数）时它会
#   自行从 x,y 反推切向与曲率，但那样对位置噪声的放大系数是 4/ds^2，1cm 点距下约 4e4。
#   倒车段这里仍填行进方向切向，不做 +pi——车头反向由 path_follower 在消费端处理。
#   末点目标朝向另由顶层 target_yaw 表达。

std_msgs/Header header

# 离散轨迹点（默认 1cm 间距）。朝向 = 该点行进方向的解析切向。
geometry_msgs/Pose[] poses

# 跟踪方式。
uint8 mode
uint8 MODE_DIFF = 1                 # 差速：三阶段（旋转、移动、旋转）
uint8 MODE_OMNI_TRANSLATION = 2     # 全向平移：三阶段（旋转、平移、旋转）
uint8 MODE_OMNI_TRANS_ROTATION = 3  # 全向一边平移一边旋转

# 倒车（仅 MODE_DIFF 生效；MODE_OMNI_* 下被忽略，全向平移无正反之分）。
uint8 backward
uint8 FORWARD = 0   # 正向行驶
uint8 REVERSE = 1   # 倒车：车头反向、vx<0

# 全向平移保持的车体朝向 rad（仅 MODE_OMNI_TRANSLATION 生效）。
# NaN = 不指定，path_follower 按默认策略处理（如保持当前朝向）。
float32 hold_direction

# 段限速 m/s。NaN = 用 path_follower 默认上限。
float32 max_speed

# 车体膨胀/停障距离 m。NaN = 系统默认。
float32 padding

# 到点目标朝向 rad：
# - 有效值：到点后旋转到该朝向（diff 阶段三 / omni 末段对齐）。
#   常见取值 = 默认目标站点朝向，由发布方写入；也可指定任意朝向。
# - NaN：到点即停，不进行最后阶段的旋转。
float32 target_yaw
