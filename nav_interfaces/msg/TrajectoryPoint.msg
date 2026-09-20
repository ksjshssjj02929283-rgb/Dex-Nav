# 单个轨迹点 —— 携带来自 RoutePlanSegment 的段语义
# 几何 + 段语义（L5 当前填充；向后兼容字段）
geometry_msgs/PoseStamped  pose
float32                    speed_hint    # 期望速度上限 (m/s)；0 = 系统默认（不限速）
string                     region_type   # 来自 RoutePlanSegment.lane_subtype（corridor/elevator/transition/...）
string                     segment_id    # 来自 RoutePlanSegment.edge_id（可追溯到段）

# —— 对齐 nav2 五层重构（项目 32 / DEC-009）新增的运动学/时间字段 ——
# 由未来 ④SpeedPlanner 填充；L5 当前不填，默认 0（向后兼容）。
# 类型统一（region_type→uint8 / segment_id→uint32）留到 32 落地子项目。
float64                    longitudinal_velocity  # v，沿轨迹切向线速度，可为负（倒车）
float64                    angular_velocity       # w，角速度
float64                    acceleration           # a，纵向加速度
float64                    jerk                   # 加加速度（平滑约束产物，可选）
builtin_interfaces/Duration relative_time         # 相对轨迹起点时间戳（ST 对齐）
float64                    path_kappa             # 该点路径曲率（供跟踪预补偿）
uint8                     decision_label         # 行为标签（0=NORMAL/1=REVERSE/2=STOP/3=DOCK…）
