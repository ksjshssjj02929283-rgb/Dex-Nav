# ── 区域类型常量 ──────────────────────────────────────────────────────
uint32 VIRTUAL_WALL       = 1    # 虚拟墙（逻辑隔离线，不可穿越）
uint32 RESTRICTED_AREA    = 2    # 禁行区（机器人禁止进入）
uint32 SPEED_LIMIT_AREA   = 3    # 减速区（限速行驶）
uint32 DETOUR_AREA        = 4    # 绕障区（遇障绕行）
uint32 STOP_OBSTACLE_AREA = 5    # 停障区（遇障停止）
uint32 PARKING_AREA       = 6    # 停车区（关联停车点）
uint32 CHARGING_AREA      = 7    # 充电区（关联充电点）
uint32 ELEVATOR_AREA      = 8    # 电梯区（进出电梯区域）
uint32 BUMPY_AREA         = 9    # 颠簸区（降速 + 关闭深度相机）

# ── 基本信息 ──────────────────────────────────────────────────────────
string                  name       # 区域名称（全局唯一）
uint32                  type       # 区域类型（见上方常量）
geometry_msgs/Pose2D[]  polygon    # 多边形顶点列表（map 坐标系，逆时针）

# ── 减速区属性 ────────────────────────────────────────────────────────
float32 max_speed          # 最大行驶速度 (m/s)；0 = 使用系统默认值
                           # 仅 SPEED_LIMIT_AREA 有效

# ── 停障区属性 ────────────────────────────────────────────────────────
float32 stop_distance      # 探测到障碍物后的停止距离 (m)；0 = 系统默认
                           # 仅 STOP_OBSTACLE_AREA 有效

# ── 停车区 / 充电区属性 ──────────────────────────────────────────────
string[] manual_linked_point_names  # 手工指定的关联导航点（停车/充电结束后的停靠点；支持多个）
                                  # PARKING_AREA / CHARGING_AREA 有效，例如多个停车位

# ── 充电区专属属性 ────────────────────────────────────────────────────
string charger_type        # 充电方式：WIRELESS（无线）/ CONTACT（接触式）
                           # 仅 CHARGING_AREA 有效

# ── 颠簸区属性 ───────────────────────────────────────────
float32 depth_min_height     # 深度相机最小有效检测高度 (m)；高于此值的物体才触发避障
                             # 用于忽略地面颠簸噪点；默认 0.1m；仅 BUMPY_AREA 有效

# ── 通用扩展（向后兼容） ──────────────────────────────────────────────
string attribute           # JSON 格式扩展属性，留作向后兼容

# ── 自动空间归属（SpatialIndexer 后台计算，业务只读）────────────────────
# 每次 NavPoint / RouteJunction / Area 变化时自动重算（polygon 内点检测）
string[] related_nav_point_names    # polygon 内的所有 NavPoint name
string[] related_junction_ids       # polygon 内的所有 RouteJunction id
