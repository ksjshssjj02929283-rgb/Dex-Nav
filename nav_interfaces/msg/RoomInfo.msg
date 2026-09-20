# 房间（空间归属语义）— DSG ROOMS 层节点的 Phase 1 具体消息
# layer 隐含为 ROOMS=3；其他 DSG 层节点未来用通用 SceneGraphNode.msg

# ── 房间类型常量 ──────────────────────────────────────────────────────
uint32 UNKNOWN       = 0
uint32 MEETING_ROOM  = 1    # 会议室
uint32 PANTRY        = 2    # 茶水间
uint32 CAFE          = 3    # 咖啡厅
uint32 CORRIDOR      = 4    # 走廊
uint32 OFFICE        = 5    # 办公室
uint32 LOBBY         = 6    # 大堂
uint32 RESTROOM      = 7    # 卫生间（机器人不进入）
uint32 STORAGE       = 8    # 储藏室

# ── DSG 节点标识 ─────────────────────────────────────────────────────
string id                   # 全局唯一 "<layer>:<name>:<source>"，由 room_manager 自动生成
string source               # "manual" | "hydra" | "hov_sg" | "clio" | "user_override"
builtin_interfaces/Time created_at
builtin_interfaces/Time updated_at

# ── 基本信息 ─────────────────────────────────────────────────────────
string name                 # 全局唯一名称（人类可读，如 "会议室3"）
uint32 type                 # 房间类型常量
string semantic_label       # 开放词汇标签（VLN/Clio 兼容，如 "meeting room"）
string display_label        # rviz 标签（空 = 用 name）

# ── 空间几何 ─────────────────────────────────────────────────────────
geometry_msgs/Pose      pose             # 3D 位姿（centroid + 朝向；自动从 polygon 推导）
geometry_msgs/Pose2D[]  polygon          # 2D 多边形（map 坐标系，闭合，逆时针）
float64[6]              bounding_box     # AABB: [min_x, min_y, min_z, max_x, max_y, max_z]

# ── 行为约束（可选，替代独立 Area 在房间内的约束作用）─────────────
# 业务诉求：机器人在 Room 内执行限速/禁入等约束时不必额外画同位置 Area。
RoomBehavior behavior         # 可选；默认全 0/false 表示无约束

# ── 关联（手工指定）─────────────────────────────────────────────────
string[] manual_anchor_point_names  # 手工指定的关联 NavPoint（NavigateToSemanticTarget 候选锚点；支持多入口/出口/工位）

# ── 扩展 ────────────────────────────────────────────────────────────
string attribute            # JSON 格式扩展属性（容量 / 占用状态 / 日历 ID / 父 Floor 等）
string external_dsg_node_id # 跨 DSG 系统关联 ID（Hydra/HOV-SG 桥接用，如 "spark_dsg:1003"）

# ── 自动空间归属（SpatialIndexer 后台计算，业务只读）────────────────────
# 每次 NavPoint / RouteJunction / Room 变化时自动重算（polygon 内点检测）
string[] related_nav_point_names    # polygon 内的所有 NavPoint name
string[] related_junction_ids       # polygon 内的所有 RouteJunction id
