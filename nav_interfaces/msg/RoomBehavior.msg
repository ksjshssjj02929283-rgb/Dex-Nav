# Room 的可选行为约束（替代独立 Area 在房间内的约束作用）
#
# 设计目的：让机器人在 Room 内执行约束行为时无需额外画一个同位置 Area。
# Room.polygon 已表达空间归属，behavior 加约束维度即可，避免双 polygon 维护。
#
# 业务侧消费方（Storytelling / Controller）应同时支持：
#   - Room.behavior（本字段）→ 内嵌约束
#   - 独立 Area（含完整 9 类 type）→ 跨房间或非房间约束（如长走廊一段减速带 / 楼梯禁区）

# ── 速度限制 ─────────────────────────────────────────────────────────
float32 max_speed_m_s         # 最大速度 (m/s)；0 = 不限速；> 0 = 业务侧自动降速
                              # 对应 Area SPEED_LIMIT_AREA 的 max_speed

# ── 通行约束 ─────────────────────────────────────────────────────────
bool restricted               # 禁入：机器人不应进入（对应 Area RESTRICTED_AREA）
bool detour_enabled           # 遇障允许绕行（对应 Area DETOUR_AREA）

# ── 障碍策略 ─────────────────────────────────────────────────────────
float32 stop_distance_m       # 遇障停止距离 (m)；0 = 使用系统默认
                              # 对应 Area STOP_OBSTACLE_AREA 的 stop_distance

# ── 颠簸 / 低矮障碍处理 ─────────────────────────────────────────────
float32 depth_min_height_m    # 深度相机最小有效检测高度 (m)；> 0 启用
                              # 对应 Area BUMPY_AREA 的 depth_min_height

# ── 关联的额外 Area（如有"会议室门口减速带"等跨房间补充约束）─────
string[] additional_area_names  # 引用其他独立 Area 资源（业务侧合并消费约束）
