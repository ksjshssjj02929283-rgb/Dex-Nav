# 单条语义事件（robot_storytelling L4 产出）
# event_type 常量
string IN_ROOM=IN_ROOM
string IN_SPEED_LIMIT=IN_SPEED_LIMIT
string CLOSE_TO_CHARGER=CLOSE_TO_CHARGER
string CLOSE_TO_ELEVATOR=CLOSE_TO_ELEVATOR
string IN_NARROW_PASSAGE=IN_NARROW_PASSAGE
string CLOSE_TO_DOOR=CLOSE_TO_DOOR
string CURRENT_EDGE=CURRENT_EDGE

string   event_type    # 见上常量
string   source_id     # 触发事件的 Area/Room name
float32  confidence    # 0~1（MVP 恒 1.0）
string[] tags          # 可选附加标签
