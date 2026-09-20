# 建图状态（结构化，替代裸 std_msgs/Int16 mapping_status）
int8 status        # 建图状态，取值见下方常量
string map_name    # 当前地图名（未设置为空）

int8 ERROR = -1    # 建图异常
int8 IDLE = 0      # 空闲（未建图）
int8 MAPPING = 1   # 建图中
int8 SAVING = 2    # 正在保存/收尾建图（finishing+saving）
