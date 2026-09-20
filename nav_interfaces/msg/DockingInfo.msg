# 对接所用的信息
uint32 task_type
uint32 model_type               # 对接结构的类型
string model_name     # 对接结构的名称
geometry_msgs/PoseStamped pose # 对接结构的位姿


uint32 UNKOWN = 0

# 任务类型
uint32 DOCKING_PILE = 1001         # 充电上桩
uint32 EXIT_PILE = 1002            # 充电下桩
uint32 PICK_UP = 1003              # 取货

# 对接结构类型
uint32 POINT = 2001           # 到达某个点
uint32 CHARGE_STATION = 2002  # 充电站
uint32 SHELF = 2003           #货架


# TODO: 结构类型还可以继续细分

# uint8 structure_type
# uint8 CHARGING_STATION_1 = 100
# uint8 CHARGING_STATION_2 = 101

# uint8 SHELF_1 = 200
# uint8 SHELF_2 = 201