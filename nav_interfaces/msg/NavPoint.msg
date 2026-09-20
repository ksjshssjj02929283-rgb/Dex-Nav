std_msgs/Header header

int32 id
string name
string type  # 点位类型: CHARGING, STATION, DOORWAIT, PARKING, FREE_GOAL, LIFT

float32 x
float32 y
float32 theta
float32 tolerance  # 导航到达精度（xy方向），0表示使用默认值
