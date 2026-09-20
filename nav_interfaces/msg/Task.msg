string name
uint32 type
string[] station_names # 途径点
int32 priority         # 任务优先级 0：普通 1：高级 2：紧急
float32 waiting_time      # 完成任务后的等待时间 (s)


uint32 UNKNOWN = 0
uint32 NAVIGATION = 1    # 导航到点
uint32 DOCKING_PILE = 2  # 充电上桩
uint32 EXIT_PILE = 3     # 充电出桩
uint32 AUTO_CHARGING = 4 # 自动充电


uint32 NORMAL = 0
uint32 HIGH = 1
uint32 EMERGENCY = 2