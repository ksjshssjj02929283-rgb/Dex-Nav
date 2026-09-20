int32 loc_mode  # 定位模式
int32 status    # 定位状态
float32 score   # 当前定位的置信度


int32 DEFULT_LOC_MODE = 0
int32 LASER_LOC_MODE = 1
int32 RTK_LOC_MODE = 2

int32 LOC_FAILED = -2         # 定位失败
int32 LOC_INIT_FAILED = -1    # 初始化失败
int32 LOC_NOT_INIT = 0        # 未初始化
int32 LOC_INITIALIZING = 1   # 系统初始化中
int32 LOC_INITIALIZED = 2          # 系统初始化成功
int32 LOC_RELOCALIZING = 3    # 重定位中
int32 LOC_SUCCESS = 4           # 定位成功   