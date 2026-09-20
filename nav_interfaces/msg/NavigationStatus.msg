int32 status

int32 IDLE = 0
int32 NAVIGATING = 1
int32 COLLISION = 2
int32 FINISHED = 3
int32 FAILED = 4

# 附加过程信息：正常状态下可为空，FAILED 时应携带失败原因，供上层透传给 action 调用方。
string message