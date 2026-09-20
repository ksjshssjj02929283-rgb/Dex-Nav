std_msgs/Header                       header
string                                plan_id
string[]                              floor_sequence    # 跨楼层序列
nav_interfaces/RoutePlanSegment[]     segments
float32                               total_distance    # 米
float32                               estimated_time    # 秒
int32                                 floor_changes
