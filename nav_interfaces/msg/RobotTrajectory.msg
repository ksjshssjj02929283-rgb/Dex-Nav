# 逐点带段语义的机器人轨迹（L5 nav2_trajectory_generator 产出）
std_msgs/Header                   header
string                            plan_id   # 对应 RoutePlan.plan_id（Path/RoutePlan 配对追溯）
nav_interfaces/TrajectoryPoint[]  points
