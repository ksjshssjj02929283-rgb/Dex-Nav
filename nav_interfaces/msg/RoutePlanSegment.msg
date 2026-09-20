string                          edge_id           # RouteEdge.id
string                          lane_subtype      # walkway / corridor / elevator / stair / ...
float32                         max_speed         # 段速度上限 (m/s)，0=系统默认
string[]                        regulatory_element_ids
string                          floor             # 楼层标识
geometry_msgs/PoseStamped[]     samples           # 该段采样位姿（含朝向）
