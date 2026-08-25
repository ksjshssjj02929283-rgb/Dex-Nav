ros2 action send_goal /navigate_through_graph_points nav_interfaces/action/NavigateThroughGraphPoints '{
        nav_point_ids: ["L2", "L4"],
        nav_point_names: ["L2", "L4"],
        pose: {x: 0.0, y: 0.0, theta: 0.0},
        target_yaw: 0.0
}'

ros2 service call /map_asset_manager/resolve_nav_points nav_interfaces/srv/ResolveNavPoints '{
        nav_point_ids: ["L2", "L3", "L4"],
        nav_point_names: ["L2", "L3", "L4"]
}'

ros2 service call /schedule_planner/force_cancel std_srvs/srv/Trigger {}

ros2 action send_goal /steering_angle_control nav_interfaces/action/SteeringAngleControl '{
        target_steer_angles_rad: [0.2, -0.4, 1.0, -1.2],
        timeout_sec: 3.0,
        settle_tolerance_rad: 0.08
}' --feedback

ros2 service call /collision_monitor/set_polygon_padding \
nav_interfaces/srv/SetPolygonPadding "{polygon_name: 'PolygonStop', padding: 0.8}"
ros2 service call /collision_monitor/set_polygon_padding \
nav_interfaces/srv/SetPolygonPadding "{polygon_name: 'PolygonStop', padding: .nan}"

# 段级失败时自动发的录包触发（bag_trigger_topic），手动复现用：
ros2 topic pub -t 3 /mobile_bag_recorder_node/trigger_event std_msgs/msg/String \
"{data: 'schedule_seg0_failed'}"