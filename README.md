# Dex Mobile Navigation

ROS 2 Humble mobile-navigation core for the Dex robot platform.

## Packages

- `nav_interfaces`: navigation actions, services, and messages
- `map_asset_manager`: map points, routes, rooms, and topology assets
- `nav2_schedule_planner`: route-segment planning and task orchestration
- `path_follower`: trajectory tracking and velocity control
- `nav2_collision_monitor`: sensor-based stop/slowdown safety monitor
- `nav2_costmap_2d`, `nav2_util`: local Nav2 infrastructure forks
- `command_board`: command and system-state orchestration
- `mobile_bag_recorder`: continuous and event-triggered diagnostics recording

## Build

This repository is intended to be built as a ROS 2 Humble colcon workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

The runtime also requires the external localization/SLAM, sensor, and base-control
nodes that provide map/odom TF, odometry, sensor topics, and consume the final `cmd_vel`.

## Road-network sync script

`nav2_schedule_planner/scripts/map_sync/sync_roadnet.py` requires credentials through
environment variables; no credentials are stored in this repository:

```bash
export PC2_PASS='...'
export CLOUD_TOKEN='...'
```

Optional connection overrides are `PC2_HOST`, `PC2_USER`, `ROS_DOMAIN_ID`, and
`RMW_IMPLEMENTATION`. Do not commit these values or generated map-upload output.
