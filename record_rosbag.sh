#!/bin/bash

if [ "$#" -ne 1 ]; then
  echo "record_rosbag Usage: $0 log_path"
  exit 1
fi

WAIT_RECORD_BAG_TIMEOUT=60
ROS_DOMAIN_ID=20
BAG_LOG_PATH="$1"
BAG_FOLDER_PATH="/home/dexforce/pzc/nav2_bags"
BAG_DURATION_SECS=300
MAX_BAG_ID=10000

TOPICS=(
  /tf
  /tf_static
  /clock
  /map
  /map_info
  /initialpose
  /goal_pose
  /active
  /behavior_tree_log
  /plan
  /plan_smoothed
  /unsmoothed_plan
  /transformed_global_plan
  /tracked_path
  /trajectories
  /trajectory_node_list
  /speed_limit
  /constraint_list
  /cmd_vel_nav
  /control/cmd_vel
  /control/cmd_vel_nav
  /control/nav2_pose
  /feedback/odom
  /odometry/filtered
  /loc_pose
  /localization_state
  /global_costmap/costmap
  /global_costmap/costmap_updates
  /global_costmap/published_footprint
  /local_costmap/costmap
  /local_costmap/costmap_updates
  /local_costmap/published_footprint
  /global_cloud_map
  /local_cloud_map
  /global_feature_map
  /local_feature_map
  /matched_feature
  /scan_matched_points2
  /laser_scan/front
  /laser_scan/rear
  /orbbec/front/pointcloud2_filtered
  /orbbec/rear/pointcloud2_filtered
)

if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/galactic/setup.bash ]; then
  source /opt/ros/galactic/setup.bash
fi

if [ -f /root/.bashrc ]; then
  source /root/.bashrc
fi

export ROS_DOMAIN_ID
mkdir -p "${BAG_FOLDER_PATH}"
mkdir -p "$(dirname "${BAG_LOG_PATH}")"

echo "bag record log path: ${BAG_LOG_PATH}"
echo "bag folder path: ${BAG_FOLDER_PATH}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"

# Wait for Nav2 related topics to appear before recording.
i=0
while true; do
  if ros2 topic list 2>/dev/null | grep -Eq '^/tf$|^/plan$|^/local_costmap/costmap$'; then
    echo "begin record bag after waiting: ${i} seconds"
    break
  fi

  if [ "${i}" -ge "${WAIT_RECORD_BAG_TIMEOUT}" ]; then
    echo "begin record bag after waiting: ${i} seconds (timeout reached)"
    break
  fi

  sleep 2
  i=$((i + 2))
  echo "waiting for nav topics in record_bag: ${i}"
done

bag_id=1
while true; do
  if [ "${bag_id}" -gt "${MAX_BAG_ID}" ]; then
    bag_id=1
  fi

  cur_time="$(date +%Y%m%d-%H%M%S)"
  bag_folder_path="${BAG_FOLDER_PATH}/bag-${cur_time}-${bag_id}"

  echo ">>>>>>>>>> Make Directory nav2_bags/ <<<<<<<<<<" >> "${BAG_LOG_PATH}"
  echo -e "    00000\n    00000\n    00000    ==========================\n    00000\n    00000      Start recording in\n    00000\n    00000      ${bag_folder_path}\n    00000\n    00000    ==========================\n    00000\n    00000\n  000000000\n   0000000\n    00000\n     000\n      0" >> "${BAG_LOG_PATH}"

  timeout --signal=INT "${BAG_DURATION_SECS}" \
    ros2 bag record --output "${bag_folder_path}" "${TOPICS[@]}"

  echo -e "      0\n     000\n    00000\n   0000000\n  000000000\n    00000\n    00000\n    00000    =============================\n    00000\n    00000      Finished recording in\n    00000\n    00000      ${bag_folder_path}\n    00000\n    00000    ============================\n    00000\n    00000" >> "${BAG_LOG_PATH}"

  bag_id=$((bag_id + 1))
done
