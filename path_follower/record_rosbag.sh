#!/bin/bash

if [ "$#" -ne 1 ]; then
  echo "record_rosbag Usage: $0 log_path"
  exit 1
fi

WAIT_RECORD_BAG_TIMEOUT=60
ROS_DOMAIN_ID=20
BAG_LOG_PATH="$1"
BAG_FOLDER_PATH="/home/dexforce/xumingkang/nav2_bags"
BAG_DURATION_SECS=60
MAX_BAG_ID=100

TOPICS=(
  /tf
  /tf_static
  /clock
  /map
  /map_info
  /cmd_vel_raw
  /cmd_vel_nav
  /control/cmd_vel
  /control/cmd_vel_nav
  /feedback/odom
  /odometry/filtered
  /laser_scan/front
  /laser_scan/rear
  /imu/chassis
  /path_follower_node/debug/reference_path
  /polygon_stop
)

if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/galactic/setup.bash ]; then
  source /opt/ros/galactic/setup.bash
fi

if [ -f "${HOME}/.bashrc" ]; then
  source "${HOME}/.bashrc"
fi

export ROS_DOMAIN_ID
mkdir -p "${BAG_FOLDER_PATH}"
mkdir -p "$(dirname "${BAG_LOG_PATH}")"

echo "bag record log path: ${BAG_LOG_PATH}"
echo "bag folder path: ${BAG_FOLDER_PATH}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"

log() {
  echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*" | tee -a "${BAG_LOG_PATH}"
}

resolve_recordable_topics() {
  RECORDABLE_TOPICS=()

  for topic in "${TOPICS[@]}"; do
    topic_type="$(ros2 topic type "${topic}" 2>/dev/null | head -n 1)"
    if [ -z "${topic_type}" ]; then
      log "skip topic ${topic}: not discovered"
      continue
    fi

    if ! ros2 interface show "${topic_type}" >/dev/null 2>&1; then
      log "skip topic ${topic}: type ${topic_type} is not available locally"
      continue
    fi

    RECORDABLE_TOPICS+=("${topic}")
  done
}

log_recordable_topics() {
  log "recordable topics count: ${#RECORDABLE_TOPICS[@]}"
  printf '[%s] topics:' "$(date '+%Y-%m-%d %H:%M:%S')" >> "${BAG_LOG_PATH}"
  printf ' %s' "${RECORDABLE_TOPICS[@]}" >> "${BAG_LOG_PATH}"
  printf '\n' >> "${BAG_LOG_PATH}"
}

if ! command -v ros2 >/dev/null 2>&1; then
  log "ros2 command not found"
  exit 1
fi

if ! command -v timeout >/dev/null 2>&1; then
  log "timeout command not found"
  exit 1
fi

# Wait for Nav2 related topics to appear before recording.
i=0
while true; do
  if ros2 topic list 2>/dev/null | grep -Eq '^/tf$|^/plan$|^/local_costmap/costmap$'; then
    log "begin record bag after waiting: ${i} seconds"
    break
  fi

  if [ "${i}" -ge "${WAIT_RECORD_BAG_TIMEOUT}" ]; then
    log "begin record bag after waiting: ${i} seconds (timeout reached)"
    break
  fi

  sleep 2
  i=$((i + 2))
  log "waiting for nav topics in record_bag: ${i}"
done

resolve_recordable_topics

if [ "${#RECORDABLE_TOPICS[@]}" -eq 0 ]; then
  log "no recordable topics found after initialization"
  exit 1
fi

log_recordable_topics

bag_id=1
while true; do
  if [ "${bag_id}" -gt "${MAX_BAG_ID}" ]; then
    bag_id=1
  fi

  cur_time="$(date +%Y%m%d-%H%M%S)"
  bag_folder_path="${BAG_FOLDER_PATH}/bag-${cur_time}-${bag_id}"
  log "prepare segment ${bag_id}: ${bag_folder_path}"
  echo ">>>>>>>>>> Make Directory nav2_bags/ <<<<<<<<<<" >> "${BAG_LOG_PATH}"
  echo -e "    00000\n    00000\n    00000    ==========================\n    00000\n    00000      Start recording in\n    00000\n    00000      ${bag_folder_path}\n    00000\n    00000    ==========================\n    00000\n    00000\n  000000000\n   0000000\n    00000\n     000\n      0" >> "${BAG_LOG_PATH}"

  timeout --foreground --signal=INT "${BAG_DURATION_SECS}" \
    ros2 bag record --output "${bag_folder_path}" "${RECORDABLE_TOPICS[@]}" \
    >> "${BAG_LOG_PATH}" 2>&1
  record_exit_code=$?

  log "ros2 bag record exit code: ${record_exit_code}"

  if [ "${record_exit_code}" -ne 0 ] && [ "${record_exit_code}" -ne 124 ] && [ "${record_exit_code}" -ne 130 ]; then
    log "record command exited abnormally, retry after 5 seconds"
    sleep 5
  fi

  echo -e "      0\n     000\n    00000\n   0000000\n  000000000\n    00000\n    00000\n    00000    =============================\n    00000\n    00000      Finished recording in\n    00000\n    00000      ${bag_folder_path}\n    00000\n    00000    ============================\n    00000\n    00000" >> "${BAG_LOG_PATH}"

  bag_id=$((bag_id + 1))
done
