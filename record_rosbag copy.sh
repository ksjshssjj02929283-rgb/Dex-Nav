#!/bin/bash
if [ "$#" -ne 1 ]; then
  echo "record_rosbag Usage: $0 log_path"
  exit 1
fi
# wait for matrix nodes ready and then start record bag
WAIT_RECORD_BAG_TIMEOUT=60
i=0
while true; do
  pgrep toolbox
  if [[ "$?" == 0 || $i -gt ${WAIT_RECORD_BAG_TIMEOUT} ]]; then
    echo "begin record bag after waiting: $i seconds"
    break
  fi
  sleep 2 && i=$((i+2))
  echo "waiting for toolbox launch in record_bag: $i"
done

if [ ! -d "$BRINGUP_PATH" ]; then
  export BRINGUP_PATH=`ros2 pkg prefix matrix_bringup --share`
fi
source /opt/ros/galactic/setup.bash
source /root/.bashrc
get_config_path=${BRINGUP_PATH}/scripts/get_config.sh
source ${get_config_path}
# record ros2bag
ROS_DOMAIN_ID=$(GET_ROS_DOMAIN_ID)
# where to store all the logs
# BAG_LOG_PATH="/home/geekplus/launch_bags/bag.log"
BAG_LOG_PATH=$1
# where to store all the bags
BAG_FOLDER_PATH="/home/geekplus/launch_bags"
mkdir -p ${BAG_FOLDER_PATH}
# maximum number of bags stored in BAG_FOLDER_PATH
MAX_NUM_BAGS=10
# duration for each bag (in seconds)
BAG_DURATION_SECS=300
# maximum bag id (loop to use ID: 1~MAX_BAG_ID; normally MAX_BAG_ID>MAX_NUM_BAGS)
MAX_BAG_ID=10000

# record bags
bag_id=1
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
echo "bag record log path: ${BAG_LOG_PATH}"
while true
do
  echo ">>>>>>>>>> Make Directory launch_bags/ <<<<<<<<<<" >> ${BAG_LOG_PATH}
  # reset bag id (in case of overflow..?)
  if [ $bag_id -gt $MAX_BAG_ID ]; then
    bag_id=1
  fi
  # cleanup launch_bags, check if number of bags exceed
  # num_bags=`ls ${BAG_FOLDER_PATH} | grep bag- | wc -l`
  # if [ $num_bags -ge $MAX_NUM_BAGS ]; then
  #       #echo "number of bags exceed the MAX_NUM_BAGS=${MAX_NUM_BAGS}, remove one..." >> ${BAG_LOG_PATH}
  #       elim_bag_name=`ls ${BAG_FOLDER_PATH} -rt | head -n1`
  #       elim_bag_path="${BAG_FOLDER_PATH}/${elim_bag_name}"
  #       rm -rf ${elim_bag_path} &
  # fi
  # store another bag
  CUR_TIME="$(date +%Y%m%d-%H%M%S)"
  bag_folder_path="${BAG_FOLDER_PATH}/bag-${CUR_TIME}-${bag_id}"
  # record
  # echo ">>>>>>>>>> Start recording ros2bags in ${bag_folder_path} <<<<<<<<<<" >> ${BAG_LOG_PATH}
  echo -e "    00000\n    00000\n    00000    ==========================\n    00000\n    00000      Start recording in\n    00000  \n    00000      ${bag_folder_path}\n    00000\n    00000    ==========================\n    00000\n    00000\n  000000000\n   0000000\n    00000\n     000\n      0" >> ${BAG_LOG_PATH}

  timeout --signal=INT ${BAG_DURATION_SECS} ros2 bag record --output ${bag_folder_path} \
    /tf \
    /tf_static \
    /map \
    /cmd_vel \
    /device/powerboard/power2pc \
    /sensor/odom_without_covariance \
    /sensor/camera/qrcode/down/result \
    /sensor/camera/qrcode/up/result \
    /planning/collied_footprint\
    /planning/obstacle_visualization \
    /planning/plan \
    /routing/plan \
    /local_costmap/published_footprint \
    /localization/lidar/feature_map \
    /localization/lidar/feature_matching_res \
    /perception/camera/freespace_detection/point_cloud0 \
    /perception/camera/freespace_detection/point_cloud1 \
    /perception/lidar/visualization_marker_tar \
    /perception/lidar/visualization_marker_pre \
    /perception/lidar/visulization_marker_text \
    /perception/camera/camera_tag_detection/tag_all \
    /perception/camera/lane/lane \
    /perception/lidar/filtered_back \
    /perception/lidar/filtered_front \
    /perception/lidar/filtered_back_left \
    /perception/lidar/filtered_back_right \
    /perception/lidar/front_reflector

  echo -e "      0\n     000\n    00000\n   0000000\n  000000000\n    00000\n    00000\n    00000    =============================\n    00000\n    00000      Finished recording in\n    00000\n    00000      ${bag_folder_path}\n    00000\n    00000    ============================\n    00000\n    00000" >> ${BAG_LOG_PATH}
  # increment counter
  ((bag_id++))
done

