#!/bin/bash
# graph_nav.py 的环境包装：source 车端 overlay + 对齐 auto_mode.sh 的 DDS 配置。
# 用法与 graph_nav.py 完全一致，例如：
#   ./graph_nav.sh L4
#   ./graph_nav.sh random -d -f
#   ./graph_nav.sh origin
#   ./graph_nav.sh recover        # 直线开回最近的导航点
#   ./graph_nav.sh recover L2
set -e

source /opt/ros/humble/setup.bash
source /home/dexforce/xumingkang/dexe_mobile_nav/install/setup.bash

export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-20}

exec python3 "$(dirname "$(readlink -f "$0")")/graph_nav.py" "$@"
