#
# Copyright (c) 2020, Geek+ Inc.
# All rights reserved.
#
# File:
#     Lane.msg
# Description:
#     CMakeLists of ros/ros_1/lane_detect in lane_detect module.
#
# Version:
#     1.0.0
# Author:
#     wangyi
# Date:
#     2020.07.15
#

float32[] confidence # confidence[0]: overall confidence
float32 dist
float32 theta
geometry_msgs/Point start
geometry_msgs/Point end
uint8 pos # 0: left
          # 1: right
