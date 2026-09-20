#
# Copyright (c) 2020, Geek+ Inc.
# All rights reserved.
#
# File:
#     LaneArray.msg
# Description:
#     Array of lanes with header.
#
# Version:
#     1.0.0
# Author:
#     wangyi
# Date:
#     2020.07.15
#

std_msgs/Header header # Stamp from galaxy nodelet.
nav_interfaces/Line[] lines # Empty means no lane is detected.