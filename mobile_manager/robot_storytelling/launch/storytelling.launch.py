# Copyright 2026 DEXE Robot Engineer
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("areas_topic", default_value="/map_asset_manager/areas"),
        DeclareLaunchArgument("rooms_topic", default_value="/map_asset_manager/rooms"),
        DeclareLaunchArgument("odom_topic", default_value="/feedback/odom"),
        Node(
            package="robot_storytelling",
            executable="storytelling_node",
            name="robot_storytelling",
            output="screen",
            parameters=[{
                "rate_hz": 10.0,
                "charger_radius": 1.0,
                "areas_topic": LaunchConfiguration("areas_topic"),
                "rooms_topic": LaunchConfiguration("rooms_topic"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "stories_topic": "/robot_stories",
            }],
        ),
    ])
