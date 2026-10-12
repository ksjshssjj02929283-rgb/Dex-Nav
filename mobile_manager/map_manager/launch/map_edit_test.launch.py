"""本机试地图编辑：一次起 map_manager + map_asset_manager + command_board（部署名）+ RViz
（预置 Map Manager / Map Asset Manager 面板、底图画笔、资产与禁行区掩膜显示、非当前图预览）。

  ros2 launch map_manager map_edit_test.launch.py              # 带 RViz
  ros2 launch map_manager map_edit_test.launch.py rviz:=false  # 不起 RViz

Ctrl-C 一并停止，不留后台进程（不要用 `ros2 run … &` 反复起：旧进程留在后台会和新进程同时响应同一个服务）。
对齐 / 替换 / 重设原点由 command_board 调 cartographer_ros 的离线工具，启动前要 source 对应 SLAM 树的 install。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(get_package_share_directory("map_manager"), "rviz", "map_edit.rviz")
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true", description="是否同时启动 RViz"),
        Node(package="map_manager", executable="map_manager_node", name="map_manager", output="screen"),
        Node(package="map_asset_manager", executable="map_asset_manager_node", name="map_asset_manager",
             output="screen"),
        Node(package="command_board", executable="command_board_node", name="command_board", output="screen"),
        Node(package="rviz2", executable="rviz2", name="rviz2", arguments=["-d", rviz_config],
             condition=IfCondition(LaunchConfiguration("rviz")), output="log"),
    ])
