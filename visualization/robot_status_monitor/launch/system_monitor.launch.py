#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_status_monitor',
            executable='system_monitor',
            name='system_monitor',
            output='screen',
            parameters=[{
                'check_nodes': [
                    '/IMU_publisher', 
                    '/behavior_server',
                    '/bluesea_node',
                    '/brainco_hands_node',
                    '/brainco_hands_sim_server',
                    '/bt_navigator',
                    '/bt_navigator_navigate_through_poses_rclcpp_node',
                    '/bt_navigator_navigate_to_pose_rclcpp_node',
                    '/cartographer_node',
                    '/controller_server',
                    '/detect_apriltags',
                    '/global_costmap/global_costmap',
                    '/dexhand_joint_converter',
                    '/joy_node',
                    '/lifecycle_manager_localization',
                    '/lifecycle_manager_navigation',
                    '/local_costmap/local_costmap',
                    '/map_server',
                    '/mobile_joy_control',
                    '/nav2_container',
                    '/planner_server',
                    '/robot_client_real',
                    '/robot_state_publisher',
                    '/ros2_node',
                    '/smoother_server'],
                'check_interval': 2.0,
                'enable_status_logging': True  # 启用状态日志
            }]
        )
    ])