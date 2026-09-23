#!/usr/bin/env python3
"""
任务管理器 + 导航点管理器 + 地图管理器启动文件

功能:
  - 启动地图管理器 (map_manager_ros2) - 发布 /map 话题，替代 map_server
  - 启动导航点管理器 (nav2_point_manager) - 管理导航点
  - 启动导航适配器 (nav_adapter) - 连接 mission_manager 和 Nav2
  - 启动任务管理器 (mission_manager) - 调度导航任务

使用:
  # 仿真环境使用（Gazebo）
  ros2 launch robot_bringup managers_bringup.launch.py \
    use_sim_time:=true \
    map_folder:=/path/to/sim_maps \
    initial_map:=warehouse

  # 实际机器人使用
  ros2 launch robot_bringup managers_bringup.launch.py \
    use_sim_time:=false \
    map_folder:=/path/to/real_maps \
    initial_map:=factory_floor

  # 加载地图后手动添加导航点
  ros2 service call /map_asset_manager/set_point nav_interfaces/srv/SetNavPoint \
    "{nav_point: {name: 'point_1', x: 1.0, y: 2.0, theta: 0.0}}"

  # 发送导航任务
  ros2 service call /mission_manager_node/add_task nav_interfaces/srv/AddTask \
    "{task_type: 1, nav_point_name: 'point_1'}"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ========================================
    # 声明参数
    # ========================================

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time (true for Gazebo, false for real robot)'
    )

    map_folder_arg = DeclareLaunchArgument(
        'map_folder',
        default_value=
        '/home/dexforce/w1/dexe_mobile_application/mobile_simulation/w1_nav2/maps',
        description='Root folder containing all maps')

    initial_map_arg = DeclareLaunchArgument(
        'initial_map',
        default_value='',
        description='Initial map to load (empty for no auto-load)')

    auto_load_map_arg = DeclareLaunchArgument(
        'auto_load_map',
        default_value='false',
        description='Automatically load initial_map on startup')

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description=
        'Top-level namespace for all nodes (empty for global namespace)')

    # ========================================
    # 获取参数
    # ========================================

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_folder = LaunchConfiguration('map_folder')
    initial_map = LaunchConfiguration('initial_map')
    auto_load_map = LaunchConfiguration('auto_load_map')
    namespace = LaunchConfiguration('namespace')

    # ========================================
    # 节点定义
    # ========================================

    # 1. 地图管理器 (替代 map_server)
    map_manager_node = Node(
        package='map_manager_ros2',
        executable='map_manager_node',
        name='map_manager',
        output='screen',
        parameters=[{
            'map_folder':
                map_folder,
            'use_sim_time':
                use_sim_time,
            # 配置各节点的 set_map_changed 服务名称（可根据命名空间灵活调整）
            'nav_point_manager_service':
                '/nav_point_manager_node/set_map_changed',
            'mission_manager_service':
                '/mission_manager_node/set_map_changed',
            'move_base_service':
                '/move_base/set_map_changed',
            'localization_service':
                '/localization/set_map_changed',
        }],
        remappings=[
            ('/map', '/map'),
            ('/map_metadata', '/map_metadata'),
        ])

    # 2. 导航点管理器
    nav_point_manager_node = Node(package='nav2_point_manager',
                                  executable='nav2_point_manager_node',
                                  name='nav_point_manager_node',
                                  output='screen',
                                  parameters=[{
                                      'map_folder': map_folder,
                                      'map_name': initial_map,
                                      'use_sim_time': use_sim_time,
                                  }])

    # 3. 导航适配器 (mission_manager <-> Nav2)
    nav_adapter_node = Node(package='nav_adapter',
                            executable='nav_adapter_node',
                            name='nav_adapter_node',
                            output='screen',
                            parameters=[{
                                'use_sim_time': use_sim_time,
                            }])

    mission_manager_node = Node(package='mission_manager',
                                executable='mission_manager_node',
                                name='mission_manager_node',
                                output='screen',
                                parameters=[{
                                    'use_sim_time': use_sim_time,
                                }])

    # 4. 任务管理器 - 不在这里启动
    # mission_manager_node 需要单独启动，避免 executor 冲突
    # 使用: ./mobile_manager/mission_manager/scripts/start_mission_manager_standalone.sh
    #
    # 或者使用命令:
    #   cd /home/dex/workspace_pzc/dexe_mobile_nav
    #   source install/setup.bash
    #   ros2 run mission_manager mission_manager_node
    #

    # ========================================
    # 可选：自动加载地图
    # ========================================

    # 延迟执行地图加载（节点启动后）
    # 注意：这需要单独的脚本或 Timer 节点来实现
    # 目前推荐手动调用服务加载地图

    # ========================================
    # 组合节点（支持命名空间）
    # ========================================

    # 使用 GroupAction 支持命名空间
    # 注意：mission_manager_node 不在这里启动，需要单独启动
    # 使用脚本: ./mobile_manager/mission_manager/scripts/start_mission_manager_standalone.sh
    bringup_cmd_group = GroupAction([
        PushRosNamespace(namespace),
        map_manager_node,
        nav_point_manager_node,
        nav_adapter_node,
        # mission_manager_node 单独启动，避免 executor 冲突
    ])

    # ========================================
    # 返回 LaunchDescription
    # ========================================

    return LaunchDescription([
        # 参数声明
        use_sim_time_arg,
        map_folder_arg,
        initial_map_arg,
        auto_load_map_arg,
        namespace_arg,

        # 节点启动
        bringup_cmd_group,
        mission_manager_node,  #需要单独启动，不在 launch 中
    ])
