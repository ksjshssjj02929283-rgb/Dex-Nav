from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='是否使用仿真时钟'
    )
    depth_max_range_arg = DeclareLaunchArgument(
        'depth_max_range', default_value='5.0', description='点云保留的最大深度（米）'
    )
    # 公共参数声明（保持不变）
    flag_voxel_filter_arg = DeclareLaunchArgument(
        'flag_voxel_filter', default_value='true', description='是否启用降采样滤波（体素滤波）'
    )
    flag_mls_filter_arg = DeclareLaunchArgument(
        'flag_mls_filter', default_value='false', description='是否启用最小二乘曲面拟合'
    )
    flag_sor_filter_arg = DeclareLaunchArgument(
        'flag_sor_filter', default_value='false', description='是否启用统计滤波'
    )
    ds_leaf_size_arg = DeclareLaunchArgument(
        'ds_leaf_size', default_value='0.05', description='降采样体素大小（米）'
    )
    mls_polynomial_order_arg = DeclareLaunchArgument(
        'mls_polynomial_order', default_value='2', description='MLS 多项式阶数'
    )
    mls_search_radius_arg = DeclareLaunchArgument(
        'mls_search_radius', default_value='0.2', description='MLS 搜索半径（米）'
    )
    sor_mean_k_arg = DeclareLaunchArgument(
        'sor_mean_k', default_value='50', description='统计滤波平均邻居数'
    )
    sor_stddev_mul_thresh_arg = DeclareLaunchArgument(
        'sor_stddev_mul_thresh', default_value='1.0', description='统计滤波标准差倍数阈值'
    )
    ground_filter_frame_arg = DeclareLaunchArgument(
        'ground_filter_frame', default_value='base_link', description='地面过滤使用的坐标系'
    )
    ground_filter_min_height_arg = DeclareLaunchArgument(
        'ground_filter_min_height', default_value='0.05', description='base_link下小于等于该高度的点认为是地面并剔除；调高会过滤更多低矮点，单位米'
    )
    ground_filter_max_height_arg = DeclareLaunchArgument(
        'ground_filter_max_height', default_value='1.5', description='base_link下高于该高度的点不作为近地障碍输入；调低会减少高处点进入costmap，单位米'
    )

    # 后置相机节点
    rear_rgbd_node = Node(
        package='dexe_sensor_process',
        executable='rgbd_to_pointcloud',
        name='rear_rgbd_to_pointcloud',  # 唯一节点名
        parameters=[{
            # 后置相机话题配置
            'camera_name': 'rear',
            'input_topic': '/orbbec/rear',
            'output_topic_raw': '/orbbec/rear/pointcloud2_raw',
            'output_topic_filtered': '/orbbec/rear/pointcloud2_filtered',
            'output_topic_laserscan': '/orbbec/rear/laser_scan',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'depth_max_range': LaunchConfiguration('depth_max_range'),
            
            # 公共参数（与前置相同）
            'flag_voxel_filter': LaunchConfiguration('flag_voxel_filter'),
            'flag_mls_filter': LaunchConfiguration('flag_mls_filter'),
            'flag_sor_filter': LaunchConfiguration('flag_sor_filter'),
            'ds_leaf_size': LaunchConfiguration('ds_leaf_size'),
            'mls_polynomial_order': LaunchConfiguration('mls_polynomial_order'),
            'mls_search_radius': LaunchConfiguration('mls_search_radius'),
            'sor_mean_k': LaunchConfiguration('sor_mean_k'),
            'sor_stddev_mul_thresh': LaunchConfiguration('sor_stddev_mul_thresh'),
            'ground_filter_frame': LaunchConfiguration('ground_filter_frame'),
            'ground_filter_min_height': LaunchConfiguration('ground_filter_min_height'),
            'ground_filter_max_height': LaunchConfiguration('ground_filter_max_height'),
            
        }],
        output='screen',
        # 可选：添加命名空间
        # namespace='rear'
    )

    return LaunchDescription([
        use_sim_time_arg,
        depth_max_range_arg,
        # 公共参数
        flag_voxel_filter_arg,
        flag_mls_filter_arg,
        flag_sor_filter_arg,
        ds_leaf_size_arg,
        mls_polynomial_order_arg,
        mls_search_radius_arg,
        sor_mean_k_arg,
        sor_stddev_mul_thresh_arg,
        ground_filter_frame_arg,
        ground_filter_min_height_arg,
        ground_filter_max_height_arg,
        # 单个节点
        rear_rgbd_node,
    ])
