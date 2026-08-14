from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时钟'
    )

    depth_max_range_arg = DeclareLaunchArgument(
        'depth_max_range',
        default_value='5.0',
        description='点云保留的最大深度（米）'
    )

    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='rear',
        description='相机名称，用于构建默认话题名称'
    )
    
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='',
        description='输入的 RGBD 话题名称（为空时使用默认规则：/orbbec/{camera_name}）'
    )

    output_topic_raw_arg = DeclareLaunchArgument(
        'output_topic_raw',
        default_value='',
        description='输出的原始点云话题（为空时使用默认规则：{input_topic}/pointcloud2_raw）'
    )
    
    output_topic_filtered_arg = DeclareLaunchArgument(
        'output_topic_filtered',
        default_value='',
        description='输出的预处理点云话题（为空时使用默认规则：{input_topic}/pointcloud2_filtered）'
    )

    output_topic_laserscan_arg = DeclareLaunchArgument(
        'output_topic_laserscan',
        default_value='',
        description='输出的 LaserScan 话题（为空时使用默认规则：{input_topic}/laser_scan）'
    )

    division_mode_arg = DeclareLaunchArgument(
        'division_mode',
        default_value='true',
        description='是否处理一帧、跳过一帧；关闭时处理全部输入帧'
    )

    # 预处理参数
    flag_voxel_filter_arg = DeclareLaunchArgument(
        'flag_voxel_filter',
        default_value='true',
        description='是否启用降采样滤波'
    )
    
    flag_mls_filter_arg = DeclareLaunchArgument(
        'flag_mls_filter',
        default_value='false',
        description='是否启用最小二乘曲面拟合（当前源码未实际执行）'
    )
    
    flag_sor_filter_arg = DeclareLaunchArgument(
        'flag_sor_filter',
        default_value='false',
        description='是否启用统计滤波（当前源码未实际执行）'
    )
    
    ds_leaf_size_arg = DeclareLaunchArgument(
        'ds_leaf_size',
        default_value='0.05',
        description='降采样体素大小（米）'
    )
    
    mls_polynomial_order_arg = DeclareLaunchArgument(
        'mls_polynomial_order',
        default_value='2',
        description='MLS 多项式阶数'
    )
    
    mls_search_radius_arg = DeclareLaunchArgument(
        'mls_search_radius',
        default_value='0.2',
        description='MLS 搜索半径（米）'
    )
    
    sor_mean_k_arg = DeclareLaunchArgument(
        'sor_mean_k',
        default_value='50',
        description='统计滤波平均邻居数'
    )
    
    sor_stddev_mul_thresh_arg = DeclareLaunchArgument(
        'sor_stddev_mul_thresh',
        default_value='1.0',
        description='统计滤波标准差倍数阈值'
    )

    # 地面过滤参数：内部临时转换到 ground_filter_frame 后，用高度范围判断是否保留点云
    ground_filter_frame_arg = DeclareLaunchArgument(
        'ground_filter_frame',
        default_value='base_link',
        description='地面过滤使用的临时判断坐标系'
    )

    ground_filter_min_height_arg = DeclareLaunchArgument(
        'ground_filter_min_height',
        default_value='0.05',
        description='小于等于该高度的点会被认为是地面或近地噪声并剔除（米）'
    )

    ground_filter_max_height_arg = DeclareLaunchArgument(
        'ground_filter_max_height',
        default_value='1.5',
        description='高于该高度的点不会作为近地障碍输入（米）'
    )

    obstacle_extraction_method_arg = DeclareLaunchArgument(
        'obstacle_extraction_method',
        default_value='height_filter',
        description='障碍物提取方法：height_filter 或 uv_disparity'
    )
    uv_disparity_min_z_arg = DeclareLaunchArgument(
        'uv_disparity_min_z',
        default_value='0.15',
        description='双视差最小深度'
    )
    uv_disparity_max_z_arg = DeclareLaunchArgument(
        'uv_disparity_max_z',
        default_value='4.0',
        description='双视差最大深度'
    )
    uv_disparity_dz_arg = DeclareLaunchArgument(
        'uv_disparity_dz',
        default_value='0.02',
        description='双视差深度分辨率'
    )
    uv_disparity_v_margin_arg = DeclareLaunchArgument(
        'uv_disparity_v_margin',
        default_value='15',
        description='双视差地面V向容差'
    )
    uv_disparity_min_u_count_arg = DeclareLaunchArgument(
        'uv_disparity_min_u_count',
        default_value='5',
        description='双视差U向噪声过滤阈值'
    )
    uv_disparity_ground_margin_h_arg = DeclareLaunchArgument(
        'uv_disparity_ground_margin_h',
        default_value='0.025',
        description='双视差地面自适应物理高度Margin(米)'
    )
    stats_window_sec_arg = DeclareLaunchArgument(
        'stats_window_sec',
        default_value='5.0',
        description='耗时对比阶段性统计时间窗口（秒）'
    )

    # 创建节点
    rgbd_to_pointcloud_node = Node(
        package='dexe_sensor_process',
        executable='rgbd_to_pointcloud',
        name='rgbd_to_pointcloud',
        parameters=[{
            'camera_name': LaunchConfiguration('camera_name'),
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic_raw': LaunchConfiguration('output_topic_raw'),
            'output_topic_filtered': LaunchConfiguration('output_topic_filtered'),
            'output_topic_laserscan': LaunchConfiguration('output_topic_laserscan'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'depth_max_range': LaunchConfiguration('depth_max_range'),
            'division_mode': LaunchConfiguration('division_mode'),
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
            'obstacle_extraction_method': LaunchConfiguration('obstacle_extraction_method'),
            'uv_disparity_min_z': LaunchConfiguration('uv_disparity_min_z'),
            'uv_disparity_max_z': LaunchConfiguration('uv_disparity_max_z'),
            'uv_disparity_dz': LaunchConfiguration('uv_disparity_dz'),
            'uv_disparity_v_margin': LaunchConfiguration('uv_disparity_v_margin'),
            'uv_disparity_min_u_count': LaunchConfiguration('uv_disparity_min_u_count'),
            'uv_disparity_ground_margin_h': LaunchConfiguration('uv_disparity_ground_margin_h'),
            'stats_window_sec': LaunchConfiguration('stats_window_sec'),
        }],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        depth_max_range_arg,
        camera_name_arg,
        input_topic_arg,
        output_topic_raw_arg,
        output_topic_filtered_arg,
        output_topic_laserscan_arg,
        division_mode_arg,
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
        obstacle_extraction_method_arg,
        uv_disparity_min_z_arg,
        uv_disparity_max_z_arg,
        uv_disparity_dz_arg,
        uv_disparity_v_margin_arg,
        uv_disparity_min_u_count_arg,
        uv_disparity_ground_margin_h_arg,
        stats_window_sec_arg,
        rgbd_to_pointcloud_node,
    ])
