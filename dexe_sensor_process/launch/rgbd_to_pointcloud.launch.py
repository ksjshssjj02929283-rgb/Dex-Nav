from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明启动参数
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

    # 预处理参数
    flag_voxel_filter_arg = DeclareLaunchArgument(
        'flag_voxel_filter',
        default_value='true',
        description='是否启用降采样滤波'
    )
    
    flag_mls_filter_arg = DeclareLaunchArgument(
        'flag_mls_filter',
        default_value='true',
        description='是否启用最小二乘曲面拟合'
    )
    
    flag_sor_filter_arg = DeclareLaunchArgument(
        'flag_sor_filter',
        default_value='true',
        description='是否启用统计滤波'
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
            'flag_voxel_filter': LaunchConfiguration('flag_voxel_filter'),
            'flag_mls_filter': LaunchConfiguration('flag_mls_filter'),
            'flag_sor_filter': LaunchConfiguration('flag_sor_filter'),
            'ds_leaf_size': LaunchConfiguration('ds_leaf_size'),
            'mls_polynomial_order': LaunchConfiguration('mls_polynomial_order'),
            'mls_search_radius': LaunchConfiguration('mls_search_radius'),
            'sor_mean_k': LaunchConfiguration('sor_mean_k'),
            'sor_stddev_mul_thresh': LaunchConfiguration('sor_stddev_mul_thresh'),
        }],
        output='screen'
    )

    return LaunchDescription([
        camera_name_arg,
        input_topic_arg,
        output_topic_raw_arg,
        output_topic_filtered_arg,
        flag_voxel_filter_arg,
        flag_mls_filter_arg,
        flag_sor_filter_arg,
        ds_leaf_size_arg,
        mls_polynomial_order_arg,
        mls_search_radius_arg,
        sor_mean_k_arg,
        sor_stddev_mul_thresh_arg,
        rgbd_to_pointcloud_node,
    ])

