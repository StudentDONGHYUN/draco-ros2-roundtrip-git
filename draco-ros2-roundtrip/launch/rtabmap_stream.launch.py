import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        os.path.dirname(__file__), '..', 'configs', 'rtabmap_stream.yaml'
    )

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to the RTAB-Map parameter file.'
    )

    cloud_topic_arg = DeclareLaunchArgument(
        'cloud_topic',
        default_value='/stream_pair/decoded',
        description='PointCloud2 topic published by the streaming decoder.'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true.'
    )

    icp_odometry_node = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[('scan_cloud', LaunchConfiguration('cloud_topic'))]
    )

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[('scan_cloud', LaunchConfiguration('cloud_topic'))],
        arguments=['--delete_db_on_start']
    )

    return LaunchDescription([
        params_arg,
        cloud_topic_arg,
        use_sim_time_arg,
        icp_odometry_node,
        rtabmap_node,
    ])
