import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        os.path.dirname(__file__), '..', 'configs', 'hdl_graph_slam_stream.yaml'
    )
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to HDL Graph SLAM parameter file'
    )

    slam_node = Node(
        package='hdl_graph_slam',
        executable='hdl_graph_slam_node',
        name='hdl_graph_slam',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    return LaunchDescription([
        params_arg,
        slam_node,
    ])
