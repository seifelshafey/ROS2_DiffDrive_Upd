"""Launch file for the FastSLAM 2.0 C++ node."""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('slam_cpp'),
        'config',
        'slam_params.yaml'
    )

    slam_node = Node(
        package='slam_cpp',
        executable='slam_node',
        name='slam_node',
        output='screen',
        parameters=[config_file],
        # Use simulation time when running with Gazebo
        arguments=['--ros-args', '-p', 'use_sim_time:=true'],
    )

    return LaunchDescription([slam_node])
