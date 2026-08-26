#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('remote_override')

    return LaunchDescription([
        DeclareLaunchArgument('vx_max', default_value='1.5'),
        DeclareLaunchArgument('vyaw_max', default_value='1.0'),
        DeclareLaunchArgument('activation_threshold', default_value='0.05'),

        Node(
            package='remote_override',
            executable='remote_override_node',
            name='remote_override',
            parameters=[{
                'vx_max': LaunchConfiguration('vx_max'),
                'vyaw_max': LaunchConfiguration('vyaw_max'),
                'activation_threshold': LaunchConfiguration('activation_threshold'),
            }],
            output='screen',
        ),
    ])
