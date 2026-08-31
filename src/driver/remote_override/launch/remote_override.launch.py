#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('vx_max', default_value='1.5'),
        DeclareLaunchArgument('vyaw_max', default_value='1.0'),
        DeclareLaunchArgument('activation_threshold', default_value='0.05'),
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='robot',
            description='Namespace for this robot instance.',
        ),
        # External control station publishes mavros_msgs/ManualControl at
        # /{robot_namespace}/control/body_cmd. The default is RELATIVE so the
        # node's own namespace (set in Node.namespace) prefixes it to
        # /{robot_namespace}/control/body_cmd. A leading slash would
        # bypass the node namespace and produce a bare /control/body_cmd
        # that collides across robots on the same DDS domain.
        DeclareLaunchArgument(
            'remote_cmd_topic_suffix',
            default_value='control/body_cmd',
            description='Relative topic suffix; the node namespace prefixes '
                        'it to /{robot_namespace}/control/body_cmd.',
        ),

        Node(
            package='remote_override',
            executable='remote_override_node',
            name='remote_override',
            namespace=LaunchConfiguration('robot_namespace'),
            parameters=[{
                'vx_max': LaunchConfiguration('vx_max'),
                'vyaw_max': LaunchConfiguration('vyaw_max'),
                'activation_threshold': LaunchConfiguration('activation_threshold'),
                # Relative topic name — the node's own namespace (set above)
                # prefixes it to /{robot_namespace}/control/body_cmd. Using
                # an (ns, suffix) tuple here would yield a literal
                # /{ns}/control/body_cmd which gets double-prefixed by the
                # node namespace and ends up at /{ns}/{ns}/control/body_cmd.
                'remote_cmd_topic': LaunchConfiguration('remote_cmd_topic_suffix'),
            }],
            output='screen',
        ),
    ])