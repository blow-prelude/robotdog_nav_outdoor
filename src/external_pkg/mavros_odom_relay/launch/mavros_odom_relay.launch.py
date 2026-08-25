"""Launch the mavros_odom_relay node.

Bridges /mavros/local_position/odom (BEST_EFFORT, from mavros) to /odom
(RELIABLE) so that Nav2 controller_server can subscribe with its default
RELIABLE QoS. Without this relay Nav2 never receives odom and never
publishes /cmd_vel.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = (
        "/home/nhy/ww/outdoor_nav/src/external_pkg/mavros_odom_relay/config/params.yaml"
    )

    args = [
        DeclareLaunchArgument("input_topic", default_value="/mavros/local_position/odom"),
        DeclareLaunchArgument("output_topic", default_value="/odom"),
        DeclareLaunchArgument("frame_id_rewrite", default_value=""),
    ]

    overrides = {
        "input_topic": LaunchConfiguration("input_topic"),
        "output_topic": LaunchConfiguration("output_topic"),
        "frame_id_rewrite": LaunchConfiguration("frame_id_rewrite"),
    }

    node = Node(
        package="mavros_odom_relay",
        executable="odom_relay",
        name="mavros_odom_relay",
        output="screen",
        parameters=[config, overrides],
    )

    return LaunchDescription(args + [node])