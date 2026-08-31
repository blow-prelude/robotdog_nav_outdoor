from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    declare_robot_namespace = DeclareLaunchArgument(
        "robot_namespace",
        default_value="robot",
        description="Namespace prefix for this robot instance. Topics and frames "
                    "are derived from this (e.g. /robot_a/cmd_vel, robot_a/base_link).",
    )

    ns = LaunchConfiguration("robot_namespace")

    return LaunchDescription(
        [
            declare_robot_namespace,
            Node(
                package="go2w_driver",
                executable="go2w_driver_node",
                name="go2w_driver",
                namespace=ns,
                output="screen",
                parameters=[
                    {
                        # === Motion limits ===
                        "vx_max": 1.5,
                        "vy_max": 0.6,
                        "vyaw_max": 1.0,
                        "cmd_timeout_sec": 0.5,
                        "startup_delay_sec": 2.0,
                        # === Topics ===
                        # RELATIVE names — the node's own namespace prefixes
                        # them to /{robot_namespace}/cmd_vel etc. Do NOT use
                        # absolute paths or (ns, "/x") tuples here; either
                        # gets double-prefixed and ends up at
                        # /robot_a/robot_a/x.
                        # /lf/sportmodestate is published by the Go2 firmware
                        # via DDS with a fixed topic name (no namespace).
                        "cmd_vel_topic": "cmd_vel",
                        "state_topic": "/lf/sportmodestate",
                        "imu_topic": "go2w/imu",
                        "battery_topic": "go2w/battery",
                        # === TF frame ===
                        # imu_frame_id is a TF frame_id; it is NOT subject to
                        # ROS namespace prefixing so we build the absolute
                        # /{robot_namespace}/go2w_imu_link here.
                        "imu_frame_id": [TextSubstitution(text='/'), ns, TextSubstitution(text='/go2w_imu_link')],
                    }
                ],
            ),
        ]
    )