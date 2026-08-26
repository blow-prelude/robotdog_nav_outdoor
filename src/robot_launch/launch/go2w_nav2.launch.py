"""Go2W Nav2 stack: nav2_input_bridge + controller_server + straight_planner + Livox MID360."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("robot_launch")
    default_params = os.path.join(pkg_share, "config", "go2w_nav2.yaml")

    livox_share = get_package_share_directory("livox_ros_driver2")
    mid360_launch_file = os.path.join(livox_share, "launch", "msg_MID360_launch.py")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Real vehicle: false. Simulation: pass use_sim_time:=true and run /clock.",
    )
    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Nav2 + nav2_input_bridge multi-node YAML (go2w_nav2.yaml).",
    )
    declare_use_mid360 = DeclareLaunchArgument(
        "use_mid360",
        default_value="true",
        description="Launch Livox MID360 driver (msg_MID360_launch from livox_ros_driver2).",
    )
    declare_use_foxglove = DeclareLaunchArgument(
        "use_foxglove",
        default_value="true",
        description="Start foxglove_bridge for Foxglove Studio.",
    )

    params_file = LaunchConfiguration("params_file")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    use_mid360 = LaunchConfiguration("use_mid360")
    use_foxglove = LaunchConfiguration("use_foxglove")

    sim_time_param = {"use_sim_time": use_sim_time}

    mid360_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mid360_launch_file),
        condition=IfCondition(use_mid360),
    )

    # 18°: "0.0", "0.15643", "0.0", "0.98769"
    # base_to_livox_xyz_ypr: [0.18, 0.0, 0.53, 0.0, 0.3142, 0.0]
    static_tf_base_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_lidar_tf",
        arguments=["0.18", "0", "0.53", "0.0", "0.15643", "0.0", "0.98769", "base_link", "livox_frame"],
        parameters=[params_file, sim_time_param],
        output="screen",
    )

    static_tf_map_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_odom_tf",
        arguments=["0", "0", "0", "0", "0", "0", "1", "map", "odom"],
        parameters=[params_file, sim_time_param],
        output="screen",
    )

    input_bridge_node = Node(
        package="nav2_input_bridge",
        executable="input_bridge_node",
        name="nav2_input_bridge",
        output="screen",
        parameters=[params_file, sim_time_param],
    )

    controller_server_node = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[params_file, sim_time_param],
        remappings=[
            ("/cmd_vel", "/cmd_vel_nav"),
        ],
    )

    straight_planner_node = Node(
        package="nav2_straight_planner",
        executable="straight_planner_node",
        name="straight_planner",
        output="screen",
        parameters=[params_file, sim_time_param],
        remappings=[("goal_pose", "/goal_pose")],
    )

    foxglove_bridge_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {
                "port": 8765,
                "address": "0.0.0.0",
                "topic_whitelist": [
                    "^/tf$",
                    "^/tf_static$",
                    "^/odom$",
                    "^/local_costmap/costmap$",
                    "^/received_global_plan$",
                    "^/plan$",
                    "^/lookahead_point$",
                    "^/lookahead_collision_arc$",
                    "^/goal_pose$",
                    #"^/livox/lidar$",
                    "^/cmd_vel$",
                ],
                "service_whitelist": ["(?!)"],
                "client_topic_whitelist": ["(?!)"],
                "param_whitelist": ["(?!)"],
                "capabilities": ["connectionGraph"],
                "tls": False,
            }
        ],
        condition=IfCondition(use_foxglove),
    )

    lc_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[params_file, sim_time_param],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_params_file,
            declare_use_mid360,
            declare_use_foxglove,
            mid360_driver,
            static_tf_base_lidar,
            static_tf_map_odom,
            input_bridge_node,
            controller_server_node,
            straight_planner_node,
            lc_manager_node,
            foxglove_bridge_node,
        ]
    )
