"""Top-level launch for the task pipeline: ExecuteTask service -> /gp_goal.

Includes sub-launches under a single per-robot namespace so the topic chain is
end-to-end namespaced:

    1. execute_task_server/execute_task.launch.py    - ExecuteTask service -> /waypoint_goals
    2. multi_goal_to_gp_goal/multi_goal_to_gp_goal.launch.py
                                                     - /waypoint_goals -> /gp_goal NavSatFix
    3. swarms_backend (backend_nodes)   [optional]   - MQTT -> /{ns}/swarms_backend/global/multitask
    4. multi_task_bridge                [optional]   - /{ns}/swarms_backend/global/multitask
                                                       -> calls /agent/{ns}/execute_task

The namespaced topics chain together:

    /agent/{ns}/execute_task    (service - ABSOLUTE path, agent_id = namespace)
    /{ns}/waypoint_goals        (MultiGoal)
    /{ns}/gp_goal               (NavSatFix, one waypoint at a time)
    /{ns}/odom                  (input to multi_goal_to_gp_goal)

3/4 are optional (MQTT command chain): comment out the corresponding block when
unused. swarms_backend's ament package name is `backend_nodes`; build it first
with `colcon build --packages-select backend_nodes`. Both packages do NOT accept
an external ROS namespace - they self-namespace via their ID params and must run
at the root namespace.

Usage:
    ros2 launch robot_launch task_pipeline.launch.py robot_namespace:=robot_a

This script must be invoked with the same robot_namespace as launch_all_go2w.sh
so the downstream /{ns}/gp_goal is consumed by the nav2 stack under the same
namespace.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ns = LaunchConfiguration("robot_namespace")

    declare_robot_namespace = DeclareLaunchArgument(
        "robot_namespace",
        default_value="",
        description="Per-robot namespace. Pass the same value used for "
                    "launch_all_go2w.sh (e.g. robot_a) so topics align.",
    )

    # 1. execute_task_server: ExecuteTask service -> MultiGoal on /waypoint_goals
    # The launch file forwards robot_namespace so the service and topic live
    # under /{ns}/agent/{ns}/execute_task and /{ns}/waypoint_goals, and sets
    # the agent_id parameter to the namespace value so the JSON-level
    # agent_id check rejects tasks meant for a different robot.
    execute_task_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("execute_task_server"),
                "launch",
                "execute_task.launch.py",
            )
        ),
        launch_arguments={
            "robot_namespace": ns,
            "agent_id": ns,
        }.items(),
    )

    # 2. multi_goal_to_gp_goal: /waypoint_goals -> /gp_goal NavSatFix
    multi_goal_to_gp_goal_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("multi_goal_to_gp_goal"),
                "launch",
                "multi_goal_to_gp_goal.launch.py",
            )
        ),
        launch_arguments={"robot_namespace": ns}.items(),
    )

    # ===================== optional: MQTT command chain =====================
    # Comment out blocks 3 and 4 when the command chain is not used
    # (execute_task can still be called manually).
    # NOTE: neither package accepts an external ROS namespace; both must run at
    # the ROOT namespace and self-namespace via their ID params. The IDs must
    # match robot_namespace (robot_a), and multi_task_bridge's drone_id must
    # correspond to the numeric drones_id in MQTT messages (default 1 = robot_a).

    # 3. swarms_backend (ament package name: backend_nodes): MQTT ->
    #    /{ns}/swarms_backend/global/multitask
    #    !!! build first: colcon build --packages-select backend_nodes
    #    usv_sn:=robot_a + prefix:="" -> swarms_backend self-namespace = /robot_a
    swarms_backend_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("backend_nodes"),
                "launch",
                "backend_nodes.launch.py",
            )
        ),
        launch_arguments={
            "usv_sn": ns,
            "prefix": "",
        }.items(),
    )

    # 4. multi_task_bridge: /{ns}/swarms_backend/global/multitask ->
    #    calls /agent/{ns}/execute_task
    multi_task_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("multi_task_bridge"),
                "launch",
                "multi_task_bridge.launch.py",
            )
        ),
        launch_arguments={
            "drone_name": ns,
            "drone_id": "1",
        }.items(),
    )

    return LaunchDescription([
        declare_robot_namespace,
        execute_task_launch,
        multi_goal_to_gp_goal_launch,
       # swarms_backend_launch,
       # multi_task_bridge_launch,
    ])
