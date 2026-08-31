#!/usr/bin/env bash
# Launch the task pipeline: ExecuteTask service → /waypoint_goals → /gp_goal
#
# 用法:
#   bash launch_task_pipeline.sh                  # 使用下方默认 NAMESPACE
#   bash launch_task_pipeline.sh robot_namespace:=robot_b
#
# 同时启动两台机器人时,在两台机器上分别把 NAMESPACE 改成 robot_a / robot_b;
# 也可以用命令行参数覆盖 NAMESPACE。
#
# 该脚本只启动 execute_task_server 和 multi_goal_to_gp_goal 两个组件,
# 启动后会提供 /{ns}/agent/{ns}/execute_task 服务,接收 JSON 任务下发,
# 并把 waypoints 转发到 /{ns}/gp_goal(NavSatFix),由 nav2_input_bridge 消费。
# 调用方需要在 JSON 中把 agent_id 设为该实例的命名空间(如 robot_a),
# 否则会被 execute_task_server 拒绝。
#
# 如需完整启用指控功能，请手动修改启动文件：
#   src/robot_launch/launch/task_pipeline.launch.py
# 取消其中 swarms_backend_launch 和 multi_task_bridge_launch 的注释，
# 并确保这两个组件都正确传入 robot_namespace 和 agent_id参数。

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# === 默认命名空间 — 改这一行即可切换机器人 ===
NAMESPACE="${NAMESPACE:-robot_a}"
# ==============================================

source "${SCRIPT_DIR}/setup_go2w.sh"
source install/setup.bash

# 仅当用户没在命令行显式传 robot_namespace 时才注入默认 NAMESPACE,
# 这样既支持脚本里改一行,也支持命令行临时覆盖。
HAS_NS_ARG=0
for arg in "$@"; do
    case "$arg" in
        robot_namespace:=*) HAS_NS_ARG=1 ;;
    esac
done

if [ "$HAS_NS_ARG" -eq 0 ]; then
    echo "[launch_task_pipeline.sh] using default namespace: ${NAMESPACE}"
    set -- "robot_namespace:=${NAMESPACE}" "$@"
fi

exec ros2 launch robot_launch task_pipeline.launch.py "$@"