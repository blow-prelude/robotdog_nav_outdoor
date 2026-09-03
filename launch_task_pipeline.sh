#!/usr/bin/env bash
# Launch the task pipeline: ExecuteTask service → /waypoint_goals → /gp_goal
#
# ============================================================
# 一、如何给值才能统一后面的话题名（必读）
# ============================================================
# 完整指控链路（4 个包全起）：
#
#   MQTT 报文 → ① swarms_backend → ② multi_task_bridge → ③ execute_task_server
#             → ④ multi_goal_to_gp_goal → /robot_a/gp_goal → 导航栈
#
# 话题名能否串起来，取决于下面三个 ID 指向同一台机器人：
#   robot_namespace : 导航栈命名空间 → /{ns}/gp_goal、/{ns}/waypoint_goals 等
#   usv_sn          : swarms_backend 自命名空间（prefix 必须为空）
#                     → 话题 /{usv_sn}/swarms_backend/global/multitask
#   drone_name      : multi_task_bridge 调用的任务服务
#                     → /agent/{drone_name}/execute_task
#
# 三者默认都取 NAMESPACE（robot_a），所以默认就串得起来；换机器人时
# 保证三者一致即可。简单例子（robot_b，同时启用指控链路）：
#
#   bash launch_task_pipeline.sh robot_namespace:=robot_b usv_sn:=robot_b \
#        drone_name:=robot_b mqtt:=true
#
# 也可以只改脚本里 NAMESPACE 一行，其余参数自动跟随。
#
# ============================================================
# 二、用法
# ============================================================
#   bash launch_task_pipeline.sh
#       仅启动 execute_task_server + multi_goal_to_gp_goal（指控链路关，默认）
#   bash launch_task_pipeline.sh mqtt:=true
#       启动全部 4 个包（mqtt 单开关，同时启用 swarms_backend 与 multi_task_bridge；
#       MQTT broker 默认 127.0.0.1:1883，可改下方 ADDRESS）
#   bash launch_task_pipeline.sh robot_namespace:=robot_b ...
#       命令行传参永远优先于脚本内默认值
#
# 注意 swarms_backend 的 ament 包名是 backend_nodes，不是 swarms_backend。
# ============================================================

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# === 默认参数 — 改这些行即可切换机器人 / MQTT 对接配置 ===
NAMESPACE="${NAMESPACE:-robot_a}"        # 导航栈命名空间，所有 /{ns}/... 话题的 ns
USV_SN="${USV_SN:-${NAMESPACE}}"         # swarms_backend 自命名空间，须与 NAMESPACE 一致
PREFIX="${PREFIX:-}"                     # swarms_backend 话题前缀，空 = /{usv_sn}/...
VEHICLE_ID="${VEHICLE_ID:-1}"            # 传统模式 ID，须对应 MQTT 报文里的 drones_id
ADDRESS="${ADDRESS:-tcp://127.0.0.1:1883}"      # MQTT broker 地址
MQTT_USERNAME="${MQTT_USERNAME:-admin}"
MQTT_PASSWD="${MQTT_PASSWD:-nanhu@2020}"
DRONE_NAME="${DRONE_NAME:-${NAMESPACE}}" # multi_task_bridge 机器人名，须与 NAMESPACE 一致
DRONE_ID="${DRONE_ID:-1}"                # 须对应 MQTT 报文里的 drones_id
MQTT="${MQTT:-false}"                    # true 时同时启用 swarms_backend 与 multi_task_bridge
# ==============================================

source "${SCRIPT_DIR}/setup_go2w.sh"
source install/setup.bash

# 收集用户命令行显式传过的参数名（key:=value 里的 key）
declare -A GIVEN
for arg in "$@"; do
    case "$arg" in
        *:=*) GIVEN["${arg%%:=*}"]=1 ;;
    esac
done

# 默认参数清单：用户没显式传才注入，命令行传的永远优先
DEFAULTS=(
    "robot_namespace:=${NAMESPACE}"
    "usv_sn:=${USV_SN}"
    "prefix:=${PREFIX}"
    "vehicle_id:=${VEHICLE_ID}"
    "address:=${ADDRESS}"
    "mqtt_username:=${MQTT_USERNAME}"
    "mqtt_passwd:=${MQTT_PASSWD}"
    "drone_name:=${DRONE_NAME}"
    "drone_id:=${DRONE_ID}"
    "mqtt:=${MQTT}"
)

INJECT=()
for kv in "${DEFAULTS[@]}"; do
    key="${kv%%:=*}"
    if [ -z "${GIVEN[$key]+x}" ]; then
        INJECT+=("$kv")
    fi
done

echo "[launch_task_pipeline.sh] injecting defaults: ${INJECT[*]}"
echo "[launch_task_pipeline.sh] user args: $*"
exec ros2 launch robot_launch task_pipeline.launch.py "${INJECT[@]}" "$@"
