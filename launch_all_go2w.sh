#!/usr/bin/env bash
# Launch Go2W full system: go2w_driver + MAVROS + Nav2 + auto gp_origin
#
# 用法:
#   bash launch_all_go2w.sh                  # 使用下方默认 NAMESPACE
#   bash launch_all_go2w.sh robot_namespace:=robot_b fcu_url:=/dev/ttyACM1:230400
#
# 同时启动两台机器人时,在两台机器上分别把 NAMESPACE 改成 robot_a / robot_b;
# 也可以用命令行参数覆盖 NAMESPACE。

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
    echo "[launch_all_go2w.sh] using default namespace: ${NAMESPACE}"
    set -- "robot_namespace:=${NAMESPACE}" "$@"
fi

exec ros2 launch robot_launch outdoor_all_go2w.launch.py "$@"