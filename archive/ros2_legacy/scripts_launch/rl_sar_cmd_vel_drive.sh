#!/usr/bin/env bash
# Smoke test: launch rl_sar Go2W policy in MuJoCo with auto-init, drive it
# from a ROS 2 /cmd_vel topic via the UDP bridge.
#
# Auto-init: rl_sim_mujoco synthesizes the Num0 keypress 2 s after the GUI
# opens, FSM transitions Passive → GetUp → (auto) → RLLocomotion. Then any
# /cmd_vel publish overrides the keyboard increments and drives the robot
# directly.
#
# Usage:
#   ./scripts/launch/rl_sar_cmd_vel_drive.sh                       # default scene
#   ./scripts/launch/rl_sar_cmd_vel_drive.sh scene_terrain         # other scene
#   TOPIC=/robot/cmd_vel ./scripts/launch/rl_sar_cmd_vel_drive.sh  # remap input
#
# Test it:
#   ros2 topic pub /cmd_vel geometry_msgs/Twist \
#     '{linear: {x: 0.5}, angular: {z: 0.0}}'
#
# Stop:
#   Ctrl-C in this terminal kills both the sim and the bridge.

set -euo pipefail

SCENE="${1:-scene}"
RL_SAR_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../src/vendor/rl_sar" && pwd)"
BIN="${RL_SAR_ROOT}/cmake_build/bin/rl_sim_mujoco"
BRIDGE="${RL_SAR_ROOT}/scripts/cmd_vel_to_udp.py"

if [[ ! -x "$BIN" ]]; then
    echo "[ERR] rl_sim_mujoco not built. Run:" >&2
    echo "    cd $RL_SAR_ROOT && ./build.sh -mj" >&2
    exit 1
fi

export LD_LIBRARY_PATH="${RL_SAR_ROOT}/library/inference_runtime/libtorch/lib:${RL_SAR_ROOT}/library/mujoco/lib:${LD_LIBRARY_PATH:-}"
export RL_SAR_AUTO_INIT="${RL_SAR_AUTO_INIT:-1}"
export RL_SAR_AUTO_INIT_DELAY="${RL_SAR_AUTO_INIT_DELAY:-2.0}"
export RL_SAR_UDP_ENABLE="${RL_SAR_UDP_ENABLE:-1}"
export RL_SAR_UDP_PORT="${RL_SAR_UDP_PORT:-9011}"

# ROS 2 env (Humble + cmu_env Python).
if ! command -v ros2 >/dev/null 2>&1; then
    if [[ -f /opt/ros/humble/setup.bash ]]; then
        source /opt/ros/humble/setup.bash
    fi
fi

cleanup() {
    echo "[rl_sar_drive] cleanup"
    [[ -n "${BRIDGE_PID:-}" ]] && kill "$BRIDGE_PID" 2>/dev/null || true
    [[ -n "${SIM_PID:-}" ]]    && kill "$SIM_PID"    2>/dev/null || true
    wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[rl_sar_drive] launching MuJoCo (scene=$SCENE) — auto-init in ${RL_SAR_AUTO_INIT_DELAY}s"
"$BIN" go2w "$SCENE" &
SIM_PID=$!

sleep "${RL_SAR_AUTO_INIT_DELAY}"
echo "[rl_sar_drive] starting cmd_vel→UDP bridge (port=$RL_SAR_UDP_PORT, topic=${TOPIC:-/cmd_vel})"
UDP_PORT="$RL_SAR_UDP_PORT" TOPIC="${TOPIC:-/cmd_vel}" \
    python3 "$BRIDGE" &
BRIDGE_PID=$!

wait "$SIM_PID"
