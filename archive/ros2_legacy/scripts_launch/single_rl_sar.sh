#!/usr/bin/env bash
# Single-robot launch using rl_sar's Go2W RL policy as the locomotion stack
# (replaces CHAMP). Optional SLAM + nav2_hybrid_astar on top.
#
# Stage 1 (default — locomotion only):
#   ./scripts/launch/single_rl_sar.sh
#   ./scripts/launch/single_rl_sar.sh scene:=demo1 gui:=true
#
# Stage 2 (full stack: SLAM + hybrid A* nav):
#   ./scripts/launch/single_rl_sar.sh slam:=true nav:=true rviz:=true
#
# Drive (in another terminal):
#   ros2 topic pub /robot/cmd_vel geometry_msgs/Twist '{linear: {x: 0.4}}' --once
#
# Stop:
#   Ctrl-C in this terminal.
set -u -o pipefail

source "$(dirname "${BASH_SOURCE[0]}")/_preflight_kill.sh"

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ROS2_SETUP_BASH="${ROS2_SETUP_BASH:-/opt/ros/humble/setup.bash}"

safe_source() { set +u; source "$1"; set -u; }

if [[ -f "${HOME}/miniforge3/etc/profile.d/conda.sh" ]]; then
  safe_source "${HOME}/miniforge3/etc/profile.d/conda.sh"
  conda activate cmu_env
elif command -v micromamba >/dev/null 2>&1; then
  eval "$(micromamba shell hook -s bash)"
  micromamba activate cmu_env
fi
safe_source "${ROS2_SETUP_BASH}"
safe_source "${WS_DIR}/install/setup.bash"

export FASTRTPS_DEFAULT_PROFILES_FILE="${WS_DIR}/config/fastdds_no_shm.xml"

exec ros2 launch go2_gazebo_sim single_rl_sar_mujoco.launch.py "$@"
