#!/usr/bin/env bash
# Single-robot A*-only nav launch wrapper (Go2W or Go2 + MuJoCo).
#
# No FAR, no MPPI, no pathFollower. Global A* on the octomap → pure-pursuit
# with curvature-aware speed shaping → hybrid_cmd_router picks wheel-mode on
# straights, legged-mode in tight corners (Go2W only).
#
# Usage:
#   ./scripts/launch/single_astar.sh                                    # Go2W, demo1, headless
#   ./scripts/launch/single_astar.sh robot:=go2 scene:=demo3 gui:=true rviz:=true
#   ./scripts/launch/single_astar.sh explore:=false                     # manual RViz goal
#   ./scripts/launch/single_astar.sh astar_config:=/tmp/my_astar.yaml   # override params
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

exec ros2 launch go2_gazebo_sim single_astar_mujoco.launch.py "$@"
