#!/usr/bin/env bash
# Minimal MuJoCo nav test: Cartographer SLAM + RRT* planner + RViz waypoint picking.
#
# Usage:
#   ./scripts/nav_test_mujoco.sh                  # defaults
#   ./scripts/nav_test_mujoco.sh gui:=false        # headless MuJoCo
#
# In RViz, use "2D Goal Pose" tool to click a waypoint on the map.
# The robot will plan and navigate to it via RRT*.
set -euo pipefail

# Kill any stale nav/sim processes from a prior launch (see _preflight_kill.sh).
source "$(dirname "${BASH_SOURCE[0]}")/_preflight_kill.sh"

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ROS2_SETUP_BASH="${ROS2_SETUP_BASH:-/opt/ros/humble/setup.bash}"

safe_source() {
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

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

echo "=== MuJoCo Nav Test ==="
echo "Use '2D Goal Pose' in RViz to send waypoints."
echo ""

exec ros2 launch go2_gazebo_sim nav_test_mujoco.launch.py "$@"
