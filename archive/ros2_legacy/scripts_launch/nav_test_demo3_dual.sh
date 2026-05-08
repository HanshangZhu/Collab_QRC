#!/usr/bin/env bash
# Launch dual-robot nav test on demo3_dual (two Go2Ws, 24×16m scene).
# Fast-LIO2 + FAR per robot, CFPA2 coordinator partitions frontiers,
# inter-robot collision monitor reports any A↔B contacts.
#
# Usage:
#   ./scripts/nav_test_demo3_dual.sh                         # GUI off
#   ./scripts/nav_test_demo3_dual.sh gui:=true               # with MuJoCo GUI
#   ./scripts/nav_test_demo3_dual.sh explore:=false          # manual goals only
#
# Collision report written to /tmp/dual_robot_collision_report.json on exit.
set -u -o pipefail

# Kill any stale nav/sim processes from a prior launch (see _preflight_kill.sh).
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
# Add sc_pgo from LRC stack (needed by Fast-LIO slam_odom_relay downstream).
SC_PGO_PREFIX="${HOME}/COMP0225_LRC_stack/install/sc_pgo"
if [[ -d "${SC_PGO_PREFIX}/share/sc_pgo" ]]; then
  export AMENT_PREFIX_PATH="${SC_PGO_PREFIX}:${AMENT_PREFIX_PATH:-}"
  export CMAKE_PREFIX_PATH="${SC_PGO_PREFIX}:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="${SC_PGO_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
  export PATH="${SC_PGO_PREFIX}/bin:${PATH}"
fi
export FASTRTPS_DEFAULT_PROFILES_FILE="${WS_DIR}/config/fastdds_no_shm.xml"

exec ros2 launch go2_gazebo_sim nav_test_mujoco_fastlio_dual.launch.py "$@"
