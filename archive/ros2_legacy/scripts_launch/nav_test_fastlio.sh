#!/usr/bin/env bash
# Nav test with Fast-LIO2 SLAM (replaces Cartographer).
# Usage:
#   ./scripts/nav_test_fastlio.sh                          # default scene
#   ./scripts/nav_test_fastlio.sh gui:=false                # headless
#   ./scripts/nav_test_fastlio.sh enable_wall_checker:=true  # test mode
set -euo pipefail

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
# Our workspace
safe_source "${WS_DIR}/install/setup.bash"

# SC-PGO from the LRC stack — add ONLY sc_pgo to the path,
# NOT the entire LRC install (which shadows our go2_gazebo_sim).
SC_PGO_PREFIX="${HOME}/COMP0225_LRC_stack/install/sc_pgo"
if [[ -d "${SC_PGO_PREFIX}" ]]; then
  export AMENT_PREFIX_PATH="${SC_PGO_PREFIX}:${AMENT_PREFIX_PATH}"
  export LD_LIBRARY_PATH="${SC_PGO_PREFIX}/lib:${LD_LIBRARY_PATH}"
fi

export FASTRTPS_DEFAULT_PROFILES_FILE="${WS_DIR}/config/fastdds_no_shm.xml"

echo "=== MuJoCo Nav Test (Fast-LIO2) ==="
echo ""

exec ros2 launch go2_gazebo_sim nav_test_mujoco_fastlio.launch.py "$@"
