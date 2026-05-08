#!/usr/bin/env bash
# Single-robot Go2 (non-W) MuJoCo sim with the REAL CMU TARE planner.
#
# TARE replaces CFPA2 as the exploration goal source. Its /way_point output
# feeds FAR's global planner (via nav_test_mujoco_fastlio's far_goal_topic
# override). Everything downstream (FAR V-graph, localPlanner primitives,
# pathFollower pure-pursuit, CHAMP locomotion) is unchanged.
#
# Usage:
#   ./scripts/launch/nav_test_go2_tare_real.sh                      # demo3 GUI
#   ./scripts/launch/nav_test_go2_tare_real.sh gui:=false rviz:=false
#   ./scripts/launch/nav_test_go2_tare_real.sh \
#       session_duration_sec:=500 session_output_path:=/tmp/go2_tare.json
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

# SC-PGO from LRC stack.
SC_PGO_PREFIX="${HOME}/COMP0225_LRC_stack/install/sc_pgo"
if [[ -d "${SC_PGO_PREFIX}" ]]; then
  export AMENT_PREFIX_PATH="${SC_PGO_PREFIX}:${AMENT_PREFIX_PATH}"
  export LD_LIBRARY_PATH="${SC_PGO_PREFIX}/lib:${LD_LIBRARY_PATH}"
fi

export FASTRTPS_DEFAULT_PROFILES_FILE="${WS_DIR}/config/fastdds_no_shm.xml"

# --- SIM LiDAR ray count (MuJoCo raycast performance lever) -------------
# Controls mj_multiRay() ray count for the SIMULATED LiDAR only. The real
# MID-360's point rate is firmware-fixed and unaffected.
#   Preset          | hz  × vt  | total sim rays / frame
#   ----------------|-----------|------------------------
#   full (C++ def.) | 1000 × 20 | 20000  (emulates real MID-360 rate)
#   balanced        |  720 × 10 |  7200  ← current
#   lean            |  512 × 8  |  4096
# Unset both to fall back to the C++ defaults (20000).
export MUJOCO_LIDAR_HZ_SAMPLES="${MUJOCO_LIDAR_HZ_SAMPLES:-720}"
export MUJOCO_LIDAR_VT_SAMPLES="${MUJOCO_LIDAR_VT_SAMPLES:-10}"

echo "=== MuJoCo Nav Test (Go2 + real CMU TARE → FAR) ==="
echo "    Sim LiDAR rays: ${MUJOCO_LIDAR_HZ_SAMPLES} x ${MUJOCO_LIDAR_VT_SAMPLES} = $((MUJOCO_LIDAR_HZ_SAMPLES * MUJOCO_LIDAR_VT_SAMPLES)) per frame (sim only — real sensor unchanged)"
exec ros2 launch go2_gazebo_sim nav_test_go2_tare_real.launch.py "$@"
