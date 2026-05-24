#!/usr/bin/env bash
# Launch demo_ramp scene with Menagerie Go2 (no wheels) instead of Go2W.
# Same autonomy stack as nav_test_3d_explore.launch.py: Fast-LIO + ETH
# elevation_mapping_cupy + grid_map_filters + CFPA2.
#
# Diagnostic purpose: the Go2W gait + wheel skid was suspected of producing
# enough base-link pose jitter to make the CNN traversability layer flicker
# red on the ramp surface during turns / gait switches. Swap in the Go2
# (legged-only, no wheel mux) to see whether the redness persists.
set -u -o pipefail

source "$(dirname "${BASH_SOURCE[0]}")/_preflight_kill.sh"
trap '' SIGHUP SIGTERM

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCENE="${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco/demo_ramp_go2_real.xml"

# trav-pipeline preflight: kill stale elevation_mapping state + CuPy cache
for _pat in elevation_mapping_node elevation_mapping_cupy filter_chain_runner \
            grid_map_to_occupancy_grid; do
  pkill -TERM -f "${_pat}" 2>/dev/null || true
done
sleep 1
rm -rf "${HOME}/.cupy/kernel_cache" 2>/dev/null || true

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

if [[ -n "${CONDA_PREFIX:-}" ]] && [[ -d "${CONDA_PREFIX}/lib/python3.10/site-packages/mujoco" ]]; then
  export LD_LIBRARY_PATH="${CONDA_PREFIX}/lib/python3.10/site-packages/mujoco:${LD_LIBRARY_PATH:-}"
fi
export FASTRTPS_DEFAULT_PROFILES_FILE="${WS_DIR}/config/fastdds_no_shm.xml"
export MUJOCO_INIT_KEYFRAME=home

echo "=== demo_ramp + Menagerie Go2 + 3D autonomy ==="
echo "  scene: $SCENE"
echo "  spawn: (2.0, 0.0, 0.32) — west end, away from ramp"
echo ""

# Default to the pretrained CNN weights (much lower false-lethal rate).
# Set TRAV_WEIGHTS=/path to override; TRAV_WEIGHTS_BASELINE=1 forces ETH.
DEFAULT_PRETRAIN="${WS_DIR}/training_runs/weights_pretrain.dat"
if [[ -z "${TRAV_WEIGHTS:-}" ]] && [[ -z "${TRAV_WEIGHTS_BASELINE:-}" ]] \
   && [[ -f "${DEFAULT_PRETRAIN}" ]]; then
  TRAV_WEIGHTS="${DEFAULT_PRETRAIN}"
fi
TRAV_WEIGHTS_ARG=()
if [[ -n "${TRAV_WEIGHTS:-}" ]]; then
  if [[ ! -f "${TRAV_WEIGHTS}" ]]; then
    echo "ERROR: TRAV_WEIGHTS file not found: ${TRAV_WEIGHTS}" >&2
    exit 1
  fi
  echo "  trav CNN weights: ${TRAV_WEIGHTS}"
  TRAV_WEIGHTS_ARG=("trav_weight_file:=${TRAV_WEIGHTS}")
fi

exec ros2 launch go2_gazebo_sim nav_test_3d_explore.launch.py \
  "mujoco_model_path:=${SCENE}" \
  "spawn_x:=2.0" \
  "spawn_y:=0.0" \
  "spawn_yaw:=0.0" \
  "has_wheels:=false" \
  "${TRAV_WEIGHTS_ARG[@]}" \
  "cfpa2_executable_suffix:=_cpp" \
  "$@"
