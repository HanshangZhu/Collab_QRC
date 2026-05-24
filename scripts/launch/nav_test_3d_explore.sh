#!/usr/bin/env bash
# 3D-frontier exploration sim: demo_ramp scene + Point-LIO / Fast-LIO +
# ETH elevation_mapping_cupy traversability + optional nvblox 3D IG + CFPA2.
#
# Two run modes:
#   ./scripts/launch/nav_test_3d_explore.sh                       # ETH elevation traversability, 2D IG
#   ./scripts/launch/nav_test_3d_explore.sh enable_nvblox_mapper:=true  # optional voxel stream
#
# CFPA2 config is passed via launch overlays. This script must not rewrite
# tracked YAML in src/; repeated launch attempts should leave the worktree clean.
set -euo pipefail
# Shield the script from signals that propagate when preflight kills a stale
# ros2 launch process (which may be the session leader of this terminal).
# SIGHUP: terminal loss / session leader death.
# SIGTERM: process-group broadcast when a group leader is killed.
# Both are restored to default after preflight by exec → ros2 launch.
trap '' SIGHUP SIGTERM

source "$(dirname "${BASH_SOURCE[0]}")/_preflight_kill.sh"

# --- 3D-only preflight: nuke stale elevation_mapping_cupy state ---
# Sim/nav preflight above doesn't know about ETH elevation_mapping_cupy or its
# trav_cost_filters companions. Survivors leave stale publishers on
# /robot/elevation_map_raw, /robot/elevation_map_filtered, /robot/traversability_grid
# (TRANSIENT_LOCAL on the last one → Nav2 StaticLayer latches the old grid).
# Also wipe CuPy NVRTC cache — a previously-failed JIT compile (e.g. the
# carray/float16 incomplete-type bug fixed 2026-05-15) leaves a poisoned
# .cubin under ~/.cupy/kernel_cache that CuPy will happily reuse on next start.
if [[ "${PREFLIGHT_KILL:-1}" != "0" ]]; then
  _3D_PATTERNS=(
    "elevation_mapping_node"
    "elevation_mapping_cupy"
    "filter_chain_runner"
    "grid_map_to_occupancy_grid"
    "ramp_ascent_goal_node"
    "ramp_cmd_vel_assist_node"
  )
  for _pat in "${_3D_PATTERNS[@]}"; do
    pkill -TERM -f "${_pat}" 2>/dev/null || true
  done
  sleep 1
  for _pat in "${_3D_PATTERNS[@]}"; do
    pkill -KILL -f "${_pat}" 2>/dev/null || true
  done
  unset _3D_PATTERNS _pat

  # CuPy JIT cache — force fresh NVRTC compile so any kernel-source edit takes effect.
  rm -rf "${HOME}/.cupy/kernel_cache" 2>/dev/null || true
  echo "[preflight-3d] cleared elevation_mapping processes + CuPy kernel cache."
fi

trap - SIGTERM   # restore SIGTERM so Ctrl+C-based kills work on ros2 launch

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

# CRITICAL: mujoco_ros2_control loads MuJoCo sensor plugins from cmu_env's
# site-packages/mujoco/plugin/, but those plugins are linked against
# libmujoco.so.3.6.0 which cmu_env's conda activation does NOT add to
# LD_LIBRARY_PATH. Without this export, libsensor.so fails to dlopen →
# MuJoCo can't bind <sensor> blocks in the MJCF → MuJoCo GUI fails to come
# up and there's no IMU / LiDAR / pose stream for Fast-LIO. Same gotcha as
# nav_test_gbplanner_demo3.sh; see feedback_libmujoco_ld_path memory.
if [[ -n "${CONDA_PREFIX:-}" ]] && [[ -d "${CONDA_PREFIX}/lib/python3.10/site-packages/mujoco" ]]; then
  export LD_LIBRARY_PATH="${CONDA_PREFIX}/lib/python3.10/site-packages/mujoco:${LD_LIBRARY_PATH:-}"
fi

# CuPy wheels only bundle part of the CUDA user-space stack. On machines with
# an NVIDIA driver but no full CUDA toolkit, NVRTC and CUDA headers come from
# pip packages (nvidia-cuda-nvrtc-cu12, nvidia-cuda-runtime-cu12). Point CuPy at
# those paths so elevation_mapping_cupy can JIT kernels without system sudo.
if [[ -n "${CONDA_PREFIX:-}" ]]; then
  CUDA_PIP_ROOT="${CONDA_PREFIX}/lib/python3.10/site-packages/nvidia"
  if [[ -d "${CUDA_PIP_ROOT}/cuda_runtime" ]]; then
    export CUDA_PATH="${CUDA_PATH:-${CUDA_PIP_ROOT}/cuda_runtime}"
    export LD_LIBRARY_PATH="${CUDA_PIP_ROOT}/cuda_runtime/lib:${LD_LIBRARY_PATH:-}"
  fi
  if [[ -d "${CUDA_PIP_ROOT}/cuda_nvrtc/lib" ]]; then
    export LD_LIBRARY_PATH="${CUDA_PIP_ROOT}/cuda_nvrtc/lib:${LD_LIBRARY_PATH:-}"
  fi
fi

SC_PGO_PREFIX="${HOME}/COMP0225_LRC_stack/install/sc_pgo"
if [[ -d "${SC_PGO_PREFIX}" ]]; then
  export AMENT_PREFIX_PATH="${SC_PGO_PREFIX}:${AMENT_PREFIX_PATH}"
  export LD_LIBRARY_PATH="${SC_PGO_PREFIX}/lib:${LD_LIBRARY_PATH}"
fi

export FASTRTPS_DEFAULT_PROFILES_FILE="${WS_DIR}/config/fastdds_no_shm.xml"

# Mid-360 sim now defaults to Livox-Risley CSV replay (~20k pts/frame at 10 Hz
# from share/mujoco_ros2_control/scan_patterns/mid360.csv) + Gaussian σ=0.02 m
# range noise, matching real sensor density and the per-frame non-repetitive
# pattern that elevation_mapping_cupy's visibility-cleanup + drift-compensation
# rely on. Uniform-grid HZ×VT is dead-code in this path; we no longer override.
# To force back to legacy uniform grid: export MUJOCO_LIDAR_SCAN_PATTERN_CSV=""
# To change density:                     export MUJOCO_LIDAR_RAYS_PER_FRAME=N
# To disable noise:                      export MUJOCO_LIDAR_NOISE_STDDEV_M=0
# No scene-coordinate stuck suppression by default. The demo now relies on the
# traversability grid and cliff-proximity cost rather than a ramp corridor gate.
export STUCK_RAMP_SUPPRESS_ENABLED="${STUCK_RAMP_SUPPRESS_ENABLED:-false}"
export STUCK_RAMP_MIN_FORWARD_GOAL_M="${STUCK_RAMP_MIN_FORWARD_GOAL_M:-0.15}"
export STUCK_WATCHDOG_WINDOW_SEC="${STUCK_WATCHDOG_WINDOW_SEC:-9999.0}"

ENABLE_NVBLOX_REQUESTED=0
NAV_COSTMAP_MODE="3d"
for arg in "$@"; do
  case "${arg}" in
    enable_nvblox_mapper:=true|enable_nvblox_mapper:=True|enable_nvblox_mapper:=1)
      ENABLE_NVBLOX_REQUESTED=1
      ;;
    nav_costmap_mode:=2d)
      NAV_COSTMAP_MODE="2d"
      ;;
    nav_costmap_mode:=3d)
      NAV_COSTMAP_MODE="3d"
      ;;
  esac
done

IG_DIM="${IG_DIM:-2d}"
if [[ "${IG_DIM}" != "2d" ]]; then
  echo "WARN: IG_DIM=${IG_DIM} is no longer applied by rewriting CFPA2 source YAML; current demo_ramp launch uses the 2D traversability-grid overlay." >&2
fi

echo "=== 3D Frontier Exploration Sim ==="
echo "  scene:        src/go2w/go2_gazebo_sim/mujoco/demo_ramp.xml"
echo "  ig_dimension: 2d"
echo "  trav source:  elevation_mapping_cupy + grid_map ETH-style filters"
echo "  nvblox vox:   optional 0.10 m stream when enable_nvblox_mapper:=true"
echo ""

# Default to the C++ CFPA2 binary — the Python cfpa2_single_robot_node is broken
# (ImportError: attempted relative import) and dies at startup, leaving no
# frontier goals (robot only moves on manual goals). _cpp is the production
# binary, same as the ops2 launcher. Override with cfpa2_executable_suffix:= "".
exec ros2 launch go2_gazebo_sim nav_test_3d_explore.launch.py \
  "cfpa2_executable_suffix:=_cpp" \
  "$@"
