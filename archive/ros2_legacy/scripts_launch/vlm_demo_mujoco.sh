#!/usr/bin/env bash
# VLM-in-the-loop single-robot exploration demo (MuJoCo backend).
#
# Usage:
#   ./vlm_demo_mujoco.sh                                  # defaults (xAI/Grok, FAR nav, carto_2d)
#   ./vlm_demo_mujoco.sh nav_execution_backend:=astar     # switch to A* planner
#   ./vlm_demo_mujoco.sh nav_execution_backend:=default   # switch to default_nav
#
# Nav backends since reactive_nav / mppi_nav were removed (2026-04-24):
#   far     — CMU autonomy stack (default for VLM demo)
#   astar   — C++ A* + oriented footprint check (recommended for tight scenes)
#   default — Python A* + D* Lite + recovery (legacy stable)
#
# VLM history logs are saved to ~/.ros/log/vlm_history/<run_timestamp>/
# A live debug viewer starts automatically at http://localhost:8501
# It waits for the coordinator to create the log dir, then auto-refreshes every 2s.
set -euo pipefail

# Kill any stale nav/sim processes from a prior launch (see _preflight_kill.sh).
source "$(dirname "${BASH_SOURCE[0]}")/_preflight_kill.sh"

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ROS2_SETUP_BASH="${ROS2_SETUP_BASH:-/opt/ros/humble/setup.bash}"
XAI_ENV_FILE="${XAI_ENV_FILE:-${WS_DIR}/src/exploration/cfpa2-rh-physics-exploration/.env.xai}"

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
safe_source "${WS_DIR}/scripts/common_logging.sh"

if [[ -f "${XAI_ENV_FILE}" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "${XAI_ENV_FILE}"
  set +a
fi

setup_run_logging "vlm_demo"

LAUNCH_JOB_PID=""
LAUNCH_JOB_PGID=""
SELF_PGID="$(ps -o pgid= $$ | tr -d ' ')"

terminate_matching_processes() {
  local pattern="$1"
  local signal="$2"
  local pid
  for pid in $(pgrep -f "$pattern" 2>/dev/null || true); do
    [[ "$pid" == "$$" ]] && continue
    [[ "$pid" == "$PPID" ]] && continue
    kill "-${signal}" "$pid" 2>/dev/null || true
  done
}

any_matching_processes() {
  local pattern="$1"
  local pid
  for pid in $(pgrep -f "$pattern" 2>/dev/null || true); do
    [[ "$pid" == "$$" ]] && continue
    [[ "$pid" == "$PPID" ]] && continue
    return 0
  done
  return 1
}

cleanup_stale_processes() {
  local signal="$1"
  local -a patterns=(
    # Launch processes
    'ros2 launch vlm_explorer single_vlm_mujoco_far.launch.py'
    'ros2 launch go2_gazebo_sim single_go2w_mujoco_cfpa2.launch.py'
    # Simulator
    'mujoco_ros2_control'
    'mujoco_sensor_bridge'
    # VLM layer
    'vlm_history_viewer.py'
    '/vlm_explorer/lib/vlm_explorer/'
    # SLAM
    '/opt/ros/humble/lib/cartographer_ros/cartographer_node'
    '/opt/ros/humble/lib/cartographer_ros/cartographer_occupancy_grid_node'
    '/install/go2w_perception/lib/go2w_perception/probability_grid_binarizer.py'
    '/install/go2w_perception/lib/go2w_perception/carto_odom_bridge.py'
    '/install/go2w_perception/lib/go2w_perception/pointcloud_frame_bridge.py'
    # Perception
    '/install/go2w_perception/lib/go2w_perception/qos_bridge.py'
    '/install/go2w_perception/lib/go2w_perception/twist_bridge.py'
    '/install/go2w_perception/lib/go2w_perception/pointcloud_adapter.py'
    '/pointcloud_to_laserscan_node'
    'octomap_server_node'
    # Navigation
    '/install/far_planner/lib/far_planner/far_planner'
    '/install/local_planner/lib/local_planner/localPlanner'
    '/install/local_planner/lib/local_planner/pathFollower'
    '/install/terrain_analysis/lib/terrain_analysis/terrainAnalysis'
    '/install/terrain_analysis_ext/lib/terrain_analysis_ext/terrainAnalysisExt'
    '/install/sensor_scan_generation/lib/sensor_scan_generation/sensorScanGeneration'
    '/install/go2w_nav/lib/go2w_nav/reactive_nav_node'
    '/install/go2w_nav/lib/go2w_nav/default_nav.py'
    '/install/cfpa2_collaborative_autonomy/lib/cfpa2_collaborative_autonomy/cfpa2_single_robot_node'
    '/go2_gazebo_sim/lib/go2_gazebo_sim/waypoint_mux.py'
    # Control & safety
    '/install/go2w_control/lib/go2w_control/'
    '/install/go2w_safety/lib/go2w_safety/'
    '/install/go2w_spawn/lib/go2w_spawn/'
    '/install/go2w_observability/lib/go2w_observability/'
    # CHAMP + EKF + RSP + ros2_control spawners
    '/champ_base/lib/champ_base/quadruped_controller_node'
    '/champ_base/lib/champ_base/state_estimation_node'
    '/robot_localization/ekf_node'
    '/robot_state_publisher'
    '/opt/ros/.*/lib/controller_manager/spawner'
    # RViz + static TF
    '/opt/ros/humble/lib/rviz2/rviz2'
    '/opt/ros/humble/lib/tf2_ros/static_transform_publisher .*(__ns:=/robot|/robot/tf|/robot/tf_static)'
    # Catch-all for anything from the workspace install dir
    "${WS_DIR}/install/"
  )
  local pattern
  for pattern in "${patterns[@]}"; do
    terminate_matching_processes "$pattern" "$signal"
  done
}

cleanup_targets_remaining() {
  local -a patterns=(
    'ros2 launch vlm_explorer single_vlm_mujoco_far.launch.py'
    'ros2 launch go2_gazebo_sim single_go2w_mujoco_cfpa2.launch.py'
    'mujoco_ros2_control'
    'mujoco_sensor_bridge'
    'vlm_history_viewer.py'
    '/opt/ros/humble/lib/cartographer_ros/cartographer_node'
    '/champ_base/lib/champ_base/'
    '/robot_localization/ekf_node'
    "${WS_DIR}/install/"
  )
  local pattern
  for pattern in "${patterns[@]}"; do
    if any_matching_processes "$pattern"; then
      return 0
    fi
  done
  return 1
}

terminate_launch_job() {
  local signal="$1"
  if [[ -n "${LAUNCH_JOB_PGID}" ]] && [[ "${LAUNCH_JOB_PGID}" != "${SELF_PGID}" ]]; then
    kill "-${signal}" "--" "-${LAUNCH_JOB_PGID}" 2>/dev/null || true
  fi
  if [[ -n "${LAUNCH_JOB_PID}" ]] && kill -0 "${LAUNCH_JOB_PID}" 2>/dev/null; then
    pkill "-${signal}" -P "${LAUNCH_JOB_PID}" 2>/dev/null || true
    kill "-${signal}" "${LAUNCH_JOB_PID}" 2>/dev/null || true
  fi
}

cleanup_on_exit() {
  local exit_code=$?
  local attempt
  trap - EXIT INT TERM
  terminate_launch_job INT
  sleep 1
  terminate_launch_job TERM
  for _ in 1 2 3; do
    cleanup_stale_processes TERM
    sleep 1
  done
  cleanup_stale_processes KILL
  for attempt in 1 2 3 4 5; do
    if ! cleanup_targets_remaining; then
      break
    fi
    sleep 1
    cleanup_stale_processes KILL
  done
  exit "${exit_code}"
}

trap cleanup_on_exit EXIT INT TERM

cleanup_stale_processes TERM
sleep 1
cleanup_stale_processes KILL
sleep 1

if [[ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" && -f "${WS_DIR}/config/fastdds_no_shm.xml" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="${WS_DIR}/config/fastdds_no_shm.xml"
fi

VLM_PROVIDER="${VLM_PROVIDER:-xai}"
ARTIFACT_DETECTION_MODE="${ARTIFACT_DETECTION_MODE:-placeholder}"
SLAM_SOURCE="cartographer"
NAV_EXECUTION_BACKEND="${NAV_EXECUTION_BACKEND:-far}"
MAP_BACKEND="carto_2d"
FLORENCE2_ENABLED="${FLORENCE2_ENABLED:-false}"
MISSION_PROMPT="${MISSION_PROMPT:-find any small unusual objects on the floor}"

# CLI overrides for nav backend only (SLAM and map are fixed to Cartographer 2D).
FORWARD_ARGS=()
for arg in "$@"; do
  case "$arg" in
    nav_execution_backend:=*)
      NAV_EXECUTION_BACKEND="${arg#nav_execution_backend:=}"
      ;;
    *)
      FORWARD_ARGS+=("$arg")
      ;;
  esac
done

NAV_EXECUTION_BACKEND="${NAV_EXECUTION_BACKEND,,}"
# Back-compat aliases — old callers still pass the removed planners' names.
case "${NAV_EXECUTION_BACKEND}" in
  reactive)           NAV_EXECUTION_BACKEND="default" ;;
  rrt_star|far_rrt_star) NAV_EXECUTION_BACKEND="astar" ;;
esac
case "${NAV_EXECUTION_BACKEND}" in
  far|astar|default) ;;
  *)
    echo "ERROR: unsupported NAV_EXECUTION_BACKEND='${NAV_EXECUTION_BACKEND}' (expected far|astar|default)" >&2
    exit 2
    ;;
esac

if [[ -z "${VLM_MODEL:-}" ]]; then
  if [[ "${VLM_PROVIDER}" == "xai" || ( "${VLM_PROVIDER}" == "auto" && -n "${XAI_API_KEY:-}" ) ]]; then
    VLM_MODEL="${XAI_VLM_MODEL:-grok-4-1-fast-non-reasoning}"
  else
    VLM_MODEL="gpt-4o-mini"
  fi
fi

if [[ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]]; then
  echo "Using FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE}"
fi
echo "VLM_PROVIDER=${VLM_PROVIDER}"
echo "VLM_MODEL=${VLM_MODEL}"
echo "ARTIFACT_DETECTION_MODE=${ARTIFACT_DETECTION_MODE}"
echo "SLAM_SOURCE=${SLAM_SOURCE}"
echo "NAV_EXECUTION_BACKEND=${NAV_EXECUTION_BACKEND}"
echo "MAP_BACKEND=${MAP_BACKEND}"
echo "FLORENCE2_ENABLED=${FLORENCE2_ENABLED}"
echo "MISSION_PROMPT=${MISSION_PROMPT}"

# Default all nodes to WARN; CFPA2 and nav nodes override to INFO
# via ros_arguments in their launch files.
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=WARN

# Start VLM history web viewer in background (http://localhost:8501)
python3 "${WS_DIR}/src/vlm_explorer/scripts/vlm_history_viewer.py" --port 8501 &
VIEWER_PID=$!
echo "VLM history viewer: http://localhost:8501 (pid=${VIEWER_PID})"

run_pretty_logged ros2 launch vlm_explorer single_vlm_mujoco_far.launch.py \
  vlm_model:="${VLM_MODEL}" \
  slam_source:="${SLAM_SOURCE}" \
  nav_execution_backend:="${NAV_EXECUTION_BACKEND}" \
  map_backend:="${MAP_BACKEND}" \
  florence2_enabled:="${FLORENCE2_ENABLED}" \
  florence2_goal_prompt:="small red block" \
  mission_prompt:="${MISSION_PROMPT}" \
  gui:=true \
  rviz:=true \
  "${FORWARD_ARGS[@]}" &
LAUNCH_JOB_PID=$!
LAUNCH_JOB_PGID="$(ps -o pgid= "${LAUNCH_JOB_PID}" | tr -d ' ' || true)"
wait "${LAUNCH_JOB_PID}"
