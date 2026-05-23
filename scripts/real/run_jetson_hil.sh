#!/usr/bin/env bash
# run_jetson_hil.sh — runner for the Jetson side of the HIL test bench.
# Sets env, sources ROS + workspace, launches orin_nano_hil_jetson.launch.py.
#
# Pre-req on desktop (separate machine):
#   ./scripts/launch/nav_test_hil_desktop.sh
#
# Usage on Jetson (after scripts/real/deploy_to_orin_nano.sh sync):
#   bash /tmp/run_jetson_hil.sh
unset RMW_IMPLEMENTATION CYCLONEDDS_URI
# Match the desktop HIL FastDDS profile (UDP-only, SHM disabled). REQUIRED for
# cross-host matching: without it the Jetson nodes use the default SHM-enabled
# profile, and BEST_EFFORT subscribers (notably fast_lio's SensorDataQoS lidar
# sub) fail to match the desktop's RELIABLE publishers cross-host → fast_lio
# receives no cloud → no odometry → robot frozen. RELIABLE↔RELIABLE (IMU) still
# matches, which masks the bug. Regression found + fixed 2026-05-23.
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/johnpork233/jetson_ws/config/fastdds_no_shm.xml
export ROS_DOMAIN_ID=0
export PATH=$HOME/.local/bin:/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH

# Force pure-cupy CNN backend on Orin Nano (no PyTorch installed).
# Patched into elevation_mapping.py:158 — see [[feedback-jetson-pip-jetson-ai-lab-dns]].
export ELEVATION_MAPPING_FORCE_CUPY=1

source /opt/ros/humble/setup.bash
source /home/johnpork233/jetson_ws/install/setup.bash

# ─────────────────────────────────────────────────────────────────────
# Preflight zombie killer — mirrors scripts/launch/_preflight_kill.sh from
# the desktop side. Each HIL re-run must wipe leftover Jetson nodes or
# duplicate publishers on /robot/elevation_map_raw, /robot/Odometry,
# /robot/cloud_registered_body cause noisy trav grids + competing TF.
# Discovered 2026-05-18: two elevation_mapping instances ran in parallel
# because a partial SSH-broken-pipe pkill missed the previous launch.
# ─────────────────────────────────────────────────────────────────────
_PREFLIGHT_PATTERNS=(
  # Launch supervisor
  "ros2 launch .*orin_nano_hil_jetson"
  "ros2 launch"
  # SLAM
  "fastlio_mapping"
  "/fast_lio/lib/fast_lio/fastlio_mapping"
  "__node:=slam_node"
  # TF adapter + statics
  "fast_lio_tf_adapter"
  "fast_lio_tf_adapter.py"
  "__node:=map_to_camera_init"
  "__node:=body_to_base_link_fastlio"
  "__node:=map_to_odom_identity"
  "__node:=base_link_to_body_dyn"
  "static_transform_publisher"
  "base_link_to_body"
  "dyn_baselink_to_body"
  # Trav pipeline
  "elevation_mapping_node"
  "elevation_mapping_node.py"
  "filter_chain_runner"
  "grid_map_to_occupancy_grid"
  # Nav2 stack
  "nav2_controller/controller_server"
  "controller_server"
  "nav2_planner/planner_server"
  "planner_server"
  "nav2_behaviors/behavior_server"
  "behavior_server"
  "nav2_bt_navigator/bt_navigator"
  "bt_navigator"
  "nav2_lifecycle_manager/lifecycle_manager"
  "lifecycle_manager_navigation"
  # CFPA2 (if explore enabled)
  "cfpa2_single_robot_node"
  "cfpa2_coordinator_node"
)

_ALIVE_RE='fastlio_mapping|fast_lio_tf_adapter|elevation_mapping_node|filter_chain_runner|grid_map_to_occupancy_grid|controller_server|planner_server|behavior_server|bt_navigator|lifecycle_manager_navigation|map_to_camera_init|body_to_base_link_fastlio|map_to_odom_identity|base_link_to_body_dyn|dyn_baselink_to_body|cfpa2_single_robot|cfpa2_coordinator|static_transform_publisher.*camera_init|static_transform_publisher.*base_link'

_clean_jetson_dds_shm() {
  # FastRTPS leaves /dev/shm/fastrtps_* after abrupt exit → next launch hits
  # "already-in-use" or silent discovery failure cross-host.
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
  rm -f /dev/shm/cdds_* /dev/shm/iox_* 2>/dev/null || true
}

_stop_ros2_daemon() {
  local pid
  pid=$(pgrep -af "ros2cli.*daemon" 2>/dev/null | awk '{print $1}' | head -1)
  if [[ -n "${pid}" ]]; then
    kill -TERM "${pid}" 2>/dev/null || true
    sleep 0.3
    kill -KILL "${pid}" 2>/dev/null || true
  fi
}

if pgrep -f "${_ALIVE_RE}" >/dev/null 2>&1; then
  echo "[preflight] killing stale Jetson autonomy procs (SIGKILL only)..."
  for pat in "${_PREFLIGHT_PATTERNS[@]}"; do
    pkill -KILL -f "${pat}" 2>/dev/null || true
  done
  sleep 1
  _stop_ros2_daemon
  _clean_jetson_dds_shm
  if pgrep -f "${_ALIVE_RE}" >/dev/null 2>&1; then
    echo "[preflight] WARNING: some Jetson procs survived SIGKILL:"
    pgrep -af "${_ALIVE_RE}" | head -10
  else
    echo "[preflight] clean."
  fi
else
  _clean_jetson_dds_shm
fi

WS="${HOME}/jetson_ws"
LAUNCH="${WS}/scripts/real/orin_nano_hil_jetson.launch.py"
[[ -f "$LAUNCH" ]] || { echo "ERROR: launch file not at $LAUNCH" >&2; exit 1; }

# ─────────────────────────────────────────────────────────────────────
# Persistent timestamped logs under ~/jetson_ws/logs/.
# Each run gets its own file; `latest.log` symlink always points at the
# current run for `tail -f` convenience. Pull to desktop with
# `scripts/real/fetch_jetson_logs.sh`. Retain last ~30 runs (about a
# week of daily runs); older logs auto-pruned.
# ─────────────────────────────────────────────────────────────────────
LOG_DIR="${WS}/logs"
mkdir -p "${LOG_DIR}"
RUN_TS="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="${LOG_DIR}/jetson_hil_${RUN_TS}.log"
ln -sfn "${LOG_FILE}" "${LOG_DIR}/latest.log"
# Prune logs older than the most recent 30. -t sorts by mtime desc.
ls -t "${LOG_DIR}"/jetson_hil_*.log 2>/dev/null | tail -n +31 | xargs -r rm -f

echo "═══════════════════════════════════════════════════════════════════"
echo "  Orin Nano HIL Jetson side — full autonomy stack"
echo "    fast_lio + tf_adapter + elevation_mapping + filter_chain"
echo "    + grid_map_to_occupancy_grid + Nav2 (planner/MPPI/behaviors/BT)"
echo "  ELEVATION_MAPPING_FORCE_CUPY=$ELEVATION_MAPPING_FORCE_CUPY"
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "  log file : ${LOG_FILE}"
echo "  latest   : ${LOG_DIR}/latest.log  (symlink)"
echo "═══════════════════════════════════════════════════════════════════"

# ─────────────────────────────────────────────────────────────────────
# Cross-host lifecycle gate (2026-05-20):
# Block until desktop side signals /robot/platform_ready=True (latched,
# transient_local). Desktop's wait_for_settle node publishes this AFTER
# stand_up_slowly's trajectory is physically complete + 5s of post-stand-up
# IMU stability. If we exec Fast-LIO before this signal, Fast-LIO's IMU
# initialization happens during stand-up leg motion → computed gravity
# vector is wrong → z-drift accumulates to +6m. Observed 2026-05-20.
#
# Skip the gate with WAIT_FOR_PLATFORM_READY=0 (rarely useful — debug only).
# ─────────────────────────────────────────────────────────────────────
WAIT_FOR_PLATFORM_READY="${WAIT_FOR_PLATFORM_READY:-1}"
PLATFORM_READY_TOPIC="${PLATFORM_READY_TOPIC:-/robot/platform_ready}"
PLATFORM_READY_TIMEOUT_SEC="${PLATFORM_READY_TIMEOUT_SEC:-90}"

if [[ "${WAIT_FOR_PLATFORM_READY}" == "1" ]]; then
  echo
  echo "[lifecycle] Waiting up to ${PLATFORM_READY_TIMEOUT_SEC}s for desktop"
  echo "[lifecycle]   signal on ${PLATFORM_READY_TOPIC} (latched / transient_local)..."
  echo "[lifecycle]   (set WAIT_FOR_PLATFORM_READY=0 to skip)"
  if timeout "${PLATFORM_READY_TIMEOUT_SEC}" \
      ros2 topic echo --once \
      --qos-durability transient_local \
      --qos-reliability reliable \
      "${PLATFORM_READY_TOPIC}" std_msgs/msg/Bool > /tmp/platform_ready.log 2>&1; then
    echo "[lifecycle] ✓ platform_ready received — starting autonomy stack."
  else
    echo "[lifecycle] ⚠ timed out waiting for ${PLATFORM_READY_TOPIC}."
    echo "[lifecycle]   Proceeding anyway (autonomy may use wrong pose; expect z-drift)."
    echo "[lifecycle]   Desktop side: ensure wait_for_settle is running + has signal_topic param."
  fi
  echo
fi

# `exec` so the parent shell of `nohup bash run_jetson_hil.sh > /tmp/.. &`
# correctly inherits the ros2 launch process — but we tee to the persistent
# log file too, so stdout is captured even if the caller didn't redirect.
# Caller redirection (`> /tmp/jetson_hil.log`) still works; we ALSO write
# to LOG_FILE via process substitution.
exec ros2 launch "$LAUNCH" "$@" 2>&1 | tee "${LOG_FILE}"
