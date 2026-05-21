#!/usr/bin/env bash
# real_run_monitor.sh — one-command launcher + live monitor for a real Go2 run.
#
# Does three things in one terminal session:
#   1. SSH to Orin NX → (re)start onboard_autonomy_noetic.sh with the chosen params
#   2. Start laptop-side viz (UDP relay RX + odom_to_tf + RViz2) via nx_viz_laptop.sh
#   3. Print a live dashboard (5-second refresh) showing rates for every key topic
#      and the SLAM / trav / nav / CFPA2 health summary
#
# Usage:
#   ./scripts/real/real_run_monitor.sh                          # defaults
#   ./scripts/real/real_run_monitor.sh explore=false            # nav-only
#   ./scripts/real/real_run_monitor.sh max_vel=0.50             # fast mode
#   ./scripts/real/real_run_monitor.sh max_vel=0.15 max_vel_ang=0.5  # cautious
#   ./scripts/real/real_run_monitor.sh lidar_range=8.0          # radial LiDAR range cap
#   ./scripts/real/real_run_monitor.sh trav_height_range=2.5    # taller space ceiling gate
#   ./scripts/real/real_run_monitor.sh rviz:=false              # no RViz2 on laptop
#   ./scripts/real/real_run_monitor.sh log:=true                # save ROS 2 bag + config
#   ./scripts/real/real_run_monitor.sh stop                     # kill everything
#
# Params passed through to onboard_autonomy_noetic.sh:
#   explore=   (default true)   slam=        (default pointlio)
#   max_vel=   (default 0.30)   max_vel_ang= (default 0.8)
#   lidar_range=       radial range cap in SLAM preprocess, metres
#   trav_height_range= elevation/traversability height gate above sensor, metres
#
# Laptop-side params:
#   rviz:=true|false  (default true)
#   ns=               (default robot)

set -uo pipefail

# ── Config ────────────────────────────────────────────────────────────
NX_HOST="unitree@192.168.123.18"
NX_PASS="${NX_PASS:-}"
NX_WS="/home/unitree/autonomous_exploration_zhu"
NX_SCRIPT="${NX_WS}/scripts/onboard_autonomy_noetic.sh"
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LAPTOP_VIZ_SCRIPT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/launch/nx_viz_laptop.sh"
DASHBOARD_INTERVAL=5        # seconds between dashboard refreshes
MONITOR_DURATION=0          # 0 = run forever; set to e.g. 120 to auto-stop after 2 min
SSH_COMMON_OPTS=(-o StrictHostKeyChecking=accept-new)
LIVOX_HOST_IP="${LIVOX_HOST_IP:-192.168.123.100}"
LIVOX_NIC="${LIVOX_NIC:-eth0}"
LAPTOP_ETHERNET_SETUP_SCRIPT="${WS_DIR}/scripts/real/laptop_ethernet_setup.sh"

# ── Default args ─────────────────────────────────────────────────────
NAMESPACE="robot"
EXPLORE="true"
SLAM="pointlio"
MAX_VEL="0.30"
MAX_VEL_ANG="0.8"
LIDAR_RANGE="8.0"
TRAV_HEIGHT_RANGE="1.7"
ENABLE_RVIZ="true"
EXECUTE_CONTROLLER="true"    # laptop-side /robot/cmd_vel -> /api/sport/request bridge
NX_EXTRA_ARGS=""            # any other args forwarded verbatim to onboard script
LOG_ENABLED="false"
LOG_DIR=""

# ── Parse ─────────────────────────────────────────────────────────────
ONBOARD_ARGS=()
for arg in "$@"; do
  case "$arg" in
    stop)
      echo "=== Stopping NX stack and laptop viz ==="
      ssh "${SSH_COMMON_OPTS[@]}" -o ConnectTimeout=5 "$NX_HOST" \
        "bash -l -c 'source /opt/ros/noetic/setup.bash; \
                     source ${NX_WS}/devel/setup.bash 2>/dev/null || true; \
                     ${NX_SCRIPT} stop; \
                     pgrep -f \"[b]ash .*/onboard_autonomy_noetic.sh\" | xargs -r kill -9'" 2>/dev/null || true
      bash "$LAPTOP_VIZ_SCRIPT" stop 2>/dev/null || true
      pkill -INT -f "cmd_vel_to_sport_bridge.py.*${NAMESPACE}/cmd_vel" 2>/dev/null || true
      echo "Done."
      exit 0
      ;;
    ns=*|namespace=*)  NAMESPACE="${arg#*=}" ;;
    explore=*)         EXPLORE="${arg#explore=}" ;;
    slam=*)            SLAM="${arg#slam=}" ;;
    max_vel=*)         MAX_VEL="${arg#max_vel=}" ;;
    max_vel_ang=*)     MAX_VEL_ANG="${arg#max_vel_ang=}" ;;
    lidar_range=*)     LIDAR_RANGE="${arg#lidar_range=}" ;;
    trav_height_range=*) TRAV_HEIGHT_RANGE="${arg#trav_height_range=}" ;;
    rviz:=*)           ENABLE_RVIZ="${arg#rviz:=}" ;;
    execute=*)         EXECUTE_CONTROLLER="${arg#execute=}" ;;
    log:=*)            LOG_ENABLED="${arg#log:=}" ;;
    log_dir=*)         LOG_DIR="${arg#log_dir=}" ;;
    duration=*)        MONITOR_DURATION="${arg#duration=}" ;;
    *) ONBOARD_ARGS+=("$arg") ;;   # unknown args pass through verbatim
  esac
done

safe_source() { set +u; source "$1" 2>/dev/null || true; set -u; }

# ── ROS 2 env (laptop side for monitoring) ────────────────────────────
if [[ -f "${HOME}/miniforge3/etc/profile.d/conda.sh" ]]; then
  safe_source "${HOME}/miniforge3/etc/profile.d/conda.sh"
  conda activate cmu_env 2>/dev/null || true
elif command -v micromamba >/dev/null 2>&1; then
  eval "$(micromamba shell hook -s bash)" 2>/dev/null || true
  micromamba activate cmu_env 2>/dev/null || true
fi
safe_source /opt/ros/humble/setup.bash
safe_source "${WS_DIR}/install/setup.bash"

# Unitree Sport API discovery is DDS-native. Keep laptop NIC/DDS details in a
# helper so this monitor remains an orchestration script.
if [[ -f "$LAPTOP_ETHERNET_SETUP_SCRIPT" ]]; then
  # shellcheck source=/dev/null
  source "$LAPTOP_ETHERNET_SETUP_SCRIPT"
  setup_real_monitor_laptop_ethernet || exit 1
else
  echo "WARN: ${LAPTOP_ETHERNET_SETUP_SCRIPT} not found; Unitree DDS may not discover Sport API." >&2
fi

# ── NX connectivity check ─────────────────────────────────────────────
echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  Go2 Real-Run Monitor                                ║"
echo "║  NX: ${NX_HOST}                                ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
echo "  max_vel=${MAX_VEL} m/s   wz_max=${MAX_VEL_ANG} rad/s"
echo "  lidar_range=${LIDAR_RANGE} m radial   trav_height_range=${TRAV_HEIGHT_RANGE} m above sensor"
echo "  explore=${EXPLORE}   slam=${SLAM}   ns=${NAMESPACE}   execute=${EXECUTE_CONTROLLER}"
echo ""

if ! ping -c1 -W2 192.168.123.18 &>/dev/null; then
  echo "ERROR: cannot reach NX (192.168.123.18). Check ethernet cable." >&2
  exit 1
fi
echo "  [OK] NX reachable."

# ── 1. Launch onboard stack on NX ────────────────────────────────────
# Build onboard args explicitly from the RESOLVED monitor values so the
# defaults always reach the NX even when the user
# didn't pass them. Unknown pass-through args (minus any viz_relay= the
# user typed, which we set ourselves) follow.
ONBOARD_PASSTHROUGH=()
for a in "${ONBOARD_ARGS[@]:-}"; do
  case "$a" in viz_relay=*|viz_laptop_ip=*|execute=*|sport_interface=*) ;; *) [[ -n "$a" ]] && ONBOARD_PASSTHROUGH+=("$a") ;; esac
done
ONBOARD_ARGS_STR="ns=${NAMESPACE} explore=${EXPLORE} slam=${SLAM} \
max_vel=${MAX_VEL} max_vel_ang=${MAX_VEL_ANG} lidar_range=${LIDAR_RANGE} \
trav_height_range=${TRAV_HEIGHT_RANGE} \
livox_host=${LIVOX_HOST_IP} nic=${LIVOX_NIC} execute=false \
${ONBOARD_PASSTHROUGH[*]:-}"
# laptop IP MUST be on the Go2 net (192.168.123.x) so the NX can UDP back
# over the ethernet link, NOT wifi/tailscale. hostname -I orders by NIC and
# often puts wifi first; pick the 192.168.123.x address explicitly.
LAPTOP_GO2_IP="$(hostname -I | tr ' ' '\n' | grep -E '^192\.168\.123\.' | head -1)"
if [[ -z "$LAPTOP_GO2_IP" ]]; then
  echo "ERROR: no 192.168.123.x IP on this laptop — is the Go2 ethernet up?" >&2
  echo "  Current IPs: $(hostname -I)" >&2
  exit 1
fi
VIZ_ARGS="viz_relay=true viz_laptop_ip=${LAPTOP_GO2_IP}"

echo ""
echo "  [1/3] Launching onboard stack on NX (SSH)..."
echo "        args: ${ONBOARD_ARGS_STR} ${VIZ_ARGS}"
echo ""

# Run in background via setsid/nohup so SSH disconnect doesn't kill it.
# We source devel/setup.bash first so roslaunch/rosrun resolve pkg paths.
ssh "${SSH_COMMON_OPTS[@]}" -o ConnectTimeout=10 -o BatchMode=no "$NX_HOST" "bash -s" <<EOF &
  set -e
  export PATH="\$(echo \"\$PATH\" | tr ':' '\n' | grep -vE '(miniconda|conda)' | tr '\n' ':' | sed 's/:\$//')"
  unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONPATH PYTHONHOME
  source /opt/ros/noetic/setup.bash
  source ${NX_WS}/devel/setup.bash
  if ! ip -4 addr show "${LIVOX_NIC}" | grep -q " ${LIVOX_HOST_IP}/"; then
    if [ -n "${NX_PASS}" ]; then
      echo "${NX_PASS}" | sudo -S ip addr add "${LIVOX_HOST_IP}/24" dev "${LIVOX_NIC}" 2>/tmp/real_run_livox_ip.log || true
    else
      sudo -n ip addr add "${LIVOX_HOST_IP}/24" dev "${LIVOX_NIC}" 2>/tmp/real_run_livox_ip.log || true
    fi
  fi
  # Kill any leftover stack first
  ${NX_SCRIPT} stop 2>/dev/null || true
  pgrep -f "[b]ash .*/onboard_autonomy_noetic.sh" | xargs -r kill -9 || true
  sleep 1
  # Launch detached so this SSH session can exit
  nohup setsid bash ${NX_SCRIPT} ${ONBOARD_ARGS_STR} ${VIZ_ARGS} \
    >/tmp/real_run_onboard.log 2>&1 &
  disown
  echo "  NX stack launched (log: /tmp/real_run_onboard.log)"
EOF
SSH_LAUNCH_PID=$!
wait $SSH_LAUNCH_PID || { echo "ERROR: SSH launch failed." >&2; exit 1; }
echo "  [OK] NX stack launch command sent."

# ── 2. Laptop-side viz ────────────────────────────────────────────────
echo ""
echo "  [2/3] Starting laptop viz (relay RX + TF bridge + RViz2)..."
bash "$LAPTOP_VIZ_SCRIPT" "rviz:=${ENABLE_RVIZ}" &
VIZ_PID=$!
echo "  [OK] Laptop viz PID=${VIZ_PID}"

# Real actuation path: the NX SDK2 Python runtime currently segfaults on this
# robot, so keep NX in nav dry-run mode and send the relayed /robot/cmd_vel to
# the Go2 Sport API from the laptop ROS 2 side. obstacle_avoidance=false means
# direct Sport Move api_id=1008, not OA api_id=1003.
SPORT_PID=""
if [[ "$EXECUTE_CONTROLLER" == "true" ]]; then
  echo ""
  echo "  [2b/3] Starting laptop Sport bridge (/robot/cmd_vel → /api/sport/request, OA off)..."
  pkill -INT -f "cmd_vel_to_sport_bridge.py.*${NAMESPACE}/cmd_vel" 2>/dev/null || true
  sleep 0.3
  ros2 run go2w_control cmd_vel_to_sport_bridge.py --ros-args \
    -p cmd_vel_topic:="/${NAMESPACE}/cmd_vel" \
    -p sport_topic:=/api/sport/request \
    -p obstacle_avoidance:=false \
    >/tmp/laptop_cmd_vel_to_sport.log 2>&1 &
  SPORT_PID=$!
  echo "  [OK] Laptop Sport bridge PID=${SPORT_PID} (log: /tmp/laptop_cmd_vel_to_sport.log)"
else
  echo ""
  echo "  [2b/3] execute=false — laptop will not publish /api/sport/request."
fi

# ── Optional laptop-side logging ──────────────────────────────────────
BAG_PID=""
if [[ "$LOG_ENABLED" == "true" ]]; then
  if [[ -z "$LOG_DIR" ]]; then
    LOG_DIR="${WS_DIR}/results/real_runs/$(date +%Y%m%d_%H%M%S)"
  fi
  mkdir -p "$LOG_DIR"
  cat >"${LOG_DIR}/run_config.txt" <<EOF_CONFIG
timestamp=$(date --iso-8601=seconds)
nx_host=${NX_HOST}
namespace=${NAMESPACE}
explore=${EXPLORE}
slam=${SLAM}
max_vel=${MAX_VEL}
max_vel_ang=${MAX_VEL_ANG}
lidar_range=${LIDAR_RANGE}
trav_height_range=${TRAV_HEIGHT_RANGE}
livox_host=${LIVOX_HOST_IP}
livox_nic=${LIVOX_NIC}
rviz=${ENABLE_RVIZ}
execute=${EXECUTE_CONTROLLER}
EOF_CONFIG
  echo ""
  echo "  [log] Recording laptop ROS 2 topics → ${LOG_DIR}/laptop_viz_bag"
  ros2 bag record \
    "/${NAMESPACE}/Odometry" \
    "/${NAMESPACE}/traversability_grid" \
    "/${NAMESPACE}/traversability_cloud" \
    "/${NAMESPACE}/way_point_coord" \
    "/${NAMESPACE}/frontier_goal_marker" \
    "/${NAMESPACE}/planned_path" \
    "/${NAMESPACE}/cmd_vel" \
    "/api/sport/request" \
    "/goal_pose" \
    "/tf" "/tf_static" \
    -o "${LOG_DIR}/laptop_viz_bag" \
    >"${LOG_DIR}/ros2_bag_record.log" 2>&1 &
  BAG_PID=$!
  echo "  [log] ros2 bag PID=${BAG_PID}"
fi

# Wait for relay to start receiving (look for Odometry topic)
echo "       Waiting for /robot/Odometry to appear on laptop ROS 2..."
for i in $(seq 1 30); do
  ros2 topic info "/${NAMESPACE}/Odometry" 2>/dev/null | grep -q "Publisher count: [1-9]" && break
  sleep 1
done
if ros2 topic info "/${NAMESPACE}/Odometry" 2>/dev/null | grep -q "Publisher count: [1-9]"; then
  echo "  [OK] /robot/Odometry visible on laptop."
else
  echo "  WARN: /robot/Odometry not yet visible (NX may still be booting — monitor will show 0 Hz)."
fi

# ── 3. Live dashboard ─────────────────────────────────────────────────
echo ""
echo "  [3/3] Starting live dashboard (${DASHBOARD_INTERVAL}s refresh, Ctrl+C to stop)."
echo ""

cleanup() {
  echo ""
  echo "  Caught interrupt — leaving NX stack running."
  echo "  To stop NX:    $0 stop"
  echo "  To stop viz:   $LAPTOP_VIZ_SCRIPT stop"
  [[ -n "${BAG_PID:-}" ]] && kill "$BAG_PID" 2>/dev/null || true
  [[ -n "${SPORT_PID:-}" ]] && kill "$SPORT_PID" 2>/dev/null || true
  kill "$VIZ_PID" 2>/dev/null || true
  exit 0
}
trap cleanup INT TERM

# topic_hz_once: quickly check a topic's rate (1-second window via timeout)
topic_hz_once() {
  local topic="$1"
  # ros2 topic hz exits after the first line; capture with timeout
  timeout 1.5s ros2 topic hz --window 5 "$topic" 2>/dev/null \
    | awk '/average rate:/{printf "%.1f", $3; found=1} END{if(!found) print "0.0"}'
}

# node_alive: check if a ROS 2 node is running (best-effort via topic/service)
node_alive() {
  ros2 node list 2>/dev/null | grep -q "$1" && echo "UP" || echo "--"
}

START_TS=$(date +%s)
ITERATION=0
while true; do
  ITERATION=$((ITERATION + 1))
  NOW=$(date +%s)
  ELAPSED=$((NOW - START_TS))

  # Gather topic rates in parallel via temp files (a backgrounded
  # `VAR=$(...) &` assigns in a subshell — the parent never sees it, which
  # under `set -u` blows up as "unbound variable"). Write to files instead.
  TMPD="$(mktemp -d)"
  topic_hz_once "/${NAMESPACE}/Odometry"            >"$TMPD/odom" 2>/dev/null &
  topic_hz_once "/${NAMESPACE}/traversability_grid" >"$TMPD/trav" 2>/dev/null &
  topic_hz_once "/${NAMESPACE}/cmd_vel"             >"$TMPD/cmd"  2>/dev/null &
  topic_hz_once "/${NAMESPACE}/move_base_simple/goal" >"$TMPD/goal" 2>/dev/null &
  wait
  HZ_ODOM="$(cat "$TMPD/odom" 2>/dev/null || echo 0.0)"; HZ_ODOM="${HZ_ODOM:-0.0}"
  HZ_TRAV="$(cat "$TMPD/trav" 2>/dev/null || echo 0.0)"; HZ_TRAV="${HZ_TRAV:-0.0}"
  HZ_CMD="$(cat "$TMPD/cmd"   2>/dev/null || echo 0.0)"; HZ_CMD="${HZ_CMD:-0.0}"
  HZ_GOAL="$(cat "$TMPD/goal" 2>/dev/null || echo 0.0)"; HZ_GOAL="${HZ_GOAL:-0.0}"
  rm -rf "$TMPD"

  # NX log tail (last error or warning lines via SSH, best-effort)
  NX_STATUS=$(ssh "${SSH_COMMON_OPTS[@]}" -o ConnectTimeout=3 -o BatchMode=yes "$NX_HOST" \
    "tail -n2 /tmp/real_run_onboard.log 2>/dev/null | tr '\n' ' '" 2>/dev/null || echo "(SSH timeout)")

  # SLAM health: check if Odometry is live
  SLAM_STATUS="OK"
  [[ "$HZ_ODOM" == "0.0" ]] && SLAM_STATUS="DEAD"

  # Trav health
  TRAV_STATUS="OK"
  [[ "$HZ_TRAV" == "0.0" ]] && TRAV_STATUS="DEAD"

  # Nav / cmd_vel health
  NAV_STATUS="IDLE"
  (( $(echo "$HZ_CMD > 0.5" | bc -l 2>/dev/null || echo 0) )) && NAV_STATUS="DRIVING"

  # Print dashboard (overwrite previous with \033[<N>A cursor-up)
  DASH_LINES=20
  if [[ $ITERATION -gt 1 ]]; then
    printf "\033[${DASH_LINES}A"
  fi

  printf "┌─────────────────────────────────────────────────────────┐\n"
  printf "│  Go2 Real-Run Monitor  │  t=%4ds  │  %s                 │\n" \
    "$ELAPSED" "$(date '+%H:%M:%S')"
  printf "├──────────────────┬──────────┬──────────────────────────┤\n"
  printf "│  Topic           │  Hz      │  Status                  │\n"
  printf "├──────────────────┼──────────┼──────────────────────────┤\n"
  printf "│  /robot/Odometry │  %6s  │  SLAM: %-17s │\n"  "$HZ_ODOM" "$SLAM_STATUS"
  printf "│  trav_grid       │  %6s  │  Trav: %-17s │\n"  "$HZ_TRAV" "$TRAV_STATUS"
  printf "│  cmd_vel         │  %6s  │  Nav:  %-17s │\n"  "$HZ_CMD"  "$NAV_STATUS"
  printf "│  goal            │  %6s  │                          │\n"  "$HZ_GOAL"
  printf "├──────────────────┴──────────┴──────────────────────────┤\n"
  printf "│  Params: max_vel=%-5s  wz=%-5s  lidar_r=%-5s        │\n" \
    "$MAX_VEL" "$MAX_VEL_ANG" "$LIDAR_RANGE"
  printf "├─────────────────────────────────────────────────────────┤\n"
  # NX log tail — truncate to 53 chars to fit box
  NX_SHORT="${NX_STATUS:0:53}"
  printf "│  NX: %-53s│\n" "$NX_SHORT"
  printf "└─────────────────────────────────────────────────────────┘\n"
  printf "  Ctrl+C to exit monitor (NX stack keeps running)\n"
  printf "  To stop NX: %s stop\n" "$(basename "$0")"
  printf "  To reconfigure vel: ssh %s '%s max_vel=X'\n" "$NX_HOST" "$(basename "$NX_SCRIPT")"
  printf "\n"
  printf "\n"

  # Check auto-stop duration
  if [[ $MONITOR_DURATION -gt 0 ]] && [[ $ELAPSED -ge $MONITOR_DURATION ]]; then
    echo "  Monitor duration ${MONITOR_DURATION}s reached — exiting monitor."
    break
  fi

  sleep "$DASHBOARD_INTERVAL"
done

cleanup
