#!/usr/bin/env bash
# hil_orin_nx.sh — single entry point for the Orin NX HIL bench.
#
# The laptop SIMULATES THE PHYSICAL WORLD (MuJoCo ops2-v4 + fake Mid-360 + CHAMP);
# the Orin NX is a PURE REACTIVE COMPUTE UNIT running the full ROS 1 Noetic
# autonomy stack. Sensors + cmd_vel + viz cross via a from-source ros1_bridge
# on the NX. See docs/claude/orin_nx_hil_design.md.
#
#   • Laptop : MuJoCo (slam_ops2_v4_go2_handwalls) + pc2_to_livox + IMU relay +
#              CHAMP (consumes /robot/cmd_vel) + RViz2
#   • NX     : ros1_bridge (CustomMsg-capable) + onboard_autonomy_noetic.sh hil=true
#              (Point-LIO + trav + move_base CUDA-MPPI + CFPA2)
#
# Order: preflight-kill both → start laptop sim (sensors up) → start NX bridge →
#        start NX stack (hil=true) → optional jtop + topic-rate monitors.
#
# Usage:
#   ./scripts/launch/hil_orin_nx.sh up           # start everything
#   ./scripts/launch/hil_orin_nx.sh up lidar_range=2.5 max_vel=0.4   # + tunables
#   ./scripts/launch/hil_orin_nx.sh stop         # kill BOTH sides
#   ./scripts/launch/hil_orin_nx.sh status        # what's running where
#   ./scripts/launch/hil_orin_nx.sh monitor       # jtop + topic hz, no (re)start
#
# Any k=v args after `up` are forwarded to the NX onboard stack
# (lidar_range, max_vel, max_vel_ang, slam, namespace, …).
#
# Env knobs:
#   NX_USER (unitree)  NX_HOST (192.168.123.18)  NX_PASS (123)
#   NX_WS   (/home/unitree/autonomous_exploration_zhu)
#   NO_RVIZ=1     laptop sim without RViz2
#   NO_EXPLORE=1  NX stack with explore=false (manual goals)
#   POLYFIT_VARIANT  scene variant (default handwalls)
set -u

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
NX_USER="${NX_USER:-unitree}"
NX_HOST="${NX_HOST:-192.168.123.18}"
NX_PASS="${NX_PASS:-123}"
NX_WS="${NX_WS:-/home/unitree/autonomous_exploration_zhu}"

SSH_OPTS=(-o StrictHostKeyChecking=accept-new -o ConnectTimeout=6 -o BatchMode=no)
SSH() { sshpass -p "$NX_PASS" ssh "${SSH_OPTS[@]}" "${NX_USER}@${NX_HOST}" "$@"; }

LAPTOP_PROCS='ros2 launch go2_gazebo_sim single_go2w_mujoco_cfpa2|mujoco_ros2_control|pc2_to_livox_node|topic_tools relay|champ|robot_state_publisher|rviz2|nav_test_hil_nx_desktop'
NX_PROCS='ros1_bridge|parameter_bridge|dynamic_bridge|pointlio_mapping|laserMapping|elevation_mapping_node|trav_filter_occ_grid|move_base|cfpa2_single_robot|cfpa2_to_movebase|onboard_autonomy_noetic'

ok()     { echo -e "  \033[32m✓\033[0m $*"; }
warn()   { echo -e "  \033[33m⚠\033[0m $*"; }
banner() { echo; echo "════════════════════════════════════════════════════════════"; echo "  $*"; echo "════════════════════════════════════════════════════════════"; }

kill_laptop() {
  banner "Preflight kill — laptop (sim world)"
  local pids; pids=$(pgrep -f "${LAPTOP_PROCS}" 2>/dev/null | grep -v "^$$\$" || true)
  [[ -n "$pids" ]] && { kill -KILL $pids 2>/dev/null || true; sleep 2; }
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
  pgrep -f "${LAPTOP_PROCS}" >/dev/null 2>&1 && warn "some laptop procs survived" || ok "laptop clean"
}

kill_nx() {
  banner "Preflight kill — Orin NX (compute)"
  SSH "${NX_WS}/scripts/onboard_autonomy_noetic.sh stop" >/dev/null 2>&1 || true
  SSH "${NX_WS}/scripts/run_nx_hil_bridge.sh stop 2>/dev/null; pkill -9 -f 'ros1_bridge|parameter_bridge|dynamic_bridge' 2>/dev/null; true" >/dev/null 2>&1 || true
  ok "NX kill issued"
}

do_status() {
  banner "Status"
  echo "── laptop ──"
  pgrep -af "${LAPTOP_PROCS}" 2>/dev/null | head -8 || echo "  (none)"
  echo "── NX ──"
  SSH "pgrep -af '${NX_PROCS}' 2>/dev/null | head -12 || echo '  (none)'"
}

do_monitor() {
  banner "Monitor (jtop on NX + topic rates on laptop)"
  echo "  NX load (tegrastats one-shot):"
  SSH "echo $NX_PASS | sudo -S tegrastats --interval 1000 2>/dev/null | head -2" 2>/dev/null
  echo ""
  echo "  Laptop-side topic rates (viz bridged back from NX):"
  source "${WS_DIR}/install/setup.bash" 2>/dev/null || true
  for t in /robot/cmd_vel /robot/traversability_grid /robot/Odometry /robot/way_point_coord; do
    r=$(timeout 5 ros2 topic hz "$t" 2>/dev/null | grep -oE "average rate: [0-9.]+" | head -1)
    printf "    %-32s %s\n" "$t" "${r:-<no data>}"
  done
}

# First positional = subcommand; any k=v args AFTER `up` are forwarded verbatim
# to the NX onboard_autonomy_noetic.sh (e.g. lidar_range=2.5 max_vel=0.4).
SUBCMD="${1:-up}"; shift || true
ONBOARD_EXTRA=""
for a in "$@"; do
  case "$a" in *=*) ONBOARD_EXTRA="${ONBOARD_EXTRA} $a" ;; esac
done
case "$SUBCMD" in
  stop)    kill_laptop; kill_nx; banner "Stopped both sides"; exit 0 ;;
  status)  do_status; exit 0 ;;
  monitor) do_monitor; exit 0 ;;
  up) ;;
  *) echo "usage: $0 [up|stop|status|monitor] [lidar_range=N] [max_vel=N] [max_vel_ang=N]" >&2; exit 1 ;;
esac

# ── reachability ─────────────────────────────────────────────────────
banner "Orin NX HIL — bring up"
if ! SSH "echo ok" >/dev/null 2>&1; then
  echo "ERROR: cannot SSH ${NX_USER}@${NX_HOST}. Check the link (USB-eth dongle)." >&2
  exit 1
fi
ok "NX reachable"

kill_laptop; kill_nx

# 1) Laptop sim (sensors + CHAMP + RViz2). Detached; logs to /tmp.
banner "Start laptop sim world"
RVIZ_ARG=""; [[ "${NO_RVIZ:-0}" = "1" ]] && RVIZ_ARG="rviz:=false"
setsid nohup "${WS_DIR}/scripts/launch/nav_test_hil_nx_desktop.sh" $RVIZ_ARG \
  </dev/null >/tmp/hil_nx_laptop.log 2>&1 &
ok "laptop sim launching (log: /tmp/hil_nx_laptop.log)"
echo "  waiting for /livox/lidar to be published by the laptop..."
source "${WS_DIR}/install/setup.bash" 2>/dev/null || true
for i in $(seq 1 40); do
  timeout 3 ros2 topic info /livox/lidar 2>/dev/null | grep -q "Publisher count: [1-9]" && break
  sleep 2
done
timeout 3 ros2 topic info /livox/lidar 2>/dev/null | grep -q "Publisher count: [1-9]" \
  && ok "/livox/lidar up (laptop sensors live)" \
  || warn "/livox/lidar not seen yet — check /tmp/hil_nx_laptop.log"

# 2) NX autonomy stack in HIL mode. It brings up roscore, starts the C++ UDP
#    relay (rx: sensors in from laptop; tx: cmd_vel/viz out), then the full
#    compute stack (Point-LIO + trav + move_base + CFPA2). The relay REPLACES
#    the Foxy ros1_bridge, which is broken on this Jetson (bad_allocs on every
#    message — see docs/claude/orin_nx_hil_design.md).
banner "Start NX autonomy stack (hil=true) — roscore + UDP relay + compute"
EXPLORE_ARG="explore=true"; [[ "${NO_EXPLORE:-0}" = "1" ]] && EXPLORE_ARG="explore=false"
# Pass the laptop's IP so the NX relay tx can send cmd_vel/viz back.
LAPTOP_IP="${LAPTOP_IP:-192.168.123.222}"
[[ -n "$ONBOARD_EXTRA" ]] && echo "  forwarding to NX onboard:${ONBOARD_EXTRA}"
SSH "HIL_LAPTOP_IP=${LAPTOP_IP} setsid nohup ${NX_WS}/scripts/onboard_autonomy_noetic.sh hil=true ${EXPLORE_ARG}${ONBOARD_EXTRA} </dev/null >/tmp/hil_nx_stack.log 2>&1 & echo started" >/dev/null 2>&1
ok "NX stack launching (log on NX: /tmp/hil_nx_stack.log)"
sleep 4
SSH "grep -qE 'relay rx|/livox/lidar present' /tmp/hil_nx_stack.log 2>/dev/null && echo '  ✓ NX relay + sensors wiring up' || tail -3 /tmp/hil_nx_stack.log 2>/dev/null" 2>/dev/null

banner "HIL bench up"
echo "  Laptop = simulated world (MuJoCo ops2-v4) | NX = compute (SLAM+trav+nav+CFPA2)"
echo "  RViz2 on the laptop shows viz bridged back from the NX."
echo ""
echo "  $0 status    # what's running"
echo "  $0 monitor   # NX load (tegrastats) + topic rates"
echo "  $0 stop      # tear down both"
