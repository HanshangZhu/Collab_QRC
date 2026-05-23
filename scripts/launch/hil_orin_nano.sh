#!/usr/bin/env bash
# hil_orin_nano.sh — single entry point for the Orin Nano HIL bench.
#
# Handles BOTH sides of the cross-host setup:
#   • Desktop : MuJoCo sim + CHAMP + sensor bridges + RViz (nav_test.rviz w/
#               2D Goal Pose tool wired to /robot/goal_pose)
#   • Jetson  : Fast-LIO + elevation_mapping_cupy + filter_chain_runner +
#               grid_map_to_occupancy_grid_cpp + Nav2 (CUDA-MPPI) + CFPA2
#
# What it does in order:
#   1. Preflight kill on Jetson (over SSH) — wipe stale autonomy procs + DDS shm
#   2. Preflight kill on desktop — same for the sim side
#   3. Start desktop sim, wait for /clock + platform ready
#   4. Start Jetson autonomy with explore:=true (CFPA2 frontier exploration)
#   5. Tail logs from BOTH sides into one terminal (optional)
#   6. Optional: open jtop + topic-rate monitor in extra terminals
#
# Usage:
#   ./scripts/launch/hil_orin_nano.sh                       # default: start everything
#   ./scripts/launch/hil_orin_nano.sh stop                  # kill BOTH sides cleanly
#   ./scripts/launch/hil_orin_nano.sh status                # show what's running
#   ./scripts/launch/hil_orin_nano.sh monitor               # don't start, just spawn jtop + topic hz + logs
#   NO_RVIZ=1 ./scripts/launch/hil_orin_nano.sh             # desktop sim without RViz
#   NO_EXPLORE=1 ./scripts/launch/hil_orin_nano.sh          # start Nav2 but no CFPA2
#
# Env knobs:
#   JETSON_USER   default 'johnpork233'
#   JETSON_HOST   default '192.168.55.49'
#   JETSON_PASS   default '233'
#   JETSON_WS     default '/home/${JETSON_USER}/jetson_ws'
#   NO_RVIZ       set 1 → desktop launches with rviz:=false
#   NO_EXPLORE    set 1 → Jetson launches with explore:=false (manual goals only)
#   NO_MONITOR    set 1 → skip jtop + topic hz extra terminals
#   SCENE         override MuJoCo scene path (default slam_ops2_v4_go2_real.xml)
set -u

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
JETSON_USER="${JETSON_USER:-johnpork233}"
JETSON_HOST="${JETSON_HOST:-192.168.55.49}"
JETSON_PASS="${JETSON_PASS:-233}"
JETSON_WS="${JETSON_WS:-/home/${JETSON_USER}/jetson_ws}"

SSH_OPTS=(-o StrictHostKeyChecking=accept-new -o ConnectTimeout=5 -o BatchMode=no)
SSH() { sshpass -p "$JETSON_PASS" ssh "${SSH_OPTS[@]}" "${JETSON_USER}@${JETSON_HOST}" "$@"; }

# Process-name patterns. Note pkill matches on the FULL command line so
# patterns that match a literal command must avoid matching the running
# pkill/ssh command itself. We use pgrep + explicit kill instead.
DESKTOP_PROCS='mujoco_ros2_control|ros2 launch go2_gazebo_sim single_go2w_mujoco_cfpa2|rviz2_single_go2w|champ|robot_state_publisher|pointcloud_adapter|exploration_metrics_logger|ekf_node|spawn|stand_up_slowly|wall_collision_checker|autonomy_enabler|supervisor_panic_node|qos_bridge|twist_bridge|pointcloud_to_laserscan_node|wait_for_ready|nav_test_hil_desktop'
JETSON_PROCS='fastlio_mapping|elevation_mapping_node|filter_chain_runner|controller_server|planner_server|behavior_server|bt_navigator|lifecycle_manager_navigation|fast_lio_tf_adapter|grid_map_to_occupancy|cfpa2_single_robot|cfpa2_to_nav2|path_relay|orin_nano_hil_jetson|run_jetson_hil|static_transform_publisher'

# ── helpers ──────────────────────────────────────────────────────────
ok()    { echo -e "  \033[32m✓\033[0m $*"; }
warn()  { echo -e "  \033[33m⚠\033[0m $*"; }
banner() { echo; echo "════════════════════════════════════════════════════════════"; echo "  $*"; echo "════════════════════════════════════════════════════════════"; }

kill_desktop() {
  banner "Preflight kill — desktop"
  local pids
  pids=$(pgrep -f "${DESKTOP_PROCS}" 2>/dev/null | grep -v "^$$\$" || true)
  if [[ -n "$pids" ]]; then
    echo "  killing PIDs: $(echo $pids | tr '\n' ' ')"
    kill -KILL $pids 2>/dev/null || true
    sleep 2
  fi
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* /dev/shm/cdds_* /dev/shm/iox_* 2>/dev/null
  if pgrep -f "${DESKTOP_PROCS}" >/dev/null 2>&1; then
    warn "some processes survived:"; pgrep -af "${DESKTOP_PROCS}" | head -5
  else
    ok "desktop clean"
  fi
}

kill_jetson() {
  banner "Preflight kill — Jetson ($JETSON_HOST)"
  # Robust teardown (hardened 2026-05-23 after a respawn-zombie pileup).
  # Three things the old one-shot version got wrong:
  #   1. orin_nano_hil_jetson.launch.py runs respawn=True on some nodes, so
  #      killing the nodes alone makes the LAUNCH resurrect them. We MUST kill
  #      the launch ROOTS (run_jetson_hil.sh + `ros2 launch` + launch_service)
  #      FIRST, then the nodes — and loop, because respawn_delay=3.0 races the
  #      kill.
  #   2. The verify used `pgrep -f '<patterns>'` INSIDE an SSH command whose
  #      own argv contained those patterns, so pgrep false-matched the command
  #      shell itself ("2-3 survivors of every node" that were never real).
  #      Fix: pipe the whole teardown via STDIN to `bash -s`, so the patterns
  #      live in stdin, never in any process argv → pgrep -f is reliable.
  #   3. Single pass left survivors silently. Now: loop until clean + verify.
  local out
  out=$(SSH 'bash -s' <<'JEOF' 2>&1
ROOTS='run_jetson_hil|ros2 launch|orin_nano_hil_jetson|launch_service'
NODES='fastlio_mapping|fast_lio_tf_adapter|elevation_mapping_node|filter_chain_runner|controller_server|planner_server|behavior_server|bt_navigator|lifecycle_manager|grid_map_to_occupancy|cfpa2_single_robot|cfpa2_to_nav2|cfpa2_coordinator|path_relay|static_transform_publisher|robot_state_publisher|component_container|topic_tools'
me=$$
# 1) Respawn roots first (else respawn=True brings nodes back).
for p in 1 2 3; do
  R=$(pgrep -f "$ROOTS" 2>/dev/null | grep -vw "$me")
  [ -n "$R" ] || break
  kill -KILL $R 2>/dev/null; sleep 3
done
# 2) Nodes (no respawn source now), several passes.
for p in 1 2 3 4 5; do
  N=$(pgrep -f "$NODES" 2>/dev/null | grep -vw "$me")
  [ -n "$N" ] || break
  kill -KILL $N 2>/dev/null; sleep 2
done
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* /dev/shm/cdds_* /dev/shm/iox_* 2>/dev/null
# 3) Verify (reliable: patterns are in stdin, not this shell's argv).
LR=$(pgrep -f "$ROOTS" 2>/dev/null | grep -vw "$me")
LN=$(pgrep -f "$NODES" 2>/dev/null | grep -vw "$me")
if [ -z "$LR$LN" ]; then
  echo CLEAN
else
  echo SURVIVORS
  pgrep -af "$ROOTS" 2>/dev/null | grep -vw "$me" | head -4
  pgrep -af "$NODES" 2>/dev/null | grep -vw "$me" | head -4
fi
JEOF
)
  echo "$out" | grep -v '^CLEAN$' | sed 's/^/  /'
  if echo "$out" | grep -q '^CLEAN$'; then ok "jetson clean"; else warn "jetson NOT clean — survivors above; re-run '$0 stop'"; fi
}

start_desktop() {
  banner "Starting desktop sim"
  local rviz_arg=()
  [[ "${NO_RVIZ:-0}" = "1" ]] && rviz_arg=("rviz:=false")
  # Default to HANDWALLS variant (draw_walls_2d.py, 2026-05-20): 33 hand-traced
  # wall segments — cleanest sim-side collision geometry, no PolyFit auto-fit
  # artifacts. Override with POLYFIT_VARIANT=clustered|real|nopoly or NO_POLYFIT=1.
  export POLYFIT_VARIANT="${POLYFIT_VARIANT:-handwalls}"
  local log="/tmp/hil_desktop.log"
  rm -f "$log"
  ( "${WS_DIR}/scripts/launch/nav_test_hil_desktop.sh" "${rviz_arg[@]}" > "$log" 2>&1 ) &
  local pid=$!
  echo "  desktop launch pid=$pid (log: $log)"
  echo "  waiting for platform ready..."
  local deadline=$((SECONDS + 120))
  while ! grep -qE "platform.*Ready|stand-up trajectory" "$log" 2>/dev/null; do
    sleep 3
    if [[ $SECONDS -gt $deadline ]]; then
      warn "desktop didn't reach Ready in 120s — see $log tail:"
      tail -10 "$log"
      return 1
    fi
    if ! kill -0 $pid 2>/dev/null; then
      warn "desktop launch process exited — see $log tail:"
      tail -10 "$log"
      return 1
    fi
  done
  ok "desktop ready (after $(( SECONDS - (deadline - 120) ))s)"
}

start_jetson() {
  banner "Starting Jetson autonomy"
  local explore_arg=""
  [[ "${NO_EXPLORE:-0}" != "1" ]] && explore_arg="explore:=true"
  # nohup so SSH disconnect doesn't kill it; bash run_jetson_hil.sh writes
  # to ~/jetson_ws/logs/jetson_hil_<ts>.log + symlinks latest.log.
  SSH "nohup bash ${JETSON_WS}/scripts/real/run_jetson_hil.sh ${explore_arg} > /tmp/jetson_hil.log 2>&1 & echo started_pid=\$!" 2>&1 | sed 's/^/  /'
  echo "  waiting for Nav2 + CUDA-MPPI to come up (up to 90s)..."
  local deadline=$((SECONDS + 90))
  while true; do
    if SSH "grep -q 'CUDA backend ENABLED' /tmp/jetson_hil.log 2>/dev/null"; then
      break
    fi
    if [[ $SECONDS -gt $deadline ]]; then
      warn "Jetson didn't activate Nav2 in 90s — log tail:"
      SSH "tail -15 /tmp/jetson_hil.log" 2>&1 | sed 's/^/    /'
      return 1
    fi
    sleep 3
  done
  ok "Jetson autonomy active (CUDA backend ENABLED, Nav2 lifecycle = active)"
}

show_status() {
  banner "Status"
  echo "--- desktop processes:"
  pgrep -af "${DESKTOP_PROCS}" 2>/dev/null | grep -v "^$$\$" | head -8 | sed 's/^/  /' || echo "  (none)"
  echo
  echo "--- Jetson processes:"
  SSH "pgrep -af '${JETSON_PROCS}' | grep -v \$\$ | head -10" 2>&1 | sed 's/^/  /'
  echo
  echo "--- topic rates (5s window, cross-host):"
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash 2>/dev/null
    [[ -f "${WS_DIR}/install/setup.bash" ]] && source "${WS_DIR}/install/setup.bash" 2>/dev/null
    export FASTRTPS_DEFAULT_PROFILES_FILE="${WS_DIR}/config/fastdds_no_shm.xml"
    for t in /clock /robot/Odometry /robot/elevation_map_raw /robot/elevation_map_filtered /robot/traversability_grid /robot/way_point_coord /robot/cmd_vel_legged; do
      printf "  %-45s " "$t"
      timeout 7 ros2 topic hz "$t" 2>&1 | grep -E "average rate|no new|does not appear" | head -1 || echo "(no msg)"
    done
  fi
}

open_monitor() {
  if [[ "${NO_MONITOR:-0}" = "1" ]]; then return 0; fi
  banner "Monitor terminals"
  if ! command -v gnome-terminal &>/dev/null && ! command -v xterm &>/dev/null; then
    warn "no gnome-terminal/xterm — skipping monitor windows. Run manually:"
    echo "    sshpass -p $JETSON_PASS ssh ${JETSON_USER}@${JETSON_HOST} 'tegrastats --interval 1000'"
    echo "    sshpass -p $JETSON_PASS ssh ${JETSON_USER}@${JETSON_HOST} 'tail -f /tmp/jetson_hil.log'"
    echo "    tail -f /tmp/hil_desktop.log"
    return 0
  fi
  local term
  if command -v gnome-terminal &>/dev/null; then term=(gnome-terminal --); else term=(xterm -e); fi
  # Jetson tegrastats
  "${term[@]}" bash -c "sshpass -p $JETSON_PASS ssh ${SSH_OPTS[*]} ${JETSON_USER}@${JETSON_HOST} 'tegrastats --interval 1000' ; read -p 'tegrastats ended — enter to close'" 2>/dev/null &
  # Jetson log tail
  "${term[@]}" bash -c "sshpass -p $JETSON_PASS ssh ${SSH_OPTS[*]} ${JETSON_USER}@${JETSON_HOST} 'tail -f /tmp/jetson_hil.log' ; read -p 'log ended — enter to close'" 2>/dev/null &
  ok "spawned tegrastats + Jetson log windows"
}

case "${1:-up}" in
  up|start|"")
    kill_desktop
    kill_jetson
    start_desktop || exit 1
    start_jetson || exit 1
    open_monitor
    banner "HIL up"
    echo "  Desktop log : /tmp/hil_desktop.log"
    echo "  Jetson log  : /tmp/jetson_hil.log (on ${JETSON_HOST})"
    echo "  Jetson log local pull: ./scripts/real/fetch_jetson_logs.sh latest"
    echo "  RViz 2D Goal Pose : top toolbar, publishes to /robot/goal_pose"
    echo "  Stop both sides   : $0 stop"
    echo "  Status            : $0 status"
    ;;
  stop|down|kill)
    kill_desktop
    kill_jetson
    ok "everything down"
    ;;
  status)
    show_status
    ;;
  monitor)
    open_monitor
    show_status
    ;;
  *)
    echo "usage: $0 [up|stop|status|monitor]"
    exit 1
    ;;
esac
