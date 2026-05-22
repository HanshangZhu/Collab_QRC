#!/usr/bin/env bash
# diag_cmdvel_chain.sh — pinpoint WHERE the CFPA2→cmd_vel chain breaks on the NX.
#
# Run ON the NX (ROS 1 Noetic) while the autonomy stack is up. Walks the chain
# stage by stage and prints a verdict at the first broken link, so we don't
# guess. The cmd_vel loop is the #1 blocker for autonomous exploration: CFPA2
# emits a goal but the robot won't move unless this whole chain flows.
#
# Chain:  CFPA2 /way_point_coord (PointStamped, 2Hz)
#           → cfpa2_to_movebase_bridge
#             → /move_base_simple/goal (PoseStamped, on goal-change)
#               → move_base (SmacLattice global + CUDA-MPPI local)
#                 → /cmd_vel (Twist, 20Hz)
#
# Usage (on NX):  ./diag_cmdvel_chain.sh            # ns=robot
#                 ./diag_cmdvel_chain.sh robot_b
set -u
NS="${1:-robot}"
if echo "$PATH" | grep -q miniconda; then
  export PATH="$(echo "$PATH" | tr ':' '\n' | grep -vE 'miniconda|conda' | tr '\n' ':' | sed 's/:$//')"
  unset CONDA_PREFIX PYTHONPATH PYTHONHOME 2>/dev/null || true
fi
source /opt/ros/noetic/setup.bash
source /home/unitree/autonomous_exploration_zhu/devel/setup.bash 2>/dev/null
export ROS_MASTER_URI="http://192.168.123.18:11311" ROS_IP="${ROS_IP:-192.168.123.18}"

ok()   { echo -e "  \033[32m✓\033[0m $*"; }
bad()  { echo -e "  \033[31m✗ BREAK HERE:\033[0m $*"; }
info() { echo -e "  · $*"; }
rate() { timeout 6 rostopic hz "$1" 2>/dev/null | grep -oE "average rate: [0-9.]+" | head -1 | grep -oE "[0-9.]+"; }
pubs() { timeout 4 rostopic info "$1" 2>/dev/null | grep -A20 "Publishers:" | grep -c "\* /"; }

echo "════════ cmd_vel chain diagnosis (ns=$NS) ════════"

# Stage 0: prerequisites — odom + map must reach CFPA2
echo "[0] CFPA2 inputs"
r=$(rate "/$NS/odom/nav");  [ -n "$r" ] && ok "/$NS/odom/nav $r Hz" || bad "/$NS/odom/nav SILENT — odom relay down? (CFPA2 needs pose)"
r=$(rate "/$NS/traversability_grid"); [ -n "$r" ] && ok "/$NS/traversability_grid $r Hz" || bad "/$NS/traversability_grid SILENT — trav pipeline down"

# Stage 1: CFPA2 emitting goals
echo "[1] CFPA2 → way_point_coord"
r=$(rate "/$NS/way_point_coord")
if [ -n "$r" ]; then ok "/$NS/way_point_coord $r Hz — CFPA2 emitting goals"
else
  bad "/$NS/way_point_coord SILENT — CFPA2 not emitting."
  info "check: grep -iE 'wait|no_frontier|no_reach' /tmp/onboard_cfpa2.log | tail"
  exit 1
fi

# Stage 2: bridge alive + forwarding
echo "[2] bridge → move_base_simple/goal"
if pgrep -f cfpa2_to_movebase_bridge >/dev/null; then ok "bridge process alive"
else bad "bridge process DEAD — check /tmp/onboard_bridge.log"; tail -5 /tmp/onboard_bridge.log 2>/dev/null | sed 's/^/      /'; exit 1; fi
n=$(pubs "/$NS/move_base_simple/goal")
if [ "${n:-0}" -ge 1 ]; then ok "/$NS/move_base_simple/goal has $n publisher(s)"
else
  bad "/$NS/move_base_simple/goal NO publisher — bridge not forwarding."
  info "bridge only publishes on goal CHANGE (>0.30m). If robot is stationary +"
  info "CFPA2 holds one goal, it forwards ONCE then goes quiet — that's expected."
  info "Watch a forward live:  rostopic echo /$NS/move_base_simple/goal &  then wait for a new CFPA2 goal."
  info "bridge log:"; grep -iE "forwarded goal|error" /tmp/onboard_bridge.log 2>/dev/null | tail -3 | sed 's/^/      /'
fi

# Stage 3: move_base accepted goal + is planning
echo "[3] move_base planning"
if pgrep -f "move_base" >/dev/null; then ok "move_base alive"; else bad "move_base DEAD"; exit 1; fi
# global plan present?
n=$(pubs "/$NS/move_base/SmacLatticePlannerROS/plan"); [ "${n:-0}" -ge 1 ] 2>/dev/null && ok "global planner advertising plan" || info "no global plan topic yet"
gs=$(grep -ciE "Got new plan|goal reached|aborting|failed to" /tmp/onboard_movebase.log 2>/dev/null)
info "move_base log goal/plan events: ${gs:-0}"
grep -iE "aborting|no valid|failed to get a plan|oscillat" /tmp/onboard_movebase.log 2>/dev/null | tail -3 | sed 's/^/      /'

# Stage 4: cmd_vel output (THE payoff)
echo "[4] move_base → cmd_vel"
r=$(rate "/$NS/cmd_vel")
if [ -n "$r" ]; then ok "/$NS/cmd_vel $r Hz — CHAIN COMPLETE, robot should move"
else
  bad "/$NS/cmd_vel SILENT — move_base has no goal committed OR plan failed."
  info "Most common causes (from CLAUDE.md desktop standalone debugging):"
  info "  a) bridge never forwarded a goal (stage 2) — robot stationary, goal not changing"
  info "  b) move_base got goal but planner returns EMPTY (start-in-collision / footprint)"
  info "  c) goal in unknown space — SmacLattice can't path → no cmd_vel"
  info "Quick manual test: publish a goal 1.5m ahead by hand, see if cmd_vel appears:"
  info "  rostopic pub -1 /$NS/move_base_simple/goal geometry_msgs/PoseStamped \\"
  info "    '{header: {frame_id: map}, pose: {position: {x: 1.5}, orientation: {w: 1.0}}}'"
fi
echo "════════════════════════════════════════════════"
