#!/usr/bin/env bash
# kill_sim.sh — reliably tear down the WHOLE desktop sim/autonomy stack AND the
# cross-host Jetson HIL stack, then VERIFY a clean single-host DDS graph.
#
# Why this is hard (lessons from 2026-05-20):
#   - `pkill -f "a|b|c"` alternation silently misses processes.
#   - nav2 server comm names are truncated past 15 chars so `pkill -x` misses
#     them; must match by full-arg substring.
#   - Launch roots trap SIGTERM → must -9 by PID.
#   - THE BIG ONE: a stale Jetson HIL stack (johnpork233@192.168.55.49) on the
#     SAME DDS domain keeps publishing /robot/traversability_grid, /robot/tf,
#     goal_pose, cmd_vel … → "Publisher count: 2", RViz flashing, stale trav
#     layer, goal churn, "unknown goal response" floods. Desktop-only cleanup
#     can NEVER fix that — you must kill the Jetson stack too.
#
# NEVER touches draw_walls_2d.py / editors / this script.
#
# Usage:
#   ./scripts/debug/kill_sim.sh            # desktop + Jetson + verify
#   NO_JETSON=1 ./scripts/debug/kill_sim.sh   # desktop only (Jetson unreachable)
set -u

JETSON_USER="${ORIN_USER:-johnpork233}"
JETSON_HOST="${ORIN_IP:-192.168.55.49}"
JETSON_PASS="${JETSON_PASS:-233}"

echo "[kill_sim] tearing down sim/autonomy stack (desktop + Jetson)..."

# ── 1. Desktop: launch roots by PID (they trap SIGTERM) ────────────────
for pid in $(pgrep -f "ros2 launch go2_gazebo\|nav_test_slam_ops2\|nav_test_3d_explore\|nav_test_mujoco_fastlio\|single_go2w_mujoco\|hil_orin_nano\|nav_test_hil_desktop" 2>/dev/null); do
  kill -9 "$pid" 2>/dev/null
done

# ── 2. Desktop: every ROS node executable, one substring at a time ─────
for pat in \
  mujoco_ros2_control mujoco_odom_bridge mujoco_contact rviz2 \
  laserMapping fast_lio fastlio cfpa2_single_robot cfpa2_to_nav2 \
  elevation_mapping grid_map_to filter_chain_runner \
  nav2_controller nav2_planner nav2_bt_navigator nav2_behaviors nav2_lifecycle \
  controller_server planner_server bt_navigator behavior_server lifecycle_manager \
  slam_odom_relay gt_odom_relay pointcloud_adapter pointcloud_to_laserscan \
  twist_bridge qos_bridge autonomy_enabler exploration_metrics path_relay \
  stuck_watchdog stuck_diagnoser trajectory_monitor wait_for_ready \
  stand_up_slowly initial_pose_guard spawn_entity quadruped_controller \
  go2_rl_sar_node go2_rl_policy_node go2_rl_policy go2_rl_sar \
  state_estimation robot_state_publisher ekf_node controller_manager spawner \
  static_transform_publisher cloud_world_offset trav_grid_diag \
  "install/go2w" "install/mujoco_sensor" "install/cfpa2" "install/elevation" \
  "install/trav_cost_filters" "install/fast_lio" ; do
  pkill -9 -f "$pat" 2>/dev/null
done

# ── 3. Desktop: PID-sweep any survivor under the workspace install ──────
for pid in $(ps -eo pid,args 2>/dev/null | grep -E "install/(go2w|mujoco|cfpa2|elevation|trav|fast_lio)|/opt/ros/humble/lib/(nav2|rviz)|ros2 launch go2" | grep -vE "draw_walls|kill_sim|grep" | awk '{print $1}'); do
  kill -9 "$pid" 2>/dev/null
done

sleep 3
ros2 daemon stop >/dev/null 2>&1 || true
rm -rf /dev/shm/fastrtps_* /dev/shm/fast_dds_* /dev/shm/*fastdds* 2>/dev/null
sleep 1

# ── 4. Cross-host: kill the Jetson HIL stack (the stale-publisher source) ─
if [ "${NO_JETSON:-0}" != "1" ]; then
  if ping -c1 -W2 "$JETSON_HOST" >/dev/null 2>&1 && command -v sshpass >/dev/null 2>&1; then
    echo "[kill_sim] cleaning Jetson ${JETSON_USER}@${JETSON_HOST}..."
    timeout 25 sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=accept-new \
      -o ConnectTimeout=6 "${JETSON_USER}@${JETSON_HOST}" '
        for pid in $(pgrep -f "orin_nano_hil_jetson|run_jetson_hil|jetson_ws/install|jetson_ws/scripts|grid_map_to_occupancy|elevation_mapping|cfpa2_single|controller_server|planner_server|bt_navigator|behavior_server|lifecycle_mana|fastlio_mapping|fast_lio_tf_adapter|filter_chain_runner|static_transform_publisher|cfpa2_to_nav2|path_relay|stuck_watchdog" 2>/dev/null); do
          kill -9 $pid 2>/dev/null
        done
        sleep 2
        echo "[jetson] remaining ROS procs: $(pgrep -af "jetson_ws|grid_map|elevation_mapping|cfpa2|nav2_|fastlio|filter_chain" 2>/dev/null | grep -v grep | grep -v sshd | wc -l)"
      ' 2>&1 | grep -i "jetson" || echo "[kill_sim] (Jetson ssh returned nothing — may already be clean)"
  else
    echo "[kill_sim] Jetson unreachable or no sshpass — skipping cross-host kill (set NO_JETSON=1 to silence)"
  fi
fi

# ── 5. Verify desktop clean ────────────────────────────────────────────
sleep 1
LEFT=$(ps -eo pid,args 2>/dev/null \
  | grep -E "install/(go2w|mujoco|cfpa2|elevation|trav|fast_lio)|/opt/ros/humble/lib/(nav2|rviz)|ros2 launch go2|laserMapping|mujoco_ros2_control" \
  | grep -vE "draw_walls|kill_sim|grep" | wc -l)
if [ "$LEFT" -eq 0 ]; then
  echo "[kill_sim] ✅ desktop clean"
else
  echo "[kill_sim] ⚠ $LEFT desktop survivors — killing by PID:"
  for pid in $(ps -eo pid,args 2>/dev/null | grep -E "install/(go2w|mujoco|cfpa2|elevation|trav|fast_lio)|/opt/ros/humble/lib/(nav2|rviz)|ros2 launch go2|laserMapping|mujoco_ros2_control" | grep -vE "draw_walls|kill_sim|grep" | awk '{print $1}'); do
    kill -9 "$pid" 2>/dev/null && echo "  killed $pid"
  done
fi
echo "[kill_sim] done."
