#!/usr/bin/env bash
# Preflight zombie killer — sourced from every nav/sim launch script.
#
# Why this exists: relaunching without killing the previous stack leaves
# stale publishers on /robot/map (TRANSIENT_LOCAL → latches old grid),
# /robot/registered_scan_map, /robot/way_point_coord, etc. Multiple
# localPlanner/pathFollower processes then race on /cmd_vel_stamped and
# the robot freezes.
#
# Usage (inside a launch wrapper):
#   source "$(dirname "${BASH_SOURCE[0]}")/_preflight_kill.sh"
#
# Set PREFLIGHT_KILL=0 to skip (e.g. if you want to attach debugger to a
# currently-running stack).

if [[ "${PREFLIGHT_KILL:-1}" == "0" ]]; then
  return 0 2>/dev/null || exit 0
fi

# Patterns cover: MuJoCo plugin, sim sensor bridges, CMU stack, SLAM, loop
# closure, exploration planners, CHAMP locomotion, EKF, TF publishers we
# spawn, and the ros2 launch processes themselves. NOTE: any new node
# spawned by launch files should be added here too — a surviving child
# leaves stale publishers on /robot/map, state_estimation_at_scan, etc.
_PREFLIGHT_PATTERNS=(
  "ros2 launch .*(nav_test|door_demo|vlm_demo|real_autonomy)"
  # Sim core
  "mujoco_ros2_control"
  "mujoco_sensor_bridge"
  "mujoco_odom_bridge"
  "mujoco_contact_node"
  "mujoco_lidar_node"
  # Frame + scan plumbing
  "sensor_scan_generation"
  "pointcloud_frame_bridge"
  "pointcloud_adapter"
  "qos_bridge"
  "twist_bridge"
  "slam_odom_relay"
  "stand_up_slowly"
  "go2w_hybrid_cmd_router"
  "multi_tf_relay"
  "__node:=map_to_odom_tf"
  "__node:=world_to_map_tf"
  "__node:=base_link_to_body_tf"
  "__node:=b_base_link_to_body_tf"
  "__node:=far_vehicle_tf"
  "__node:=far_camera_tf"
  # Mapping / SLAM / loop closure
  "octomap_server_node"
  "fastlio_mapping"
  "cartographer_node"
  "sc_pgo_node"
  # CMU stack
  "tare_planner_node"
  "far_planner"
  "far_status_adapter"
  "terrainAnalysis"
  "localPlanner"
  "pathFollower"
  "exploration_metrics_logger"
  # Nav / exploration
  "simple_scan_mapper"
  "cfpa2_"
  "astar_nav_node"
  "hybrid_astar_nav_node"
  "nav2_hybrid_astar_nav_node"
  "default_nav.py"
  # CHAMP locomotion + EKF
  "champ_base"
  "quadruped_controller_node"
  "state_estimation_node"
  "__node:=base_to_footprint_ekf"
  "__node:=footprint_to_odom_ekf"
  # Multi-robot map merge + collision monitor + bootstrap + augmenter
  "multirobot_map_merge"
  "map_merge "
  "bootstrap_map_merge_poses"
  "dual_robot_collision_monitor"
  "map_augmenter"
  "robot_self_filter"
)

_preflight_kill_patterns() {
  local sig="$1"
  local pat
  for pat in "${_PREFLIGHT_PATTERNS[@]}"; do
    pkill "${sig}" -f "${pat}" 2>/dev/null || true
  done
}

# Regex reused for pre/post-kill detection. Matches any process likely to
# hold DDS resources, GPU contexts, or a LiDAR plugin handle.
_PREFLIGHT_ALIVE_RE='mujoco_ros2_control|tare_planner_node|far_planner|localPlanner|pathFollower|sensor_scan_generation|champ_base|fastlio_mapping|octomap_server|cfpa2_coordinator_node|cartographer_node|sc_pgo_node|pointcloud_frame_bridge|pointcloud_adapter|qos_bridge|twist_bridge|slam_odom_relay|astar_nav_node|hybrid_astar_nav_node|nav2_hybrid_astar_nav_node|stand_up_slowly|go2w_hybrid_cmd_router|map_merge|map_augmenter|robot_self_filter|multi_tf_relay|dual_robot_collision_monitor|__node:=(world_to_map_tf|base_link_to_body_tf|b_base_link_to_body_tf|far_vehicle_tf|far_camera_tf|map_to_odom_tf)'

# Report any stuck processes (D = kernel I/O, Z = zombie waiting reap).
# D-state cannot be killed by SIGKILL — usually a wedged GPU/DDS syscall.
# Z-state means parent hasn't reaped; init adopts if parent is also gone.
_preflight_report_stuck() {
  local stuck
  # Only match our patterns; don't flag unrelated kernel threads.
  stuck=$(ps -eo pid,state,comm,args --no-headers 2>/dev/null \
            | awk -v re="${_PREFLIGHT_ALIVE_RE}" \
                  '($2 == "D" || $2 == "Z") && $0 ~ re {print}')
  if [[ -n "${stuck}" ]]; then
    echo "[preflight] STUCK processes (cannot be killed — wedged kernel syscall):"
    printf '  %s\n' "${stuck}"
    # Detect GPU type to suggest the right fd-release command.
    local gpu_hint=""
    if [[ -e /dev/nvidia0 ]] || [[ -e /dev/nvidiactl ]]; then
      gpu_hint="sudo fuser -k /dev/nvidia*"
    elif [[ -e /dev/dri/card0 ]] || [[ -e /dev/dri/renderD128 ]]; then
      gpu_hint="sudo fuser -k /dev/dri/*"
    fi
    if [[ -n "${gpu_hint}" ]]; then
      echo "[preflight] If relaunch fails, try releasing GPU fds: ${gpu_hint}"
    fi
    echo "[preflight] Then re-run this script. Last resort: reboot."
  fi
}

# Bounded `ros2 daemon stop` — hangs forever if daemon pipe is broken.
_preflight_stop_ros2_daemon() {
  local pid
  # Prefer direct kill of the daemon's process; `ros2 daemon stop` blocks
  # on a socket read to a daemon that's already half-dead.
  pid=$(pgrep -af "ros2cli.*daemon" 2>/dev/null | awk '{print $1}' | head -1)
  if [[ -n "${pid}" ]]; then
    kill -TERM "${pid}" 2>/dev/null || true
    sleep 0.3
    kill -KILL "${pid}" 2>/dev/null || true
  fi
}

# Clean DDS shared-memory files. FastRTPS leaves these after an abrupt exit;
# the next launch then sees "already-in-use" errors or silent discovery failures.
_preflight_clean_shm() {
  # Tolerate stat failures (files may have been removed by another process).
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
  # CycloneDDS uses /dev/shm/cdds_*; harmless if absent.
  rm -f /dev/shm/cdds_* /dev/shm/iox_* 2>/dev/null || true
  # Stale params files leak across runs; bootstrap_map_merge_poses' yaml is
  # consumed by map_merge unconditionally on bootstrap exit, so a stale file
  # silently misleads map_merge with prior-run init poses if bootstrap fails.
  rm -f /tmp/map_merge_params.yaml /tmp/dual_robot_collision_report.json 2>/dev/null || true
}

if pgrep -f "${_PREFLIGHT_ALIVE_RE}" >/dev/null 2>&1; then
  echo "[preflight] killing stale sim/nav processes..."
  _preflight_kill_patterns "-TERM"
  sleep 2
  _preflight_kill_patterns "-KILL"
  sleep 1
  _preflight_stop_ros2_daemon
  _preflight_clean_shm
  if pgrep -f "${_PREFLIGHT_ALIVE_RE}" >/dev/null 2>&1; then
    echo "[preflight] WARNING: some processes survived SIGKILL — check manually:"
    pgrep -af "${_PREFLIGHT_ALIVE_RE}" | head -20
    _preflight_report_stuck
  else
    echo "[preflight] clean."
  fi
else
  # Even on a clean system, nuke leftover DDS shm from a crashed previous run.
  _preflight_clean_shm
fi

unset _PREFLIGHT_PATTERNS _PREFLIGHT_ALIVE_RE
unset -f _preflight_kill_patterns _preflight_report_stuck
unset -f _preflight_stop_ros2_daemon _preflight_clean_shm
