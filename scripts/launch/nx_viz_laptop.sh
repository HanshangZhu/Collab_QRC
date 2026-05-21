#!/usr/bin/env bash
# nx_viz_laptop.sh — laptop-side viewer for a REAL-robot Orin NX run.
#
# When the Go2 runs autonomy onboard the Orin NX with real sensors
# (onboard_autonomy_noetic.sh viz_relay=true), the NX streams its viz topics
# back over the C++ UDP relay. This script runs the laptop half: the relay RX
# (receives Odometry / traversability_grid / cmd_vel over UDP, republishes on
# the laptop's ROS 2) + RViz2 — so the operator watches the NX's SLAM map,
# trav grid, trajectory, and frontier goals live, WITHOUT the NX paying any
# DDS/ros1_bridge cost. This is the "complete validation" observation path; the
# HIL bench (nav_test_hil_nx_desktop.sh) is the same RX, just with MuJoCo as
# the world instead of the real building.
#
# Usage (laptop):  ./scripts/launch/nx_viz_laptop.sh            # RX + RViz2
#                  ./scripts/launch/nx_viz_laptop.sh rviz:=false # RX only
#                  ./scripts/launch/nx_viz_laptop.sh stop
set -u
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

if [[ "${1:-}" == "stop" ]]; then
  pkill -9 -f "hil_relay_rx_node" 2>/dev/null || true
  pkill -9 -f "hil_relay_tx_node" 2>/dev/null || true
  pkill -9 -f "odom_to_tf" 2>/dev/null || true
  pkill -9 -f "occupancy_grid_to_cloud.py" 2>/dev/null || true
  pkill -9 -f "pointstamped_to_marker.py" 2>/dev/null || true
  pkill -9 -f "posestamped_to_marker.py" 2>/dev/null || true
  pkill -9 -f "static_transform_publisher.*map.*camera_init" 2>/dev/null || true
  pkill -9 -f "rviz2.*nx_viz" 2>/dev/null || true
  echo "nx_viz stopped."
  exit 0
fi
ENABLE_RVIZ="true"; [[ "${1:-}" == "rviz:=false" ]] && ENABLE_RVIZ="false"
NX_IP="${NX_IP:-192.168.123.18}"

safe_source() { set +u; source "$1"; set -u; }
if [[ -f "${HOME}/miniforge3/etc/profile.d/conda.sh" ]]; then
  safe_source "${HOME}/miniforge3/etc/profile.d/conda.sh"; conda activate cmu_env
elif command -v micromamba >/dev/null 2>&1; then
  eval "$(micromamba shell hook -s bash)"; micromamba activate cmu_env
fi
safe_source /opt/ros/humble/setup.bash
safe_source "${WS_DIR}/install/setup.bash"

# If this script inherits a stale CycloneDDS file from another terminal, ROS 2
# nodes can abort before logging anything useful:
#   "<iface>: does not match an available interface."
# Keep a valid DDS env when present; drop only clearly stale interface bindings.
if [[ "${CYCLONEDDS_URI:-}" == file://* ]]; then
  _dds_file="${CYCLONEDDS_URI#file://}"
  if [[ -f "$_dds_file" ]]; then
    _dds_iface="$(sed -nE 's/.*<NetworkInterface name="([^"]+)".*/\1/p' "$_dds_file" | head -1)"
    if [[ -n "$_dds_iface" ]] && ! ip link show "$_dds_iface" &>/dev/null; then
      echo "  WARN: ignoring stale CYCLONEDDS_URI (${_dds_iface} is not present)."
      unset CYCLONEDDS_URI RMW_IMPLEMENTATION
    fi
  fi
fi

echo "=================================================================="
echo "  Orin NX viz viewer (laptop side)"
echo "    receiving NX viz over UDP relay → /robot/{Odometry,traversability_grid,planned_path,cmd_vel,...}"
echo "    NX must run: onboard_autonomy_noetic.sh viz_relay=true viz_laptop_ip=<this laptop>"
echo "=================================================================="

# Relay RX: bind the UP ports, republish NX viz on the laptop's ROS 2.
# trav grid is published transient_local+reliable (matches RViz2 map display).
ros2 run hil_udp_relay hil_relay_rx_node --ros-args \
  -p cmd_vel_port:=9003 -p odom_port:=9004 -p trav_port:=9005 \
  -p waypoint_port:=9006 -p plan_port:=9008 -p enable_viz:=true \
  >/tmp/nx_viz_rx.log 2>&1 &
RX_PID=$!
echo "  relay RX up (log: /tmp/nx_viz_rx.log)"

# Relay TX: forward RViz2's 2D Goal Pose (/goal_pose) to the NX ROS 1
# move_base goal receiver. The same tx node can also forward simulated Livox
# topics in HIL; during a real-robot viz session those topics are simply idle.
ros2 run hil_udp_relay hil_relay_tx_node --ros-args \
  -p nx_ip:="${NX_IP}" -p lidar_port:=9001 -p imu_port:=9002 -p goal_port:=9007 \
  >/tmp/nx_viz_tx.log 2>&1 &
TX_PID=$!
echo "  relay TX up (/goal_pose → ${NX_IP}:9007, log: /tmp/nx_viz_tx.log)"

# TF bridge: /robot/Odometry → TF (camera_init→body).
# The NX tx_node doesn't forward /tf; we reconstruct it from the Odometry pose.
python3 "${WS_DIR}/scripts/runtime/odom_to_tf.py" \
  >/tmp/nx_viz_odom_tf.log 2>&1 &
ODOM_TF_PID=$!
echo "  odom_to_tf bridge up (camera_init→body, log: /tmp/nx_viz_odom_tf.log)"

# Static TF: map→camera_init (identity).
# Point-LIO initialises camera_init at the origin; trav_grid is published in
# the 'map' frame by move_base. Publishing identity here aligns both frames so
# RViz2 can show trav grid and the robot pose in the same Fixed Frame (map).
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 map camera_init \
  >/tmp/nx_viz_static_tf.log 2>&1 &
STATIC_TF_PID=$!
echo "  static TF map→camera_init published (identity)"

# Debug visualization: RViz2's Map display can hit an OpenGL shader issue on
# some laptop runtimes. Publish known grid cells as PointCloud2 as a fallback.
python3 "${WS_DIR}/scripts/runtime/occupancy_grid_to_cloud.py" \
  >/tmp/nx_viz_grid_cloud.log 2>&1 &
GRID_CLOUD_PID=$!
echo "  traversability grid cloud bridge up (log: /tmp/nx_viz_grid_cloud.log)"

python3 "${WS_DIR}/scripts/runtime/pointstamped_to_marker.py" \
  >/tmp/nx_viz_frontier_marker.log 2>&1 &
FRONTIER_MARKER_PID=$!
echo "  frontier marker bridge up (log: /tmp/nx_viz_frontier_marker.log)"

python3 "${WS_DIR}/scripts/runtime/posestamped_to_marker.py" \
  >/tmp/nx_viz_manual_goal_marker.log 2>&1 &
MANUAL_GOAL_MARKER_PID=$!
echo "  manual 2D goal marker bridge up (log: /tmp/nx_viz_manual_goal_marker.log)"

if [[ "$ENABLE_RVIZ" == "true" ]]; then
  # Reuse the HIL rviz config if present, else the nav_test default.
  RVIZ_CFG="${WS_DIR}/src/go2w/go2_gazebo_sim/rviz/hil_nx.rviz"
  [[ -f "$RVIZ_CFG" ]] || RVIZ_CFG="$(ros2 pkg prefix go2_gazebo_sim 2>/dev/null)/share/go2_gazebo_sim/rviz/nav_test.rviz"
  echo "  launching RViz2 ($RVIZ_CFG)"
  # VS Code launched from Snap exports SNAP/GIO/GTK paths that can make RViz2
  # load Snap's core20 libc/pthread and crash with a symbol lookup error.
  env -u SNAP -u SNAP_NAME -u SNAP_INSTANCE_NAME -u SNAP_REVISION -u SNAP_ARCH \
      -u SNAP_LIBRARY_PATH -u SNAP_DATA -u SNAP_COMMON -u SNAP_USER_DATA \
      -u SNAP_USER_COMMON -u SNAP_REAL_HOME -u SNAP_CONTEXT -u SNAP_COOKIE \
      -u SNAP_EUID -u SNAP_UID -u SNAP_LAUNCHER_ARCH_TRIPLET -u SNAP_VERSION \
      -u GTK_PATH -u GIO_MODULE_DIR -u LOCPATH \
      QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}" \
      LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}" \
      rviz2 -d "$RVIZ_CFG" >/tmp/nx_viz_rviz.log 2>&1 &
  RVIZ_PID=$!
fi

echo "  Ctrl+C to stop."
cleanup() { kill "$RX_PID" "${TX_PID:-}" "${ODOM_TF_PID:-}" "${STATIC_TF_PID:-}" "${GRID_CLOUD_PID:-}" "${FRONTIER_MARKER_PID:-}" "${MANUAL_GOAL_MARKER_PID:-}" "${RVIZ_PID:-}" 2>/dev/null || true; }
trap cleanup INT TERM EXIT
wait "$RX_PID"
