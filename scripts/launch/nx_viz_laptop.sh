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
  pkill -9 -f "rviz2.*nx_viz" 2>/dev/null || true
  echo "nx_viz stopped."
  exit 0
fi
ENABLE_RVIZ="true"; [[ "${1:-}" == "rviz:=false" ]] && ENABLE_RVIZ="false"

safe_source() { set +u; source "$1"; set -u; }
if [[ -f "${HOME}/miniforge3/etc/profile.d/conda.sh" ]]; then
  safe_source "${HOME}/miniforge3/etc/profile.d/conda.sh"; conda activate cmu_env
elif command -v micromamba >/dev/null 2>&1; then
  eval "$(micromamba shell hook -s bash)"; micromamba activate cmu_env
fi
safe_source /opt/ros/humble/setup.bash
safe_source "${WS_DIR}/install/setup.bash"

echo "=================================================================="
echo "  Orin NX viz viewer (laptop side)"
echo "    receiving NX viz over UDP relay → /robot/{Odometry,traversability_grid,cmd_vel,...}"
echo "    NX must run: onboard_autonomy_noetic.sh viz_relay=true viz_laptop_ip=<this laptop>"
echo "=================================================================="

# Relay RX: bind the UP ports, republish NX viz on the laptop's ROS 2.
# trav grid is published transient_local+reliable (matches RViz2 map display).
ros2 run hil_udp_relay hil_relay_rx_node --ros-args \
  -p cmd_vel_port:=9003 -p odom_port:=9004 -p trav_port:=9005 -p enable_viz:=true \
  >/tmp/nx_viz_rx.log 2>&1 &
RX_PID=$!
echo "  relay RX up (log: /tmp/nx_viz_rx.log)"

if [[ "$ENABLE_RVIZ" == "true" ]]; then
  # Reuse the HIL rviz config if present, else the nav_test default.
  RVIZ_CFG="${WS_DIR}/src/go2w/go2_gazebo_sim/rviz/hil_nx.rviz"
  [[ -f "$RVIZ_CFG" ]] || RVIZ_CFG="$(ros2 pkg prefix go2_gazebo_sim 2>/dev/null)/share/go2_gazebo_sim/rviz/nav_test.rviz"
  echo "  launching RViz2 ($RVIZ_CFG)"
  rviz2 -d "$RVIZ_CFG" >/tmp/nx_viz_rviz.log 2>&1 &
  RVIZ_PID=$!
fi

echo "  Ctrl+C to stop."
cleanup() { kill "$RX_PID" "${RVIZ_PID:-}" 2>/dev/null || true; }
trap cleanup INT TERM EXIT
wait "$RX_PID"
