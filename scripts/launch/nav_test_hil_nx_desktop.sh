#!/usr/bin/env bash
# nav_test_hil_nx_desktop.sh — laptop half of the Orin NX HIL test bench.
#
# The laptop SIMULATES THE PHYSICAL WORLD; the Orin NX is a pure reactive
# compute unit running the full ROS 1 Noetic autonomy stack. Unlike the Orin
# NANO HIL (nav_test_hil_desktop.sh), here the NX runs SLAM too, so the laptop
# feeds RAW Mid-360 data (livox CustomMsg + IMU), NOT registered clouds.
#
# Laptop runs:  MuJoCo (ops2-v4 scene) + lidar/imu sim + pc2_to_livox converter
#               + IMU relay + CHAMP control (consumes /robot/cmd_vel from NX)
#               + RViz2 (hil_nx.rviz, viz topics bridged back from NX).
# Laptop does NOT run: SLAM / elevation / nav / CFPA2 — all on the NX.
#
# Sensor interface to the NX (carried by ros1_bridge on the NX side):
#   laptop → NX :  /livox/lidar (livox_ros_driver2/CustomMsg)  10 Hz
#                  /livox/imu   (sensor_msgs/Imu)              200 Hz
#   NX → laptop :  /robot/cmd_vel + viz (trav grid, cloud, odom, plan, tf)
#
# Companion (NX side): scripts/real/run_nx_hil_bridge.sh + onboard_autonomy_noetic.sh hil=true
# Orchestrator       : scripts/launch/hil_orin_nx.sh up
#
# Usage:
#   ./scripts/launch/nav_test_hil_nx_desktop.sh                 # handwalls scene
#   POLYFIT_VARIANT=nopoly ./scripts/launch/nav_test_hil_nx_desktop.sh
#   ./scripts/launch/nav_test_hil_nx_desktop.sh rviz:=false gui:=false

set -u -o pipefail

source "$(dirname "${BASH_SOURCE[0]}")/_preflight_kill.sh"
trap '' SIGHUP SIGTERM

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

# Scene variant (same set as nav_test_hil_desktop.sh).
POLYFIT_VARIANT="${POLYFIT_VARIANT:-handwalls}"
[[ "${NO_POLYFIT:-0}" = "1" ]] && POLYFIT_VARIANT="nopoly"
case "${POLYFIT_VARIANT}" in
  handwalls) SCENE="${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco/slam_ops2_v4_go2_handwalls.xml" ;;
  clustered) SCENE="${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco/slam_ops2_v4_go2_clustered.xml" ;;
  nopoly)    SCENE="${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco/slam_ops2_v4_go2_nopoly.xml" ;;
  real)      SCENE="${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco/slam_ops2_v4_go2_real.xml" ;;
  *) echo "unknown POLYFIT_VARIANT=${POLYFIT_VARIANT}" >&2; exit 1 ;;
esac

ROS2_SETUP_BASH="${ROS2_SETUP_BASH:-/opt/ros/humble/setup.bash}"
safe_source() { set +u; source "$1"; set -u; }
if [[ -f "${HOME}/miniforge3/etc/profile.d/conda.sh" ]]; then
  safe_source "${HOME}/miniforge3/etc/profile.d/conda.sh"; conda activate cmu_env
elif command -v micromamba >/dev/null 2>&1; then
  eval "$(micromamba shell hook -s bash)"; micromamba activate cmu_env
fi
safe_source "${ROS2_SETUP_BASH}"
safe_source "${WS_DIR}/install/setup.bash"

if [[ -n "${CONDA_PREFIX:-}" ]] && [[ -d "${CONDA_PREFIX}/lib/python3.10/site-packages/mujoco" ]]; then
  export LD_LIBRARY_PATH="${CONDA_PREFIX}/lib/python3.10/site-packages/mujoco:${LD_LIBRARY_PATH:-}"
fi
# CycloneDDS recommended for Foxy(NX bridge)↔Humble(laptop) cross-version viz.
# Override RMW_IMPLEMENTATION=rmw_cyclonedds_cpp + matching ROS_DOMAIN_ID on both
# sides if topics don't appear in RViz2. Default keeps the sim's no-shm FastDDS.
export FASTRTPS_DEFAULT_PROFILES_FILE="${WS_DIR}/config/fastdds_no_shm.xml"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export MUJOCO_INIT_KEYFRAME=home

ROBOT_NS="robot"
HIL_RVIZ_CONFIG="${HIL_RVIZ_CONFIG:-${WS_DIR}/src/go2w/go2_gazebo_sim/rviz/hil_nx.rviz}"
[[ -f "$HIL_RVIZ_CONFIG" ]] || HIL_RVIZ_CONFIG="$(ros2 pkg prefix go2_gazebo_sim 2>/dev/null)/share/go2_gazebo_sim/rviz/nav_test.rviz"

echo "=================================================================="
echo "  Orin NX HIL — LAPTOP side (simulated world)"
echo "    scene        : $SCENE"
echo "    sensors out  : /livox/lidar (CustomMsg) + /livox/imu  → NX"
echo "    consumes     : /${ROBOT_NS}/cmd_vel  ← NX (drives MuJoCo CHAMP)"
echo "    ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "    NX companion : scripts/real/run_nx_hil_bridge.sh + onboard_autonomy_noetic.sh hil=true"
echo "=================================================================="
echo ""

# 1) MuJoCo + sensors + CHAMP (NO SLAM, NO nav, NO cfpa2 — those are on the NX).
#    use_fast_lio:=false → laptop does NOT register clouds; we feed raw scan to
#    pc2_to_livox instead. odom_bridge_publish_tf:=false → NX SLAM owns map→base_link.
ros2 launch go2_gazebo_sim single_go2w_mujoco_cfpa2.launch.py \
  "mujoco_model_path:=${SCENE}" \
  "robot_namespace:=${ROBOT_NS}" \
  "use_sim_time:=true" \
  "spawn_x:=0.0" "spawn_y:=0.0" "spawn_yaw:=0.0" \
  "has_wheels:=false" \
  "enable_assets:=true" \
  "enable_perception:=true" \
  "enable_slam:=false" \
  "use_fast_lio:=false" \
  "enable_control:=true" \
  "enable_navigation:=false" \
  "odom_bridge_publish_tf:=false" \
  "cleanup_stale:=true" \
  "rviz_config:=${HIL_RVIZ_CONFIG}" \
  "$@" &
LAUNCH_PID=$!

# 2) pc2_to_livox: MuJoCo lidar (PointCloud2) → /livox/lidar (CustomMsg).
#    The MuJoCo lidar plugin publishes on /<ns>/mujoco_lidar_sensor/registered_scan
#    (BestEffort); a qos_bridge republishes it RELIABLE to
#    /<ns>/registered_scan_reliable. We consume the reliable one (matches our
#    SensorDataQoS sub + survives the UDP relay path deterministically).
#    Wall-clock vs sim-time: the converter copies the input header stamp; run with
#    use_sim_time so stamps match the sim clock.
sleep 6
SCAN_IN="${HIL_SCAN_TOPIC:-/${ROBOT_NS}/registered_scan_reliable}"
echo "  → starting pc2_to_livox (${SCAN_IN} → /livox/lidar)"
ros2 run pc2_to_livox pc2_to_livox_node --ros-args \
  -p use_sim_time:=true \
  -p input_topic:="${SCAN_IN}" \
  -p output_topic:=/livox/lidar \
  -p frame_id:=body &
PC2_PID=$!

# 3) IMU relay: MuJoCo /robot/imu/data → /livox/imu (what Point-LIO subscribes).
echo "  → starting IMU relay (/${ROBOT_NS}/imu/data → /livox/imu)"
ros2 run topic_tools relay /${ROBOT_NS}/imu/data /livox/imu &
IMU_PID=$!

# 4) UDP relay (replaces the broken Foxy ros1_bridge). The Jetson's Foxy
#    ros1_bridge bad_allocs on every message; our C++ UDP relay carries the
#    sensors DOWN to the NX and cmd_vel/viz back UP. See docs/claude/orin_nx_hil_design.md.
#    TX: /livox/{lidar,imu} → NX ports 9001/9002.  RX: NX cmd_vel/odom/trav → ROS2.
NX_IP="${NX_IP:-192.168.123.18}"
sleep 2
echo "  → starting UDP relay TX (/livox/* → ${NX_IP}:9001/9002)"
ros2 run hil_udp_relay hil_relay_tx_node --ros-args \
  -p nx_ip:="${NX_IP}" -p lidar_port:=9001 -p imu_port:=9002 &
RELAY_TX_PID=$!
echo "  → starting UDP relay RX (NX cmd_vel/odom/trav → ROS2 :9003/9004/9005)"
ros2 run hil_udp_relay hil_relay_rx_node --ros-args \
  -p cmd_vel_port:=9003 -p odom_port:=9004 -p trav_port:=9005 -p enable_viz:=true &
RELAY_RX_PID=$!

echo ""
echo "  Laptop sim up. Sensors → /livox/{lidar,imu} → UDP → NX."
echo "  cmd_vel/viz ← UDP ← NX. Now start the NX side: scripts/launch/hil_orin_nx.sh up"
echo "  Ctrl+C to tear down."

cleanup() {
  kill "$RELAY_TX_PID" "$RELAY_RX_PID" "$PC2_PID" "$IMU_PID" "$LAUNCH_PID" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup INT TERM EXIT
wait "$LAUNCH_PID"
