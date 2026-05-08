#!/bin/bash
# monitor.sh — Go2W topic monitor + optional cmd_vel publisher.
# Works with Ethernet/CycloneDDS and WebRTC connections.
#
# Usage:
#   ./monitor.sh                         # watch topics (auto-detect mode)
#   ./monitor.sh 0.3                     # watch + publish vx=0.3
#   ./monitor.sh 0.3 0.5                 # watch + publish vx=0.3 wz=0.5
#   ./monitor.sh stop                    # publish zero velocity
#   GO2W_DDS=1 ./monitor.sh              # force Ethernet/DDS mode
#   GO2W_DDS=1 ./monitor.sh topics       # list robot topics
#   GO2W_DDS=1 ./monitor.sh pc           # stream /utlidar/cloud rate

source /opt/ros/humble/setup.bash
REPO_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." &> /dev/null && pwd )"
[[ -f "$REPO_ROOT/install/setup.bash" ]] && source "$REPO_ROOT/install/setup.bash"

ETH_IFACE="${GO2W_ETH_IFACE:-enxc8a36240a4c7}"
ROBOT_IP="${GO2W_ETH_IP:-192.168.123.161}"

if [[ "${CONN_TYPE:-}" == "cyclonedds" || "${GO2W_DDS:-}" == "1" ]]; then
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI="<CycloneDDS><Domain>
    <General>
      <Interfaces>
        <NetworkInterface name=\"${ETH_IFACE}\" priority=\"default\" multicast=\"true\" />
      </Interfaces>
    </General>
    <Discovery>
      <Peers><Peer address=\"${ROBOT_IP}\"/></Peers>
      <ParticipantIndex>auto</ParticipantIndex>
    </Discovery>
  </Domain></CycloneDDS>"
  export ROS_DOMAIN_ID=0
  DDS_MODE=1
  echo "📡 Ethernet/DDS mode → $ETH_IFACE → $ROBOT_IP"
fi
ros2 daemon stop 2>/dev/null || true

ARG1="${1:-}"
WZ="${2:-0.0}"

if [[ "$ARG1" == "stop" ]]; then
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
  echo "Stopped."; exit 0
fi

if [[ "$ARG1" == "topics" ]]; then
  echo "=== All robot topics ==="
  timeout 8 ros2 topic list 2>/dev/null
  exit 0
fi

if [[ "$ARG1" == "pc" ]]; then
  echo "=== /utlidar/cloud ==="; timeout 10 ros2 topic hz /utlidar/cloud 2>/dev/null &
  echo "=== /utlidar/cloud_deskewed ==="; timeout 10 ros2 topic hz /utlidar/cloud_deskewed 2>/dev/null &
  wait; exit 0
fi

# ── Optional cmd_vel publisher ────────────────────────────────────────
if [[ -n "$ARG1" ]]; then
  VX="$ARG1"
  echo "Publishing cmd_vel: vx=$VX wz=$WZ (10 Hz)"
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: $VX}, angular: {z: $WZ}}" --rate 10 &
  PUB_PID=$!
  trap "kill $PUB_PID 2>/dev/null; ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'; exit" INT TERM
fi

# ── Watch topics ─────────────────────────────────────────────────────
if [[ "${DDS_MODE:-}" == "1" ]]; then
  echo "=== Monitoring (Ethernet/DDS) — Ctrl+C to stop ==="
  ros2 topic echo /utlidar/robot_pose --field pose.position --field pose.orientation &
  ros2 topic hz /utlidar/cloud &
  ros2 topic echo /cmd_vel 2>/dev/null &
  wait
else
  echo "=== Monitoring (WebRTC) — Ctrl+C to stop ==="
  ros2 topic echo /cmd_vel &
  ros2 topic echo /joint_states --field name --field position &
  wait
fi
