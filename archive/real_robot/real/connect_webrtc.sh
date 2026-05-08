#!/bin/bash
# connect_webrtc.sh — connect to Go2W WiFi and launch go2_ros2_sdk (WebRTC).
# No Docker, no Ethernet. Close the Unitree phone app first — WebRTC can't
# share the connection.
#
# Env overrides:
#   GO2W_WIFI_SSID    (default: Go2_21585)
#   GO2W_WIFI_PASS    (default: 00000000)
#   GO2W_WIFI_ROBOT   Robot IP on WiFi (default: 192.168.12.1)
#
# Usage:
#   ./connect_webrtc.sh                   # driver only
#   ./connect_webrtc.sh slam:=true rviz2:=true   # passed through to go2_robot_sdk

set -e
REPO_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." &> /dev/null && pwd )"

SSID="${GO2W_WIFI_SSID:-Go2_21585}"
PASSWORD="${GO2W_WIFI_PASS:-00000000}"
ROBOT_IP="${GO2W_WIFI_ROBOT:-192.168.12.1}"

echo "=== [1/2] Connecting to WiFi '$SSID' ==="
ros2 daemon stop 2>/dev/null || true
CURRENT=$(nmcli -t -f ACTIVE,SSID dev wifi | awk -F: '$1=="yes"{print $2}')
if [[ "$CURRENT" == "$SSID" ]]; then
  echo "  Already connected."
else
  nmcli device wifi rescan 2>/dev/null || true; sleep 2
  nmcli device wifi connect "$SSID" password "$PASSWORD"
fi
sleep 2

WIFI_IFACE=$(iw dev 2>/dev/null | awk '/Interface/{print $2; exit}')
WIFI_IFACE=${WIFI_IFACE:-wlp0s20f3}
MY_IP=$(ip -4 addr show "$WIFI_IFACE" 2>/dev/null | awk '/inet /{print $2}' | cut -d/ -f1)
echo "  Interface: $WIFI_IFACE   Host IP: $MY_IP"

if ! ping -c 1 -W 2 "$ROBOT_IP" &>/dev/null; then
  echo "ERROR: Cannot reach robot at $ROBOT_IP — is the robot powered?" >&2
  exit 1
fi
echo "  Robot reachable at $ROBOT_IP"

echo ""
echo "=== [2/2] Launching go2_ros2_sdk (WebRTC) ==="
echo "  Monitor in another terminal: scripts/real/monitor.sh"
echo "  Ctrl+C to stop"

source /opt/ros/humble/setup.bash
[[ -f "$REPO_ROOT/install/setup.bash" ]] && source "$REPO_ROOT/install/setup.bash"
export ROBOT_IP
export CONN_TYPE="webrtc"

ros2 launch go2_robot_sdk robot.launch.py \
  nav2:=false slam:=false rviz2:=false foxglove:=false joystick:=false teleop:=false \
  "$@"
