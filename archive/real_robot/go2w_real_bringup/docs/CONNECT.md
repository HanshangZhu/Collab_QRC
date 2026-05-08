#!/bin/bash
# Connect laptop to Unitree Go2W over WiFi and set up ROS2 DDS environment.
# Source this file in every terminal you want to use with the robot:
#   source REAL_GO2W_Connect

# ─────────────────────────────────────────────
# STEP 1: Connect to robot WiFi (run once per session)
# ─────────────────────────────────────────────
# nmcli device wifi rescan && sleep 2
# nmcli device wifi connect "Go2_21585" password "00000000"

# Verify connection:
# ping -c 3 192.168.12.1

# ─────────────────────────────────────────────
# STEP 2: Source ROS2 + DDS (source this file)
# ─────────────────────────────────────────────
source /opt/ros/humble/setup.bash

# Detect WiFi interface dynamically
WIFI_IFACE=$(iw dev 2>/dev/null | awk '/Interface/{print $2; exit}')
WIFI_IFACE=${WIFI_IFACE:-wlp0s20f3}

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="<CycloneDDS><Domain>
  <General>
    <Interfaces>
      <NetworkInterface name=\"${WIFI_IFACE}\" priority=\"default\" multicast=\"false\" />
    </Interfaces>
    <AllowMulticast>false</AllowMulticast>
  </General>
  <Discovery>
    <Peers>
      <Peer address=\"192.168.12.1\"/>
    </Peers>
    <ParticipantIndex>auto</ParticipantIndex>
  </Discovery>
</Domain></CycloneDDS>"
export ROS_DOMAIN_ID=0

echo "✅ ROS2 + CycloneDDS configured for Go2W WiFi (${WIFI_IFACE} → 192.168.12.1, unicast)"

# ─────────────────────────────────────────────
# VERIFY: After sourcing, check robot topics are visible
# ─────────────────────────────────────────────
# ros2 topic list
# ros2 topic echo /utlidar/robot_pose --once   # raw pose from robot (no driver needed)
# ros2 topic echo /joint_states --once         # needs go2w_driver running
