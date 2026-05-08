#!/bin/bash
set -e

# Source ROS2 base
source /opt/ros/humble/setup.bash

# Source our workspace overlay
source /ws/install/setup.bash

# If WIFI_IFACE is set, rewrite the CycloneDDS config dynamically
if [ -n "${WIFI_IFACE}" ]; then
  cat > /ws/cyclonedds_wifi.xml <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="${WIFI_IFACE}" priority="default" multicast="false" />
      </Interfaces>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <Peer address="192.168.12.1"/>
      </Peers>
      <ParticipantIndex>auto</ParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF
  echo "[entrypoint] CycloneDDS bound to interface: ${WIFI_IFACE} (unicast → 192.168.12.1)"
fi

# Execute the CMD
exec "$@"
