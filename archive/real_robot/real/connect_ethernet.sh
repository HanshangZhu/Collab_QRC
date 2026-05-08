#!/bin/bash
# connect_ethernet.sh — Ethernet + CycloneDDS setup for real Unitree Go2W / Go2.
#
# Two modes:
#   1. Sourced from another script     → exposes `ensure_link` and
#                                         `setup_cyclonedds_ethernet` functions.
#   2. Run directly                     → runs a CLI subcommand (see below).
#
# CLI subcommands (standalone mode):
#   ./connect_ethernet.sh                 full: link preflight + DDS env print
#   ./connect_ethernet.sh link            link preflight only (no DDS)
#   ./connect_ethernet.sh dds             DDS env setup only (no link check)
#   ./connect_ethernet.sh doctor          diagnostic info (iface, IPs, robot ping)
#   ./connect_ethernet.sh lidar           detect Livox Mid-360 vs Unitree L1
#
# Env overrides (all optional):
#   GO2W_ETH_IFACE   USB-C dongle interface name   default: enxc8a36240a4c7
#   GO2W_ETH_IP      Robot IP on the subnet        default: 192.168.123.161
#   GO2W_HOST_IP     Host IP on the subnet         default: 192.168.123.100
#   GO2W_ETH_SUBNET  Subnet prefix length          default: 24
#   GO2W_MID360_IP            Primary Mid-360 IP    default: 192.168.123.20
#                    (Verified on this unit 2026-04-17 via ARP + TCP-absence
#                    fingerprint. Unitree EDU+ factory-integrated units
#                    usually sit at 192.168.123.120; factory-fresh Livox
#                    units default to 192.168.1.1XX where XX = serial last
#                    two digits and are unreachable from the laptop without
#                    re-IP via Livox Viewer.)
#   GO2W_MID360_IP_CANDIDATES Fallback IP list      default: ".20 .120 (1.120)"
#                    (detect_lidar probes the primary first, then each
#                    candidate in order until one responds.)
#   GO2W_LIDAR_PROBE_TIMEOUT  Per-IP ping timeout   default: 2 seconds
#
# Note: Go2W and Go2 both ship with the same default IP; same script.

set -e

# ═══════════════════════════════════════════════════════════════════════
# Configuration — defaults, overridable via env
# ═══════════════════════════════════════════════════════════════════════

REPO_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." &> /dev/null && pwd )"

ETH_IFACE="${GO2W_ETH_IFACE:-enxc8a36240a4c7}"
ROBOT_IP="${GO2W_ETH_IP:-192.168.123.161}"
HOST_IP="${GO2W_HOST_IP:-192.168.123.100}"
SUBNET="${GO2W_ETH_SUBNET:-24}"
MID360_IP="${GO2W_MID360_IP:-192.168.123.20}"      # this unit's Mid-360 (2026-04-17)
MID360_IP_CANDIDATES="${GO2W_MID360_IP_CANDIDATES:-192.168.123.20 192.168.123.120 192.168.1.120}"
LIDAR_PROBE_TIMEOUT="${GO2W_LIDAR_PROBE_TIMEOUT:-2}"


# ═══════════════════════════════════════════════════════════════════════
# Public functions — safe to source
# ═══════════════════════════════════════════════════════════════════════

# setup_cyclonedds_ethernet
#   Sources ROS 2 + workspace, then exports CycloneDDS URI that binds the
#   given interface, peers the robot IP, and sets ROS_DOMAIN_ID=0. Stops
#   any running ros2 daemon so the new env takes effect.
setup_cyclonedds_ethernet() {
  source /opt/ros/humble/setup.bash
  [[ -f "$REPO_ROOT/install/setup.bash" ]] && source "$REPO_ROOT/install/setup.bash"

  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export ROS_DOMAIN_ID=0
  export CONN_TYPE="cyclonedds"

  # Build peer list. Always peer the Go2 main controller. When onboard SLAM
  # is in use (real_autonomy.sh's onboard=true flag → exports ONBOARD_SLAM=1),
  # also peer the Jetson at 192.168.123.18 so DDS discovers /Odometry +
  # /cloud_registered_body published from there.
  local _peers="<Peer address=\"${ROBOT_IP}\"/>"
  if [[ "${ONBOARD_SLAM:-0}" == "1" ]]; then
    local _jetson_ip="${GO2W_JETSON_IP:-192.168.123.18}"
    _peers+="<Peer address=\"${_jetson_ip}\"/>"
    echo "  CycloneDDS peer added: ${_jetson_ip} (onboard SLAM)"
  fi

  # 2026-05-01: switched from inline multi-line XML to file:// URI.
  # ros2 launch was truncating the multi-line CYCLONEDDS_URI env var when
  # spawning child node processes — the parent had the full XML, but every
  # child saw only `<CycloneDDS><Domain></Domain></CycloneDDS>`. Result:
  # children defaulted to multicast-only discovery on a network where the
  # USB-Ethernet dongle's multicast doesn't reach the Go2 — Sport API
  # subscription invisible, /api/sport/request had 0 subscribers, robot
  # didn't move despite cmd_vel flowing through the laptop side. file://
  # URI sidesteps the issue entirely (single-line env value, all children
  # inherit cleanly).
  local _xml="/tmp/cyclonedds_${USER}_eth.xml"
  cat > "${_xml}" <<EOF
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="${ETH_IFACE}" priority="default" multicast="true" />
      </Interfaces>
    </General>
    <Discovery>
      <Peers>${_peers}</Peers>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF
  export CYCLONEDDS_URI="file://${_xml}"

  (ros2 daemon stop &>/dev/null &); sleep 1
  pkill -9 -f _ros2_daemon 2>/dev/null || true
}

# ensure_link
#   Verifies the dongle is present, ensures HOST_IP/SUBNET is on the
#   interface (as primary OR secondary), and pings the robot.
#
#   Livox Mid-360 needs a stable laptop IP to send UDP to — the host IP
#   in MID360_config.json (default 192.168.123.100) must actually be
#   bound to our interface or the driver gets `bind failed`. NetworkManager
#   often auto-assigns a different IP (e.g. 192.168.123.222), so we
#   idempotently add 192.168.123.100 as a SECONDARY when missing; both
#   IPs coexist on the same subnet fine.
#
#   Non-zero exit on any failure so callers can abort.
ensure_link() {
  if ! ip link show "$ETH_IFACE" &>/dev/null; then
    echo "ERROR: Interface $ETH_IFACE not found." >&2
    echo "  Is the USB-C Ethernet dongle plugged in?" >&2
    echo "  Override with GO2W_ETH_IFACE=<iface>" >&2
    return 1
  fi

  # If the interface has no IPv4 at all, bring it up and assign primary.
  local has_any_ip
  has_any_ip=$(ip -4 addr show "$ETH_IFACE" 2>/dev/null | awk '/inet /{print $2}' | head -1)
  if [[ -z "$has_any_ip" ]]; then
    echo "  Assigning $HOST_IP/$SUBNET to $ETH_IFACE (primary)..."
    sudo ip addr add "$HOST_IP/$SUBNET" dev "$ETH_IFACE" 2>/dev/null || true
    sudo ip link set "$ETH_IFACE" up
    sleep 1
  fi

  # Always ensure HOST_IP is present (primary OR secondary). Needed for
  # Livox driver bind() to succeed when the Mid-360 points back at HOST_IP.
  if ! ip -4 addr show "$ETH_IFACE" | grep -q " $HOST_IP/"; then
    echo "  Adding $HOST_IP/$SUBNET as secondary on $ETH_IFACE (Livox bind)..."
    sudo ip addr add "$HOST_IP/$SUBNET" dev "$ETH_IFACE" 2>/dev/null || true
  fi

  # Retry the reachability probe — covers two real-world races:
  #   1. Cable just plugged / dongle-on-dongle handoff: link state is "up"
  #      but ARP / spanning-tree hasn't settled, first ping drops.
  #   2. Robot just powered: networking comes up over a few seconds.
  # Override the wait via ROBOT_PING_RETRIES (default 8 ≈ 24 s wall time).
  local _retries="${ROBOT_PING_RETRIES:-8}"
  local _attempt=1
  while ! ping -c 2 -W 2 "$ROBOT_IP" &>/dev/null; do
    if (( _attempt >= _retries )); then
      echo "ERROR: Cannot reach robot at $ROBOT_IP after ${_retries} attempts" >&2
      echo "       (override with ROBOT_PING_RETRIES=N)" >&2
      return 1
    fi
    echo "  ping ${_attempt}/${_retries} to $ROBOT_IP failed — retrying in 1 s..." >&2
    sleep 1
    _attempt=$((_attempt + 1))
  done
  echo "  Ethernet link OK  ($ETH_IFACE → $ROBOT_IP, host bound @ $HOST_IP$([ "$_attempt" -gt 1 ] && echo " — settled after $_attempt attempts"))"
}

# detect_lidar
#   Network probe: echoes "mid360" if a Livox Mid-360 is found on the Go2
#   subnet within LIDAR_PROBE_TIMEOUT seconds per IP, otherwise "l1".
#
#   Method: ping the primary MID360_IP first. If it misses, try each
#   MID360_IP_CANDIDATES in order. A match on any of them confirms
#   Mid-360 presence.
#
#   Ping is ~99% reliable for presence — Livox devices answer ICMP once the
#   IP is configured and ~3 s boot completes. The "authoritative" UDP
#   0x0000 device-info query per Livox protocol 2.0 needs CRC-16 + CRC-32
#   framing (fragile in shell).
#
#   Proxy-ARP caveat: the Go2 dev board answers ARP for every IP in
#   192.168.123.0/24 — but only real devices have a unique MAC in the
#   neighbor table. Ping reliably distinguishes because the Go2 doesn't
#   forge ICMP replies.
#
#   On success sets DETECTED_LIDAR_IP so callers can log which IP
#   responded. Always exits 0.
detect_lidar() {
  local timeout_sec="${1:-$LIDAR_PROBE_TIMEOUT}"
  local ip
  for ip in $MID360_IP $MID360_IP_CANDIDATES; do
    if ping -c 1 -W "$timeout_sec" "$ip" &>/dev/null; then
      DETECTED_LIDAR_IP="$ip"
      echo "mid360"
      return 0
    fi
  done
  DETECTED_LIDAR_IP=""
  echo "l1"
}


# ═══════════════════════════════════════════════════════════════════════
# CLI subcommands — only reached when run directly
# ═══════════════════════════════════════════════════════════════════════

_print_config() {
  echo "  ETH_IFACE       = $ETH_IFACE"
  echo "  ROBOT_IP        = $ROBOT_IP"
  echo "  HOST_IP         = $HOST_IP"
  echo "  SUBNET          = /$SUBNET"
  echo "  MID360_IP       = $MID360_IP"
  echo "  MID360 FALLBACKS= $MID360_IP_CANDIDATES"
  echo "  LIDAR_TIMEOUT   = ${LIDAR_PROBE_TIMEOUT}s per IP"
}

_doctor() {
  echo "=== Ethernet connection diagnostic ==="
  _print_config
  echo ""
  echo "--- Interface state ---"
  ip link show "$ETH_IFACE" 2>&1 | sed 's/^/  /' || true
  echo ""
  echo "--- IPv4 addrs on $ETH_IFACE ---"
  ip -4 addr show "$ETH_IFACE" 2>&1 | awk '/inet /{print "  "$0}' || true
  echo ""
  echo "--- Robot ping (2 pkts) ---"
  ping -c 2 -W 2 "$ROBOT_IP" 2>&1 | sed 's/^/  /' || true
  echo ""
  echo "--- ROS 2 env ---"
  echo "  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}"
  echo "  CONN_TYPE=${CONN_TYPE:-unset}"
  echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
}

_cli_main() {
  local cmd="${1:-full}"
  case "$cmd" in
    full)
      echo "=== Ethernet setup ($ETH_IFACE) ==="
      _print_config
      echo ""
      ensure_link
      setup_cyclonedds_ethernet
      echo "=== CycloneDDS env exported (CONN_TYPE=cyclonedds) ==="
      ;;
    link)
      _print_config
      ensure_link
      ;;
    dds)
      _print_config
      setup_cyclonedds_ethernet
      echo "CycloneDDS env exported."
      ;;
    doctor)
      _doctor
      ;;
    lidar)
      echo "=== LiDAR detection ==="
      _print_config
      echo ""
      echo "  Probing candidates (primary first):"
      local detected=""
      for ip in $MID360_IP $MID360_IP_CANDIDATES; do
        printf "    %-17s " "$ip"
        if ping -c 1 -W "$LIDAR_PROBE_TIMEOUT" "$ip" &>/dev/null; then
          echo "responded"
          detected="mid360"
          DETECTED_LIDAR_IP="$ip"
          break
        else
          echo "—"
        fi
      done
      echo ""
      if [[ "$detected" == "mid360" ]]; then
        echo "  Detected LiDAR : mid360 @ $DETECTED_LIDAR_IP"
        echo "  Auto SLAM      : fastlio_mid360 (will use livox_ros_driver2)"
      else
        echo "  Detected LiDAR : l1 (Unitree built-in /utlidar/cloud)"
        echo "  Auto SLAM      : carto_l1"
      fi
      ;;
    *)
      echo "Usage: $0 [full|link|dds|doctor|lidar]" >&2
      echo "  full    — link preflight + DDS setup (default)" >&2
      echo "  link    — link preflight only" >&2
      echo "  dds     — DDS env setup only" >&2
      echo "  doctor  — diagnostic dump, no changes" >&2
      echo "  lidar   — probe for Livox Mid-360 vs Unitree L1 (prints result)" >&2
      exit 2
      ;;
  esac
}

# Only run CLI when invoked directly — safe to source.
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  _cli_main "$@"
fi
