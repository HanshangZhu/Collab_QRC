#!/usr/bin/env bash
# onboard_slam.sh — bring up Fast-LIO + livox_ros_driver2 (+ optional SC-PGO)
# on the Go2 Jetson, replacing the laptop's slam.launch.py SLAM block.
#
# What this brings up (per crispy-stargazing-kurzweil plan, Phase 3 step 5):
#   1. Mid-360 NIC bind (192.168.123.100 on the Jetson's Ethernet, required by
#      MID360_config.json — without it Livox driver bind() fails)
#   2. CycloneDDS env (ROS_DOMAIN_ID=0, peer = laptop IP)
#   3. livox_ros_driver2_node     → /livox/lidar + /livox/imu
#   4. fastlio_mapping            → /Odometry + /cloud_registered + /cloud_registered_body
#   5. Static TFs:                  map → camera_init, body → base_link, map → odom
#   6. fast_lio_tf_adapter         → /<ns>/odom/nav (consumed by laptop nav stack)
#   7. (optional) sc_pgo_node      → /<ns>/corrected_odom (loop closure)
#
# What stays on the laptop:
#   - Nav2 / costmap / behaviors / bt_navigator / lifecycle_manager
#   - CFPA2, octomap_server, RViz, supervisor_panic, cmd_vel mux + Sport bridge
#
# Usage (run on Jetson):
#   ./onboard_slam.sh                            # default: foxy, no sc_pgo
#   ./onboard_slam.sh laptop_ip=192.168.123.222
#   ./onboard_slam.sh sc_pgo=true                # enable loop closure (post-port)
#   ./onboard_slam.sh stop                       # tear down everything
#
# Ctrl+C exits cleanly via trap.

set -e
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WS_ROOT="$( cd "$SCRIPT_DIR/.." &> /dev/null && pwd )"   # /home/unitree/onboard_ws

# ── Defaults ─────────────────────────────────────────────────────────
ROS_DISTRO="foxy"
NAMESPACE="robot"
LAPTOP_IP="192.168.123.222"     # laptop's Ethernet IP (per real_robot.md table)
LIVOX_HOST_IP="192.168.123.100" # Mid-360 expects this on the host NIC
LIVOX_NIC=""                    # auto-detect if empty
DOMAIN_ID="0"
ENABLE_SC_PGO="false"
ENABLE_FAST_LIO_TF="true"

# ── Cleanup ──────────────────────────────────────────────────────────
_kill_onboard_stack() {
  for p in livox_ros_driver2_node fastlio_mapping fast_lio_tf_adapter \
           sc_pgo_node static_transform_publisher; do
    pkill -9 -f "$p" 2>/dev/null || true
  done
  (ros2 daemon stop &>/dev/null &); sleep 1
  pkill -9 -f _ros2_daemon 2>/dev/null || true
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
}

# ── Parse args ───────────────────────────────────────────────────────
for arg in "$@"; do
  case "$arg" in
    stop)
      echo "Stopping onboard SLAM..."
      _kill_onboard_stack
      echo "Done."
      exit 0
      ;;
    laptop_ip=*)     LAPTOP_IP="${arg#laptop_ip=}" ;;
    livox_host=*)    LIVOX_HOST_IP="${arg#livox_host=}" ;;
    nic=*)           LIVOX_NIC="${arg#nic=}" ;;
    namespace=*|ns=*) NAMESPACE="${arg#*=}" ;;
    domain=*)        DOMAIN_ID="${arg#domain=}" ;;
    distro=*)        ROS_DISTRO="${arg#distro=}" ;;
    sc_pgo=*)        ENABLE_SC_PGO="${arg#sc_pgo=}" ;;
    fast_lio_tf=*)   ENABLE_FAST_LIO_TF="${arg#fast_lio_tf=}" ;;
    *) echo "WARN: unknown arg '$arg'" >&2 ;;
  esac
done

# ── Validate ─────────────────────────────────────────────────────────
case "$ENABLE_SC_PGO"      in true|false) ;; *) echo "ERROR: sc_pgo must be true|false" >&2; exit 1 ;; esac
case "$ENABLE_FAST_LIO_TF" in true|false) ;; *) echo "ERROR: fast_lio_tf must be true|false" >&2; exit 1 ;; esac
case "$ROS_DISTRO"         in foxy|humble) ;; *) echo "ERROR: distro must be foxy|humble" >&2; exit 1 ;; esac

# ── Mid-360 NIC bind ─────────────────────────────────────────────────
# Without 192.168.123.100 on the host's Ethernet, Livox driver bind() fails.
# This is the laptop's responsibility today (connect_ethernet.sh:114-119);
# now the Jetson must own it when SLAM moves onboard.
if [[ -z "$LIVOX_NIC" ]]; then
  LIVOX_NIC="$(ip -o link show | awk -F': ' '$2 ~ /^en|^eth/ {print $2; exit}')"
fi
if [[ -z "$LIVOX_NIC" ]]; then
  echo "ERROR: could not auto-detect Ethernet NIC. Pass nic=eth0 (or similar)." >&2
  exit 1
fi
echo "  Ethernet NIC: $LIVOX_NIC"

if ! ip -4 addr show "$LIVOX_NIC" | grep -q " ${LIVOX_HOST_IP}/"; then
  echo "  Adding ${LIVOX_HOST_IP}/24 as secondary on $LIVOX_NIC (Mid-360 bind)..."
  # When this script runs under nohup, stdin is closed so `sudo -S` can't
  # read a password from the pipe. Caller is expected to pre-cache sudo
  # via `echo PASSWORD | sudo -S true` before invoking the script — or
  # to set up passwordless sudo for this command. We try -n (non-
  # interactive); if it fails the user gets a clear error.
  sudo -n ip addr add "${LIVOX_HOST_IP}/24" dev "$LIVOX_NIC" 2>/dev/null || \
    sudo ip addr add "${LIVOX_HOST_IP}/24" dev "$LIVOX_NIC" 2>/dev/null || {
    echo "  WARN: could not add ${LIVOX_HOST_IP}. Mid-360 bind() will fail." >&2
    echo "        Run: sudo ip addr add ${LIVOX_HOST_IP}/24 dev $LIVOX_NIC" >&2
  }
fi
# Verify it landed (defensive — sudo can silently fail under nohup)
if ! ip -4 addr show "$LIVOX_NIC" | grep -q " ${LIVOX_HOST_IP}/"; then
  echo "  ERROR: ${LIVOX_HOST_IP} still missing on ${LIVOX_NIC}." >&2
  echo "         Run BEFORE this script:" >&2
  echo "         sudo ip addr add ${LIVOX_HOST_IP}/24 dev ${LIVOX_NIC}" >&2
  exit 1
fi

# ── Verify Mid-360 reachable ─────────────────────────────────────────
if ! ping -c 1 -W 2 192.168.123.20 &>/dev/null; then
  echo "WARN: Mid-360 at 192.168.123.20 didn't answer ping. Driver may fail." >&2
fi

# ── ROS env ──────────────────────────────────────────────────────────
source "/opt/ros/${ROS_DISTRO}/setup.bash"
[[ -f "$WS_ROOT/install/setup.bash" ]] && source "$WS_ROOT/install/setup.bash"

# Livox driver's own colcon ws (built by ./build.sh ROS2 inside src/livox_ros_driver2/)
[[ -f "$WS_ROOT/src/livox_ros_driver2/install/setup.bash" ]] && \
  source "$WS_ROOT/src/livox_ros_driver2/install/setup.bash"

export ROS_DOMAIN_ID="$DOMAIN_ID"

# Foxy on this Tegra Jetson has a broken shared-memory transport in BOTH
# FastDDS and CycloneDDS — observed symptom is endless `bad_alloc caught:
# std::bad_alloc` spam, plus topic discovery fails between nodes (publishers
# work but `ros2 topic list` returns nothing). Force UDP-only via a FastDDS
# profile and skip CycloneDDS entirely on Foxy. Humble's DDS is fine, so the
# laptop's connect_ethernet.sh (Cyclone) cross-talks with FastDDS-Foxy here
# via the standard wire protocol — DDS is interoperable across RMWs.
if [[ "$ROS_DISTRO" == "foxy" ]]; then
  # Leave RMW_IMPLEMENTATION UNSET — Foxy defaults are tested working with
  # the no-shm profile below; explicitly setting rmw_fastrtps_cpp produces
  # endless `bad_alloc caught: std::bad_alloc` and processes silently exit.
  unset RMW_IMPLEMENTATION
  unset CYCLONEDDS_URI

  # ── Foxy/aarch64 vmem cap — THE fix for the OOM cascade ──
  # On this Tegra Jetson, every Foxy rclcpp+FastDDS process speculatively
  # mmap's a single ~14.5 GB anonymous region at startup (probably an
  # internal pool sized from /proc/meminfo). Successful allocation gets
  # lazy-committed as threads touch it → 13 GB resident per node → 5
  # nodes × 13 GB = OOM.
  # Capping virtual address space to 1.5 GB makes the speculative
  # allocation fail; the bad_alloc gets caught by rclcpp's signal handler
  # (the "bad_alloc caught: std::bad_alloc" log spam — harmless), the
  # library falls back to bounded buffers, and the process runs in ~25 MB.
  # All 5 SLAM nodes fit in <250 MB total RSS with this single env line.
  # Verified empirically 2026-04-30. Do NOT remove without re-testing.
  ulimit -v 1500000

  mkdir -p /tmp/dds_cfg
  cat > /tmp/dds_cfg/fastdds_no_shm.xml <<'XML'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_only</transport_id>
      <type>UDPv4</type>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="participant_profile" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>udp_only</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
XML
  export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/dds_cfg/fastdds_no_shm.xml
else
  export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
fi

# Inline CycloneDDS XML — peer = laptop. Only set when CycloneDDS is the
# active RMW (Humble path). On Foxy this Jetson uses FastDDS UDP-only
# (see fastdds_no_shm.xml above) because Foxy's cyclonedds segfaults
# during participant init on Tegra; the FastDDS shim talks to the
# laptop's CycloneDDS over the wire transparently (DDS RTPS is
# interoperable across implementations).
if [[ "$RMW_IMPLEMENTATION" == "rmw_cyclonedds_cpp" ]]; then
  export CYCLONEDDS_URI="<CycloneDDS><Domain>
    <General>
      <Interfaces>
        <NetworkInterface name=\"${LIVOX_NIC}\" priority=\"default\" multicast=\"true\" />
      </Interfaces>
    </General>
    <Discovery>
      <Peers><Peer address=\"${LAPTOP_IP}\"/></Peers>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>
    </Discovery>
  </Domain></CycloneDDS>"
fi

# ── Banner ───────────────────────────────────────────────────────────
echo ""
echo "################################################"
echo "  Onboard SLAM (Jetson @ $(hostname))"
echo "    distro      : $ROS_DISTRO"
echo "    namespace   : $NAMESPACE  (TF + topics use this)"
echo "    laptop peer : $LAPTOP_IP"
echo "    livox host  : $LIVOX_HOST_IP on $LIVOX_NIC"
echo "    sc_pgo      : $ENABLE_SC_PGO"
echo "    domain      : $DOMAIN_ID"
echo "  Stop          : Ctrl+C  or  ./onboard_slam.sh stop"
echo "################################################"
echo ""

# ── Trap ─────────────────────────────────────────────────────────────
cleanup_on_signal() {
  trap - INT TERM EXIT
  echo ""
  echo "Caught interrupt — tearing down onboard stack..."
  _kill_onboard_stack
  echo "Done."
  exit 0
}
trap cleanup_on_signal INT TERM

# Reset shm state from any previous run.
_kill_onboard_stack 2>/dev/null || true
sleep 1

# ── 1. livox_ros_driver2 ─────────────────────────────────────────────
# Same parameter set as the laptop's slam.launch.py:106-124.
LIVOX_CFG="$WS_ROOT/config/slam/MID360_config.json"
[[ -f "$LIVOX_CFG" ]] || { echo "ERROR: missing $LIVOX_CFG (run deploy_to_jetson.sh first)" >&2; exit 1; }

# setsid -f fully detaches each node into its own session — required on this
# Jetson because plain `&` children inherit the script's controlling terminal,
# and when the script's `wait` returns (which happens on Foxy because the
# `ros2 run` python wrapper apparently exits early after exec'ing), bash's
# huponexit kills the lot. Verified empirically: setsid'd nodes survive,
# `&`-only nodes die ~5 s after init.
setsid -f ros2 run livox_ros_driver2 livox_ros_driver2_node \
  --ros-args \
  -r __node:=livox_lidar_publisher \
  -p xfer_format:=1 \
  -p multi_topic:=0 \
  -p data_src:=0 \
  -p publish_freq:=10.0 \
  -p output_data_type:=0 \
  -p frame_id:=body \
  -p lvx_file_path:='""' \
  -p user_config_path:="$LIVOX_CFG" \
  -p cmdline_input_bd_code:=livox0000000001 \
  </dev/null >/tmp/livox.log 2>&1
LIVOX_PID="setsid"

# ── 2. fastlio_mapping ───────────────────────────────────────────────
# Remap /Odometry → /<ns>/odom/nav at the publisher to avoid needing a
# separate relay node. rclpy on Foxy 0.9.x segfaults on the Jetson under
# any non-trivial Python ROS 2 node (verified with two minimal scripts);
# C++ Fast-LIO doesn't go through rclpy, so the remap is reliable.
# Other consumers (laptop's octomap, future SC-PGO) subscribe to
# /cloud_registered and /cloud_registered_body, which stay un-namespaced.
FASTLIO_CFG="$WS_ROOT/config/slam/fastlio_mid360.yaml"
[[ -f "$FASTLIO_CFG" ]] || { echo "ERROR: missing $FASTLIO_CFG" >&2; exit 1; }

setsid -f ros2 run fast_lio fastlio_mapping --ros-args \
  -r __node:=fastlio_mapping \
  -r /Odometry:="/${NAMESPACE}/odom/nav" \
  --params-file "$FASTLIO_CFG" \
  -p use_sim_time:=false \
  </dev/null >/tmp/fastlio.log 2>&1
FASTLIO_PID="setsid"

# ── 3. Static TFs (3 of them — same as slam.launch.py:170-211 + plan §3-step5) ───
# Foxy's static_transform_publisher uses POSITIONAL args, not Humble-style
# --frame-id / --x / --roll flags. Two valid forms:
#   x y z qx qy qz qw parent child           (quaternion)
#   x y z yaw pitch roll parent child        (Euler — NOTE order: yaw pitch roll)
#
# map → camera_init: Mid-360 mount tilt compensation (measured 2026-04-17)
#   yaw=0, pitch=+0.263591, roll=-0.036809
setsid -f ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0.263591 -0.036809 map camera_init \
  --ros-args -r __node:=map_to_camera_init \
  </dev/null >/tmp/tf_map_camera_init.log 2>&1

setsid -f ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 -0.263591 0.036809 body base_link \
  --ros-args -r __node:=body_to_base_link_fastlio \
  </dev/null >/tmp/tf_body_base_link.log 2>&1

setsid -f ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 1 map odom \
  --ros-args -r __node:=map_to_odom_identity \
  </dev/null >/tmp/tf_map_odom.log 2>&1

# ── 4. /<NAMESPACE>/odom/nav ─────────────────────────────────────────
# Already handled by Fast-LIO's -r /Odometry:=/${NAMESPACE}/odom/nav remap
# above (step 2). No separate relay node needed — the laptop's nav stack
# (cfpa2_to_nav2_bridge, stuck_watchdog) sees the same Odometry messages
# under the namespaced topic name, just published directly by Fast-LIO.
# When migrating to Humble, switch back to the full fast_lio_tf_adapter
# so the SC-PGO bootstrap-from-GT logic is available.
true  # placeholder; ENABLE_FAST_LIO_TF flag retained for future Humble path

# ── 5. (optional) sc_pgo loop closure ────────────────────────────────
if [[ "$ENABLE_SC_PGO" == "true" ]]; then
  if ! ros2 pkg list 2>/dev/null | grep -q '^sc_pgo$'; then
    echo "ERROR: sc_pgo package not found. Phase 2 port may not be done yet." >&2
    echo "       Re-run with sc_pgo=false, or finish PORT_TO_ROS2.md first." >&2
    _kill_onboard_stack
    exit 1
  fi
  ros2 run sc_pgo sc_pgo_node --ros-args \
    -r __node:=sc_pgo \
    -r /aft_mapped_to_init:=/Odometry \
    -r /cloud_registered:=/cloud_registered_body \
    -r /corrected_odom:=/${NAMESPACE}/corrected_odom \
    -r /corrected_path:=/${NAMESPACE}/corrected_path \
    -r /corrected_cloud:=/${NAMESPACE}/corrected_cloud \
    -r /corrected_map:=/${NAMESPACE}/corrected_map \
    &
  SCPGO_PID=$!
fi

# ── Done ─────────────────────────────────────────────────────────────
# All children are detached via `setsid -f` and run independently. The
# script exits cleanly here — children keep running. To stop them, use:
#   $WS_ROOT/scripts/onboard_slam.sh stop
echo ""
echo "  All nodes detached. Logs at /tmp/{livox,fastlio,tf_*}.log"
echo "  Verify cross-host (from laptop):"
echo "    ros2 topic hz /livox/lidar       # 10 Hz from this Jetson"
echo "    ros2 topic hz /${NAMESPACE}/odom/nav  # /Odometry remapped"
echo "  Stop:    $WS_ROOT/scripts/onboard_slam.sh stop"
