#!/usr/bin/env bash
# onboard_autonomy_noetic.sh — full autonomy stack on the Go2 Orin NX (ROS 1
# Noetic, native, no ros1_bridge). This is the ROS 1 equivalent of the Orin
# Nano HIL launcher (scripts/real/orin_nano_hil_jetson.launch.py), but here
# EVERYTHING runs onboard the real robot in one ROS 1 master.
#
# Workspace: /home/unitree/autonomous_exploration_zhu  (consolidated catkin ws)
#
# Pipeline (brought up in dependency order):
#   1. roscore
#   2. livox_ros_driver2 (Mid-360, CustomMsg)          → /livox/{lidar,imu}
#   3. Point-LIO (SLAM)                                 → /<ns>/Odometry,
#                                                          /<ns>/cloud_registered_body
#   4. static TFs   map→camera_init, body→base_link, map→odom
#   5. trav pipeline (elevation_mapping_cupy + filter)  → /<ns>/traversability_grid
#   6. move_base (nav_algo SmacLattice + CUDA-MPPI)     → /<ns>/cmd_vel
#   7. CFPA2 single-robot (C++)                         → /<ns>/way_point_coord
#   8. cfpa2_to_movebase_bridge                         way_point_coord → goal
#
# Frame chain (provided by Point-LIO + the static TFs in step 4):
#   map → camera_init → body → base_link    (move_base needs map→base_link)
#
# Usage (run ON the NX):
#   ./onboard_autonomy_noetic.sh                      # full stack, ns=robot
#   ./onboard_autonomy_noetic.sh explore=false        # nav only, no CFPA2
#   ./onboard_autonomy_noetic.sh slam=fastlio          # FAST-LIO instead of Point-LIO
#   ./onboard_autonomy_noetic.sh rviz=true
#   ./onboard_autonomy_noetic.sh stop                  # tear everything down
#
# Ctrl+C exits cleanly via trap. Component logs in /tmp/onboard_*.log.

set -e
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
# When deployed, this script lives at $WS_ROOT/scripts/; default WS_ROOT to the
# consolidated workspace but allow override.
WS_ROOT="${ONBOARD_WS_ROOT:-/home/unitree/autonomous_exploration_zhu}"

# ── Strip miniconda BEFORE sourcing ROS (same guard as onboard_pointlio) ──
# The NX .bashrc auto-activates conda base → contaminates the C++ node link
# path (libstdc++/libpython mismatch) → segfault before main(). Scrub it.
if [[ -n "${CONDA_PREFIX:-}" ]] || echo "$PATH" | grep -q miniconda; then
  echo "  Stripping miniconda from env..."
  type conda 2>/dev/null | head -1 | grep -q function && conda deactivate 2>/dev/null || true
  export PATH="$(echo "$PATH" | tr ':' '\n' | grep -vE "(miniconda|conda)" | tr '\n' ':' | sed 's/:$//')"
  unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL CONDA_PYTHON_EXE
fi
unset PYTHONPATH PYTHONHOME

# ── Defaults ─────────────────────────────────────────────────────────
NAMESPACE="robot"
SLAM="pointlio"                 # pointlio | fastlio
EXPLORE="true"                  # run CFPA2 + bridge
ENABLE_RVIZ="false"
LIVOX_HOST_IP="192.168.123.100"
LIVOX_NIC=""
ROS_MASTER_PORT="11311"
TRAV_WEIGHTS=""                 # empty = pkg default weights
HIL="false"                     # HIL bench mode: sensors come from ros1_bridge
                                # (laptop MuJoCo), NOT a real Mid-360. Skips the
                                # NIC bind + the livox driver; expects the bridge
                                # (run_nx_hil_bridge.sh) to publish /livox/{lidar,imu}.
VIZ_RELAY="false"               # REAL-robot + viz: real Mid-360 sensors, but ALSO
                                # start the UDP relay TX so the NX's viz topics
                                # (Odometry/trav_grid/cloud/plan/way_point/cmd_vel)
                                # stream back to the laptop's RViz2. Reuses the HIL
                                # viz uplink (hil_relay_tx_node). Ignored if HIL=true
                                # (HIL already runs the tx). Needs viz_laptop_ip.
VIZ_LAPTOP_IP=""                # laptop IP for viz_relay (default 192.168.123.222)

# ── Cleanup ──────────────────────────────────────────────────────────
_kill_stack() {
  for p in cfpa2_single_robot_node_cpp cfpa2_coordinator_node_cpp \
           cfpa2_to_movebase_bridge move_base \
           elevation_mapping_node trav_filter_occ_grid \
           pointlio_mapping laserMapping fastlio_mapping \
           livox_ros_driver2_node static_transform_publisher \
           hil_relay_rx_node hil_relay_tx_node \
           "topic_tools relay" rosout rosmaster; do
    pkill -9 -f "$p" 2>/dev/null || true
  done
  pkill -9 -f "roslaunch" 2>/dev/null || true
  pkill -9 -f "roscore" 2>/dev/null || true
  sleep 1
}

# ── Parse args ───────────────────────────────────────────────────────
for arg in "$@"; do
  case "$arg" in
    stop)
      echo "Stopping onboard autonomy stack..."
      _kill_stack
      echo "Done."
      exit 0
      ;;
    namespace=*|ns=*) NAMESPACE="${arg#*=}" ;;
    slam=*)           SLAM="${arg#slam=}" ;;
    explore=*)        EXPLORE="${arg#explore=}" ;;
    rviz=*)           ENABLE_RVIZ="${arg#rviz=}" ;;
    livox_host=*)     LIVOX_HOST_IP="${arg#livox_host=}" ;;
    nic=*)            LIVOX_NIC="${arg#nic=}" ;;
    port=*)           ROS_MASTER_PORT="${arg#port=}" ;;
    weights=*)        TRAV_WEIGHTS="${arg#weights=}" ;;
    ros_ip=*)         OVERRIDE_ROS_IP="${arg#ros_ip=}" ;;
    hil=*)            HIL="${arg#hil=}" ;;
    viz_relay=*)      VIZ_RELAY="${arg#viz_relay=}" ;;
    viz_laptop_ip=*)  VIZ_LAPTOP_IP="${arg#viz_laptop_ip=}" ;;
    *) echo "WARN: unknown arg '$arg'" >&2 ;;
  esac
done

case "$SLAM" in pointlio|fastlio) ;; *) echo "ERROR: slam must be pointlio|fastlio" >&2; exit 1 ;; esac
case "$EXPLORE" in true|false) ;; *) echo "ERROR: explore must be true|false" >&2; exit 1 ;; esac
case "$ENABLE_RVIZ" in true|false) ;; *) echo "ERROR: rviz must be true|false" >&2; exit 1 ;; esac
case "$HIL" in true|false) ;; *) echo "ERROR: hil must be true|false" >&2; exit 1 ;; esac
case "$VIZ_RELAY" in true|false) ;; *) echo "ERROR: viz_relay must be true|false" >&2; exit 1 ;; esac

# ── Mid-360 NIC bind (real-LiDAR only; skipped in HIL — sensors come from the
#    ros1_bridge publishing /livox/{lidar,imu} from the laptop MuJoCo) ──────
if [[ "$HIL" == "true" ]]; then
  echo "  HIL mode: skipping Mid-360 NIC bind + livox driver (sensors from bridge)."
else
  if [[ -z "$LIVOX_NIC" ]]; then
    LIVOX_NIC="$(ip -o link show | awk -F': ' '$2 ~ /^en|^eth/ {print $2; exit}')"
  fi
  if [[ -z "$LIVOX_NIC" ]]; then
    echo "ERROR: could not auto-detect Ethernet NIC. Pass nic=eth0." >&2
    exit 1
  fi
  echo "  Ethernet NIC: $LIVOX_NIC"
  if ! ip -4 addr show "$LIVOX_NIC" | grep -q " ${LIVOX_HOST_IP}/"; then
    echo "  Adding ${LIVOX_HOST_IP}/24 as secondary on $LIVOX_NIC..."
    sudo -n ip addr add "${LIVOX_HOST_IP}/24" dev "$LIVOX_NIC" 2>/dev/null || \
      sudo ip addr add "${LIVOX_HOST_IP}/24" dev "$LIVOX_NIC" 2>/dev/null || {
      echo "  WARN: could not add ${LIVOX_HOST_IP}. Mid-360 bind() may fail." >&2
    }
  fi
fi

# ── ROS 1 env ────────────────────────────────────────────────────────
source /opt/ros/noetic/setup.bash
if [[ -f "$WS_ROOT/devel/setup.bash" ]]; then
  source "$WS_ROOT/devel/setup.bash"
else
  echo "ERROR: ${WS_ROOT}/devel/setup.bash not found — build the workspace first:" >&2
  echo "  cd ${WS_ROOT}/src/livox_ros_driver2 && ./build.sh ROS1" >&2
  echo "  cd ${WS_ROOT} && catkin_make -DCMAKE_BUILD_TYPE=Release -j4" >&2
  exit 1
fi

# rospack sanity — fail fast if a package didn't build/install.
for pkg in livox_ros_driver2 trav_pipeline_ros1 nav_algo_bringup \
           cfpa2_collaborative_autonomy; do
  if ! rospack find "$pkg" &>/dev/null; then
    echo "ERROR: rospack can't find '$pkg'. Build incomplete?" >&2
    exit 1
  fi
done
SLAM_PKG="point_lio"; SLAM_LAUNCH="mapping_mid360.launch"
if [[ "$SLAM" == "fastlio" ]]; then SLAM_PKG="fast_lio"; SLAM_LAUNCH="mapping_mid360.launch"; fi
rospack find "$SLAM_PKG" &>/dev/null || { echo "ERROR: SLAM pkg '$SLAM_PKG' not found." >&2; exit 1; }

if [[ -n "${OVERRIDE_ROS_IP:-}" ]]; then
  export ROS_IP="$OVERRIDE_ROS_IP"
else
  export ROS_IP="$(hostname -I | awk '{print $1}')"
fi
export ROS_MASTER_URI="http://${ROS_IP}:${ROS_MASTER_PORT}"

# ── Banner ───────────────────────────────────────────────────────────
echo ""
echo "################################################"
echo "  Onboard autonomy (Go2 Orin NX, ROS 1 Noetic)"
echo "    workspace : $WS_ROOT"
echo "    namespace : $NAMESPACE"
echo "    SLAM      : $SLAM ($SLAM_PKG/$SLAM_LAUNCH)"
echo "    explore   : $EXPLORE (CFPA2 + goal bridge)"
echo "    HIL       : $HIL $([ "$HIL" = true ] && echo '(sensors from UDP relay / laptop MuJoCo)')"
echo "    viz_relay : $VIZ_RELAY $([ "$VIZ_RELAY" = true ] && echo "(NX viz → laptop RViz2 @ ${VIZ_LAPTOP_IP:-192.168.123.222})")"
echo "    ROS_MASTER: $ROS_MASTER_URI"
echo "    rviz      : $ENABLE_RVIZ"
echo "  Stop        : Ctrl+C  or  $0 stop"
echo "################################################"
echo ""

cleanup_on_signal() {
  trap - INT TERM EXIT
  echo ""; echo "Caught interrupt — tearing down..."
  _kill_stack
  echo "Done."; exit 0
}
trap cleanup_on_signal INT TERM

_kill_stack 2>/dev/null || true
sleep 1

# ── 1. roscore ───────────────────────────────────────────────────────
echo "[1/8] roscore on :${ROS_MASTER_PORT}..."
setsid -f roscore -p "$ROS_MASTER_PORT" </dev/null >/tmp/onboard_roscore.log 2>&1
for i in $(seq 1 10); do rostopic list &>/dev/null && break; sleep 1; done
rostopic list &>/dev/null || { echo "ERROR: roscore failed (see /tmp/onboard_roscore.log)" >&2; cleanup_on_signal; }
echo "      roscore up."

# ── 2. livox_ros_driver2 (Mid-360, CustomMsg) ────────────────────────
# HIL: the real driver is NOT started — the ros1_bridge publishes /livox/lidar
# + /livox/imu from the laptop MuJoCo sim. We just wait for them to appear.
if [[ "$HIL" == "true" ]]; then
  echo "[2/8] HIL: waiting for bridged /livox/lidar (from laptop MuJoCo via ros1_bridge)..."
  # Start the UDP relay (replaces the broken Foxy ros1_bridge):
  #   rx: receives /livox/lidar(CustomMsg)+/livox/imu from the laptop (ports 9001/9002)
  #   tx: sends /<ns>/cmd_vel (+ viz) back to the laptop
  echo "      starting hil_udp_relay rx (sensors in) + tx (cmd_vel/viz out)..."
  nohup rosrun hil_udp_relay hil_relay_rx_node \
    _lidar_port:=9001 _imu_port:=9002 \
    </dev/null >/tmp/onboard_relay_rx.log 2>&1 &
  disown $! 2>/dev/null || true
  HIL_LAPTOP_IP="${HIL_LAPTOP_IP:-192.168.123.222}"
  nohup rosrun hil_udp_relay hil_relay_tx_node \
    _laptop_ip:="$HIL_LAPTOP_IP" _cmd_vel_topic:=/${NAMESPACE}/cmd_vel \
    _cmd_vel_port:=9003 _odom_port:=9004 _trav_port:=9005 _enable_viz:=true \
    </dev/null >/tmp/onboard_relay_tx.log 2>&1 &
  disown $! 2>/dev/null || true
  for i in $(seq 1 60); do
    rostopic info /livox/lidar 2>/dev/null | grep -q "Publishers:" && break; sleep 1
  done
  if rostopic info /livox/lidar 2>/dev/null | grep -q "Publishers:"; then
    echo "      /livox/lidar present (UDP relay from laptop)."
  else
    echo "      WARN: /livox/lidar not seen after 60s — is the laptop sim + relay up?" >&2
  fi
else
  echo "[2/8] livox_ros_driver2 (Mid-360)..."
  nohup roslaunch livox_ros_driver2 msg_MID360.launch \
    xfer_format:=1 multi_topic:=0 publish_freq:=10.0 output_type:=0 \
    msg_frame_id:=body rviz_enable:=false rosbag_enable:=false \
    </dev/null >/tmp/onboard_livox.log 2>&1 &
  disown $! 2>/dev/null || true
  for i in $(seq 1 10); do
    rostopic info /livox/lidar 2>/dev/null | grep -q "Publishers:" && break; sleep 1
  done
  echo "      /livox/lidar advertised."

  # REAL-robot + viz uplink: real Mid-360 above provides the sensors; here we
  # ALSO start the UDP relay TX so the NX's viz topics (Odometry, trav_grid,
  # cloud_registered_body, plan, way_point_coord, cmd_vel, tf) stream back to
  # the laptop's RViz2 — the "complete validation" observation path. Reuses the
  # exact HIL viz uplink; only the sensor source differs (real vs simulated).
  if [[ "$VIZ_RELAY" == "true" ]]; then
    VIZ_LAPTOP_IP="${VIZ_LAPTOP_IP:-192.168.123.222}"
    echo "      viz_relay: streaming NX viz → laptop ${VIZ_LAPTOP_IP} (RViz2)"
    nohup rosrun hil_udp_relay hil_relay_tx_node \
      _laptop_ip:="$VIZ_LAPTOP_IP" _cmd_vel_topic:=/${NAMESPACE}/cmd_vel \
      _cmd_vel_port:=9003 _odom_port:=9004 _trav_port:=9005 _enable_viz:=true \
      </dev/null >/tmp/onboard_relay_tx.log 2>&1 &
    disown $! 2>/dev/null || true
  fi
fi

# ── 3. SLAM (Point-LIO default) ──────────────────────────────────────
echo "[3/8] $SLAM_PKG ($SLAM_LAUNCH, ns=${NAMESPACE})..."
ROS_NAMESPACE="$NAMESPACE" nohup \
  roslaunch "$SLAM_PKG" "$SLAM_LAUNCH" rviz:=false \
  </dev/null >/tmp/onboard_slam.log 2>&1 &
disown $! 2>/dev/null || true
for i in $(seq 1 15); do
  rostopic info "/${NAMESPACE}/Odometry" 2>/dev/null | grep -q "Publishers:" && break; sleep 1
done
echo "      /${NAMESPACE}/Odometry up."

# ── 4. static TFs (Mid-360 mount tilt; chain map→camera_init→body→base_link) ─
echo "[4/8] static TFs..."
# Noetic static_transform_publisher: x y z yaw pitch roll parent child
setsid -f rosrun tf2_ros static_transform_publisher \
  0 0 0 0 0.263591 -0.036809 map camera_init </dev/null >/tmp/onboard_tf1.log 2>&1
setsid -f rosrun tf2_ros static_transform_publisher \
  0 0 0 0 -0.263591 0.036809 body base_link </dev/null >/tmp/onboard_tf2.log 2>&1
setsid -f rosrun tf2_ros static_transform_publisher \
  0 0 0 0 0 0 map odom </dev/null >/tmp/onboard_tf3.log 2>&1
echo "      map→camera_init, body→base_link, map→odom published."

# ── 5. traversability pipeline ───────────────────────────────────────
echo "[5/8] trav pipeline (elevation_mapping_cupy + filter)..."
WEIGHT_ARG=""
[[ -n "$TRAV_WEIGHTS" ]] && WEIGHT_ARG="weight_file:=${TRAV_WEIGHTS}"
nohup roslaunch trav_pipeline_ros1 trav_pipeline.launch $WEIGHT_ARG \
  </dev/null >/tmp/onboard_trav.log 2>&1 &
disown $! 2>/dev/null || true
for i in $(seq 1 20); do
  rostopic info "/${NAMESPACE}/traversability_grid" 2>/dev/null | grep -q "Publishers:" && break; sleep 1
done
echo "      /${NAMESPACE}/traversability_grid up (or still warming — check /tmp/onboard_trav.log)."

# ── 6. move_base (nav_algo SmacLattice + CUDA-MPPI) ──────────────────
echo "[6/8] move_base (nav_algo)..."
nohup roslaunch nav_algo_bringup move_base.launch robot_ns:="$NAMESPACE" \
  </dev/null >/tmp/onboard_movebase.log 2>&1 &
disown $! 2>/dev/null || true
for i in $(seq 1 15); do
  rosnode list 2>/dev/null | grep -q "/${NAMESPACE}/move_base" && break; sleep 1
done
echo "      move_base up."

# ── 7 + 8. CFPA2 + goal bridge (explore mode) ────────────────────────
if [[ "$EXPLORE" == "true" ]]; then
  echo "[7/8] CFPA2 single-robot (C++)..."
  # The ROS 1 CFPA2 node reads every param individually from its PRIVATE
  # namespace (~) via the param_facade — there is NO config_file loader.
  # So we rosparam-load the base yaml + ops2 overlay into the node's private
  # namespace (/<ns>/cfpa2_single_robot/) BEFORE starting it. Loading the
  # overlay second lets it override base keys (rosparam merges by key).
  # ROS 1 flat yamls (no /**:ros__parameters: wrapper — that's ROS2-only and
  # rosparam can't load it). Generated by flatten step in deploy script /
  # generate_cfpa2_ros1_yaml.py. Fall back to a runtime flatten if missing.
  CFG_DIR="$(rospack find cfpa2_collaborative_autonomy)/config"
  CFPA2_YAML="${CFG_DIR}/cfpa2_single_robot_ros1.yaml"
  CFPA2_OPS2_OVERLAY="${CFG_DIR}/cfpa2_single_robot_ops2_ros1.yaml"
  [[ -f "$CFPA2_YAML" ]] || CFPA2_YAML="${CFG_DIR}/cfpa2_single_robot.yaml"
  [[ -f "$CFPA2_OPS2_OVERLAY" ]] || CFPA2_OPS2_OVERLAY="${CFG_DIR}/cfpa2_single_robot_ops2.yaml"
  CFPA2_PRIV="/${NAMESPACE}/cfpa2_single_robot"
  if [[ -f "$CFPA2_YAML" ]]; then
    rosparam load "$CFPA2_YAML" "$CFPA2_PRIV"
    echo "      loaded base yaml → ${CFPA2_PRIV}/"
  fi
  if [[ -f "$CFPA2_OPS2_OVERLAY" ]]; then
    rosparam load "$CFPA2_OPS2_OVERLAY" "$CFPA2_PRIV"
    echo "      loaded ops2 overlay → ${CFPA2_PRIV}/"
  fi
  # robot_namespace must match the launch namespace so topics resolve.
  rosparam set "${CFPA2_PRIV}/robot_namespace" "$NAMESPACE"
  # ROS1/move_base override: the ops2 overlay sets planning_map_topic_suffix to
  # /global_costmap/costmap (Nav2 naming, /<ns>/global_costmap/costmap). On ROS1
  # move_base nests costmaps under /<ns>/move_base/global_costmap/costmap, so
  # that topic never appears at the Nav2 path → CFPA2 hangs "Waiting for map".
  # Plan directly on the trav grid (publishes at ~5 Hz) instead — same 2D BFS.
  rosparam set "${CFPA2_PRIV}/planning_map_topic_suffix" "/traversability_grid"
  # CFPA2 reads robot pose from /<ns>/odom/nav (hardcoded). Point-LIO publishes
  # /<ns>/Odometry; relay it so CFPA2 (and the bridge) get pose. In the Nav2 sim
  # fast_lio_tf_adapter does this; onboard we use a topic_tools relay.
  echo "      odom relay /${NAMESPACE}/Odometry → /${NAMESPACE}/odom/nav"
  setsid -f rosrun topic_tools relay "/${NAMESPACE}/Odometry" "/${NAMESPACE}/odom/nav" \
    </dev/null >/tmp/onboard_odom_relay.log 2>&1
  ROS_NAMESPACE="$NAMESPACE" nohup \
    rosrun cfpa2_collaborative_autonomy cfpa2_single_robot_node_cpp \
      </dev/null >/tmp/onboard_cfpa2.log 2>&1 &
  disown $! 2>/dev/null || true
  echo "      CFPA2 started (log: /tmp/onboard_cfpa2.log)."

  echo "[8/8] cfpa2_to_movebase_bridge..."
  ROS_NAMESPACE="$NAMESPACE" nohup \
    rosrun trav_pipeline_ros1 cfpa2_to_movebase_bridge.py \
      _namespace:="$NAMESPACE" \
      </dev/null >/tmp/onboard_bridge.log 2>&1 &
  disown $! 2>/dev/null || true
  echo "      bridge: /${NAMESPACE}/way_point_coord → /${NAMESPACE}/move_base_simple/goal"
else
  echo "[7/8] explore=false — CFPA2 + bridge skipped (nav-only / manual-goal mode)."
fi

# ── Optional RViz ────────────────────────────────────────────────────
if [[ "$ENABLE_RVIZ" == "true" ]]; then
  RVIZ_CFG="$(rospack find "$SLAM_PKG")/rviz_cfg/loam_livox.rviz"
  [[ -f "$RVIZ_CFG" ]] && setsid -f rviz -d "$RVIZ_CFG" </dev/null >/tmp/onboard_rviz.log 2>&1
fi

# ── Done ─────────────────────────────────────────────────────────────
echo ""
echo "  Stack up. Component logs: /tmp/onboard_*.log"
echo ""
echo "  Verify (same ROS_MASTER_URI):"
echo "    rostopic hz /livox/lidar                     # ~10 Hz"
echo "    rostopic hz /${NAMESPACE}/Odometry           # ~10 Hz (SLAM)"
echo "    rostopic hz /${NAMESPACE}/traversability_grid"
echo "    rostopic echo -n1 /${NAMESPACE}/cmd_vel      # move_base output"
echo "    rostopic echo -n1 /${NAMESPACE}/way_point_coord  # CFPA2 frontier goal"
echo "    rosrun tf2_tools view_frames.py              # TF tree PDF"
echo ""
echo "  Stop: $0 stop"
echo ""

# Keep the script alive so Ctrl+C tears down the detached children.
echo "  (running — Ctrl+C to tear down)"
while true; do sleep 3600; done
