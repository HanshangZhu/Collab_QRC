#!/usr/bin/env bash
# deploy_to_orin_nano.sh — rsync the HIL autonomy subset to the Orin Nano.
#
# Differs from deploy_to_jetson.sh (which targets the Go2's onboard Jetson):
#   - Target user/host : johnpork233 @ 192.168.55.49 (direct ethernet to enp10s0)
#   - ROS distro       : Humble (Ubuntu 22.04), not Foxy
#   - Package set      : autonomy stack only (SLAM + trav + Nav2 + CFPA2)
#                        — no Livox driver, no real-robot bringup, no MuJoCo
#   - Desktop role     : plays the "real world" — runs MuJoCo + sensor pubs
#   - Jetson role      : SLAM → trav → cmd_vel, sends back over DDS
#
# Usage:
#   JETSON_PASS=233 ./scripts/real/deploy_to_orin_nano.sh                # default
#   ./scripts/real/deploy_to_orin_nano.sh dry                            # dry run
#   ./scripts/real/deploy_to_orin_nano.sh host=192.168.55.49 pass=233    # explicit
#   ./scripts/real/deploy_to_orin_nano.sh build                          # rsync + colcon build remotely
#
# Idempotent. Uses rsync --update so unchanged files skip.

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO_ROOT="$( cd "$SCRIPT_DIR/../.." &> /dev/null && pwd )"

# ── Defaults ──────────────────────────────────────────────────────────
JETSON_USER="${ORIN_USER:-johnpork233}"
JETSON_PASS="${JETSON_PASS:-}"
JETSON_HOST="${ORIN_IP:-192.168.55.49}"
JETSON_WS="${ORIN_WS:-/home/${JETSON_USER}/jetson_ws}"
DRY_RUN="false"
DO_BUILD="false"

for arg in "$@"; do
  case "$arg" in
    user=*)    JETSON_USER="${arg#user=}" ;;
    pass=*)    JETSON_PASS="${arg#pass=}" ;;
    host=*)    JETSON_HOST="${arg#host=}" ;;
    ws=*)      JETSON_WS="${arg#ws=}" ;;
    dry|dry_run|dryrun) DRY_RUN="true" ;;
    build)     DO_BUILD="true" ;;
    *) echo "WARN: unknown arg '$arg'" >&2 ;;
  esac
done

command -v rsync   &>/dev/null || { echo "ERROR: rsync missing" >&2; exit 1; }
command -v sshpass &>/dev/null || { echo "ERROR: sshpass missing (apt install sshpass)" >&2; exit 1; }
[[ -n "$JETSON_PASS" ]] || { echo "ERROR: set JETSON_PASS env or pass=... arg" >&2; exit 1; }

SSH_OPTS=(-o StrictHostKeyChecking=accept-new -o ConnectTimeout=5 -o BatchMode=no)
SSH="sshpass -p $JETSON_PASS ssh ${SSH_OPTS[*]} ${JETSON_USER}@${JETSON_HOST}"
RSYNC_RSH="sshpass -p $JETSON_PASS ssh ${SSH_OPTS[*]}"

# ── Reachability ──────────────────────────────────────────────────────
echo ""
echo "################################################"
echo "  Deploying HIL autonomy stack to Orin Nano"
echo "    target  : ${JETSON_USER}@${JETSON_HOST}"
echo "    workspace: ${JETSON_WS}"
echo "    dry_run : $DRY_RUN"
echo "    build   : $DO_BUILD"
echo "################################################"
echo ""

if ! ping -c 2 -W 2 "$JETSON_HOST" &>/dev/null; then
  echo "ERROR: Cannot reach $JETSON_HOST. Bring up enp10s0 + DHCP first." >&2
  exit 1
fi
if ! sshpass -p "$JETSON_PASS" ssh "${SSH_OPTS[@]}" "${JETSON_USER}@${JETSON_HOST}" "echo OK" &>/dev/null; then
  echo "ERROR: SSH auth failed." >&2
  exit 1
fi
echo "  SSH + ping OK"

$SSH "mkdir -p ${JETSON_WS}/{src,install,build,log,config,scripts/runtime,scripts/launch}"

# ── rsync options ─────────────────────────────────────────────────────
RSYNC_OPTS=(
  -avz --update
  --exclude=build/ --exclude=install/ --exclude=log/
  --exclude=.git/ --exclude=__pycache__/ --exclude='*.pyc'
  --exclude='*.pcd' --exclude='*.bag' --exclude='*.npz'
  --exclude=node_modules/
)
[[ "$DRY_RUN" == "true" ]] && RSYNC_OPTS+=(--dry-run)

rsync_to_jetson() {
  local src="$REPO_ROOT/$1"
  local dst="${JETSON_USER}@${JETSON_HOST}:${JETSON_WS}/$2"
  if [[ ! -e "$src" ]]; then echo "  SKIP (missing): $1"; return 0; fi
  echo "  → $1  →  $2"
  rsync "${RSYNC_OPTS[@]}" -e "$RSYNC_RSH" "$src" "$dst"
}

# ── Vendor: SLAM + elevation_mapping_cupy ─────────────────────────────
echo ""
echo "── vendor/ ──"
rsync_to_jetson  src/vendor/fast_lio                                        src/vendor/
rsync_to_jetson  src/vendor/point_lio_ros2                                  src/vendor/
rsync_to_jetson  src/vendor/livox_ros_driver2_msgs                          src/vendor/
rsync_to_jetson  src/vendor/elevation_mapping_cupy/elevation_map_msgs       src/vendor/elevation_mapping_cupy/
rsync_to_jetson  src/vendor/elevation_mapping_cupy/elevation_mapping_cupy   src/vendor/elevation_mapping_cupy/
rsync_to_jetson  src/vendor/elevation_mapping_cupy/sensor_processing        src/vendor/elevation_mapping_cupy/

# Nav2 CUDA-MPPI vendored controller (Optimizer with cuda_backend_ hook) +
# nav_algo_mppi_cuda CUDA kernels (consumed by the plugin via relative path).
# COLCON_IGNORE the rest of nav_algo_ros1/ — those are ROS 1 catkin packages.
rsync_to_jetson  src/vendor/nav2_mppi_controller_cuda                       src/vendor/
$SSH "mkdir -p ${JETSON_WS}/src/vendor/nav_algo_ros1"
rsync_to_jetson  src/vendor/nav_algo_ros1/nav_algo_mppi_cuda                src/vendor/nav_algo_ros1/
$SSH "touch ${JETSON_WS}/src/vendor/nav_algo_ros1/COLCON_IGNORE"

# ── Collab autonomy ───────────────────────────────────────────────────
echo ""
echo "── collaborative_exploration/ ──"
rsync_to_jetson  src/collaborative_exploration/cfpa2_collaborative_autonomy     src/collaborative_exploration/
rsync_to_jetson  src/collaborative_exploration/cfpa2_peer_coordination           src/collaborative_exploration/
rsync_to_jetson  src/collaborative_exploration/cfpa2_peer_coordination_msgs      src/collaborative_exploration/
rsync_to_jetson  src/collaborative_exploration/trav_cost_filters                 src/collaborative_exploration/
rsync_to_jetson  src/collaborative_exploration/slam_backend_adapters             src/collaborative_exploration/

# ── Native ROS 2 CUDA-MPPI plugin (top-level src/, not under vendor/) ──
rsync_to_jetson  src/nav2_mppi_controller_cuda_plugin                       src/

# ── Configs needed onboard ────────────────────────────────────────────
echo ""
echo "── configs ──"
rsync_to_jetson  src/go2w/go2w_config/config/nav                              config/
# FastDDS no-shm profile — Jetson nodes MUST use the same UDP-only profile as
# the desktop or cross-host BEST_EFFORT subs (fast_lio lidar) won't match.
rsync_to_jetson  config/fastdds_no_shm.xml                                    config/
rsync_to_jetson  scripts/runtime/fast_lio_tf_adapter.py                       scripts/runtime/
# Explore-mode runtime nodes (started by orin_nano_hil_jetson.launch.py when
# explore:=true): CFPA2→Nav2 goal bridge + plan→planned_path relay for RViz.
rsync_to_jetson  scripts/runtime/cfpa2_to_nav2_bridge.py                      scripts/runtime/
rsync_to_jetson  scripts/runtime/path_relay.py                                scripts/runtime/

# ── HIL launchers (we'll author these in the runbook) ─────────────────
echo ""
echo "── HIL launchers ──"
for f in orin_nano_phase1_slam.sh orin_nano_phase2_trav.sh orin_nano_phase3_full.sh; do
  if [[ -f "$REPO_ROOT/scripts/real/$f" ]]; then
    rsync_to_jetson  "scripts/real/$f"  scripts/launch/
    $SSH "chmod +x ${JETSON_WS}/scripts/launch/$f" 2>/dev/null || true
  fi
done

# ── CycloneDDS XML for Jetson side ────────────────────────────────────
if [[ -f "$REPO_ROOT/scripts/real/cyclonedds_orin_nano.xml" ]]; then
  rsync_to_jetson  scripts/real/cyclonedds_orin_nano.xml  config/
fi

# ── Optional: build on Jetson ────────────────────────────────────────
if [[ "$DO_BUILD" == "true" && "$DRY_RUN" != "true" ]]; then
  echo ""
  echo "── colcon build on Jetson ──"
  $SSH "
    set -e
    source /opt/ros/humble/setup.bash
    export PATH=/usr/local/cuda/bin:\$PATH
    cd ${JETSON_WS}
    colcon build --symlink-install \
      --packages-up-to elevation_mapping_cupy cfpa2_collaborative_autonomy cfpa2_peer_coordination trav_cost_filters point_lio nav2_mppi_controller_cuda_plugin \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
      2>&1 | tail -50
  "
fi

echo ""
echo "################################################"
if [[ "$DRY_RUN" == "true" ]]; then
  echo "  DRY RUN — nothing transferred."
else
  echo "  Sync complete."
  echo ""
  echo "  Next:"
  echo "    1. First-time setup (once per Jetson):"
  echo "       ssh ${JETSON_USER}@${JETSON_HOST} 'bash -s' < scripts/real/orin_nano_setup.sh"
  echo ""
  echo "    2. Build (once after first sync):"
  echo "       $0 build"
  echo ""
  echo "    3. Phase 1 SLAM HIL test:"
  echo "       see docs/claude/orin_nano_hil_runbook.md"
fi
echo "################################################"
