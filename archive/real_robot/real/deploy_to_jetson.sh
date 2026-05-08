#!/usr/bin/env bash
# deploy_to_jetson.sh — rsync the onboard SLAM source tree to the Go2 Jetson.
#
# What goes onboard (per crispy-stargazing-kurzweil plan, Phase 1 step 1):
#   - src/vendor/Livox-SDK2/             (workspace-local CMake)
#   - src/vendor/livox_ros_driver2/      (ROS 2 driver, builds with ./build.sh ROS2)
#   - src/vendor/fast_lio/               (Fast-LIO 2)
#   - src/vendor/sc_pgo/                 (loop closure — only after Phase 2 port)
#   - src/go2w/go2w_real_bringup/config/slam/   (fastlio_mid360.yaml + MID360_config.json)
#   - scripts/runtime/fast_lio_tf_adapter.py    (publishes /<ns>/odom/nav onboard)
#   - scripts/real/onboard_slam.sh              (onboard launcher; deployed alongside)
#
# What stays on the laptop:
#   - Nav2, CFPA2, octomap, RViz, supervisor, cmd_vel mux + Sport bridge
#
# Usage:
#   ./deploy_to_jetson.sh                  # default Jetson 192.168.123.18
#   ./deploy_to_jetson.sh user=unitree pass=123 host=192.168.123.18
#   ./deploy_to_jetson.sh dry              # dry run — show what would transfer
#   ./deploy_to_jetson.sh sc_pgo=true      # also rsync sc_pgo/ (post-port)
#
# Idempotent: safe to re-run. Uses rsync --update so unchanged files skip.
# Excludes build/, install/, log/, .git/, __pycache__, *.pyc.

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO_ROOT="$( cd "$SCRIPT_DIR/../.." &> /dev/null && pwd )"

# ── Defaults ─────────────────────────────────────────────────────────
# Pass via env: JETSON_PASS=xyz ./deploy_to_jetson.sh   or   pass=xyz arg.
# Default is empty so we don't ship a credential in the script. The
# `unitree` user's Unitree-factory password is public ("123"), but if
# you've rotated it, set JETSON_PASS in your shell profile.
JETSON_USER="unitree"
JETSON_PASS="${JETSON_PASS:-}"
JETSON_HOST="${GO2W_JETSON_IP:-192.168.123.18}"
JETSON_WS="/home/unitree/onboard_ws"
INCLUDE_SC_PGO="false"
DRY_RUN="false"

for arg in "$@"; do
  case "$arg" in
    user=*)    JETSON_USER="${arg#user=}" ;;
    pass=*)    JETSON_PASS="${arg#pass=}" ;;
    host=*)    JETSON_HOST="${arg#host=}" ;;
    ws=*)      JETSON_WS="${arg#ws=}" ;;
    sc_pgo=*)  INCLUDE_SC_PGO="${arg#sc_pgo=}" ;;
    dry|dry_run|dryrun) DRY_RUN="true" ;;
    *) echo "WARN: unknown arg '$arg'" >&2 ;;
  esac
done

case "$INCLUDE_SC_PGO" in true|false) ;; *) echo "ERROR: sc_pgo must be true|false" >&2; exit 1 ;; esac
case "$DRY_RUN"        in true|false) ;; *) echo "ERROR: dry must be true|false" >&2; exit 1 ;; esac

# ── Tools ────────────────────────────────────────────────────────────
command -v rsync &>/dev/null || { echo "ERROR: rsync not installed" >&2; exit 1; }
command -v sshpass &>/dev/null || { echo "ERROR: sshpass not installed (apt install sshpass)" >&2; exit 1; }
if [[ -z "$JETSON_PASS" ]]; then
  echo "ERROR: JETSON_PASS not set." >&2
  echo "  Either: export JETSON_PASS=<robot-password>" >&2
  echo "  Or:     pass it on the cmdline: pass=<password> $0 ..." >&2
  echo "  (Unitree factory default for the 'unitree' user is widely documented.)" >&2
  exit 1
fi

SSH_OPTS=(-o StrictHostKeyChecking=accept-new -o ConnectTimeout=5 -o BatchMode=no)
SSH="sshpass -p $JETSON_PASS ssh ${SSH_OPTS[*]} ${JETSON_USER}@${JETSON_HOST}"
RSYNC_RSH="sshpass -p $JETSON_PASS ssh ${SSH_OPTS[*]}"

# ── Reachability ─────────────────────────────────────────────────────
echo ""
echo "################################################"
echo "  Deploying onboard SLAM tree to Jetson"
echo "    target  : ${JETSON_USER}@${JETSON_HOST}"
echo "    workspace: ${JETSON_WS}"
echo "    sc_pgo  : $INCLUDE_SC_PGO"
echo "    dry_run : $DRY_RUN"
echo "################################################"
echo ""

if ! ping -c 2 -W 2 "$JETSON_HOST" &>/dev/null; then
  echo "ERROR: Cannot reach $JETSON_HOST. Check Ethernet (./connect_ethernet.sh first)." >&2
  exit 1
fi

if ! sshpass -p "$JETSON_PASS" ssh "${SSH_OPTS[@]}" "${JETSON_USER}@${JETSON_HOST}" "echo OK" &>/dev/null; then
  echo "ERROR: SSH auth to ${JETSON_USER}@${JETSON_HOST} failed (wrong password?)." >&2
  exit 1
fi
echo "  SSH + ping OK"

# ── Ensure remote workspace exists ───────────────────────────────────
echo "  ensuring ${JETSON_WS}/{src,install,build,log} exist..."
$SSH "mkdir -p ${JETSON_WS}/{src,install,build,log,launch,config,scripts}"

# ── Common rsync options ─────────────────────────────────────────────
RSYNC_OPTS=(
  -avz --update
  --exclude=build/ --exclude=install/ --exclude=log/
  --exclude=.git/ --exclude=__pycache__/ --exclude='*.pyc'
  --exclude=.colcon/
  # NOTE: do NOT exclude COLCON_IGNORE — Livox-SDK2 needs its COLCON_IGNORE
  # marker preserved on the Jetson so colcon doesn't try to build it as a
  # ROS package (it's raw CMake, no package.xml). Without it, the second
  # colcon invocation on the Jetson auto-discovers Livox-SDK2/, builds an
  # incomplete livox_sdk2 colcon project, and clobbers the manual install
  # at ~/onboard_ws/install/Livox-SDK2/.
)
[[ "$DRY_RUN" == "true" ]] && RSYNC_OPTS+=(--dry-run)

# Helper to rsync a tree from repo to the Jetson workspace.
#   $1 = source path under REPO_ROOT
#   $2 = destination path under JETSON_WS  (trailing slash semantics same as rsync)
rsync_to_jetson() {
  local src="$REPO_ROOT/$1"
  local dst="${JETSON_USER}@${JETSON_HOST}:${JETSON_WS}/$2"
  if [[ ! -e "$src" ]]; then
    echo "  SKIP (missing locally): $src"
    return 0
  fi
  echo "  → $1  →  $2"
  rsync "${RSYNC_OPTS[@]}" -e "$RSYNC_RSH" "$src" "$dst"
}

# ── Vendored sources ─────────────────────────────────────────────────
echo ""
echo "── rsyncing vendor/ packages ──"
rsync_to_jetson  src/vendor/Livox-SDK2          src/
rsync_to_jetson  src/vendor/livox_ros_driver2   src/
rsync_to_jetson  src/vendor/fast_lio            src/

if [[ "$INCLUDE_SC_PGO" == "true" ]]; then
  rsync_to_jetson src/vendor/sc_pgo              src/
else
  echo "  (skipping sc_pgo — pass sc_pgo=true once Phase 2 port is green)"
fi

# ── Configs ──────────────────────────────────────────────────────────
echo ""
echo "── rsyncing SLAM configs ──"
rsync_to_jetson  src/go2w/go2w_real_bringup/config/slam   config/

# ── Helper scripts ───────────────────────────────────────────────────
echo ""
echo "── rsyncing helper scripts ──"
rsync_to_jetson  scripts/runtime/fast_lio_tf_adapter.py   scripts/runtime/

# Onboard launcher (sibling to this script in repo, deployed to Jetson).
if [[ -f "$REPO_ROOT/scripts/real/onboard_slam.sh" ]]; then
  rsync_to_jetson  scripts/real/onboard_slam.sh           scripts/
  $SSH "chmod +x ${JETSON_WS}/scripts/onboard_slam.sh" 2>/dev/null || true
fi

# CycloneDDS XML for the Jetson (deployed if present).
if [[ -f "$REPO_ROOT/scripts/real/cyclonedds_jetson.xml" ]]; then
  rsync_to_jetson  scripts/real/cyclonedds_jetson.xml     config/
fi

# ── Done ─────────────────────────────────────────────────────────────
echo ""
echo "################################################"
if [[ "$DRY_RUN" == "true" ]]; then
  echo "  DRY RUN complete — no files were transferred."
else
  echo "  Sync complete."
  echo ""
  echo "  Next steps (on the Jetson):"
  echo "    ssh ${JETSON_USER}@${JETSON_HOST}"
  echo "    cd ${JETSON_WS}"
  echo "    # 1. Build Livox-SDK2 (workspace-local)"
  echo "    cd src/Livox-SDK2 && mkdir -p build && cd build"
  echo "    cmake -DCMAKE_INSTALL_PREFIX=${JETSON_WS}/install/Livox-SDK2 \\"
  echo "          -DCMAKE_POSITION_INDEPENDENT_CODE=ON .."
  echo "    make -j\$(nproc) && make install"
  echo "    # 2. Build livox_ros_driver2"
  echo "    cd ${JETSON_WS}/src/livox_ros_driver2 && ./build.sh ROS2"
  echo "    # 3. Build fast_lio"
  echo "    cd ${JETSON_WS}"
  echo "    source /opt/ros/foxy/setup.bash"
  echo "    colcon build --symlink-install --packages-select fast_lio \\"
  echo "      --cmake-args -DLivox-SDK2_DIR=${JETSON_WS}/install/Livox-SDK2/lib/cmake/Livox-SDK2"
  echo "    # 4. (after Phase 2) sc_pgo build"
  echo "    # 5. Run:  ${JETSON_WS}/scripts/onboard_slam.sh"
fi
echo "################################################"
