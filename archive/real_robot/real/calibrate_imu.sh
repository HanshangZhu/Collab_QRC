#!/bin/bash
# calibrate_imu.sh — two-phase IMU calibration for real Go2W.
# Runs against the live robot over Ethernet/CycloneDDS. Output:
# imu_calib_data.yaml in CWD — move it to
#   src/go2w/go2w_real_bringup/config/imu_calib.yaml
# or set GO2W_IMU_CALIB env to the path you want transform_everything to load.
#
# Prerequisites:
#   - Ethernet link up (connect_ethernet.sh)
#   - Robot powered and on flat ground
#
# Usage:
#   ./calibrate_imu.sh                   # both phases, 30s each
#   ./calibrate_imu.sh static 30
#   ./calibrate_imu.sh spin 20
#   ./calibrate_imu.sh both 30

set -e
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO_ROOT="$( cd "$SCRIPT_DIR/../.." &> /dev/null && pwd )"

PHASE="${1:-both}"
DURATION="${2:-30}"

# Kill stale processes that would contend on the IMU topic.
pkill -9 -f cartographer 2>/dev/null || true
pkill -9 -f transform_everything 2>/dev/null || true
pkill -9 -f rviz2 2>/dev/null || true
pkill -9 -f "ros2 daemon" 2>/dev/null || true
sleep 2
ros2 daemon start 2>/dev/null || true
sleep 1

source /opt/ros/humble/setup.bash
[[ -f "$REPO_ROOT/install/setup.bash" ]] && source "$REPO_ROOT/install/setup.bash"

# Source ethernet DDS setup.
source "$SCRIPT_DIR/connect_ethernet.sh"
setup_cyclonedds_ethernet

CALIB_SCRIPT="$REPO_ROOT/src/go2w/go2w_real_bringup/tools/calibrate_imu.py"
if [[ ! -f "$CALIB_SCRIPT" ]]; then
  echo "ERROR: Calibration script missing: $CALIB_SCRIPT" >&2
  exit 1
fi

python3 "$CALIB_SCRIPT" --phase "$PHASE" --duration "$DURATION"

echo ""
echo "Done. Move imu_calib_data.yaml (CWD) into:"
echo "  $REPO_ROOT/src/go2w/go2w_real_bringup/config/imu_calib.yaml"
